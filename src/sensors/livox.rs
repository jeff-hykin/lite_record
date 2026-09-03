//! Livox Mid-360 backend.
//!
//! The Mid-360 pushes points and IMU over plain UDP once it has been told where
//! to send them, so neither half of this needs a vendor SDK: two receive sockets
//! feeding the decoder in `crate::livox`, and the command channel in
//! `crate::livox_command` on the way in. That is deliberate, because it means
//! the lidar path can be exercised on any machine that can be sent UDP, hardware
//! present or not.
//!
//! The configuration step is not optional. A lidar that has never been told an
//! address transmits nothing at all, so opening the data sockets alone leaves
//! the recorder waiting on a lidar that looks broken and is merely idle.

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use socket2::{Domain, Protocol, Socket, Type};

use super::{Backend, BackendStatus, LivoxConfig, Produced, Sink, StreamId};
use crate::livox::{self, FrameAccumulator};
use crate::livox_command;
use crate::record::now_nanos;

/// Big enough for the largest packet the lidar emits, with room to spare so a
/// firmware change cannot silently truncate a frame.
const RECEIVE_BUFFER: usize = 2048;

/// A quiet socket must still let the thread notice a stop request.
const READ_TIMEOUT: Duration = Duration::from_millis(200);

/// The kernel's default receive buffer is far too small for 200k points a
/// second and shows up as sporadic missing points that look like lidar dropout.
const SOCKET_RECEIVE_BUFFER: usize = 4 << 20;

/// The lidar answers in milliseconds when it answers at all, so a short wait
/// with retries diagnoses a wrong subnet far quicker than one long one while
/// still riding out a dropped datagram.
const COMMAND_TIMEOUT: Duration = Duration::from_millis(300);
const COMMAND_ATTEMPTS: u32 = 4;

/// How long the lidar may go quiet before it is assumed to have forgotten where
/// to send. Unplugging one cuts its power, and it comes back configured for
/// nothing at all, so silence is the only signal that it needs telling again.
/// Two seconds is twenty missed frames at 10 Hz: long enough that a busy host
/// does not trigger it, short enough to be imperceptible when replugging.
const SILENCE_BEFORE_RECONFIGURE: Duration = Duration::from_secs(2);

/// Also the longest `stop()` can be delayed, so it is kept well under a second.
const SUPERVISOR_INTERVAL: Duration = Duration::from_millis(250);

/// Used when no lidar address is configured. `privileged::mid360_network_plan`
/// installs the host route that makes this deliverable.
const BROADCAST: Ipv4Addr = Ipv4Addr::new(255, 255, 255, 255);

pub struct LivoxBackend {
    config: LivoxConfig,
    running: Arc<AtomicBool>,
    workers: Vec<JoinHandle<()>>,
    dropped: Arc<AtomicU64>,
    /// Written by the supervisor as well as by `start`, so a lidar that is
    /// unplugged mid-recording says so in the UI rather than looking idle.
    error: Arc<std::sync::Mutex<Option<String>>>,
    /// Host time the last packet arrived, or 0 before any has.
    last_packet_nanos: Arc<AtomicU64>,
}

impl LivoxBackend {
    pub fn new(config: LivoxConfig) -> Self {
        LivoxBackend {
            config,
            running: Arc::new(AtomicBool::new(false)),
            workers: Vec::new(),
            dropped: Arc::new(AtomicU64::new(0)),
            error: Arc::new(std::sync::Mutex::new(None)),
            last_packet_nanos: Arc::new(AtomicU64::new(0)),
        }
    }

    pub fn config(&self) -> &LivoxConfig {
        &self.config
    }

    pub fn set_config(&mut self, config: LivoxConfig) {
        self.config = config;
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// Binds a data port and joins the lidar's multicast group.
///
/// Binding without the group join is the mistake that costs an afternoon: the
/// socket opens, reads block forever, and everything looks like a dead lidar.
/// `SO_REUSEADDR` is set so a second reader (a packet capture, say) can share
/// the port rather than one of them failing to start.
pub fn open_data_socket(port: u16, host_address: Option<Ipv4Addr>) -> Result<UdpSocket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))
        .context("creating the lidar socket")?;
    socket.set_reuse_address(true)?;
    // On Linux SO_REUSEADDR already lets a second reader share the port. The BSDs
    // do not: there SO_REUSEADDR only relaxes the rule for a multicast bind
    // address, and this binds the wildcard, so sharing needs SO_REUSEPORT as well.
    // It is deliberately not set on Linux, where SO_REUSEPORT would distribute
    // unicast packets between the readers instead of giving each a full copy.
    #[cfg(all(unix, not(target_os = "linux")))]
    socket.set_reuse_port(true)?;
    let bind_address = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port);
    socket
        .bind(&bind_address.into())
        .with_context(|| format!("binding udp {port}; another livox reader may already hold it"))?;
    socket.set_recv_buffer_size(SOCKET_RECEIVE_BUFFER).ok();
    socket.set_read_timeout(Some(READ_TIMEOUT))?;

    let group = Ipv4Addr::from(livox::DEFAULT_MULTICAST_GROUP);
    let interface = host_address.unwrap_or(Ipv4Addr::UNSPECIFIED);
    // A unicast-configured lidar is not in the group, so a failed join is not
    // fatal; a bind that heard nothing because of a missing join is.
    if let Err(error) = socket.join_multicast_v4(&group, &interface) {
        eprintln!("livox: could not join {group} on {interface}: {error}");
    }
    Ok(socket.into())
}

/// With no lidar address configured the lidar is whichever one answered the
/// broadcast, so there is nothing to compare against and everything is accepted.
fn is_expected_lidar(expected: Option<Ipv4Addr>, from: std::net::SocketAddr) -> bool {
    match (expected, from.ip()) {
        (Some(expected), std::net::IpAddr::V4(actual)) => expected == actual,
        (Some(_), _) => false,
        (None, _) => true,
    }
}

fn parse_host_address(text: &str) -> Option<Ipv4Addr> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return None;
    }
    trimmed.parse().ok()
}

/// Sends one config command and waits for the lidar to accept it.
///
/// Deliberately `send_to`/`recv_from` rather than a connected socket: the reply
/// to a broadcast comes back from the lidar's own address, so a connected socket
/// would filter out the very ack it is waiting for.
fn send_command(
    socket: &UdpSocket,
    target: SocketAddrV4,
    sequence_number: u32,
    payload: &[u8],
    what: &str,
) -> Result<()> {
    let frame = livox_command::frame(livox_command::CMD_WORK_MODE_CONTROL, sequence_number, payload);
    let mut buffer = [0u8; RECEIVE_BUFFER];
    for _ in 0..COMMAND_ATTEMPTS {
        socket
            .send_to(&frame, target)
            .with_context(|| format!("sending {what}"))?;
        // Drains until the read times out, because a late ack for the previous
        // command or a stray packet on this port must not be read as the answer
        // to this one.
        while let Ok((count, _)) = socket.recv_from(&mut buffer) {
            let Ok(ack) = livox_command::parse_ack(&buffer[..count]) else {
                continue;
            };
            if ack.sequence_number != sequence_number & 0xffff {
                continue;
            }
            if ack.return_code != 0 {
                bail!(
                    "the lidar refused {what} (code {}, key 0x{:04x})",
                    ack.return_code,
                    ack.error_key
                );
            }
            return Ok(());
        }
    }
    bail!("no answer to {what} after {COMMAND_ATTEMPTS} tries")
}

/// Points the lidar at this host and starts it sampling, returning the address
/// it was told to send to.
///
/// With the host address left blank that is whatever source address the kernel
/// picks for the route to the lidar, which is the only address a reply could
/// reach anyway — guessing at it from the interface list gets this wrong on any
/// machine with more than one network.
fn configure_lidar(config: &LivoxConfig, host_address: Option<Ipv4Addr>) -> Result<Ipv4Addr> {
    let lidar = match config.lidar_address.as_deref().map(str::trim) {
        Some(text) if !text.is_empty() => text
            .parse()
            .with_context(|| format!("{text:?} is not a lidar address"))?,
        _ => BROADCAST,
    };
    let target = SocketAddrV4::new(lidar, livox_command::LIDAR_CMD_PORT);

    let host = match host_address {
        Some(address) => address,
        None => {
            // Connecting a UDP socket sends nothing; it just asks the routing
            // table which way the lidar is and what address that leaves from.
            let probe = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0))?;
            probe.set_broadcast(true)?;
            probe
                .connect(target)
                .with_context(|| format!("no route to the lidar at {lidar}"))?;
            match probe.local_addr()?.ip() {
                std::net::IpAddr::V4(address) if !address.is_unspecified() => address,
                _ => bail!("could not tell which address the lidar should send to; set the host address"),
            }
        }
    };

    let socket = UdpSocket::bind(SocketAddrV4::new(
        Ipv4Addr::UNSPECIFIED,
        livox_command::HOST_CMD_PORT,
    ))
    .with_context(|| {
        format!(
            "binding udp {}; another livox client may already hold it",
            livox_command::HOST_CMD_PORT
        )
    })?;
    socket.set_broadcast(true)?;
    socket.set_read_timeout(Some(COMMAND_TIMEOUT))?;

    // Order matters: the destinations have to be in place before sampling
    // starts, or the first points are produced with nowhere to go.
    for (sequence_number, (what, payload)) in [
        (
            "the host address",
            livox_command::host_destination_payload(host),
        ),
        ("the imu setting", livox_command::imu_data_payload(config.imu)),
        (
            "normal work mode",
            livox_command::work_mode_payload(livox_command::WORK_MODE_NORMAL),
        ),
    ]
    .into_iter()
    .enumerate()
    {
        send_command(&socket, target, sequence_number as u32 + 1, &payload, what)
            .with_context(|| format!("configuring the lidar at {lidar}"))?;
    }
    Ok(host)
}

impl Backend for LivoxBackend {
    fn start(&mut self, sink: Sink) -> Result<()> {
        if self.running.load(Ordering::SeqCst) {
            return Ok(());
        }
        *self.error.lock().unwrap() = None;
        let host_address = parse_host_address(&self.config.host_address);
        // The data ports are fixed by the protocol and the bind is the wildcard,
        // so a second Mid-360 on the same wire lands in this socket too. Its
        // packets are not this sensor's, and worse, they would keep a lidar that
        // is actually unplugged looking alive to the supervisor.
        let expected_lidar = self
            .config
            .lidar_address
            .as_deref()
            .and_then(parse_host_address);
        let point_socket = open_data_socket(livox::HOST_POINT_PORT, host_address)?;
        let imu_socket = if self.config.imu {
            Some(open_data_socket(livox::HOST_IMU_PORT, host_address)?)
        } else {
            None
        };

        self.running.store(true, Ordering::SeqCst);
        self.last_packet_nanos.store(0, Ordering::Relaxed);
        let cloud_topic = self.config.naming.points_topic();
        let cloud_frame = self.config.naming.frame_id(StreamId::PointCloud);
        let imu_topic = self.config.naming.imu_topic();
        let imu_frame = self.config.naming.frame_id(StreamId::Imu);
        let leaf_size = self.config.voxel_leaf_size;
        let frame_hz = self.config.frame_hz;

        // Configuring the lidar is the supervisor's first act rather than a
        // separate step here, because a lidar that is unplugged and replugged
        // needs exactly the same commands again and there is no reason to have
        // two copies of that.
        {
            let running = Arc::clone(&self.running);
            let last_packet_nanos = Arc::clone(&self.last_packet_nanos);
            let error = Arc::clone(&self.error);
            let config = self.config.clone();
            self.workers.push(
                std::thread::Builder::new()
                    .name("livox-supervisor".into())
                    .spawn(move || {
                        // A failure that repeats every quarter second would bury
                        // the log, so only a change of state is worth printing.
                        let mut last_reported: Option<String> = None;
                        while running.load(Ordering::SeqCst) {
                            let last = last_packet_nanos.load(Ordering::Relaxed);
                            let silent_for = now_nanos().saturating_sub(last);
                            if silent_for >= SILENCE_BEFORE_RECONFIGURE.as_nanos() as u64 {
                                let outcome = match configure_lidar(&config, host_address) {
                                    Ok(host) => {
                                        // Gives the lidar until the next check to
                                        // start, rather than reconfiguring it
                                        // again while it is still spinning up.
                                        last_packet_nanos.store(now_nanos(), Ordering::Relaxed);
                                        *error.lock().unwrap() = None;
                                        format!("lidar configured to send to {host}")
                                    }
                                    Err(failure) => {
                                        let reason = format!("{failure:#}");
                                        *error.lock().unwrap() = Some(reason.clone());
                                        reason
                                    }
                                };
                                if last_reported.as_deref() != Some(outcome.as_str()) {
                                    eprintln!("livox: {outcome}");
                                    last_reported = Some(outcome);
                                }
                            }
                            std::thread::sleep(SUPERVISOR_INTERVAL);
                        }
                    })?,
            );
        }

        {
            let running = Arc::clone(&self.running);
            let dropped = Arc::clone(&self.dropped);
            let last_packet_nanos = Arc::clone(&self.last_packet_nanos);
            let sink = Arc::clone(&sink);
            self.workers.push(
                std::thread::Builder::new()
                    .name("livox-points".into())
                    .spawn(move || {
                        let mut accumulator = FrameAccumulator::new(frame_hz, cloud_frame)
                            .with_voxel_leaf_size(leaf_size);
                        let mut buffer = [0u8; RECEIVE_BUFFER];
                        while running.load(Ordering::SeqCst) {
                            let Ok((count, from)) = point_socket.recv_from(&mut buffer) else {
                                continue;
                            };
                            if !is_expected_lidar(expected_lidar, from) {
                                continue;
                            }
                            last_packet_nanos.store(now_nanos(), Ordering::Relaxed);
                            match accumulator.push_packet(&buffer[..count], now_nanos()) {
                                Ok(Some(cloud)) => {
                                    if !sink(Produced::Cloud {
                                        topic: cloud_topic.clone(),
                                        cloud,
                                    }) {
                                        dropped.fetch_add(1, Ordering::Relaxed);
                                    }
                                }
                                Ok(None) => {}
                                Err(error) => eprintln!("livox point packet: {error}"),
                            }
                        }
                    })?,
            );
        }

        if let Some(imu_socket) = imu_socket {
            let running = Arc::clone(&self.running);
            let dropped = Arc::clone(&self.dropped);
            self.workers.push(
                std::thread::Builder::new()
                    .name("livox-imu".into())
                    .spawn(move || {
                        let mut accumulator = FrameAccumulator::new(frame_hz, imu_frame.clone());
                        let mut buffer = [0u8; RECEIVE_BUFFER];
                        while running.load(Ordering::SeqCst) {
                            let Ok((count, from)) = imu_socket.recv_from(&mut buffer) else {
                                continue;
                            };
                            if !is_expected_lidar(expected_lidar, from) {
                                continue;
                            }
                            match accumulator.push_imu(&buffer[..count], now_nanos(), &imu_frame) {
                                Ok(imu) => {
                                    if !sink(Produced::Imu {
                                        topic: imu_topic.clone(),
                                        imu: Box::new(imu),
                                    }) {
                                        dropped.fetch_add(1, Ordering::Relaxed);
                                    }
                                }
                                Err(error) => eprintln!("livox imu packet: {error}"),
                            }
                        }
                    })?,
            );
        }
        Ok(())
    }

    /// Closes both sockets, which is what actually releases the multicast group
    /// so another process can take over the lidar.
    fn stop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        for worker in self.workers.drain(..) {
            let _ = worker.join();
        }
    }

    fn status(&self) -> BackendStatus {
        BackendStatus {
            running: self.running.load(Ordering::SeqCst),
            detail: format!(
                "{} at {} Hz{}",
                self.config
                    .lidar_address
                    .clone()
                    .unwrap_or_else(|| "multicast 224.1.1.5".into()),
                self.config.frame_hz,
                if self.config.voxel_leaf_size > 0.0 {
                    format!(", voxel {} m", self.config.voxel_leaf_size)
                } else {
                    String::new()
                }
            ),
            error: self.error.lock().unwrap().clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::Naming;
    use std::net::SocketAddr;

    #[test]
    fn an_empty_host_address_means_let_the_os_choose() {
        assert_eq!(parse_host_address(""), None);
        assert_eq!(parse_host_address("   "), None);
        assert_eq!(parse_host_address("not an ip"), None);
        assert_eq!(
            parse_host_address(" 192.168.1.50 "),
            Some(Ipv4Addr::new(192, 168, 1, 50))
        );
    }

    #[test]
    fn both_data_ports_can_be_opened_and_reopened() {
        // Re-engaging after a disengage must not fail on a still-held port,
        // which is what SO_REUSEADDR plus dropping the socket buys.
        for _ in 0..2 {
            let points = open_data_socket(livox::HOST_POINT_PORT, None).unwrap();
            let imu = open_data_socket(livox::HOST_IMU_PORT, None).unwrap();
            assert_eq!(points.local_addr().unwrap().port(), livox::HOST_POINT_PORT);
            assert_eq!(imu.local_addr().unwrap().port(), livox::HOST_IMU_PORT);
        }
    }

    #[test]
    fn a_second_reader_can_share_the_data_port() {
        // A packet capture alongside the recorder, or a stale copy of this program
        // that has not exited yet, must not stop the port from opening.
        let first = open_data_socket(livox::HOST_POINT_PORT, None).unwrap();
        let second = open_data_socket(livox::HOST_POINT_PORT, None).unwrap();
        assert_eq!(first.local_addr().unwrap().port(), livox::HOST_POINT_PORT);
        assert_eq!(second.local_addr().unwrap().port(), livox::HOST_POINT_PORT);
    }

    /// A lidar stand-in on loopback. Answers `ignore_first` requests with
    /// silence, then acks every later one with `return_code`, and hands back
    /// what it was sent so the request itself can be inspected.
    /// Keeps the stand-in alive only as long as the test holds it. Without this
    /// a test that fails early leaves a thread squatting the fixed command port,
    /// and every later run dies with `AddrInUse` instead of its real result.
    struct FakeLidar {
        address: SocketAddrV4,
        requests: std::sync::mpsc::Receiver<Vec<u8>>,
        running: Arc<AtomicBool>,
        thread: Option<JoinHandle<()>>,
    }

    impl Drop for FakeLidar {
        fn drop(&mut self) {
            self.running.store(false, Ordering::SeqCst);
            if let Some(thread) = self.thread.take() {
                let _ = thread.join();
            }
        }
    }

    fn fake_lidar(port: u16, ignore_first: usize, return_code: u8) -> FakeLidar {
        let socket = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, port)).unwrap();
        socket.set_read_timeout(Some(READ_TIMEOUT)).unwrap();
        let SocketAddr::V4(address) = socket.local_addr().unwrap() else {
            unreachable!("bound to an ipv4 address")
        };
        let (sender, requests) = std::sync::mpsc::channel();
        let running = Arc::new(AtomicBool::new(true));
        let thread_running = running.clone();
        let thread = std::thread::spawn(move || {
            let mut buffer = [0u8; RECEIVE_BUFFER];
            let mut seen = 0;
            while thread_running.load(Ordering::SeqCst) {
                let Ok((count, from)) = socket.recv_from(&mut buffer) else {
                    continue;
                };
                let request = buffer[..count].to_vec();
                let sequence_number =
                    u32::from_le_bytes([request[4], request[5], request[6], request[7]]);
                if sender.send(request).is_err() {
                    return;
                }
                seen += 1;
                if seen > ignore_first {
                    let ack = livox_command::ack_frame(
                        livox_command::CMD_WORK_MODE_CONTROL,
                        sequence_number,
                        &[return_code, 0, 0],
                    );
                    let _ = socket.send_to(&ack, from);
                }
            }
        });
        FakeLidar {
            address,
            requests,
            running,
            thread: Some(thread),
        }
    }

    fn command_socket() -> UdpSocket {
        let socket = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
        socket.set_read_timeout(Some(COMMAND_TIMEOUT)).unwrap();
        socket
    }

    #[test]
    fn a_command_carries_the_payload_and_is_accepted() {
        let lidar = fake_lidar(0, 0, 0);
        let payload = livox_command::work_mode_payload(livox_command::WORK_MODE_NORMAL);
        send_command(
            &command_socket(),
            lidar.address,
            1,
            &payload,
            "normal work mode",
        )
        .unwrap();

        let request = lidar.requests.recv().unwrap();
        assert_eq!(&request[livox_command::HEADER_LEN..], &payload[..]);
        assert!(
            livox_command::parse_ack(&request).is_err(),
            "the host sends commands, not acks"
        );
    }

    #[test]
    fn a_dropped_datagram_is_retried_rather_than_failing_the_lidar() {
        // A single lost request on a busy link would otherwise look identical to
        // a lidar that is not there.
        let lidar = fake_lidar(0, 1, 0);
        let payload = livox_command::imu_data_payload(true);
        send_command(
            &command_socket(),
            lidar.address,
            9,
            &payload,
            "the imu setting",
        )
        .unwrap();
        assert_eq!(
            lidar.requests.iter().take(2).count(),
            2,
            "the request was resent"
        );
    }

    #[test]
    fn a_refusal_is_reported_instead_of_being_read_as_success() {
        let lidar = fake_lidar(0, 0, 1);
        let payload = livox_command::work_mode_payload(livox_command::WORK_MODE_NORMAL);
        let error = send_command(
            &command_socket(),
            lidar.address,
            1,
            &payload,
            "normal work mode",
        )
        .expect_err("a non-zero return code is a refusal");
        assert!(error.to_string().contains("refused"), "{error}");
    }

    #[test]
    fn a_silent_lidar_gives_up_rather_than_blocking_the_recorder() {
        // Nothing is bound to this port, so every attempt times out. Start must
        // not hang here, because the other sensors are already waiting on it.
        let target = SocketAddrV4::new(Ipv4Addr::LOCALHOST, 56199);
        let error = send_command(&command_socket(), target, 1, &[0, 0, 0, 0], "the host address")
            .expect_err("no lidar is listening");
        assert!(error.to_string().contains("no answer"), "{error}");
    }

    #[test]
    fn a_lidar_that_goes_quiet_is_configured_again_without_a_restart() {
        // Unplugging a Mid-360 cuts its power, and it comes back knowing nothing
        // about where to send. Configuring it once at start would make a replug a
        // permanent loss of the stream until the whole sensor was re-engaged by
        // hand, so silence has to re-arm it.
        let lidar = fake_lidar(livox_command::LIDAR_CMD_PORT, 0, 0);
        let config = LivoxConfig {
            lidar_address: Some(lidar.address.ip().to_string()),
            host_address: Ipv4Addr::LOCALHOST.to_string(),
            imu: false,
            ..LivoxConfig::default()
        };
        let mut backend = LivoxBackend::new(config);
        let sink: Sink = Arc::new(|_| true);
        backend.start(sink).unwrap();

        // Nothing is ever sent back on the data port, which is exactly what an
        // unplugged lidar looks like from here.
        // A fixed budget rather than a multiple of SILENCE_BEFORE_RECONFIGURE: if
        // that constant is ever raised the test must fail, not run for hours.
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        let mut sampling_starts = 0;
        while sampling_starts < 2 && std::time::Instant::now() < deadline {
            let Ok(request) = lidar.requests.recv_timeout(SUPERVISOR_INTERVAL) else {
                continue;
            };
            let key = request.get(livox_command::HEADER_LEN + 4..livox_command::HEADER_LEN + 6);
            if key == Some(&[0x1a, 0x00][..]) {
                sampling_starts += 1;
            }
        }
        backend.stop();
        assert!(
            sampling_starts >= 2,
            "the lidar was told to start sampling only {sampling_starts} time(s)"
        );
    }

    #[test]
    fn a_lidar_address_that_is_not_an_address_is_refused_before_any_traffic() {
        let config = LivoxConfig {
            lidar_address: Some("192.168.1".into()),
            ..LivoxConfig::default()
        };
        let error = configure_lidar(&config, None).expect_err("not an address");
        assert!(error.to_string().contains("not a lidar address"), "{error}");
    }

    #[test]
    fn the_topics_a_stopped_backend_would_publish_are_already_known() {
        let config = LivoxConfig {
            naming: Naming {
                topic_prefix: "/mid360".into(),
                frame_prefix: "mid360".into(),
            },
            ..LivoxConfig::default()
        };
        let backend = LivoxBackend::new(config);
        assert_eq!(backend.config().naming.points_topic(), "/mid360/points");
        assert_eq!(backend.config().naming.imu_topic(), "/mid360/imu");
        assert_eq!(
            backend.config().naming.frame_id(StreamId::PointCloud),
            "mid360_frame"
        );
        assert!(!backend.status().running);
    }
}
