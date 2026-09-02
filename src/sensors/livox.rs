//! Livox Mid-360 backend.
//!
//! The Mid-360 pushes points and IMU to a multicast group over plain UDP once
//! it has been configured, so the receive path needs no vendor SDK: two sockets
//! and the decoder in `crate::livox` are the whole of it. That is deliberate,
//! because it means the lidar path can be exercised on any machine that can be
//! sent UDP, hardware present or not.
//!
//! The `livox` cargo feature adds the SDK's configuration handshake — the
//! command channel that points a factory-fresh unit at this host — which is the
//! only part that genuinely needs the vendor library.

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use anyhow::{Context, Result};
use socket2::{Domain, Protocol, Socket, Type};

use super::{Backend, BackendStatus, LivoxConfig, Produced, Sink, StreamId};
use crate::livox::{self, FrameAccumulator};
use crate::record::now_nanos;

/// Big enough for the largest packet the lidar emits, with room to spare so a
/// firmware change cannot silently truncate a frame.
const RECEIVE_BUFFER: usize = 2048;

/// A quiet socket must still let the thread notice a stop request.
const READ_TIMEOUT: Duration = Duration::from_millis(200);

/// The kernel's default receive buffer is far too small for 200k points a
/// second and shows up as sporadic missing points that look like lidar dropout.
const SOCKET_RECEIVE_BUFFER: usize = 4 << 20;

pub struct LivoxBackend {
    config: LivoxConfig,
    running: Arc<AtomicBool>,
    workers: Vec<JoinHandle<()>>,
    dropped: Arc<AtomicU64>,
    error: Option<String>,
}

impl LivoxBackend {
    pub fn new(config: LivoxConfig) -> Self {
        LivoxBackend {
            config,
            running: Arc::new(AtomicBool::new(false)),
            workers: Vec::new(),
            dropped: Arc::new(AtomicU64::new(0)),
            error: None,
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

fn parse_host_address(text: &str) -> Option<Ipv4Addr> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return None;
    }
    trimmed.parse().ok()
}

impl Backend for LivoxBackend {
    fn start(&mut self, sink: Sink) -> Result<()> {
        if self.running.load(Ordering::SeqCst) {
            return Ok(());
        }
        self.error = None;
        let host_address = parse_host_address(&self.config.host_address);
        let point_socket = open_data_socket(livox::HOST_POINT_PORT, host_address)?;
        let imu_socket = if self.config.imu {
            Some(open_data_socket(livox::HOST_IMU_PORT, host_address)?)
        } else {
            None
        };

        self.running.store(true, Ordering::SeqCst);
        let cloud_topic = self.config.naming.points_topic();
        let cloud_frame = self.config.naming.frame_id(StreamId::PointCloud);
        let imu_topic = self.config.naming.imu_topic();
        let imu_frame = self.config.naming.frame_id(StreamId::Imu);
        let leaf_size = self.config.voxel_leaf_size;
        let frame_hz = self.config.frame_hz;

        {
            let running = Arc::clone(&self.running);
            let dropped = Arc::clone(&self.dropped);
            let sink = Arc::clone(&sink);
            self.workers.push(
                std::thread::Builder::new()
                    .name("livox-points".into())
                    .spawn(move || {
                        let mut accumulator = FrameAccumulator::new(frame_hz, cloud_frame)
                            .with_voxel_leaf_size(leaf_size);
                        let mut buffer = [0u8; RECEIVE_BUFFER];
                        while running.load(Ordering::SeqCst) {
                            let Ok(count) = point_socket.recv(&mut buffer) else {
                                continue;
                            };
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
                            let Ok(count) = imu_socket.recv(&mut buffer) else {
                                continue;
                            };
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
            error: self.error.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::Naming;

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
