//! Replays real Mid-360 datagrams over real UDP sockets, through the real
//! backend, encoder and recorder, and reads the resulting mcap back.
//!
//! The point of doing it over the network rather than by calling the decoder is
//! that the socket setup is the part that has historically been wrong: a missing
//! multicast join or a too-small receive buffer looks exactly like a dead lidar.
//! Nothing here is mocked — the bytes are a capture from the lidar on the Alfred
//! Jetson, and they travel through a genuine `UdpSocket`.

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use lite_record::hub::{Hub, Settings};
use lite_record::livox::{self, parse_capture, CapturedPacket};
use lite_record::sensors::SensorKind;

/// The capture is only 99.7 ms long — just under one 10 Hz frame — so a single
/// pass never crosses a frame boundary. Looping it is what a longer recording
/// looks like, but the lidar's clock has to keep moving forward across loops:
/// frames are cut on the timestamp in the packet header, and replaying the same
/// bytes unchanged would send that clock backwards.
const CAPTURE_SPAN_NANOS: u64 = 100_000_000;

/// The lidar's ports are fixed numbers, and the sockets are opened shared so a
/// packet capture can run alongside the recorder. Two of these tests at once
/// would therefore each receive the other's replay, so they take turns.
static LIDAR_PORTS: Mutex<()> = Mutex::new(());

fn hold_the_lidar_ports() -> MutexGuard<'static, ()> {
    // A panic in one test must not turn every later one into a lock error.
    LIDAR_PORTS.lock().unwrap_or_else(|held| held.into_inner())
}

/// Sends every captured datagram to the loopback port its stream really uses,
/// `rounds` times, pacing each round by the capture's own inter-arrival gaps so
/// the accumulator sees the real 10 Hz cloud and 200 Hz IMU cadence.
fn replay(packets: &[CapturedPacket], rounds: u64) {
    let sender = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
    let started = Instant::now();
    let first = packets.first().map(|packet| packet.received_nanos).unwrap_or(0);
    for round in 0..rounds {
        let advance = round * CAPTURE_SPAN_NANOS;
        for packet in packets {
            let due = Duration::from_nanos(
                packet.received_nanos.saturating_sub(first) + advance,
            );
            let elapsed = started.elapsed();
            if due > elapsed {
                std::thread::sleep(due - elapsed);
            }
            let mut payload = packet.payload.clone();
            // Bytes 28..36 are the header's lidar timestamp, the field the frame
            // accumulator bins on. Everything else stays exactly as captured.
            let stamp = u64::from_le_bytes(payload[28..36].try_into().unwrap());
            payload[28..36].copy_from_slice(&(stamp + advance).to_le_bytes());
            let target = SocketAddrV4::new(Ipv4Addr::LOCALHOST, packet.port);
            sender.send_to(&payload, target).unwrap();
        }
    }
}

fn recorded_settings(directory: &std::path::Path) -> Settings {
    let mut settings = Settings {
        record_dir: directory.to_path_buf(),
        ..Settings::default()
    };
    settings.livox.enabled = true;
    settings.livox.imu = true;
    // The replay comes from loopback. Naming it as the lidar keeps a real
    // Mid-360 on the same network — which is unicasting to these very ports —
    // from adding its packets to the count this test asserts on.
    settings.livox.lidar_address = Some(Ipv4Addr::LOCALHOST.to_string());
    settings.realsense.enabled = false;
    settings.orbbec.enabled = false;
    settings.preview_enabled = false;
    settings
}

#[test]
fn captured_mid360_traffic_lands_in_an_mcap_that_reads_back() {
    let _ports = hold_the_lidar_ports();
    let directory = std::env::temp_dir().join(format!("lite_record_livox_{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();

    let hub = Hub::new(recorded_settings(&directory), directory.join("settings.json"));
    hub.engage(SensorKind::Livox).unwrap();
    let status = hub.start_recording(Some("replay")).unwrap();
    let path = status.path.clone().unwrap();

    let packets = parse_capture(livox::CAPTURE);
    assert!(!packets.is_empty(), "the capture fixture is empty");
    // Three passes, so more than one 100 ms cloud boundary is crossed and the
    // test proves frames are emitted repeatedly rather than once at shutdown.
    replay(&packets, 3);

    // The sensor thread hands off to the encode pool, which hands off to the
    // writer; draining all three is what makes the message count deterministic.
    std::thread::sleep(Duration::from_millis(600));
    hub.disengage(SensorKind::Livox);
    let finished = hub.stop_recording().unwrap();

    assert!(finished.messages > 0, "nothing was recorded");
    assert_eq!(finished.dropped, 0, "the pipeline dropped messages on a tiny replay");

    let bytes = std::fs::read(&path).unwrap();
    let mut channels: std::collections::BTreeMap<String, (String, u64)> = Default::default();
    for message in mcap::MessageStream::new(&bytes).unwrap() {
        let message = message.unwrap();
        let schema = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.clone())
            .unwrap_or_default();
        let entry = channels
            .entry(message.channel.topic.clone())
            .or_insert((schema, 0));
        entry.1 += 1;
    }

    let points = channels
        .get("/livox/lidar")
        .expect("no point cloud topic in the mcap");
    assert_eq!(points.0, "sensor_msgs/msg/PointCloud2");
    assert!(points.1 >= 2, "expected repeated clouds, got {}", points.1);

    let imu = channels.get("/livox/imu").expect("no imu topic in the mcap");
    assert_eq!(imu.0, "sensor_msgs/msg/Imu");
    // The capture holds 20 IMU datagrams and each becomes one message, replayed
    // three times.
    assert_eq!(imu.1, 60);

    assert!(
        channels.contains_key(lite_record::record::TF_TOPIC),
        "every recording must carry the static transforms on {}, got {:?}",
        lite_record::record::TF_TOPIC,
        channels.keys().collect::<Vec<_>>()
    );

    std::fs::remove_dir_all(&directory).ok();
}

/// A recording has to survive the sensor being taken away and given back, since
/// that is the whole point of the disengage button.
#[test]
fn the_lidar_can_be_disengaged_and_re_engaged_without_restarting() {
    let _ports = hold_the_lidar_ports();
    let directory =
        std::env::temp_dir().join(format!("lite_record_cycle_{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();

    let hub: Arc<Hub> = Hub::new(recorded_settings(&directory), directory.join("settings.json"));
    for _ in 0..3 {
        hub.engage(SensorKind::Livox).unwrap();
        assert!(hub.sensor_status()["livox"].running);
        hub.disengage(SensorKind::Livox);
        assert!(!hub.sensor_status()["livox"].running);
    }

    hub.engage(SensorKind::Livox).unwrap();
    hub.start_recording(Some("after_cycling")).unwrap();
    replay(&parse_capture(livox::CAPTURE), 2);
    std::thread::sleep(Duration::from_millis(400));
    let finished = hub.stop_recording().unwrap();
    hub.disengage(SensorKind::Livox);

    assert!(
        finished.messages > 0,
        "a re-engaged lidar recorded nothing, so the socket was not reopened"
    );

    std::fs::remove_dir_all(&directory).ok();
}
