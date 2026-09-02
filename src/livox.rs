//! Livox Mid-360 wire format: packet decoding, 10 Hz frame accumulation and
//! voxel downsampling. Deliberately free of any SDK dependency so it compiles
//! and is tested on a laptop; `sensors::livox` drives it from the real SDK.

use crate::msgs::{
    Header, Imu, PointCloud2, PointField, NANOS_PER_SEC, POINT_FIELD_FLOAT32, POINT_FIELD_FLOAT64,
    POINT_FIELD_UINT32, POINT_FIELD_UINT8,
};
use anyhow::{bail, Result};

/// `LivoxLidarEthernetPacket` up to but not including `data`.
pub const HEADER_LEN: usize = 36;

pub const DATA_TYPE_IMU: u8 = 0x00;
pub const DATA_TYPE_CARTESIAN_HIGH: u8 = 0x01;
pub const DATA_TYPE_CARTESIAN_LOW: u8 = 0x02;
pub const DATA_TYPE_SPHERICAL: u8 = 0x03;

/// Default UDP ports the lidar sends to. The SDK hardcodes these too.
pub const HOST_POINT_PORT: u16 = 56301;
pub const HOST_IMU_PORT: u16 = 56401;
pub const DEFAULT_MULTICAST_GROUP: [u8; 4] = [224, 1, 1, 5];

/// The Mid-360 interleaves four scan lines. `livox_ros_driver2` tags each point
/// with `index % LINE_COUNT`, and downstream de-skewing relies on it.
pub const LINE_COUNT: u8 = 4;

const HIGH_POINT_LEN: usize = 14;
const LOW_POINT_LEN: usize = 8;
const SPHERICAL_POINT_LEN: usize = 10;
const IMU_PAYLOAD_LEN: usize = 24;

/// `time_interval` counts 0.1 microsecond ticks.
const TIME_INTERVAL_NANOS: u64 = 100;

/// The lidar reports acceleration in g, ROS wants metres per second squared.
const STANDARD_GRAVITY: f64 = 9.80665;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimeSource {
    /// Free-running lidar clock. Not an epoch time, so the recorder has to
    /// re-base it against the host clock before anything downstream is happy.
    LidarLocal,
    Gptp,
    GpsPps,
    PpsOnly,
    Unknown(u8),
}

impl TimeSource {
    fn from_code(code: u8) -> Self {
        match code {
            0 => TimeSource::LidarLocal,
            1 => TimeSource::Gptp,
            2 => TimeSource::GpsPps,
            3 => TimeSource::PpsOnly,
            other => TimeSource::Unknown(other),
        }
    }

    /// Only a synchronised clock can be trusted as an absolute timestamp.
    pub fn is_absolute(self) -> bool {
        matches!(self, TimeSource::Gptp | TimeSource::GpsPps)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PacketHeader {
    pub version: u8,
    pub length: u16,
    /// Span covered by the points in this packet, in 0.1 us ticks.
    pub time_interval: u16,
    pub dot_num: u16,
    pub udp_cnt: u16,
    pub frame_cnt: u8,
    pub data_type: u8,
    pub time_source: TimeSource,
    pub timestamp: u64,
}

impl PacketHeader {
    pub fn parse(packet: &[u8]) -> Result<PacketHeader> {
        if packet.len() < HEADER_LEN {
            bail!("livox packet is {} bytes, shorter than its 36 byte header", packet.len());
        }
        Ok(PacketHeader {
            version: packet[0],
            length: u16::from_le_bytes([packet[1], packet[2]]),
            time_interval: u16::from_le_bytes([packet[3], packet[4]]),
            dot_num: u16::from_le_bytes([packet[5], packet[6]]),
            udp_cnt: u16::from_le_bytes([packet[7], packet[8]]),
            frame_cnt: packet[9],
            data_type: packet[10],
            time_source: TimeSource::from_code(packet[11]),
            // Unaligned in the struct, so it must be read byte-wise.
            timestamp: u64::from_le_bytes(packet[28..36].try_into().unwrap()),
        })
    }

    /// Nanoseconds between consecutive points inside this packet.
    ///
    /// The SDK does not supply per-point timestamps, so this is derived from
    /// `time_interval`, which is the span the packet's points cover. Verified
    /// against hardware: 4750 ticks over 96 points is 4948 ns apart, and the
    /// 475 us total matches the observed 480 us packet cadence.
    pub fn point_spacing_nanos(&self) -> u64 {
        if self.dot_num == 0 {
            return 0;
        }
        self.time_interval as u64 * TIME_INTERVAL_NANOS / self.dot_num as u64
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub reflectivity: u8,
    pub tag: u8,
    /// Absolute, in the lidar's own time base.
    pub timestamp_nanos: u64,
    pub line: u8,
}

#[derive(Debug)]
pub enum Decoded {
    Points(Vec<Point>),
    Imu(ImuSample),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    /// rad/s.
    pub angular_velocity: [f64; 3],
    /// m/s^2, already converted out of g.
    pub linear_acceleration: [f64; 3],
    pub timestamp_nanos: u64,
}

/// Decodes one UDP datagram. `line_offset` is the running point index, which is
/// what determines the scan line each point is tagged with.
pub fn decode_packet(packet: &[u8], line_offset: u64) -> Result<Decoded> {
    let header = PacketHeader::parse(packet)?;
    let body = &packet[HEADER_LEN..];
    match header.data_type {
        DATA_TYPE_IMU => Ok(Decoded::Imu(decode_imu(&header, body)?)),
        DATA_TYPE_CARTESIAN_HIGH => Ok(Decoded::Points(decode_cartesian(
            &header,
            body,
            line_offset,
            HIGH_POINT_LEN,
        )?)),
        DATA_TYPE_CARTESIAN_LOW => Ok(Decoded::Points(decode_cartesian(
            &header,
            body,
            line_offset,
            LOW_POINT_LEN,
        )?)),
        DATA_TYPE_SPHERICAL => Ok(Decoded::Points(decode_spherical(&header, body, line_offset)?)),
        other => bail!("unsupported livox data_type 0x{other:02x}"),
    }
}

fn decode_imu(header: &PacketHeader, body: &[u8]) -> Result<ImuSample> {
    if body.len() < IMU_PAYLOAD_LEN {
        bail!("livox imu payload is {} bytes, need 24", body.len());
    }
    let value = |index: usize| {
        f32::from_le_bytes(body[index * 4..index * 4 + 4].try_into().unwrap()) as f64
    };
    Ok(ImuSample {
        angular_velocity: [value(0), value(1), value(2)],
        linear_acceleration: [
            value(3) * STANDARD_GRAVITY,
            value(4) * STANDARD_GRAVITY,
            value(5) * STANDARD_GRAVITY,
        ],
        timestamp_nanos: header.timestamp,
    })
}

fn decode_cartesian(
    header: &PacketHeader,
    body: &[u8],
    line_offset: u64,
    point_len: usize,
) -> Result<Vec<Point>> {
    let count = header.dot_num as usize;
    if body.len() < count * point_len {
        bail!(
            "livox packet claims {count} points but carries {} bytes of payload",
            body.len()
        );
    }
    // High precision is millimetres in i32, low precision centimetres in i16.
    let scale = if point_len == HIGH_POINT_LEN { 0.001 } else { 0.01 };
    let spacing = header.point_spacing_nanos();
    let mut points = Vec::with_capacity(count);
    for index in 0..count {
        let entry = &body[index * point_len..(index + 1) * point_len];
        let (x, y, z, reflectivity, tag) = if point_len == HIGH_POINT_LEN {
            let axis = |at: usize| i32::from_le_bytes(entry[at..at + 4].try_into().unwrap()) as f32;
            (axis(0), axis(4), axis(8), entry[12], entry[13])
        } else {
            let axis = |at: usize| i16::from_le_bytes(entry[at..at + 2].try_into().unwrap()) as f32;
            (axis(0), axis(2), axis(4), entry[6], entry[7])
        };
        points.push(Point {
            x: x * scale,
            y: y * scale,
            z: z * scale,
            reflectivity,
            tag,
            timestamp_nanos: header.timestamp + spacing * index as u64,
            line: ((line_offset + index as u64) % LINE_COUNT as u64) as u8,
        });
    }
    Ok(points)
}

fn decode_spherical(header: &PacketHeader, body: &[u8], line_offset: u64) -> Result<Vec<Point>> {
    let count = header.dot_num as usize;
    if body.len() < count * SPHERICAL_POINT_LEN {
        bail!("livox spherical packet is short");
    }
    let spacing = header.point_spacing_nanos();
    let mut points = Vec::with_capacity(count);
    for index in 0..count {
        let entry = &body[index * SPHERICAL_POINT_LEN..(index + 1) * SPHERICAL_POINT_LEN];
        let depth = u32::from_le_bytes(entry[0..4].try_into().unwrap()) as f32 * 0.001;
        // Zenith and azimuth arrive in hundredths of a degree.
        let zenith = u16::from_le_bytes(entry[4..6].try_into().unwrap()) as f32 * 0.01;
        let azimuth = u16::from_le_bytes(entry[6..8].try_into().unwrap()) as f32 * 0.01;
        let (zenith, azimuth) = (zenith.to_radians(), azimuth.to_radians());
        points.push(Point {
            x: depth * zenith.sin() * azimuth.cos(),
            y: depth * zenith.sin() * azimuth.sin(),
            z: depth * zenith.cos(),
            reflectivity: entry[8],
            tag: entry[9],
            timestamp_nanos: header.timestamp + spacing * index as u64,
            line: ((line_offset + index as u64) % LINE_COUNT as u64) as u8,
        });
    }
    Ok(points)
}

/// Byte layout of the `PointCloud2` we emit, matching `livox_ros_driver2` so
/// existing tooling reads it, plus a `offset_time` that fills what would
/// otherwise be alignment padding.
pub const POINT_STEP: u32 = 32;
const OFFSET_X: usize = 0;
const OFFSET_Y: usize = 4;
const OFFSET_Z: usize = 8;
const OFFSET_INTENSITY: usize = 12;
const OFFSET_TAG: usize = 16;
const OFFSET_LINE: usize = 17;
const OFFSET_OFFSET_TIME: usize = 20;
const OFFSET_TIMESTAMP: usize = 24;

pub fn point_fields() -> Vec<PointField> {
    let field = |name: &str, offset: usize, datatype: u8| PointField {
        name: name.to_string(),
        offset: offset as u32,
        datatype,
        count: 1,
    };
    vec![
        field("x", OFFSET_X, POINT_FIELD_FLOAT32),
        field("y", OFFSET_Y, POINT_FIELD_FLOAT32),
        field("z", OFFSET_Z, POINT_FIELD_FLOAT32),
        field("intensity", OFFSET_INTENSITY, POINT_FIELD_FLOAT32),
        field("tag", OFFSET_TAG, POINT_FIELD_UINT8),
        field("line", OFFSET_LINE, POINT_FIELD_UINT8),
        field("offset_time", OFFSET_OFFSET_TIME, POINT_FIELD_UINT32),
        field("timestamp", OFFSET_TIMESTAMP, POINT_FIELD_FLOAT64),
    ]
}

/// Collects packets into fixed-duration clouds. `frame_cnt` in the packet header
/// does not actually increment on current Mid-360 firmware, so binning by time is
/// the only option.
pub struct FrameAccumulator {
    frame_nanos: u64,
    frame_id: String,
    points: Vec<Point>,
    frame_start: Option<u64>,
    line_counter: u64,
    /// Offset added to lidar-local stamps to put them on the host clock.
    clock_offset_nanos: i64,
    clock_locked: bool,
    /// Voxel edge length in metres, applied just before packing. Zero keeps
    /// every point. Thinning here rather than after packing avoids unpacking a
    /// cloud that was only just assembled.
    voxel_leaf_size: f32,
}

impl FrameAccumulator {
    pub fn new(frame_hz: f64, frame_id: impl Into<String>) -> Self {
        FrameAccumulator {
            frame_nanos: (NANOS_PER_SEC as f64 / frame_hz) as u64,
            frame_id: frame_id.into(),
            points: Vec::with_capacity(32_768),
            frame_start: None,
            line_counter: 0,
            clock_offset_nanos: 0,
            clock_locked: false,
            voxel_leaf_size: 0.0,
        }
    }

    pub fn with_voxel_leaf_size(mut self, leaf_size: f32) -> Self {
        self.voxel_leaf_size = leaf_size;
        self
    }

    /// Pins the lidar's free-running clock to the host clock using the first
    /// packet seen. Without this every stamp in the file is an arbitrary number
    /// of seconds since the lidar booted.
    fn lock_clock(&mut self, lidar_nanos: u64, host_nanos: u64, source: TimeSource) {
        if self.clock_locked {
            return;
        }
        self.clock_locked = true;
        self.clock_offset_nanos = if source.is_absolute() {
            0
        } else {
            host_nanos as i64 - lidar_nanos as i64
        };
    }

    fn to_host_nanos(&self, lidar_nanos: u64) -> u64 {
        (lidar_nanos as i64 + self.clock_offset_nanos).max(0) as u64
    }

    /// Feeds one datagram in. Returns a cloud whenever a frame boundary is
    /// crossed. `host_nanos` is the wall clock at receipt.
    pub fn push_packet(&mut self, packet: &[u8], host_nanos: u64) -> Result<Option<PointCloud2>> {
        let header = PacketHeader::parse(packet)?;
        self.lock_clock(header.timestamp, host_nanos, header.time_source);
        let decoded = decode_packet(packet, self.line_counter)?;
        let Decoded::Points(points) = decoded else {
            return Ok(None);
        };
        self.line_counter += points.len() as u64;

        let start = *self.frame_start.get_or_insert(header.timestamp);
        let mut finished = None;
        if header.timestamp.saturating_sub(start) >= self.frame_nanos {
            finished = self.flush();
            self.frame_start = Some(header.timestamp);
        }
        self.points.extend(points);
        Ok(finished)
    }

    /// Decodes an IMU datagram onto the same re-based clock.
    pub fn push_imu(&mut self, packet: &[u8], host_nanos: u64, frame_id: &str) -> Result<Imu> {
        let header = PacketHeader::parse(packet)?;
        self.lock_clock(header.timestamp, host_nanos, header.time_source);
        let Decoded::Imu(sample) = decode_packet(packet, 0)? else {
            bail!("expected an imu packet");
        };
        Ok(Imu::unoriented(
            Header::new(self.to_host_nanos(sample.timestamp_nanos), frame_id),
            sample.angular_velocity,
            sample.linear_acceleration,
        ))
    }

    /// Emits whatever has accumulated, even if the frame is not full.
    pub fn flush(&mut self) -> Option<PointCloud2> {
        if self.points.is_empty() {
            return None;
        }
        let points = std::mem::replace(&mut self.points, Vec::with_capacity(32_768));
        let base = self.to_host_nanos(points[0].timestamp_nanos);
        let points = if self.voxel_leaf_size > 0.0 {
            voxel_downsample(&points, self.voxel_leaf_size)
        } else {
            points
        };
        Some(build_cloud(&points, base, &self.frame_id, self.clock_offset_nanos))
    }

    pub fn pending_points(&self) -> usize {
        self.points.len()
    }
}

pub fn build_cloud(
    points: &[Point],
    base_nanos: u64,
    frame_id: &str,
    clock_offset_nanos: i64,
) -> PointCloud2 {
    let mut data = vec![0u8; points.len() * POINT_STEP as usize];
    for (index, point) in points.iter().enumerate() {
        let entry = &mut data[index * POINT_STEP as usize..(index + 1) * POINT_STEP as usize];
        entry[OFFSET_X..OFFSET_X + 4].copy_from_slice(&point.x.to_le_bytes());
        entry[OFFSET_Y..OFFSET_Y + 4].copy_from_slice(&point.y.to_le_bytes());
        entry[OFFSET_Z..OFFSET_Z + 4].copy_from_slice(&point.z.to_le_bytes());
        entry[OFFSET_INTENSITY..OFFSET_INTENSITY + 4]
            .copy_from_slice(&(point.reflectivity as f32).to_le_bytes());
        entry[OFFSET_TAG] = point.tag;
        entry[OFFSET_LINE] = point.line;
        let absolute = (point.timestamp_nanos as i64 + clock_offset_nanos).max(0) as u64;
        let offset = absolute.saturating_sub(base_nanos).min(u32::MAX as u64) as u32;
        entry[OFFSET_OFFSET_TIME..OFFSET_OFFSET_TIME + 4].copy_from_slice(&offset.to_le_bytes());
        let seconds = absolute as f64 / NANOS_PER_SEC as f64;
        entry[OFFSET_TIMESTAMP..OFFSET_TIMESTAMP + 8].copy_from_slice(&seconds.to_le_bytes());
    }
    PointCloud2 {
        header: Header::new(base_nanos, frame_id),
        height: 1,
        width: points.len() as u32,
        fields: point_fields(),
        is_bigendian: false,
        point_step: POINT_STEP,
        row_step: POINT_STEP * points.len() as u32,
        data,
        is_dense: true,
    }
}

/// Keeps the first point landing in each `leaf_size` cube. Chosen over centroid
/// averaging because averaging invents a point that was never measured, which
/// destroys the per-point timestamps that de-skewing depends on.
pub fn voxel_downsample(points: &[Point], leaf_size: f32) -> Vec<Point> {
    if leaf_size <= 0.0 {
        return points.to_vec();
    }
    let mut seen = std::collections::HashSet::with_capacity(points.len());
    let mut kept = Vec::with_capacity(points.len() / 2);
    for point in points {
        let key = (
            (point.x / leaf_size).floor() as i32,
            (point.y / leaf_size).floor() as i32,
            (point.z / leaf_size).floor() as i32,
        );
        if seen.insert(key) {
            kept.push(*point);
        }
    }
    kept
}

/// One datagram as it came off the wire, with the port it arrived on and the
/// host time it was seen.
#[derive(Debug, Clone)]
pub struct CapturedPacket {
    pub port: u16,
    pub received_nanos: u64,
    pub payload: Vec<u8>,
}

/// Real datagrams captured from the Mid-360 at 192.168.1.189 on the Alfred
/// Jetson, framed as `port: u16, length: u16, received_nanos: u64, payload`.
pub const CAPTURE: &[u8] = include_bytes!("../tests/data/livox_mid360_capture.bin");

/// Splits the capture framing. Public because the integration test replays these
/// bytes over real sockets, and one parser beats two that can disagree.
pub fn parse_capture(bytes: &[u8]) -> Vec<CapturedPacket> {
    let mut packets = Vec::new();
    let mut offset = 0;
    while offset + 12 <= bytes.len() {
        let port = u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap());
        let length = u16::from_le_bytes(bytes[offset + 2..offset + 4].try_into().unwrap()) as usize;
        let received_nanos = u64::from_le_bytes(bytes[offset + 4..offset + 12].try_into().unwrap());
        offset += 12;
        packets.push(CapturedPacket {
            port,
            received_nanos,
            payload: bytes[offset..offset + length].to_vec(),
        });
        offset += length;
    }
    packets
}

#[cfg(test)]
mod tests {
    use super::*;

    fn captured_packets() -> Vec<(u16, u64, Vec<u8>)> {
        parse_capture(CAPTURE)
            .into_iter()
            .map(|packet| (packet.port, packet.received_nanos, packet.payload))
            .collect()
    }

    fn point_packets() -> Vec<(u64, Vec<u8>)> {
        captured_packets()
            .into_iter()
            .filter(|(port, _, _)| *port == HOST_POINT_PORT)
            .map(|(_, received, packet)| (received, packet))
            .collect()
    }

    fn imu_packets() -> Vec<(u64, Vec<u8>)> {
        captured_packets()
            .into_iter()
            .filter(|(port, _, _)| *port == HOST_IMU_PORT)
            .map(|(_, received, packet)| (received, packet))
            .collect()
    }

    #[test]
    fn the_capture_fixture_holds_both_streams() {
        assert_eq!(point_packets().len(), 208);
        assert_eq!(imu_packets().len(), 20);
    }

    #[test]
    fn a_real_point_packet_header_matches_the_documented_layout() {
        let (_, packet) = point_packets().remove(0);
        let header = PacketHeader::parse(&packet).unwrap();
        assert_eq!(packet.len(), 1380);
        assert_eq!(header.length, 1380);
        assert_eq!(header.dot_num, 96);
        assert_eq!(header.data_type, DATA_TYPE_CARTESIAN_HIGH);
        assert_eq!(header.time_source, TimeSource::LidarLocal);
        // 36 byte header + 96 points at 14 bytes each is exactly the datagram.
        assert_eq!(HEADER_LEN + header.dot_num as usize * HIGH_POINT_LEN, packet.len());
    }

    #[test]
    fn real_points_decode_to_plausible_metres() {
        let (_, packet) = point_packets().remove(0);
        let Decoded::Points(points) = decode_packet(&packet, 0).unwrap() else {
            panic!("expected points");
        };
        assert_eq!(points.len(), 96);
        let ranges: Vec<f32> = points
            .iter()
            .map(|point| (point.x * point.x + point.y * point.y + point.z * point.z).sqrt())
            .collect();
        // The Mid-360 sees out to 70 m; anything past that means a scale error.
        assert!(
            ranges.iter().all(|range| *range < 100.0),
            "implausible range: {:?}",
            ranges.iter().cloned().fold(0.0f32, f32::max)
        );
        assert!(
            ranges.iter().any(|range| *range > 0.1),
            "every point was at the origin, so the scale factor is wrong"
        );
    }

    #[test]
    fn per_point_timestamps_rise_and_land_inside_the_packet_interval() {
        let (_, packet) = point_packets().remove(0);
        let header = PacketHeader::parse(&packet).unwrap();
        let Decoded::Points(points) = decode_packet(&packet, 0).unwrap() else {
            panic!("expected points");
        };
        assert_eq!(points[0].timestamp_nanos, header.timestamp);
        for pair in points.windows(2) {
            assert!(
                pair[1].timestamp_nanos > pair[0].timestamp_nanos,
                "per-point timestamps must strictly increase"
            );
        }
        let span = points.last().unwrap().timestamp_nanos - points[0].timestamp_nanos;
        let declared = header.time_interval as u64 * TIME_INTERVAL_NANOS;
        assert!(
            span < declared,
            "the last point at {span} ns must fall inside the declared {declared} ns interval"
        );
    }

    /// The inferred spacing is only trustworthy if it lines up with the gap
    /// between the base stamps of consecutive packets, which is measured, not
    /// documented. Hardware shows 480 us cadence against a 475 us declared span.
    #[test]
    fn inferred_point_spacing_agrees_with_the_packet_cadence() {
        let packets = point_packets();
        let first = PacketHeader::parse(&packets[0].1).unwrap();
        let second = PacketHeader::parse(&packets[1].1).unwrap();
        let cadence = second.timestamp - first.timestamp;
        let covered = first.point_spacing_nanos() * first.dot_num as u64;
        assert!(
            covered <= cadence,
            "points would overrun into the next packet: {covered} ns of points every {cadence} ns"
        );
        // Within 5% of back to back, i.e. no large unexplained dead time.
        assert!(
            covered * 100 / cadence >= 95,
            "only {covered} ns of the {cadence} ns cadence is accounted for"
        );
    }

    #[test]
    fn real_imu_packets_decode_to_roughly_one_g() {
        let packets = imu_packets();
        assert!(!packets.is_empty());
        for (_, packet) in &packets {
            let header = PacketHeader::parse(packet).unwrap();
            assert_eq!(header.data_type, DATA_TYPE_IMU);
            assert_eq!(header.dot_num, 1);
            assert_eq!(packet.len(), HEADER_LEN + IMU_PAYLOAD_LEN);
            let Decoded::Imu(sample) = decode_packet(packet, 0).unwrap() else {
                panic!("expected imu");
            };
            let magnitude = sample
                .linear_acceleration
                .iter()
                .map(|axis| axis * axis)
                .sum::<f64>()
                .sqrt();
            assert!(
                (magnitude - STANDARD_GRAVITY).abs() < 1.5,
                "a stationary lidar should read about 9.81 m/s^2, got {magnitude}"
            );
            assert!(
                sample.angular_velocity.iter().all(|rate| rate.abs() < 1.0),
                "a stationary lidar should not be spinning: {:?}",
                sample.angular_velocity
            );
        }
    }

    #[test]
    fn the_imu_stream_runs_at_about_200_hz() {
        let packets = imu_packets();
        let first = PacketHeader::parse(&packets[0].1).unwrap();
        let last = PacketHeader::parse(&packets.last().unwrap().1).unwrap();
        let span = last.timestamp - first.timestamp;
        let rate = (packets.len() - 1) as f64 * NANOS_PER_SEC as f64 / span as f64;
        assert!(
            (rate - 200.0).abs() < 20.0,
            "expected about 200 Hz of imu, measured {rate:.1} Hz"
        );
    }

    #[test]
    fn a_frame_of_real_packets_becomes_one_ten_hz_cloud() {
        let mut accumulator = FrameAccumulator::new(10.0, "livox_frame");
        let host_base = 1_700_000_000_000_000_000u64;
        let mut clouds = Vec::new();
        for (index, (_, packet)) in point_packets().into_iter().enumerate() {
            if let Some(cloud) = accumulator
                .push_packet(&packet, host_base + index as u64 * 480_000)
                .unwrap()
            {
                clouds.push(cloud);
            }
        }
        clouds.extend(accumulator.flush());
        assert!(!clouds.is_empty(), "208 packets is a full 100 ms frame");

        let cloud = &clouds[0];
        assert_eq!(cloud.point_step, POINT_STEP);
        assert_eq!(cloud.height, 1);
        assert_eq!(cloud.data.len(), cloud.width as usize * POINT_STEP as usize);
        assert_eq!(cloud.row_step, cloud.width * POINT_STEP);
        // 200,000 points per second means about 20,000 in a 100 ms frame.
        assert!(
            (15_000..=25_000).contains(&cloud.width),
            "a 10 Hz frame should hold about 20k points, got {}",
            cloud.width
        );
        // The clock was re-based onto the host, so the stamp is a real epoch time.
        assert!(
            cloud.header.stamp_nanos() > 1_600_000_000 * NANOS_PER_SEC,
            "lidar-local time leaked into the header instead of host time"
        );
    }

    #[test]
    fn every_point_in_a_real_cloud_carries_a_rising_offset_time() {
        let mut accumulator = FrameAccumulator::new(10.0, "livox_frame");
        for (index, (_, packet)) in point_packets().into_iter().enumerate() {
            accumulator
                .push_packet(&packet, 1_700_000_000_000_000_000 + index as u64 * 480_000)
                .unwrap();
        }
        let cloud = accumulator.flush().unwrap();
        let offset_at = |index: usize| {
            let entry = &cloud.data[index * POINT_STEP as usize..];
            u32::from_le_bytes(entry[OFFSET_OFFSET_TIME..OFFSET_OFFSET_TIME + 4].try_into().unwrap())
        };
        assert_eq!(offset_at(0), 0);
        let last = offset_at(cloud.width as usize - 1);
        // A 100 ms frame, so the final point is offset by about 100 ms.
        assert!(
            (80_000_000..=120_000_000).contains(&last),
            "last point offset {last} ns is not a 100 ms frame"
        );
        for index in 1..cloud.width as usize {
            assert!(offset_at(index) >= offset_at(index - 1), "offset_time went backwards");
        }
    }

    #[test]
    fn scan_lines_cycle_through_all_four() {
        let (_, packet) = point_packets().remove(0);
        let Decoded::Points(points) = decode_packet(&packet, 0).unwrap() else {
            panic!("expected points");
        };
        let lines: std::collections::HashSet<u8> = points.iter().map(|point| point.line).collect();
        assert_eq!(lines.len(), LINE_COUNT as usize);
    }

    #[test]
    fn voxel_downsampling_thins_a_real_cloud_and_keeps_timestamps() {
        let (_, packet) = point_packets().remove(0);
        let Decoded::Points(points) = decode_packet(&packet, 0).unwrap() else {
            panic!("expected points");
        };
        let coarse = voxel_downsample(&points, 1.0);
        assert!(
            coarse.len() < points.len(),
            "a 1 m leaf should merge at least some of 96 points"
        );
        assert!(!coarse.is_empty());
        // Kept points are originals, not invented averages, so their stamps are real.
        for point in &coarse {
            assert!(points.contains(point));
        }
        assert_eq!(voxel_downsample(&points, 0.0).len(), points.len());
    }

    #[test]
    fn a_truncated_packet_is_rejected_rather_than_read_out_of_bounds() {
        let (_, packet) = point_packets().remove(0);
        assert!(PacketHeader::parse(&packet[..10]).is_err());
        assert!(decode_packet(&packet[..HEADER_LEN + 10], 0).is_err());
    }

    #[test]
    fn an_unknown_data_type_is_reported_not_guessed() {
        let (_, mut packet) = point_packets().remove(0);
        packet[10] = 0x7f;
        assert!(decode_packet(&packet, 0).is_err());
    }

    /// Low precision cartesian is centimetres in i16 rather than millimetres in
    /// i32, and getting the scale backwards is a silent 10x error.
    #[test]
    fn low_precision_points_use_the_centimetre_scale() {
        let mut packet = vec![0u8; HEADER_LEN + LOW_POINT_LEN];
        let length = packet.len() as u16;
        packet[1..3].copy_from_slice(&length.to_le_bytes());
        packet[3..5].copy_from_slice(&100u16.to_le_bytes());
        packet[5..7].copy_from_slice(&1u16.to_le_bytes());
        packet[10] = DATA_TYPE_CARTESIAN_LOW;
        packet[28..36].copy_from_slice(&1_000u64.to_le_bytes());
        packet[HEADER_LEN..HEADER_LEN + 2].copy_from_slice(&500i16.to_le_bytes());
        let Decoded::Points(points) = decode_packet(&packet, 0).unwrap() else {
            panic!("expected points");
        };
        assert!((points[0].x - 5.0).abs() < 1e-6, "500 cm should be 5 m, got {}", points[0].x);
    }
}
