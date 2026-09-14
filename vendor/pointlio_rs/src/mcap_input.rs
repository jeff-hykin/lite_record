//! Ingest a ROS2-CDR `.mcap` recording into [`SyncPackage`]s.
//!
//! The rig that produces these writes the Mid-360 through `lite_record`, so a
//! scan arrives as one `sensor_msgs/PointCloud2` at ~10 Hz and the IMU as
//! `sensor_msgs/Imu` at ~200 Hz. Unlike the raw SDK2 capture that [`crate::pcap`]
//! reads, the accelerations here are already in m/s^2 and the per-point times
//! have already been pulled onto the host clock, so neither is rescaled.
//!
//! Scans are handed to a callback rather than collected: an hour of Mid-360
//! data will not fit in memory, and the estimator only ever looks at one scan.

use crate::config::Config;
use crate::types::{ImuData, Point, SyncPackage, V3D};

/// Little-endian CDR, enough of it for the two messages read here.
struct Cdr<'a> {
    data: &'a [u8],
    at: usize,
}

impl<'a> Cdr<'a> {
    fn new(data: &'a [u8]) -> Option<Self> {
        // Byte 1 of the encapsulation header picks the endianness. Everything
        // this crate reads is little-endian; a big-endian bag is a different
        // recorder and would silently decode to nonsense.
        if data.len() < 4 || data[1] & 1 != 1 {
            return None;
        }
        Some(Cdr { data, at: 4 })
    }

    fn align(&mut self, width: usize) {
        self.at += (width - ((self.at - 4) % width)) % width;
    }

    fn u8(&mut self) -> u8 {
        let value = self.data[self.at];
        self.at += 1;
        value
    }

    fn u32(&mut self) -> u32 {
        self.align(4);
        let value = u32::from_le_bytes(self.data[self.at..self.at + 4].try_into().unwrap());
        self.at += 4;
        value
    }

    fn f64(&mut self) -> f64 {
        self.align(8);
        let value = f64::from_le_bytes(self.data[self.at..self.at + 8].try_into().unwrap());
        self.at += 8;
        value
    }

    fn string(&mut self) -> String {
        let length = self.u32() as usize;
        let text = String::from_utf8_lossy(&self.data[self.at..self.at + length - 1]).into_owned();
        self.at += length;
        text
    }

    fn skip_f64(&mut self, count: usize) {
        for _ in 0..count {
            self.f64();
        }
    }

    /// Returns the stamp in seconds, discarding the frame id.
    fn header(&mut self) -> f64 {
        let seconds = self.u32() as i32 as f64;
        let nanos = self.u32() as f64;
        self.string();
        seconds + nanos * 1e-9
    }
}

pub fn decode_imu(data: &[u8]) -> Option<ImuData> {
    let mut cdr = Cdr::new(data)?;
    let time = cdr.header();
    cdr.skip_f64(4 + 9); // orientation and its covariance
    let gyro = V3D::new(cdr.f64(), cdr.f64(), cdr.f64());
    cdr.skip_f64(9);
    let acc = V3D::new(cdr.f64(), cdr.f64(), cdr.f64());
    Some(ImuData { acc, gyro, time })
}

/// A decoded scan: the points in the LiDAR frame with absolute per-point times.
pub struct Scan {
    pub points: Vec<(f32, f32, f32, f32, f64)>,
    pub start: f64,
    pub end: f64,
}

/// Where each value sits inside one point, worked out from the message's own
/// `fields` rather than assumed, because a different driver lays them out
/// differently even when the topic name matches.
struct Layout {
    x: usize,
    y: usize,
    z: usize,
    intensity: Option<usize>,
    /// Absolute time per point, `float64` seconds. Preferred over `offset_time`
    /// because it needs no reference to the header stamp.
    timestamp: Option<usize>,
    /// Time since the scan start, `uint32` nanoseconds.
    offset_time: Option<usize>,
}

pub fn decode_cloud(data: &[u8], cfg: &Config) -> Option<Scan> {
    let mut cdr = Cdr::new(data)?;
    let stamp = cdr.header();
    let _height = cdr.u32();
    let width = cdr.u32() as usize;

    let count = cdr.u32() as usize;
    let mut layout =
        Layout { x: 0, y: 4, z: 8, intensity: None, timestamp: None, offset_time: None };
    let mut found_xyz = 0;
    for _ in 0..count {
        let name = cdr.string();
        let offset = cdr.u32() as usize;
        let _datatype = cdr.u8();
        let _count = cdr.u32();
        match name.as_str() {
            "x" => {
                layout.x = offset;
                found_xyz += 1;
            }
            "y" => {
                layout.y = offset;
                found_xyz += 1;
            }
            "z" => {
                layout.z = offset;
                found_xyz += 1;
            }
            "intensity" | "reflectivity" => layout.intensity = Some(offset),
            "timestamp" | "t" => layout.timestamp = Some(offset),
            "offset_time" => layout.offset_time = Some(offset),
            _ => {}
        }
    }
    if found_xyz != 3 {
        return None;
    }

    let _is_bigendian = cdr.u8();
    let point_step = cdr.u32() as usize;
    let _row_step = cdr.u32();
    let length = cdr.u32() as usize;
    let body = &cdr.data[cdr.at..cdr.at + length];

    let filter_num = cfg.point_filter_num.max(1) as usize;
    let min_r2 = cfg.blind * cfg.blind;
    let max_r2 = cfg.max_range * cfg.max_range;

    let mut points = Vec::with_capacity(width / filter_num + 1);
    let read_f32 = |entry: &[u8], at: usize| {
        f32::from_le_bytes(entry[at..at + 4].try_into().unwrap())
    };
    for index in 0..width {
        let entry = match body.get(index * point_step..(index + 1) * point_step) {
            Some(entry) => entry,
            None => break,
        };
        if !index.is_multiple_of(filter_num) {
            continue;
        }
        let x = read_f32(entry, layout.x) as f64;
        let y = read_f32(entry, layout.y) as f64;
        let z = read_f32(entry, layout.z) as f64;
        let r2 = x * x + y * y + z * z;
        if r2 < min_r2 || r2 > max_r2 {
            continue;
        }
        let intensity = layout.intensity.map(|at| read_f32(entry, at)).unwrap_or(0.0);
        let time = match (layout.timestamp, layout.offset_time) {
            (Some(at), _) => f64::from_le_bytes(entry[at..at + 8].try_into().unwrap()),
            (None, Some(at)) => {
                let nanos = u32::from_le_bytes(entry[at..at + 4].try_into().unwrap());
                stamp + nanos as f64 * 1e-9
            }
            (None, None) => stamp,
        };
        points.push((x as f32, y as f32, z as f32, intensity, time));
    }
    if points.is_empty() {
        return None;
    }

    // The returns come off the sensor in time order, but a downsample upstream
    // can reorder them, and the propagation is driven by these times.
    let start = points.iter().map(|p| p.4).fold(f64::INFINITY, f64::min);
    let end = points.iter().map(|p| p.4).fold(f64::NEG_INFINITY, f64::max);
    Some(Scan { points, start, end })
}

/// Hands every message to `handle`, in LOG-TIME order, stopping when it returns
/// false.
///
/// Two reasons this is not `mcap::MessageStream`. That reader is **linear**, so
/// it learns channels as it passes their records and dies with `Message N
/// referenced unknown channel M` on a file that declares one later — legal,
/// since the summary is what a reader resolves channels from. And **file order
/// is not log order**: a tool that rewrites chunks in place puts the rewritten
/// ones at the end, so a recording can run to its last second and then jump
/// back to its first. An estimator fed that does not fail, it quietly returns a
/// worse trajectory with the teleports rejected, which is far harder to notice.
///
/// So merge the chunks by log time, opening each only when a message could come
/// out of it, and keep the linear read for a file with no summary — one the
/// recorder was killed part way through.
fn for_each_message(
    mapped: &[u8],
    mut handle: impl FnMut(&mcap::Message<'_>) -> Result<bool, Box<dyn std::error::Error>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let indexed = match mcap::Summary::read(mapped) {
        Ok(Some(summary)) if !summary.chunk_indexes.is_empty() => Some(summary),
        _ => None,
    };
    let Some(summary) = indexed else {
        for message in mcap::MessageStream::new(mapped)? {
            if !handle(&message?)? {
                return Ok(());
            }
        }
        return Ok(());
    };

    let mut chunks = summary.chunk_indexes.clone();
    chunks.sort_by_key(|chunk| (chunk.message_start_time, chunk.chunk_start_offset));

    type Rest<'a> = Box<dyn Iterator<Item = mcap::McapResult<mcap::Message<'a>>> + 'a>;
    let mut open: Vec<(mcap::Message<'_>, Rest<'_>)> = Vec::new();
    let mut unopened = 0;

    loop {
        loop {
            let earliest = open.iter().map(|(message, _)| message.log_time).min();
            let should_open = match (unopened < chunks.len(), earliest) {
                (false, _) => false,
                (true, None) => true,
                (true, Some(time)) => chunks[unopened].message_start_time <= time,
            };
            if !should_open {
                break;
            }
            let mut rest = summary.stream_chunk(mapped, &chunks[unopened])?;
            unopened += 1;
            if let Some(first) = rest.next() {
                open.push((first?, Box::new(rest)));
            }
        }
        let Some(next) = open
            .iter()
            .enumerate()
            .min_by_key(|(_, (message, _))| message.log_time)
            .map(|(index, _)| index)
        else {
            return Ok(());
        };
        let following = open[next].1.next().transpose()?;
        let message = match following {
            Some(following) => std::mem::replace(&mut open[next].0, following),
            None => open.swap_remove(next).0,
        };
        if !handle(&message)? {
            return Ok(());
        }
    }
}

/// Seconds to add to a scan's own timestamps to land on the clock the recording
/// was logged with.
///
/// The Livox and RealSense stamps in these recordings run about half an hour
/// behind the host clock in the message log times. Both are unix-epoch, so
/// nothing complains; a trajectory estimated from the sensor stamps just lands
/// somewhere a player's timeline never reaches, and anything matching odometry
/// to scans by log time pairs every scan with the same pose.
pub fn log_time_offset(
    mapped: &[u8],
    cfg: &Config,
    lidar_topic: &str,
) -> Result<Option<f64>, Box<dyn std::error::Error>> {
    let mut offset = None;
    for_each_message(mapped, |message| {
        if message.channel.topic != lidar_topic {
            return Ok(true);
        }
        match decode_cloud(&message.data, cfg) {
            Some(scan) => {
                offset = Some(message.log_time as f64 * 1e-9 - scan.start);
                Ok(false)
            }
            None => Ok(true),
        }
    })?;
    Ok(offset)
}

/// The lidar message a `SyncPackage` was built from, for a caller that needs
/// more of the cloud than the estimator kept. The package's points are
/// downsampled and range-filtered; these bytes are the whole scan as recorded.
pub struct RawScan<'a> {
    pub data: &'a [u8],
    pub log_time: u64,
}

/// Walks the recording in file order, handing each scan and the IMU samples
/// that cover it to `handle`. `duration_s` of 0 reads the whole file.
pub fn for_each_package(
    mapped: &[u8],
    cfg: &Config,
    duration_s: f64,
    lidar_topic: &str,
    imu_topic: &str,
    mut handle: impl FnMut(SyncPackage),
) -> Result<usize, Box<dyn std::error::Error>> {
    for_each_package_raw(mapped, cfg, duration_s, lidar_topic, imu_topic, |package, _raw| handle(package))
}

/// [`for_each_package`], also handing over the lidar message each package came
/// from.
pub fn for_each_package_raw(
    mapped: &[u8],
    cfg: &Config,
    duration_s: f64,
    lidar_topic: &str,
    imu_topic: &str,
    mut handle: impl FnMut(SyncPackage, RawScan<'_>),
) -> Result<usize, Box<dyn std::error::Error>> {
    let mut imu_buf: Vec<ImuData> = Vec::new();
    let mut first: Option<f64> = None;
    let mut scans = 0;

    for_each_message(mapped, |message| {
        let topic = message.channel.topic.as_str();
        if topic == imu_topic {
            if let Some(sample) = decode_imu(&message.data) {
                imu_buf.push(sample);
            }
            return Ok(true);
        }
        if topic != lidar_topic {
            return Ok(true);
        }
        let scan = match decode_cloud(&message.data, cfg) {
            Some(scan) => scan,
            None => return Ok(true),
        };
        if first.is_none() {
            first = Some(scan.start);
        }
        if duration_s > 0.0 && scan.start - first.unwrap() > duration_s {
            return Ok(false);
        }

        // Point-LIO needs the IMU that brackets the scan; anything later belongs
        // to the next one and stays buffered.
        let split = imu_buf.partition_point(|sample| sample.time <= scan.end);
        let imus: Vec<ImuData> = imu_buf.drain(..split).collect();
        if imus.is_empty() {
            return Ok(true);
        }

        let cloud: Vec<Point> = scan
            .points
            .iter()
            .map(|&(x, y, z, intensity, time)| {
                Point::new(x, y, z, intensity, ((time - scan.start) * 1000.0) as f32)
            })
            .collect();
        scans += 1;
        handle(
            SyncPackage {
                imus,
                cloud,
                cloud_start_time: scan.start,
                cloud_end_time: scan.end,
            },
            RawScan { data: &message.data, log_time: message.log_time },
        );
        Ok(true)
    })?;
    Ok(scans)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds little-endian CDR the way the recorder does, so the decoders are
    /// tested against the layout they will actually meet rather than against
    /// themselves.
    struct Writer {
        out: Vec<u8>,
    }

    impl Writer {
        fn new() -> Self {
            Writer { out: vec![0x00, 0x01, 0x00, 0x00] }
        }
        fn align(&mut self, width: usize) {
            while !(self.out.len() - 4).is_multiple_of(width) {
                self.out.push(0);
            }
        }
        fn u8(&mut self, value: u8) {
            self.out.push(value);
        }
        fn u32(&mut self, value: u32) {
            self.align(4);
            self.out.extend_from_slice(&value.to_le_bytes());
        }
        fn f64(&mut self, value: f64) {
            self.align(8);
            self.out.extend_from_slice(&value.to_le_bytes());
        }
        fn string(&mut self, value: &str) {
            self.u32(value.len() as u32 + 1);
            self.out.extend_from_slice(value.as_bytes());
            self.out.push(0);
        }
        fn header(&mut self, seconds: i32, nanos: u32) {
            self.u32(seconds as u32);
            self.u32(nanos);
            self.string("frame");
        }
    }

    #[test]
    fn an_imu_message_keeps_its_units_and_its_stamp() {
        let mut writer = Writer::new();
        writer.header(100, 250_000_000);
        for value in [0.0, 0.0, 0.0, 1.0] {
            writer.f64(value);
        }
        for _ in 0..9 {
            writer.f64(0.0);
        }
        for value in [0.1, 0.2, 0.3] {
            writer.f64(value);
        }
        for _ in 0..9 {
            writer.f64(0.0);
        }
        for value in [0.0, 0.0, 9.81] {
            writer.f64(value);
        }
        for _ in 0..9 {
            writer.f64(0.0);
        }

        let imu = decode_imu(&writer.out).unwrap();
        assert!((imu.time - 100.25).abs() < 1e-9);
        assert!((imu.gyro[1] - 0.2).abs() < 1e-12);
        // Already m/s^2 on this topic, so nothing may rescale it by gravity.
        assert!((imu.acc[2] - 9.81).abs() < 1e-12);
    }

    /// The recorder's 32-byte point: xyz f32, intensity f32, tag, line, an
    /// offset_time in nanoseconds and an absolute timestamp in seconds.
    fn cloud_message(points: &[(f32, f32, f32, f32, f64)], stamp: f64) -> Vec<u8> {
        let mut writer = Writer::new();
        let seconds = stamp.floor();
        writer.header(seconds as i32, ((stamp - seconds) * 1e9).round() as u32);
        writer.u32(1);
        writer.u32(points.len() as u32);
        writer.u32(7);
        for (name, offset, datatype) in [
            ("x", 0u32, 7u8),
            ("y", 4, 7),
            ("z", 8, 7),
            ("intensity", 12, 7),
            ("tag", 16, 2),
            ("line", 17, 2),
            ("offset_time", 20, 6),
        ] {
            writer.string(name);
            writer.u32(offset);
            writer.u8(datatype);
            writer.u32(1);
        }
        writer.u8(0);
        writer.u32(32);
        writer.u32(32 * points.len() as u32);
        writer.u32(32 * points.len() as u32);
        for &(x, y, z, intensity, time) in points {
            let mut entry = [0u8; 32];
            entry[0..4].copy_from_slice(&x.to_le_bytes());
            entry[4..8].copy_from_slice(&y.to_le_bytes());
            entry[8..12].copy_from_slice(&z.to_le_bytes());
            entry[12..16].copy_from_slice(&intensity.to_le_bytes());
            let offset = ((time - stamp) * 1e9).round() as u32;
            entry[20..24].copy_from_slice(&offset.to_le_bytes());
            entry[24..32].copy_from_slice(&time.to_le_bytes());
            writer.out.extend_from_slice(&entry);
        }
        writer.u8(1);
        writer.out
    }

    #[test]
    fn a_cloud_is_read_through_its_own_field_table() {
        let stamp = 1_000.0;
        let points = [
            (1.0f32, 0.0f32, 0.0f32, 12.0f32, stamp),
            (2.0, 0.0, 0.0, 13.0, stamp + 0.005),
            (3.0, 0.0, 0.0, 14.0, stamp + 0.010),
        ];
        let mut config = Config::go2_mid360();
        config.point_filter_num = 1;
        config.blind = 0.1;

        let scan = decode_cloud(&cloud_message(&points, stamp), &config).unwrap();
        assert_eq!(scan.points.len(), 3);
        assert!((scan.start - stamp).abs() < 1e-9);
        assert!((scan.end - (stamp + 0.010)).abs() < 1e-9);
        assert!((scan.points[1].3 - 13.0).abs() < 1e-6);
    }

    #[test]
    fn returns_inside_the_blind_radius_and_past_the_range_are_dropped() {
        let stamp = 1_000.0;
        let points = [
            (0.05f32, 0.0f32, 0.0f32, 1.0f32, stamp),
            (5.0, 0.0, 0.0, 1.0, stamp + 0.001),
            (10_000.0, 0.0, 0.0, 1.0, stamp + 0.002),
        ];
        let mut config = Config::go2_mid360();
        config.point_filter_num = 1;
        config.blind = 0.5;
        config.max_range = 100.0;

        let scan = decode_cloud(&cloud_message(&points, stamp), &config).unwrap();
        assert_eq!(scan.points.len(), 1);
        assert!((scan.points[0].0 - 5.0).abs() < 1e-6);
    }
}
