//! Motion compensation for the lidar, written back as `/pointlio_lidar`.
//!
//! A Mid-360 sweeps for the whole 100 ms of a "frame" and stamps every return
//! with when it was taken, but the returns are all expressed in the lidar frame
//! as if the sensor had not moved — so on a rig somebody is carrying, a scan is
//! smeared the way a rolling shutter smears a photograph. The Livox driver does
//! not correct this and neither did anything downstream of it here.
//!
//! Point-LIO already knows the answer. Its update walks the scan group by group
//! and propagates the state to each group's time, so [`PointLio::scan_states`]
//! holds where the IMU actually was at every instant inside the scan. This
//! module takes those states and rewrites each return into where it would have
//! been seen from the scan's reference instant — its header stamp, which is the
//! time of the first return — so the corrected cloud keeps the same stamp, the
//! same frame and the same field layout as the original and drops straight into
//! anything that read the original.
//!
//! Interpolation between two states is a linear blend of position and a
//! spherical one of rotation. That is not an approximation of the estimator's
//! path over the scan so much as a reading of it: the states are dense (one per
//! time group, tens per scan), so consecutive ones are a millisecond or two
//! apart.
//!
//! What is *not* corrected: a scan the velocity cap rolled back has no usable
//! states, and a scan processed before the estimator's map finished
//! initialising has none either. Those scans are passed through unchanged and
//! counted, rather than dropped — a hole in the stream is worse than a smeared
//! frame, and the count says how many there were.

use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use crate::mcap_append::Appender;
use crate::msgs::{PointCloud2, POINT_FIELD_FLOAT64, POINT_FIELD_UINT32};
use crate::record::channel_metadata;
use crate::tf::Pose;

/// The corrected clouds' topic. Named for the estimator that produced them, the
/// way `/pointlio_odometry` is: it is not the lidar's own output and should not
/// be mistaken for it.
pub const DESKEWED_TOPIC: &str = "/pointlio_lidar";

/// How many corrected clouds a recording already carries, so a second run does
/// not append a duplicate set alongside the first.
pub fn already_present(path: &Path) -> Result<u64> {
    let file = std::fs::File::open(path)?;
    let mapped = unsafe { memmap2::Mmap::map(&file)? };
    let Some(summary) = mcap::Summary::read(&mapped)? else {
        return Ok(0);
    };
    Ok(crate::walk::message_count(&summary, DESKEWED_TOPIC))
}

/// A pose the estimator held at a known instant on the IMU clock.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub time: f64,
    pub pose: Pose,
}

/// The pose at `time`, holding the ends rather than extrapolating past them.
/// `samples` must be ascending and non-empty.
fn pose_at(samples: &[Sample], time: f64) -> Pose {
    let first = samples[0];
    if time <= first.time || samples.len() == 1 {
        return first.pose;
    }
    let last = samples[samples.len() - 1];
    if time >= last.time {
        return last.pose;
    }
    // The points come off the sensor in time order, so the previous answer is
    // nearly always the right bracket; a binary search is still cheap and does
    // not depend on that holding.
    let after = samples.partition_point(|sample| sample.time < time);
    let (before, after) = (samples[after - 1], samples[after]);
    let span = after.time - before.time;
    if span <= 0.0 {
        return before.pose;
    }
    before.pose.interpolate(&after.pose, (time - before.time) / span)
}

/// Where each point's own capture time is written in the cloud.
enum PointClock {
    /// `float64` seconds, absolute on the sensor clock.
    Absolute(usize),
    /// `uint32` nanoseconds after the cloud's header stamp.
    Offset(usize),
}

impl PointClock {
    fn find(fields: &[crate::msgs::PointField]) -> Option<PointClock> {
        let at = |name: &str, datatype: u8| {
            fields
                .iter()
                .find(|field| field.name == name && field.datatype == datatype)
                .map(|field| field.offset as usize)
        };
        // Absolute first: it needs no assumption about what the header stamp
        // means, and it is what a downsample upstream keeps correct.
        at("timestamp", POINT_FIELD_FLOAT64)
            .or_else(|| at("t", POINT_FIELD_FLOAT64))
            .map(PointClock::Absolute)
            .or_else(|| at("offset_time", POINT_FIELD_UINT32).map(PointClock::Offset))
    }

    fn read(&self, entry: &[u8], stamp_seconds: f64) -> Option<f64> {
        match *self {
            PointClock::Absolute(at) => {
                let bytes = entry.get(at..at + 8)?;
                Some(f64::from_le_bytes(bytes.try_into().ok()?))
            }
            PointClock::Offset(at) => {
                let bytes = entry.get(at..at + 4)?;
                let nanos = u32::from_le_bytes(bytes.try_into().ok()?);
                Some(stamp_seconds + nanos as f64 * 1e-9)
            }
        }
    }
}

/// Why a scan came through uncorrected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PassedThrough {
    /// The estimator had no state inside this scan (map still initialising, or
    /// the velocity cap rolled the scan back).
    NoStates,
    /// The cloud carries no per-point time, so there is nothing to correct
    /// against — every point already describes the same instant.
    NoPointClock,
}

/// A corrected cloud, or the reason there isn't one.
pub enum Deskewed {
    Corrected(Vec<u8>),
    Unchanged(PassedThrough),
}

/// Rewrites every return in `cloud` into the lidar frame as it stood at the
/// cloud's header stamp.
///
/// `states` are IMU poses in the odometry frame on the IMU clock;
/// `lidar_in_imu` places the lidar in the IMU's frame, and `clock_offset` is
/// what the estimator adds to a lidar stamp to reach that clock. Fields,
/// `point_step` and every byte that is not x/y/z are copied through, so the
/// per-point times still say when each return was really taken.
/// Returns closer than this are dropped rather than corrected. See the long note
/// in [`deskew`] for why, and for the measurements behind the value.
pub const BLIND_RANGE: f64 = 0.5;

pub fn deskew(cloud: &crate::cdr::CloudView<'_>, states: &[Sample], lidar_in_imu: &Pose, clock_offset: f64) -> Deskewed {
    if states.is_empty() {
        return Deskewed::Unchanged(PassedThrough::NoStates);
    }
    let Some(clock) = PointClock::find(&cloud.fields) else {
        return Deskewed::Unchanged(PassedThrough::NoPointClock);
    };
    let step = cloud.point_step as usize;
    if step < 12 || cloud.is_bigendian {
        return Deskewed::Unchanged(PassedThrough::NoPointClock);
    }
    let (x_at, y_at, z_at) = match axes(&cloud.fields) {
        Some(axes) => axes,
        None => return Deskewed::Unchanged(PassedThrough::NoPointClock),
    };

    let stamp_seconds = cloud.header.stamp_nanos() as f64 * 1e-9;
    // Everything is expressed relative to the lidar at the reference instant,
    // so a scan taken standing still comes out bit-identical to its input.
    let reference = pose_at(states, stamp_seconds + clock_offset).then(lidar_in_imu).inverse();

    let mut data = cloud.data.to_vec();
    for entry in data.chunks_exact_mut(step) {
        let read = |at: usize| f32::from_le_bytes(entry[at..at + 4].try_into().unwrap()) as f64;
        let point = [read(x_at), read(y_at), read(z_at)];
        // Blind zone, applied BEFORE the correction and in place.
        //
        // Two different things sit under half a metre and neither belongs in a
        // published cloud. Most of it is the driver's invalid returns, which the
        // Mid-360 reports as exactly (0, 0, 0) -- roughly half of every scan. Run
        // those through motion compensation and they stop being zero: they land
        // wherever the sensor travelled during the sweep, a shell of phantom
        // points centimetres to decimetres out whose radius grows with speed. A
        // consumer testing for zero then keeps all of them. Measured on park.mcap
        // index for index, 100% of the driver's zero slots came out inside 0.5 m.
        // The rest is the rig returning its own structure, ~3000 points a scan
        // holding their offset from the sensor to within 5 cm over 200 s of travel.
        //
        // 0.5 m is hku-mars' own `blind` default for this sensor in FAST-LIO's
        // mid360.yaml, and Point-LIO here already sets it -- but in the preprocess
        // block, so it gates what feeds the state estimator and never reached what
        // we publish. The cut costs nothing: the range histogram is bimodal, with
        // 76,883 points in 0.20-0.50 m, then 314 (0.1%) in the whole of 0.50-1.00 m,
        // then real structure. Anywhere in that valley gives the same cloud.
        //
        // Zeroed rather than removed, so the point count, the per-point times and
        // the slot correspondence with `livox_lidar` all survive -- and so that a
        // discarded return keeps saying the one thing the driver already says
        // about it.
        //
        // Worth knowing: this is a correctness fix for anything counting points,
        // not a map improvement. Rebuilding a voxel map with and without it moved
        // 296 of 36,152 voxels, 0.8% -- a hundred thousand points packed inside a
        // half-metre sphere collapse into a handful of cells that the real ground
        // return under the sensor already occupies.
        if point[0] * point[0] + point[1] * point[1] + point[2] * point[2] < BLIND_RANGE * BLIND_RANGE
        {
            for at in [x_at, y_at, z_at] {
                entry[at..at + 4].copy_from_slice(&0f32.to_le_bytes());
            }
            continue;
        }
        let Some(time) = clock.read(entry, stamp_seconds) else {
            continue;
        };
        // lidar-at-capture -> odom -> lidar-at-reference, in one composition.
        let moved = reference.then(&pose_at(states, time + clock_offset).then(lidar_in_imu)).apply(point);
        for (at, value) in [(x_at, moved[0]), (y_at, moved[1]), (z_at, moved[2])] {
            entry[at..at + 4].copy_from_slice(&(value as f32).to_le_bytes());
        }
    }

    let corrected = PointCloud2 {
        header: cloud.header.clone(),
        height: cloud.height,
        width: cloud.width,
        fields: cloud.fields.clone(),
        is_bigendian: false,
        point_step: cloud.point_step,
        row_step: cloud.point_step * cloud.width,
        data,
        is_dense: true,
    };
    Deskewed::Corrected(crate::cdr::point_cloud2(&corrected).data)
}

fn axes(fields: &[crate::msgs::PointField]) -> Option<(usize, usize, usize)> {
    let at = |name: &str| {
        fields
            .iter()
            .find(|field| field.name == name && field.datatype == crate::msgs::POINT_FIELD_FLOAT32)
            .map(|field| field.offset as usize)
    };
    Some((at("x")?, at("y")?, at("z")?))
}

/// Corrected clouds held on disk between the estimator's pass and the append.
///
/// They cannot go straight into the file: the estimator reads the recording
/// through a memory map, and [`Appender::open`] cuts the summary off the end of
/// that same file. So the clouds are spooled next to the recording and copied
/// in afterwards. The estimator's pass over a 58 GB recording takes half an
/// hour; walking it a second time to avoid a spool would cost another one, and
/// the spool is a few GB written once and read once.
pub struct Spool {
    path: PathBuf,
    file: BufWriter<std::fs::File>,
    clouds: u64,
    bytes: u64,
    passed_through: u64,
}

impl Spool {
    /// Alongside the recording rather than in a temp directory: it is sized
    /// like the recording's lidar stream, and the disk with room for one is the
    /// disk with room for the other.
    pub fn beside(recording: &Path) -> Result<Spool> {
        let mut name = recording.as_os_str().to_os_string();
        name.push(".deskew-spool");
        let path = PathBuf::from(name);
        let file = std::fs::File::create(&path)
            .with_context(|| format!("could not open the deskew spool {}", path.display()))?;
        Ok(Spool {
            path,
            file: BufWriter::with_capacity(1 << 20, file),
            clouds: 0,
            bytes: 0,
            passed_through: 0,
        })
    }

    /// A spool for something other than corrected scans, named by `suffix`
    /// next to the recording. Same format, same reasons.
    pub fn beside_named(recording: &Path, suffix: &str) -> Result<Spool> {
        let mut name = recording.as_os_str().to_os_string();
        name.push(suffix);
        let path = PathBuf::from(name);
        let file = std::fs::File::create(&path)
            .with_context(|| format!("could not open the spool {}", path.display()))?;
        Ok(Spool {
            path,
            file: BufWriter::with_capacity(1 << 20, file),
            clouds: 0,
            bytes: 0,
            passed_through: 0,
        })
    }

    pub fn push(&mut self, log_time: u64, cloud: &Deskewed) -> Result<()> {
        match cloud {
            Deskewed::Corrected(data) => self.push_bytes(log_time, data),
            Deskewed::Unchanged(_) => {
                self.passed_through += 1;
                Ok(())
            }
        }
    }

    /// One already-encoded message, stamped `log_time`.
    pub fn push_bytes(&mut self, log_time: u64, data: &[u8]) -> Result<()> {
        self.file.write_all(&log_time.to_le_bytes())?;
        self.file.write_all(&(data.len() as u32).to_le_bytes())?;
        self.file.write_all(data)?;
        self.clouds += 1;
        self.bytes += data.len() as u64;
        Ok(())
    }

    pub fn clouds(&self) -> u64 {
        self.clouds
    }

    pub fn bytes(&self) -> u64 {
        self.bytes
    }

    /// Scans the estimator could not correct, which were left out.
    pub fn passed_through(&self) -> u64 {
        self.passed_through
    }

    /// Copies the spool into the recording and removes it. `frame` is the
    /// lidar's frame, recorded on the channel so a reader knows the clouds are
    /// placed by the same tf edge as the originals.
    pub fn drain_into(self, appender: &mut Appender, frame: &str, gauge: Option<&crate::progress::Gauge>) -> Result<u64> {
        let sample = crate::cdr::point_cloud2(&PointCloud2 {
            header: crate::msgs::Header::new(0, frame),
            height: 1,
            width: 0,
            fields: Vec::new(),
            is_bigendian: false,
            point_step: 0,
            row_step: 0,
            data: Vec::new(),
            is_dense: true,
        });
        let schema = appender.schema(sample.schema_name, "ros2msg", sample.schema_text.as_bytes());
        let channel = appender.channel(DESKEWED_TOPIC, schema, "cdr", &channel_metadata(DESKEWED_TOPIC));
        self.drain(|log_time, data| {
            if let Some(gauge) = gauge {
                gauge.at(log_time);
            }
            appender.write_stream(channel, log_time, data)
        })
    }

    /// Reads the spool back in the order it was written, one message at a
    /// time, handing each to `sink`, then removes it -- on failure too, since
    /// a spool is worthless once its pass is over. Streamed, never held: the
    /// spool exists because its contents do not fit in memory.
    pub fn drain(self, mut sink: impl FnMut(u64, Vec<u8>) -> Result<()>) -> Result<u64> {
        let Spool { path, mut file, clouds, .. } = self;
        file.flush()?;
        drop(file);
        let result = (|| -> Result<u64> {
            let mut reader = BufReader::with_capacity(1 << 20, std::fs::File::open(&path)?);
            let mut written = 0;
            for _ in 0..clouds {
                let mut stamp = [0u8; 8];
                reader.read_exact(&mut stamp)?;
                let mut length = [0u8; 4];
                reader.read_exact(&mut length)?;
                let mut data = vec![0u8; u32::from_le_bytes(length) as usize];
                reader.read_exact(&mut data)?;
                sink(u64::from_le_bytes(stamp), data)?;
                written += 1;
            }
            Ok(written)
        })();
        let _ = std::fs::remove_file(&path);
        result
    }

    /// Throws the spool away without appending it.
    pub fn discard(self) {
        let Spool { path, file, .. } = self;
        drop(file);
        let _ = std::fs::remove_file(&path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{Header, PointField, POINT_FIELD_FLOAT32};

    fn fields() -> Vec<PointField> {
        vec![
            PointField { name: "x".into(), offset: 0, datatype: POINT_FIELD_FLOAT32, count: 1 },
            PointField { name: "y".into(), offset: 4, datatype: POINT_FIELD_FLOAT32, count: 1 },
            PointField { name: "z".into(), offset: 8, datatype: POINT_FIELD_FLOAT32, count: 1 },
            PointField { name: "offset_time".into(), offset: 12, datatype: POINT_FIELD_UINT32, count: 1 },
        ]
    }

    /// One point per millisecond across 100 ms, all at the same place in the
    /// lidar frame — so any real motion smears them apart.
    fn scan(stamp_nanos: u64, point: [f32; 3], count: u32) -> Vec<u8> {
        let mut data = Vec::new();
        for index in 0..count {
            for axis in point {
                data.extend_from_slice(&axis.to_le_bytes());
            }
            data.extend_from_slice(&(index * 1_000_000).to_le_bytes());
        }
        crate::cdr::point_cloud2(&PointCloud2 {
            header: Header::new(stamp_nanos, "livox_frame"),
            height: 1,
            width: count,
            fields: fields(),
            is_bigendian: false,
            point_step: 16,
            row_step: 16 * count,
            data,
            is_dense: true,
        })
        .data
    }

    fn points_of(encoded: &[u8]) -> Vec<[f64; 3]> {
        let view = crate::cdr::decode_point_cloud2(encoded).unwrap();
        view.data
            .chunks_exact(view.point_step as usize)
            .map(|entry| {
                std::array::from_fn(|axis| {
                    f32::from_le_bytes(entry[axis * 4..axis * 4 + 4].try_into().unwrap()) as f64
                })
            })
            .collect()
    }

    fn corrected(encoded: &[u8], states: &[Sample]) -> Vec<[f64; 3]> {
        let view = crate::cdr::decode_point_cloud2(encoded).unwrap();
        match deskew(&view, states, &Pose::IDENTITY, 0.0) {
            Deskewed::Corrected(data) => points_of(&data),
            Deskewed::Unchanged(reason) => panic!("not corrected: {reason:?}"),
        }
    }

    #[test]
    fn an_invalid_return_stays_at_the_origin_instead_of_being_flung_into_a_shell() {
        // The bug this filter exists for. The driver reports a non-return as exactly
        // (0, 0, 0) -- about half of every Mid-360 scan. Deskewing one moves it to
        // wherever the sensor travelled during the sweep, so it stops being zero and
        // every consumer testing for zero then keeps it. Measured on a real recording,
        // index for index, 100% of the driver's zero slots came out inside 0.5 m.
        let encoded = scan(0, [0.0, 0.0, 0.0], 8);
        let states = [
            Sample { time: 0.0, pose: Pose::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 0.1, pose: Pose::new([3.0, 4.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
        ];
        for point in corrected(&encoded, &states) {
            assert_eq!(point, [0.0, 0.0, 0.0], "an invalid return was displaced: {point:?}");
        }
    }

    #[test]
    fn a_return_inside_the_blind_radius_is_discarded_rather_than_corrected() {
        // The rig returning its own structure -- a real observation, at a fixed offset
        // from the sensor, that does not belong in a published cloud. Same disposal as
        // an invalid return so that a discarded point keeps saying what the driver
        // already says about one.
        let near = (BLIND_RANGE as f32) * 0.5;
        let encoded = scan(0, [near, 0.0, 0.0], 8);
        let states = [
            Sample { time: 0.0, pose: Pose::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 0.1, pose: Pose::new([3.0, 4.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
        ];
        for point in corrected(&encoded, &states) {
            assert_eq!(point, [0.0, 0.0, 0.0], "a blind-zone return survived: {point:?}");
        }
    }

    #[test]
    fn a_return_just_outside_the_blind_radius_is_kept_and_corrected() {
        // The other side of the cut. The histogram is bimodal with an almost empty
        // 0.5-1.0 m valley, so the threshold must not be eating real structure.
        let far = (BLIND_RANGE as f32) * 1.2;
        let encoded = scan(0, [far, 0.0, 0.0], 8);
        let states = [
            Sample { time: 0.0, pose: Pose::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 0.1, pose: Pose::new([3.0, 4.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
        ];
        let points = corrected(&encoded, &states);
        assert!(points.iter().any(|p| *p != [0.0, 0.0, 0.0]), "everything was discarded");
        // A moving sensor must actually spread them, i.e. they went through the correction.
        assert!(
            points.iter().any(|p| (p[0] - far as f64).abs() > 1e-3),
            "kept but not corrected: {points:?}"
        );
    }

    #[test]
    fn a_stationary_scan_comes_back_untouched() {
        let encoded = scan(0, [1.0, 2.0, 3.0], 8);
        let states = [
            Sample { time: 0.0, pose: Pose::new([5.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 0.1, pose: Pose::new([5.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
        ];
        for point in corrected(&encoded, &states) {
            for (axis, expected) in point.iter().zip([1.0, 2.0, 3.0]) {
                assert!((axis - expected).abs() < 1e-5, "{point:?}");
            }
        }
    }

    /// The whole point of the exercise: the sensor slides 1 m along x over the
    /// scan while every return says the same thing, so the returns really were
    /// a metre apart in the world and must come out a metre apart.
    #[test]
    fn a_sensor_moving_through_the_scan_spreads_its_returns() {
        let encoded = scan(0, [10.0, 0.0, 0.0], 101);
        let states = [
            Sample { time: 0.0, pose: Pose::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 0.1, pose: Pose::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
        ];
        let points = corrected(&encoded, &states);
        assert!((points[0][0] - 10.0).abs() < 1e-4, "the first return is the reference: {:?}", points[0]);
        assert!((points[100][0] - 11.0).abs() < 1e-4, "the last is a metre further out: {:?}", points[100]);
        // and evenly, since the motion was
        assert!((points[50][0] - 10.5).abs() < 1e-4, "{:?}", points[50]);
    }

    /// A quarter turn over the scan: a return straight ahead at the end of the
    /// scan was really off to the side when the scan began.
    #[test]
    fn a_turn_through_the_scan_swings_its_returns_across() {
        let encoded = scan(0, [10.0, 0.0, 0.0], 101);
        let quarter = (std::f64::consts::FRAC_PI_4).sin();
        let states = [
            Sample { time: 0.0, pose: Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 0.1, pose: Pose::new([0.0; 3], [0.0, 0.0, quarter, quarter]) },
        ];
        let points = corrected(&encoded, &states);
        assert!((points[0][0] - 10.0).abs() < 1e-4 && points[0][1].abs() < 1e-4, "{:?}", points[0]);
        assert!(points[100][0].abs() < 1e-3 && (points[100][1] - 10.0).abs() < 1e-3, "{:?}", points[100]);
    }

    /// The lidar is not the IMU, and correcting as though it were leaves the
    /// lever arm's contribution in.
    #[test]
    fn the_lever_arm_between_lidar_and_imu_is_taken_out() {
        let encoded = scan(0, [10.0, 0.0, 0.0], 101);
        let quarter = (std::f64::consts::FRAC_PI_4).sin();
        let states = [
            Sample { time: 0.0, pose: Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 0.1, pose: Pose::new([0.0; 3], [0.0, 0.0, quarter, quarter]) },
        ];
        let lidar_in_imu = Pose::new([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        let view = crate::cdr::decode_point_cloud2(&encoded).unwrap();
        let Deskewed::Corrected(data) = deskew(&view, &states, &lidar_in_imu, 0.0) else {
            panic!("not corrected");
        };
        let points = points_of(&data);
        // The IMU turned in place, so the lidar swung through an arc of radius
        // 0.5 m: the last return lands 0.5 m short of where an IMU-centred
        // correction would put it, and 0.5 m off to the side.
        assert!((points[100][0] + 0.5).abs() < 1e-3, "{:?}", points[100]);
        assert!((points[100][1] - 10.5).abs() < 1e-3, "{:?}", points[100]);
    }

    #[test]
    fn everything_that_is_not_a_coordinate_is_copied_through() {
        let encoded = scan(1_000_000_000, [1.0, 0.0, 0.0], 4);
        let states = [Sample { time: 1.0, pose: Pose::IDENTITY }, Sample { time: 1.1, pose: Pose::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) }];
        let view = crate::cdr::decode_point_cloud2(&encoded).unwrap();
        let Deskewed::Corrected(data) = deskew(&view, &states, &Pose::IDENTITY, 0.0) else {
            panic!("not corrected");
        };
        let after = crate::cdr::decode_point_cloud2(&data).unwrap();
        assert_eq!(after.header, view.header);
        assert_eq!(after.fields, view.fields);
        assert_eq!(after.point_step, view.point_step);
        assert_eq!(after.width, view.width);
        for (before, now) in view.data.as_chunks::<16>().0.iter().zip(after.data.as_chunks::<16>().0.iter()) {
            assert_eq!(&before[12..16], &now[12..16], "the per-point time was rewritten");
        }
    }

    #[test]
    fn a_scan_the_estimator_could_not_place_is_left_alone() {
        let encoded = scan(0, [1.0, 0.0, 0.0], 4);
        let view = crate::cdr::decode_point_cloud2(&encoded).unwrap();
        assert!(matches!(
            deskew(&view, &[], &Pose::IDENTITY, 0.0),
            Deskewed::Unchanged(PassedThrough::NoStates)
        ));
    }

    #[test]
    fn a_cloud_with_no_per_point_time_is_left_alone() {
        let mut without = fields();
        without.pop();
        let encoded = crate::cdr::point_cloud2(&PointCloud2 {
            header: Header::new(0, "livox_frame"),
            height: 1,
            width: 1,
            fields: without,
            is_bigendian: false,
            point_step: 16,
            row_step: 16,
            data: vec![0u8; 16],
            is_dense: true,
        })
        .data;
        let view = crate::cdr::decode_point_cloud2(&encoded).unwrap();
        let states = [Sample { time: 0.0, pose: Pose::IDENTITY }];
        assert!(matches!(
            deskew(&view, &states, &Pose::IDENTITY, 0.0),
            Deskewed::Unchanged(PassedThrough::NoPointClock)
        ));
    }

    #[test]
    fn a_time_outside_the_states_holds_the_nearest_one_rather_than_flying_off() {
        let states = [
            Sample { time: 1.0, pose: Pose::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
            Sample { time: 2.0, pose: Pose::new([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]) },
        ];
        assert_eq!(pose_at(&states, -50.0).translation, [1.0, 0.0, 0.0]);
        assert_eq!(pose_at(&states, 50.0).translation, [2.0, 0.0, 0.0]);
        assert!((pose_at(&states, 1.25).translation[0] - 1.25).abs() < 1e-12);
    }
}
