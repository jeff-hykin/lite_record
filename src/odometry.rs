//! Lidar-inertial odometry for a recording that has none.
//!
//! Runs Point-LIO (the pure-Rust port under `vendor/`) over the recording's
//! lidar and IMU streams, then appends the trajectory as `/pointlio_odometry`
//! and as `odom -> <root>` edges on `/tf`, where `<root>` is the top of the
//! static tree the lidar hangs from — `base_link` once a URDF has been applied,
//! the lidar's own link otherwise. The odometry is therefore the pose of the
//! rig, not of the lidar, and composes with the static edges to place every
//! sensor.
//!
//! Point-LIO estimates on the sensors' own clock, which in these recordings can
//! sit a long way from the clock the messages were logged on (the Pi's clock
//! stepped after the lidar's offset was taken). The appended messages are
//! stamped on the log clock, like the camera streams and like the tf that
//! places them, so they line up on a player's timeline; a consumer matching
//! them to the lidar's own header stamps has to add the offset reported here.

use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use anyhow::{bail, Context, Result};
use pointlio_rs::{mcap_input, Config, PointLio, PoseSample};

use crate::mcap_append::Appender;
use crate::msgs::{Header, Odometry, NANOS_PER_SEC, ODOMETRY_TYPE};
use crate::record::{channel_metadata, TF_TOPIC};
use crate::tf::{Pose, StaticTree};

pub const ODOMETRY_TOPIC: &str = "/pointlio_odometry";
pub const ODOM_FRAME: &str = "odom";
/// Channel metadata naming the geometry the odometry was computed under: the
/// frame it describes, and where the lidar sat in that frame. A later run whose
/// URDF disagrees is reading poses estimated for a different rig.
pub const GEOMETRY_KEY: &str = "lite_record.odometry_geometry";
/// Metres per second, twice a brisk walk.
pub const HANDHELD_MAX_VELOCITY: f64 = 3.0;

/// `<root frame> <x> <y> <z> <qx> <qy> <qz> <qw>`, the lidar's pose in the frame
/// the odometry describes.
fn geometry_marker(root: &str, lidar_in_root: &Pose) -> String {
    let [x, y, z] = lidar_in_root.translation;
    let [qx, qy, qz, qw] = lidar_in_root.rotation;
    format!("{root} {x:.6} {y:.6} {z:.6} {qx:.6} {qy:.6} {qz:.6} {qw:.6}")
}

/// How far the recorded geometry is from `tree`'s, as metres and radians.
/// `None` when the file carries no marker, which is every file written before
/// this existed.
pub fn geometry_drift(marker: &str, tree: &StaticTree, lidar_frame: &str) -> Option<(f64, f64)> {
    let mut parts = marker.split_whitespace();
    let root = parts.next()?;
    let numbers: Vec<f64> = parts.filter_map(|part| part.parse().ok()).collect();
    let recorded = Pose::new(
        [*numbers.first()?, *numbers.get(1)?, *numbers.get(2)?],
        [*numbers.get(3)?, *numbers.get(4)?, *numbers.get(5)?, *numbers.get(6)?],
    );
    let now = tree.pose_in(root, lidar_frame)?;
    let metres = (0..3)
        .map(|axis| (now.translation[axis] - recorded.translation[axis]).abs())
        .fold(0.0, f64::max);
    // The angle of the rotation that takes one orientation to the other.
    let dot: f64 = (0..4).map(|index| now.rotation[index] * recorded.rotation[index]).sum();
    let radians = 2.0 * dot.abs().clamp(0.0, 1.0).acos();
    Some((metres, radians))
}

/// The estimator's settings for a rig somebody carries.
///
/// The one departure from the Mid-360 defaults is the velocity cap, and it
/// matters more than it looks. A scan the filter cannot match sends the state
/// off at metres per second and every pose after it is lost; the cap rolls that
/// scan back and lets the next one try again. Point-LIO ships it disabled.
/// Measured on the grocery recording: uncapped, x86 and aarch64 agree to under a
/// micron for 648 s and then split at a single decision, the aarch64 run ending
/// 240 m away with a 537 m path against the x86 run's 307 m. Capped, they agree
/// — 304.9 m, one scan rolled back, final pose within 2 cm.
pub fn handheld_config() -> Config {
    Config {
        max_velocity: HANDHELD_MAX_VELOCITY,
        ..Config::go2_mid360()
    }
}

/// The lidar and IMU topics to estimate from: the first PointCloud2 channel
/// and the Imu channel that shares its prefix.
pub fn find_lidar_and_imu(summary: &mcap::Summary) -> Option<(String, String)> {
    let mut clouds: Vec<&str> = summary
        .channels
        .values()
        .filter(|channel| channel.schema.as_ref().is_some_and(|schema| schema.name == crate::msgs::POINT_CLOUD2_TYPE))
        .map(|channel| channel.topic.as_str())
        .collect();
    clouds.sort_unstable();
    let imus: Vec<&str> = summary
        .channels
        .values()
        .filter(|channel| channel.schema.as_ref().is_some_and(|schema| schema.name == crate::msgs::IMU_TYPE))
        .map(|channel| channel.topic.as_str())
        .collect();
    for cloud in clouds {
        let prefix = cloud.rsplit_once('/').map_or("", |(prefix, _)| prefix);
        if let Some(imu) = imus.iter().find(|imu| imu.rsplit_once('/').map_or("", |(imu_prefix, _)| imu_prefix) == prefix) {
            return Some((cloud.to_string(), imu.to_string()));
        }
    }
    None
}

pub struct Estimate {
    pub poses: Vec<PoseSample>,
    /// How far the log clock runs ahead of the lidar's header stamps, seconds.
    pub log_offset_seconds: f64,
    /// The frame the lidar's clouds are stamped in.
    pub lidar_frame: String,
    pub path_length_metres: f64,
    /// Scans the velocity cap rolled back. A handful is the guard doing its job;
    /// a large share means the estimate is not to be trusted.
    pub rejected_scans: usize,
    /// Lidar-in-IMU, from the estimator's configuration: the estimator tracks
    /// the IMU, the tree hangs off the lidar.
    lidar_in_imu: Pose,
}

impl Estimate {
    /// The lidar's pose in `odom` at `sample`.
    pub fn lidar_pose(&self, sample: &PoseSample) -> Pose {
        let rotation: [f64; 9] = std::array::from_fn(|index| sample.rot[(index / 3, index % 3)]);
        Pose::from_matrix(rotation, [sample.pos[0], sample.pos[1], sample.pos[2]]).then(&self.lidar_in_imu)
    }

    pub fn log_stamp_nanos(&self, sample: &PoseSample) -> u64 {
        ((sample.time + self.log_offset_seconds) * NANOS_PER_SEC as f64).round() as u64
    }
}

/// Runs the estimator over `mapped`. `scans` counts processed scans as it goes,
/// for a progress line. Takes tens of minutes on an hour of video, because the
/// walk decompresses every chunk to find the lidar's messages.
pub fn estimate(
    mapped: &[u8],
    lidar_topic: &str,
    imu_topic: &str,
    scans: &Arc<AtomicU64>,
) -> Result<Estimate> {
    let config = handheld_config();
    let lidar_frame = first_frame(mapped, lidar_topic)?
        .with_context(|| format!("no decodable message on {lidar_topic}"))?;

    let mut lio = PointLio::new(config.clone());
    mcap_input::for_each_package(mapped, &config, 0.0, lidar_topic, imu_topic, |package| {
        lio.process(&package);
        scans.fetch_add(1, Ordering::Relaxed);
    })
    .map_err(|error| anyhow::anyhow!("{error}"))?;
    if lio.trajectory.is_empty() {
        bail!("no poses came out — is {lidar_topic} the lidar and {imu_topic} its imu?");
    }
    let log_offset_seconds = mcap_input::log_time_offset(mapped, &config, lidar_topic)
        .map_err(|error| anyhow::anyhow!("{error}"))?
        .unwrap_or(0.0);
    let path_length_metres =
        pointlio_rs::metrics::path_length(&pointlio_rs::trajectory::samples_to_traj(&lio.trajectory));
    let rotation: [f64; 9] = std::array::from_fn(|index| config.lidar_to_imu_rot[(index / 3, index % 3)]);
    let translation = [config.lidar_to_imu_trans[0], config.lidar_to_imu_trans[1], config.lidar_to_imu_trans[2]];
    Ok(Estimate {
        rejected_scans: lio.rejected_scans,
        poses: lio.trajectory,
        log_offset_seconds,
        lidar_frame,
        path_length_metres,
        lidar_in_imu: Pose::from_matrix(rotation, translation),
    })
}

fn first_frame(mapped: &[u8], topic: &str) -> Result<Option<String>> {
    let summary = mcap::Summary::read(mapped)?.context("no summary")?;
    let Some(channel) = summary.channels.values().find(|channel| channel.topic == topic) else {
        return Ok(None);
    };
    let mut chunks = summary.chunk_indexes.clone();
    chunks.sort_by_key(|chunk| chunk.chunk_start_offset);
    for chunk in chunks.iter().filter(|chunk| chunk.message_index_offsets.contains_key(&channel.id)) {
        for message in summary.stream_chunk(mapped, chunk)? {
            let message = message?;
            if message.channel.id == channel.id {
                if let Some(header) = crate::cdr::decode_header(&message.data) {
                    return Ok(Some(header.frame_id));
                }
            }
        }
    }
    Ok(None)
}

/// What `append` wrote.
pub struct Appended {
    pub odometry_messages: u64,
    pub tf_messages: u64,
    /// The frame the odometry describes: the root above the lidar.
    pub child_frame: String,
}

/// Appends the trajectory as odometry of the tree's root and as tf edges.
pub fn append(appender: &mut Appender, estimate: &Estimate, tree: &StaticTree) -> Result<Appended> {
    let (child_frame, lidar_in_root) = match tree.root_of(&estimate.lidar_frame) {
        Some(root) if root != estimate.lidar_frame => {
            let pose = tree
                .pose_in(&root, &estimate.lidar_frame)
                .context("the lidar frame is under a root it cannot be composed to")?;
            (root, pose)
        }
        _ => (estimate.lidar_frame.clone(), Pose::IDENTITY),
    };
    let root_in_lidar = lidar_in_root.inverse();

    let sample_encoded = crate::cdr::odometry(&Odometry {
        header: Header::new(0, ODOM_FRAME),
        child_frame_id: child_frame.clone(),
        position: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
        linear_velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
    });
    let odometry_schema = appender.schema(ODOMETRY_TYPE, "ros2msg", sample_encoded.schema_text.as_bytes());
    let mut metadata = channel_metadata(ODOMETRY_TOPIC);
    metadata.insert(GEOMETRY_KEY.to_string(), geometry_marker(&child_frame, &lidar_in_root));
    let odometry_channel = appender.channel(ODOMETRY_TOPIC, odometry_schema, "cdr", &metadata);
    let tf_encoded = crate::cdr::tf_message(&[]);
    let tf_schema = appender.schema(tf_encoded.schema_name, "ros2msg", tf_encoded.schema_text.as_bytes());
    let tf_channel = appender.channel(TF_TOPIC, tf_schema, "cdr", &channel_metadata(TF_TOPIC));

    let mut odometry_messages = 0;
    for sample in &estimate.poses {
        let stamp = estimate.log_stamp_nanos(sample);
        let root_in_odom = estimate.lidar_pose(sample).then(&root_in_lidar);
        // The estimator's velocity is in the world frame; a Twist is in the
        // child frame, which is what anything integrating it assumes.
        let world_velocity = [sample.vel[0], sample.vel[1], sample.vel[2]];
        let body_velocity = Pose::new([0.0; 3], root_in_odom.rotation).inverse().apply(world_velocity);
        let odometry = Odometry {
            header: Header::new(stamp, ODOM_FRAME),
            child_frame_id: child_frame.clone(),
            position: root_in_odom.translation,
            orientation: root_in_odom.rotation,
            linear_velocity: body_velocity,
            angular_velocity: [0.0; 3],
        };
        appender.write(odometry_channel, stamp, crate::cdr::odometry(&odometry).data)?;
        let edge = root_in_odom.stamped(stamp, ODOM_FRAME, &child_frame);
        appender.write(tf_channel, stamp, crate::cdr::tf_message(&[edge]).data)?;
        odometry_messages += 1;
    }
    Ok(Appended {
        odometry_messages,
        tf_messages: odometry_messages,
        child_frame,
    })
}

/// The IMU trajectory as TUM lines, on the sensor clock, the way
/// `pointlio_rs`'s own tools write it.
pub fn write_tum(estimate: &Estimate, path: &Path) -> Result<()> {
    use std::io::Write;
    let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
    for sample in &estimate.poses {
        let rotation: [f64; 9] = std::array::from_fn(|index| sample.rot[(index / 3, index % 3)]);
        let [qx, qy, qz, qw] = crate::msgs::quaternion_from_matrix(rotation);
        writeln!(
            out,
            "{:.6} {:.6} {:.6} {:.6} {:.7} {:.7} {:.7} {:.7}",
            sample.time, sample.pos[0], sample.pos[1], sample.pos[2], qx, qy, qz, qw
        )?;
    }
    Ok(())
}

/// The odometry a recording already carries: how many messages, and the rig
/// geometry it was computed under. `None` when there is none, so a second run
/// does not double it.
pub struct Existing {
    pub messages: u64,
    pub geometry: Option<String>,
}

pub fn already_present(path: &Path) -> Result<Option<Existing>> {
    let file = std::fs::File::open(path)?;
    let mapped = unsafe { memmap2::Mmap::map(&file)? };
    let summary = mcap::Summary::read(&mapped)?.context("no summary")?;
    let Some(channel) = summary.channels.values().find(|channel| channel.topic == ODOMETRY_TOPIC) else {
        return Ok(None);
    };
    Ok(Some(Existing {
        messages: summary
            .stats
            .as_ref()
            .and_then(|stats| stats.channel_message_counts.get(&channel.id).copied())
            .unwrap_or(0),
        geometry: channel.metadata.get(GEOMETRY_KEY).cloned(),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pointlio_rs::types::{M3D, V3D};

    fn sample(time: f64, position: [f64; 3], yaw: f64) -> PoseSample {
        let (sin, cos) = yaw.sin_cos();
        PoseSample {
            time,
            pos: V3D::new(position[0], position[1], position[2]),
            rot: M3D::new(cos, -sin, 0.0, sin, cos, 0.0, 0.0, 0.0, 1.0),
            vel: V3D::new(1.0, 0.0, 0.0),
        }
    }

    fn estimate_with(poses: Vec<PoseSample>) -> Estimate {
        Estimate {
            poses,
            log_offset_seconds: 2005.0,
            lidar_frame: "livox_frame".into(),
            path_length_metres: 0.0,
            rejected_scans: 0,
            lidar_in_imu: Pose::new([-0.011, -0.02329, 0.04412], [0.0, 0.0, 0.0, 1.0]),
        }
    }

    #[test]
    fn the_odometry_describes_the_tree_root_and_is_stamped_on_the_log_clock() {
        let path = std::env::temp_dir().join(format!("lite_record_odom_{}.mcap", crate::record::now_nanos()));
        {
            let file = std::fs::File::create(&path).unwrap();
            let mut writer = mcap::WriteOptions::new().profile("ros2").create(std::io::BufWriter::new(file)).unwrap();
            let schema = writer.add_schema("x", "ros2msg", b"y").unwrap();
            let channel = writer.add_channel(schema, "/x", "cdr", &Default::default()).unwrap();
            writer
                .write_to_known_channel(
                    &mcap::records::MessageHeader { channel_id: channel, sequence: 0, log_time: 1, publish_time: 1 },
                    &[0],
                )
                .unwrap();
            writer.finish().unwrap();
        }
        let mut tree = StaticTree::new();
        tree.insert("base_link", "livox_link", Pose::new([0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]));
        tree.insert("livox_link", "livox_frame", Pose::IDENTITY);
        let estimate = estimate_with(vec![
            sample(10.0, [0.0, 0.0, 0.0], 0.0),
            sample(10.1, [1.0, 0.0, 0.0], std::f64::consts::FRAC_PI_2),
        ]);

        let mut appender = Appender::open(&path).unwrap();
        let appended = append(&mut appender, &estimate, &tree).unwrap();
        appender.finish().unwrap();
        assert_eq!(appended.child_frame, "base_link");
        assert_eq!(appended.odometry_messages, 2);
        let recorded = already_present(&path).unwrap().unwrap();
        assert_eq!(recorded.messages, 2);
        let (metres, radians) = geometry_drift(recorded.geometry.as_deref().unwrap(), &tree, "livox_frame").unwrap();
        assert!(metres < 1e-6 && radians < 1e-6, "{metres} {radians}");

        let bytes = std::fs::read(&path).unwrap();
        let messages: Vec<_> = mcap::MessageStream::new(&bytes).unwrap().map(Result::unwrap).collect();
        let odometry: Vec<Odometry> = messages
            .iter()
            .filter(|message| message.channel.topic == ODOMETRY_TOPIC)
            .map(|message| crate::cdr::decode_odometry_message(&message.data).unwrap())
            .collect();
        assert_eq!(odometry.len(), 2);
        assert_eq!(odometry[0].header.frame_id, "odom");
        assert_eq!(odometry[0].child_frame_id, "base_link");
        assert_eq!(odometry[0].header.stamp_nanos(), 2_015_000_000_000);
        // The lidar is 0.5 m above base_link and sits 4.4 cm above its own IMU,
        // so with the IMU at the origin the base is 0.5 - 0.044 below it.
        let base_z = odometry[0].position[2];
        assert!((base_z - (-0.5 + 0.04412)).abs() < 1e-9, "{base_z}");
        // Turned 90 degrees, the lidar's +x offset from base swings to +y.
        assert!((odometry[1].orientation[2] - std::f64::consts::FRAC_PI_4.sin()).abs() < 1e-9);
        // The estimator's world-frame velocity (1,0,0) seen from a body yawed
        // 90 degrees is along -y.
        assert!((odometry[1].linear_velocity[1] + 1.0).abs() < 1e-9, "{:?}", odometry[1].linear_velocity);
        let tf_messages = messages.iter().filter(|message| message.channel.topic == TF_TOPIC).count();
        assert_eq!(tf_messages, 2);
        std::fs::remove_file(&path).ok();
    }

    /// The cap is the whole reason two machines agree on this data, so it is
    /// pinned rather than left to a config default that ships disabled.
    #[test]
    fn the_handheld_estimator_caps_its_velocity_where_the_stock_config_does_not() {
        assert_eq!(Config::go2_mid360().max_velocity, 0.0, "upstream still ships the guard off");
        let config = handheld_config();
        assert_eq!(config.max_velocity, HANDHELD_MAX_VELOCITY);
        assert!(config.max_velocity > 2.0, "a brisk walk must not be rejected");
        assert!(config.max_velocity < 10.0, "a runaway scan must be");
        // Everything else is still the Mid-360 tuning.
        assert_eq!(config.filter_size_map, Config::go2_mid360().filter_size_map);
        assert_eq!(config.lidar_to_imu_trans, Config::go2_mid360().lidar_to_imu_trans);
    }

    /// A recording keeps the geometry its odometry was computed under, so a
    /// later urdf that moves the lidar can be told from one that does not.
    #[test]
    fn a_urdf_that_moves_the_lidar_shows_up_as_drift_against_the_recorded_geometry() {
        let mut tree = StaticTree::new();
        tree.insert("base_link", "livox_link", Pose::new([0.0, 0.0, 0.10], [0.0, 0.0, 0.0, 1.0]));
        tree.insert("livox_link", "livox_frame", Pose::IDENTITY);
        let marker = geometry_marker("base_link", &tree.pose_in("base_link", "livox_frame").unwrap());

        let (metres, radians) = geometry_drift(&marker, &tree, "livox_frame").unwrap();
        assert!(metres < 1e-6 && radians < 1e-6, "the same tree drifted: {metres} m {radians} rad");

        // The real rig pitches the mount back 25 degrees and sits it higher.
        let mut corrected = StaticTree::new();
        let quarter = (0.436332_f64 / 2.0).sin();
        corrected.insert(
            "base_link",
            "livox_link",
            Pose::new([0.0, 0.029, 0.1356], [quarter, 0.0, 0.0, (0.436332_f64 / 2.0).cos()]),
        );
        corrected.insert("livox_link", "livox_frame", Pose::IDENTITY);
        let (metres, radians) = geometry_drift(&marker, &corrected, "livox_frame").unwrap();
        assert!(metres > 0.03, "{metres}");
        assert!((radians.to_degrees() - 25.0).abs() < 0.5, "{}", radians.to_degrees());

        assert!(geometry_drift("nonsense", &tree, "livox_frame").is_none());
        assert!(geometry_drift(&marker, &tree, "no_such_frame").is_none());
    }

    #[test]
    fn the_topic_pair_is_the_cloud_and_the_imu_beside_it() {
        let path = std::env::temp_dir().join(format!("lite_record_pair_{}.mcap", crate::record::now_nanos()));
        let file = std::fs::File::create(&path).unwrap();
        let mut writer = mcap::WriteOptions::new().profile("ros2").create(std::io::BufWriter::new(file)).unwrap();
        let cloud = writer.add_schema(crate::msgs::POINT_CLOUD2_TYPE, "ros2msg", b"").unwrap();
        let imu = writer.add_schema(crate::msgs::IMU_TYPE, "ros2msg", b"").unwrap();
        writer.add_channel(imu, "/realsense/imu", "cdr", &Default::default()).unwrap();
        writer.add_channel(cloud, "/livox/lidar", "cdr", &Default::default()).unwrap();
        writer.add_channel(imu, "/livox/imu", "cdr", &Default::default()).unwrap();
        writer.finish().unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
        assert_eq!(find_lidar_and_imu(&summary), Some(("/livox/lidar".into(), "/livox/imu".into())));
        std::fs::remove_file(&path).ok();
    }
}
