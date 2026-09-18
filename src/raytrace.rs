//! Builds a global occupancy map from a recording, with dimos' raycast-clearing
//! voxel mapper.
//!
//! The estimator leaves motion-compensated clouds on `/pointlio_lidar` and poses
//! on `/pointlio_odometry`; this folds each scan into a voxel map from the pose
//! it was taken at. Raycasting is what makes it more than an accumulation: a ray
//! that reaches a return clears whatever it passed through, so a person who
//! walked through the scan does not stay in the map as a smear of occupied
//! voxels.
//!
//! The result goes two places, because the two are read by different things:
//! a `/global_map` topic inside the recording, for anything reading the mcap,
//! and a `.pc2.lcm` beside it, which is what `dimos map view` and that tooling
//! open.

use anyhow::{bail, Context, Result};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use voxel_ray_tracing::mapper::{Mapper, Pose as MapperPose};
use voxel_ray_tracing::voxel_ray_tracer::Config;

use crate::msgs::{Header, PointCloud2, PointField};
use crate::topics::Recording;

/// The topic the map is written back to.
pub const GLOBAL_MAP_TOPIC: &str = "/global_map";

/// Returns closer than this are dropped before mapping; see the note where it
/// is used.
const MIN_RANGE_M: f32 = 1.0;

/// dimos' own defaults for this mapper, from
/// `dimos/mapping/ray_tracing/module.py`. Only the frame is ours: dimos takes it
/// from a blueprint, and here it is whatever the recording's odometry is in.
pub fn default_config(world_frame: &str) -> Config {
    Config {
        voxel_size: 0.08,
        fine_divisor: 3,
        emit_fine: false,
        max_range: 30.0,
        ray_subsample: 1,
        shadow_depth: 0.1,
        grace_depth: 0.2,
        min_health: -1,
        max_health: 5,
        range_error_coeff: 0.0,
        graze_cos: 0.7,
        support_min: 4,
        // The local map and its region bounds are a live-navigation concern:
        // they exist so a planner can be handed the part of the map near the
        // robot, frame by frame. Nothing here consumes them, and leaving them on
        // costs a batch copy of every scan, so both cadences are off and only
        // the global map is built.
        emit_every: 0,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: world_frame.to_string(),
        tf_match_tolerance_s: 0.1,
        worker_threads: 4,
    }
}

struct StampedPose {
    stamp: f64,
    position: (f32, f32, f32),
    orientation: (f32, f32, f32, f32),
}

/// Header stamps are the sensor's own clock; a stream whose header clock is off
/// still sorts correctly by log time, so fall back to it the way heatmap does.
fn stamp_seconds(header: &Header, log_time: u64) -> f64 {
    if header.stamp_sec <= 0 {
        return log_time as f64 / 1e9;
    }
    header.stamp_sec as f64 + header.stamp_nsec as f64 / 1e9
}

fn read_poses(recording: &Recording, topic: &str) -> Result<Vec<StampedPose>> {
    let channel = recording.channel(topic)?;
    let mut poses = Vec::new();
    for message in recording.messages(channel.id, None)? {
        let message = message?;
        let pose = crate::cdr::decode_odometry(&message.data)
            .with_context(|| format!("message {} on {topic}", message.sequence))?;
        poses.push(StampedPose {
            stamp: stamp_seconds(&pose.header, message.log_time),
            position: (
                pose.position[0] as f32,
                pose.position[1] as f32,
                pose.position[2] as f32,
            ),
            orientation: (
                pose.orientation[0] as f32,
                pose.orientation[1] as f32,
                pose.orientation[2] as f32,
                pose.orientation[3] as f32,
            ),
        });
    }
    if poses.is_empty() {
        bail!("no poses on {topic}");
    }
    poses.sort_by(|left, right| left.stamp.total_cmp(&right.stamp));
    Ok(poses)
}

fn nearest<'a>(poses: &'a [StampedPose], stamp: f64) -> &'a StampedPose {
    let after = poses.partition_point(|pose| pose.stamp < stamp).min(poses.len() - 1);
    match after.checked_sub(1).map(|index| &poses[index]) {
        Some(before) if (before.stamp - stamp).abs() < (poses[after].stamp - stamp).abs() => before,
        _ => &poses[after],
    }
}

/// What a run produced, for the caller's progress line.
///
/// The map is one message covering the whole run, and a viewer shows the latest
/// message at or before the playhead -- so stamping it at the end, when it was
/// finished, hides it for the entire recording bar the last frame. It is
/// stamped at the *first* scan that went into it instead, which is the earliest
/// moment it can honestly be said to describe, and makes it visible wherever
/// you scrub.
pub struct Map {
    pub cloud: PointCloud2,
    pub scans: usize,
    /// Scans with no pose within tolerance, which are left out rather than
    /// placed somewhere wrong.
    pub unplaced: usize,
    pub voxel_size: f32,
}

impl Map {
    pub fn points(&self) -> usize {
        self.cloud.width as usize
    }
}

/// Folds every scan on `cloud_topic`, placed by `odom_topic`, into one map.
pub fn build(
    recording: &Recording,
    cloud_topic: &str,
    odom_topic: &str,
    world_frame: &str,
    scans_done: &Arc<AtomicU64>,
) -> Result<Map> {
    let config = default_config(world_frame);
    let tolerance = config.tf_match_tolerance_s;
    let voxel_size = config.voxel_size;
    let poses = read_poses(recording, odom_topic)?;
    let channel = recording.channel(cloud_topic)?;
    let mut mapper = Mapper::new(config);
    let (mut scans, mut unplaced) = (0usize, 0usize);
    let mut first_stamp_nanos = u64::MAX;

    for message in recording.messages(channel.id, None)? {
        let message = message?;
        let cloud = crate::cdr::decode_point_cloud2(&message.data)
            .with_context(|| format!("message {} on {cloud_topic}", message.sequence))?;
        let stamp = stamp_seconds(&cloud.header, message.log_time);
        let pose = nearest(&poses, stamp);
        // Placing a scan by a pose from a different part of the run puts a whole
        // sweep somewhere it never was, which is worse than leaving it out.
        if (pose.stamp - stamp).abs() > tolerance {
            unplaced += 1;
            continue;
        }
        first_stamp_nanos = first_stamp_nanos.min(message.log_time);
        // A Livox sweep is a fixed 20064 slots and the ones that got no return
        // are written as (0, 0, 0). Deskewing rotates those off the origin
        // rather than dropping them, so they arrive as a shell of points within
        // half a metre of the sensor -- a fifth of every scan. Fed in, they
        // carve a sphere of "occupied" around the whole trajectory and raycast
        // away the real map behind it. A zero test no longer catches them once
        // they have been moved, so this goes by range.
        let points: Vec<(f32, f32, f32)> = crate::heatmap::cloud_points(&cloud)?
            .into_iter()
            .filter(|[x, y, z]| x * x + y * y + z * z > MIN_RANGE_M * MIN_RANGE_M)
            .map(|[x, y, z]| (x, y, z))
            .collect();
        if points.is_empty() {
            unplaced += 1;
            continue;
        }
        mapper.add_frame(
            points,
            MapperPose { position: pose.position, orientation: pose.orientation },
        );
        scans += 1;
        scans_done.fetch_add(1, Ordering::Relaxed);
    }
    if scans == 0 {
        bail!("no scan on {cloud_topic} had a pose on {odom_topic} within {tolerance} s");
    }

    let flat = mapper.global_points();
    let mut data = Vec::with_capacity(flat.len() * 4);
    for value in &flat {
        data.extend_from_slice(&value.to_le_bytes());
    }
    let points = flat.len() / 3;
    Ok(Map {
        cloud: PointCloud2 {
            header: Header::new(first_stamp_nanos, world_frame),
            height: 1,
            width: points as u32,
            fields: vec![
                PointField { name: "x".into(), offset: 0, datatype: crate::msgs::POINT_FIELD_FLOAT32, count: 1 },
                PointField { name: "y".into(), offset: 4, datatype: crate::msgs::POINT_FIELD_FLOAT32, count: 1 },
                PointField { name: "z".into(), offset: 8, datatype: crate::msgs::POINT_FIELD_FLOAT32, count: 1 },
            ],
            is_bigendian: false,
            point_step: 12,
            row_step: 12 * points as u32,
            data,
            is_dense: true,
        },
        scans,
        unplaced,
        voxel_size,
    })
}

/// Writes the map into the recording as [`GLOBAL_MAP_TOPIC`] and beside it as a
/// `.pc2.lcm`, and returns where that file went.
///
/// The two carry the same cloud in different encodings on purpose: the topic is
/// CDR so anything reading the mcap sees it, the file is LCM because that is
/// what dimos' map tooling opens.
pub fn write(recording: &std::path::Path, map: &Map) -> Result<std::path::PathBuf> {
    let encoded = crate::cdr::point_cloud2(&map.cloud);
    let mut appender = crate::mcap_append::Appender::open(recording)?;
    let schema = appender.schema(encoded.schema_name, "ros2msg", encoded.schema_text.as_bytes());
    let channel = appender.channel(
        GLOBAL_MAP_TOPIC,
        schema,
        "cdr",
        &crate::record::channel_metadata(GLOBAL_MAP_TOPIC),
    );
    let stamp = map.cloud.header.stamp_sec as u64 * 1_000_000_000
        + map.cloud.header.stamp_nsec as u64;
    appender.write(channel, stamp, encoded.data)?;
    appender.finish()?;

    let beside = recording.with_extension("pc2.lcm");
    std::fs::write(&beside, crate::lcm::point_cloud2(&map.cloud))
        .with_context(|| format!("could not write {}", beside.display()))?;
    Ok(beside)
}

/// Whether the recording already carries a map, so a second run does not leave
/// two of them on the topic -- appending cannot remove.
pub fn already_present(recording: &std::path::Path) -> Result<u64> {
    let opened = Recording::open(recording)?;
    let Ok(channel) = opened.channel(GLOBAL_MAP_TOPIC) else {
        return Ok(0);
    };
    Ok(opened.message_count(channel.id).unwrap_or(0))
}
