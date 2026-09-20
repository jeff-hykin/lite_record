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

use crate::deskew::Spool;
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

/// Header stamps are the sensor's own clock; a stream whose header clock is off
/// still sorts correctly by log time, so fall back to it the way heatmap does.
fn stamp_seconds(header: &Header, log_time: u64) -> f64 {
    if header.stamp_sec <= 0 {
        return log_time as f64 / 1e9;
    }
    header.stamp_sec as f64 + header.stamp_nsec as f64 / 1e9
}

/// The fewest scans between two snapshots of the accumulating map. The lidar
/// runs at 10 Hz, so this is one snapshot a second on a short recording.
///
/// The map is worth watching build, not just seeing finished: a single message
/// covering the whole run would only ever draw at whatever instant it carried,
/// and every other moment on the timeline would show the live scan and nothing
/// else. Snapshots cost size -- the last ones are the full map -- which is why
/// this is a second rather than every frame.
pub const SNAPSHOT_EVERY_SCANS: usize = 10;

/// The most snapshots a recording gets, however long it is. Each snapshot is
/// the whole map so far, so a fixed cadence makes the series quadratic in the
/// recording's length: an hour at one a second would append thousands of
/// copies of a city block. Spreading a bounded count over the run keeps the
/// series a fraction of the recording whatever its length.
pub const MAX_SNAPSHOTS: usize = 120;

/// Scans between snapshots for a run of `scans` scans.
pub fn snapshot_every(scans: usize) -> usize {
    scans.div_ceil(MAX_SNAPSHOTS).max(SNAPSHOT_EVERY_SCANS)
}

/// What a run produced. The snapshots are on disk in `snapshots`, encoded and
/// in log-time order, because on a long recording they are many times the
/// size of the finished map; only the finished map itself is in memory.
pub struct Map {
    pub snapshots: Spool,
    /// The whole run, stamped at its last placed scan.
    pub final_cloud: PointCloud2,
    pub final_stamp_nanos: u64,
    pub scans: usize,
    /// Scans with no pose within tolerance, which are left out rather than
    /// placed somewhere wrong.
    pub unplaced: usize,
    pub voxel_size: f32,
}

impl Map {
    /// Points in the finished map.
    pub fn points(&self) -> usize {
        self.final_cloud.width as usize
    }

    /// Snapshots spooled so far, the finished map included.
    pub fn snapshot_count(&self) -> u64 {
        self.snapshots.clouds()
    }

    pub fn bytes(&self) -> u64 {
        self.snapshots.bytes()
    }
}

/// A flat xyz triple list as a `PointCloud2` in `frame`.
fn cloud_of(flat: &[f32], stamp_nanos: u64, frame: &str) -> PointCloud2 {
    let mut data = Vec::with_capacity(flat.len() * 4);
    for value in flat {
        data.extend_from_slice(&value.to_le_bytes());
    }
    let points = (flat.len() / 3) as u32;
    PointCloud2 {
        header: Header::new(stamp_nanos, frame),
        height: 1,
        width: points,
        fields: vec![
            PointField { name: "x".into(), offset: 0, datatype: crate::msgs::POINT_FIELD_FLOAT32, count: 1 },
            PointField { name: "y".into(), offset: 4, datatype: crate::msgs::POINT_FIELD_FLOAT32, count: 1 },
            PointField { name: "z".into(), offset: 8, datatype: crate::msgs::POINT_FIELD_FLOAT32, count: 1 },
        ],
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * points,
        data,
        is_dense: true,
    }
}

/// Folds every scan on `cloud_topic` into one map, each placed by walking tf
/// from the cloud's own frame up to the world.
///
/// **Not** by the odometry pose directly. The estimator's pose is for the body
/// -- `odom -> base_link` once a urdf has re-rooted it -- while the clouds are
/// stamped in `livox_frame`, and between them sit the mount joints, including
/// the Mid-360's 90 degree rotation. Applying the body pose to lidar-frame
/// points turns every scan by that mount and the map comes out an inflated
/// blob rather than a building. tf already carries the whole chain, moving
/// edges and fixed ones alike, so walking it is both correct and indifferent to
/// where the odometry happens to be rooted.
pub fn build(
    recording: &Recording,
    cloud_topic: &str,
    tf_topic: &str,
    world_frame: &str,
    scans_done: &Arc<AtomicU64>,
    gauge: Option<&crate::progress::Gauge>,
) -> Result<Map> {
    let config = default_config(world_frame);
    let voxel_size = config.voxel_size;
    let transforms = crate::heatmap::TfHistory::read(recording, tf_topic)?;
    let channel = recording.channel(cloud_topic)?;
    let every = snapshot_every(recording.message_count(channel.id).unwrap_or(0) as usize);
    let mut mapper = Mapper::new(config);
    let (mut scans, mut unplaced) = (0usize, 0usize);
    let mut snapshots = Spool::beside_named(&recording.path, ".map-spool")?;
    let mut last_stamp_nanos = 0u64;
    let mut last_snapshot_stamp = None;

    for message in recording.messages(channel.id, None)? {
        let message = message?;
        let cloud = crate::cdr::decode_point_cloud2(&message.data)
            .with_context(|| format!("message {} on {cloud_topic}", message.sequence))?;
        let stamp = stamp_seconds(&cloud.header, message.log_time);
        let (placement, root) = transforms.chain_to_root(&cloud.header.frame_id, stamp);
        // A frame tf does not reach the world is a frame nothing can place, and
        // guessing is worse than leaving the scan out.
        if root != world_frame {
            unplaced += 1;
            continue;
        }
        last_stamp_nanos = message.log_time;
        if let Some(gauge) = gauge {
            gauge.at(message.log_time);
        }
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
            MapperPose {
                position: (
                    placement.translation[0] as f32,
                    placement.translation[1] as f32,
                    placement.translation[2] as f32,
                ),
                orientation: (
                    placement.rotation[0] as f32,
                    placement.rotation[1] as f32,
                    placement.rotation[2] as f32,
                    placement.rotation[3] as f32,
                ),
            },
        );
        scans += 1;
        scans_done.fetch_add(1, Ordering::Relaxed);
        if let Some(gauge) = gauge.filter(|_| scans.is_multiple_of(10)) {
            gauge.detail(format!("{scans} scans"));
        }
        if scans % every == 0 {
            let snapshot = cloud_of(&mapper.global_points(), message.log_time, world_frame);
            snapshots.push_bytes(message.log_time, &crate::cdr::point_cloud2(&snapshot).data)?;
            last_snapshot_stamp = Some(message.log_time);
        }
    }
    if scans == 0 {
        snapshots.discard();
        bail!("tf never placed {cloud_topic}'s frame in {world_frame}");
    }

    // Only now, with every scan folded in and every ray cast, is the map
    // finished: this cloud is what the .pc2.lcm carries and what anyone
    // scrubbing to the end expects to see. The last scan rarely lands on the
    // snapshot cadence, so it is usually one more snapshot too.
    let final_cloud = cloud_of(&mapper.global_points(), last_stamp_nanos, world_frame);
    if last_snapshot_stamp != Some(last_stamp_nanos) {
        snapshots.push_bytes(last_stamp_nanos, &crate::cdr::point_cloud2(&final_cloud).data)?;
    }
    Ok(Map { snapshots, final_cloud, final_stamp_nanos: last_stamp_nanos, scans, unplaced, voxel_size })
}

/// Writes the map into the recording as [`GLOBAL_MAP_TOPIC`] and beside it as a
/// `.pc2.lcm`, and returns where that file went.
///
/// The two carry the same cloud in different encodings on purpose: the topic is
/// CDR so anything reading the mcap sees it, the file is LCM because that is
/// what dimos' map tooling opens.
pub fn write(recording: &std::path::Path, map: Map, gauge: Option<&crate::progress::Gauge>) -> Result<std::path::PathBuf> {
    if map.final_cloud.width == 0 {
        map.snapshots.discard();
        bail!("the map came out empty");
    }
    let sample = crate::cdr::point_cloud2(&map.final_cloud);
    let mut appender = crate::mcap_append::Appender::open(recording)?;
    let schema = appender.schema(sample.schema_name, "ros2msg", sample.schema_text.as_bytes());
    let channel = appender.channel(
        GLOBAL_MAP_TOPIC,
        schema,
        "cdr",
        &crate::record::channel_metadata(GLOBAL_MAP_TOPIC),
    );
    // In log-time order, which is the order a reader will want them, and each at
    // the scan that completed it so the map grows as the recording plays.
    // Streamed straight from the spool: the series is far larger than memory.
    map.snapshots.drain(|stamp, data| {
        if let Some(gauge) = gauge {
            gauge.at(stamp);
        }
        appender.write_stream(channel, stamp, data)
    })?;
    appender.finish()?;

    // The file beside it is the finished map, not the series: it is a single
    // cloud by definition, and what anybody opening it wants is the whole thing.
    // Written last, after the recording is complete, so a .pc2.lcm on disk
    // always means the whole run went in.
    let beside = recording.with_extension("pc2.lcm");
    std::fs::write(&beside, crate::lcm::point_cloud2(&map.final_cloud))
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
