//! Loop closure for `post_process`: the estimator's trajectory drifts, a
//! little per metre, and a walk that comes back to where it started ends with
//! the same wall twice, some centimetres apart. This stage corrects the
//! trajectory before anything is appended, so the odometry, the tf edge and
//! the map the recording ends up carrying are the corrected ones -- there is
//! no `_corrected` copy of anything.
//!
//! The work is icp_stitch's, used as a library: keyframes along the
//! trajectory, AprilTag landmark factors when the recording has a colour
//! camera with intrinsics, point-to-plane ICP between revisited keyframes,
//! and a GTSAM pose-graph solve. What comes back is a rigid correction per
//! keyframe; every pose between two keyframes gets the interpolated one.
//!
//! Only the trajectory changes. The motion-compensated clouds are in the
//! lidar's own frame and are placed through tf, so they need no rewriting.

use std::collections::{BTreeMap, HashSet};

use anyhow::{bail, Context, Result};
use icp_stitch::apriltags::{filter_glimpses, GlimpseGates};
use icp_stitch::artifacts::interpolate_correction;
use icp_stitch::detect::{CameraModel, TagDetector};
use icp_stitch::memory2::ScanRow;
use icp_stitch::pgo::{self, OdomPoseRow, Tuning};
use icp_stitch::se3::{self, Pose3};

use crate::deskew::Spool;
use crate::odometry::{pose_of, Estimate};
use crate::progress::Gauge;
use crate::tf::{Pose, StaticTree};
use crate::topics::Recording;

/// What the stage is asked to do.
pub struct Options {
    /// Side length of the AprilTags in the recording, in metres.
    pub tag_size_m: f64,
    /// Whether to look for tags at all; needs a colour camera with intrinsics.
    pub tags: bool,
    /// Whether to add ICP closures between revisited keyframes.
    pub icp: bool,
}

/// The tag dictionary the rigs carry.
const TAG_DICTIONARY: &str = "DICT_APRILTAG_36h11";
/// icp_stitch's default: metres between keyframes considered for a closure.
const CLOSURE_SPACING_M: f64 = 2.0;

/// What the stage did, for the display.
#[derive(Debug, Default)]
pub struct Report {
    pub keyframes: usize,
    pub camera: Option<String>,
    pub images: usize,
    pub detections: usize,
    pub tag_factors: usize,
    pub tags_seen: usize,
    pub closures_accepted: usize,
    /// The largest translation any keyframe moved by, in metres.
    pub max_shift_m: f64,
}

/// A colour camera the tags can be seen with: its frames and its intrinsics.
struct Camera {
    image_topic: String,
    model: CameraModel,
    optical_frame: String,
}

fn pose3_of(pose: &Pose) -> Pose3 {
    let [x, y, z] = pose.translation;
    let [qx, qy, qz, qw] = pose.rotation;
    se3::from_xyzquat(&[x, y, z, qx, qy, qz, qw])
}

fn pose_of3(pose: &Pose3) -> Pose {
    Pose { translation: pose.translation, rotation: se3::quaternion_xyzw(pose) }
}

/// The odometry as icp_stitch reads it: one row per pose, IMU in odom.
fn odometry_rows(estimate: &Estimate) -> Vec<OdomPoseRow> {
    estimate
        .poses
        .iter()
        .map(|sample| {
            let pose = pose_of(sample);
            let [x, y, z] = pose.translation;
            let [qx, qy, qz, qw] = pose.rotation;
            [sample.time, x, y, z, qx, qy, qz, qw]
        })
        .collect()
}

/// The rig's linear and angular speed at `ts`, from the poses around it.
fn speed_at(rows: &[OdomPoseRow], ts: f64) -> (f64, f64) {
    let after = rows.partition_point(|row| row[0] < ts).min(rows.len() - 1);
    let before = after.saturating_sub(1);
    let (a, b) = (&rows[before], &rows[after]);
    let dt = b[0] - a[0];
    if dt <= 0.0 {
        return (-1.0, -1.0);
    }
    let moved = ((b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2) + (b[3] - a[3]).powi(2)).sqrt();
    let turned = icp_stitch::mat3::norm(&se3::so3_log(
        &se3::between(&se3::from_xyzquat(&a[1..]), &se3::from_xyzquat(&b[1..])).rotation,
    ));
    (moved / dt, turned / dt)
}

/// The first colour CompressedImage channel with a CameraInfo beside it.
fn find_camera(recording: &Recording) -> Result<Option<Camera>> {
    let channels = recording.channels()?;
    let is = |channel: &mcap::Channel, schema: &str| channel.schema.as_ref().is_some_and(|s| s.name == schema);
    let mut infos: BTreeMap<String, u16> = BTreeMap::new();
    for channel in &channels {
        if is(channel, crate::msgs::CAMERA_INFO_TYPE) {
            infos.insert(channel.topic.clone(), channel.id);
        }
    }
    let mut images: Vec<&mcap::Channel> = channels
        .iter()
        .map(|channel| channel.as_ref())
        .filter(|channel| is(channel, crate::msgs::COMPRESSED_IMAGE_TYPE) && channel.topic.contains("color"))
        .collect();
    images.sort_by(|a, b| a.topic.cmp(&b.topic));
    for channel in images {
        // /x/color/image_raw/compressed sits beside /x/color/camera_info, and
        // the recorder also writes the sibling name /x/color/image_raw/camera_info.
        let stem = channel.topic.trim_end_matches("/compressed");
        let candidates = [
            format!("{stem}/camera_info"),
            format!("{}/camera_info", stem.rsplit_once('/').map_or(stem, |(head, _)| head)),
        ];
        let Some(info_id) = candidates.iter().find_map(|name| infos.get(name)).copied() else {
            continue;
        };
        let Some(first) = recording.messages(info_id, None)?.next() else {
            continue;
        };
        let info = crate::distortion::parse_camera_info(&first?.data);
        if info.intrinsics[0] <= 0.0 {
            continue;
        }
        return Ok(Some(Camera {
            image_topic: channel.topic.clone(),
            model: CameraModel::from_info(&info.intrinsics, &info.distortion, &info.distortion_model),
            optical_frame: info.header.frame_id,
        }));
    }
    Ok(None)
}

/// A CompressedImage's pixels as 8-bit grey, whatever codec it came in.
fn decode_grey(payload: &[u8]) -> Option<(Vec<u8>, usize, usize, u64)> {
    let mut reader = crate::cdr::CdrReader::new(payload);
    let header = reader.header();
    let format = reader.string().to_ascii_lowercase();
    let bytes = reader.bytes();
    let image = match format.as_str() {
        "jpeg" | "jpg" => crate::image::decode_jpeg(&bytes),
        "jxl" | "jpegxl" => crate::image::decode_jpegxl(&bytes),
        "png" => crate::image::decode_png(&bytes),
        "webp" => crate::image::decode_webp(&bytes),
        _ => return None,
    }
    .ok()?;
    let (width, height) = (image.width, image.height);
    // Rec. 601 luma, the same weights either way round (b and r swap, g stays).
    let luma = |p: &[u8]| ((p[0] as u32 * 77 + p[1] as u32 * 150 + p[2] as u32 * 29) >> 8) as u8;
    let channels = match image.encoding.as_str() {
        "mono8" => return Some((image.data, width, height, header.stamp_nanos())),
        "rgb8" | "bgr8" => 3,
        "rgba8" | "bgra8" => 4,
        _ => return None,
    };
    let grey: Vec<u8> = image.data.chunks_exact(channels).map(luma).collect();
    Some((grey, width, height, header.stamp_nanos()))
}

/// Where the camera's optical frame sits in the IMU frame the poses describe:
/// lidar-in-IMU from the estimator, camera-in-lidar from the static tree.
fn optical_in_imu(tree: &StaticTree, estimate: &Estimate, optical_frame: &str) -> Option<Pose> {
    let root = tree.root_of(&estimate.lidar_frame)?;
    let lidar_in_root = tree.pose_in(&root, &estimate.lidar_frame)?;
    let optical_in_root = tree.pose_in(&root, optical_frame)?;
    let optical_in_lidar = lidar_in_root.inverse().then(&optical_in_root);
    Some(estimate.lidar_in_imu.then(&optical_in_lidar))
}

/// Corrects `estimate`'s poses in place. `spool` holds the motion-compensated
/// clouds the ICP closures register; without it (or with `icp` off) only the
/// tags can pull the trajectory into shape.
pub fn close_loops(
    recording: &Recording,
    estimate: &mut Estimate,
    spool: Option<&mut Spool>,
    tree: &StaticTree,
    options: &Options,
    gauge: &Gauge,
) -> Result<Report> {
    if estimate.poses.len() < 2 {
        bail!("loop closure needs a trajectory, and the estimate has {} pose(s)", estimate.poses.len());
    }
    let tuning = Tuning::default();
    let rows = odometry_rows(estimate);
    let (_, keyframe_poses, keyframe_times) = pgo::select_keyframes(&rows, &tuning);
    let mut report = Report { keyframes: keyframe_poses.len(), ..Report::default() };
    gauge.detail(format!("{} keyframes", report.keyframes));

    // Tags: every colour frame is searched, each detection posed against the
    // camera model, the glimpses filtered the way icp_stitch filters them, and
    // the best sighting of each tag per keyframe becomes a landmark factor.
    let mut best = BTreeMap::new();
    let mut base_optical = Pose3::identity();
    if options.tags {
        match find_camera(recording)? {
            None => {}
            Some(camera) => match optical_in_imu(tree, estimate, &camera.optical_frame) {
                None => bail!(
                    "tf does not place the camera frame {} against the lidar {}, so tags cannot be used; pass --no-tags",
                    camera.optical_frame,
                    estimate.lidar_frame
                ),
                Some(placement) => {
                    base_optical = pose3_of(&placement);
                    report.camera = Some(camera.image_topic.clone());
                    let mut detector = TagDetector::new(TAG_DICTIONARY).map_err(anyhow::Error::msg)?;
                    let mut raw = Vec::new();
                    let channel = recording.channel(&camera.image_topic)?;
                    for message in recording.messages(channel.id, None)? {
                        let message = message?;
                        report.images += 1;
                        gauge.at(message.log_time);
                        let Some((grey, width, height, stamp)) = decode_grey(&message.data) else {
                            continue;
                        };
                        let ts = if stamp == 0 { message.log_time } else { stamp } as f64 * 1e-9;
                        raw.extend(
                            detector
                                .detect(ts, &grey, width, height, &camera.model, options.tag_size_m, speed_at(&rows, ts))
                                .map_err(anyhow::Error::msg)?,
                        );
                        if report.images.is_multiple_of(50) {
                            gauge.detail(format!("{} keyframes, {} frames, {} tag sightings", report.keyframes, report.images, raw.len()));
                        }
                    }
                    report.detections = raw.len();
                    let kept = filter_glimpses(&raw, &HashSet::new(), &GlimpseGates::default());
                    best = pgo::best_factor_per_keyframe_marker(&kept, &keyframe_times);
                    report.tag_factors = best.len();
                }
            },
        }
    }

    let (mut graph, values, seen) =
        pgo::build_tag_graph(&keyframe_poses, &best, &base_optical, &tuning).map_err(anyhow::Error::msg)?;
    report.tags_seen = seen.len();
    gauge.detail(format!("{} keyframes, {} tag factors; solving", report.keyframes, report.tag_factors));
    let mut solved = pgo::solve(&graph, &values, &tuning).map_err(anyhow::Error::msg)?;

    // ICP: the motion-compensated scans, placed by the trajectory as it stands,
    // registered between keyframes that come back within reach of each other.
    if options.icp {
        if let Some(spool) = spool {
            let mut scans: Vec<ScanRow> = Vec::new();
            spool.for_each(|log_time, data| {
                let cloud = crate::cdr::decode_point_cloud2(data)?;
                let stamp = cloud.header.stamp_nanos();
                let ts = if stamp == 0 { log_time } else { stamp } as f64 * 1e-9;
                let points = crate::heatmap::cloud_points(&cloud)?;
                scans.push(ScanRow { ts, points, intensities: None, frame_id: cloud.header.frame_id.clone() });
                Ok(())
            })?;
            gauge.detail(format!("{} keyframes, {} scans; registering revisits", report.keyframes, scans.len()));
            let lidar_in_imu = pose3_of(&estimate.lidar_in_imu);
            let world_points = |scan: &ScanRow| -> Result<Vec<[f64; 3]>, String> {
                // The scan was taken at one of the odometry's own instants.
                let index = icp_stitch::artifacts::nearest_index(&keyframe_times, scan.ts);
                let _ = index;
                let at = rows.partition_point(|row| row[0] < scan.ts).min(rows.len() - 1);
                let at = if at > 0 && (rows[at - 1][0] - scan.ts).abs() < (rows[at][0] - scan.ts).abs() { at - 1 } else { at };
                let lidar_in_world = se3::compose(&se3::from_xyzquat(&rows[at][1..]), &lidar_in_imu);
                Ok(scan
                    .points
                    .iter()
                    .map(|p| {
                        let point = [p[0] as f64, p[1] as f64, p[2] as f64];
                        icp_stitch::mat3::add(
                            &icp_stitch::mat3::mat_vec(&lidar_in_world.rotation, &point),
                            &lidar_in_world.translation,
                        )
                    })
                    .collect())
            };
            report.closures_accepted = pgo::add_icp_closures(
                &mut graph,
                &solved,
                &scans,
                &keyframe_poses,
                &keyframe_times,
                &world_points,
                CLOSURE_SPACING_M,
                &tuning,
            )
            .map_err(anyhow::Error::msg)?;
            if report.closures_accepted > 0 {
                gauge.detail(format!("{} closures; solving again", report.closures_accepted));
                solved = pgo::solve(&graph, &solved, &tuning).map_err(anyhow::Error::msg)?;
            }
        }
    }

    // What the solve moved each keyframe by, then every pose gets the
    // interpolated correction, on the world side: the body offsets below the
    // IMU are untouched.
    let corrections: Vec<Pose3> = keyframe_poses
        .iter()
        .enumerate()
        .map(|(index, raw)| {
            let optimized = solved
                .pose3(index as u64)
                .with_context(|| format!("the solve lost keyframe {index}"))?;
            Ok(se3::compose(&optimized, &se3::inverse(raw)))
        })
        .collect::<Result<_>>()?;
    report.max_shift_m = corrections
        .iter()
        .map(|c| icp_stitch::mat3::norm(&c.translation))
        .fold(0.0, f64::max);
    for sample in &mut estimate.poses {
        let correction = interpolate_correction(&keyframe_times, &corrections, sample.time);
        let corrected = pose_of3(&se3::compose(&correction, &pose3_of(&pose_of(sample))));
        let rotation = icp_stitch::mat3::mat_from_quat(&[
            corrected.rotation[3],
            corrected.rotation[0],
            corrected.rotation[1],
            corrected.rotation[2],
        ]);
        for (index, value) in corrected.translation.iter().enumerate() {
            sample.pos[index] = *value;
        }
        for (row, values) in rotation.iter().enumerate() {
            for (col, value) in values.iter().enumerate() {
                sample.rot[(row, col)] = *value;
            }
        }
    }
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn poses_round_trip_through_icp_stitch() {
        let pose = Pose { translation: [1.0, -2.0, 0.5], rotation: [0.0, 0.0, 0.7071068, 0.7071068] };
        let back = pose_of3(&pose3_of(&pose));
        for i in 0..3 {
            assert!((back.translation[i] - pose.translation[i]).abs() < 1e-9);
        }
        for i in 0..4 {
            assert!((back.rotation[i] - pose.rotation[i]).abs() < 1e-6, "{back:?}");
        }
    }

    #[test]
    fn speed_is_taken_from_the_poses_around_the_instant() {
        // 1 m/s along x, no turning.
        let rows: Vec<OdomPoseRow> = (0..10).map(|i| [i as f64, i as f64, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]).collect();
        let (linear, angular) = speed_at(&rows, 4.5);
        assert!((linear - 1.0).abs() < 1e-9);
        assert!(angular.abs() < 1e-9);
    }
}
