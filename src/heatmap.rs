//! Top-down dark render of a recording: a point cloud stream as a density
//! heatmap, with the odometry path drawn over it coloured start-to-end.
//!
//! A port of the `heatmap` Deno tool's mcap path. Every scan is carried into
//! the world by the pose at its own stamp — stacking them raw only draws the
//! lidar's field of view over itself — either from the nearest odometry pose or,
//! with `--tf`, along the tf chain from the scan's frame to the tree's root.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use clap::Args;

use crate::cdr::{self, CloudView};
use crate::msgs::{Header, POINT_FIELD_FLOAT32};
use crate::tf::{Pose, StaticTree};
use crate::topics::{schema_name, Recording};

const ODOMETRY_TYPE: &str = "nav_msgs/msg/Odometry";
const POSE_STAMPED_TYPE: &str = "geometry_msgs/msg/PoseStamped";

#[derive(Args, Debug, Clone)]
pub struct Options {
    pub recording: PathBuf,

    /// Defaults to `<recording>_heatmap.png` beside the input.
    pub output: Option<PathBuf>,

    /// Image width in pixels.
    #[arg(short = 'w', long, default_value_t = 1600)]
    pub width: usize,

    /// Point cloud topic.
    #[arg(long, default_value = "pointlio_lidar")]
    pub cloud: String,

    /// Odometry topic (nav_msgs/Odometry or geometry_msgs/PoseStamped).
    #[arg(long, default_value = "pointlio_odometry")]
    pub odom: String,

    /// Place clouds via this tf topic (plus /tf_static) instead of the odometry pose.
    #[arg(long)]
    pub tf: Option<String>,

    /// Rigidly align onto this odometry topic's frame.
    #[arg(long)]
    pub align_to: Option<String>,

    /// Force the world extent as minX_minY_maxX_maxY (commas or underscores),
    /// for comparable renders.
    #[arg(long)]
    pub extent: Option<String>,

    /// Drop points below this world z, in metres.
    #[arg(long)]
    pub min_height: Option<f64>,

    /// Drop points above this world z, in metres.
    #[arg(long)]
    pub max_height: Option<f64>,

    /// Use every Nth lidar scan.
    #[arg(long, default_value_t = 1)]
    pub stride: usize,

    /// Place scans by their mcap log time instead of their header stamp, for a
    /// recording whose sensor clock was stepped away from the clock the
    /// odometry runs on.
    #[arg(long)]
    pub by_log_time: bool,
}

/// Which clock a scan is placed on. A lidar's header stamps can sit a long way
/// from the odometry's if the machine's clock stepped after the sensor offset
/// was taken; when that happens the log time is the one clock both share.
struct ScanClock {
    by_log_time: bool,
    /// Stamp range of the poses or tf edges the scans are placed against.
    reference: (f64, f64),
    warned: bool,
}

/// How far outside the reference range a header stamp may sit before it is
/// treated as being on a different clock.
const CLOCK_TOLERANCE_SECONDS: f64 = 5.0;

impl ScanClock {
    fn stamp_for(&mut self, header: &Header, log_time: u64) -> f64 {
        let logged = log_time as f64 / 1e9;
        if self.by_log_time {
            return logged;
        }
        let stamped = stamp_seconds(header, log_time);
        let (low, high) = self.reference;
        let inside = |stamp: f64| (low - CLOCK_TOLERANCE_SECONDS..=high + CLOCK_TOLERANCE_SECONDS).contains(&stamp);
        if inside(stamped) || !inside(logged) {
            return stamped;
        }
        if !self.warned {
            self.warned = true;
            eprintln!(
                "heatmap: scan header stamps run {:.1} s from the odometry clock; placing scans by log time instead",
                logged - stamped
            );
        }
        logged
    }
}

#[derive(Clone, Debug)]
struct StampedPose {
    stamp: f64,
    pose: Pose,
}

/// Seconds from the header stamp, which is when the sensor saw the data. A
/// message with no stamp falls back to when it was written.
fn stamp_seconds(header: &Header, log_time: u64) -> f64 {
    let nanos = header.stamp_nanos();
    (if nanos == 0 { log_time } else { nanos }) as f64 / 1e9
}

/// The odometry stream sorted by stamp, since chunks of it may sit anywhere
/// in the file.
fn read_poses(recording: &Recording, topic: &str) -> Result<Vec<StampedPose>> {
    let channel = recording.channel(topic)?;
    let decode: fn(&[u8]) -> Result<cdr::Pose> = match schema_name(&channel) {
        ODOMETRY_TYPE => cdr::decode_odometry,
        POSE_STAMPED_TYPE => cdr::decode_pose_stamped,
        other => bail!("{} is {other}, not {ODOMETRY_TYPE} or {POSE_STAMPED_TYPE}", channel.topic),
    };
    let mut poses = Vec::new();
    for message in recording.messages(channel.id, None)? {
        let message = message?;
        let pose = decode(&message.data)
            .with_context(|| format!("message {} on {}", message.sequence, channel.topic))?;
        poses.push(StampedPose {
            stamp: stamp_seconds(&pose.header, message.log_time),
            pose: Pose::new(pose.position, pose.orientation),
        });
    }
    if poses.is_empty() {
        bail!("no {topic} in {}", recording.path.display());
    }
    poses.sort_by(|left, right| left.stamp.total_cmp(&right.stamp));
    Ok(poses)
}

fn nearest(poses: &[StampedPose], stamp: f64) -> &StampedPose {
    let after = poses.partition_point(|pose| pose.stamp < stamp).min(poses.len() - 1);
    match after.checked_sub(1).map(|index| &poses[index]) {
        Some(before) if (before.stamp - stamp).abs() < (poses[after].stamp - stamp).abs() => before,
        _ => &poses[after],
    }
}

#[derive(Clone, Debug)]
struct TfSample {
    stamp: f64,
    parent: String,
    pose: Pose,
}

/// Every edge of the tf tree over time. A single message rarely holds the
/// whole tree — dimos publishes the mount frames from one module and the
/// moving odom->base_link from another — so edges are kept per child and
/// looked up by stamp, with the static ones valid at every time.
#[derive(Default)]
struct TfHistory {
    dynamic: HashMap<String, Vec<TfSample>>,
    fixed: StaticTree,
}

impl TfHistory {
    fn read(recording: &Recording, topic: &str) -> Result<Self> {
        let mut history = TfHistory::default();
        let channel = recording.channel(topic)?;
        for message in recording.messages(channel.id, None)? {
            let message = message?;
            for transform in cdr::decode_tf_message(&message.data)? {
                history
                    .dynamic
                    .entry(transform.child_frame_id.clone())
                    .or_default()
                    .push(TfSample {
                        stamp: stamp_seconds(&transform.header, message.log_time),
                        parent: transform.header.frame_id.clone(),
                        pose: Pose::from_transform(&transform),
                    });
            }
        }
        if history.dynamic.is_empty() {
            bail!("no {topic} in {}", recording.path.display());
        }
        for samples in history.dynamic.values_mut() {
            samples.sort_by(|left, right| left.stamp.total_cmp(&right.stamp));
        }
        if let Ok(fixed) = recording.channel("tf_static") {
            for message in recording.messages(fixed.id, None)? {
                for transform in cdr::decode_tf_message(&message?.data)? {
                    history.fixed.insert_transform(&transform);
                }
            }
        }
        Ok(history)
    }

    /// The edge placing `child` at `stamp`: interpolated between the bracketing
    /// dynamic samples, clamped at either end, or the static edge.
    fn edge_at(&self, child: &str, stamp: f64) -> Option<(&str, Pose)> {
        if let Some(samples) = self.dynamic.get(child) {
            let after = samples.partition_point(|sample| sample.stamp < stamp);
            let pose = match (after.checked_sub(1), samples.get(after)) {
                (Some(before), Some(next)) => {
                    let before = &samples[before];
                    let span = next.stamp - before.stamp;
                    let fraction = if span > 0.0 { (stamp - before.stamp) / span } else { 0.0 };
                    before.pose.interpolate(&next.pose, fraction)
                }
                (None, Some(next)) => next.pose,
                (Some(before), None) => samples[before].pose,
                (None, None) => return None,
            };
            return Some((samples[after.min(samples.len() - 1)].parent.as_str(), pose));
        }
        let parent = self.fixed.parent_of(child)?;
        Some((parent, self.fixed.pose_in(parent, child)?))
    }

    /// Walks `frame` up to the root of the tree at `stamp`, giving the pose of
    /// `frame` in that root and the root's name.
    fn chain_to_root(&self, frame: &str, stamp: f64) -> (Pose, String) {
        let mut pose = Pose::IDENTITY;
        let mut cursor = frame.to_string();
        let mut seen = HashSet::new();
        while seen.insert(cursor.clone()) {
            let Some((parent, edge)) = self.edge_at(&cursor, stamp) else {
                break;
            };
            pose = edge.then(&pose);
            cursor = parent.to_string();
        }
        (pose, cursor)
    }
}

/// The xyz of every point, in whatever frame the cloud was stored.
fn cloud_points(cloud: &CloudView<'_>) -> Result<Vec<[f32; 3]>> {
    if cloud.is_bigendian {
        bail!("big-endian point data is not supported");
    }
    let offset_of = |name: &str| -> Result<usize> {
        let field = cloud
            .fields
            .iter()
            .find(|field| field.name == name)
            .with_context(|| format!("cloud has no {name} field"))?;
        if field.datatype != POINT_FIELD_FLOAT32 {
            bail!("field {name} is datatype {}, not float32", field.datatype);
        }
        Ok(field.offset as usize)
    };
    let offsets = [offset_of("x")?, offset_of("y")?, offset_of("z")?];
    let step = cloud.point_step as usize;
    if step == 0 {
        bail!("cloud has a zero point_step");
    }
    let count = (cloud.width as usize * (cloud.height as usize).max(1)).min(cloud.data.len() / step);
    Ok((0..count)
        .map(|index| {
            let base = index * step;
            offsets.map(|offset| {
                f32::from_le_bytes(cloud.data[base + offset..base + offset + 4].try_into().unwrap())
            })
        })
        .collect())
}

/// Nearest-in-time pose pairs, dropped when nothing lands within `tolerance` seconds.
fn time_matched(source: &[StampedPose], target: &[StampedPose], tolerance: f64) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let mut pairs = (Vec::new(), Vec::new());
    for pose in source {
        let other = nearest(target, pose.stamp);
        if (other.stamp - pose.stamp).abs() <= tolerance {
            pairs.0.push(pose.pose.translation);
            pairs.1.push(other.pose.translation);
        }
    }
    pairs
}

/// Horn's absolute orientation: the rigid transform carrying `source` onto `target`.
fn align_rigid(source: &[[f64; 3]], target: &[[f64; 3]]) -> Pose {
    let mean = |points: &[[f64; 3]]| -> [f64; 3] {
        let mut sum = [0.0; 3];
        for point in points {
            for axis in 0..3 {
                sum[axis] += point[axis] / points.len() as f64;
            }
        }
        sum
    };
    let source_centre = mean(source);
    let target_centre = mean(target);
    let mut cross = [[0.0; 3]; 3];
    for (from, to) in source.iter().zip(target) {
        for (row, cross_row) in cross.iter_mut().enumerate() {
            for (column, cell) in cross_row.iter_mut().enumerate() {
                *cell += (from[row] - source_centre[row]) * (to[column] - target_centre[column]);
            }
        }
    }
    let [[xx, xy, xz], [yx, yy, yz], [zx, zy, zz]] = cross;
    let symmetric = [
        [xx + yy + zz, yz - zy, zx - xz, xy - yx],
        [yz - zy, xx - yy - zz, xy + yx, zx + xz],
        [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
        [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
    ];
    // Shifted power iteration: the shift makes the wanted (largest algebraic)
    // eigenvalue also the largest in magnitude, which is what iteration finds.
    let shift = 3.0 * symmetric.iter().flatten().fold(1.0f64, |peak, value| peak.max(value.abs()));
    let mut vector = [1.0, 0.0, 0.0, 0.0];
    for _ in 0..400 {
        let mut next = [0.0; 4];
        for (row, value) in next.iter_mut().enumerate() {
            *value = (0..4).map(|column| symmetric[row][column] * vector[column]).sum::<f64>()
                + shift * vector[row];
        }
        let norm = next.iter().map(|value| value * value).sum::<f64>().sqrt();
        vector = next.map(|value| value / norm);
    }
    let [qw, qx, qy, qz] = vector;
    let rotation = Pose::new([0.0; 3], [qx, qy, qz, qw]);
    let rotated = rotation.apply(source_centre);
    Pose::new(
        [
            target_centre[0] - rotated[0],
            target_centre[1] - rotated[1],
            target_centre[2] - rotated[2],
        ],
        rotation.rotation,
    )
}

/// Turbo-ish ramp: blue at the start of the run, red at the end.
fn path_colour(position: f64) -> [u8; 3] {
    const STOPS: [(f64, [f64; 3]); 5] = [
        (0.0, [64.0, 110.0, 255.0]),
        (0.25, [0.0, 220.0, 220.0]),
        (0.5, [90.0, 235.0, 90.0]),
        (0.75, [255.0, 205.0, 60.0]),
        (1.0, [255.0, 70.0, 70.0]),
    ];
    for pair in STOPS.windows(2) {
        let (start, from) = pair[0];
        let (end, to) = pair[1];
        if position <= end {
            let fraction = (position - start) / (end - start);
            return std::array::from_fn(|channel| {
                (from[channel] + (to[channel] - from[channel]) * fraction).round() as u8
            });
        }
    }
    [255, 70, 70]
}

fn parse_extent(text: &str) -> Result<[f64; 4]> {
    let values: Vec<f64> = text
        .split([',', '_'])
        .map(|part| part.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .with_context(|| format!("extent {text:?} is not four numbers"))?;
    values
        .try_into()
        .map_err(|_| anyhow::anyhow!("extent {text:?} needs exactly minX_minY_maxX_maxY"))
}

/// Draws `rgb` (a `width * height * 3` buffer) as an 8-bit truecolour PNG.
fn write_png(path: &Path, width: usize, height: usize, rgb: &[u8]) -> Result<()> {
    let file = std::fs::File::create(path).with_context(|| format!("could not create {}", path.display()))?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), width as u32, height as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgb)?;
    Ok(())
}

pub fn run(options: &Options) -> Result<()> {
    let target = options.output.clone().unwrap_or_else(|| {
        let stem = options.recording.file_stem().unwrap_or_default().to_string_lossy();
        options.recording.with_file_name(format!("{stem}_heatmap.png"))
    });
    let recording = Recording::open(&options.recording)?;

    let mut odom = read_poses(&recording, &options.odom)?;

    // The two SLAM systems have unrelated world origins, so nothing can be
    // compared until one trajectory is carried onto the other's frame.
    let mut alignment = Pose::IDENTITY;
    if let Some(reference) = &options.align_to {
        let (source, target_points) = time_matched(&odom, &read_poses(&recording, reference)?, 0.05);
        if source.len() < 3 {
            bail!("only {} poses matched {reference} in time", source.len());
        }
        alignment = align_rigid(&source, &target_points);
        let residual = (source
            .iter()
            .zip(&target_points)
            .map(|(point, expected)| {
                let moved = alignment.apply(*point);
                (0..3).map(|axis| (moved[axis] - expected[axis]).powi(2)).sum::<f64>()
            })
            .sum::<f64>()
            / source.len() as f64)
            .sqrt();
        println!(
            "heatmap: aligned {} poses onto {reference}, fit rmse {residual:.3} m",
            source.len()
        );
    }
    for stamped in odom.iter_mut() {
        stamped.pose = alignment.then(&stamped.pose);
    }

    // A cloud whose frame is not the odometry body frame (an RTAB-Map keyframe
    // cloud sits in the camera optical frame) needs the whole tf chain, not a pose.
    let tf = match &options.tf {
        Some(topic) => Some(TfHistory::read(&recording, topic)?),
        None => None,
    };
    let mut root_counts: BTreeMap<String, usize> = BTreeMap::new();
    let reference = match &tf {
        Some(tf) => tf
            .dynamic
            .values()
            .flatten()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), sample| {
                (low.min(sample.stamp), high.max(sample.stamp))
            }),
        None => (odom[0].stamp, odom[odom.len() - 1].stamp),
    };
    let mut clock = ScanClock {
        by_log_time: options.by_log_time,
        reference,
        warned: false,
    };

    let cloud_channel = recording.channel(&options.cloud)?;
    if schema_name(&cloud_channel) != crate::msgs::POINT_CLOUD2_TYPE {
        bail!(
            "{} is {}, not {}",
            cloud_channel.topic,
            schema_name(&cloud_channel),
            crate::msgs::POINT_CLOUD2_TYPE
        );
    }
    let scan_count = recording.message_count(cloud_channel.id);
    let mut world_points: Vec<[f32; 3]> = Vec::new();
    let mut scans_placed = 0usize;
    let mut scans_seen = 0usize;
    let stride = options.stride.max(1);
    for message in recording.messages(cloud_channel.id, None)? {
        let message = message?;
        scans_seen += 1;
        // Skipping before decoding is what makes an hour-long recording fit.
        if !(scans_seen - 1).is_multiple_of(stride) {
            continue;
        }
        let cloud = cdr::decode_point_cloud2(&message.data)
            .with_context(|| format!("message {} on {}", message.sequence, cloud_channel.topic))?;
        let stamp = clock.stamp_for(&cloud.header, message.log_time);
        let placement = match &tf {
            Some(tf) => {
                let (pose, root) = tf.chain_to_root(&cloud.header.frame_id, stamp);
                *root_counts.entry(root).or_default() += 1;
                alignment.then(&pose)
            }
            None => nearest(&odom, stamp).pose,
        };
        for point in cloud_points(&cloud)? {
            let moved = placement.apply(point.map(f64::from));
            world_points.push(moved.map(|value| value as f32));
        }
        scans_placed += 1;
    }

    // Every scan should reach the same root. More than one means the tf tree is
    // broken somewhere, and those scans are drawn short of the world frame.
    let mut roots: Vec<(&String, &usize)> = root_counts.iter().collect();
    roots.sort_by(|left, right| right.1.cmp(left.1));
    if !roots.is_empty() {
        let listing: Vec<String> = roots.iter().map(|(frame, count)| format!("{frame} x{count}")).collect();
        eprintln!("heatmap: tf roots {}", listing.join(", "));
    }
    if roots.len() > 1 {
        let stranded: Vec<&str> = roots[1..].iter().map(|(frame, _)| frame.as_str()).collect();
        eprintln!(
            "heatmap: tf tree is disconnected — scans under {} are misplaced",
            stranded.join(", ")
        );
    }

    // Report the z distribution, because "chop above 2 m" is unanswerable
    // without knowing where this recording's floor actually sits.
    let mut heights: Vec<f32> = world_points.iter().map(|point| point[2]).collect();
    heights.sort_by(f32::total_cmp);
    let percentile = |fraction: f64| -> f32 {
        let index = ((heights.len() as f64 * fraction) as usize).min(heights.len().saturating_sub(1));
        heights.get(index).copied().unwrap_or(0.0)
    };
    println!(
        "heatmap: world z  min {:.2}  p2 {:.2}  median {:.2}  p98 {:.2}  max {:.2} m",
        percentile(0.0),
        percentile(0.02),
        percentile(0.5),
        percentile(0.98),
        percentile(1.0)
    );

    let low = options.min_height.unwrap_or(f64::NEG_INFINITY) as f32;
    let high = options.max_height.unwrap_or(f64::INFINITY) as f32;
    let kept: Vec<[f32; 2]> = world_points
        .iter()
        .filter(|point| point[2] >= low && point[2] <= high)
        .map(|point| [point[0], point[1]])
        .collect();
    drop(world_points);

    let mut extent = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut note = |x: f64, y: f64| {
        extent[0] = extent[0].min(x);
        extent[1] = extent[1].min(y);
        extent[2] = extent[2].max(x);
        extent[3] = extent[3].max(y);
    };
    for point in &kept {
        note(point[0] as f64, point[1] as f64);
    }
    for stamped in &odom {
        note(stamped.pose.translation[0], stamped.pose.translation[1]);
    }
    let margin = 1.0;
    let [mut min_x, mut min_y, mut max_x, mut max_y] =
        [extent[0] - margin, extent[1] - margin, extent[2] + margin, extent[3] + margin];
    if let Some(forced) = &options.extent {
        [min_x, min_y, max_x, max_y] = parse_extent(forced)?;
    }
    println!("heatmap: extent {min_x:.2},{min_y:.2},{max_x:.2},{max_y:.2}");
    let width = options.width.max(1);
    let scale = width as f64 / (max_x - min_x);
    let height = (((max_y - min_y) * scale).round() as usize).max(1);
    let to_pixel = |x: f64, y: f64| -> (i64, i64) {
        (
            ((x - min_x) * scale).round() as i64,
            height as i64 - 1 - ((y - min_y) * scale).round() as i64,
        )
    };

    // Accumulate hits per pixel, then map density through a log ramp: a single
    // stray return should stay dim while a wall seen a thousand times is bright.
    let mut density = vec![0.0f32; width * height];
    for point in &kept {
        let (px, py) = to_pixel(point[0] as f64, point[1] as f64);
        if px >= 0 && (px as usize) < width && py >= 0 && (py as usize) < height {
            density[py as usize * width + px as usize] += 1.0;
        }
    }
    let peak = density.iter().copied().fold(0.0f32, f32::max);
    let ceiling = (peak * 0.25).max(1.0);

    let mut rgb = vec![0u8; width * height * 3];
    for (pixel, hits) in density.iter().enumerate() {
        let colour = if *hits > 0.0 {
            let position = (hits.ln_1p() / ceiling.ln_1p()).min(1.0);
            let level = (30.0 + position * 205.0).round();
            [level as u8, level as u8, (level * 0.95 + 12.0).round() as u8]
        } else {
            [12, 14, 18]
        };
        rgb[pixel * 3..pixel * 3 + 3].copy_from_slice(&colour);
    }

    // Path last, so it is never buried by the cloud, and drawn as joined
    // segments: at 30 Hz the poses are far enough apart to read as dots.
    let mut plot = |x: i64, y: i64, colour: [u8; 3], radius: i64| {
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                let (px, py) = (x + dx, y + dy);
                if px >= 0 && (px as usize) < width && py >= 0 && (py as usize) < height {
                    let at = (py as usize * width + px as usize) * 3;
                    rgb[at..at + 3].copy_from_slice(&colour);
                }
            }
        }
    };
    let pixel_of = |stamped: &StampedPose| to_pixel(stamped.pose.translation[0], stamped.pose.translation[1]);
    let mut previous = pixel_of(&odom[0]);
    for (index, stamped) in odom.iter().enumerate().skip(1) {
        let current = pixel_of(stamped);
        let colour = path_colour(index as f64 / (odom.len() - 1) as f64);
        let steps = (current.0 - previous.0).abs().max((current.1 - previous.1).abs()).max(1);
        for step in 0..=steps {
            let x = previous.0 + ((current.0 - previous.0) * step) / steps;
            let y = previous.1 + ((current.1 - previous.1) * step) / steps;
            plot(x, y, colour, 1);
        }
        previous = current;
    }
    let first = pixel_of(&odom[0]);
    let last = pixel_of(&odom[odom.len() - 1]);
    plot(first.0, first.1, [255, 255, 255], 4);
    plot(first.0, first.1, [64, 110, 255], 3);
    plot(last.0, last.1, [255, 255, 255], 4);
    plot(last.0, last.1, [255, 70, 70], 3);

    write_png(&target, width, height, &rgb)?;
    let span = ((max_x - min_x - 2.0 * margin).powi(2) + (max_y - min_y - 2.0 * margin).powi(2)).sqrt();
    println!(
        "heatmap: {} poses, {scans_placed} of {} scans, {} points drawn",
        odom.len(),
        scan_count.map_or(scans_seen as u64, |count| count.max(scans_seen as u64)),
        kept.len()
    );
    println!(
        "heatmap: {:.1} x {:.1} m, diagonal {span:.1} m",
        max_x - min_x,
        max_y - min_y
    );
    println!("heatmap: blue = start, red = end -> {}", target.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr::Encoded;
    use crate::msgs::{PointCloud2, PointField, TransformStamped};
    use crate::topics::test_support::{scratch, stamped, write_recording};
    use std::f64::consts::FRAC_PI_2;

    const SECOND: u64 = 1_000_000_000;

    fn cloud_at(stamp: u64, frame: &str, points: &[[f32; 3]]) -> Encoded {
        let mut data = Vec::new();
        for point in points {
            for value in point {
                data.extend_from_slice(&value.to_le_bytes());
            }
            data.extend_from_slice(&[0; 4]); // padding to a 16-byte point
        }
        cdr::point_cloud2(&PointCloud2 {
            header: Header::new(stamp, frame),
            height: 1,
            width: points.len() as u32,
            fields: ["x", "y", "z"]
                .iter()
                .enumerate()
                .map(|(index, name)| PointField {
                    name: name.to_string(),
                    offset: index as u32 * 4,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                })
                .collect(),
            is_bigendian: false,
            point_step: 16,
            row_step: 16 * points.len() as u32,
            data,
            is_dense: true,
        })
    }

    fn odometry_at(stamp: u64, position: [f64; 3], yaw: f64) -> Encoded {
        cdr::odometry(
            &cdr::Pose {
                header: Header::new(stamp, "odom"),
                position,
                orientation: crate::msgs::quaternion_from_rpy(0.0, 0.0, yaw),
            },
            "base_link",
        )
    }

    fn pixel(png_path: &Path, x: usize, y: usize) -> ([u8; 3], (u32, u32)) {
        let decoder = png::Decoder::new(std::io::BufReader::new(std::fs::File::open(png_path).unwrap()));
        let mut reader = decoder.read_info().unwrap();
        let mut pixels = vec![0u8; reader.output_buffer_size().unwrap()];
        let info = reader.next_frame(&mut pixels).unwrap();
        let at = (y * info.width as usize + x) * 3;
        (pixels[at..at + 3].try_into().unwrap(), (info.width, info.height))
    }

    fn options(recording: &Path, output: &Path) -> Options {
        Options {
            recording: recording.to_path_buf(),
            output: Some(output.to_path_buf()),
            width: 300,
            cloud: "livox/lidar".into(),
            odom: "/odom".into(),
            tf: None,
            align_to: None,
            extent: None,
            min_height: None,
            max_height: None,
            stride: 1,
            by_log_time: false,
        }
    }

    /// The lidar's header clock sits 2005 s behind the clock the odometry was
    /// stamped on, as in the grocery recording. The log time is the one clock
    /// both share, so the scan still lands where the pose at its log time is.
    #[test]
    fn a_scan_whose_header_clock_is_off_is_placed_by_its_log_time() {
        let directory = scratch("heatmap_clock");
        let recording = directory.join("clock.mcap");
        let skew = 2005 * SECOND;
        write_recording(
            &recording,
            &[
                vec![stamped("/livox/lidar", cloud_at(3000 * SECOND - skew, "livox_frame", &[[1.0, 0.0, 0.0]]), 3000 * SECOND)],
                vec![
                    stamped("/odom", odometry_at(3000 * SECOND, [5.0, 0.0, 0.0], FRAC_PI_2), 3000 * SECOND),
                    stamped("/odom", odometry_at(3001 * SECOND, [6.0, 0.0, 0.0], 0.0), 3001 * SECOND),
                ],
            ],
        );
        let output = directory.join("clock.png");
        run(&options(&recording, &output)).unwrap();
        assert_eq!(pixel(&output, 100, 99).0, [235, 235, 235]);

        let mut forced = options(&recording, &output);
        forced.by_log_time = true;
        run(&forced).unwrap();
        assert_eq!(pixel(&output, 100, 99).0, [235, 235, 235]);
    }

    /// The pose sits at (5, 0) turned a quarter turn left, so a point one metre
    /// ahead of the sensor lands at world (5, 1). The odometry chunk is written
    /// after the cloud's, the way the recorder appends it.
    #[test]
    fn a_scan_lands_where_the_odometry_pose_at_its_stamp_puts_it() {
        let directory = scratch("heatmap_odom");
        let recording = directory.join("odom.mcap");
        write_recording(
            &recording,
            &[
                vec![stamped("/livox/lidar", cloud_at(10 * SECOND, "livox_frame", &[[1.0, 0.0, 0.0]]), 10 * SECOND)],
                vec![
                    // Stamped out of order on purpose: the nearest pose is found by stamp.
                    stamped("/odom", odometry_at(11 * SECOND, [6.0, 0.0, 0.0], 0.0), 11 * SECOND),
                    stamped("/odom", odometry_at(10 * SECOND, [5.0, 0.0, 0.0], FRAC_PI_2), 10 * SECOND),
                ],
            ],
        );
        let output = directory.join("odom.png");
        run(&options(&recording, &output)).unwrap();

        // Extent is x 4..7, y -1..2 with the 1 m margin, so 100 px per metre.
        let (colour, size) = pixel(&output, 100, 99);
        assert_eq!(size, (300, 300));
        assert_eq!(colour, [235, 235, 235], "the point should be a bright hit");
        assert_eq!(pixel(&output, 200, 99).0, [12, 14, 18], "nothing else is on that row");
        // Path start at (5, 0) is a blue disc inside a white ring.
        assert_eq!(pixel(&output, 100, 199).0, [64, 110, 255]);
        assert_eq!(pixel(&output, 104, 199).0, [255, 255, 255]);
    }

    /// With `--tf` the scan's frame is walked to the root: a static mount edge
    /// from /tf_static and a moving odom->base_link edge from /tf, with the
    /// cloud stamped halfway between two tf samples.
    #[test]
    fn a_scan_is_placed_along_the_tf_chain_interpolated_at_its_stamp() {
        let directory = scratch("heatmap_tf");
        let recording = directory.join("tf.mcap");
        let moving = |stamp: u64, x: f64| {
            stamped(
                "/tf",
                cdr::tf_message(&[TransformStamped {
                    header: Header::new(stamp, "odom"),
                    child_frame_id: "base_link".into(),
                    translation: [x, 0.0, 0.0],
                    rotation: [0.0, 0.0, 0.0, 1.0],
                }]),
                stamp,
            )
        };
        let mount = TransformStamped {
            header: Header::new(0, "base_link"),
            child_frame_id: "livox_frame".into(),
            translation: [0.0, 1.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        };
        write_recording(
            &recording,
            &[
                vec![stamped("/tf_static", cdr::tf_message(&[mount]), 0)],
                vec![stamped("/livox/lidar", cloud_at(11 * SECOND, "livox_frame", &[[1.0, 0.0, 0.0]]), 11 * SECOND)],
                vec![moving(10 * SECOND, 4.0), moving(12 * SECOND, 6.0)],
                vec![
                    stamped("/odom", odometry_at(10 * SECOND, [4.0, 0.0, 0.0], 0.0), 10 * SECOND),
                    stamped("/odom", odometry_at(12 * SECOND, [6.0, 0.0, 0.0], 0.0), 12 * SECOND),
                ],
            ],
        );
        let output = directory.join("tf.png");
        let mut options = options(&recording, &output);
        options.tf = Some("tf".into());
        run(&options).unwrap();

        // base_link is at x=5 at 11 s; the mount adds y=1; the point adds x=1:
        // world (6, 1). Extent x 3..7, y -1..2 => 75 px per metre, 225 px tall.
        let (colour, size) = pixel(&output, 225, 74);
        assert_eq!(size, (300, 225));
        assert_eq!(colour, [235, 235, 235]);
    }

    #[test]
    fn a_missing_odometry_topic_is_named_in_the_error() {
        let directory = scratch("heatmap_missing");
        let recording = directory.join("missing.mcap");
        write_recording(
            &recording,
            &[vec![stamped("/livox/lidar", cloud_at(SECOND, "livox_frame", &[[1.0, 0.0, 0.0]]), SECOND)]],
        );
        let mut options = options(&recording, &directory.join("missing.png"));
        options.odom = "/nonexistent".into();
        let error = run(&options).unwrap_err().to_string();
        assert!(error.contains("no topic /nonexistent"), "{error}");
        assert!(error.contains("/livox/lidar"), "{error}");
    }

    #[test]
    fn a_pose_stamped_stream_serves_as_odometry() {
        let directory = scratch("heatmap_pose_stamped");
        let recording = directory.join("pose.mcap");
        let pose = |stamp: u64, x: f64| {
            stamped(
                "/pose",
                cdr::pose_stamped(&cdr::Pose {
                    header: Header::new(stamp, "map"),
                    position: [x, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                }),
                stamp,
            )
        };
        write_recording(
            &recording,
            &[vec![
                stamped("/livox/lidar", cloud_at(SECOND, "livox_frame", &[[1.0, 0.0, 0.0]]), SECOND),
                pose(SECOND, 0.0),
                pose(2 * SECOND, 1.0),
            ]],
        );
        let mut options = options(&recording, &directory.join("pose.png"));
        options.odom = "pose".into();
        run(&options).unwrap();
        assert!(directory.join("pose.png").exists());
    }

    #[test]
    fn alignment_recovers_a_known_rigid_motion() {
        let motion = Pose::new([1.0, -2.0, 0.5], crate::msgs::quaternion_from_rpy(0.0, 0.0, 0.7));
        let source: Vec<[f64; 3]> = (0..12)
            .map(|index| [index as f64, (index * index) as f64 * 0.1, (index % 3) as f64])
            .collect();
        let target: Vec<[f64; 3]> = source.iter().map(|point| motion.apply(*point)).collect();
        let fitted = align_rigid(&source, &target);
        for (point, expected) in source.iter().zip(&target) {
            let moved = fitted.apply(*point);
            for axis in 0..3 {
                // Power iteration converges to about a micron here; the
                // alignment is only ever used to overlay two trajectories.
                assert!((moved[axis] - expected[axis]).abs() < 1e-4, "{moved:?} vs {expected:?}");
            }
        }
    }

    #[test]
    fn the_path_ramp_runs_blue_to_red() {
        assert_eq!(path_colour(0.0), [64, 110, 255]);
        assert_eq!(path_colour(0.5), [90, 235, 90]);
        assert_eq!(path_colour(1.0), [255, 70, 70]);
    }

    #[test]
    fn an_extent_accepts_commas_or_underscores() {
        assert_eq!(parse_extent("-1_2_3.5_4").unwrap(), [-1.0, 2.0, 3.5, 4.0]);
        assert_eq!(parse_extent("0,0,10,10").unwrap(), [0.0, 0.0, 10.0, 10.0]);
        assert!(parse_extent("1,2,3").is_err());
    }
}
