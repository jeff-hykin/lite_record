//! Completing a recording's frame tree after the fact.
//!
//! A recording carries the transforms its sensors knew about themselves — the
//! imagers inside a camera body, the lidar and its IMU — as separate little
//! trees with nothing joining them, because only the rig's URDF knows how the
//! sensors are mounted. `tf_fixup` reads those edges, corrects the ones an
//! older recorder wrote backwards, adds the URDF's joints, and appends the
//! completed set to `/tf` the way dimos publishes static transforms: repeated at
//! 5 Hz across the whole recording, since dimos has no latched tf topic.
//!
//! Nothing is rewritten. The original `/tf_static` stays where it was; the
//! appended `/tf` carries the same edges corrected, stamped from the same
//! instant onward, and a tf consumer that keeps a history per edge takes the
//! later, correct value. Running twice appends nothing the second time.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::{Context, Result};

use crate::mcap_append::Appender;
use crate::msgs::{TransformStamped, NANOS_PER_SEC, TF_TYPE};
use crate::record::{
    channel_metadata, STATIC_TRANSFORM_HZ, TF_TOPIC, TRANSFORM_CONVENTION_KEY,
    TRANSFORM_CONVENTION_VALUE,
};
use crate::tf::{Pose, StaticTree, TreeProblem};

/// Frames whose edges on `/tf` are moving, not mounts: an edge out of one of
/// these is odometry and must not be mistaken for a static transform.
const MOVING_PARENTS: [&str; 2] = ["odom", "map"];

/// What a recording says about its own frames, read from its index and the
/// first message of each channel rather than a full pass.
#[derive(Debug, Default)]
pub struct Recording {
    pub start_nanos: u64,
    pub end_nanos: u64,
    /// The static edges the file already carries, in tf semantics — an
    /// unmarked `/tf_static` has been inverted on the way in.
    pub tree: StaticTree,
    /// `(parent, child)` pairs already published on `/tf`, which a fixup need
    /// not publish again.
    pub published: BTreeSet<(String, String)>,
    /// Whether the `/tf_static` edges had to be inverted, i.e. were written by a
    /// recorder that copied the SDK extrinsics through unchanged.
    pub inverted_tf_static: bool,
    /// The frame each data topic is stamped in.
    pub frame_of_topic: BTreeMap<String, String>,
}

impl Recording {
    /// Every frame that data is stamped in, once each. The moving frames are
    /// left out: odometry stamped in `odom` is placed by its own edge, not by
    /// the static tree.
    pub fn data_frames(&self) -> Vec<String> {
        let unique: BTreeSet<&String> = self
            .frame_of_topic
            .values()
            .filter(|frame| !MOVING_PARENTS.contains(&frame.as_str()))
            .collect();
        unique.into_iter().cloned().collect()
    }
}

pub fn inspect(mapped: &[u8]) -> Result<Recording> {
    let summary = mcap::Summary::read(mapped)?.context("the recording has no summary section")?;
    let stats = summary.stats.as_ref().context("the recording has no statistics record")?;
    let mut recording = Recording {
        start_nanos: stats.message_start_time,
        end_nanos: stats.message_end_time,
        ..Recording::default()
    };

    let tf_static_channel = summary
        .channels
        .values()
        .find(|channel| channel.topic == "/tf_static")
        .map(|channel| (channel.id, channel.metadata.contains_key(TRANSFORM_CONVENTION_KEY)));
    let tf_channel = summary.channels.values().find(|channel| channel.topic == TF_TOPIC).map(|channel| channel.id);

    let mut chunks = summary.chunk_indexes.clone();
    chunks.sort_by_key(|chunk| chunk.chunk_start_offset);

    // One header per data channel, from the first chunk that lists it.
    let mut unresolved: BTreeSet<u16> = summary
        .channels
        .values()
        .filter(|channel| {
            channel.schema.as_ref().is_some_and(|schema| schema.name != TF_TYPE)
        })
        .map(|channel| channel.id)
        .collect();
    let mut tf_static_edges: Vec<TransformStamped> = Vec::new();
    let mut tf_samples: Vec<Vec<TransformStamped>> = Vec::new();
    let tf_chunks: Vec<usize> = chunks
        .iter()
        .enumerate()
        .filter(|(_, chunk)| tf_channel.is_some_and(|id| chunk.message_index_offsets.contains_key(&id)))
        .map(|(index, _)| index)
        .collect();
    // Static edges repeat identically, so the first and last chunk carrying
    // `/tf` between them show every edge ever published on it — including
    // those an earlier fixup appended at the end of the file.
    let tf_chunks_to_read: BTreeSet<usize> = tf_chunks.first().into_iter().chain(tf_chunks.last()).copied().collect();

    for (index, chunk) in chunks.iter().enumerate() {
        let wants_headers = chunk.message_index_offsets.keys().any(|id| unresolved.contains(id));
        let wants_tf_static = tf_static_channel.is_some_and(|(id, _)| chunk.message_index_offsets.contains_key(&id));
        let wants_tf = tf_chunks_to_read.contains(&index);
        if !wants_headers && !wants_tf_static && !wants_tf {
            continue;
        }
        let mut took_tf_sample = false;
        for message in summary.stream_chunk(mapped, chunk)? {
            let message = message?;
            let id = message.channel.id;
            if tf_static_channel.is_some_and(|(static_id, _)| static_id == id) {
                if let Ok(transforms) = crate::cdr::decode_tf_message(&message.data) {
                    tf_static_edges.extend(transforms);
                }
            } else if tf_channel == Some(id) {
                if wants_tf && !took_tf_sample {
                    if let Ok(transforms) = crate::cdr::decode_tf_message(&message.data) {
                        tf_samples.push(transforms);
                        took_tf_sample = true;
                    }
                }
            } else if unresolved.remove(&id) {
                if let Some(header) = crate::cdr::decode_header(&message.data) {
                    recording
                        .frame_of_topic
                        .insert(message.channel.topic.clone(), header.frame_id);
                }
            }
        }
        if unresolved.is_empty() && !tf_static_edges.is_empty() && tf_chunks_to_read.iter().all(|read| *read <= index) {
            break;
        }
    }

    if let Some((_, marked)) = tf_static_channel {
        recording.inverted_tf_static = !marked && !tf_static_edges.is_empty();
        for edge in &tf_static_edges {
            let pose = Pose::from_transform(edge);
            let pose = if marked { pose } else { pose.inverse() };
            recording.tree.insert(edge.parent(), &edge.child_frame_id, pose);
        }
    }
    for transforms in &tf_samples {
        for edge in transforms {
            recording
                .published
                .insert((edge.parent().to_string(), edge.child_frame_id.clone()));
            if !MOVING_PARENTS.contains(&edge.parent()) {
                recording
                    .tree
                    .insert(edge.parent(), &edge.child_frame_id, Pose::from_transform(edge));
            }
        }
    }
    Ok(recording)
}

/// The completed tree and what it took to complete it.
#[derive(Debug)]
pub struct Plan {
    pub tree: StaticTree,
    /// Edges `/tf` does not carry yet, to be appended.
    pub new_edges: Vec<(String, String, Pose)>,
    /// Recorded edges a URDF joint replaced, named so the operator can see it.
    pub overridden: Vec<String>,
    pub problems: Vec<TreeProblem>,
    /// Things that are not wrong with the tree but will bite: chiefly odometry
    /// already appended for a frame the URDF has since put under another.
    pub warnings: Vec<String>,
}

/// Merges the URDF's joints over the recorded edges. A joint whose child the
/// recording already places wins — the URDF is the operator's statement of
/// how the rig is built, and the recorded edge for a mount is at best an
/// identity placeholder — but every such replacement is reported.
pub fn plan(recording: &Recording, urdf: Option<&crate::urdf::Urdf>) -> Plan {
    let mut tree = recording.tree.clone();
    let mut overridden = Vec::new();
    if let Some(urdf) = urdf {
        for joint in &urdf.joints {
            let pose = Pose::new(
                joint.translation,
                crate::msgs::quaternion_from_rpy(
                    joint.rotation_rpy[0],
                    joint.rotation_rpy[1],
                    joint.rotation_rpy[2],
                ),
            );
            if let Some(previous) = tree.parent_of(&joint.child) {
                if previous != joint.parent || tree.pose_in(previous, &joint.child) != Some(pose) {
                    overridden.push(format!(
                        "{} <- {} (was under {previous})",
                        joint.child, joint.parent
                    ));
                }
            }
            tree.insert(&joint.parent, &joint.child, pose);
        }
    }
    let new_edges = tree
        .edges()
        .filter(|(parent, child, _)| {
            !recording
                .published
                .contains(&((*parent).to_string(), (*child).to_string()))
        })
        .map(|(parent, child, pose)| (parent.to_string(), child.to_string(), *pose))
        .collect();
    let problems = tree.problems(&recording.data_frames());
    // Odometry is appended as `odom -> <root>`. A URDF applied afterwards can
    // put that frame under a new root, and then it has two parents: the
    // odometry cannot be moved, so say so rather than let a tree that looks
    // connected hide it.
    let warnings = recording
        .published
        .iter()
        .filter(|(parent, _)| MOVING_PARENTS.contains(&parent.as_str()))
        .filter_map(|(parent, child)| {
            tree.parent_of(child).map(|new_parent| {
                format!(
                    "{child} already carries {parent} -> {child} odometry but is now under {new_parent}; \
                     the odometry was estimated before this urdf and describes the wrong frame — \
                     re-run post_process on a copy taken before the odometry was added"
                )
            })
        })
        .collect();
    Plan {
        tree,
        new_edges,
        overridden,
        problems,
        warnings,
    }
}

/// Appends `edges` to `/tf` at 5 Hz from `start_nanos` to `end_nanos`, each
/// copy re-stamped, the way dimos' StaticTfPublisher would have published them
/// live. Returns how many messages that was.
pub fn append_static_transforms(
    appender: &mut Appender,
    edges: &[(String, String, Pose)],
    start_nanos: u64,
    end_nanos: u64,
) -> Result<u64> {
    if edges.is_empty() {
        return Ok(0);
    }
    let encoded = crate::cdr::tf_message(&[]);
    let schema = appender.schema(encoded.schema_name, "ros2msg", encoded.schema_text.as_bytes());
    let channel = appender.channel(TF_TOPIC, schema, "cdr", &channel_metadata(TF_TOPIC));

    let period = (NANOS_PER_SEC as f64 / STATIC_TRANSFORM_HZ) as u64;
    let mut stamp = start_nanos;
    let mut written = 0;
    while stamp <= end_nanos.max(start_nanos) {
        let transforms: Vec<TransformStamped> = edges
            .iter()
            .map(|(parent, child, pose)| pose.stamped(stamp, parent, child))
            .collect();
        appender.write(channel, stamp, crate::cdr::tf_message(&transforms).data)?;
        written += 1;
        stamp += period;
    }
    Ok(written)
}

/// The whole `tf_fixup` command: inspect, plan, append, report. Returns the
/// plan so the caller can print the tree and decide on an exit code.
pub fn fix(recording_path: &Path, urdf: Option<&crate::urdf::Urdf>) -> Result<(Plan, u64)> {
    let file = std::fs::File::open(recording_path)
        .with_context(|| format!("could not open {}", recording_path.display()))?;
    let mapped = unsafe { memmap2::Mmap::map(&file)? };
    let recording = inspect(&mapped)?;
    drop(mapped);
    let plan = plan(&recording, urdf);
    if plan.new_edges.is_empty() {
        return Ok((plan, 0));
    }
    let mut appender = Appender::open(recording_path)?;
    let written =
        append_static_transforms(&mut appender, &plan.new_edges, recording.start_nanos, recording.end_nanos)?;
    appender.finish()?;
    Ok((plan, written))
}

/// The lines `tf_fixup` prints: the tree, then every problem and override.
pub fn describe(plan: &Plan, appended: u64) -> String {
    let mut out = String::new();
    out.push_str("frame tree:\n");
    for line in plan.tree.render().lines() {
        out.push_str("  ");
        out.push_str(line);
        out.push('\n');
    }
    for replaced in &plan.overridden {
        out.push_str(&format!("replaced by the urdf: {replaced}\n"));
    }
    for problem in &plan.problems {
        out.push_str(&format!("problem: {problem}\n"));
    }
    for warning in &plan.warnings {
        out.push_str(&format!("warning: {warning}\n"));
    }
    out.push_str(&format!(
        "appended {appended} /tf messages carrying {} new edge(s)\n",
        plan.new_edges.len()
    ));
    if plan.problems.is_empty() {
        out.push_str("the tree is connected and places every data frame\n");
    }
    out
}

/// Whether `metadata` says the channel's edges are already tf poses.
pub fn is_marked(metadata: &BTreeMap<String, String>) -> bool {
    metadata.get(TRANSFORM_CONVENTION_KEY).map(String::as_str) == Some(TRANSFORM_CONVENTION_VALUE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::Header;
    use std::collections::BTreeMap;
    use std::io::BufWriter;

    fn scratch(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("lite_record_fixup_{name}_{}.mcap", crate::record::now_nanos()))
    }

    /// A recording the way the old recorder wrote it: one `/tf_static` with the
    /// SDK-direction infra2 edge and two disconnected roots, plus a lidar cloud
    /// header and an image header so data frames are known.
    fn old_style_recording(path: &Path) {
        let file = std::fs::File::create(path).unwrap();
        let mut writer = mcap::WriteOptions::new()
            .compression(Some(mcap::Compression::Zstd))
            .chunk_size(Some(4096))
            .profile("ros2")
            .create(BufWriter::new(file))
            .unwrap();
        let tf = crate::cdr::tf_message(&[
            TransformStamped::identity("camera_link", "camera_depth_optical_frame"),
            TransformStamped {
                header: Header::new(1_000, "camera_depth_optical_frame"),
                child_frame_id: "camera_infra2_optical_frame".into(),
                translation: [-0.09486, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            TransformStamped::identity("livox_link", "livox_frame"),
        ]);
        let tf_schema = writer.add_schema(tf.schema_name, "ros2msg", tf.schema_text.as_bytes()).unwrap();
        let tf_channel = writer.add_channel(tf_schema, "/tf_static", "cdr", &BTreeMap::new()).unwrap();
        writer
            .write_to_known_channel(
                &mcap::records::MessageHeader { channel_id: tf_channel, sequence: 0, log_time: 1_000, publish_time: 1_000 },
                &tf.data,
            )
            .unwrap();
        let imu = crate::cdr::imu(&crate::msgs::Imu::unoriented(Header::new(1_000, "livox_imu_frame"), [0.0; 3], [0.0; 3]));
        let imu_schema = writer.add_schema(imu.schema_name, "ros2msg", imu.schema_text.as_bytes()).unwrap();
        let imu_channel = writer.add_channel(imu_schema, "/livox/imu", "cdr", &BTreeMap::new()).unwrap();
        for index in 0..50u64 {
            let stamp = 1_000 + index * 200_000_000;
            writer
                .write_to_known_channel(
                    &mcap::records::MessageHeader { channel_id: imu_channel, sequence: index as u32, log_time: stamp, publish_time: stamp },
                    &imu.data,
                )
                .unwrap();
        }
        writer.finish().unwrap();
    }

    fn rig_urdf() -> crate::urdf::Urdf {
        crate::urdf::parse(
            r#"<robot name="rig">
                <link name="base_link"/><link name="camera_link"/><link name="livox_link"/>
                <joint name="cam" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="0.1 0 0"/></joint>
                <joint name="lidar" type="fixed"><parent link="base_link"/><child link="livox_link"/><origin xyz="0 0 0.2"/></joint>
            </robot>"#,
        )
        .unwrap()
    }

    #[test]
    fn an_old_recording_has_its_edges_inverted_and_its_roots_reported() {
        let path = scratch("inspect");
        old_style_recording(&path);
        let bytes = std::fs::read(&path).unwrap();
        let recording = inspect(&bytes).unwrap();
        assert!(recording.inverted_tf_static);
        let infra2 = recording.tree.pose_in("camera_depth_optical_frame", "camera_infra2_optical_frame").unwrap();
        assert!((infra2.translation[0] - 0.09486).abs() < 1e-12, "{:?}", infra2.translation);
        assert_eq!(recording.frame_of_topic["/livox/imu"], "livox_imu_frame");
        assert_eq!(recording.start_nanos, 1_000);

        let plan = plan(&recording, None);
        assert_eq!(plan.new_edges.len(), 3);
        assert!(plan.problems.contains(&TreeProblem::MultipleRoots(vec!["camera_link".into(), "livox_link".into()])));
        assert!(plan.problems.contains(&TreeProblem::Unplaced("livox_imu_frame".into())));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_urdf_joins_the_roots_and_the_fixup_is_idempotent() {
        let path = scratch("fix");
        old_style_recording(&path);
        let urdf = rig_urdf();
        let (plan, appended) = fix(&path, Some(&urdf)).unwrap();
        assert_eq!(plan.tree.roots(), vec!["base_link".to_string()]);
        assert_eq!(plan.new_edges.len(), 5);
        // 50 imu messages at 5 Hz span 9.8 s, so 5 Hz of transforms is 50 messages.
        assert_eq!(appended, 50, "{}", describe(&plan, appended));
        assert!(plan.problems.contains(&TreeProblem::Unplaced("livox_imu_frame".into())));
        assert!(!plan.problems.iter().any(|problem| matches!(problem, TreeProblem::MultipleRoots(_))));

        let bytes = std::fs::read(&path).unwrap();
        let recording = inspect(&bytes).unwrap();
        assert_eq!(recording.published.len(), 5);
        assert_eq!(recording.tree.roots(), vec!["base_link".to_string()]);
        let infra2 = recording.tree.pose_in("base_link", "camera_infra2_optical_frame").unwrap();
        assert!((infra2.translation[0] - (0.1 + 0.09486)).abs() < 1e-12, "{:?}", infra2.translation);
        let tf_messages = mcap::MessageStream::new(&bytes)
            .unwrap()
            .filter(|message| message.as_ref().unwrap().channel.topic == TF_TOPIC)
            .count();
        assert_eq!(tf_messages, 50);
        let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
        let tf_channel = summary.channels.values().find(|channel| channel.topic == TF_TOPIC).unwrap();
        assert!(is_marked(&tf_channel.metadata));

        // Again with the same urdf: nothing new to say, nothing appended.
        let (plan, appended) = fix(&path, Some(&urdf)).unwrap();
        assert_eq!(appended, 0);
        assert!(plan.new_edges.is_empty());
        assert_eq!(std::fs::read(&path).unwrap().len(), bytes.len());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn odometry_appended_before_a_urdf_rerooted_its_frame_is_called_out() {
        let path = scratch("reroot");
        old_style_recording(&path);
        // No urdf yet: the lidar's own link is a root, and odometry gets hung off it.
        let (first, _) = fix(&path, None).unwrap();
        assert!(first.tree.roots().contains(&"livox_link".to_string()));
        let mut appender = Appender::open(&path).unwrap();
        let tf = crate::cdr::tf_message(&[]);
        let schema = appender.schema(tf.schema_name, "ros2msg", tf.schema_text.as_bytes());
        let channel = appender.channel(TF_TOPIC, schema, "cdr", &channel_metadata(TF_TOPIC));
        let edge = Pose::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]).stamped(2_000, "odom", "livox_link");
        appender.write(channel, 2_000, crate::cdr::tf_message(&[edge]).data).unwrap();
        appender.finish().unwrap();

        let bytes = std::fs::read(&path).unwrap();
        let recording = inspect(&bytes).unwrap();
        assert!(recording.published.contains(&("odom".to_string(), "livox_link".to_string())));
        assert!(recording.tree.parent_of("livox_link").is_none(), "odometry is not a static edge");
        let plan = plan(&recording, Some(&rig_urdf()));
        assert_eq!(plan.warnings.len(), 1, "{:?}", plan.warnings);
        assert!(plan.warnings[0].contains("livox_link already carries odom -> livox_link odometry"));
        assert!(plan.warnings[0].contains("now under base_link"));
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_urdf_that_moves_a_recorded_edge_is_reported_not_silently_applied() {
        let path = scratch("override");
        old_style_recording(&path);
        let urdf = crate::urdf::parse(
            r#"<robot name="rig"><link name="base_link"/><link name="livox_frame"/>
                <joint name="j" type="fixed"><parent link="base_link"/><child link="livox_frame"/><origin xyz="0 0 1"/></joint>
            </robot>"#,
        )
        .unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let plan = plan(&inspect(&bytes).unwrap(), Some(&urdf));
        assert_eq!(plan.overridden, vec!["livox_frame <- base_link (was under livox_link)".to_string()]);
        assert_eq!(plan.tree.parent_of("livox_frame"), Some("base_link"));
        std::fs::remove_file(&path).ok();
    }
}
