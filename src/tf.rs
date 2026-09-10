//! Rigid transforms and the static frame tree, in ROS tf semantics.
//!
//! A `parent -> child` transform here is the *pose of the child in the parent*:
//! `apply` takes a point expressed in the child frame and returns it in the
//! parent frame. That is the direction every tf consumer assumes, and the
//! opposite of what the camera SDKs hand back — see [`Pose::inverse`] and the
//! sensor backends for where the flip happens.

use std::collections::{BTreeMap, BTreeSet};

use crate::msgs::TransformStamped;

/// A translation plus a unit quaternion `[x, y, z, w]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

impl Pose {
    pub const IDENTITY: Pose = Pose {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };

    pub fn new(translation: [f64; 3], rotation: [f64; 4]) -> Self {
        Pose {
            translation,
            rotation: normalised(rotation),
        }
    }

    /// From a row-major 3x3 rotation matrix and a translation, both describing
    /// the child's pose in the parent frame.
    pub fn from_matrix(rotation: [f64; 9], translation: [f64; 3]) -> Self {
        Pose::new(translation, crate::msgs::quaternion_from_matrix(rotation))
    }

    pub fn from_transform(transform: &TransformStamped) -> Self {
        Pose::new(transform.translation, transform.rotation)
    }

    /// The point `child_point`, expressed in the parent frame.
    pub fn apply(&self, child_point: [f64; 3]) -> [f64; 3] {
        let rotated = rotate(self.rotation, child_point);
        [
            rotated[0] + self.translation[0],
            rotated[1] + self.translation[1],
            rotated[2] + self.translation[2],
        ]
    }

    /// `self` followed by `next`: if `self` is `a -> b` and `next` is `b -> c`,
    /// the result is `a -> c`.
    pub fn then(&self, next: &Pose) -> Pose {
        Pose {
            translation: self.apply(next.translation),
            rotation: normalised(multiply(self.rotation, next.rotation)),
        }
    }

    /// The same rigid relation read the other way: a `parent -> child` pose
    /// becomes `child -> parent`. This is also the conversion from an SDK
    /// extrinsic (which maps points *from* one frame *to* the other) into the
    /// tf pose of the destination frame.
    pub fn inverse(&self) -> Pose {
        let rotation = conjugate(self.rotation);
        let moved = rotate(rotation, self.translation);
        Pose {
            translation: [-moved[0], -moved[1], -moved[2]],
            rotation,
        }
    }

    /// Linear blend of the translations and a spherical blend of the
    /// rotations, `fraction` 0 giving `self` and 1 giving `other`.
    pub fn interpolate(&self, other: &Pose, fraction: f64) -> Pose {
        let fraction = fraction.clamp(0.0, 1.0);
        let translation = std::array::from_fn(|axis| {
            self.translation[axis] + (other.translation[axis] - self.translation[axis]) * fraction
        });
        Pose {
            translation,
            rotation: slerp(self.rotation, other.rotation, fraction),
        }
    }

    /// The pose as a tf edge.
    pub fn stamped(&self, stamp_nanos: u64, parent: &str, child: &str) -> TransformStamped {
        TransformStamped {
            header: crate::msgs::Header::new(stamp_nanos, parent),
            child_frame_id: child.to_string(),
            translation: self.translation,
            rotation: self.rotation,
        }
    }
}

fn normalised(quaternion: [f64; 4]) -> [f64; 4] {
    let norm = quaternion.iter().map(|value| value * value).sum::<f64>().sqrt();
    if norm == 0.0 {
        return [0.0, 0.0, 0.0, 1.0];
    }
    quaternion.map(|value| value / norm)
}

fn conjugate(quaternion: [f64; 4]) -> [f64; 4] {
    [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]
}

/// Hamilton product `a * b`, both `[x, y, z, w]`.
fn multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let (ax, ay, az, aw) = (a[0], a[1], a[2], a[3]);
    let (bx, by, bz, bw) = (b[0], b[1], b[2], b[3]);
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

fn rotate(quaternion: [f64; 4], point: [f64; 3]) -> [f64; 3] {
    // q * (p, 0) * q^-1, with the middle product expanded so no allocation or
    // trig is needed per point.
    let (qx, qy, qz, qw) = (quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    let (px, py, pz) = (point[0], point[1], point[2]);
    let cross_x = qy * pz - qz * py;
    let cross_y = qz * px - qx * pz;
    let cross_z = qx * py - qy * px;
    let cross2_x = qy * cross_z - qz * cross_y;
    let cross2_y = qz * cross_x - qx * cross_z;
    let cross2_z = qx * cross_y - qy * cross_x;
    [
        px + 2.0 * (qw * cross_x + cross2_x),
        py + 2.0 * (qw * cross_y + cross2_y),
        pz + 2.0 * (qw * cross_z + cross2_z),
    ]
}

fn slerp(from: [f64; 4], to: [f64; 4], fraction: f64) -> [f64; 4] {
    let mut dot: f64 = (0..4).map(|index| from[index] * to[index]).sum();
    // The two signs of a quaternion are the same rotation; take the short way.
    let mut to = to;
    if dot < 0.0 {
        dot = -dot;
        to = to.map(|value| -value);
    }
    if dot > 0.9995 {
        return normalised(std::array::from_fn(|index| {
            from[index] + (to[index] - from[index]) * fraction
        }));
    }
    let angle = dot.acos();
    let sin_angle = angle.sin();
    let weight_from = ((1.0 - fraction) * angle).sin() / sin_angle;
    let weight_to = (fraction * angle).sin() / sin_angle;
    std::array::from_fn(|index| from[index] * weight_from + to[index] * weight_to)
}

/// Row-major 3x3 rotation matrix of a quaternion.
pub fn matrix_from_quaternion(quaternion: [f64; 4]) -> [f64; 9] {
    let [x, y, z, w] = normalised(quaternion);
    [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * z + y * w),
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - x * w),
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y),
    ]
}

/// The rigid frame tree: every child has at most one parent, and each edge is
/// the child's pose in its parent.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StaticTree {
    /// child -> (parent, pose of child in parent)
    edges: BTreeMap<String, (String, Pose)>,
}

/// One thing wrong with a frame tree, named so it can be printed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TreeProblem {
    /// More than one frame has no parent, so the two halves cannot be related.
    MultipleRoots(Vec<String>),
    /// Following parents from this frame never reaches a root.
    Cycle(String),
    /// A frame that data is stamped in but no edge places.
    Unplaced(String),
}

impl std::fmt::Display for TreeProblem {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TreeProblem::MultipleRoots(roots) => {
                write!(formatter, "disconnected: {} separate roots ({})", roots.len(), roots.join(", "))
            }
            TreeProblem::Cycle(frame) => write!(formatter, "cycle: {frame} never reaches a root"),
            TreeProblem::Unplaced(frame) => {
                write!(formatter, "unplaced: data is stamped in {frame} but no transform places it")
            }
        }
    }
}

impl StaticTree {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds or replaces the edge that places `child`.
    pub fn insert(&mut self, parent: &str, child: &str, pose: Pose) {
        self.edges.insert(child.to_string(), (parent.to_string(), pose));
    }

    pub fn insert_transform(&mut self, transform: &TransformStamped) {
        self.insert(transform.parent(), &transform.child_frame_id, Pose::from_transform(transform));
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    pub fn contains(&self, frame: &str) -> bool {
        self.edges.contains_key(frame) || self.edges.values().any(|(parent, _)| parent == frame)
    }

    pub fn parent_of(&self, child: &str) -> Option<&str> {
        self.edges.get(child).map(|(parent, _)| parent.as_str())
    }

    pub fn edges(&self) -> impl Iterator<Item = (&str, &str, &Pose)> {
        self.edges
            .iter()
            .map(|(child, (parent, pose))| (parent.as_str(), child.as_str(), pose))
    }

    pub fn frames(&self) -> BTreeSet<String> {
        let mut frames = BTreeSet::new();
        for (child, (parent, _)) in &self.edges {
            frames.insert(child.clone());
            frames.insert(parent.clone());
        }
        frames
    }

    /// Frames that are a parent but never a child.
    pub fn roots(&self) -> Vec<String> {
        self.frames()
            .into_iter()
            .filter(|frame| !self.edges.contains_key(frame))
            .collect()
    }

    /// The top of the tree `frame` belongs to, walking parents until there is
    /// none. `None` when the walk loops.
    pub fn root_of(&self, frame: &str) -> Option<String> {
        let mut cursor = frame.to_string();
        for _ in 0..=self.edges.len() {
            match self.edges.get(&cursor) {
                Some((parent, _)) => cursor = parent.clone(),
                None => return Some(cursor),
            }
        }
        None
    }

    /// The pose of `frame` in `ancestor`, composed along the parent chain.
    /// `None` when `ancestor` is not above `frame`.
    pub fn pose_in(&self, ancestor: &str, frame: &str) -> Option<Pose> {
        let mut chain = Vec::new();
        let mut cursor = frame.to_string();
        for _ in 0..=self.edges.len() {
            if cursor == ancestor {
                return Some(
                    chain
                        .iter()
                        .rev()
                        .fold(Pose::IDENTITY, |accumulated: Pose, edge: &Pose| accumulated.then(edge)),
                );
            }
            let (parent, pose) = self.edges.get(&cursor)?;
            chain.push(*pose);
            cursor = parent.clone();
        }
        None
    }

    /// Everything wrong with the tree, plus every `data_frame` it fails to
    /// place.
    pub fn problems(&self, data_frames: &[String]) -> Vec<TreeProblem> {
        let mut problems = Vec::new();
        let roots = self.roots();
        if roots.len() > 1 {
            problems.push(TreeProblem::MultipleRoots(roots));
        }
        for child in self.edges.keys() {
            if self.root_of(child).is_none() {
                problems.push(TreeProblem::Cycle(child.clone()));
            }
        }
        for frame in data_frames {
            if !self.contains(frame) {
                problems.push(TreeProblem::Unplaced(frame.clone()));
            }
        }
        problems
    }

    /// The edges as tf transforms, all carrying `stamp_nanos`, sorted by child
    /// so the output is stable.
    pub fn transforms(&self, stamp_nanos: u64) -> Vec<TransformStamped> {
        self.edges
            .iter()
            .map(|(child, (parent, pose))| pose.stamped(stamp_nanos, parent, child))
            .collect()
    }

    /// An indented listing, one frame per line, each root at column zero.
    pub fn render(&self) -> String {
        let mut children_of: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
        for (child, (parent, _)) in &self.edges {
            children_of.entry(parent.as_str()).or_default().push(child.as_str());
        }
        let mut out = String::new();
        let mut visited = BTreeSet::new();
        let roots = self.roots();
        for root in &roots {
            render_branch(&children_of, root, 0, &mut out, &mut visited);
        }
        for child in self.edges.keys() {
            if !visited.contains(child.as_str()) {
                out.push_str(&format!("{child}  (in a cycle)\n"));
            }
        }
        out
    }
}

fn render_branch<'a>(
    children_of: &BTreeMap<&str, Vec<&'a str>>,
    frame: &'a str,
    depth: usize,
    out: &mut String,
    visited: &mut BTreeSet<&'a str>,
) {
    out.push_str(&format!("{}{}\n", "  ".repeat(depth), frame));
    visited.insert(frame);
    if let Some(children) = children_of.get(frame) {
        for child in children {
            if visited.insert(child) {
                render_branch(children_of, child, depth + 1, out, visited);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    /// A 90 degree yaw about z.
    fn yaw_90() -> [f64; 4] {
        let half = std::f64::consts::FRAC_PI_4;
        [0.0, 0.0, half.sin(), half.cos()]
    }

    #[test]
    fn applying_a_pose_places_a_child_point_in_the_parent() {
        let pose = Pose::new([1.0, 2.0, 3.0], yaw_90());
        // A point one metre along the child's x lands along the parent's y.
        assert!(close(pose.apply([1.0, 0.0, 0.0]), [1.0, 3.0, 3.0]));
    }

    #[test]
    fn a_pose_followed_by_its_inverse_is_the_identity() {
        let pose = Pose::new([0.3, -0.2, 0.1], yaw_90());
        let round_trip = pose.then(&pose.inverse());
        assert!(close(round_trip.translation, [0.0; 3]));
        assert!((round_trip.rotation[3].abs() - 1.0).abs() < 1e-12);
    }

    /// The D455's right imager sits 95 mm along +x of the left one. librealsense
    /// hands back the map from left-imager points to right-imager points, whose
    /// translation is therefore -95 mm; the tf pose of the right imager is the
    /// inverse of that map.
    #[test]
    fn inverting_an_sdk_extrinsic_puts_the_right_imager_on_the_right() {
        let sdk = Pose::from_matrix([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [-0.09486, 0.0, 0.0]);
        let tf = sdk.inverse();
        assert!(close(tf.translation, [0.09486, 0.0, 0.0]));
    }

    #[test]
    fn composition_walks_the_chain_in_order() {
        let base_to_lidar = Pose::new([0.0, 0.0, 0.12], yaw_90());
        let lidar_to_imu = Pose::new([0.011, 0.02329, -0.04412], [0.0, 0.0, 0.0, 1.0]);
        let mut tree = StaticTree::new();
        tree.insert("base_link", "lidar", base_to_lidar);
        tree.insert("lidar", "imu", lidar_to_imu);

        let imu_in_base = tree.pose_in("base_link", "imu").unwrap();
        let expected = base_to_lidar.then(&lidar_to_imu);
        assert!(close(imu_in_base.translation, expected.translation));
        assert!(close(imu_in_base.apply([0.0; 3]), base_to_lidar.apply(lidar_to_imu.translation)));
        assert_eq!(tree.root_of("imu").as_deref(), Some("base_link"));
        assert!(tree.pose_in("imu", "base_link").is_none());
    }

    #[test]
    fn interpolation_meets_both_ends_and_takes_the_short_way_round() {
        let start = Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]);
        let end = Pose::new([2.0, 0.0, 0.0], yaw_90());
        assert_eq!(start.interpolate(&end, 0.0), start);
        let finish = start.interpolate(&end, 1.0);
        assert!(close(finish.translation, end.translation));
        let half = start.interpolate(&end, 0.5);
        assert!(close(half.translation, [1.0, 0.0, 0.0]));
        // Halfway through a 90 degree yaw is a 45 degree yaw.
        let quarter_turn = std::f64::consts::FRAC_PI_8;
        assert!((half.rotation[2] - quarter_turn.sin()).abs() < 1e-9);
        // The negated quaternion is the same rotation, so the blend must not
        // swing the long way through it.
        let negated = Pose::new(end.translation, end.rotation.map(|value| -value));
        let half_negated = start.interpolate(&negated, 0.5);
        assert!((half_negated.rotation[2].abs() - quarter_turn.sin()).abs() < 1e-9);
    }

    #[test]
    fn a_tree_reports_disconnection_cycles_and_unplaced_frames() {
        let mut tree = StaticTree::new();
        tree.insert("camera_link", "camera_depth_optical_frame", Pose::IDENTITY);
        tree.insert("livox_link", "livox_frame", Pose::IDENTITY);
        tree.insert("loop_a", "loop_b", Pose::IDENTITY);
        tree.insert("loop_b", "loop_a", Pose::IDENTITY);

        let problems = tree.problems(&["livox_frame".into(), "camera_imu_frame".into()]);
        assert!(problems.contains(&TreeProblem::MultipleRoots(vec!["camera_link".into(), "livox_link".into()])));
        assert!(problems.contains(&TreeProblem::Cycle("loop_a".into())));
        assert!(problems.contains(&TreeProblem::Unplaced("camera_imu_frame".into())));
        assert!(!problems.contains(&TreeProblem::Unplaced("livox_frame".into())));

        tree.insert("base_link", "camera_link", Pose::IDENTITY);
        tree.insert("base_link", "livox_link", Pose::IDENTITY);
        assert_eq!(tree.roots(), vec!["base_link".to_string()]);
        let rendered = tree.render();
        assert!(rendered.starts_with("base_link\n  camera_link\n    camera_depth_optical_frame\n"));
        assert!(rendered.contains("loop_a  (in a cycle)"));
    }

    #[test]
    fn a_quaternion_round_trips_through_its_matrix() {
        let quaternion = normalised([0.1, -0.2, 0.3, 0.9]);
        let back = crate::msgs::quaternion_from_matrix(matrix_from_quaternion(quaternion));
        let same = (0..4).all(|index| (back[index] - quaternion[index]).abs() < 1e-9)
            || (0..4).all(|index| (back[index] + quaternion[index]).abs() < 1e-9);
        assert!(same, "{back:?} vs {quaternion:?}");
    }
}
