//! URDF parsing and TF-tree validation.
//!
//! Only the parts that produce a transform are read: links, and each joint's
//! parent, child and origin. Visual geometry is skipped, since the browser
//! renders the file it uploaded rather than anything we hand back.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use anyhow::{bail, Context, Result};
use quick_xml::events::Event;
use quick_xml::Reader;
use serde::Serialize;

use crate::msgs::{quaternion_from_rpy, Header, TransformStamped};

#[derive(Debug, Clone, PartialEq)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub translation: [f64; 3],
    pub rotation_rpy: [f64; 3],
}

#[derive(Debug, Clone, Default)]
pub struct Urdf {
    pub robot_name: String,
    pub links: Vec<String>,
    pub joints: Vec<Joint>,
}

/// Everything that can be wrong with a tree, kept separate rather than collapsed
/// into one "broken" flag so the browser can say which frame to fix.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TreeProblem {
    /// Every link is somebody's child, so there is nowhere to hang the tree.
    NoRoot,
    /// Two or more links have no parent: these are disconnected trees, and a TF
    /// consumer can only resolve transforms inside one of them.
    MultipleRoots { roots: Vec<String> },
    /// A link is the child of more than one joint. TF is a tree, not a graph.
    DoubleParent { child: String, parents: Vec<String> },
    Cycle { frames: Vec<String> },
    /// A joint names a link the file never declares.
    UndeclaredLink { joint: String, link: String },
    /// A sensor frame we are about to publish data on is not in the tree, so
    /// nothing downstream can place that sensor in the robot.
    UncoveredFrame { frame: String },
}

impl TreeProblem {
    pub fn message(&self) -> String {
        match self {
            TreeProblem::NoRoot => {
                "no root link: every link is the child of a joint, so the tree has no base".into()
            }
            TreeProblem::MultipleRoots { roots } => format!(
                "{} disconnected roots ({}): the urdf describes separate trees, not one robot",
                roots.len(),
                roots.join(", ")
            ),
            TreeProblem::DoubleParent { child, parents } => format!(
                "link {child} is the child of {} joints ({}); tf allows exactly one",
                parents.len(),
                parents.join(", ")
            ),
            TreeProblem::Cycle { frames } => {
                format!("cycle through {}", frames.join(" -> "))
            }
            TreeProblem::UndeclaredLink { joint, link } => {
                format!("joint {joint} refers to link {link}, which the file never declares")
            }
            TreeProblem::UncoveredFrame { frame } => format!(
                "sensor frame {frame} is not in the urdf, so its data cannot be placed on the robot"
            ),
        }
    }
}

pub fn parse(xml: &str) -> Result<Urdf> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut urdf = Urdf::default();
    let mut current: Option<Joint> = None;
    let mut saw_robot = false;

    loop {
        match reader.read_event().context("malformed xml")? {
            Event::Eof => break,
            Event::Start(tag) | Event::Empty(tag) => {
                let name = tag.local_name();
                let name = std::str::from_utf8(name.as_ref())?.to_string();
                let attributes = collect_attributes(&tag)?;
                match name.as_str() {
                    "robot" => {
                        saw_robot = true;
                        urdf.robot_name = attributes.get("name").cloned().unwrap_or_default();
                    }
                    "link" => {
                        // A <link> nested inside a <joint> would be malformed
                        // urdf, so treat every link tag as a declaration.
                        if let Some(link_name) = attributes.get("name") {
                            urdf.links.push(link_name.clone());
                        }
                    }
                    "joint" => {
                        let joint_name = attributes
                            .get("name")
                            .cloned()
                            .unwrap_or_else(|| format!("joint_{}", urdf.joints.len()));
                        current = Some(Joint {
                            name: joint_name,
                            parent: String::new(),
                            child: String::new(),
                            translation: [0.0; 3],
                            rotation_rpy: [0.0; 3],
                        });
                    }
                    "parent" => {
                        if let (Some(joint), Some(link)) = (current.as_mut(), attributes.get("link"))
                        {
                            joint.parent = link.clone();
                        }
                    }
                    "child" => {
                        if let (Some(joint), Some(link)) = (current.as_mut(), attributes.get("link"))
                        {
                            joint.child = link.clone();
                        }
                    }
                    "origin" => {
                        if let Some(joint) = current.as_mut() {
                            if let Some(xyz) = attributes.get("xyz") {
                                joint.translation = triple(xyz)
                                    .with_context(|| format!("joint {} xyz", joint.name))?;
                            }
                            if let Some(rpy) = attributes.get("rpy") {
                                joint.rotation_rpy = triple(rpy)
                                    .with_context(|| format!("joint {} rpy", joint.name))?;
                            }
                        }
                    }
                    _ => {}
                }
            }
            Event::End(tag) if tag.local_name().as_ref() == b"joint" => {
                if let Some(joint) = current.take() {
                    if joint.parent.is_empty() || joint.child.is_empty() {
                        bail!("joint {} is missing a parent or child link", joint.name);
                    }
                    urdf.joints.push(joint);
                }
            }
            _ => {}
        }
    }

    if !saw_robot {
        bail!("no <robot> element: this is not a urdf");
    }
    Ok(urdf)
}

fn collect_attributes(tag: &quick_xml::events::BytesStart) -> Result<HashMap<String, String>> {
    let mut attributes = HashMap::new();
    for attribute in tag.attributes() {
        let attribute = attribute.context("malformed attribute")?;
        let key = std::str::from_utf8(attribute.key.local_name().as_ref())?.to_string();
        let value = attribute.unescape_value()?.into_owned();
        attributes.insert(key, value);
    }
    Ok(attributes)
}

fn triple(text: &str) -> Result<[f64; 3]> {
    let mut values = [0.0; 3];
    let mut count = 0;
    for field in text.split_whitespace() {
        if count == 3 {
            bail!("expected three numbers, got more in {text:?}");
        }
        values[count] = field
            .parse()
            .with_context(|| format!("{field:?} is not a number"))?;
        count += 1;
    }
    if count != 3 {
        bail!("expected three numbers, got {count} in {text:?}");
    }
    Ok(values)
}

impl Urdf {
    /// Links that no joint claims as a child.
    pub fn roots(&self) -> Vec<String> {
        let children: BTreeSet<&str> = self.joints.iter().map(|j| j.child.as_str()).collect();
        let mut roots: Vec<String> = self
            .frames()
            .into_iter()
            .filter(|link| !children.contains(link.as_str()))
            .collect();
        roots.sort();
        roots
    }

    /// Every frame name the file mentions, whether or not it was declared as a
    /// `<link>`.
    pub fn frames(&self) -> BTreeSet<String> {
        let mut frames: BTreeSet<String> = self.links.iter().cloned().collect();
        for joint in &self.joints {
            frames.insert(joint.parent.clone());
            frames.insert(joint.child.clone());
        }
        frames
    }

    /// Checks the tree, and additionally that every frame we are about to
    /// publish sensor data on is reachable from the root.
    pub fn problems(&self, sensor_frames: &[String]) -> Vec<TreeProblem> {
        let mut problems = Vec::new();
        let declared: BTreeSet<&str> = self.links.iter().map(String::as_str).collect();

        let mut parents_of: BTreeMap<&str, Vec<String>> = BTreeMap::new();
        for joint in &self.joints {
            for link in [&joint.parent, &joint.child] {
                if !declared.contains(link.as_str()) {
                    problems.push(TreeProblem::UndeclaredLink {
                        joint: joint.name.clone(),
                        link: link.clone(),
                    });
                }
            }
            parents_of
                .entry(joint.child.as_str())
                .or_default()
                .push(joint.name.clone());
        }
        for (child, joints) in &parents_of {
            if joints.len() > 1 {
                problems.push(TreeProblem::DoubleParent {
                    child: (*child).to_string(),
                    parents: joints.clone(),
                });
            }
        }

        let roots = self.roots();
        match roots.len() {
            // Zero roots on a non-empty file means every link is a child, which
            // only happens when the parent chain closes on itself.
            0 if !self.frames().is_empty() => {
                problems.push(TreeProblem::NoRoot);
                if let Some(cycle) = self.find_cycle() {
                    problems.push(TreeProblem::Cycle { frames: cycle });
                }
            }
            0 | 1 => {}
            _ => problems.push(TreeProblem::MultipleRoots { roots: roots.clone() }),
        }

        // A cycle hanging off a valid root does not reduce the root count, so
        // look for one regardless.
        if !problems.iter().any(|p| matches!(p, TreeProblem::Cycle { .. })) {
            if let Some(cycle) = self.find_cycle() {
                problems.push(TreeProblem::Cycle { frames: cycle });
            }
        }

        let frames = self.frames();
        for frame in sensor_frames {
            if !frames.contains(frame) {
                problems.push(TreeProblem::UncoveredFrame {
                    frame: frame.clone(),
                });
            }
        }
        problems
    }

    /// Walks parent links from every frame. A frame whose walk revisits a frame
    /// it already stepped through is in a cycle.
    fn find_cycle(&self) -> Option<Vec<String>> {
        let parent_of: BTreeMap<&str, &str> = self
            .joints
            .iter()
            .map(|joint| (joint.child.as_str(), joint.parent.as_str()))
            .collect();
        for start in self.frames() {
            let mut path = vec![start.clone()];
            let mut seen: BTreeSet<&str> = BTreeSet::new();
            seen.insert(start.as_str());
            let mut cursor = start.as_str();
            while let Some(parent) = parent_of.get(cursor) {
                path.push((*parent).to_string());
                if !seen.insert(parent) {
                    // Trim the lead-in so the report names only the loop.
                    let first = path
                        .iter()
                        .position(|frame| frame == path.last().unwrap())
                        .unwrap_or(0);
                    return Some(path[first..].to_vec());
                }
                cursor = parent;
            }
        }
        None
    }

    pub fn static_transforms(&self, stamp_nanos: u64) -> Vec<TransformStamped> {
        self.joints
            .iter()
            .map(|joint| TransformStamped {
                header: Header::new(stamp_nanos, joint.parent.clone()),
                child_frame_id: joint.child.clone(),
                translation: joint.translation,
                rotation: quaternion_from_rpy(
                    joint.rotation_rpy[0],
                    joint.rotation_rpy[1],
                    joint.rotation_rpy[2],
                ),
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HANDHELD: &str = r#"
        <robot name="handheld">
            <link name="base_link"/>
            <link name="lidar_link"/>
            <link name="camera_link"/>
            <joint name="base_to_lidar" type="fixed">
                <parent link="base_link"/>
                <child link="lidar_link"/>
                <origin xyz="0.0 0.0 0.12" rpy="0 0 1.5707963267948966"/>
            </joint>
            <joint name="base_to_camera" type="fixed">
                <parent link="base_link"/>
                <child link="camera_link"/>
                <origin xyz="0.05 -0.01 0.02"/>
            </joint>
        </robot>
    "#;

    #[test]
    fn a_valid_handheld_rig_parses_and_reports_nothing_wrong() {
        let urdf = parse(HANDHELD).unwrap();
        assert_eq!(urdf.robot_name, "handheld");
        assert_eq!(urdf.links.len(), 3);
        assert_eq!(urdf.joints.len(), 2);
        assert_eq!(urdf.roots(), vec!["base_link"]);
        let frames = vec!["lidar_link".to_string(), "camera_link".to_string()];
        assert_eq!(urdf.problems(&frames), vec![]);
    }

    #[test]
    fn joint_origins_become_tf_static_transforms() {
        let urdf = parse(HANDHELD).unwrap();
        let transforms = urdf.static_transforms(1_000);
        assert_eq!(transforms.len(), 2);

        let lidar = transforms
            .iter()
            .find(|t| t.child_frame_id == "lidar_link")
            .unwrap();
        assert_eq!(lidar.header.frame_id, "base_link");
        assert_eq!(lidar.translation, [0.0, 0.0, 0.12]);
        // A quarter turn about z is (0, 0, sin(pi/4), cos(pi/4)).
        let root_half = std::f64::consts::FRAC_1_SQRT_2;
        assert!((lidar.rotation[2] - root_half).abs() < 1e-9);
        assert!((lidar.rotation[3] - root_half).abs() < 1e-9);
        assert!(lidar.rotation[0].abs() < 1e-9 && lidar.rotation[1].abs() < 1e-9);

        // An omitted <origin> is the identity, not a parse failure.
        let camera = transforms
            .iter()
            .find(|t| t.child_frame_id == "camera_link")
            .unwrap();
        assert_eq!(camera.rotation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(camera.translation, [0.05, -0.01, 0.02]);
    }

    #[test]
    fn two_disconnected_trees_are_reported_as_multiple_roots() {
        let urdf = parse(
            r#"<robot name="split">
                <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
                <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
                <joint name="cd" type="fixed"><parent link="c"/><child link="d"/></joint>
            </robot>"#,
        )
        .unwrap();
        assert_eq!(
            urdf.problems(&[]),
            vec![TreeProblem::MultipleRoots {
                roots: vec!["a".into(), "c".into()]
            }]
        );
    }

    #[test]
    fn a_link_with_two_parents_is_reported_separately_from_a_cycle() {
        let urdf = parse(
            r#"<robot name="diamond">
                <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
                <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
                <joint name="ac" type="fixed"><parent link="a"/><child link="c"/></joint>
                <joint name="bd" type="fixed"><parent link="b"/><child link="d"/></joint>
                <joint name="cd" type="fixed"><parent link="c"/><child link="d"/></joint>
            </robot>"#,
        )
        .unwrap();
        let problems = urdf.problems(&[]);
        assert_eq!(
            problems,
            vec![TreeProblem::DoubleParent {
                child: "d".into(),
                parents: vec!["bd".into(), "cd".into()]
            }]
        );
        assert!(problems[0].message().contains("tf allows exactly one"));
    }

    #[test]
    fn a_closed_loop_is_reported_as_no_root_and_a_named_cycle() {
        let urdf = parse(
            r#"<robot name="loop">
                <link name="a"/><link name="b"/><link name="c"/>
                <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
                <joint name="bc" type="fixed"><parent link="b"/><child link="c"/></joint>
                <joint name="ca" type="fixed"><parent link="c"/><child link="a"/></joint>
            </robot>"#,
        )
        .unwrap();
        let problems = urdf.problems(&[]);
        assert!(problems.contains(&TreeProblem::NoRoot));
        let cycle = problems
            .iter()
            .find_map(|problem| match problem {
                TreeProblem::Cycle { frames } => Some(frames.clone()),
                _ => None,
            })
            .expect("the loop must be named, not just counted");
        assert!(cycle.len() >= 3);
        assert_eq!(cycle.first(), cycle.last());
    }

    #[test]
    fn a_sensor_frame_missing_from_the_tree_is_called_out_by_name() {
        let urdf = parse(HANDHELD).unwrap();
        let problems = urdf.problems(&["livox_frame".to_string()]);
        assert_eq!(
            problems,
            vec![TreeProblem::UncoveredFrame {
                frame: "livox_frame".into()
            }]
        );
        assert!(problems[0].message().contains("livox_frame"));
    }

    #[test]
    fn a_joint_naming_an_undeclared_link_is_reported() {
        let urdf = parse(
            r#"<robot name="typo">
                <link name="base_link"/>
                <joint name="j" type="fixed">
                    <parent link="base_link"/><child link="lidar_lnik"/>
                </joint>
            </robot>"#,
        )
        .unwrap();
        assert!(urdf.problems(&[]).contains(&TreeProblem::UndeclaredLink {
            joint: "j".into(),
            link: "lidar_lnik".into()
        }));
    }

    #[test]
    fn a_file_that_is_not_a_urdf_is_rejected_rather_than_silently_empty() {
        assert!(parse("<sdf version='1.6'><model name='x'/></sdf>").is_err());
        assert!(parse("not xml at all <<<").is_err());
    }

    #[test]
    fn a_joint_missing_its_child_is_an_error_not_an_empty_frame_name() {
        let result = parse(
            r#"<robot name="half"><link name="a"/>
                <joint name="j" type="fixed"><parent link="a"/></joint>
            </robot>"#,
        );
        assert!(result.is_err());
    }

    #[test]
    fn a_malformed_origin_names_the_joint_it_came_from() {
        let error = parse(
            r#"<robot name="bad"><link name="a"/><link name="b"/>
                <joint name="ab" type="fixed">
                    <parent link="a"/><child link="b"/><origin xyz="0 0"/>
                </joint>
            </robot>"#,
        )
        .unwrap_err();
        assert!(format!("{error:#}").contains("ab"));
    }
}
