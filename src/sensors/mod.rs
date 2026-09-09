//! Sensor backends.
//!
//! Every vendor SDK sits behind a cargo feature, so this crate builds and its
//! tests run on a laptop with no hardware and no SDK installed. The parts that
//! can be tested without hardware — configuration, topic naming, frame ids,
//! intrinsics conversion — live here rather than inside the feature-gated
//! modules, so they are covered on every build.

pub mod livox;
pub mod oakd;
pub mod orbbec;
pub mod realsense;

use serde::{Deserialize, Serialize};

use crate::msgs::{CameraInfo, Imu, PointCloud2, RawImage};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorKind {
    Realsense,
    Orbbec,
    OakD,
    Livox,
}

impl SensorKind {
    pub fn as_str(self) -> &'static str {
        match self {
            SensorKind::Realsense => "realsense",
            SensorKind::Orbbec => "orbbec",
            SensorKind::OakD => "oakd",
            SensorKind::Livox => "livox",
        }
    }

    pub fn default_topic_prefix(self) -> &'static str {
        match self {
            SensorKind::Realsense => "/realsense",
            SensorKind::Orbbec => "/orbbec",
            SensorKind::OakD => "/oakd",
            SensorKind::Livox => "/livox",
        }
    }

    pub fn default_frame_prefix(self) -> &'static str {
        match self {
            // `camera` so the tf tree matches the dimos rust realsense module,
            // whose frame_id defaults to `camera_link`.
            SensorKind::Realsense => "camera",
            SensorKind::Orbbec => "orbbec",
            SensorKind::OakD => "oakd",
            SensorKind::Livox => "livox",
        }
    }

    /// Whether the binary was built with this backend's SDK linked in.
    pub fn compiled_in(self) -> bool {
        match self {
            SensorKind::Realsense => cfg!(feature = "realsense"),
            SensorKind::Orbbec => cfg!(feature = "orbbec"),
            SensorKind::OakD => cfg!(feature = "oakd"),
            // The lidar speaks plain UDP, so its receive path needs no SDK at
            // all; the feature only adds the SDK's configuration handshake.
            SensorKind::Livox => true,
        }
    }
}

/// The five stream kinds a depth camera produces. Named rather than indexed so
/// a settings blob stays readable and a renamed topic is obvious in a diff.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum StreamId {
    Depth,
    Color,
    InfraLeft,
    InfraRight,
    Imu,
    PointCloud,
}

impl StreamId {
    pub fn as_str(self) -> &'static str {
        match self {
            StreamId::Depth => "depth",
            StreamId::Color => "color",
            StreamId::InfraLeft => "infra_left",
            StreamId::InfraRight => "infra_right",
            StreamId::Imu => "imu",
            StreamId::PointCloud => "points",
        }
    }

    /// The topic's last segment. These are the dimos module output names, so a
    /// recording drops into a dimos graph without a remapping table. A frame
    /// that goes to the card compressed is written one level below this, see
    /// [`crate::record`].
    pub fn topic_leaf(self) -> &'static str {
        match self {
            StreamId::Depth => "depth_image",
            StreamId::Color => "color_image",
            StreamId::InfraLeft => "infrared_left",
            StreamId::InfraRight => "infrared_right",
            StreamId::Imu => "imu",
            // dimos names the RealSense's cloud `pointcloud` and the Mid-360's
            // `lidar`; only the lidar publishes a cloud here.
            StreamId::PointCloud => "lidar",
        }
    }

    /// The `CameraInfo` topic's last segment. Colour's is bare `camera_info`
    /// rather than `color_camera_info`, matching dimos, whose RealSense module
    /// treats colour as the camera's default intrinsics.
    pub fn camera_info_leaf(self) -> &'static str {
        match self {
            StreamId::Color => "camera_info",
            StreamId::Depth => "depth_camera_info",
            StreamId::InfraLeft => "infrared_left_camera_info",
            StreamId::InfraRight => "infrared_right_camera_info",
            StreamId::Imu | StreamId::PointCloud => "camera_info",
        }
    }

    /// The frame id suffix. Both infrared images come out of the depth module's
    /// two imagers, which are distinct optical centres, so they cannot share
    /// one frame.
    pub fn frame_suffix(self) -> &'static str {
        match self {
            StreamId::Depth => "depth_optical_frame",
            StreamId::Color => "color_optical_frame",
            StreamId::InfraLeft => "infra1_optical_frame",
            StreamId::InfraRight => "infra2_optical_frame",
            StreamId::Imu => "imu_frame",
            StreamId::PointCloud => "frame",
        }
    }

    pub fn is_image(self) -> bool {
        matches!(
            self,
            StreamId::Depth | StreamId::Color | StreamId::InfraLeft | StreamId::InfraRight
        )
    }
}

/// Per-sensor naming. Both prefixes are editable from the UI because a rig with
/// two RealSenses needs them to differ, and because a TF tree authored
/// elsewhere dictates the frame names we must publish under.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Naming {
    pub topic_prefix: String,
    pub frame_prefix: String,
}

impl Naming {
    pub fn for_kind(kind: SensorKind) -> Self {
        Naming {
            topic_prefix: kind.default_topic_prefix().to_string(),
            frame_prefix: kind.default_frame_prefix().to_string(),
        }
    }

    /// `/realsense/depth_image`
    pub fn image_topic(&self, stream: StreamId) -> String {
        self.topic(stream.topic_leaf())
    }

    /// `/realsense/depth_camera_info`
    pub fn camera_info_topic(&self, stream: StreamId) -> String {
        self.topic(stream.camera_info_leaf())
    }

    pub fn imu_topic(&self) -> String {
        self.topic(StreamId::Imu.topic_leaf())
    }

    /// `/livox/lidar`
    pub fn points_topic(&self) -> String {
        self.topic(StreamId::PointCloud.topic_leaf())
    }

    pub fn topic(&self, leaf: &str) -> String {
        format!("{}/{leaf}", self.topic_prefix.trim_end_matches('/'))
    }

    /// `camera_depth_optical_frame`
    pub fn frame_id(&self, stream: StreamId) -> String {
        format!(
            "{}_{}",
            self.frame_prefix.trim_end_matches('_'),
            stream.frame_suffix()
        )
    }

    /// The frame every other frame on this sensor hangs off, and the one a URDF
    /// has to name to attach the sensor to the robot.
    pub fn root_frame_id(&self) -> String {
        format!("{}_link", self.frame_prefix.trim_end_matches('_'))
    }
}

/// The gyro rate every D4xx offers, and the one both IMU parts share.
fn default_imu_rate() -> u32 {
    200
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraConfig {
    pub enabled: bool,
    pub naming: Naming,
    /// Serial number, so a two-camera rig is deterministic about which is which.
    #[serde(default)]
    pub serial: Option<String>,
    pub depth: bool,
    pub color: bool,
    pub infrared: bool,
    pub imu: bool,
    pub width: u32,
    pub height: u32,
    pub frame_rate: u32,
    /// Requested IMU rate in Hz. Every IMU part offers its own fixed set of
    /// rates — a BMI055 accelerometer does 63 and 250, a BMI085 does 100, 200
    /// and 400 — so this is matched to the nearest rate the device actually has
    /// rather than passed through. Defaulted on read so a settings file written
    /// before this field existed still loads.
    #[serde(default = "default_imu_rate")]
    pub imu_rate: u32,
    /// The IR projector. It has to be off for stereo feature tracking and on
    /// for dense depth, and switching it is the single most common runtime
    /// change, so it is exposed as its own control.
    pub emitter: bool,
    /// Reprojects depth into the colour camera's optical frame. Costs CPU on
    /// the Pi, so it is off by default and the raw depth keeps its own
    /// CameraInfo either way.
    pub align_depth_to_color: bool,
}

impl CameraConfig {
    pub fn for_kind(kind: SensorKind) -> Self {
        CameraConfig {
            enabled: false,
            naming: Naming::for_kind(kind),
            serial: None,
            depth: true,
            color: true,
            infrared: true,
            imu: true,
            width: 640,
            height: 480,
            frame_rate: 30,
            imu_rate: default_imu_rate(),
            // Off by default: the projector's speckle blankets the IR pair and
            // ruins stereo VO (grocery recording, 2026-09-09).
            emitter: false,
            align_depth_to_color: false,
        }
    }

    pub fn streams(&self) -> Vec<StreamId> {
        let mut streams = Vec::new();
        if self.depth {
            streams.push(StreamId::Depth);
        }
        if self.color {
            streams.push(StreamId::Color);
        }
        if self.infrared {
            streams.push(StreamId::InfraLeft);
            streams.push(StreamId::InfraRight);
        }
        if self.imu {
            streams.push(StreamId::Imu);
        }
        streams
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LivoxConfig {
    pub enabled: bool,
    pub naming: Naming,
    /// Which NIC the lidar is on. Empty means "let the OS choose", which works
    /// on a single-NIC Pi and fails on a multi-NIC Jetson.
    #[serde(default)]
    pub interface: String,
    #[serde(default)]
    pub host_address: String,
    #[serde(default)]
    pub lidar_address: Option<String>,
    /// Clouds published per second. The lidar always emits at its own rate; this
    /// is the bin width the points are accumulated into.
    pub frame_hz: f64,
    pub imu: bool,
    /// Leaf size in metres. Zero keeps every point.
    pub voxel_leaf_size: f32,
}

impl Default for LivoxConfig {
    fn default() -> Self {
        LivoxConfig {
            enabled: false,
            naming: Naming::for_kind(SensorKind::Livox),
            interface: String::new(),
            host_address: String::new(),
            lidar_address: None,
            frame_hz: 10.0,
            imu: true,
            voxel_leaf_size: 0.0,
        }
    }
}

impl LivoxConfig {
    pub fn streams(&self) -> Vec<StreamId> {
        let mut streams = vec![StreamId::PointCloud];
        if self.imu {
            streams.push(StreamId::Imu);
        }
        streams
    }
}

/// What a backend produces. Encoding to CDR happens off the capture thread, so
/// these are the decoded structs rather than bytes.
pub enum Produced {
    Image {
        stream: StreamId,
        topic: String,
        image: RawImage,
    },
    CameraInfo {
        topic: String,
        info: Box<CameraInfo>,
    },
    /// Boxed for the same reason CameraInfo is: an Imu carries three 9-element
    /// covariance matrices, and an unboxed variant would set the size of every
    /// message the encode queue holds, image frames included.
    Imu {
        topic: String,
        imu: Box<Imu>,
    },
    Cloud {
        topic: String,
        cloud: PointCloud2,
    },
}

impl Produced {
    pub fn topic(&self) -> &str {
        match self {
            Produced::Image { topic, .. }
            | Produced::CameraInfo { topic, .. }
            | Produced::Imu { topic, .. }
            | Produced::Cloud { topic, .. } => topic,
        }
    }
}

/// Where a backend sends what it captured. Returns false when the pipeline is
/// saturated, which the backend counts as a drop rather than blocking on.
pub type Sink = std::sync::Arc<dyn Fn(Produced) -> bool + Send + Sync>;

/// The state a running backend reports back to the UI.
#[derive(Debug, Clone, Serialize, Default)]
pub struct BackendStatus {
    pub running: bool,
    pub detail: String,
    pub error: Option<String>,
}

pub trait Backend: Send {
    /// Opens the device and begins streaming. Called again after `stop` to
    /// re-engage, so it must not assume a fresh process.
    fn start(&mut self, sink: Sink) -> anyhow::Result<()>;
    /// Releases the device so another process can claim it. The backend object
    /// survives, holding its configuration.
    fn stop(&mut self);
    fn status(&self) -> BackendStatus;
    /// Applies settings that can change while streaming. Returns false when the
    /// change needs a restart, which the hub then performs.
    fn apply_live(&mut self, _config: &CameraConfig) -> bool {
        false
    }
    /// The transforms only the device itself can supply, read from its factory
    /// calibration once it is open. Empty when the backend has none, in which
    /// case the hub falls back to identity edges so the frames still exist.
    fn body_transforms(&self) -> Vec<crate::msgs::TransformStamped> {
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topics_and_frames_follow_the_editable_prefixes() {
        let naming = Naming {
            topic_prefix: "/front_cam".into(),
            frame_prefix: "front".into(),
        };
        assert_eq!(naming.image_topic(StreamId::Depth), "/front_cam/depth_image");
        assert_eq!(
            naming.image_topic(StreamId::InfraLeft),
            "/front_cam/infrared_left"
        );
        // Colour's intrinsics are the camera's, so this one is not prefixed.
        assert_eq!(
            naming.camera_info_topic(StreamId::Color),
            "/front_cam/camera_info"
        );
        assert_eq!(
            naming.camera_info_topic(StreamId::Depth),
            "/front_cam/depth_camera_info"
        );
        assert_eq!(naming.imu_topic(), "/front_cam/imu");
        assert_eq!(
            naming.frame_id(StreamId::Color),
            "front_color_optical_frame"
        );
        assert_eq!(naming.root_frame_id(), "front_link");
    }

    #[test]
    fn a_trailing_slash_in_a_prefix_does_not_double_up() {
        let naming = Naming {
            topic_prefix: "/cam/".into(),
            frame_prefix: "cam_".into(),
        };
        assert_eq!(naming.image_topic(StreamId::Depth), "/cam/depth_image");
        assert_eq!(naming.frame_id(StreamId::Depth), "cam_depth_optical_frame");
    }

    #[test]
    fn the_two_infrared_imagers_get_different_frames() {
        let naming = Naming::for_kind(SensorKind::Realsense);
        assert_ne!(
            naming.frame_id(StreamId::InfraLeft),
            naming.frame_id(StreamId::InfraRight)
        );
    }

    #[test]
    fn two_cameras_with_different_prefixes_never_collide() {
        let front = Naming {
            topic_prefix: "/front".into(),
            frame_prefix: "front".into(),
        };
        let back = Naming {
            topic_prefix: "/back".into(),
            frame_prefix: "back".into(),
        };
        for stream in [StreamId::Depth, StreamId::Color, StreamId::InfraLeft] {
            assert_ne!(front.image_topic(stream), back.image_topic(stream));
            assert_ne!(front.frame_id(stream), back.frame_id(stream));
        }
    }

    #[test]
    fn turning_off_a_stream_removes_it_from_the_list() {
        let mut config = CameraConfig::for_kind(SensorKind::Realsense);
        assert_eq!(config.streams().len(), 5);
        config.color = false;
        assert!(!config.streams().contains(&StreamId::Color));
        config.infrared = false;
        assert_eq!(config.streams(), vec![StreamId::Depth, StreamId::Imu]);
    }

    #[test]
    fn settings_round_trip_through_json() {
        let config = CameraConfig::for_kind(SensorKind::Orbbec);
        let text = serde_json::to_string(&config).unwrap();
        assert_eq!(serde_json::from_str::<CameraConfig>(&text).unwrap(), config);

        let lidar = LivoxConfig::default();
        let text = serde_json::to_string(&lidar).unwrap();
        assert_eq!(serde_json::from_str::<LivoxConfig>(&text).unwrap(), lidar);
    }

    #[test]
    fn the_lidar_receive_path_needs_no_sdk() {
        assert!(SensorKind::Livox.compiled_in());
        assert_eq!(
            SensorKind::Realsense.compiled_in(),
            cfg!(feature = "realsense")
        );
    }
}
