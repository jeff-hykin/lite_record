//! Intel RealSense backend.
//!
//! The SDK-independent half — distortion mapping, pixel format naming, stream
//! selection — is compiled and tested unconditionally. The FFI half is behind
//! the `realsense` feature so this crate builds on a machine with no
//! librealsense.

use serde::Serialize;

use super::{CameraConfig, Naming, StreamId};
use crate::msgs::{CameraInfo, DistortionModel, Header};

/// `rs2_distortion`, from librealsense's `rs_types.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum RsDistortion {
    None,
    ModifiedBrownConrady,
    InverseBrownConrady,
    Ftheta,
    BrownConrady,
    KannalaBrandt4,
    Unrecognised(u32),
}

impl RsDistortion {
    pub fn from_raw(raw: u32) -> Self {
        match raw {
            0 => RsDistortion::None,
            1 => RsDistortion::ModifiedBrownConrady,
            2 => RsDistortion::InverseBrownConrady,
            3 => RsDistortion::Ftheta,
            4 => RsDistortion::BrownConrady,
            5 => RsDistortion::KannalaBrandt4,
            other => RsDistortion::Unrecognised(other),
        }
    }
}

/// Maps a RealSense distortion model onto a ROS one.
///
/// This is the part that is easy to get quietly wrong. `plumb_bob` is the
/// forward Brown-Conrady model: undistort by *applying* the polynomial.
/// `INVERSE_BROWN_CONRADY` is the other direction, so labelling it plumb_bob
/// makes every consumer undistort backwards. RealSense only ever reports it on
/// streams that are already rectified, where all five coefficients are zero and
/// the two models coincide, so the mapping is allowed exactly in that case and
/// refused otherwise.
pub fn distortion_for(model: RsDistortion, coefficients: [f64; 5]) -> DistortionModel {
    let undistorted = coefficients.iter().all(|value| *value == 0.0);
    match model {
        RsDistortion::None => DistortionModel::PlumbBob,
        RsDistortion::BrownConrady | RsDistortion::ModifiedBrownConrady => {
            DistortionModel::PlumbBob
        }
        RsDistortion::InverseBrownConrady if undistorted => DistortionModel::PlumbBob,
        RsDistortion::KannalaBrandt4 => DistortionModel::Equidistant,
        RsDistortion::InverseBrownConrady
        | RsDistortion::Ftheta
        | RsDistortion::Unrecognised(_) => DistortionModel::Unknown,
    }
}

/// `rs2_format` values this backend knows how to publish, mapped to the ROS
/// `sensor_msgs/Image` encoding string.
pub fn ros_encoding(rs2_format: u32) -> Option<(&'static str, usize)> {
    // From rs_sensor.h. Only the formats a D400 or D455 actually emits on the
    // streams we enable are listed; anything else is refused loudly rather than
    // written to the file with a wrong encoding string.
    match rs2_format {
        1 => Some(("16UC1", 2)),  // RS2_FORMAT_Z16
        4 => Some(("yuv422_yuy2", 2)), // RS2_FORMAT_YUYV
        5 => Some(("rgb8", 3)),   // RS2_FORMAT_RGB8
        6 => Some(("bgr8", 3)),   // RS2_FORMAT_BGR8
        7 => Some(("rgba8", 4)),  // RS2_FORMAT_RGBA8
        8 => Some(("bgra8", 4)),  // RS2_FORMAT_BGRA8
        9 => Some(("mono8", 1)),  // RS2_FORMAT_Y8
        11 => Some(("mono16", 2)), // RS2_FORMAT_Y16
        _ => None,
    }
}

/// Builds the CameraInfo that goes alongside an image stream.
///
/// The baseline term only belongs on the right imager of a rectified stereo
/// pair; putting it on the left one shifts every reprojected point by the
/// baseline. Depth is registered to the left imager, so it gets zero too.
#[allow(clippy::too_many_arguments)]
pub fn camera_info(
    stamp_nanos: u64,
    frame_id: &str,
    width: u32,
    height: u32,
    focal: [f64; 2],
    principal: [f64; 2],
    model: RsDistortion,
    coefficients: [f64; 5],
    baseline_meters: f64,
) -> CameraInfo {
    CameraInfo::pinhole(
        Header::new(stamp_nanos, frame_id),
        width,
        height,
        focal[0],
        focal[1],
        principal[0],
        principal[1],
        distortion_for(model, coefficients),
        coefficients.to_vec(),
        baseline_meters,
    )
}

/// `rs2_stream`, only the ones we enable.
pub const RS2_STREAM_DEPTH: i32 = 1;
pub const RS2_STREAM_COLOR: i32 = 2;
pub const RS2_STREAM_INFRARED: i32 = 3;
pub const RS2_STREAM_GYRO: i32 = 5;
pub const RS2_STREAM_ACCEL: i32 = 6;

/// The (stream, index) pairs librealsense wants for a given logical stream.
/// Infrared is the awkward one: it is one stream type with two indices, and
/// asking for index 0 gives whichever imager the SDK feels like.
pub fn rs2_stream_for(stream: StreamId) -> Option<(i32, i32)> {
    match stream {
        StreamId::Depth => Some((RS2_STREAM_DEPTH, 0)),
        StreamId::Color => Some((RS2_STREAM_COLOR, 0)),
        StreamId::InfraLeft => Some((RS2_STREAM_INFRARED, 1)),
        StreamId::InfraRight => Some((RS2_STREAM_INFRARED, 2)),
        StreamId::Imu | StreamId::PointCloud => None,
    }
}

/// The transforms inside the camera body, which no URDF can supply because they
/// come from the unit's own factory calibration. Publishing them means a
/// recording is self-contained: colour pixels can be placed against depth
/// without the calibration file.
pub struct BodyExtrinsic {
    pub child: StreamId,
    pub rotation: [f64; 9],
    pub translation: [f64; 3],
}

pub fn body_transforms(
    naming: &Naming,
    stamp_nanos: u64,
    extrinsics: &[BodyExtrinsic],
) -> Vec<crate::msgs::TransformStamped> {
    let parent = naming.frame_id(StreamId::Depth);
    let mut transforms = vec![crate::msgs::TransformStamped::identity(
        &naming.root_frame_id(),
        &parent,
    )];
    transforms[0].header = Header::new(stamp_nanos, naming.root_frame_id());
    for extrinsic in extrinsics {
        transforms.push(crate::msgs::TransformStamped {
            header: Header::new(stamp_nanos, parent.clone()),
            child_frame_id: naming.frame_id(extrinsic.child),
            translation: extrinsic.translation,
            rotation: crate::msgs::quaternion_from_matrix(extrinsic.rotation),
        });
    }
    transforms
}

#[cfg(not(feature = "realsense"))]
mod backend {
    use super::*;
    use anyhow::Result;
    use crate::sensors::{Backend, BackendStatus, Sink};

    pub struct RealsenseBackend {
        pub config: CameraConfig,
    }

    impl RealsenseBackend {
        pub fn new(config: CameraConfig) -> Self {
            RealsenseBackend { config }
        }
    }

    impl Backend for RealsenseBackend {
        fn start(&mut self, _sink: Sink) -> Result<()> {
            anyhow::bail!(
                "this binary was built without the `realsense` feature, so librealsense is not linked in"
            )
        }

        fn stop(&mut self) {}

        fn status(&self) -> BackendStatus {
            BackendStatus {
                running: false,
                detail: "not compiled in".into(),
                error: Some("rebuild with --features realsense".into()),
            }
        }
    }
}

#[cfg(feature = "realsense")]
#[path = "realsense_ffi.rs"]
mod backend;

pub use backend::RealsenseBackend;

impl RealsenseBackend {
    pub fn config(&self) -> &CameraConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::Backend;

    #[test]
    fn a_rectified_inverse_brown_conrady_stream_is_plumb_bob() {
        // Every D435/D455 depth and infrared stream reports this model with all
        // coefficients zero, which is the same map as plumb_bob.
        assert_eq!(
            distortion_for(RsDistortion::InverseBrownConrady, [0.0; 5]),
            DistortionModel::PlumbBob
        );
    }

    #[test]
    fn an_inverse_model_with_real_coefficients_is_not_claimed_to_be_plumb_bob() {
        // plumb_bob is the forward map. Labelling the inverse map as plumb_bob
        // would make every consumer undistort in the wrong direction.
        assert_eq!(
            distortion_for(RsDistortion::InverseBrownConrady, [0.1, 0.0, 0.0, 0.0, 0.0]),
            DistortionModel::Unknown
        );
    }

    #[test]
    fn the_fisheye_models_map_to_equidistant_and_the_rest_are_admitted_unknown() {
        assert_eq!(
            distortion_for(RsDistortion::KannalaBrandt4, [0.1; 5]),
            DistortionModel::Equidistant
        );
        assert_eq!(
            distortion_for(RsDistortion::Ftheta, [0.1; 5]),
            DistortionModel::Unknown
        );
        assert_eq!(
            distortion_for(RsDistortion::Unrecognised(99), [0.0; 5]),
            DistortionModel::Unknown
        );
    }

    #[test]
    fn the_forward_brown_conrady_models_are_plumb_bob_with_or_without_coefficients() {
        for model in [
            RsDistortion::BrownConrady,
            RsDistortion::ModifiedBrownConrady,
        ] {
            assert_eq!(
                distortion_for(model, [0.15, -0.4, 0.001, 0.0, 0.3]),
                DistortionModel::PlumbBob
            );
        }
        assert_eq!(
            distortion_for(RsDistortion::None, [0.0; 5]),
            DistortionModel::PlumbBob
        );
    }

    #[test]
    fn the_raw_enum_values_match_librealsense() {
        assert_eq!(RsDistortion::from_raw(0), RsDistortion::None);
        assert_eq!(RsDistortion::from_raw(2), RsDistortion::InverseBrownConrady);
        assert_eq!(RsDistortion::from_raw(4), RsDistortion::BrownConrady);
        assert_eq!(RsDistortion::from_raw(5), RsDistortion::KannalaBrandt4);
        assert_eq!(RsDistortion::from_raw(77), RsDistortion::Unrecognised(77));
    }

    /// Real numbers read off the D435IF on the Alfred Jetson.
    #[test]
    fn real_depth_intrinsics_become_a_usable_camera_info() {
        let info = camera_info(
            1_000_000_000,
            "camera_depth_optical_frame",
            848,
            480,
            [423.4735107421875, 423.4735107421875],
            [424.2603759765625, 240.7674560546875],
            RsDistortion::BrownConrady,
            [0.0; 5],
            0.0,
        );
        assert_eq!(info.width, 848);
        assert_eq!(info.height, 480);
        assert_eq!(info.distortion_model, "plumb_bob");
        // K is row-major [fx 0 cx; 0 fy cy; 0 0 1].
        assert_eq!(info.intrinsics[0], 423.4735107421875);
        assert_eq!(info.intrinsics[2], 424.2603759765625);
        assert_eq!(info.intrinsics[4], 423.4735107421875);
        assert_eq!(info.intrinsics[5], 240.7674560546875);
        assert_eq!(info.intrinsics[8], 1.0);
        // R is identity on a rectified stream.
        assert_eq!(info.rectification, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
        // No baseline term on the left imager.
        assert_eq!(info.projection[3], 0.0);
    }

    #[test]
    fn the_right_imager_carries_the_baseline_in_p() {
        let baseline = 0.0499585904181004;
        let info = camera_info(
            0,
            "camera_infra2_optical_frame",
            848,
            480,
            [423.4735107421875, 423.4735107421875],
            [424.2603759765625, 240.7674560546875],
            RsDistortion::BrownConrady,
            [0.0; 5],
            baseline,
        );
        // P[3] is -fx * baseline; getting the sign wrong reprojects the right
        // image one baseline the wrong way.
        assert!((info.projection[3] + 423.4735107421875 * baseline).abs() < 1e-9);
    }

    #[test]
    fn only_formats_we_can_name_correctly_are_accepted() {
        assert_eq!(ros_encoding(1), Some(("16UC1", 2)));
        assert_eq!(ros_encoding(9), Some(("mono8", 1)));
        assert_eq!(ros_encoding(5), Some(("rgb8", 3)));
        // An unmapped format must be refused rather than guessed at, since a
        // wrong encoding string silently corrupts every reader.
        assert_eq!(ros_encoding(30), None);
    }

    #[test]
    fn the_two_infrared_imagers_are_requested_by_index_not_by_luck() {
        assert_eq!(rs2_stream_for(StreamId::InfraLeft), Some((3, 1)));
        assert_eq!(rs2_stream_for(StreamId::InfraRight), Some((3, 2)));
        assert_eq!(rs2_stream_for(StreamId::Depth), Some((1, 0)));
        assert_eq!(rs2_stream_for(StreamId::Imu), None);
    }

    /// The real depth-to-colour extrinsic from the D435IF on Alfred.
    #[test]
    fn factory_extrinsics_become_tf_edges_under_the_camera_root() {
        let naming = Naming::for_kind(super::super::SensorKind::Realsense);
        let transforms = body_transforms(
            &naming,
            5,
            &[BodyExtrinsic {
                child: StreamId::Color,
                rotation: [
                    0.9999404, -0.0089, 0.0063, 0.0089, 0.9999, -0.0021, -0.0063, 0.0022, 0.99998,
                ],
                translation: [
                    0.0146514037624002,
                    -0.000171391482581384,
                    0.000417986128013581,
                ],
            }],
        );
        assert_eq!(transforms.len(), 2);
        // The root edge is what a urdf attaches to.
        assert_eq!(transforms[0].header.frame_id, "camera_link");
        assert_eq!(transforms[0].child_frame_id, "camera_depth_optical_frame");

        let color = &transforms[1];
        assert_eq!(color.header.frame_id, "camera_depth_optical_frame");
        assert_eq!(color.child_frame_id, "camera_color_optical_frame");
        // ~14.65 mm to the right, which is the physical spacing on a D435.
        assert!((color.translation[0] - 0.0146514037624002).abs() < 1e-12);
        let norm: f64 = color.rotation.iter().map(|v| v * v).sum();
        assert!((norm - 1.0).abs() < 1e-6);
    }

    #[test]
    fn without_the_feature_the_backend_says_so_instead_of_pretending() {
        let mut backend = RealsenseBackend::new(CameraConfig::for_kind(
            super::super::SensorKind::Realsense,
        ));
        assert!(!backend.status().running);
        if !cfg!(feature = "realsense") {
            let error = backend
                .start(std::sync::Arc::new(|_| true))
                .unwrap_err();
            assert!(format!("{error:#}").contains("realsense"));
        }
    }
}
