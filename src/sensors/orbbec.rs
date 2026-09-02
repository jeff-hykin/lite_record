//! Orbbec stereo camera backend (Orbbec SDK v2 / libobsensor).
//!
//! No Orbbec hardware was available while this was written, so everything that
//! could be verified without one is separated out and tested: the distortion
//! and coefficient-ordering conversion, the pixel format table, and the stream
//! selection. The FFI half sits behind the `orbbec` feature and mirrors the
//! RealSense backend's shape exactly, so the two can be diffed against each
//! other.

use serde::Serialize;

use super::realsense::BodyExtrinsic;
use super::{CameraConfig, StreamId};
use crate::msgs::{CameraInfo, DistortionModel, Header};

/// `OBCameraDistortion` from libobsensor. Orbbec supplies eight coefficients
/// where OpenCV's plumb_bob takes five, so the extra three decide which ROS
/// model the CameraInfo can honestly claim.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ObDistortion {
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub k4: f64,
    pub k5: f64,
    pub k6: f64,
    pub p1: f64,
    pub p2: f64,
}

/// `ob_camera_distortion_model`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ObDistortionModel {
    None,
    ModifiedBrownConrady,
    InverseBrownConrady,
    BrownConrady,
    BrownConradyK6,
    KannalaBrandt4,
    Unrecognised(u32),
}

impl ObDistortionModel {
    pub fn from_raw(raw: u32) -> Self {
        match raw {
            0 => ObDistortionModel::None,
            1 => ObDistortionModel::ModifiedBrownConrady,
            2 => ObDistortionModel::InverseBrownConrady,
            3 => ObDistortionModel::BrownConrady,
            4 => ObDistortionModel::BrownConradyK6,
            5 => ObDistortionModel::KannalaBrandt4,
            other => ObDistortionModel::Unrecognised(other),
        }
    }
}

/// Converts Orbbec's eight coefficients into a ROS model and the coefficient
/// order that model expects.
///
/// The ordering is the trap. ROS `plumb_bob` is `[k1, k2, p1, p2, k3]` —
/// tangential terms in the middle — while `rational_polynomial` is
/// `[k1, k2, p1, p2, k3, k4, k5, k6]`. Emitting Orbbec's struct order would put
/// k3 where p1 belongs and quietly ruin every undistort.
pub fn distortion_for(
    model: ObDistortionModel,
    distortion: ObDistortion,
) -> (DistortionModel, Vec<f64>) {
    let plumb_bob = vec![
        distortion.k1,
        distortion.k2,
        distortion.p1,
        distortion.p2,
        distortion.k3,
    ];
    let rational = vec![
        distortion.k1,
        distortion.k2,
        distortion.p1,
        distortion.p2,
        distortion.k3,
        distortion.k4,
        distortion.k5,
        distortion.k6,
    ];
    let has_denominator = distortion.k4 != 0.0 || distortion.k5 != 0.0 || distortion.k6 != 0.0;

    match model {
        ObDistortionModel::None => (DistortionModel::PlumbBob, vec![0.0; 5]),
        ObDistortionModel::KannalaBrandt4 => (
            DistortionModel::Equidistant,
            vec![distortion.k1, distortion.k2, distortion.k3, distortion.k4],
        ),
        ObDistortionModel::BrownConradyK6 => (DistortionModel::RationalPolynomial, rational),
        ObDistortionModel::BrownConrady | ObDistortionModel::ModifiedBrownConrady => {
            // The k6 model is a superset of plumb_bob; if the denominator terms
            // are all zero the two are identical, so prefer the simpler name
            // that more consumers implement.
            if has_denominator {
                (DistortionModel::RationalPolynomial, rational)
            } else {
                (DistortionModel::PlumbBob, plumb_bob)
            }
        }
        // Same reasoning as the RealSense backend: the inverse map is only
        // interchangeable with plumb_bob when there is nothing to undistort.
        ObDistortionModel::InverseBrownConrady => {
            if plumb_bob.iter().all(|value| *value == 0.0) && !has_denominator {
                (DistortionModel::PlumbBob, plumb_bob)
            } else {
                (DistortionModel::Unknown, rational)
            }
        }
        ObDistortionModel::Unrecognised(_) => (DistortionModel::Unknown, rational),
    }
}

/// `ob_format`, restricted to the uncompressed layouts that map onto a ROS
/// image encoding one-for-one. Anything absent here — MJPG, and the packed
/// 12-bit Y12 whose rows are not a whole number of bytes per pixel — has no
/// honest `sensor_msgs/Image` encoding, so it is refused rather than mislabelled.
pub fn ros_encoding(ob_format: u32) -> Option<(&'static str, usize)> {
    match ob_format {
        OB_FORMAT_YUYV => Some(("yuv422_yuy2", 2)),
        OB_FORMAT_Y8 => Some(("mono8", 1)),
        OB_FORMAT_Y16 => Some(("mono16", 2)),
        OB_FORMAT_Z16 => Some(("16UC1", 2)),
        OB_FORMAT_RGB => Some(("rgb8", 3)),
        OB_FORMAT_BGR => Some(("bgr8", 3)),
        _ => None,
    }
}

/// `ob_format`.
pub const OB_FORMAT_YUYV: u32 = 0;
pub const OB_FORMAT_MJPG: u32 = 5;
pub const OB_FORMAT_Y16: u32 = 8;
pub const OB_FORMAT_Y8: u32 = 9;
pub const OB_FORMAT_Y12: u32 = 12;
pub const OB_FORMAT_RGB: u32 = 22;
pub const OB_FORMAT_BGR: u32 = 23;
pub const OB_FORMAT_Z16: u32 = 28;

/// `ob_stream_type`.
pub const OB_STREAM_IR: i32 = 1;
pub const OB_STREAM_COLOR: i32 = 2;
pub const OB_STREAM_DEPTH: i32 = 3;
pub const OB_STREAM_ACCEL: i32 = 4;
pub const OB_STREAM_GYRO: i32 = 5;
pub const OB_STREAM_IR_LEFT: i32 = 6;
pub const OB_STREAM_IR_RIGHT: i32 = 7;

/// `ob_frame_type`. Deliberately not the same numbering as `ob_stream_type`:
/// the infrared frames are 8/9 where the streams are 6/7, so a frameset
/// demultiplexed with the stream constants silently returns the wrong images.
pub const OB_FRAME_IR: i32 = 1;
pub const OB_FRAME_COLOR: i32 = 2;
pub const OB_FRAME_DEPTH: i32 = 3;
pub const OB_FRAME_ACCEL: i32 = 4;
pub const OB_FRAME_GYRO: i32 = 7;
pub const OB_FRAME_IR_LEFT: i32 = 8;
pub const OB_FRAME_IR_RIGHT: i32 = 9;

/// The frame type carried on a given stream.
pub fn ob_frame_for(stream: StreamId) -> Option<i32> {
    match stream {
        StreamId::Depth => Some(OB_FRAME_DEPTH),
        StreamId::Color => Some(OB_FRAME_COLOR),
        StreamId::InfraLeft => Some(OB_FRAME_IR_LEFT),
        StreamId::InfraRight => Some(OB_FRAME_IR_RIGHT),
        StreamId::Imu | StreamId::PointCloud => None,
    }
}

/// Unlike RealSense, Orbbec gives the two infrared imagers distinct stream
/// types rather than one type with two indices.
pub fn ob_stream_for(stream: StreamId) -> Option<i32> {
    match stream {
        StreamId::Depth => Some(OB_STREAM_DEPTH),
        StreamId::Color => Some(OB_STREAM_COLOR),
        StreamId::InfraLeft => Some(OB_STREAM_IR_LEFT),
        StreamId::InfraRight => Some(OB_STREAM_IR_RIGHT),
        StreamId::Imu | StreamId::PointCloud => None,
    }
}

/// The `ob_format` to request on each stream.
///
/// RGB rather than the sensor-native YUYV/MJPG for colour: the SDK converts, and
/// asking for it here makes that conversion explicit rather than leaving a
/// packed buffer every downstream reader would have to unpack itself.
pub fn ob_format_for(stream: StreamId) -> u32 {
    match stream {
        StreamId::Depth => OB_FORMAT_Z16,
        StreamId::Color => OB_FORMAT_RGB,
        StreamId::InfraLeft | StreamId::InfraRight => OB_FORMAT_Y8,
        StreamId::Imu | StreamId::PointCloud => OB_FORMAT_Y8,
    }
}

/// Converts one `ob_extrinsic` into the units `/tf_static` is written in.
///
/// `OBExtrinsic::trans` is documented as millimetres, where RealSense's
/// equivalent is already metres. Passing it through unscaled would put the
/// colour camera 15 metres from the depth camera — far enough that every
/// projected point lands outside the image, and subtle enough that the tree
/// still validates.
pub fn extrinsic_to_body(
    child: StreamId,
    rotation: [f32; 9],
    translation_millimeters: [f32; 3],
) -> BodyExtrinsic {
    BodyExtrinsic {
        child,
        rotation: std::array::from_fn(|slot| rotation[slot] as f64),
        translation: std::array::from_fn(|slot| translation_millimeters[slot] as f64 / 1000.0),
    }
}

#[allow(clippy::too_many_arguments)]
pub fn camera_info(
    stamp_nanos: u64,
    frame_id: &str,
    width: u32,
    height: u32,
    focal: [f64; 2],
    principal: [f64; 2],
    model: ObDistortionModel,
    distortion: ObDistortion,
    baseline_meters: f64,
) -> CameraInfo {
    let (ros_model, coefficients) = distortion_for(model, distortion);
    CameraInfo::pinhole(
        Header::new(stamp_nanos, frame_id),
        width,
        height,
        focal[0],
        focal[1],
        principal[0],
        principal[1],
        ros_model,
        coefficients,
        baseline_meters,
    )
}

#[cfg(not(feature = "orbbec"))]
mod backend {
    use super::*;
    use crate::sensors::{Backend, BackendStatus, Sink};
    use anyhow::Result;

    pub struct OrbbecBackend {
        pub config: CameraConfig,
    }

    impl OrbbecBackend {
        pub fn new(config: CameraConfig) -> Self {
            OrbbecBackend { config }
        }
    }

    impl Backend for OrbbecBackend {
        fn start(&mut self, _sink: Sink) -> Result<()> {
            anyhow::bail!(
                "this binary was built without the `orbbec` feature, so libobsensor is not linked in"
            )
        }

        fn stop(&mut self) {}

        fn status(&self) -> BackendStatus {
            BackendStatus {
                running: false,
                detail: "not compiled in".into(),
                error: Some("rebuild with --features orbbec".into()),
            }
        }
    }
}

#[cfg(feature = "orbbec")]
#[path = "orbbec_ffi.rs"]
mod backend;

pub use backend::OrbbecBackend;

impl OrbbecBackend {
    pub fn config(&self) -> &CameraConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::Backend;

    const WITH_K6: ObDistortion = ObDistortion {
        k1: 0.1,
        k2: -0.2,
        k3: 0.3,
        k4: 0.4,
        k5: 0.5,
        k6: 0.6,
        p1: 0.01,
        p2: 0.02,
    };

    #[test]
    fn plumb_bob_puts_the_tangential_terms_in_the_middle() {
        let (model, coefficients) = distortion_for(
            ObDistortionModel::BrownConrady,
            ObDistortion {
                k4: 0.0,
                k5: 0.0,
                k6: 0.0,
                ..WITH_K6
            },
        );
        assert_eq!(model, DistortionModel::PlumbBob);
        // Not Orbbec's struct order (k1 k2 k3 p1 p2) — ROS wants k1 k2 p1 p2 k3.
        assert_eq!(coefficients, vec![0.1, -0.2, 0.01, 0.02, 0.3]);
    }

    #[test]
    fn nonzero_denominator_terms_force_the_rational_model() {
        let (model, coefficients) = distortion_for(ObDistortionModel::BrownConrady, WITH_K6);
        assert_eq!(model, DistortionModel::RationalPolynomial);
        assert_eq!(
            coefficients,
            vec![0.1, -0.2, 0.01, 0.02, 0.3, 0.4, 0.5, 0.6]
        );
    }

    #[test]
    fn the_k6_model_stays_rational_even_when_its_extra_terms_are_zero() {
        let (model, coefficients) = distortion_for(
            ObDistortionModel::BrownConradyK6,
            ObDistortion {
                k4: 0.0,
                k5: 0.0,
                k6: 0.0,
                ..WITH_K6
            },
        );
        assert_eq!(model, DistortionModel::RationalPolynomial);
        assert_eq!(coefficients.len(), 8);
    }

    #[test]
    fn a_fisheye_lens_reports_four_equidistant_terms() {
        let (model, coefficients) = distortion_for(ObDistortionModel::KannalaBrandt4, WITH_K6);
        assert_eq!(model, DistortionModel::Equidistant);
        assert_eq!(coefficients, vec![0.1, -0.2, 0.3, 0.4]);
    }

    #[test]
    fn an_inverse_model_with_real_coefficients_is_admitted_unknown() {
        assert_eq!(
            distortion_for(ObDistortionModel::InverseBrownConrady, WITH_K6).0,
            DistortionModel::Unknown
        );
        assert_eq!(
            distortion_for(ObDistortionModel::InverseBrownConrady, ObDistortion::default()).0,
            DistortionModel::PlumbBob
        );
    }

    #[test]
    fn an_uncalibrated_stream_reports_zeros_rather_than_junk() {
        let (model, coefficients) = distortion_for(ObDistortionModel::None, WITH_K6);
        assert_eq!(model, DistortionModel::PlumbBob);
        assert_eq!(coefficients, vec![0.0; 5]);
    }

    #[test]
    fn intrinsics_become_a_camera_info_with_the_converted_coefficients() {
        let info = camera_info(
            7,
            "orbbec_depth_optical_frame",
            1280,
            800,
            [640.0, 640.0],
            [639.5, 399.5],
            ObDistortionModel::BrownConrady,
            ObDistortion {
                k4: 0.0,
                k5: 0.0,
                k6: 0.0,
                ..WITH_K6
            },
            0.0,
        );
        assert_eq!(info.distortion_model, "plumb_bob");
        assert_eq!(info.distortion, vec![0.1, -0.2, 0.01, 0.02, 0.3]);
        assert_eq!(info.intrinsics[0], 640.0);
        assert_eq!(info.intrinsics[2], 639.5);
        assert_eq!(info.header.frame_id, "orbbec_depth_optical_frame");
    }

    #[test]
    fn the_two_infrared_imagers_have_distinct_stream_types() {
        assert_ne!(
            ob_stream_for(StreamId::InfraLeft),
            ob_stream_for(StreamId::InfraRight)
        );
        assert_eq!(ob_stream_for(StreamId::Depth), Some(OB_STREAM_DEPTH));
        assert_eq!(ob_stream_for(StreamId::PointCloud), None);
    }

    #[test]
    fn only_formats_we_can_name_correctly_are_accepted() {
        assert_eq!(ros_encoding(OB_FORMAT_Z16), Some(("16UC1", 2)));
        assert_eq!(ros_encoding(OB_FORMAT_Y8), Some(("mono8", 1)));
        assert_eq!(ros_encoding(OB_FORMAT_RGB), Some(("rgb8", 3)));
        // Packed 12-bit rows and JPEG have no honest raw-image encoding.
        assert_eq!(ros_encoding(OB_FORMAT_Y12), None);
        assert_eq!(ros_encoding(OB_FORMAT_MJPG), None);
        assert_eq!(ros_encoding(255), None);
    }

    /// These are transcribed from `libobsensor/h/ObTypes.h` in OrbbecSDK
    /// v2.9.3. Getting one wrong opens the wrong sensor and produces a
    /// plausible-looking recording of the wrong stream, so they are pinned.
    #[test]
    fn stream_and_frame_enums_match_the_sdk_header() {
        assert_eq!(
            [
                OB_STREAM_IR,
                OB_STREAM_COLOR,
                OB_STREAM_DEPTH,
                OB_STREAM_ACCEL,
                OB_STREAM_GYRO,
                OB_STREAM_IR_LEFT,
                OB_STREAM_IR_RIGHT
            ],
            [1, 2, 3, 4, 5, 6, 7]
        );
        assert_eq!(
            [
                OB_FRAME_IR,
                OB_FRAME_COLOR,
                OB_FRAME_DEPTH,
                OB_FRAME_ACCEL,
                OB_FRAME_GYRO,
                OB_FRAME_IR_LEFT,
                OB_FRAME_IR_RIGHT
            ],
            [1, 2, 3, 4, 7, 8, 9]
        );
    }

    /// Orbbec reports translation in millimetres where RealSense reports metres.
    #[test]
    fn extrinsic_translation_is_converted_out_of_millimetres() {
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let extrinsic = extrinsic_to_body(StreamId::Color, identity, [15.0, -0.2, 0.4]);
        // 15 mm across, the physical spacing on a Gemini — not 15 metres.
        assert!((extrinsic.translation[0] - 0.015).abs() < 1e-9);
        assert!((extrinsic.translation[1] + 0.0002).abs() < 1e-9);
        assert_eq!(extrinsic.rotation[0], 1.0);
    }

    #[test]
    fn every_requested_format_is_one_we_can_publish() {
        for stream in [
            StreamId::Depth,
            StreamId::Color,
            StreamId::InfraLeft,
            StreamId::InfraRight,
        ] {
            assert!(
                ros_encoding(ob_format_for(stream)).is_some(),
                "{stream:?} is opened in a format with no ROS encoding"
            );
        }
    }

    #[test]
    fn infrared_frames_are_not_numbered_like_infrared_streams() {
        assert_eq!(ob_stream_for(StreamId::InfraLeft), Some(6));
        assert_eq!(ob_frame_for(StreamId::InfraLeft), Some(8));
        assert_eq!(ob_frame_for(StreamId::PointCloud), None);
    }

    #[test]
    fn without_the_feature_the_backend_says_so_instead_of_pretending() {
        let mut backend =
            OrbbecBackend::new(CameraConfig::for_kind(super::super::SensorKind::Orbbec));
        assert!(!backend.status().running);
        if !cfg!(feature = "orbbec") {
            let error = backend.start(std::sync::Arc::new(|_| true)).unwrap_err();
            assert!(format!("{error:#}").contains("orbbec"));
        }
    }
}
