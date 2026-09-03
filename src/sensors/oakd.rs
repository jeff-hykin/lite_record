//! Luxonis OAK-D backend (depthai-core v3).
//!
//! Same split as the other two: everything that can be decided without the SDK
//! — the device clock mapping, the calibration conversion, the socket table —
//! is compiled and tested unconditionally, and the FFI half sits behind the
//! `oakd` feature.
//!
//! Two things about depthai differ from librealsense and libobsensor and are
//! the reason this file exists rather than a straight copy of one of them.
//! Timestamps arrive on the host's *steady* clock rather than as an epoch, so
//! they have to be offset onto the wall clock without letting the wall clock's
//! rate leak into them. And calibration translations come back in centimetres,
//! which is a unit no ROS message uses anywhere.

use serde::Serialize;

use super::{CameraConfig, Naming, StreamId};
use crate::msgs::{CameraInfo, DistortionModel, Header};

/// `dai::CameraBoardSocket`, from `depthai/common/CameraBoardSocket.hpp`.
pub const CAM_A: i32 = 0;
pub const CAM_B: i32 = 1;
pub const CAM_C: i32 = 2;

/// Which imager backs each logical stream on an OAK-D.
///
/// CAM_A is the colour camera, CAM_B and CAM_C the two global-shutter mono
/// imagers that form the stereo pair. Depth is computed in the left imager's
/// frame, so it shares CAM_B's intrinsics rather than having any of its own.
pub fn socket_for(stream: StreamId) -> Option<i32> {
    match stream {
        StreamId::Color => Some(CAM_A),
        StreamId::InfraLeft | StreamId::Depth => Some(CAM_B),
        StreamId::InfraRight => Some(CAM_C),
        StreamId::Imu | StreamId::PointCloud => None,
    }
}

/// The stream indices the shim is addressed by, mirroring the constants at the
/// top of `oakd_shim.cpp`. Both sides are pinned by the test below, because a
/// mismatch here would record one imager's frames under another's topic without
/// anything failing.
pub fn shim_stream_index(stream: StreamId) -> Option<i32> {
    match stream {
        StreamId::Depth => Some(0),
        StreamId::Color => Some(1),
        StreamId::InfraLeft => Some(2),
        StreamId::InfraRight => Some(3),
        StreamId::Imu => Some(4),
        StreamId::PointCloud => None,
    }
}

/// The pixel layouts the shim reports, and the ROS encoding each becomes.
///
/// Stereo depth arrives as unsigned 16-bit millimetres, which ROS spells
/// `16UC1` rather than `mono16`: the two are the same bytes, but only the former
/// tells a consumer the values are a depth rather than a brightness.
pub fn ros_encoding(pixels: i32) -> Option<(&'static str, usize)> {
    match pixels {
        0 => Some(("mono8", 1)),
        1 => Some(("16UC1", 2)),
        2 => Some(("bgr8", 3)),
        _ => None,
    }
}

/// `dai::CameraModel`, from `depthai/common/CameraModel.hpp`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum OakCameraModel {
    Perspective,
    Fisheye,
    Equirectangular,
    RadialDivision,
    Unrecognised(i32),
}

impl OakCameraModel {
    pub fn from_raw(raw: i32) -> Self {
        match raw {
            0 => OakCameraModel::Perspective,
            1 => OakCameraModel::Fisheye,
            2 => OakCameraModel::Equirectangular,
            3 => OakCameraModel::RadialDivision,
            other => OakCameraModel::Unrecognised(other),
        }
    }
}

/// Maps depthai's calibration onto a ROS distortion model, and trims the
/// coefficient vector to the length that model is defined for.
///
/// `getDistortionCoefficients` hands back fourteen numbers for a perspective
/// camera, in OpenCV's order: `k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4 tx ty`. ROS
/// has a name for the first five of those (`plumb_bob`) and for the first eight
/// (`rational_polynomial`), and no name at all for the thin-prism `s` terms or
/// the tilted-sensor `t` terms. So the model is chosen by which coefficients are
/// actually non-zero rather than by the length of the vector: claiming
/// plumb_bob while k4..k6 are populated would make every consumer undistort with
/// a different model than the one the factory measured.
pub fn distortion_for(model: OakCameraModel, coefficients: &[f64]) -> (DistortionModel, Vec<f64>) {
    let at = |index: usize| coefficients.get(index).copied().unwrap_or(0.0);
    let all_zero = |range: std::ops::Range<usize>| range.map(at).all(|value| value == 0.0);

    match model {
        OakCameraModel::Perspective => {
            // s1..s4, tx, ty have no ROS spelling at all.
            if !all_zero(8..14) {
                return (DistortionModel::Unknown, coefficients.to_vec());
            }
            if all_zero(5..8) {
                (DistortionModel::PlumbBob, (0..5).map(at).collect())
            } else {
                (DistortionModel::RationalPolynomial, (0..8).map(at).collect())
            }
        }
        OakCameraModel::Fisheye => (DistortionModel::Equidistant, (0..4).map(at).collect()),
        OakCameraModel::Equirectangular
        | OakCameraModel::RadialDivision
        | OakCameraModel::Unrecognised(_) => (DistortionModel::Unknown, coefficients.to_vec()),
    }
}

/// Every length in a depthai calibration is centimetres. ROS is metres
/// everywhere, so nothing may leave this file without passing through here.
pub const CENTIMETRES_PER_METRE: f64 = 100.0;

pub fn metres_from_centimetres(centimetres: f64) -> f64 {
    centimetres / CENTIMETRES_PER_METRE
}

/// One imager's factory calibration, at the resolution being streamed.
///
/// `intrinsics` is the row-major 3x3 depthai returns, already scaled by the SDK
/// to that resolution. The baseline term belongs only on the right imager,
/// exactly as on a RealSense.
pub struct StreamCalibration<'a> {
    pub width: u32,
    pub height: u32,
    pub intrinsics: [f64; 9],
    pub model: OakCameraModel,
    pub coefficients: &'a [f64],
    pub baseline_centimetres: f64,
}

/// Builds the CameraInfo that goes alongside an image stream.
pub fn camera_info(
    stamp_nanos: u64,
    frame_id: &str,
    calibration: &StreamCalibration<'_>,
) -> CameraInfo {
    let (distortion_model, trimmed) =
        distortion_for(calibration.model, calibration.coefficients);
    CameraInfo::pinhole(
        Header::new(stamp_nanos, frame_id),
        calibration.width,
        calibration.height,
        calibration.intrinsics[0],
        calibration.intrinsics[4],
        calibration.intrinsics[2],
        calibration.intrinsics[5],
        distortion_model,
        trimmed,
        metres_from_centimetres(calibration.baseline_centimetres),
    )
}

/// Turns a depthai timestamp into the epoch nanoseconds a ROS header carries.
///
/// `dai::Buffer::getTimestamp()` returns a `steady_clock` time point: the
/// camera's own timestamp for the frame, already translated onto the host's
/// monotonic clock by depthai's XLink clock synchronisation. It is therefore a
/// sensor time, not an arrival time, but it is not an epoch and cannot be
/// written into a header as it stands.
///
/// The offset between the two host clocks is sampled once, when streaming
/// starts, and then held. Re-sampling it per frame would be more accurate
/// against wall-clock but would fold the system clock's own corrections — NTP
/// slew, a step from a settling RTC — into the spacing between consecutive
/// headers, which is precisely the sensor timing this exists to preserve. A
/// fixed offset means every interval between headers is the camera's, and only
/// the absolute placement of the whole run can drift.
#[derive(Debug, Clone, Copy)]
pub struct SteadyToEpoch {
    offset_nanos: i128,
}

impl SteadyToEpoch {
    /// Both readings must be taken as close together as the host allows: their
    /// separation is the whole error in the result.
    pub fn sample(steady_nanos: u64, epoch_nanos: u64) -> Self {
        SteadyToEpoch {
            offset_nanos: epoch_nanos as i128 - steady_nanos as i128,
        }
    }

    pub fn epoch_for(&self, steady_nanos: u64) -> u64 {
        (steady_nanos as i128 + self.offset_nanos).max(0) as u64
    }
}

/// Holds one stream's header stamps strictly increasing.
///
/// A fixed offset preserves whatever order the device produced, so this is not
/// correcting a drifting estimate the way the realsense path has to. It closes
/// the remaining case: a device that repeats or reorders a stamp on one stream.
/// A duplicate is the damaging one — readers index on the header stamp, so two
/// frames sharing one become indistinguishable and a seek can land on either.
#[derive(Debug, Default, Clone, Copy)]
pub struct StrictlyIncreasing {
    last_nanos: Option<u64>,
}

impl StrictlyIncreasing {
    /// Returns `stamp_nanos` when it is genuinely ahead, and the smallest stamp
    /// that is still ahead when it is not.
    pub fn next(&mut self, stamp_nanos: u64) -> u64 {
        let stamp = match self.last_nanos {
            Some(last) if stamp_nanos <= last => last + 1,
            _ => stamp_nanos,
        };
        self.last_nanos = Some(stamp);
        stamp
    }
}

/// The rates an OAK-D Pro's BMI270 will accept for both of its parts. Asking for
/// anything else gets silently rounded by the firmware, so the request is
/// matched here instead and the achieved rate is what gets reported.
pub const BMI270_RATES_HZ: [u32; 6] = [15, 25, 50, 100, 200, 400];

/// A request exactly between two available rates rounds up. The rates are an
/// octave apart at the top of the table, so rounding down from 300 would halve
/// the requested rate, while rounding up costs a third more messages.
pub fn nearest_imu_rate(requested: u32) -> u32 {
    *BMI270_RATES_HZ
        .iter()
        .rev()
        .min_by_key(|rate| rate.abs_diff(requested))
        .expect("the rate table is never empty")
}

/// The transforms inside the camera body. Built here rather than shared with the
/// RealSense version because the parent frame differs: depth on an OAK-D is
/// computed in the left mono imager's frame, so that imager is the root every
/// other optical frame hangs off.
pub struct BodyExtrinsic {
    pub child: StreamId,
    pub rotation: [f64; 9],
    /// As depthai reports it, in centimetres.
    pub translation_centimetres: [f64; 3],
}

pub fn body_transforms(
    naming: &Naming,
    stamp_nanos: u64,
    extrinsics: &[BodyExtrinsic],
) -> Vec<crate::msgs::TransformStamped> {
    let parent = naming.frame_id(StreamId::InfraLeft);
    let mut root = crate::msgs::TransformStamped::identity(&naming.root_frame_id(), &parent);
    root.header = Header::new(stamp_nanos, naming.root_frame_id());
    let mut transforms = vec![root];
    for extrinsic in extrinsics {
        transforms.push(crate::msgs::TransformStamped {
            header: Header::new(stamp_nanos, parent.clone()),
            child_frame_id: naming.frame_id(extrinsic.child),
            translation: extrinsic.translation_centimetres.map(metres_from_centimetres),
            rotation: crate::msgs::quaternion_from_matrix(extrinsic.rotation),
        });
    }
    transforms
}

#[cfg(not(feature = "oakd"))]
mod backend {
    use super::*;
    use crate::sensors::{Backend, BackendStatus, Sink};
    use anyhow::Result;

    pub struct OakdBackend {
        pub config: CameraConfig,
    }

    impl OakdBackend {
        pub fn new(config: CameraConfig) -> Self {
            OakdBackend { config }
        }
    }

    impl Backend for OakdBackend {
        fn start(&mut self, _sink: Sink) -> Result<()> {
            anyhow::bail!(
                "this binary was built without the `oakd` feature, so depthai-core is not linked in"
            )
        }

        fn stop(&mut self) {}

        fn status(&self) -> BackendStatus {
            BackendStatus {
                running: false,
                detail: "not compiled in".into(),
                error: Some("rebuild with --features oakd".into()),
            }
        }
    }
}

#[cfg(feature = "oakd")]
#[path = "oakd_ffi.rs"]
mod backend;

pub use backend::OakdBackend;

impl OakdBackend {
    pub fn config(&self) -> &CameraConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::{Backend, SensorKind};

    #[test]
    fn the_stereo_pair_is_addressed_by_socket_and_depth_shares_the_left_one() {
        assert_eq!(socket_for(StreamId::Color), Some(CAM_A));
        assert_eq!(socket_for(StreamId::InfraLeft), Some(CAM_B));
        assert_eq!(socket_for(StreamId::InfraRight), Some(CAM_C));
        // Depth is rectified into the left imager, so borrowing CAM_C's
        // intrinsics for it would shift every reprojected point by the baseline.
        assert_eq!(socket_for(StreamId::Depth), Some(CAM_B));
        assert_eq!(socket_for(StreamId::Imu), None);
    }

    #[test]
    fn a_perspective_camera_with_only_the_first_five_terms_is_plumb_bob() {
        let coefficients = vec![-0.05, 0.12, 0.0002, -0.0001, -0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (model, trimmed) = distortion_for(OakCameraModel::Perspective, &coefficients);
        assert_eq!(model, DistortionModel::PlumbBob);
        // plumb_bob is defined for exactly five, in this order.
        assert_eq!(trimmed, vec![-0.05, 0.12, 0.0002, -0.0001, -0.04]);
    }

    #[test]
    fn populated_k4_to_k6_are_reported_as_rational_polynomial_not_plumb_bob() {
        // Claiming plumb_bob here throws away three real coefficients and makes
        // every consumer undistort with a model the factory did not measure.
        let mut coefficients = vec![0.0; 14];
        coefficients[0] = -0.05;
        coefficients[5] = 0.3;
        let (model, trimmed) = distortion_for(OakCameraModel::Perspective, &coefficients);
        assert_eq!(model, DistortionModel::RationalPolynomial);
        assert_eq!(trimmed.len(), 8);
        assert_eq!(trimmed[5], 0.3);
    }

    #[test]
    fn thin_prism_terms_have_no_ros_name_and_are_admitted_unknown() {
        let mut coefficients = vec![0.0; 14];
        coefficients[8] = 0.001;
        let (model, kept) = distortion_for(OakCameraModel::Perspective, &coefficients);
        assert_eq!(model, DistortionModel::Unknown);
        // Nothing is dropped when the model is unknown; a consumer that knows
        // OpenCV's fourteen-term form can still use them.
        assert_eq!(kept.len(), 14);
    }

    #[test]
    fn a_fisheye_camera_is_equidistant_with_four_coefficients() {
        let (model, trimmed) =
            distortion_for(OakCameraModel::Fisheye, &[0.1, -0.2, 0.03, -0.004]);
        assert_eq!(model, DistortionModel::Equidistant);
        assert_eq!(trimmed, vec![0.1, -0.2, 0.03, -0.004]);
    }

    /// The shim's constants, copied from `oakd_shim.cpp`. A mismatch would file
    /// one imager's frames under another imager's topic with nothing failing, so
    /// the two lists are compared rather than trusted.
    #[test]
    fn the_stream_indices_match_the_shim() {
        assert_eq!(shim_stream_index(StreamId::Depth), Some(0));
        assert_eq!(shim_stream_index(StreamId::Color), Some(1));
        assert_eq!(shim_stream_index(StreamId::InfraLeft), Some(2));
        assert_eq!(shim_stream_index(StreamId::InfraRight), Some(3));
        assert_eq!(shim_stream_index(StreamId::Imu), Some(4));
        // The OAK-D publishes no cloud of its own; depth is the raster.
        assert_eq!(shim_stream_index(StreamId::PointCloud), None);
    }

    #[test]
    fn depth_is_announced_as_a_depth_rather_than_as_a_grey_image() {
        // Same two bytes per pixel as mono16, but only this spelling tells a
        // consumer the values are millimetres rather than brightness.
        assert_eq!(ros_encoding(1), Some(("16UC1", 2)));
        assert_eq!(ros_encoding(0), Some(("mono8", 1)));
        assert_eq!(ros_encoding(2), Some(("bgr8", 3)));
        assert_eq!(ros_encoding(7), None);
    }

    #[test]
    fn the_raw_model_values_match_depthai() {
        assert_eq!(OakCameraModel::from_raw(0), OakCameraModel::Perspective);
        assert_eq!(OakCameraModel::from_raw(1), OakCameraModel::Fisheye);
        assert_eq!(OakCameraModel::from_raw(3), OakCameraModel::RadialDivision);
        assert_eq!(OakCameraModel::from_raw(9), OakCameraModel::Unrecognised(9));
    }

    /// Intrinsics of the shape an OAK-D Pro reports for its 1280x800 left mono.
    #[test]
    fn calibration_becomes_a_usable_camera_info() {
        let info = camera_info(
            1_000_000_000,
            "oakd_infra1_optical_frame",
            &StreamCalibration {
                width: 1280,
                height: 800,
                intrinsics: [796.5, 0.0, 636.2, 0.0, 796.5, 397.4, 0.0, 0.0, 1.0],
                model: OakCameraModel::Perspective,
                coefficients: &[0.0; 14],
                baseline_centimetres: 0.0,
            },
        );
        assert_eq!(info.width, 1280);
        assert_eq!(info.distortion_model, "plumb_bob");
        assert_eq!(info.intrinsics[0], 796.5);
        assert_eq!(info.intrinsics[2], 636.2);
        assert_eq!(info.intrinsics[5], 397.4);
        assert_eq!(info.projection[3], 0.0);
    }

    #[test]
    fn the_baseline_is_converted_out_of_centimetres_before_it_reaches_p() {
        // An OAK-D Pro's stereo baseline is 7.5 cm. Passing 7.5 straight through
        // would put the right imager 7.5 *metres* away.
        let info = camera_info(
            0,
            "oakd_infra2_optical_frame",
            &StreamCalibration {
                width: 1280,
                height: 800,
                intrinsics: [796.5, 0.0, 636.2, 0.0, 796.5, 397.4, 0.0, 0.0, 1.0],
                model: OakCameraModel::Perspective,
                coefficients: &[0.0; 14],
                baseline_centimetres: 7.5,
            },
        );
        assert!((info.projection[3] + 796.5 * 0.075).abs() < 1e-9);
    }

    #[test]
    fn factory_extrinsics_are_metres_under_the_left_imager() {
        let naming = Naming::for_kind(SensorKind::OakD);
        let transforms = body_transforms(
            &naming,
            5,
            &[BodyExtrinsic {
                child: StreamId::Color,
                rotation: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                translation_centimetres: [3.75, 0.0, 0.0],
            }],
        );
        assert_eq!(transforms.len(), 2);
        assert_eq!(transforms[0].header.frame_id, "oakd_link");
        assert_eq!(transforms[0].child_frame_id, "oakd_infra1_optical_frame");

        let color = &transforms[1];
        assert_eq!(color.header.frame_id, "oakd_infra1_optical_frame");
        assert_eq!(color.child_frame_id, "oakd_color_optical_frame");
        // 3.75 cm, which is 37.5 mm and not 3.75 m.
        assert!((color.translation[0] - 0.0375).abs() < 1e-12);
    }

    #[test]
    fn a_device_stamp_keeps_its_own_spacing_after_being_offset_onto_the_epoch() {
        // The host booted 12 s ago; the wall clock says 2024-ish.
        let clock = SteadyToEpoch::sample(12_000_000_000, 1_700_000_000_000_000_000);
        let first = clock.epoch_for(12_100_000_000);
        let second = clock.epoch_for(12_133_333_333);
        assert_eq!(first, 1_700_000_000_100_000_000);
        // 33.333333 ms apart on the device is 33.333333 ms apart in the header.
        assert_eq!(second - first, 33_333_333);
    }

    #[test]
    fn the_offset_survives_a_steady_clock_larger_than_the_wall_clock() {
        // Nothing says the monotonic clock starts at zero; on some hosts it is
        // an uptime counter that has already passed the epoch value in tests.
        let clock = SteadyToEpoch::sample(2_000_000_000_000_000_000, 1_700_000_000_000_000_000);
        assert_eq!(
            clock.epoch_for(2_000_000_001_000_000_000),
            1_700_000_001_000_000_000
        );
    }

    #[test]
    fn a_repeated_device_stamp_is_never_written_to_two_headers() {
        // The damaging case, and the one a plain `>` comparison would let
        // through: two frames carrying one stamp are indistinguishable to any
        // reader that indexes on it.
        let mut stamps = StrictlyIncreasing::default();
        assert_eq!(stamps.next(1_000), 1_000);
        assert_eq!(stamps.next(1_000), 1_001);
        assert_eq!(stamps.next(1_000), 1_002);
    }

    #[test]
    fn a_backwards_device_stamp_does_not_walk_the_stream_backwards() {
        let mut stamps = StrictlyIncreasing::default();
        assert_eq!(stamps.next(5_000), 5_000);
        assert_eq!(stamps.next(4_000), 5_001);
        // Once the device is ahead again its own stamps are used unchanged,
        // rather than the stream staying stuck on the nudged values.
        assert_eq!(stamps.next(6_000), 6_000);
    }

    #[test]
    fn every_stamp_a_stream_emits_is_strictly_greater_than_the_last() {
        // Whatever the device does, the emitted sequence has to be usable. This
        // is the property the guard exists for, so it is asserted directly.
        let mut stamps = StrictlyIncreasing::default();
        let device_stamps = [10, 20, 20, 19, 21, 21, 100, 99, 100, 5];
        let emitted: Vec<u64> = device_stamps.iter().map(|s| stamps.next(*s)).collect();
        assert!(
            emitted.windows(2).all(|pair| pair[1] > pair[0]),
            "not strictly increasing: {emitted:?}"
        );
    }

    #[test]
    fn an_imu_rate_is_matched_to_one_the_part_actually_has() {
        // 200 Hz is exact, and matching the Mid-360 means the two inertial
        // streams in one recording line up.
        assert_eq!(nearest_imu_rate(200), 200);
        assert_eq!(nearest_imu_rate(250), 200);
        // Exactly between 200 and 400, and rounding down would halve it.
        assert_eq!(nearest_imu_rate(300), 400);
        assert_eq!(nearest_imu_rate(1), 15);
        assert_eq!(nearest_imu_rate(100_000), 400);
    }

    #[test]
    fn without_the_feature_the_backend_says_so_instead_of_pretending() {
        let mut backend = OakdBackend::new(CameraConfig::for_kind(SensorKind::OakD));
        assert!(!backend.status().running);
        if !cfg!(feature = "oakd") {
            let error = backend.start(std::sync::Arc::new(|_| true)).unwrap_err();
            assert!(format!("{error:#}").contains("oakd"));
        }
    }
}
