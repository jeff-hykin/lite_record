//! Plain in-memory shapes for the ROS2 messages this recorder produces. Sensor
//! backends fill these in and `crate::cdr` turns them into the bytes that land in
//! the mcap file.

pub const IMAGE_TYPE: &str = "sensor_msgs/msg/Image";
pub const COMPRESSED_IMAGE_TYPE: &str = "sensor_msgs/msg/CompressedImage";
pub const TF_TYPE: &str = "tf2_msgs/msg/TFMessage";
pub const POINT_CLOUD2_TYPE: &str = "sensor_msgs/msg/PointCloud2";
pub const IMU_TYPE: &str = "sensor_msgs/msg/Imu";
pub const CAMERA_INFO_TYPE: &str = "sensor_msgs/msg/CameraInfo";
pub const ODOMETRY_TYPE: &str = "nav_msgs/msg/Odometry";
pub const NAV_SAT_FIX_TYPE: &str = "sensor_msgs/msg/NavSatFix";
pub const STRING_TYPE: &str = "std_msgs/msg/String";

pub const NANOS_PER_SEC: u64 = 1_000_000_000;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Header {
    pub stamp_sec: i32,
    pub stamp_nsec: i32,
    pub frame_id: String,
}

impl Header {
    pub fn new(stamp_nanos: u64, frame_id: impl Into<String>) -> Self {
        Header {
            stamp_sec: (stamp_nanos / NANOS_PER_SEC) as i32,
            stamp_nsec: (stamp_nanos % NANOS_PER_SEC) as i32,
            frame_id: frame_id.into(),
        }
    }

    pub fn stamp_nanos(&self) -> u64 {
        self.stamp_sec as u64 * NANOS_PER_SEC + self.stamp_nsec as u64
    }
}

#[derive(Clone, Debug)]
pub struct RawImage {
    pub header: Header,
    pub width: usize,
    pub height: usize,
    pub step: usize,
    pub is_bigendian: u8,
    pub encoding: String,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String,
    pub data: Vec<u8>,
}

pub enum ImageMessage {
    Raw(RawImage),
    Compressed(CompressedImage),
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}

/// ROS2 only defines a handful of distortion model strings. `librealsense` and the
/// Orbbec SDK both expose models that have no ROS equivalent, so backends must map
/// deliberately rather than guessing — see `DistortionModel::from_realsense`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DistortionModel {
    PlumbBob,
    RationalPolynomial,
    Equidistant,
    /// The sensor reported a model with no ROS2 spelling. Recorded verbatim as
    /// `"unknown"` so a downstream consumer cannot silently mis-rectify.
    Unknown,
}

impl DistortionModel {
    pub fn as_str(self) -> &'static str {
        match self {
            DistortionModel::PlumbBob => "plumb_bob",
            DistortionModel::RationalPolynomial => "rational_polynomial",
            DistortionModel::Equidistant => "equidistant",
            DistortionModel::Unknown => "unknown",
        }
    }
}

#[derive(Clone, Debug)]
pub struct CameraInfo {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: String,
    pub distortion: Vec<f64>,
    /// Row-major 3x3 `K`.
    pub intrinsics: [f64; 9],
    /// Row-major 3x3 `R`, identity for an unrectified stream.
    pub rectification: [f64; 9],
    /// Row-major 3x4 `P`.
    pub projection: [f64; 12],
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: RegionOfInterest,
}

impl CameraInfo {
    /// Builds `K`, `R` and `P` from a pinhole model. `baseline_meters` is the
    /// stereo offset of this camera from the left one, which ROS folds into
    /// `P[3]` as `-fx * baseline`; it is zero for every non-right camera.
    #[allow(clippy::too_many_arguments)]
    pub fn pinhole(
        header: Header,
        width: u32,
        height: u32,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        model: DistortionModel,
        coefficients: Vec<f64>,
        baseline_meters: f64,
    ) -> Self {
        CameraInfo {
            header,
            height,
            width,
            distortion_model: model.as_str().to_string(),
            distortion: coefficients,
            intrinsics: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            rectification: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            projection: [
                fx,
                0.0,
                cx,
                -fx * baseline_meters,
                0.0,
                fy,
                cy,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
            binning_x: 0,
            binning_y: 0,
            roi: RegionOfInterest::default(),
        }
    }
}

/// `sensor_msgs/PointField` datatype codes.
pub const POINT_FIELD_UINT8: u8 = 2;
pub const POINT_FIELD_UINT16: u8 = 4;
pub const POINT_FIELD_UINT32: u8 = 6;
pub const POINT_FIELD_FLOAT32: u8 = 7;
pub const POINT_FIELD_FLOAT64: u8 = 8;

#[derive(Clone, Debug, PartialEq)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Clone, Debug)]
pub struct PointCloud2 {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

/// `sensor_msgs/NavSatFix`, with `NavSatStatus` flattened into its two fields.
#[derive(Clone, Debug, PartialEq)]
pub struct NavSatFix {
    pub header: Header,
    pub status: i8,
    pub service: u16,
    pub latitude: f64,
    pub longitude: f64,
    /// Above the WGS84 ellipsoid, not sea level.
    pub altitude: f64,
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8,
}

#[derive(Clone, Debug)]
pub struct Imu {
    pub header: Header,
    pub orientation: [f64; 4],
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: [f64; 3],
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: [f64; 3],
    pub linear_acceleration_covariance: [f64; 9],
}

impl Imu {
    /// A `-1` in the first covariance slot is the ROS convention for "this field
    /// is not measured", which is true of orientation on every sensor we record.
    pub fn unoriented(
        header: Header,
        angular_velocity: [f64; 3],
        linear_acceleration: [f64; 3],
    ) -> Self {
        let mut orientation_covariance = [0.0; 9];
        orientation_covariance[0] = -1.0;
        Imu {
            header,
            orientation: [0.0, 0.0, 0.0, 1.0],
            orientation_covariance,
            angular_velocity,
            angular_velocity_covariance: [0.0; 9],
            linear_acceleration,
            linear_acceleration_covariance: [0.0; 9],
        }
    }
}

/// `nav_msgs/Odometry`: the pose of `child_frame_id` in `header.frame_id`,
/// with the twist in the child frame. Covariances are written as zeros.
#[derive(Clone, Debug, PartialEq)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub linear_velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

impl TransformStamped {
    pub fn identity(parent: &str, child: &str) -> Self {
        TransformStamped {
            header: Header::new(0, parent),
            child_frame_id: child.to_string(),
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    pub fn parent(&self) -> &str {
        &self.header.frame_id
    }
}

/// Converts a row-major 3x3 rotation matrix to `[x, y, z, w]`.
///
/// Uses the largest-diagonal branch rather than the naive `w`-first formula,
/// which loses all precision when the trace approaches -1 (a 180-degree turn).
/// The result is renormalised because a factory extrinsic read off a camera is
/// only orthonormal to the precision it was stored at, and a quaternion that is
/// off by 1e-5 makes tf2 refuse the transform outright.
pub fn quaternion_from_matrix(rotation: [f64; 9]) -> [f64; 4] {
    let quaternion = quaternion_from_matrix_unnormalised(rotation);
    let norm = quaternion
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if norm == 0.0 {
        return [0.0, 0.0, 0.0, 1.0];
    }
    quaternion.map(|value| value / norm)
}

fn quaternion_from_matrix_unnormalised(rotation: [f64; 9]) -> [f64; 4] {
    let (m00, m01, m02) = (rotation[0], rotation[1], rotation[2]);
    let (m10, m11, m12) = (rotation[3], rotation[4], rotation[5]);
    let (m20, m21, m22) = (rotation[6], rotation[7], rotation[8]);
    let trace = m00 + m11 + m22;
    if trace > 0.0 {
        let scale = (trace + 1.0).sqrt() * 2.0;
        [
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        ]
    } else if m00 > m11 && m00 > m22 {
        let scale = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        [
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        ]
    } else if m11 > m22 {
        let scale = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        [
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        ]
    } else {
        let scale = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        [
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        ]
    }
}

/// Roll-pitch-yaw as URDF spells it: intrinsic X then Y then Z.
pub fn quaternion_from_rpy(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sin_roll, cos_roll) = (roll / 2.0).sin_cos();
    let (sin_pitch, cos_pitch) = (pitch / 2.0).sin_cos();
    let (sin_yaw, cos_yaw) = (yaw / 2.0).sin_cos();
    [
        sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
        cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_round_trips_nanoseconds() {
        let header = Header::new(1_234_567_890_123, "camera");
        assert_eq!(header.stamp_sec, 1234);
        assert_eq!(header.stamp_nsec, 567_890_123);
        assert_eq!(header.stamp_nanos(), 1_234_567_890_123);
    }

    #[test]
    fn the_right_camera_gets_the_baseline_folded_into_p() {
        let info = CameraInfo::pinhole(
            Header::new(0, "ir_right"),
            848,
            480,
            420.0,
            420.0,
            424.0,
            240.0,
            DistortionModel::PlumbBob,
            vec![0.0; 5],
            0.05,
        );
        assert_eq!(info.projection[3], -21.0);
        assert_eq!(info.intrinsics, [420.0, 0.0, 424.0, 0.0, 420.0, 240.0, 0.0, 0.0, 1.0]);
        assert_eq!(info.rectification[0], 1.0);
    }

    #[test]
    fn identity_rotation_is_the_identity_quaternion() {
        let quaternion = quaternion_from_matrix([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
        assert!((quaternion[3] - 1.0).abs() < 1e-12);
        assert!(quaternion[..3].iter().all(|value| value.abs() < 1e-12));
    }

    /// The naive trace formula divides by zero here, which is exactly the case
    /// the largest-diagonal branch exists to handle.
    #[test]
    fn a_half_turn_stays_finite_and_normalised() {
        let quaternion = quaternion_from_matrix([-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]);
        let norm: f64 = quaternion.iter().map(|value| value * value).sum();
        assert!((norm - 1.0).abs() < 1e-12, "not normalised: {quaternion:?}");
        assert!((quaternion[2].abs() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn rpy_and_matrix_agree_on_a_quarter_turn_about_z() {
        let from_rpy = quaternion_from_rpy(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let from_matrix =
            quaternion_from_matrix([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
        for axis in 0..4 {
            assert!(
                (from_rpy[axis] - from_matrix[axis]).abs() < 1e-12,
                "axis {axis}: {from_rpy:?} vs {from_matrix:?}"
            );
        }
    }

    #[test]
    fn an_unmeasured_orientation_is_flagged_with_negative_one() {
        let imu = Imu::unoriented(Header::new(0, "imu"), [0.1, 0.2, 0.3], [0.0, 0.0, 9.81]);
        assert_eq!(imu.orientation_covariance[0], -1.0);
        assert_eq!(imu.linear_acceleration[2], 9.81);
    }
}
