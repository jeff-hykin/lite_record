//! Core data types shared across the crate. A `Point` carries its per-point
//! `offset_time` (ms since the scan start) which Point-LIO uses to drive the
//! point-by-point propagation; an `ImuData` is one raw IMU sample; a
//! `SyncPackage` is one ~10 Hz LiDAR frame together with every IMU sample up
//! to the frame end.

use nalgebra::{Matrix3, Vector3, Vector4};

pub type M3D = Matrix3<f64>;
pub type V3D = Vector3<f64>;
pub type V4D = Vector4<f64>;

/// A single LiDAR return in the sensor (LiDAR) frame. `offset_time` is the
/// time of the return in milliseconds after the owning scan's start time
/// (mirrors Point-LIO's `curvature` field).
#[derive(Clone, Copy, Debug, Default)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub offset_time: f32,
}

impl Point {
    pub fn new(x: f32, y: f32, z: f32, intensity: f32, offset_time: f32) -> Self {
        Point { x, y, z, intensity, offset_time }
    }
    #[inline]
    pub fn vec3(&self) -> V3D {
        V3D::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

pub type PointCloud = Vec<Point>;

/// One raw IMU sample. `acc` is in m/s^2, `gyro` in rad/s, `time` in seconds.
#[derive(Clone, Copy, Debug)]
pub struct ImuData {
    pub acc: V3D,
    pub gyro: V3D,
    pub time: f64,
}

/// One LiDAR scan plus the IMU samples covering it.
#[derive(Clone, Debug)]
pub struct SyncPackage {
    pub imus: Vec<ImuData>,
    pub cloud: PointCloud,
    pub cloud_start_time: f64,
    pub cloud_end_time: f64,
}

/// A single estimated pose sample (the odometry output).
#[derive(Clone, Copy, Debug)]
pub struct PoseSample {
    pub time: f64,
    pub pos: V3D,
    pub rot: M3D,
    pub vel: V3D,
}
