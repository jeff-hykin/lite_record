//! Point-LIO configuration. Defaults mirror `parameters.cpp` / `avia.yaml`.
//! [`Config::from_point_lio_yaml`] parses Point-LIO's nested YAML schema
//! (`common` / `preprocess` / `mapping`); unknown keys are ignored and any
//! missing key falls back to the default.

use crate::types::{M3D, V3D};
use serde::Deserialize;

#[derive(Clone, Debug)]
pub struct Config {
    // ---- estimator mode ----
    /// Fuse IMU (always true for the datasets we target).
    pub imu_en: bool,
    /// `false` => 30-DOF output model (default/recommended); `true` => 24-DOF input model.
    pub use_imu_as_input: bool,
    /// Propagate covariance only at IMU timestamps (default true).
    pub prop_at_freq_of_imu: bool,
    /// Estimate the LiDAR->IMU extrinsic online.
    pub extrinsic_est_en: bool,
    /// Clamp IMU residuals when the sensor saturates.
    pub check_satu: bool,

    // ---- extrinsic (LiDAR in IMU frame) ----
    pub lidar_to_imu_rot: M3D,
    pub lidar_to_imu_trans: V3D,

    // ---- gravity / IMU units ----
    pub gravity: V3D,
    pub gravity_init: V3D,
    /// IMU accel unit: 1.0 if accel is in g, 9.81 if in m/s^2.
    pub acc_norm: f64,
    pub satu_acc: f64,
    pub satu_gyro: f64,

    // ---- process noise ----
    pub vel_cov: f64,
    pub acc_cov_output: f64,
    pub gyr_cov_output: f64,
    pub acc_cov_input: f64,
    pub gyr_cov_input: f64,
    pub b_acc_cov: f64,
    pub b_gyr_cov: f64,

    // ---- measurement noise ----
    pub lidar_meas_cov: f64,
    pub imu_meas_acc_cov: f64,
    pub imu_meas_omg_cov: f64,

    // ---- point-to-plane matching ----
    pub plane_thr: f64,
    pub match_s: f64,
    pub num_match_points: usize,

    // ---- map / downsample ----
    pub filter_size_surf: f64,
    pub filter_size_map: f64,
    pub ivox_resolution: f64,
    pub ivox_nearby_type: i32,
    pub init_map_size: usize,

    // ---- preprocessing / ingestion ----
    pub blind: f64,
    pub max_range: f64,
    pub point_filter_num: i32,
    pub imu_init_num: usize,

    /// Seconds added to each point's timestamp before it is matched against the
    /// IMU-propagated state (`common.time_diff_lidar_to_imu`). The point-wise
    /// filter is sensitive to LiDAR/IMU clock skew; a small offset removes the
    /// motion-correlated bias it would otherwise absorb.
    pub time_offset_lidar_to_imu: f64,

    /// Velocity-cap guardrail (m/s). If a scan's post-update speed exceeds this,
    /// the scan is rejected and the state rolled back to the last good one
    /// (mirrors the sibling FAST-LIO `MapBuilder`). 0 disables the guard.
    pub max_velocity: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            imu_en: true,
            use_imu_as_input: false,
            prop_at_freq_of_imu: true,
            extrinsic_est_en: false,
            check_satu: true,
            lidar_to_imu_rot: M3D::identity(),
            lidar_to_imu_trans: V3D::zeros(),
            gravity: V3D::new(0.0, 0.0, -9.81),
            gravity_init: V3D::new(0.0, 0.0, -9.81),
            acc_norm: 9.81,
            // In the SAME units as `acc_norm` and `gravity` above. Point-LIO's
            // published value is 3.0 because the Avia reports acceleration in
            // g; against m/s² data a 3.0 clamp saturates on gravity alone, and
            // a saturated axis has its accelerometer update thrown away.
            satu_acc: 3.0 * 9.81,
            satu_gyro: 35.0,
            vel_cov: 20.0,
            acc_cov_output: 500.0,
            gyr_cov_output: 1000.0,
            acc_cov_input: 0.1,
            gyr_cov_input: 0.01,
            b_acc_cov: 0.0001,
            b_gyr_cov: 0.0001,
            lidar_meas_cov: 0.01,
            imu_meas_acc_cov: 0.1,
            imu_meas_omg_cov: 0.1,
            plane_thr: 0.1,
            match_s: 81.0,
            num_match_points: 5,
            filter_size_surf: 0.5,
            filter_size_map: 0.5,
            // Must not exceed `filter_size_map`. `map_incremental` keeps at
            // most one point per `filter_size_map` cell, and that is the only
            // thing bounding how many points a voxel holds — while `closest`
            // pays linearly for every point in all 19 voxels of its stencil.
            // A voxel `n` times the cell width holds `n^3` times the points,
            // so 2.0 against a 0.5 m cell is 64x the neighbour search for the
            // same map. Upstream's avia/mid360 configs match the two.
            ivox_resolution: 0.5,
            ivox_nearby_type: 18,
            init_map_size: 10,
            blind: 0.5,
            max_range: 100.0,
            point_filter_num: 1,
            imu_init_num: 100,
            time_offset_lidar_to_imu: 0.0,
            max_velocity: 0.0,
        }
    }
}

impl Config {
    pub fn from_point_lio_yaml(s: &str) -> Result<Config, serde_yaml::Error> {
        let raw: RawConfig = serde_yaml::from_str(s)?;
        Ok(raw.into_config())
    }

    pub fn from_yaml_path<P: AsRef<std::path::Path>>(path: P) -> std::io::Result<Config> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_point_lio_yaml(&contents)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }

    /// Convenience constructor for the Unitree Go2 / Livox Mid-360 rig used by
    /// the `~/datasets/go2_recordings` captures. Extrinsic and ingestion
    /// settings match the validated FAST-LIO `mid360.yaml`.
    pub fn go2_mid360() -> Config {
        let mut c = Config::default();
        c.lidar_to_imu_trans = V3D::new(-0.011, -0.02329, 0.04412);
        c.lidar_to_imu_rot = M3D::identity();
        c.blind = 0.5;
        c.max_range = 60.0;
        c.point_filter_num = 3;
        c.filter_size_surf = 0.5;
        c.filter_size_map = 0.5;
        c.ivox_resolution = 0.5;
        c.acc_norm = 9.81;
        c.satu_gyro = 35.0;
        // Output model — Point-LIO's accurate, high-bandwidth mode (handles the
        // gait/vibration that is the whole point of point-wise LIO). No velocity
        // guardrail.
        c.use_imu_as_input = false;
        c
    }
}

fn vec_to_m3d(v: &[f64]) -> Option<M3D> {
    (v.len() == 9).then(|| M3D::from_row_slice(v))
}
fn vec_to_v3d(v: &[f64]) -> Option<V3D> {
    (v.len() == 3).then(|| V3D::new(v[0], v[1], v[2]))
}

#[derive(Deserialize, Default)]
struct RawConfig {
    common: Option<Common>,
    preprocess: Option<Preprocess>,
    mapping: Option<Mapping>,
}

#[derive(Deserialize, Default)]
struct Common {
    time_diff_lidar_to_imu: Option<f64>,
}

#[derive(Deserialize, Default)]
struct Preprocess {
    blind: Option<f64>,
    point_filter_num: Option<i32>,
}

#[derive(Deserialize, Default)]
struct Mapping {
    imu_en: Option<bool>,
    use_imu_as_input: Option<bool>,
    prop_at_freq_of_imu: Option<bool>,
    extrinsic_est_en: Option<bool>,
    check_satu: Option<bool>,
    acc_norm: Option<f64>,
    satu_acc: Option<f64>,
    satu_gyro: Option<f64>,
    lidar_meas_cov: Option<f64>,
    acc_cov_output: Option<f64>,
    gyr_cov_output: Option<f64>,
    acc_cov_input: Option<f64>,
    gyr_cov_input: Option<f64>,
    vel_cov: Option<f64>,
    b_acc_cov: Option<f64>,
    b_gyr_cov: Option<f64>,
    imu_meas_acc_cov: Option<f64>,
    imu_meas_omg_cov: Option<f64>,
    plane_thr: Option<f64>,
    match_s: Option<f64>,
    ivox_grid_resolution: Option<f64>,
    init_map_size: Option<usize>,
    gravity: Option<Vec<f64>>,
    gravity_init: Option<Vec<f64>>,
    extrinsic_t: Option<Vec<f64>>,
    extrinsic_r: Option<Vec<f64>>,
    filter_size_surf: Option<f64>,
    filter_size_map: Option<f64>,
    max_velocity: Option<f64>,
}

impl RawConfig {
    fn into_config(self) -> Config {
        let mut c = Config::default();
        let common = self.common.unwrap_or_default();
        let pre = self.preprocess.unwrap_or_default();
        let map = self.mapping.unwrap_or_default();
        if let Some(v) = common.time_diff_lidar_to_imu { c.time_offset_lidar_to_imu = v; }
        if let Some(v) = pre.blind { c.blind = v; }
        if let Some(v) = pre.point_filter_num { c.point_filter_num = v; }
        if let Some(v) = map.imu_en { c.imu_en = v; }
        if let Some(v) = map.use_imu_as_input { c.use_imu_as_input = v; }
        if let Some(v) = map.prop_at_freq_of_imu { c.prop_at_freq_of_imu = v; }
        if let Some(v) = map.extrinsic_est_en { c.extrinsic_est_en = v; }
        if let Some(v) = map.check_satu { c.check_satu = v; }
        if let Some(v) = map.acc_norm { c.acc_norm = v; }
        if let Some(v) = map.satu_acc { c.satu_acc = v; }
        if let Some(v) = map.satu_gyro { c.satu_gyro = v; }
        if let Some(v) = map.lidar_meas_cov { c.lidar_meas_cov = v; }
        if let Some(v) = map.acc_cov_output { c.acc_cov_output = v; }
        if let Some(v) = map.gyr_cov_output { c.gyr_cov_output = v; }
        if let Some(v) = map.acc_cov_input { c.acc_cov_input = v; }
        if let Some(v) = map.gyr_cov_input { c.gyr_cov_input = v; }
        if let Some(v) = map.vel_cov { c.vel_cov = v; }
        if let Some(v) = map.b_acc_cov { c.b_acc_cov = v; }
        if let Some(v) = map.b_gyr_cov { c.b_gyr_cov = v; }
        if let Some(v) = map.imu_meas_acc_cov { c.imu_meas_acc_cov = v; }
        if let Some(v) = map.imu_meas_omg_cov { c.imu_meas_omg_cov = v; }
        if let Some(v) = map.plane_thr { c.plane_thr = v; }
        if let Some(v) = map.match_s { c.match_s = v; }
        if let Some(v) = map.ivox_grid_resolution { c.ivox_resolution = v; }
        if let Some(v) = map.init_map_size { c.init_map_size = v; }
        if let Some(v) = map.filter_size_surf { c.filter_size_surf = v; }
        if let Some(v) = map.filter_size_map { c.filter_size_map = v; }
        if let Some(v) = map.max_velocity { c.max_velocity = v; }
        if let Some(v) = map.gravity.as_deref().and_then(vec_to_v3d) { c.gravity = v; }
        if let Some(v) = map.gravity_init.as_deref().and_then(vec_to_v3d) { c.gravity_init = v; }
        if let Some(v) = map.extrinsic_t.as_deref().and_then(vec_to_v3d) { c.lidar_to_imu_trans = v; }
        if let Some(v) = map.extrinsic_r.as_deref().and_then(vec_to_m3d) { c.lidar_to_imu_rot = v; }
        c
    }
}
