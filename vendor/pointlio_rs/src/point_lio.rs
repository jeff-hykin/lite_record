//! The Point-LIO driver (output model, `use_imu_as_input = false` — the default
//! and recommended mode). Ported from `laserMapping.cpp` lines ~401–1015:
//! per-scan IMU initialisation, gravity-aligned bootstrap, map initialisation,
//! and the point-by-point iterated-EKF loop that interleaves IMU covariance
//! updates with per-point plane updates, followed by `MapIncremental`.

use std::collections::VecDeque;

use nalgebra::{DMatrix, DVector};

use crate::config::Config;
use crate::esekf::Esekf;
use crate::frontend;
use crate::ivox::IVox;
use crate::so3;
use crate::state::{Imu, StateOutput};
use crate::types::{ImuData, M3D, Point, PoseSample, SyncPackage, V3D};
use crate::util;

/// Most points a single Kalman update may carry. `update_plane` inverts an
/// `m x m` matrix, so the cost of a group is cubic in its size: a scan whose
/// points all share one timestamp becomes a 20 000-square inverse that never
/// returns. Splitting an oversized group changes nothing about the algorithm —
/// it already applies groups sequentially against the same linearisation.
const MAX_GROUP: usize = 64;

pub struct PointLio {
    pub cfg: Config,
    pub kf: Esekf<StateOutput>,
    pub ivox: IVox,
    q: DMatrix<f64>,
    g_m_s2: f64,

    // IMU stream cursor (mirrors the C++ imu_deque / imu_last / imu_next).
    imu_queue: VecDeque<ImuData>,
    imu_last: Option<ImuData>,
    imu_next: Option<ImuData>,

    // IMU initialisation accumulators.
    imu_need_init: bool,
    after_imu_init: bool,
    b_first_frame_imu: bool,
    init_iter_num: usize,
    mean_acc: V3D,
    mean_gyr: V3D,

    // map / scan bookkeeping.
    init_map: bool,
    init_feats_world: Vec<Point>,
    first_scan: bool,
    pub first_lidar_time: f64,
    is_first_frame: bool,
    time_update_last: f64,
    time_predict_last_const: f64,
    angvel_avr: V3D,
    acc_avr: V3D,

    last_good: Option<(StateOutput, DMatrix<f64>)>,
    pub trajectory: Vec<PoseSample>,
    pub scan_count: usize,
    pub rejected_scans: usize,
    /// Effective plane correspondences used in the most recent scan (debug).
    pub dbg_effct: usize,
    /// Number of downsampled points in the most recent scan (debug).
    pub dbg_npts: usize,
}

impl PointLio {
    pub fn new(cfg: Config) -> Self {
        let q = build_q_output(&cfg);
        let p = reset_cov_output();
        let mut x = StateOutput::default();
        x.offset_r = cfg.lidar_to_imu_rot;
        x.offset_t = cfg.lidar_to_imu_trans;
        PointLio {
            kf: Esekf::new(x, p),
            ivox: IVox::new(cfg.ivox_resolution, cfg.ivox_nearby_type),
            q,
            g_m_s2: cfg.gravity.norm(),
            cfg,
            imu_queue: VecDeque::new(),
            imu_last: None,
            imu_next: None,
            imu_need_init: true,
            after_imu_init: false,
            b_first_frame_imu: true,
            init_iter_num: 1,
            mean_acc: V3D::zeros(),
            mean_gyr: V3D::zeros(),
            init_map: false,
            init_feats_world: Vec::new(),
            first_scan: true,
            first_lidar_time: 0.0,
            is_first_frame: true,
            time_update_last: 0.0,
            time_predict_last_const: 0.0,
            angvel_avr: V3D::zeros(),
            acc_avr: V3D::zeros(),
            last_good: None,
            trajectory: Vec::new(),
            scan_count: 0,
            rejected_scans: 0,
            dbg_effct: 0,
            dbg_npts: 0,
        }
    }

    /// Transform a LiDAR-frame point to world frame using the current state.
    #[inline]
    fn body_to_world(&self, p: &Point) -> Point {
        frontend::body_to_world(&self.kf, p)
    }

    fn imu_init_accumulate(&mut self, imus: &[ImuData]) {
        if imus.is_empty() {
            return;
        }
        if self.b_first_frame_imu {
            self.init_iter_num = 1;
            self.b_first_frame_imu = false;
            self.mean_acc = imus[0].acc;
            self.mean_gyr = imus[0].gyro;
        }
        for imu in imus {
            let n = self.init_iter_num as f64;
            self.mean_acc += (imu.acc - self.mean_acc) / n;
            self.mean_gyr += (imu.gyro - self.mean_gyr) / n;
            self.init_iter_num += 1;
        }
    }

    fn pop_imu(&mut self) {
        self.imu_last = self.imu_next;
        self.imu_next = self.imu_queue.pop_front();
    }

    pub fn process(&mut self, pkg: &SyncPackage) {
        // Ingest this scan's IMU samples into the persistent stream.
        for imu in &pkg.imus {
            self.imu_queue.push_back(*imu);
        }
        if self.imu_next.is_none() {
            self.imu_next = self.imu_queue.pop_front();
            self.imu_last = self.imu_next;
        }

        // ---- first scan: set gravity, advance IMU cursor to scan start ----
        if self.first_scan {
            self.first_lidar_time = pkg.cloud_start_time;
            self.first_scan = false;
            self.kf.x.gravity = self.cfg.gravity;
            self.g_m_s2 = self.cfg.gravity.norm();
            while let Some(nx) = self.imu_next {
                if pkg.cloud_start_time > nx.time {
                    self.pop_imu();
                    if self.imu_next.is_none() {
                        break;
                    }
                } else {
                    break;
                }
            }
        }

        // ---- IMU initialisation (p_imu->Process) ----
        if self.imu_need_init {
            self.imu_init_accumulate(&pkg.imus);
            if self.init_iter_num > self.cfg.imu_init_num {
                self.imu_need_init = false;
            }
        } else if !self.after_imu_init {
            self.after_imu_init = true;
        }

        let feats_undistort = &pkg.cloud;

        // Downsample + sort by per-point time, then compress into time groups.
        let mut feats_down = util::downsample(feats_undistort, self.cfg.filter_size_surf);
        feats_down.sort_by(|a, b| a.offset_time.partial_cmp(&b.offset_time).unwrap());
        let time_seq = util::split_groups(util::time_compressing(&feats_down), MAX_GROUP);

        // ---- gravity-aligned rotation bootstrap (runs the scan init completes) ----
        if !self.after_imu_init {
            if !self.imu_need_init {
                let tmp_gravity = -self.mean_acc / self.mean_acc.norm() * self.g_m_s2;
                let rot_init = set_init_rot(&tmp_gravity, &self.cfg.gravity);
                self.kf.x.rot = rot_init;
                self.kf.x.acc = -rot_init.transpose() * self.kf.x.gravity;
                // Seed the gyro bias from the stationary IMU average. Point-LIO
                // computes `mean_gyr` during init but does not use it; FAST-LIO
                // does (`bg = gyro_mean`). On IMUs with a non-trivial gyro bias
                // (the Go2's is ~0.05 rad/s) leaving `bg = 0` lets a persistent
                // rotation-rate error tilt gravity and corrupt `ba`/velocity.
                self.kf.x.bg = self.mean_gyr;
                log::info!(
                    "imu init: |mean_acc|={:.4} mean_acc={:.3?} mean_gyr={:.4?} G={:.3} init_acc={:.3?}",
                    self.mean_acc.norm(),
                    self.mean_acc.as_slice(),
                    self.mean_gyr.as_slice(),
                    self.g_m_s2,
                    self.kf.x.acc.as_slice(),
                );
            } else {
                return;
            }
        }

        // ---- map initialisation ----
        if !self.init_map {
            for p in feats_undistort {
                let w = self.body_to_world(p);
                self.init_feats_world.push(w);
            }
            if self.init_feats_world.len() >= self.cfg.init_map_size {
                let pts = std::mem::take(&mut self.init_feats_world);
                self.ivox.add_points(&pts);
                self.init_map = true;
            }
            return;
        }

        if feats_down.is_empty() {
            return;
        }
        self.dbg_effct = 0;
        self.dbg_npts = feats_down.len();

        // Per-point LiDAR-frame vector and IMU-frame skew (crossmat) caches.
        let pbody_list: Vec<V3D> = feats_down.iter().map(|p| p.vec3()).collect();
        let crossmat_list: Vec<M3D> = feats_down
            .iter()
            .map(|p| {
                let p_imu = self.kf.x.offset_r * p.vec3() + self.kf.x.offset_t;
                so3::hat(&p_imu)
            })
            .collect();

        // World-frame copy of each downsampled point (filled post-update) and
        // its nearest map points (for MapIncremental).
        let mut feats_world: Vec<Point> = vec![Point::default(); feats_down.len()];
        let mut nearest: Vec<Vec<Point>> = vec![Vec::new(); feats_down.len()];

        // ================= point-by-point update (output model) =================
        let pcl_beg_time = pkg.cloud_start_time;
        let mut idx: isize = -1;
        for &group_len in &time_seq {
            let last_in_group = (idx + group_len as isize) as usize;
            let time_current = feats_down[last_in_group].offset_time as f64 / 1000.0
                + pcl_beg_time
                + self.cfg.time_offset_lidar_to_imu;

            if self.is_first_frame {
                if self.cfg.imu_en {
                    while let Some(nx) = self.imu_next {
                        if time_current > nx.time {
                            self.pop_imu();
                            if self.imu_next.is_none() {
                                break;
                            }
                        } else {
                            break;
                        }
                    }
                    if let Some(last) = self.imu_last {
                        self.angvel_avr = last.gyro;
                        self.acc_avr = last.acc;
                    }
                }
                self.is_first_frame = false;
                self.time_update_last = time_current;
                self.time_predict_last_const = time_current;
            }

            // Interleave IMU covariance + IMU measurement updates up to this point.
            if self.cfg.imu_en {
                while let Some(nx) = self.imu_next {
                    if !(time_current > nx.time) {
                        break;
                    }
                    self.angvel_avr = nx.gyro;
                    self.acc_avr = nx.acc;
                    let input = Imu { acc: nx.acc, gyro: nx.gyro };

                    // State propagation to the IMU time.
                    let dt = nx.time - self.time_predict_last_const;
                    if dt > 0.0 {
                        self.kf.predict(dt, &self.q, &input, true, false);
                    }
                    self.time_predict_last_const = nx.time;

                    // Covariance propagation + IMU measurement update.
                    let dt_cov = nx.time - self.time_update_last;
                    if dt_cov > 0.0 {
                        self.time_update_last = nx.time;
                        self.kf.predict(dt_cov, &self.q, &input, false, true);
                        self.imu_measurement_update();
                    }

                    self.pop_imu();
                    if self.imu_next.is_none() {
                        break;
                    }
                }
            }

            // Propagate state (and, if requested, covariance) to the point time.
            let input = Imu { acc: self.acc_avr, gyro: self.angvel_avr };
            let dt = time_current - self.time_predict_last_const;
            if !self.cfg.prop_at_freq_of_imu {
                let dt_cov = time_current - self.time_update_last;
                if dt_cov > 0.0 {
                    self.kf.predict(dt_cov, &self.q, &input, false, true);
                    self.time_update_last = time_current;
                }
            }
            self.kf.predict(dt, &self.q, &input, true, false);
            self.time_predict_last_const = time_current;

            // Build the plane measurement for this group and apply the update.
            let g0 = (idx + 1) as usize;
            let g1 = last_in_group;
            if let Some((h_x, z)) = frontend::build_plane_measurement(
                &self.kf, &self.ivox, &self.cfg, &feats_down, &pbody_list, &crossmat_list, g0, g1, &mut nearest,
            ) {
                self.dbg_effct += h_x.nrows();
                self.kf.update_plane(&h_x, &z, self.cfg.lidar_meas_cov);
            }

            // Re-project this group's points to world with the updated state.
            for j in g0..=g1 {
                feats_world[j] = self.body_to_world(&feats_down[j]);
            }

            idx += group_len as isize;
        }

        // Velocity-cap guardrail: roll a blown-up scan back to the last good
        // (state, covariance) and skip the map insert.
        let speed = self.kf.x.vel.norm();
        if self.cfg.max_velocity > 0.0 && speed > self.cfg.max_velocity {
            if let Some((x, p)) = self.last_good.clone() {
                self.kf.x = x;
                self.kf.p = p;
            }
            self.kf.x.vel = V3D::zeros();
            self.rejected_scans += 1;
        } else {
            frontend::map_incremental(&mut self.ivox, &self.cfg, &feats_world, &nearest);
            self.last_good = Some((self.kf.x.clone(), self.kf.p.clone()));
        }

        // ---- record the scan pose ----
        self.trajectory.push(PoseSample {
            time: pkg.cloud_end_time,
            pos: self.kf.x.pos,
            rot: self.kf.x.rot,
            vel: self.kf.x.vel,
        });
        self.scan_count += 1;
    }

    /// `h_model_IMU_output` + the IMU Kalman update.
    fn imu_measurement_update(&mut self) {
        let mut z = DVector::<f64>::zeros(6);
        let zg = self.angvel_avr - self.kf.x.omg - self.kf.x.bg;
        let za = self.acc_avr * self.g_m_s2 / self.cfg.acc_norm - self.kf.x.acc - self.kf.x.ba;
        for i in 0..3 {
            z[i] = zg[i];
            z[i + 3] = za[i];
        }
        let r = [
            self.cfg.imu_meas_omg_cov,
            self.cfg.imu_meas_omg_cov,
            self.cfg.imu_meas_omg_cov,
            self.cfg.imu_meas_acc_cov,
            self.cfg.imu_meas_acc_cov,
            self.cfg.imu_meas_acc_cov,
        ];
        let mut satu = [false; 6];
        if self.cfg.check_satu {
            for i in 0..3 {
                if self.angvel_avr[i].abs() >= 0.99 * self.cfg.satu_gyro {
                    satu[i] = true;
                    z[i] = 0.0;
                }
                if self.acc_avr[i].abs() >= 0.99 * self.cfg.satu_acc {
                    satu[i + 3] = true;
                    z[i + 3] = 0.0;
                }
            }
        }
        self.kf.update_imu(&z, &r, &satu);
    }
}

/// Process-noise covariance `Q` for the output model (`process_noise_cov_output`).
fn build_q_output(cfg: &Config) -> DMatrix<f64> {
    let mut q = DMatrix::<f64>::zeros(30, 30);
    for i in 0..3 {
        q[(12 + i, 12 + i)] = cfg.vel_cov;
        q[(15 + i, 15 + i)] = cfg.gyr_cov_output;
        q[(18 + i, 18 + i)] = cfg.acc_cov_output;
        q[(24 + i, 24 + i)] = cfg.b_gyr_cov;
        q[(27 + i, 27 + i)] = cfg.b_acc_cov;
    }
    q
}

/// Initial covariance for the output model (`reset_cov_output`).
fn reset_cov_output() -> DMatrix<f64> {
    let mut p = DMatrix::<f64>::identity(30, 30) * 0.01;
    for i in 0..3 {
        p[(21 + i, 21 + i)] = 0.0001;
    }
    for i in 0..6 {
        p[(24 + i, 24 + i)] = 0.001;
    }
    p
}

/// Port of `ImuProcess::Set_init`: rotation aligning `tmp_gravity` to the
/// reference `gravity`.
pub(crate) fn set_init_rot(tmp_gravity: &V3D, gravity: &V3D) -> M3D {
    let hat_grav = M3D::new(
        0.0, gravity[2], -gravity[1],
        -gravity[2], 0.0, gravity[0],
        gravity[1], -gravity[0], 0.0,
    );
    let cross = hat_grav * tmp_gravity;
    let align_norm = cross.norm() / gravity.norm() / tmp_gravity.norm();
    let mut align_cos = gravity.dot(tmp_gravity) / gravity.norm() / tmp_gravity.norm();
    align_cos = align_cos.clamp(-1.0, 1.0);
    if align_norm < 1e-6 {
        if align_cos > 1e-6 {
            M3D::identity()
        } else {
            // Exactly upside down. The cross product vanishes, so there is no
            // axis to rotate about, and the C++ answers `-I` — which has
            // determinant -1 and is a reflection, not a rotation. It does carry
            // gravity onto gravity, so nothing downstream complains, but the
            // map it builds is mirrored and every yaw runs backwards. Any half
            // turn about an axis perpendicular to gravity aligns the two just as
            // well and is a real rotation; the candidates differ only by the yaw
            // gravity cannot observe anyway.
            let off_axis = if gravity[0].abs() < gravity[2].abs() {
                V3D::new(1.0, 0.0, 0.0)
            } else {
                V3D::new(0.0, 0.0, 1.0)
            };
            so3::exp(&(gravity.cross(&off_axis).normalize() * std::f64::consts::PI))
        }
    } else {
        let align_angle = cross / cross.norm() * align_cos.acos();
        so3::exp(&align_angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Go2 carries its Mid-360 upside down, so the stationary accelerometer
    /// reads exactly `-g` and the bootstrap's cross product vanishes. The
    /// degenerate branch has to answer with a *rotation*: a reflection also
    /// carries gravity onto gravity, so the estimator runs happily while
    /// building a mirrored map and turning the wrong way.
    #[test]
    fn an_exactly_upside_down_imu_bootstraps_to_a_rotation_not_a_reflection() {
        let gravity = V3D::new(0.0, 0.0, -9.81);
        let mean_acc = V3D::new(0.0, 0.0, -9.81);
        let tmp_gravity = -mean_acc / mean_acc.norm() * gravity.norm();

        let rot = set_init_rot(&tmp_gravity, &gravity);

        assert!(
            (rot.determinant() - 1.0).abs() < 1e-9,
            "bootstrap returned determinant {} — a reflection, not a rotation:\n{rot}",
            rot.determinant(),
        );
        assert!(
            (rot.transpose() * rot - M3D::identity()).norm() < 1e-9,
            "bootstrap rotation is not orthonormal:\n{rot}",
        );
        assert!(
            (rot * tmp_gravity - gravity).norm() < 1e-6,
            "bootstrap did not level the IMU: gravity came out {:?}",
            (rot * tmp_gravity).as_slice(),
        );
    }

    /// A right-handed frame stays right-handed: `x cross y` must still be `z`
    /// after the bootstrap. `-I` passes the levelling check above's spirit but
    /// silently negates one horizontal axis, which reverses yaw.
    #[test]
    fn the_upside_down_bootstrap_keeps_the_frame_right_handed() {
        let gravity = V3D::new(0.0, 0.0, -9.81);
        let rot = set_init_rot(&V3D::new(0.0, 0.0, 9.81), &gravity);
        let (x, y, z) = (rot * V3D::x(), rot * V3D::y(), rot * V3D::z());
        assert!(
            (x.cross(&y) - z).norm() < 1e-9,
            "the bootstrapped frame is left-handed: x x y = {:?} but z = {:?}",
            x.cross(&y).as_slice(),
            z.as_slice(),
        );
    }

    /// An upright IMU must not be rotated at all, and a merely tilted one must
    /// still take the ordinary (non-degenerate) path.
    #[test]
    fn an_upright_imu_is_left_alone_and_a_tilted_one_is_levelled() {
        let gravity = V3D::new(0.0, 0.0, -9.81);
        let upright = set_init_rot(&gravity, &gravity);
        assert!((upright - M3D::identity()).norm() < 1e-9, "upright IMU got rotated:\n{upright}");

        let tilted = V3D::new(0.0, 4.0, -8.96).normalize() * 9.81;
        let rot = set_init_rot(&tilted, &gravity);
        assert!((rot.determinant() - 1.0).abs() < 1e-9, "tilted bootstrap is not a rotation");
        assert!(
            (rot * tilted - gravity).norm() < 1e-6,
            "tilted IMU not levelled: gravity came out {:?}",
            (rot * tilted).as_slice(),
        );
    }
}
