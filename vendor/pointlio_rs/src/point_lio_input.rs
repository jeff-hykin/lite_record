//! Point-LIO input model (`use_imu_as_input = true`): the 24-DOF formulation
//! that treats gyro/accel as process *inputs* and estimates only the slowly
//! varying biases (`bg`, `ba`). Ported from `laserMapping.cpp` lines ~810–935.
//!
//! Unlike the output model there is no IMU measurement update — the IMU drives
//! the propagation directly and only the LiDAR plane update corrects the state.
//! Because the biases carry small process noise they stay tightly observable,
//! which makes this mode markedly more robust on a legged platform's gait than
//! the high-process-noise output model.

use std::collections::VecDeque;

use nalgebra::DMatrix;

use crate::config::Config;
use crate::esekf::Esekf;
use crate::frontend;
use crate::ivox::IVox;
use crate::so3;
use crate::state::{Imu, StateInput};
use crate::types::{ImuData, M3D, Point, PoseSample, SyncPackage, V3D};
use crate::util;

pub struct PointLioInput {
    pub cfg: Config,
    pub kf: Esekf<StateInput>,
    pub ivox: IVox,
    q: DMatrix<f64>,
    g_m_s2: f64,

    imu_queue: VecDeque<ImuData>,
    imu_last: Option<ImuData>,
    imu_next: Option<ImuData>,

    imu_need_init: bool,
    after_imu_init: bool,
    b_first_frame_imu: bool,
    init_iter_num: usize,
    mean_acc: V3D,
    mean_gyr: V3D,

    init_map: bool,
    init_feats_world: Vec<Point>,
    first_scan: bool,
    pub first_lidar_time: f64,
    is_first_frame: bool,
    t_last: f64,
    time_update_last: f64,
    input_in: Imu,

    last_good: Option<(StateInput, DMatrix<f64>)>,
    pub trajectory: Vec<PoseSample>,
    pub scan_count: usize,
    pub rejected_scans: usize,
    pub dbg_effct: usize,
    pub dbg_npts: usize,
}

impl PointLioInput {
    pub fn new(cfg: Config) -> Self {
        let q = build_q_input(&cfg);
        let p = reset_cov_input();
        let mut x = StateInput::default();
        x.offset_r = cfg.lidar_to_imu_rot;
        x.offset_t = cfg.lidar_to_imu_trans;
        PointLioInput {
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
            t_last: 0.0,
            time_update_last: 0.0,
            input_in: Imu { acc: V3D::zeros(), gyro: V3D::zeros() },
            last_good: None,
            trajectory: Vec::new(),
            scan_count: 0,
            rejected_scans: 0,
            dbg_effct: 0,
            dbg_npts: 0,
        }
    }

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

    #[inline]
    fn scaled_input(&self, s: &ImuData) -> Imu {
        Imu { gyro: s.gyro, acc: s.acc * self.g_m_s2 / self.cfg.acc_norm }
    }

    pub fn process(&mut self, pkg: &SyncPackage) {
        for imu in &pkg.imus {
            self.imu_queue.push_back(*imu);
        }
        if self.imu_next.is_none() {
            self.imu_next = self.imu_queue.pop_front();
            self.imu_last = self.imu_next;
        }

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

        if self.imu_need_init {
            self.imu_init_accumulate(&pkg.imus);
            if self.init_iter_num > self.cfg.imu_init_num {
                self.imu_need_init = false;
            }
        } else if !self.after_imu_init {
            self.after_imu_init = true;
        }

        let feats_undistort = &pkg.cloud;
        let mut feats_down = util::downsample(feats_undistort, self.cfg.filter_size_surf);
        feats_down.sort_by(|a, b| a.offset_time.partial_cmp(&b.offset_time).unwrap());
        let time_seq = util::time_compressing(&feats_down);

        if !self.after_imu_init {
            if !self.imu_need_init {
                let tmp_gravity = -self.mean_acc / self.mean_acc.norm() * self.g_m_s2;
                let rot_init = crate::point_lio::set_init_rot(&tmp_gravity, &self.cfg.gravity);
                self.kf.x.rot = rot_init;
                self.kf.x.bg = self.mean_gyr;
            } else {
                return;
            }
        }

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

        let pbody_list: Vec<V3D> = feats_down.iter().map(|p| p.vec3()).collect();
        let crossmat_list: Vec<M3D> = feats_down
            .iter()
            .map(|p| so3::hat(&(self.kf.x.offset_r * p.vec3() + self.kf.x.offset_t)))
            .collect();
        let mut feats_world: Vec<Point> = vec![Point::default(); feats_down.len()];
        let mut nearest: Vec<Vec<Point>> = vec![Vec::new(); feats_down.len()];

        let pcl_beg_time = pkg.cloud_start_time;
        let mut idx: isize = -1;
        for &group_len in &time_seq {
            let last_in_group = (idx + group_len as isize) as usize;
            let time_current = feats_down[last_in_group].offset_time as f64 / 1000.0
                + pcl_beg_time
                + self.cfg.time_offset_lidar_to_imu;

            if self.is_first_frame {
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
                self.is_first_frame = false;
                self.t_last = time_current;
                self.time_update_last = time_current;
                if let Some(last) = self.imu_last {
                    self.input_in = self.scaled_input(&last);
                }
            }

            // Integrate IMU input forward to this point's time.
            while let Some(nx) = self.imu_next {
                if !(time_current > nx.time) {
                    break;
                }
                let last = self.imu_last.expect("imu_last set");
                self.input_in = self.scaled_input(&last);
                let dt = last.time - self.t_last;
                let dt_cov = last.time - self.time_update_last;
                if dt_cov > 0.0 {
                    self.kf.predict(dt_cov, &self.q, &self.input_in, false, true);
                    self.time_update_last = last.time;
                }
                if dt > 0.0 {
                    self.kf.predict(dt, &self.q, &self.input_in, true, false);
                }
                self.t_last = last.time;
                self.pop_imu();
                if self.imu_next.is_none() {
                    break;
                }
            }

            let dt = time_current - self.t_last;
            self.t_last = time_current;
            if !self.cfg.prop_at_freq_of_imu {
                let dt_cov = time_current - self.time_update_last;
                if dt_cov > 0.0 {
                    self.kf.predict(dt_cov, &self.q, &self.input_in, false, true);
                    self.time_update_last = time_current;
                }
            }
            if dt > 0.0 {
                self.kf.predict(dt, &self.q, &self.input_in, true, false);
            }

            let g0 = (idx + 1) as usize;
            let g1 = last_in_group;
            if let Some((h_x, z)) = frontend::build_plane_measurement(
                &self.kf, &self.ivox, &self.cfg, &feats_down, &pbody_list, &crossmat_list, g0, g1, &mut nearest,
            ) {
                self.dbg_effct += h_x.nrows();
                self.kf.update_plane(&h_x, &z, self.cfg.lidar_meas_cov);
            }

            for j in g0..=g1 {
                feats_world[j] = self.body_to_world(&feats_down[j]);
            }
            idx += group_len as isize;
        }

        // Velocity-cap guardrail: a single-scan blow-up shows up as an
        // implausible post-update speed. Roll the whole scan back to the last
        // good (state, covariance) and skip the map insert so the bad pose
        // never pollutes the map.
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

        self.trajectory.push(PoseSample {
            time: pkg.cloud_end_time,
            pos: self.kf.x.pos,
            rot: self.kf.x.rot,
            vel: self.kf.x.vel,
        });
        self.scan_count += 1;
    }
}

/// Process-noise covariance `Q` for the input model (`process_noise_cov_input`).
fn build_q_input(cfg: &Config) -> DMatrix<f64> {
    let mut q = DMatrix::<f64>::zeros(24, 24);
    for i in 0..3 {
        q[(3 + i, 3 + i)] = cfg.gyr_cov_input;
        q[(12 + i, 12 + i)] = cfg.acc_cov_input;
        q[(15 + i, 15 + i)] = cfg.b_gyr_cov;
        q[(18 + i, 18 + i)] = cfg.b_acc_cov;
    }
    q
}

/// Initial covariance for the input model (`reset_cov`).
fn reset_cov_input() -> DMatrix<f64> {
    let mut p = DMatrix::<f64>::identity(24, 24) * 0.1;
    for i in 0..3 {
        p[(21 + i, 21 + i)] = 0.0001;
    }
    for i in 0..6 {
        p[(15 + i, 15 + i)] = 0.001;
    }
    p
}
