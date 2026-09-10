//! The iterated error-state EKF, ported from `esekfom.hpp`.
//!
//! `predict` performs the manifold state propagation and the covariance
//! propagation `P = Fx P Fxᵀ + Q dt²`, where `Fx` is built block-wise with the
//! SO(3) blocks handled through `exp(-f dt)` and the `A_matrix` left Jacobian
//! (exactly as in `esekf::predict`). `update_plane` is the point-to-plane
//! Kalman update (the `n > dof_measurement` branch — always taken here because
//! point groups are tiny). `update_imu` is the output-model IMU update.

use crate::so3;
use crate::state::{Imu, LioState, StateOutput};
use nalgebra::{DMatrix, DVector};

pub struct Esekf<S: LioState> {
    pub x: S,
    pub p: DMatrix<f64>,
}

impl<S: LioState> Esekf<S> {
    pub fn new(x: S, p: DMatrix<f64>) -> Self {
        debug_assert_eq!(p.nrows(), S::DOF);
        Esekf { x, p }
    }

    /// One propagation step. `predict_state` advances the mean; `prop_cov`
    /// advances the covariance. Mirrors `esekf::predict(dt, Q, in, a, b)`.
    pub fn predict(
        &mut self,
        dt: f64,
        q: &DMatrix<f64>,
        input: &Imu,
        predict_state: bool,
        prop_cov: bool,
    ) {
        if predict_state {
            let f = self.x.f(input);
            self.x.boxplus(&(f * dt));
        }
        if prop_cov {
            let n = S::DOF;
            let f = self.x.f(input);
            let fx = self.x.fx(input);
            let mut fx_final = fx.clone();
            let mut fx1 = DMatrix::<f64>::identity(n, n);
            for &off in S::so3_offsets() {
                // seg = -f[off..off+3] * dt  (error-state convention)
                let seg = -f.fixed_rows::<3>(off) * dt;
                let seg = seg.into_owned();
                fx1.view_mut((off, off), (3, 3)).copy_from(&so3::exp(&seg));
                let a = so3::a_matrix(&seg);
                let new_rows = a * fx.fixed_rows::<3>(off);
                fx_final.view_mut((off, 0), (3, n)).copy_from(&new_rows);
            }
            fx1 += fx_final * dt;
            self.p = &fx1 * &self.p * fx1.transpose() + q * (dt * dt);
            self.symmetrize();
        }
    }

    /// Point-to-plane update. `h_x` is `m x 12` (only the first 12 error-state
    /// dims have non-zero measurement Jacobian), `z` is `m`, `m_noise` scalar.
    /// Returns false if there were no valid correspondences.
    pub fn update_plane(&mut self, h_x: &DMatrix<f64>, z: &DVector<f64>, m_noise: f64) -> bool {
        let m = h_x.nrows();
        if m == 0 {
            return false;
        }
        // PHT = P[:, 0..12] * h_xᵀ   (n x m)
        let pht = self.p.columns(0, 12) * h_x.transpose();
        // HPHT = h_x * PHT[0..12, :]  (m x m)
        let mut hpht = h_x * pht.rows(0, 12);
        for i in 0..m {
            hpht[(i, i)] += m_noise;
        }
        let hpht_inv = match hpht.try_inverse() {
            Some(v) => v,
            None => return false,
        };
        let k = pht * hpht_inv; // n x m
        let dx = &k * z; // n
        self.x.boxplus(&dx);
        // P = P - K * h_x * P[0..12, :]
        let correction = &k * (h_x * self.p.rows(0, 12));
        self.p -= correction;
        self.symmetrize();
        true
    }

    /// Force the covariance symmetric. Sequential point updates and the
    /// non-Joseph `P -= KHP` form slowly accumulate asymmetry; over thousands
    /// of per-point updates this erodes positive-definiteness and destabilises
    /// the filter. Re-symmetrising each step is a cheap, standard safeguard.
    fn symmetrize(&mut self) {
        let pt = self.p.transpose();
        self.p += &pt;
        self.p *= 0.5;
    }
}

impl Esekf<StateOutput> {
    /// Output-model IMU update (`update_iterated_dyn_share_IMU`). Couples the
    /// gyro measurement to (`omg` @15, `bg` @24) and the accel measurement to
    /// (`acc` @18, `ba` @27). `z`/`r` are 6-vectors, `satu` masks saturated axes.
    pub fn update_imu(&mut self, z: &DVector<f64>, r: &[f64; 6], satu: &[bool; 6]) {
        let mut pht = DMatrix::<f64>::zeros(30, 6);
        let mut hp = DMatrix::<f64>::zeros(6, 30);
        for l in 0..6 {
            if !satu[l] {
                let col = self.p.column(15 + l) + self.p.column(24 + l);
                pht.column_mut(l).copy_from(&col);
                let row = self.p.row(15 + l) + self.p.row(24 + l);
                hp.row_mut(l).copy_from(&row);
            }
        }
        let mut hpht = DMatrix::<f64>::zeros(6, 6);
        for l in 0..6 {
            if !satu[l] {
                let col = hp.column(15 + l) + hp.column(24 + l);
                hpht.column_mut(l).copy_from(&col);
            }
            hpht[(l, l)] += r[l];
        }
        let hpht_inv = match hpht.try_inverse() {
            Some(v) => v,
            None => return,
        };
        let k = &pht * hpht_inv; // 30 x 6
        let dx = &k * z; // 30
        let correction = &k * &hp;
        self.p -= correction;
        self.symmetrize();
        self.x.boxplus(&dx);
    }
}
