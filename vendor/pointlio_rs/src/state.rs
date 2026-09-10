//! Point-LIO state manifolds and their process models, ported verbatim from
//! `common_lib.h` (`MTK_BUILD_MANIFOLD(state_input/state_output)`) and
//! `Estimator.cpp` (`get_f_*`, `df_dx_*`).
//!
//! Both states are composites of `vect3` blocks and two `SO3` blocks (`rot`,
//! `offset_R_L_I`). Gravity is a plain `vect3` here (not S2), so every block
//! has `DIM == DOF` and the "flatted" derivative shares the error-state
//! layout — `boxplus(f * dt)` is exactly the state propagation `oplus`.

use crate::so3;
use crate::types::{M3D, V3D};
use nalgebra::{DMatrix, DVector};

/// One IMU measurement fed to the process model.
#[derive(Clone, Copy, Debug)]
pub struct Imu {
    pub acc: V3D,
    pub gyro: V3D,
}

/// Shared manifold interface used by the generic [`crate::esekf::Esekf`].
pub trait LioState: Clone {
    const DOF: usize;
    /// Starting error-state indices of the SO(3) blocks (`rot`, `offset_R_L_I`).
    fn so3_offsets() -> &'static [usize];
    /// Apply an error-state increment on the manifold (`vect += dx`, `R *= exp(dx)`).
    fn boxplus(&mut self, dx: &DVector<f64>);
    /// Continuous-time derivative `dx/dt` (the "flatted state", length DOF).
    fn f(&self, input: &Imu) -> DVector<f64>;
    /// Process Jacobian `df/dx` (DOF x DOF).
    fn fx(&self, input: &Imu) -> DMatrix<f64>;
    fn rot(&self) -> M3D;
    fn pos(&self) -> V3D;
    fn vel(&self) -> V3D;
    fn offset_r(&self) -> M3D;
    fn offset_t(&self) -> V3D;
}

/// 30-DOF output model (`use_imu_as_input = false`, the default). Angular
/// velocity (`omg`) and acceleration (`acc`) are state variables driven by the
/// IMU through a measurement update.
#[derive(Clone, Debug)]
pub struct StateOutput {
    pub pos: V3D,
    pub rot: M3D,
    pub offset_r: M3D,
    pub offset_t: V3D,
    pub vel: V3D,
    pub omg: V3D,
    pub acc: V3D,
    pub gravity: V3D,
    pub bg: V3D,
    pub ba: V3D,
}

impl Default for StateOutput {
    fn default() -> Self {
        StateOutput {
            pos: V3D::zeros(),
            rot: M3D::identity(),
            offset_r: M3D::identity(),
            offset_t: V3D::zeros(),
            vel: V3D::zeros(),
            omg: V3D::zeros(),
            acc: V3D::zeros(),
            gravity: V3D::zeros(),
            bg: V3D::zeros(),
            ba: V3D::zeros(),
        }
    }
}

impl LioState for StateOutput {
    const DOF: usize = 30;

    fn so3_offsets() -> &'static [usize] {
        &[3, 6]
    }

    fn boxplus(&mut self, dx: &DVector<f64>) {
        self.pos += dx.fixed_rows::<3>(0);
        self.rot *= so3::exp(&dx.fixed_rows::<3>(3).into());
        self.offset_r *= so3::exp(&dx.fixed_rows::<3>(6).into());
        self.offset_t += dx.fixed_rows::<3>(9);
        self.vel += dx.fixed_rows::<3>(12);
        self.omg += dx.fixed_rows::<3>(15);
        self.acc += dx.fixed_rows::<3>(18);
        self.gravity += dx.fixed_rows::<3>(21);
        self.bg += dx.fixed_rows::<3>(24);
        self.ba += dx.fixed_rows::<3>(27);
    }

    fn f(&self, _input: &Imu) -> DVector<f64> {
        let mut res = DVector::zeros(30);
        let a_inertial = self.rot * self.acc;
        for i in 0..3 {
            res[i] = self.vel[i];
            res[i + 3] = self.omg[i];
            res[i + 12] = a_inertial[i] + self.gravity[i];
        }
        res
    }

    fn fx(&self, _input: &Imu) -> DMatrix<f64> {
        let mut cov = DMatrix::zeros(30, 30);
        cov.fixed_view_mut::<3, 3>(0, 12).copy_from(&M3D::identity());
        cov.fixed_view_mut::<3, 3>(12, 3).copy_from(&(-self.rot * so3::hat(&self.acc)));
        cov.fixed_view_mut::<3, 3>(12, 18).copy_from(&self.rot);
        cov.fixed_view_mut::<3, 3>(12, 21).copy_from(&M3D::identity());
        cov.fixed_view_mut::<3, 3>(3, 15).copy_from(&M3D::identity());
        cov
    }

    fn rot(&self) -> M3D { self.rot }
    fn pos(&self) -> V3D { self.pos }
    fn vel(&self) -> V3D { self.vel }
    fn offset_r(&self) -> M3D { self.offset_r }
    fn offset_t(&self) -> V3D { self.offset_t }
}

/// 24-DOF input model (`use_imu_as_input = true`). Gyro/accel are treated as
/// process inputs rather than estimated states.
#[derive(Clone, Debug)]
pub struct StateInput {
    pub pos: V3D,
    pub rot: M3D,
    pub offset_r: M3D,
    pub offset_t: V3D,
    pub vel: V3D,
    pub bg: V3D,
    pub ba: V3D,
    pub gravity: V3D,
}

impl Default for StateInput {
    fn default() -> Self {
        StateInput {
            pos: V3D::zeros(),
            rot: M3D::identity(),
            offset_r: M3D::identity(),
            offset_t: V3D::zeros(),
            vel: V3D::zeros(),
            bg: V3D::zeros(),
            ba: V3D::zeros(),
            gravity: V3D::zeros(),
        }
    }
}

impl LioState for StateInput {
    const DOF: usize = 24;

    fn so3_offsets() -> &'static [usize] {
        &[3, 6]
    }

    fn boxplus(&mut self, dx: &DVector<f64>) {
        self.pos += dx.fixed_rows::<3>(0);
        self.rot *= so3::exp(&dx.fixed_rows::<3>(3).into());
        self.offset_r *= so3::exp(&dx.fixed_rows::<3>(6).into());
        self.offset_t += dx.fixed_rows::<3>(9);
        self.vel += dx.fixed_rows::<3>(12);
        self.bg += dx.fixed_rows::<3>(15);
        self.ba += dx.fixed_rows::<3>(18);
        self.gravity += dx.fixed_rows::<3>(21);
    }

    fn f(&self, input: &Imu) -> DVector<f64> {
        let mut res = DVector::zeros(24);
        let omega = input.gyro - self.bg;
        let a_inertial = self.rot * (input.acc - self.ba);
        for i in 0..3 {
            res[i] = self.vel[i];
            res[i + 3] = omega[i];
            res[i + 12] = a_inertial[i] + self.gravity[i];
        }
        res
    }

    fn fx(&self, input: &Imu) -> DMatrix<f64> {
        let mut cov = DMatrix::zeros(24, 24);
        let acc_ = input.acc - self.ba;
        cov.fixed_view_mut::<3, 3>(0, 12).copy_from(&M3D::identity());
        cov.fixed_view_mut::<3, 3>(12, 3).copy_from(&(-self.rot * so3::hat(&acc_)));
        cov.fixed_view_mut::<3, 3>(12, 18).copy_from(&(-self.rot));
        cov.fixed_view_mut::<3, 3>(12, 21).copy_from(&M3D::identity());
        cov.fixed_view_mut::<3, 3>(3, 15).copy_from(&(-M3D::identity()));
        cov
    }

    fn rot(&self) -> M3D { self.rot }
    fn pos(&self) -> V3D { self.pos }
    fn vel(&self) -> V3D { self.vel }
    fn offset_r(&self) -> M3D { self.offset_r }
    fn offset_t(&self) -> V3D { self.offset_t }
}
