//! SO(3) helpers matching the math used by Point-LIO's IKFoM toolkit.
//!
//! `exp`/`log` are the standard Rodrigues maps. `a_matrix` is a verbatim port
//! of `MTK::A_matrix` (used in the covariance propagation of every SO(3) state
//! block); it is the left Jacobian of SO(3).

use crate::types::M3D;
use nalgebra::Vector3;

pub const TOLERANCE: f64 = 1e-11;

#[inline]
pub fn hat(v: &Vector3<f64>) -> M3D {
    M3D::new(
        0.0, -v[2], v[1],
        v[2], 0.0, -v[0],
        -v[1], v[0], 0.0,
    )
}

/// Exponential map exp: so(3) -> SO(3) (Rodrigues' formula).
pub fn exp(v: &Vector3<f64>) -> M3D {
    let theta = v.norm();
    if theta < TOLERANCE {
        return M3D::identity() + hat(v);
    }
    let axis = v / theta;
    let k = hat(&axis);
    M3D::identity() + theta.sin() * k + (1.0 - theta.cos()) * (k * k)
}

/// Logarithm map log: SO(3) -> so(3).
pub fn log(r: &M3D) -> Vector3<f64> {
    let cos_theta = ((r.trace() - 1.0) / 2.0).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    if theta.abs() < 1e-12 {
        return Vector3::zeros();
    }
    if (std::f64::consts::PI - theta).abs() < 1e-6 {
        let col = if r[(0, 0)] > r[(1, 1)] && r[(0, 0)] > r[(2, 2)] {
            0
        } else if r[(1, 1)] > r[(2, 2)] {
            1
        } else {
            2
        };
        let mut v = r.column(col) + Vector3::ith(col, 1.0);
        v /= v.norm();
        return v * theta;
    }
    let lnr = (r - r.transpose()) * (theta / (2.0 * theta.sin()));
    Vector3::new(lnr[(2, 1)], lnr[(0, 2)], lnr[(1, 0)])
}

/// Verbatim port of `MTK::A_matrix` from the IKFoM toolkit (the left Jacobian
/// of SO(3)). Used when propagating the covariance through an SO(3) state.
pub fn a_matrix(v: &Vector3<f64>) -> M3D {
    let squared_norm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    let norm = squared_norm.sqrt();
    if norm < TOLERANCE {
        return M3D::identity();
    }
    let h = hat(v);
    M3D::identity()
        + (1.0 - norm.cos()) / squared_norm * h
        + (1.0 - norm.sin() / norm) / squared_norm * (h * h)
}

/// Convert a rotation matrix to an xyzw quaternion (for rerun logging).
pub fn rot_to_quat_xyzw(r: &M3D) -> [f32; 4] {
    let trace = r[(0, 0)] + r[(1, 1)] + r[(2, 2)];
    let (w, x, y, z) = if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).sqrt();
        (0.25 / s, (r[(2, 1)] - r[(1, 2)]) * s, (r[(0, 2)] - r[(2, 0)]) * s, (r[(1, 0)] - r[(0, 1)]) * s)
    } else if r[(0, 0)] > r[(1, 1)] && r[(0, 0)] > r[(2, 2)] {
        let s = 2.0 * (1.0 + r[(0, 0)] - r[(1, 1)] - r[(2, 2)]).sqrt();
        ((r[(2, 1)] - r[(1, 2)]) / s, 0.25 * s, (r[(0, 1)] + r[(1, 0)]) / s, (r[(0, 2)] + r[(2, 0)]) / s)
    } else if r[(1, 1)] > r[(2, 2)] {
        let s = 2.0 * (1.0 + r[(1, 1)] - r[(0, 0)] - r[(2, 2)]).sqrt();
        ((r[(0, 2)] - r[(2, 0)]) / s, (r[(0, 1)] + r[(1, 0)]) / s, 0.25 * s, (r[(1, 2)] + r[(2, 1)]) / s)
    } else {
        let s = 2.0 * (1.0 + r[(2, 2)] - r[(0, 0)] - r[(1, 1)]).sqrt();
        ((r[(1, 0)] - r[(0, 1)]) / s, (r[(0, 2)] + r[(2, 0)]) / s, (r[(1, 2)] + r[(2, 1)]) / s, 0.25 * s)
    };
    [x as f32, y as f32, z as f32, w as f32]
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn exp_log_roundtrip() {
        let v = Vector3::new(0.1, -0.2, 0.3);
        assert_relative_eq!(log(&exp(&v)), v, epsilon = 1e-10);
    }

    #[test]
    fn a_matrix_small_angle_is_identity() {
        let v = Vector3::new(1e-13, 0.0, 0.0);
        assert_relative_eq!(a_matrix(&v), M3D::identity(), epsilon = 1e-9);
    }

    #[test]
    fn a_matrix_matches_left_jacobian_series() {
        // For a moderate rotation, A_matrix == sum_{n>=0} 1/(n+1)! hat(v)^n.
        let v = Vector3::new(0.2, -0.1, 0.05);
        let h = hat(&v);
        let mut series = M3D::identity();
        let mut term = M3D::identity();
        let mut fact = 1.0;
        for n in 1..12 {
            term *= h;
            fact *= (n + 1) as f64;
            series += term / fact;
        }
        assert_relative_eq!(a_matrix(&v), series, epsilon = 1e-9);
    }
}
