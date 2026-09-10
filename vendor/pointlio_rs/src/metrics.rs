//! Trajectory-comparison metrics: rigid Kabsch alignment and Absolute
//! Trajectory Error (ATE). Two association strategies are provided because the
//! trajectories we compare do not all share a clock — pcap-driven runs use the
//! Livox device clock, while the recorded C++ FAST-LIO / GTSAM trajectories use
//! the ROS wall clock. `ate_time_aligned` is for same-clock series;
//! `ate_resampled` is clock-free (arc-length resampling), comparing pure shape.

use crate::trajectory::{TrajPoint, Trajectory};
use crate::types::{M3D, V3D};

#[derive(Clone, Copy, Debug)]
pub struct AteResult {
    pub rmse: f64,
    pub mean: f64,
    pub max: f64,
    pub count: usize,
}

/// Optimal rigid transform `(R, t)` mapping `src` onto `dst` (least squares).
pub fn kabsch(src: &[V3D], dst: &[V3D]) -> (M3D, V3D) {
    let n = src.len().min(dst.len());
    assert!(n >= 3, "kabsch needs >= 3 correspondences");
    let nf = n as f64;
    let cs = src.iter().take(n).fold(V3D::zeros(), |a, p| a + p) / nf;
    let cd = dst.iter().take(n).fold(V3D::zeros(), |a, p| a + p) / nf;
    let mut h = M3D::zeros();
    for i in 0..n {
        h += (src[i] - cs) * (dst[i] - cd).transpose();
    }
    let svd = h.svd(true, true);
    let u = svd.u.unwrap();
    let vt = svd.v_t.unwrap();
    let mut d = M3D::identity();
    if (vt.transpose() * u.transpose()).determinant() < 0.0 {
        d[(2, 2)] = -1.0;
    }
    let r = vt.transpose() * d * u.transpose();
    let t = cd - r * cs;
    (r, t)
}

fn stats(residuals: &[f64]) -> AteResult {
    let count = residuals.len();
    if count == 0 {
        return AteResult { rmse: f64::NAN, mean: f64::NAN, max: f64::NAN, count: 0 };
    }
    let sum_sq: f64 = residuals.iter().map(|e| e * e).sum();
    let sum: f64 = residuals.iter().sum();
    let max = residuals.iter().cloned().fold(0.0, f64::max);
    AteResult {
        rmse: (sum_sq / count as f64).sqrt(),
        mean: sum / count as f64,
        max,
        count,
    }
}

/// ATE after Kabsch-aligning matched pairs. `est` is aligned onto `refr`.
fn ate_from_pairs(est: &[V3D], refr: &[V3D]) -> AteResult {
    if est.len() < 3 {
        return AteResult { rmse: f64::NAN, mean: f64::NAN, max: f64::NAN, count: est.len() };
    }
    let (r, t) = kabsch(est, refr);
    let res: Vec<f64> = est
        .iter()
        .zip(refr.iter())
        .map(|(e, g)| (r * e + t - g).norm())
        .collect();
    stats(&res)
}

/// Same-clock ATE: associate each `est` sample to the nearest-in-time `refr`
/// sample within `max_dt` seconds, then align and score.
pub fn ate_time_aligned(est: &Trajectory, refr: &Trajectory, max_dt: f64) -> AteResult {
    if est.is_empty() || refr.is_empty() {
        return AteResult { rmse: f64::NAN, mean: f64::NAN, max: f64::NAN, count: 0 };
    }
    let mut e = Vec::new();
    let mut g = Vec::new();
    let mut j = 0usize;
    for p in est {
        while j + 1 < refr.len() && (refr[j + 1].time - p.time).abs() <= (refr[j].time - p.time).abs() {
            j += 1;
        }
        if (refr[j].time - p.time).abs() <= max_dt {
            e.push(p.pos);
            g.push(refr[j].pos);
        }
    }
    ate_from_pairs(&e, &g)
}

/// Resample a trajectory to `n` points uniformly in cumulative arc length
/// (clock-free; captures only path shape).
pub fn resample_arc_length(traj: &Trajectory, n: usize) -> Vec<V3D> {
    if traj.len() < 2 || n < 2 {
        return traj.iter().map(|p| p.pos).collect();
    }
    let mut cum = vec![0.0f64; traj.len()];
    for i in 1..traj.len() {
        cum[i] = cum[i - 1] + (traj[i].pos - traj[i - 1].pos).norm();
    }
    let total = cum[cum.len() - 1];
    if total <= 0.0 {
        return vec![traj[0].pos; n];
    }
    let mut out = Vec::with_capacity(n);
    let mut j = 0usize;
    for k in 0..n {
        let target = total * k as f64 / (n - 1) as f64;
        while j + 1 < traj.len() && cum[j + 1] < target {
            j += 1;
        }
        let seg = (cum[j + 1] - cum[j]).max(1e-12);
        let alpha = ((target - cum[j]) / seg).clamp(0.0, 1.0);
        out.push(traj[j].pos + (traj[j + 1].pos - traj[j].pos) * alpha);
    }
    out
}

/// Clock-free ATE: resample both trajectories to a common arc-length
/// parameterisation, then Kabsch-align and score.
pub fn ate_resampled(est: &Trajectory, refr: &Trajectory, n: usize) -> AteResult {
    let e = resample_arc_length(est, n);
    let g = resample_arc_length(refr, n);
    ate_from_pairs(&e, &g)
}

/// Total path length (metres).
pub fn path_length(traj: &Trajectory) -> f64 {
    traj.windows(2).map(|w| (w[1].pos - w[0].pos).norm()).sum()
}

/// Max distance of any sample from the origin (a crude divergence check).
pub fn max_excursion(traj: &[TrajPoint]) -> f64 {
    traj.iter().map(|p| p.pos.norm()).fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tp(t: f64, x: f64, y: f64, z: f64) -> TrajPoint {
        TrajPoint { time: t, pos: V3D::new(x, y, z) }
    }

    #[test]
    fn ate_zero_for_rotated_copy() {
        // est is ref rotated 90deg about z + translated; Kabsch should recover it.
        let refr: Trajectory = (0..20).map(|i| tp(i as f64, i as f64, 0.3 * i as f64, 0.0)).collect();
        let est: Trajectory = refr
            .iter()
            .map(|p| tp(p.time, -p.pos[1] + 5.0, p.pos[0] - 2.0, p.pos[2]))
            .collect();
        let a = ate_time_aligned(&est, &refr, 0.5);
        assert!(a.rmse < 1e-9, "rmse {}", a.rmse);
    }

    #[test]
    fn resample_preserves_endpoints() {
        let traj: Trajectory = (0..10).map(|i| tp(i as f64, i as f64, 0.0, 0.0)).collect();
        let r = resample_arc_length(&traj, 5);
        assert!((r[0] - V3D::new(0.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((r[4] - V3D::new(9.0, 0.0, 0.0)).norm() < 1e-9);
    }
}
