//! Mode-agnostic measurement front-end shared by the input and output drivers.
//! `body_to_world` is `pointBodyToWorld`; `build_plane_measurement` is the
//! `h_model_*` point-to-plane Jacobian/residual builder. Both work on any
//! [`LioState`] since the measurement only touches the first 12 error-state
//! dims (pos, rot, offset_R, offset_T), which are identical across both models.

use nalgebra::{DMatrix, DVector};

use crate::config::Config;
use crate::esekf::Esekf;
use crate::ivox::IVox;
use crate::so3;
use crate::state::LioState;
use crate::types::{Point, M3D, V3D};
use crate::util;

/// `pointBodyToWorld` using the current state's pose + LiDAR-IMU extrinsic.
pub fn body_to_world<S: LioState>(kf: &Esekf<S>, p: &Point) -> Point {
    let pb = p.vec3();
    let p_imu = kf.x.offset_r() * pb + kf.x.offset_t();
    let g = kf.x.rot() * p_imu + kf.x.pos();
    Point::new(g[0] as f32, g[1] as f32, g[2] as f32, p.intensity, p.offset_time)
}

/// Build the point-to-plane measurement (`h_x` m×12, residual `z` m) for the
/// points `[g0, g1]`, recording each point's nearest map points in `nearest`.
/// Returns `None` when no point yields a valid plane correspondence.
#[allow(clippy::too_many_arguments)]
pub fn build_plane_measurement<S: LioState>(
    kf: &Esekf<S>,
    ivox: &IVox,
    cfg: &Config,
    feats_down: &[Point],
    pbody_list: &[V3D],
    crossmat_list: &[M3D],
    g0: usize,
    g1: usize,
    nearest: &mut [Vec<Point>],
) -> Option<(DMatrix<f64>, DVector<f64>)> {
    let mut rows: Vec<[f64; 12]> = Vec::new();
    let mut zs: Vec<f64> = Vec::new();
    let rot = kf.x.rot();
    for j in g0..=g1 {
        let world = body_to_world(kf, &feats_down[j]);
        let near = ivox.closest(&world, cfg.num_match_points);
        nearest[j] = near.clone();
        if near.len() < cfg.num_match_points {
            continue;
        }
        let plane = match util::esti_plane(&near, cfg.plane_thr) {
            Some(p) => p,
            None => continue,
        };
        let pd2 = (plane[0] * world.x as f64
            + plane[1] * world.y as f64
            + plane[2] * world.z as f64
            + plane[3])
            .abs();
        let p_norm = pbody_list[j].norm();
        if !(p_norm > cfg.match_s * pd2 * pd2) {
            continue;
        }
        let norm_vec = V3D::new(plane[0], plane[1], plane[2]);
        let c = rot.transpose() * norm_vec; // C
        let mut row = [0.0f64; 12];
        row[0] = norm_vec[0];
        row[1] = norm_vec[1];
        row[2] = norm_vec[2];
        if cfg.extrinsic_est_en {
            let p_body = pbody_list[j];
            let point_imu = kf.x.offset_r() * p_body + kf.x.offset_t();
            let a = so3::hat(&point_imu) * c; // A
            let b = so3::hat(&p_body) * kf.x.offset_r().transpose() * c; // B
            row[3] = a[0]; row[4] = a[1]; row[5] = a[2];
            row[6] = b[0]; row[7] = b[1]; row[8] = b[2];
            row[9] = c[0]; row[10] = c[1]; row[11] = c[2];
        } else {
            let a = crossmat_list[j] * c; // A = [point_imu]_x * C
            row[3] = a[0]; row[4] = a[1]; row[5] = a[2];
        }
        let z = -(norm_vec[0] * world.x as f64
            + norm_vec[1] * world.y as f64
            + norm_vec[2] * world.z as f64
            + plane[3]);
        rows.push(row);
        zs.push(z);
    }
    if rows.is_empty() {
        return None;
    }
    let m = rows.len();
    let mut h_x = DMatrix::<f64>::zeros(m, 12);
    for (i, r) in rows.iter().enumerate() {
        for c in 0..12 {
            h_x[(i, c)] = r[c];
        }
    }
    Some((h_x, DVector::from_vec(zs)))
}

/// `MapIncremental`: add each world point to `ivox` unless a map point already
/// occupies its `filter_size_map` cell near the cell centre.
pub fn map_incremental(ivox: &mut IVox, cfg: &Config, feats_world: &[Point], nearest: &[Vec<Point>]) {
    let fsm = cfg.filter_size_map as f32;
    let mut to_add: Vec<Point> = Vec::new();
    for (i, pw) in feats_world.iter().enumerate() {
        let near = &nearest[i];
        if near.is_empty() {
            to_add.push(*pw);
            continue;
        }
        let center = (
            ((pw.x / fsm).floor() + 0.5) * fsm,
            ((pw.y / fsm).floor() + 0.5) * fsm,
            ((pw.z / fsm).floor() + 0.5) * fsm,
        );
        let mut need_add = true;
        for q in near {
            if (q.x - center.0).abs() < 0.5 * fsm
                && (q.y - center.1).abs() < 0.5 * fsm
                && (q.z - center.2).abs() < 0.5 * fsm
            {
                need_add = false;
                break;
            }
        }
        if need_add {
            to_add.push(*pw);
        }
    }
    ivox.add_points(&to_add);
}
