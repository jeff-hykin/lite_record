//! Small helpers: voxel downsampling, `time_compressing`, and plane fitting.

use crate::types::{Point, V4D};
use std::collections::HashMap;

/// Voxel downsample keeping, per occupied voxel, the point closest to the
/// voxel centre. (Point-LIO uses a PCL centroid VoxelGrid; keeping a real
/// return preserves the exact per-point `offset_time` that `time_compressing`
/// relies on.)
pub fn downsample(cloud: &[Point], leaf: f64) -> Vec<Point> {
    if leaf <= 0.0 {
        return cloud.to_vec();
    }
    let inv = 1.0 / leaf;
    let mut grid: HashMap<(i64, i64, i64), (Point, f64)> = HashMap::new();
    for p in cloud {
        let key = (
            (p.x as f64 * inv).floor() as i64,
            (p.y as f64 * inv).floor() as i64,
            (p.z as f64 * inv).floor() as i64,
        );
        let mid = (
            (key.0 as f64 + 0.5) * leaf,
            (key.1 as f64 + 0.5) * leaf,
            (key.2 as f64 + 0.5) * leaf,
        );
        let d = (p.x as f64 - mid.0).powi(2)
            + (p.y as f64 - mid.1).powi(2)
            + (p.z as f64 - mid.2).powi(2);
        let e = grid.entry(key).or_insert((*p, f64::MAX));
        if d < e.1 {
            *e = (*p, d);
        }
    }
    grid.values().map(|(p, _)| *p).collect()
}

/// Port of `time_compressing` (`common_lib.h`): given points sorted by
/// `offset_time`, return run-lengths of consecutive points that share the same
/// timestamp. The sum of the returned lengths equals `points.len()`.
pub fn time_compressing(points: &[Point]) -> Vec<usize> {
    let n = points.len();
    let mut seq = Vec::with_capacity(n);
    if n == 0 {
        return seq;
    }
    let mut j = 0usize;
    for i in 0..n - 1 {
        j += 1;
        if points[i + 1].offset_time > points[i].offset_time {
            seq.push(j);
            j = 0;
        }
    }
    seq.push(j + 1);
    seq
}

/// Break run-lengths longer than `max` into pieces of at most `max`, keeping
/// the total. A group is one Kalman update whose gain costs a cubic-in-`m`
/// matrix inverse, so an unsplit group the size of a whole scan does not
/// finish; a producer that reports no per-point times hands us exactly that.
pub fn split_groups(groups: Vec<usize>, max: usize) -> Vec<usize> {
    let mut split = Vec::with_capacity(groups.len());
    for group in groups {
        let mut left = group;
        while left > max {
            split.push(max);
            left -= max;
        }
        split.push(left);
    }
    split
}

/// Fit a plane `n·p + d = 0` (with `|n| = 1`) to `points` by least squares,
/// returning `(nx, ny, nz, d)` iff every point is within `thresh` of it.
/// Port of `esti_plane` / `common_lib.h`.
pub fn esti_plane(points: &[Point], thresh: f64) -> Option<V4D> {
    let n = points.len();
    if n < 3 {
        return None;
    }
    // Solve A x = b, A_i = [x_i,y_i,z_i], b_i = -1, via normal equations.
    let mut ata = nalgebra::Matrix3::<f64>::zeros();
    let mut atb = nalgebra::Vector3::<f64>::zeros();
    for p in points {
        let a = p.vec3();
        ata += a * a.transpose();
        atb += a * -1.0;
    }
    let normvec = ata.try_inverse()? * atb;
    let norm = normvec.norm();
    if norm < 1e-9 {
        return None;
    }
    let nx = normvec[0] / norm;
    let ny = normvec[1] / norm;
    let nz = normvec[2] / norm;
    let d = 1.0 / norm;
    for p in points {
        if (nx * p.x as f64 + ny * p.y as f64 + nz * p.z as f64 + d).abs() > thresh {
            return None;
        }
    }
    Some(V4D::new(nx, ny, nz, d))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f32, y: f32, z: f32, t: f32) -> Point {
        Point::new(x, y, z, 0.0, t)
    }

    #[test]
    fn time_compressing_groups_equal_timestamps() {
        let pts = vec![
            pt(0.0, 0.0, 0.0, 1.0),
            pt(0.0, 0.0, 0.0, 1.0),
            pt(0.0, 0.0, 0.0, 2.0),
            pt(0.0, 0.0, 0.0, 3.0),
            pt(0.0, 0.0, 0.0, 3.0),
        ];
        let seq = time_compressing(&pts);
        assert_eq!(seq, vec![2, 1, 2]);
        assert_eq!(seq.iter().sum::<usize>(), pts.len());
    }

    #[test]
    fn esti_plane_fits_offset_z_plane() {
        // Plane z = 1 (Point-LIO's n·p = -1 form requires d != 0).
        let pts = vec![
            pt(0.0, 0.0, 1.0, 0.0),
            pt(1.0, 0.0, 1.0, 0.0),
            pt(0.0, 1.0, 1.0, 0.0),
            pt(1.0, 1.0, 1.0, 0.0),
            pt(0.5, 0.5, 1.0, 0.0),
        ];
        let plane = esti_plane(&pts, 0.1).expect("planar");
        assert!(plane[2].abs() > 0.99); // normal ~ +/- z
        // n·p + d = 0 on the plane => |d| ~ 1
        assert!((plane[3].abs() - 1.0).abs() < 1e-6);
    }
}
