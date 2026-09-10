//! Trajectory IO: load/save TUM (`timestamp tx ty tz qx qy qz qw`) and load the
//! `.npy` odometry the FAST-LIO Rust runner writes (`[t, x,y,z, vx,vy,vz]`).

use crate::so3;
use crate::types::{PoseSample, V3D};

/// A timestamped 3-D position (the minimal trajectory we compare on).
#[derive(Clone, Copy, Debug)]
pub struct TrajPoint {
    pub time: f64,
    pub pos: V3D,
}

pub type Trajectory = Vec<TrajPoint>;

/// Write our estimator output as a TUM file.
pub fn write_tum(path: &str, traj: &[PoseSample]) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    for s in traj {
        let q = so3::rot_to_quat_xyzw(&s.rot);
        writeln!(
            f,
            "{:.9} {:.6} {:.6} {:.6} {:.9} {:.9} {:.9} {:.9}",
            s.time, s.pos[0], s.pos[1], s.pos[2], q[0], q[1], q[2], q[3]
        )?;
    }
    Ok(())
}

/// Load a TUM file as positions (`timestamp tx ty tz [qx qy qz qw]`). Lines
/// starting with `#` are ignored.
pub fn read_tum(path: &str) -> std::io::Result<Trajectory> {
    let text = std::fs::read_to_string(path)?;
    let mut traj = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let v: Vec<f64> = line.split_whitespace().filter_map(|s| s.parse().ok()).collect();
        if v.len() >= 4 {
            traj.push(TrajPoint { time: v[0], pos: V3D::new(v[1], v[2], v[3]) });
        }
    }
    Ok(traj)
}

pub fn samples_to_traj(traj: &[PoseSample]) -> Trajectory {
    traj.iter().map(|s| TrajPoint { time: s.time, pos: s.pos }).collect()
}
