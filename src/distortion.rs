//! Refitting a RealSense "inverse Brown-Conrady" calibration as a forward
//! `plumb_bob` one.
//!
//! A D455's colour stream calibrates as inverse Brown-Conrady: its coefficients
//! map a distorted pixel to where it belongs, the opposite direction of every
//! model ROS names. There is no ROS spelling for that — realsense-ros writes
//! `plumb_bob` over the unconverted coefficients, which is wrong in the other
//! direction (IntelRealSense/realsense-ros#2458) — so this recorder writes
//! `distortion_model: "unknown"`, and Foxglove then refuses to draw any image
//! panel whose calibration it cannot interpret.
//!
//! Both models are the same Brown polynomial pointed in opposite directions, so
//! a genuine forward `plumb_bob` can be fitted after the fact: sample the pixel
//! grid, undistort each sample with the closed-form inverse model, and solve for
//! the forward coefficients that push the undistorted points back where they
//! came from. The polynomial is linear in its coefficients once the points are
//! fixed, so the fit is a 5x5 least-squares solve, and its residual measures
//! exactly how faithful the relabel is. A recording could also spell "unknown"
//! for some model that is not inverse Brown-Conrady at all (F-theta lands there
//! too); the residual gate is what keeps those honest — a fit that cannot
//! reproduce the recorded mapping is thrown away and the message copied through.

use crate::cdr::CdrReader;
use crate::msgs::{CameraInfo, RegionOfInterest};

/// Largest acceptable disagreement, in pixels anywhere in the frame, between
/// the recorded mapping and the fitted one. It cannot be zero even for a
/// genuine inverse Brown-Conrady stream — the exact inverse of the polynomial
/// is not itself the polynomial — and a D455 colour calibration fits to
/// ~0.09 px at the far corners. Half a pixel is invisible in a rendered image
/// while a wrong model (a fisheye, say) misses by whole pixels, so this is a
/// gate between "faithful relabel" and "different lens", not a precision claim.
const MAX_RESIDUAL_PX: f64 = 0.5;

/// Pixels between fit samples. 16 px over a 1280x720 frame is ~3,700 points
/// for a 5-unknown fit — dense enough that nothing between samples can stray.
const GRID_STEP: usize = 16;

/// Reads a `sensor_msgs/msg/CameraInfo` back out of its CDR bytes.
pub fn parse_camera_info(payload: &[u8]) -> CameraInfo {
    let mut reader = CdrReader::new(payload);
    let header = reader.header();
    let height = reader.u32();
    let width = reader.u32();
    let distortion_model = reader.string();
    let count = reader.u32() as usize;
    let distortion = (0..count).map(|_| reader.f64()).collect();
    CameraInfo {
        header,
        height,
        width,
        distortion_model,
        distortion,
        intrinsics: reader.f64_array(),
        rectification: reader.f64_array(),
        projection: reader.f64_array(),
        binning_x: reader.u32(),
        binning_y: reader.u32(),
        roi: RegionOfInterest {
            x_offset: reader.u32(),
            y_offset: reader.u32(),
            height: reader.u32(),
            width: reader.u32(),
            do_rectify: reader.boolean(),
        },
    }
}

/// The Brown polynomial with coefficients in ROS `D` order
/// `[k1, k2, p1, p2, k3]`, which is also the order librealsense stores its
/// inverse coefficients in. This one formula is both models: fed distorted
/// coordinates and the inverse coefficients it undistorts, fed ideal
/// coordinates and forward coefficients it distorts.
fn brown(x: f64, y: f64, coefficients: &[f64]) -> (f64, f64) {
    let (k1, k2, p1, p2, k3) = (
        coefficients[0],
        coefficients[1],
        coefficients[2],
        coefficients[3],
        coefficients[4],
    );
    let r2 = x * x + y * y;
    let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    (
        x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
        y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
    )
}

/// Fits forward `plumb_bob` coefficients reproducing an inverse Brown-Conrady
/// calibration, returned in ROS `D` order `[k1, k2, p1, p2, k3]`. `None` when
/// the info is not shaped like one, the system is degenerate, or the best fit
/// still disagrees with the recorded mapping by more than [`MAX_RESIDUAL_PX`].
pub fn fit_forward_plumb_bob(info: &CameraInfo) -> Option<[f64; 5]> {
    if info.distortion.len() != 5 || info.width == 0 || info.height == 0 {
        return None;
    }
    let (fx, cx, fy, cy) = (
        info.intrinsics[0],
        info.intrinsics[2],
        info.intrinsics[4],
        info.intrinsics[5],
    );
    if fx == 0.0 || fy == 0.0 {
        return None;
    }

    // Each sample is (ideal, distorted), both in normalised coordinates: walk
    // the distorted pixel grid and undistort it with the closed-form inverse.
    let mut samples = Vec::new();
    for row in (0..=info.height as usize).step_by(GRID_STEP) {
        for column in (0..=info.width as usize).step_by(GRID_STEP) {
            let distorted = ((column as f64 - cx) / fx, (row as f64 - cy) / fy);
            let ideal = brown(distorted.0, distorted.1, &info.distortion);
            samples.push((ideal, distorted));
        }
    }

    // The displacement the forward model must produce is linear in the
    // coefficients, so accumulate normal equations directly. Unknowns are
    // ordered [k1, k2, p1, p2, k3] to match the output.
    let mut normal = [[0.0; 5]; 5];
    let mut moment = [0.0; 5];
    for &((x, y), (dx, dy)) in &samples {
        let r2 = x * x + y * y;
        let rows = [
            ([x * r2, x * r2 * r2, 2.0 * x * y, r2 + 2.0 * x * x, x * r2 * r2 * r2], dx - x),
            ([y * r2, y * r2 * r2, r2 + 2.0 * y * y, 2.0 * x * y, y * r2 * r2 * r2], dy - y),
        ];
        for (row, target) in rows {
            for i in 0..5 {
                moment[i] += row[i] * target;
                for j in 0..5 {
                    normal[i][j] += row[i] * row[j];
                }
            }
        }
    }
    let coefficients = solve(normal, moment)?;

    // The fit minimised squared error; what matters is the worst pixel.
    let worst = samples
        .iter()
        .map(|&((x, y), (dx, dy))| {
            let (px, py) = brown(x, y, &coefficients);
            ((px - dx) * fx).hypot((py - dy) * fy)
        })
        .fold(0.0, f64::max);
    (worst <= MAX_RESIDUAL_PX).then_some(coefficients)
}

/// Gaussian elimination with partial pivoting on a 5x5 system. `None` when a
/// pivot vanishes, which for the normal equations means a degenerate frame
/// (such as all-zero coefficients making every unknown trade against another —
/// it does not: zero distortion solves cleanly to zero, this is for genuinely
/// rank-deficient input).
fn solve(mut matrix: [[f64; 5]; 5], mut vector: [f64; 5]) -> Option<[f64; 5]> {
    for column in 0..5 {
        let pivot = (column..5).max_by(|&a, &b| {
            matrix[a][column].abs().total_cmp(&matrix[b][column].abs())
        })?;
        if matrix[pivot][column].abs() < 1e-15 {
            return None;
        }
        matrix.swap(column, pivot);
        vector.swap(column, pivot);
        let pivot_row = matrix[column];
        for row in column + 1..5 {
            let factor = matrix[row][column] / pivot_row[column];
            for (value, pivot) in matrix[row][column..].iter_mut().zip(&pivot_row[column..]) {
                *value -= factor * pivot;
            }
            vector[row] -= factor * vector[column];
        }
    }
    let mut solution = [0.0; 5];
    for row in (0..5).rev() {
        let mut value = vector[row];
        for column in row + 1..5 {
            value -= matrix[row][column] * solution[column];
        }
        solution[row] = value / matrix[row][row];
    }
    Some(solution)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{DistortionModel, Header};

    /// The colour calibration off the D455 that motivated all of this,
    /// exactly as it appears in a recording: inverse Brown-Conrady
    /// coefficients under `distortion_model: "unknown"`.
    fn d455_colour() -> CameraInfo {
        CameraInfo::pinhole(
            Header::new(0, "camera_color_optical_frame"),
            1280,
            720,
            643.5430297851562,
            642.5350341796875,
            642.154296875,
            363.13568115234375,
            DistortionModel::Unknown,
            vec![
                -0.05730598792433739,
                0.06331121921539307,
                -0.0006074838456697762,
                0.0005913236527703702,
                -0.020124996080994606,
            ],
            0.0,
        )
    }

    #[test]
    fn the_d455_colour_calibration_fits_to_a_fraction_of_a_pixel() {
        let info = d455_colour();
        let fitted = fit_forward_plumb_bob(&info).expect("the real calibration must fit");

        // The forward fit must invert the recorded inverse model: distorting
        // an ideal point with it and undistorting the result with the recorded
        // coefficients has to come back to where it started, across the frame.
        let (fx, cx, fy, cy) = (643.543, 642.154, 642.535, 363.135);
        let mut worst: f64 = 0.0;
        for row in (0..=720).step_by(24) {
            for column in (0..=1280).step_by(24) {
                let distorted = ((column as f64 - cx) / fx, (row as f64 - cy) / fy);
                let ideal = brown(distorted.0, distorted.1, &info.distortion);
                let (round_x, round_y) = brown(ideal.0, ideal.1, &fitted);
                let error = ((round_x - distorted.0) * fx).hypot((round_y - distorted.1) * fy);
                worst = worst.max(error);
            }
        }
        assert!(worst < 0.2, "round trip off by {worst} px");
        // And it is a real distortion, not a zero fit: ~10 px at the corners.
        let corner = ((0.0 - cx) / fx, (0.0 - cy) / fy);
        let (bent_x, bent_y) = brown(corner.0, corner.1, &fitted);
        let moved = ((bent_x - corner.0) * fx).hypot((bent_y - corner.1) * fy);
        assert!(moved > 5.0, "corner only moved {moved} px");
    }

    #[test]
    fn zero_distortion_fits_to_zero() {
        let mut info = d455_colour();
        info.distortion = vec![0.0; 5];
        let fitted = fit_forward_plumb_bob(&info).expect("zeros must fit");
        assert!(fitted.iter().all(|value| value.abs() < 1e-12), "{fitted:?}");
    }

    #[test]
    fn a_fisheye_pretending_to_be_brown_is_rejected() {
        // Distortion far outside what the polynomial can reproduce over the
        // frame: the gate has to refuse rather than write a bad plumb_bob.
        let mut info = d455_colour();
        info.distortion = vec![-0.9, 2.5, 0.1, -0.1, -3.0];
        assert!(fit_forward_plumb_bob(&info).is_none());
    }

    #[test]
    fn camera_info_survives_the_cdr_round_trip() {
        let info = d455_colour();
        let encoded = crate::cdr::camera_info(&info);
        let decoded = parse_camera_info(&encoded.data);
        assert_eq!(decoded.header, info.header);
        assert_eq!(decoded.distortion_model, "unknown");
        assert_eq!(decoded.distortion, info.distortion);
        assert_eq!(decoded.intrinsics, info.intrinsics);
        assert_eq!(decoded.projection, info.projection);
        assert_eq!(decoded.roi, info.roi);
    }
}
