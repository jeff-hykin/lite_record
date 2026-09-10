//! pointlio_rs — a pure-Rust, ROS-free translation of Point-LIO
//! (point-by-point iterated-EKF LiDAR-inertial odometry).
//!
//! The estimator core lives in [`point_lio::PointLio`]; feed it
//! [`types::SyncPackage`]s (one LiDAR scan + its IMU samples) and read the
//! resulting [`types::PoseSample`] trajectory. [`pcap`] ingests raw Livox
//! Mid-360 SDK2 captures and [`mcap_input`] ingests ROS2-CDR recordings;
//! [`trajectory`] and [`metrics`] support validation against other LIO
//! trajectories.

pub mod config;
pub mod esekf;
pub mod frontend;
pub mod ivox;
pub mod mcap_input;
pub mod point_lio_input;
pub mod metrics;
pub mod pcap;
pub mod point_lio;
pub mod so3;
pub mod state;
pub mod trajectory;
pub mod types;
pub mod util;

pub use config::Config;
pub use point_lio::PointLio;
pub use point_lio_input::PointLioInput;
pub use types::{ImuData, Point, PoseSample, SyncPackage};

/// Test/diagnostic re-export of the gravity-alignment rotation.
pub fn point_lio_set_init_rot(tmp_gravity: &types::V3D, gravity: &types::V3D) -> types::M3D {
    point_lio::set_init_rot(tmp_gravity, gravity)
}
