//! A handheld multi-sensor recorder: RealSense, Orbbec and a Livox Mid-360 into
//! one mcap file, driven from a browser.
//!
//! This is a library with a thin binary on top rather than one big binary, so
//! integration tests can drive the real decode, encode and recording paths the
//! same way the running program does — including pushing genuine Mid-360 packets
//! at the real UDP ports.

pub mod access;
pub mod cdr;
pub mod clock;
pub mod convert;
pub mod deskew;
pub mod distortion;
pub mod fixup;
pub mod heatmap;
pub mod hub;
pub mod image;
pub mod lcm;
pub mod livox;
pub mod livox_command;
pub mod mcap_append;
pub mod msgs;
pub mod network;
pub mod odometry;
pub mod privileged;
pub mod raytrace;
pub mod record;
pub mod restamp;
pub mod rvl;
pub mod sensors;
pub mod service;
pub mod storage;
pub mod summary;
pub mod sysmon;
pub mod tf;
pub mod topics;
pub mod urdf;
pub mod video;
pub mod walk;
pub mod web;
