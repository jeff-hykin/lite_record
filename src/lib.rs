//! A handheld multi-sensor recorder: RealSense, Orbbec and a Livox Mid-360 into
//! one mcap file, driven from a browser.
//!
//! This is a library with a thin binary on top rather than one big binary, so
//! integration tests can drive the real decode, encode and recording paths the
//! same way the running program does — including pushing genuine Mid-360 packets
//! at the real UDP ports.

pub mod cdr;
pub mod hub;
pub mod image;
pub mod livox;
pub mod livox_command;
pub mod msgs;
pub mod privileged;
pub mod record;
pub mod rvl;
pub mod sensors;
pub mod service;
pub mod sysmon;
pub mod urdf;
pub mod web;
