//! dimos' raycast-clearing voxel mapper, carried over from
//! `dimos/mapping/ray_tracing/rust` so a recording can be mapped after the fact.
//!
//! Only `mapper` and `voxel_ray_tracer` come across. The original's `module.rs`
//! is its dimos-module runtime glue -- topics, tf lookups, a process to run in --
//! and none of that applies to a file being walked offline; dropping it is what
//! keeps this free of the dimos workspace, exactly as `pointlio_rs` is.
//!
//! The algorithm is untouched. Two things had to move because they came from
//! `dimos_module`:
//!
//! - `worker_pool`, a seven-line rayon `ThreadPool` builder, is inlined below.
//! - `#[native_config]`, which expands to
//!   `#[derive(Debug, Deserialize, Serialize, Validate)]` with
//!   `#[serde(deny_unknown_fields)]` plus a marker trait, is written out in full
//!   on `Config`.
//!
//! `voxel_ray_tracer/tests.rs` comes across with it, so the port is checked
//! against the original's behaviour rather than assumed equal to it.

pub mod mapper;
pub mod voxel_ray_tracer;

use std::sync::Arc;

/// The mapper owns its pool so its thread setting cannot collide with whatever
/// else is running in the process. Same as `dimos_module::worker_pool`.
pub fn worker_pool(threads: u32) -> Arc<rayon::ThreadPool> {
    Arc::new(
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads as usize)
            .build()
            .expect("failed to build the worker thread pool"),
    )
}
