# pointlio_rs (vendored)

A copy of the library half of `~/repos/pointlio_rs` (Jeff Hykin's pure-Rust
Point-LIO translation, no git remote as of 2026-09-10), taken from the working
tree that produced the reference grocery-store run on CudaLaptop (8691 poses,
307.4 m), which includes three then-uncommitted changes: `satu_acc` in m/s^2,
`ivox_resolution` matched to `filter_size_map`, and the upside-down IMU
bootstrap returning a rotation rather than a reflection.

Vendored rather than referenced by path so `nix build` and a fresh clone need
nothing outside this repo. Stripped: the three binaries, the rerun `viz`
feature, the FAST-LIO comparison feature, and `trajectory::read_npy_odom` (the
only user of `ndarray`). Everything else is byte-identical.
