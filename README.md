# lite_record

A single self-contained binary that records a handheld multi-sensor rig to one mcap file,
with a browser UI for driving it. Built for a Raspberry Pi or Jetson: the capture threads
never wait on the disk, on image compression, or on the browser.

Supported sensors:

| Sensor | Streams | Needs an SDK |
| --- | --- | --- |
| Intel RealSense (D4xx) | depth, colour, both IR imagers, IMU, per-stream `CameraInfo`, optional depth→colour alignment | librealsense2 |
| Orbbec stereo depth camera | depth, colour, both IR imagers, IMU, per-stream `CameraInfo` | Orbbec SDK |
| Livox Mid-360 | `PointCloud2` at 10 Hz with a per-point timestamp offset, IMU at 200 Hz | no |

The Mid-360 needs no SDK to record: it multicasts its point and IMU datagrams, and
`lite_record` decodes them off the wire directly.

## Build

Every vendor SDK is behind a cargo feature, so the crate builds and its whole test suite
runs on a laptop with no hardware and no SDK installed.

```sh
cargo build --release                              # no camera SDKs; lidar still works
cargo build --release --features realsense,livox   # needs librealsense2 via pkg-config
```

`nix develop` gives you the pinned toolchain plus the cross linkers.

### Cross-compiling to a Pi or Jetson

```sh
nix build .#linux-arm64      # aarch64, static musl, no camera SDKs
nix build .#linux-x86        # x86_64, static musl, no camera SDKs
```

Those are static and depend on nothing on the target. A build **with a camera SDK** cannot
be static, because the vendor `.so` is dynamically linked (and Orbbec's is closed source),
so it targets `aarch64-unknown-linux-gnu` and needs the target's own `librealsense2.so`
plus a `realsense2.pc` describing it:

```sh
rustup target add aarch64-unknown-linux-gnu
export PKG_CONFIG_ALLOW_CROSS=1
export PKG_CONFIG_PATH=/path/to/aarch64/lib/pkgconfig
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-unknown-linux-gnu-cc
cargo build --release --target aarch64-unknown-linux-gnu --features realsense,livox
```

No headers and no cross clang are needed: `realsense-sys` is used without its
`buildtime-bindgen` feature, so it links against the shipped bindings rather than
re-running bindgen over the SDK's headers for the target.

## Run

```sh
lite_record --record-dir /media/usb/recordings --engage realsense,livox
```

Then open the URL it prints (`http://<lan ip>:8099`) from a phone or laptop on the same
network.

```
--port <PORT>             http port [default: 8099]
--bind <ADDR>             bind address [default: 0.0.0.0] -- see Security
--record-dir <DIR>        where mcap recordings are written [default: recordings]
--settings-file <PATH>    persisted settings [default: ~/.dimos/lite_record.json]
--engage <LIST>           sensors to open at startup, e.g. realsense,livox
```

## Security

**The UI exposes an unauthenticated remote shell.** The Terminal panel runs arbitrary
commands on the machine hosting the binary, and the settings panel accepts a sudo password
so those commands can run as root. The server binds `0.0.0.0` with no authentication, so
**anyone who can reach port 8099 can run commands as the user running the binary** — and as
root if a password has been entered.

That is deliberate: the whole point is to fix a rig from a phone in the field, where the
alternative is carrying a keyboard. But it means the bind address is a real decision:

- On a trusted, isolated network (a rig's own hotspot), the default `0.0.0.0` is fine.
- On anything else, run with `--bind 127.0.0.1` and reach it over an ssh tunnel:
  `ssh -L 8099:127.0.0.1:8099 pi@rig`.

The sudo password is held only in the process's memory. It is fed to `sudo -S` on stdin
rather than passed in argv, because argv is world-readable through `/proc`; it is never
written to the settings file, never logged, and never reaches the mcap. It is gone on
restart, so it has to be re-entered.

## Start on boot

```sh
lite_record survive_reboot [same options you would normally pass]
```

Installs a systemd unit on Linux or a launchd daemon on macOS, then starts it, so the rig
comes back on its own after a power cut. It asks for sudo. The options you pass are baked
into the service, and relative paths are made absolute first since a service does not
inherit your shell's directory.

Undo it with `sudo systemctl disable --now lite_record`, or on macOS
`sudo launchctl bootout system/com.jeffhykin.lite_record`.

## The web UI

- **Record** — starts and stops one mcap file, and shows its size and message count while
  running.
- **Preview** — a dropdown of every image topic the engaged sensors publish. It decodes and
  re-encodes one stream for the browser, which costs CPU, so it has its own on/off toggle;
  turning it off leaves the recording completely untouched.
- **Monitor** — a collapsible panel showing per-stream Hz and drop counts, total and
  per-core CPU, memory, temperature, free space on the recording disk, and the Pi's
  throttle word (it warns on under-voltage, current throttling, and soft temperature
  limits, and distinguishes "happening now" from "has happened since boot"). It updates at
  5 Hz over a websocket, and while collapsed the socket is closed so it costs nothing.
- **Settings** — everything below.
- **Terminal** — a shell, plus the box for the sudo password. See Security.

Each sensor also has a **disengage** button, which closes the device and frees it for
another process, and re-engages it later without restarting `lite_record`.

## Settings

- **Recording directory**, and a **Mount USB drives** button that finds unmounted removable
  partitions with `lsblk` and mounts each under `/media/<label>`.
- **Per sensor**: enabled, topic prefix, frame prefix, serial number, which streams
  (depth / colour / IR / IMU), resolution, frame rate, IR emitter on/off, and depth→colour
  alignment. The emitter is applied live; everything else cycles the device, which the hub
  does for you.
- **Mid-360**: interface, host address, frame rate, IMU on/off, voxel leaf size, and a
  **Configure network** button that assigns the host address on the lidar's `/24` and adds
  the multicast route.
- **URDF**: upload the file that completes the TF tree. It warns when no URDF is uploaded,
  and when the uploaded one leaves a broken tree — a sensor frame with no parent, a frame
  with two parents, a cycle, or more than one root. A three.js viewer renders it as a sanity
  check.
- **Compression**: mcap chunk compression (none / lz4 / zstd), and the image codec, chosen
  separately for colour and for depth/IR because a colour codec would silently truncate
  16-bit depth. Colour can be jpeg, webp, png or jpeg-xl; depth and IR fall back to a
  lossless codec when the chosen one cannot hold their bit depth.

### Why there is no H.264

There is an `h264` cargo feature wired to `openh264`, but it is off by default and not
recommended. Software H.264 on a Pi 4 costs more CPU per frame than jpeg for a 640×480
stream, and using the Pi's hardware encoder means going through V4L2 M2M, which would put a
vendor-specific path in the capture loop. Recording jpeg and transcoding later, off the
rig, is both faster on the rig and better quality. Depth and IR cannot use it at all — H.264
is 8-bit.

## What the file contains

One mcap, ROS2 CDR encoded with schemas, so it opens in Foxglove without a conversion step.

Topics are named after the matching dimos module's outputs, so a recording drops into a
dimos graph without a remapping table.

- `<prefix>/depth_image`, `/color_image`, `/infrared_left`, `/infrared_right` as
  `sensor_msgs/Image`, and the same names with a `/compressed` suffix as
  `sensor_msgs/CompressedImage` when a codec is chosen. Post-processing decodes the
  compressed depth and gives the result the bare `<prefix>/depth_image` name, so a
  processed recording is the one that drops straight into a dimos graph
- `<prefix>/camera_info` (colour) and `<prefix>/depth_camera_info`,
  `/infrared_left_camera_info`, `/infrared_right_camera_info` as `sensor_msgs/CameraInfo`,
  carrying the intrinsics read from the camera's own factory calibration. Published once
  per stream, not once per frame.
- `<prefix>/aligned_depth_image` when alignment is on
- `<prefix>/imu` as `sensor_msgs/Imu`
- `<prefix>/lidar` as `sensor_msgs/PointCloud2`, with `x y z intensity tag line offset_time`
  — the per-point time offset the Mid-360 reports, so the cloud can be de-skewed
- `/tf_static` as `tf2_msgs/TFMessage`, written once at the start of each recording. It
  carries the uploaded URDF's joints plus one edge per sensor stream. An engaged RealSense
  supplies those edges from its own factory extrinsics, which is the only place the
  millimetre offsets between its imagers exist; a sensor that is configured but not open
  falls back to identity edges, so the frames still exist.

Writes are batched and done on their own thread. A saturated queue sheds frames and counts
them rather than blocking the capture thread, and the count is what the monitor's drop
column shows.

## Develop

```sh
nix develop
cargo test
cargo clippy --all-targets -- -D warnings
```
