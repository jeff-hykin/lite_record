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

### Post processing

Foxglove ships png, jpeg, webp and avif decoders and nothing for jpeg-xl, so a jxl recording
opens with every image panel blank. **Post process** rewrites the file in place, moving each
image stream into the smallest format that still holds its pixels exactly and that *both*
viewers can decode: png for colour and infrared, raw `16UC1` for depth. Nothing is thrown
away, so the result is still the archive — there is no separate viewing copy to keep track of.

The two viewers do not overlap by much. rerun 0.32 exposes jpeg, png and RVL and has no webp
media type at all, while Foxglove has no jxl decoder — so jxl draws in rerun only, webp draws
in Foxglove only, and png is the one codec both read. Colour used to come out as webp, which
is about 1.4x smaller; it is png now, and a file converted under the old rule is mended by
running post processing over it again rather than by re-recording.

It costs disk. Measured on real recordings the file grows 15–26%, most of it raw depth that
mcap's zstd only partly takes back, and the rewrite is written beside the original before it
replaces it, so the card needs room for both. The button refuses when it does not have it.

When that room is the only thing missing, it offers to convert **reclaiming** instead. The work
goes one source chunk at a time: convert the chunk, flush it so it is a complete chunk rather than
a half-written compression stream, read it back off the disk to prove it parses and holds every
message, and only then punch the source's copy out of the file. Peak usage becomes the output
alone rather than both files, which is the difference between a 51 GB recording fitting on a
117 GB card and not.

The catch, and it is why this takes a deliberate yes: once punching starts the original is no
longer a whole recording. Nothing is released before its replacement is verified, so no messages
are lost, but if the job is interrupted they are split across the partial output and the untouched
tail of the original, and putting them back together is a manual job.

### The command line

Everything the Post process button does, plus what it cannot, is one command that runs
on any Mac or Linux box with the recording (`cargo build --release`, or a static
`nix build .#linux-x86`):

```
lite_record post_process <file.mcap> [--urdf rig.urdf] [--no-odom] [--no-deskew]
                                     [--deskew-only] [--allow-tf-conflict] [--reclaim]
```

It runs three stages. The image recode above is the first and is skipped when there is
nothing to recode. The other two **append** to the file rather than rewriting it — the
summary is cut off, new chunks are written where it was, and a summary covering old and
new is put back, so a 63 GB recording grows by the megabytes added and is never copied:

- **Frame tree.** The sensors' own transforms are read (and inverted if an older recorder
  wrote them in the SDK's direction), the URDF's joints are merged over them, and the
  complete set is appended to `/tf` at 5 Hz across the recording. `lite_record tf_fixup
  <file.mcap> --urdf rig.urdf` runs this stage alone, prints the tree and every problem
  with it, and exits non-zero while the tree is still disconnected.

  **It refuses to write a transform for a frame the recording already places.**
  Appending cannot remove, so writing a second value for such an edge leaves the file
  publishing both, for ever, with nothing saying which is meant — a consumer has to
  guess, and a tf tree that interpolates slerps between them. That is what happened to
  `sensor_mount_link -> livox_link` in the grocery recording, and it cost two people
  most of a day: a tf tree sweeping through 90 degrees across the span, and two
  depth-projection experiments that each cleanly measured a different answer. Cut the
  old value out first (`mcap_edit --drop-tf-edge <parent>:<child>`) and run this again,
  or pass `--allow-tf-conflict` to write it anyway and accept the ambiguity. An edge the
  recording does not already place is not a conflict and is appended as before. Running it again
  appends only edges that are not already there.
  Appended chunks are capped at five seconds of log time as well as by size. Transforms
  and odometry are tiny, so a size-only limit put the whole recording in one chunk, and a
  chunk that spans the recording overlaps every other chunk in the file — which stops a
  reader getting messages in log order by sorting the chunk index. It cannot remove the
  overlap entirely, since appended data covers time the original chunks already cover,
  but it bounds it: the widest appended chunk went from the full 804 s to 0.7 s.

- **Odometry.** Point-LIO (pure Rust, vendored) runs over the lidar and IMU and the
  trajectory is appended as `/pointlio_odometry` and as `odom -> <root>` edges on `/tf`,
  where `<root>` is the top of the tree the lidar hangs from. Give the URDF in the same
  run or before: odometry describes the root at the time it is written and cannot be
  re-rooted afterwards. The lidar's header stamps and the log clock can differ (the Pi's
  clock stepping after the lidar's offset was taken); the odometry is stamped on the log
  clock, like the cameras and the tf that places them, and the offset is printed.

  The same pass writes a **motion-compensated copy of the lidar** as
  `/pointlio_lidar`, unless `--no-deskew`. A Mid-360 sweeps for the whole 100 ms
  of a frame and stamps every return with when it was taken, but expresses them
  all as though the sensor had not moved, so on a rig somebody is carrying a
  scan is smeared the way a rolling shutter smears a photograph. Point-LIO's
  update already propagates the state to each point-group's time, so those poses
  are kept and every return is rewritten into where it would have been seen from
  the scan's own header stamp. Same stamp, same frame, same fields, same
  `point_step` — a consumer that read `/livox/lidar` reads this instead. Measured
  on the grocery recording at a 0.6 m/s walk, the correction grows through the
  sweep from 2 cm at the start to 26 cm at the end; accumulating 120 scans of the
  fastest-turning stretch (32 deg/s) into `odom` and counting occupied voxels,
  the corrected cloud is 1.9% tighter at 6 cm and 1.1% at 3 cm, while the same
  correction applied backwards is 3.1% and 2.5% *looser* — the sign and rough
  magnitude are what they should be. At 1.5 cm it is a wash, because the
  sensor's own range noise is that size. Scans the estimator could not place —
  the first few, while the map initialises, and any the velocity cap rolls back
  — are left out rather than passed through uncorrected, and counted in the
  report. The corrected clouds are spooled beside the recording during the walk
  and appended afterwards, so the disk needs room for the lidar stream twice; on
  the 58 GB grocery recording that is about 5 GB. A recording that was
  post-processed before this existed cannot gain the topic on a normal re-run,
  because the estimator is skipped once `/pointlio_odometry` is there and the
  corrected clouds come out of that same pass — `--deskew-only` runs the
  estimator anyway and appends *only* the clouds. It appends only the clouds
  because appending cannot remove anything, so a second odometry pass would
  leave two full sets on the topic instead of replacing the first; the run is
  deterministic given the same input and `--max-speed`, so the clouds agree with
  the odometry already in the file (verified: one-step and two-step runs produce
  byte-identical clouds). A recording that already has `/pointlio_lidar` is left
  alone either way.

Two more take an mcap and produce something to look at: `lite_record heatmap` (a
top-down density render of a cloud stream with the odometry path over it) and
`lite_record to_video` (an image topic as mp4 through ffmpeg). `--help` on each lists
the options.

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
  `sensor_msgs/CompressedImage` when a codec is chosen. Post-processing gives decoded
  depth the bare `<prefix>/depth_image` name, so a processed recording is the one that
  drops straight into a dimos graph
- `<prefix>/camera_info` (colour) and `<prefix>/depth_camera_info`,
  `/infrared_left_camera_info`, `/infrared_right_camera_info` as `sensor_msgs/CameraInfo`,
  carrying the intrinsics read from the camera's own factory calibration. Published once
  per stream, not once per frame.
- `<prefix>/aligned_depth_image` when alignment is on
- `<prefix>/imu` as `sensor_msgs/Imu`
- `<prefix>/lidar` as `sensor_msgs/PointCloud2`, with `x y z intensity tag line offset_time`
  — the per-point time offset the Mid-360 reports, so the cloud can be de-skewed
- `/tf` as `tf2_msgs/TFMessage`, the rig's static transforms repeated at 5 Hz for the whole
  recording, re-stamped each time — the way dimos' `StaticTfPublisher` publishes them, since
  dimos has no latched `/tf_static`. It carries the uploaded URDF's joints plus one edge per
  sensor stream. An engaged RealSense supplies those edges from its own factory extrinsics,
  which is the only place the millimetre offsets between its imagers exist, inverted from the
  SDK's point-map direction into tf's child-pose-in-parent; a sensor that is configured but
  not open falls back to identity edges, so the frames still exist. The channel is marked
  `lite_record.transform_convention=child_pose_in_parent`; a recording without that mark
  was written before the inversion and `post_process` corrects it.

Writes are batched and done on their own thread. A saturated queue sheds frames and counts
them rather than blocking the capture thread, and the count is what the monitor's drop
column shows.

## Develop

```sh
nix develop
cargo test
cargo clippy --all-targets -- -D warnings
```
