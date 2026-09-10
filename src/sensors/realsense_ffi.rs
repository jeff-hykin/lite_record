//! The librealsense half of the RealSense backend, compiled only under the
//! `realsense` feature.
//!
//! Everything the SDK does not force into `unsafe` — distortion mapping, format
//! naming, stream selection, intrinsics conversion — lives in `realsense.rs` and
//! is tested on every build. What is left here is the C API dance: enumerate,
//! configure, start, pull frames, and hand the decoded structs to the sink.
//!
//! Two of librealsense's habits shape this file. Every call takes an out-param
//! `rs2_error*` that must be checked and freed, and every handle has its own
//! `rs2_delete_*`; both are wrapped so a `?` cannot leak. And frames are
//! reference counted, so a frame must be released even on the paths that
//! discard it.

use std::ffi::{CStr, CString};
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Context, Result};
use realsense_sys as sys;
// Taken from the bindings rather than written out as numbers. Transcribing them
// by hand once put MJPEG's 22 on `MOTION_XYZ32F` and 12 on the emitter switch,
// and asking a gyro for MJPEG fails the whole resolve with nothing but
// "Couldn't resolve requests" to say why.
use sys::{
    rs2_camera_info_RS2_CAMERA_INFO_SERIAL_NUMBER as RS2_CAMERA_INFO_SERIAL_NUMBER,
    rs2_format_RS2_FORMAT_BGR8 as RS2_FORMAT_BGR8,
    rs2_format_RS2_FORMAT_MOTION_XYZ32F as RS2_FORMAT_MOTION_XYZ32F,
    rs2_format_RS2_FORMAT_Y8 as RS2_FORMAT_Y8,
    rs2_format_RS2_FORMAT_Z16 as RS2_FORMAT_Z16,
    rs2_option_RS2_OPTION_EMITTER_ENABLED as RS2_OPTION_EMITTER_ENABLED,
    rs2_option_RS2_OPTION_ERROR_POLLING_ENABLED as RS2_OPTION_ERROR_POLLING_ENABLED,
    rs2_option_RS2_OPTION_GLOBAL_TIME_ENABLED as RS2_OPTION_GLOBAL_TIME_ENABLED,
};

use crate::clock::HostClock;
use super::{
    body_transforms, camera_info, ros_encoding, rs2_stream_for, BodyExtrinsic, RsDistortion,
    RS2_STREAM_ACCEL, RS2_STREAM_COLOR, RS2_STREAM_DEPTH, RS2_STREAM_GYRO,
};
use super::super::{Backend, BackendStatus, CameraConfig, Produced, Sink, StreamId};
use crate::msgs::{Header, Imu, RawImage};

/// How long to wait on a frame before looking at the stop flag again. Long
/// enough that a 6 fps stream does not trip it, short enough that disengaging
/// feels immediate in the browser.
const FRAME_TIMEOUT_MS: u32 = 2_000;

/// How long to wait for the aligner's output after handing it a frame. It is
/// already holding everything it needs, so this only has to cover the hop onto
/// its dispatch thread.
const ALIGN_TIMEOUT_MS: u32 = 200;

/// The generated bindings give `rs2_stream` an unsigned repr. The stream
/// constants live in the SDK-free half as plain `i32` so they stay testable
/// without the bindings present, so they are widened at this boundary only.
fn stream_enum(value: i32) -> sys::rs2_stream {
    value as sys::rs2_stream
}

// -- error and handle plumbing ------------------------------------------------

/// Turns librealsense's out-param error into a Rust one, freeing it either way.
///
/// # Safety
/// `error` must be the out-param of exactly one librealsense call.
unsafe fn check(error: *mut sys::rs2_error, doing: &str) -> Result<()> {
    if error.is_null() {
        return Ok(());
    }
    let message = CStr::from_ptr(sys::rs2_get_error_message(error))
        .to_string_lossy()
        .into_owned();
    sys::rs2_free_error(error);
    Err(anyhow!("librealsense failed {doing}: {message}"))
}

/// Runs one librealsense call, checking and freeing its error out-param.
macro_rules! rs {
    ($doing:expr, $call:ident ( $($argument:expr),* $(,)? )) => {{
        let mut error: *mut sys::rs2_error = ptr::null_mut();
        let value = sys::$call($($argument,)* &mut error);
        check(error, $doing)?;
        value
    }};
}

/// Runs a librealsense call whose failure is not worth aborting for.
macro_rules! rs_ignoring_errors {
    ($call:ident ( $($argument:expr),* $(,)? )) => {{
        let mut error: *mut sys::rs2_error = ptr::null_mut();
        let value = sys::$call($($argument,)* &mut error);
        if !error.is_null() {
            sys::rs2_free_error(error);
        }
        value
    }};
}

/// An owned librealsense handle. The SDK has one deleter per type and none of
/// them are interchangeable, so the deleter comes in with the pointer.
struct Owned<T> {
    raw: *mut T,
    delete: unsafe extern "C" fn(*mut T),
}

impl<T> Owned<T> {
    /// # Safety
    /// `pointer` must be a live handle that `delete` is the correct deleter for,
    /// and must not be freed anywhere else.
    unsafe fn new(pointer: *mut T, delete: unsafe extern "C" fn(*mut T)) -> Result<Self> {
        if pointer.is_null() {
            return Err(anyhow!("librealsense returned a null handle"));
        }
        Ok(Owned { raw: pointer, delete })
    }
}

impl<T> Owned<T> {
    /// Deliberately a method rather than a public field. A closure that reads
    /// `guard.raw` captures only the raw pointer, which is not `Send`, and the
    /// guard is then dropped at the end of the enclosing scope while the thread
    /// is still using what it pointed at. Going through `&self` captures the
    /// whole guard instead.
    fn pointer(&self) -> *mut T {
        self.raw
    }
}

impl<T> Drop for Owned<T> {
    fn drop(&mut self) {
        unsafe { (self.delete)(self.raw) }
    }
}

// A pipeline handle is moved onto the capture thread. librealsense is
// thread-safe for these objects; the raw pointer is what Rust objects to.
unsafe impl<T> Send for Owned<T> {}

// -- the streaming session ----------------------------------------------------

/// The depth-to-colour aligner and the queue it writes into.
///
/// They travel together because a processing block only produces anything once
/// it has been handed a frame *and* has somewhere to put the result; holding one
/// without the other is how this silently published nothing.
struct Aligner {
    block: Owned<sys::rs2_processing_block>,
    queue: Owned<sys::rs2_frame_queue>,
}

/// Everything created by one `start`, dropped as a unit by `stop`.
struct Session {
    running: Arc<AtomicBool>,
    worker: Option<std::thread::JoinHandle<()>>,
    /// Absent when the device has no motion module or the user turned it off.
    motion_worker: Option<std::thread::JoinHandle<()>>,
    /// Held so the emitter can be toggled while streaming.
    depth_sensor: Arc<Mutex<Owned<sys::rs2_sensor>>>,
    /// The factory extrinsics, read once at start because they cannot change
    /// while the device streams.
    body_transforms: Vec<crate::msgs::TransformStamped>,
}

pub struct RealsenseBackend {
    pub config: CameraConfig,
    session: Option<Session>,
    error: Option<String>,
    detail: String,
}

impl RealsenseBackend {
    pub fn new(config: CameraConfig) -> Self {
        RealsenseBackend {
            config,
            session: None,
            error: None,
            detail: String::new(),
        }
    }
}

/// The device matching `serial`, or the only one attached when no serial is set.
///
/// Refusing to guess between two cameras matters here: this program is aimed at
/// a rig that carries a RealSense *and* an Orbbec, and picking "device 0" would
/// silently swap which body a recording's frames belong to between runs.
unsafe fn find_device(
    context: *mut sys::rs2_context,
    serial: Option<&str>,
) -> Result<(Owned<sys::rs2_device_list>, Owned<sys::rs2_device>, String)> {
    let list = Owned::new(
        rs!("listing devices", rs2_query_devices(context)),
        sys::rs2_delete_device_list,
    )?;
    let count = rs!("counting devices", rs2_get_device_count(list.pointer()));
    if count == 0 {
        anyhow::bail!("no RealSense device is attached");
    }

    let mut found = Vec::new();
    for index in 0..count {
        let device = Owned::new(
            rs!("opening a device", rs2_create_device(list.pointer(), index)),
            sys::rs2_delete_device,
        )?;
        let raw = rs_ignoring_errors!(rs2_get_device_info(
            device.pointer(),
            RS2_CAMERA_INFO_SERIAL_NUMBER
        ));
        let number = if raw.is_null() {
            String::new()
        } else {
            CStr::from_ptr(raw).to_string_lossy().into_owned()
        };
        match serial {
            Some(wanted) if wanted == number => return Ok((list, device, number)),
            Some(_) => found.push(number),
            None if count == 1 => return Ok((list, device, number)),
            None => found.push(number),
        }
    }

    match serial {
        Some(wanted) => anyhow::bail!("no RealSense with serial {wanted}; attached: {found:?}"),
        None => anyhow::bail!(
            "{count} RealSense devices are attached ({found:?}), so the serial setting must say which"
        ),
    }
}

/// The device's depth sensor, which is where the emitter option lives.
unsafe fn depth_sensor_of(device: *mut sys::rs2_device) -> Result<Owned<sys::rs2_sensor>> {
    let sensors = Owned::new(
        rs!("listing sensors", rs2_query_sensors(device)),
        sys::rs2_delete_sensor_list,
    )?;
    let count = rs!("counting sensors", rs2_get_sensors_count(sensors.pointer()));
    for index in 0..count {
        let sensor = Owned::new(
            rs!("opening a sensor", rs2_create_sensor(sensors.pointer(), index)),
            sys::rs2_delete_sensor,
        )?;
        let supported = rs_ignoring_errors!(rs2_supports_option(
            sensor.pointer() as *const sys::rs2_options,
            RS2_OPTION_EMITTER_ENABLED
        ));
        if supported != 0 {
            return Ok(sensor);
        }
    }
    anyhow::bail!("this device has no sensor carrying the emitter option")
}

/// Turns off the two device options this program is better off without.
///
/// `GLOBAL_TIME_ENABLED` is librealsense's device-to-host clock fit, and it goes
/// wrong once the camera bus is busy (IntelRealSense/librealsense#9131): it
/// stamped a 200 Hz gyro as if it ran at 191 Hz, a rate error that compounds to
/// over a second across a half-minute recording. Off, every stream arrives on the
/// raw hardware clock and `HostClock` puts it on wall time instead.
///
/// `ERROR_POLLING_ENABLED` reads a status control ten times a second that nothing
/// here looks at. On a bus already carrying four video streams that is worth
/// removing on its own.
///
/// Note that turning global time off does *not* stop the `time_diff_keeper`
/// thread behind it -- `uvc_sensor::start` calls `enable_time_diff_keeper(true)`
/// without consulting the option, so the keeper polls regardless. That thread is
/// why every librealsense handle for this camera has to come from one device
/// instance; see `open`.
unsafe fn quiet_device(device: *mut sys::rs2_device) -> Result<()> {
    let sensors = Owned::new(
        rs!("listing sensors", rs2_query_sensors(device)),
        sys::rs2_delete_sensor_list,
    )?;
    let count = rs!("counting sensors", rs2_get_sensors_count(sensors.pointer()));
    for index in 0..count {
        let sensor = Owned::new(
            rs!("opening a sensor", rs2_create_sensor(sensors.pointer(), index)),
            sys::rs2_delete_sensor,
        )?;
        let options = sensor.pointer() as *const sys::rs2_options;
        for option in [RS2_OPTION_ERROR_POLLING_ENABLED, RS2_OPTION_GLOBAL_TIME_ENABLED] {
            rs_ignoring_errors!(rs2_set_option(options, option, 0.0));
        }
    }
    Ok(())
}

/// The motion module, opened directly rather than through the pipeline.
///
/// The pipeline's frame syncer emits one frameset per *video* frame and folds
/// whatever motion samples have arrived into it, so a 200 Hz gyro read that way
/// arrives at 30 Hz with six of every seven samples thrown away. A frame queue
/// hung straight off the sensor has no syncer and hands back every sample once.
struct MotionStream {
    sensor: Owned<sys::rs2_sensor>,
    queue: Owned<sys::rs2_frame_queue>,
    gyro_hz: i32,
    /// Read here rather than in `body_extrinsics`, because librealsense's
    /// extrinsics graph is per device instance and this is where both the depth
    /// and the gyro profile are in hand.
    depth_to_imu: Option<BodyExtrinsic>,
}

/// Opens the gyro and accel at whichever offered rate sits closest to
/// `requested_hz`, each chosen independently.
///
/// The rate sets differ by IMU part — a D435i's BMI055 offers accel at 63 and
/// 250, a D435IF's BMI085 offers 100, 200 and 400 — so a requested rate cannot
/// be passed through, and gyro and accel routinely land on different numbers.
/// That is why a `sensor_msgs/Imu` pairs each gyro sample with the most recent
/// acceleration instead of waiting for a matching one.
unsafe fn open_motion(
    device: *mut sys::rs2_device,
    requested_hz: u32,
) -> Result<MotionStream> {
    let requested_hz = requested_hz as i32;
    let sensors = Owned::new(
        rs!("listing sensors", rs2_query_sensors(device)),
        sys::rs2_delete_sensor_list,
    )?;
    let count = rs!("counting sensors", rs2_get_sensors_count(sensors.pointer()));

    let mut depth_profile: *const sys::rs2_stream_profile = ptr::null();
    let mut motion = None;
    for index in 0..count {
        let sensor = Owned::new(
            rs!("opening a sensor", rs2_create_sensor(sensors.pointer(), index)),
            sys::rs2_delete_sensor,
        )?;
        let profiles = Owned::new(
            rs!("listing stream profiles", rs2_get_stream_profiles(sensor.pointer())),
            sys::rs2_delete_stream_profiles_list,
        )?;
        let total = rs!(
            "counting stream profiles",
            rs2_get_stream_profiles_count(profiles.pointer())
        );

        let mut best: [Option<(i32, *const sys::rs2_stream_profile)>; 2] = [None, None];
        for slot in 0..total {
            let profile = rs_ignoring_errors!(rs2_get_stream_profile(profiles.pointer(), slot));
            if profile.is_null() {
                continue;
            }
            let Some((kind, _, format, rate)) = stream_profile_data(profile) else {
                continue;
            };
            if kind == RS2_STREAM_DEPTH && depth_profile.is_null() {
                depth_profile = profile;
            }
            let wanted = match kind {
                _ if kind == RS2_STREAM_GYRO => 0,
                _ if kind == RS2_STREAM_ACCEL => 1,
                _ => continue,
            };
            if format != RS2_FORMAT_MOTION_XYZ32F {
                continue;
            }
            // Ties go to the faster rate, so a request halfway between two
            // offered rates errs towards more data rather than less.
            let closer = |candidate: i32, chosen: i32| {
                let (candidate_gap, chosen_gap) = (
                    (candidate - requested_hz).abs(),
                    (chosen - requested_hz).abs(),
                );
                candidate_gap < chosen_gap
                    || (candidate_gap == chosen_gap && candidate > chosen)
            };
            if best[wanted].is_none_or(|(chosen, _)| closer(rate, chosen)) {
                best[wanted] = Some((rate, profile));
            }
        }

        let (Some((gyro_hz, gyro_profile)), Some((_, accel_profile))) = (best[0], best[1]) else {
            continue;
        };
        // The profiles list has to outlive the open call, so the sensor is kept
        // rather than the loop moving on to the next one.
        motion = Some((sensor, profiles, gyro_hz, gyro_profile, accel_profile));
        break;
    }
    let Some((sensor, _profiles, gyro_hz, gyro_profile, accel_profile)) = motion else {
        anyhow::bail!("this device has no motion module")
    };

    let depth_to_imu = (!depth_profile.is_null())
        .then(|| extrinsic_between(depth_profile, gyro_profile, StreamId::Imu))
        .flatten();

    let mut chosen = [gyro_profile, accel_profile];
    rs!(
        "opening the motion module",
        rs2_open_multiple(sensor.pointer(), chosen.as_mut_ptr(), 2)
    );
    // Deep enough to ride out a scheduling hiccup at 400 Hz without the SDK
    // silently dropping the oldest sample.
    let queue = Owned::new(
        rs!("creating the motion queue", rs2_create_frame_queue(64)),
        sys::rs2_delete_frame_queue,
    )?;
    rs!(
        "starting the motion module",
        rs2_start_queue(sensor.pointer(), queue.pointer())
    );
    Ok(MotionStream { sensor, queue, gyro_hz, depth_to_imu })
}

/// One edge of the factory calibration, or `None` when the SDK has no such edge.
///
/// A missing edge means one fewer transform in `/tf_static`, which is worth far
/// less than the frames that would be lost by treating it as fatal.
unsafe fn extrinsic_between(
    from: *const sys::rs2_stream_profile,
    to: *const sys::rs2_stream_profile,
    child: StreamId,
) -> Option<BodyExtrinsic> {
    let mut raw = std::mem::zeroed::<sys::rs2_extrinsics>();
    let mut error: *mut sys::rs2_error = ptr::null_mut();
    sys::rs2_get_extrinsics(from, to, &mut raw, &mut error);
    if !error.is_null() {
        eprintln!(
            "realsense: no {child:?} extrinsic: {}",
            CStr::from_ptr(sys::rs2_get_error_message(error)).to_string_lossy()
        );
        sys::rs2_free_error(error);
        return None;
    }
    Some(BodyExtrinsic {
        child,
        rotation: std::array::from_fn(|slot| raw.rotation[slot] as f64),
        translation: std::array::from_fn(|slot| raw.translation[slot] as f64),
    })
}

/// A stream profile's kind, index, format and rate.
unsafe fn stream_profile_data(
    profile: *const sys::rs2_stream_profile,
) -> Option<(i32, i32, sys::rs2_format, i32)> {
    let mut kind: sys::rs2_stream = 0;
    let mut format: sys::rs2_format = 0;
    let mut index: i32 = 0;
    let mut unique_id: i32 = 0;
    let mut rate: i32 = 0;
    let mut error: *mut sys::rs2_error = ptr::null_mut();
    sys::rs2_get_stream_profile_data(
        profile,
        &mut kind,
        &mut format,
        &mut index,
        &mut unique_id,
        &mut rate,
        &mut error,
    );
    if !error.is_null() {
        sys::rs2_free_error(error);
        return None;
    }
    Some((kind as i32, index, format, rate))
}

unsafe fn set_emitter(sensor: *mut sys::rs2_sensor, on: bool) -> Result<()> {
    let value = if on { 1.0 } else { 0.0 };
    rs!(
        "setting the emitter",
        rs2_set_option(
            sensor as *const sys::rs2_options,
            RS2_OPTION_EMITTER_ENABLED,
            value,
        )
    );
    Ok(())
}

/// The format to ask for on each stream.
///
/// BGR8 rather than RGB8 for colour because the D400 firmware produces YUYV and
/// the SDK converts; asking for BGR8 makes that conversion explicit instead of
/// leaving a YUYV buffer that every downstream reader has to know how to unpack.
fn format_for(stream: StreamId) -> sys::rs2_format {
    match stream {
        StreamId::Depth => RS2_FORMAT_Z16,
        StreamId::Color => RS2_FORMAT_BGR8,
        StreamId::InfraLeft | StreamId::InfraRight => RS2_FORMAT_Y8,
        StreamId::Imu | StreamId::PointCloud => RS2_FORMAT_MOTION_XYZ32F,
    }
}

impl Backend for RealsenseBackend {
    fn start(&mut self, sink: Sink) -> Result<()> {
        if self.session.is_some() {
            return Ok(());
        }
        self.error = None;
        match unsafe { self.open(sink) } {
            Ok(()) => Ok(()),
            Err(error) => {
                self.error = Some(format!("{error:#}"));
                Err(error)
            }
        }
    }

    fn stop(&mut self) {
        let Some(session) = self.session.take() else {
            return;
        };
        session.running.store(false, Ordering::SeqCst);
        // The worker owns the pipeline and stops it on its way out, which is what
        // actually releases the USB device for the next process to claim.
        if let Some(worker) = session.worker {
            let _ = worker.join();
        }
        if let Some(worker) = session.motion_worker {
            let _ = worker.join();
        }
        self.detail = "released".into();
    }

    fn status(&self) -> BackendStatus {
        BackendStatus {
            running: self.session.is_some(),
            detail: self.detail.clone(),
            error: self.error.clone(),
        }
    }

    fn body_transforms(&self) -> Vec<crate::msgs::TransformStamped> {
        self.session
            .as_ref()
            .map(|session| session.body_transforms.clone())
            .unwrap_or_default()
    }

    /// Only the emitter can change without a restart. Resolution, frame rate and
    /// which streams are enabled are all fixed when the pipeline starts, and
    /// alignment changes what is published, so those return false and let the hub
    /// cycle the device.
    fn apply_live(&mut self, config: &CameraConfig) -> bool {
        let restart_needed = config.width != self.config.width
            || config.height != self.config.height
            || config.frame_rate != self.config.frame_rate
            || config.imu_rate != self.config.imu_rate
            || config.streams() != self.config.streams()
            || config.align_depth_to_color != self.config.align_depth_to_color
            || config.serial != self.config.serial;
        if restart_needed {
            return false;
        }
        if config.emitter != self.config.emitter {
            let Some(session) = self.session.as_ref() else {
                self.config = config.clone();
                return true;
            };
            let sensor = session.depth_sensor.lock().unwrap();
            if let Err(error) = unsafe { set_emitter(sensor.pointer(), config.emitter) } {
                self.error = Some(format!("{error:#}"));
                return false;
            }
        }
        self.config = config.clone();
        true
    }
}

type StartedPipeline = (
    Owned<sys::rs2_config>,
    Owned<sys::rs2_pipeline>,
    Owned<sys::rs2_pipeline_profile>,
    Owned<sys::rs2_device>,
);

impl RealsenseBackend {
    /// Builds a config for this camera's settings and starts a pipeline on it.
    ///
    /// The config is built fresh each call rather than edited, because a failed
    /// resolve leaves librealsense's request set in a state that is easier to
    /// throw away than to reason about.
    ///
    /// # Safety
    /// `context` must be a live context, outliving the returned handles.
    unsafe fn start_pipeline(
        &self,
        context: *mut sys::rs2_context,
        serial: &str,
    ) -> Result<StartedPipeline> {
        let config = Owned::new(
            rs!("creating a config", rs2_create_config()),
            sys::rs2_delete_config,
        )?;
        let serial_c = CString::new(serial.to_owned())?;
        rs!(
            "pinning the config to a serial",
            rs2_config_enable_device(config.pointer(), serial_c.as_ptr())
        );

        for stream in self.config.streams() {
            if let Some((kind, index)) = rs2_stream_for(stream) {
                rs!(
                    "enabling an image stream",
                    rs2_config_enable_stream(
                        config.pointer(),
                        stream_enum(kind),
                        index,
                        self.config.width as i32,
                        self.config.height as i32,
                        format_for(stream),
                        self.config.frame_rate as i32,
                    )
                );
            }
        }
        let pipeline = Owned::new(
            rs!("creating the pipeline", rs2_create_pipeline(context)),
            sys::rs2_delete_pipeline,
        )?;
        let profile = Owned::new(
            rs!(
                "starting the pipeline",
                rs2_pipeline_start_with_config(pipeline.pointer(), config.pointer())
            ),
            sys::rs2_delete_pipeline_profile,
        )?;
        // The pipeline holds its own copy of the device. Everything else that
        // touches this camera has to go through *this* copy, because librealsense
        // gives each copy its own power refcount and its own background pollers.
        let device = Owned::new(
            rs!(
                "reading the pipeline's device",
                rs2_pipeline_profile_get_device(profile.pointer())
            ),
            sys::rs2_delete_device,
        )?;
        quiet_device(device.pointer())?;
        Ok((config, pipeline, profile, device))
    }

    /// # Safety
    /// Calls into librealsense throughout. Every handle taken here is owned by an
    /// `Owned` or moved onto the worker thread, so none outlive this function
    /// uncontrolled.
    unsafe fn open(&mut self, sink: Sink) -> Result<()> {
        let context = Owned::new(
            rs!(
                "creating the context",
                rs2_create_context(sys::RS2_API_VERSION as i32)
            ),
            sys::rs2_delete_context,
        )?;
        let (_list, device, serial) =
            find_device(context.pointer(), self.config.serial.as_deref())
                .context("selecting a RealSense")?;

        drop(device);
        let (config, pipeline, profile, device) =
            self.start_pipeline(context.pointer(), &serial)?;

        let depth_sensor = depth_sensor_of(device.pointer())?;
        set_emitter(depth_sensor.pointer(), self.config.emitter)?;

        // From the pipeline's copy of the device, not a second one. Two copies
        // means two `time_diff_keeper` threads, and each one's ten-times-a-second
        // hardware-monitor read has to power the USB interface up through
        // `usb_device_libusb::open`. Whichever loses the `claim_interface` race
        // throws out of the handle constructor without closing the descriptor
        // `libusb_open` just took -- eight leaked `/dev/bus/usb` fds a second,
        // and a process that can no longer `accept()` two minutes later. Sharing
        // one copy means sharing one power refcount, so the poll is a refcount
        // bump on an interface the video streams already hold open.
        //
        // Losing every image stream because the host cannot reach the IMU would
        // be a worse outcome than recording without one, so a failure here is
        // reported rather than fatal.
        let mut motion = match self
            .config
            .imu
            .then(|| open_motion(device.pointer(), self.config.imu_rate))
        {
            Some(Ok(stream)) => Some(stream),
            Some(Err(error)) => {
                self.error = Some(format!("IMU unavailable, streaming video only: {error:#}"));
                None
            }
            None => None,
        };

        let aligner = if self.config.align_depth_to_color {
            let block = Owned::new(
                rs!(
                    "creating the depth-to-colour aligner",
                    rs2_create_align(stream_enum(RS2_STREAM_COLOR))
                ),
                sys::rs2_delete_processing_block,
            )?;
            let queue = Owned::new(
                rs!("creating the align queue", rs2_create_frame_queue(1)),
                sys::rs2_delete_frame_queue,
            )?;
            rs!(
                "wiring the aligner to its queue",
                rs2_start_processing_queue(block.pointer(), queue.pointer())
            );
            Some(Aligner { block, queue })
        } else {
            None
        };

        let running = Arc::new(AtomicBool::new(true));
        let depth_sensor = Arc::new(Mutex::new(depth_sensor));
        self.detail = match &motion {
            Some(motion) => format!(
                "{serial} @ {}x{}, imu {} Hz",
                self.config.width, self.config.height, motion.gyro_hz
            ),
            None => format!("{serial} @ {}x{}", self.config.width, self.config.height),
        };
        // Read before the worker takes the profile: these are what let a reader
        // place colour pixels against depth without the calibration file.
        let transforms = body_extrinsics(
            profile.pointer(),
            motion.as_mut().and_then(|motion| motion.depth_to_imu.take()),
            &self.config.naming,
            crate::record::now_nanos(),
        )?;

        let motion_worker = motion
            .map(|motion| {
                let running = Arc::clone(&running);
                let naming = self.config.naming.clone();
                let sink = Arc::clone(&sink);
                std::thread::Builder::new().name("realsense-imu".into()).spawn(move || {
                    // The whole struct crosses the boundary so the sensor
                    // outlives the loop reading the queue it feeds.
                    let motion = motion;
                    let mut pump = Pump::new(naming, sink, true);
                    while running.load(Ordering::SeqCst) {
                        unsafe {
                            if let Err(error) = pump.step_motion(motion.queue.pointer()) {
                                eprintln!("realsense imu: {error:#}");
                            }
                        }
                    }
                    unsafe {
                        rs_ignoring_errors!(rs2_stop(motion.sensor.pointer()));
                        rs_ignoring_errors!(rs2_close(motion.sensor.pointer()));
                    }
                })
            })
            .transpose()?;

        let worker = {
            let running = Arc::clone(&running);
            let naming = self.config.naming.clone();
            // Held for the thread's lifetime so the device is not released early.
            let held = (context, device, config, profile, depth_sensor.clone());
            std::thread::Builder::new()
                .name("realsense".into())
                .spawn(move || {
                    let _held = held;
                    // The IMU has its own thread now, so no motion frame ever
                    // reaches this one.
                    let mut pump = Pump::new(naming, sink, false);
                    while running.load(Ordering::SeqCst) {
                        unsafe {
                            if let Err(error) = pump.step(pipeline.pointer(), aligner.as_ref()) {
                                eprintln!("realsense: {error:#}");
                            }
                        }
                    }
                    unsafe {
                        rs_ignoring_errors!(rs2_pipeline_stop(pipeline.pointer()));
                    }
                })?
        };

        self.session = Some(Session {
            running,
            worker: Some(worker),
            motion_worker,
            depth_sensor,
            body_transforms: transforms,
        });
        Ok(())
    }
}

/// How far back to look for the smallest device-to-host offset seen.


// -- the frame pump -----------------------------------------------------------

/// Turns composite frames into `Produced` values.
///
/// It carries state for exactly two reasons: CameraInfo is republished only when
/// the intrinsics change rather than on every frame, and a gyro sample needs the
/// most recent acceleration to become one `sensor_msgs/Imu`.
struct Pump {
    naming: super::super::Naming,
    sink: Sink,
    publish_imu: bool,
    announced: std::collections::BTreeSet<StreamId>,
    latest_acceleration: [f64; 3],
    /// Maps the camera's hardware clock onto the host epoch. Both the video and
    /// the motion pipeline run with `GLOBAL_TIME_ENABLED` off, so every frame
    /// arrives stamped on the same raw device clock and this is what puts it on
    /// wall time.
    clock: HostClock,
    /// The last stamp published on each stream, keyed by stream and by whether
    /// it was the reprojected copy.
    ///
    /// `HostClock` estimates the device-to-host offset as the smallest one it
    /// has seen, so the estimate keeps improving while a faster sample can still
    /// turn up, and each improvement lands the next stamp behind its
    /// predecessor. On the Pi that was one 5 ms step, 0.7 s into a minute-long
    /// recording, while the first window was still converging. Small, but a
    /// header stamp that goes backwards is a fault to a ROS reader, so stamps
    /// are kept strictly increasing per stream.
    last_stamp: std::collections::BTreeMap<(StreamId, bool), u64>,
}

impl Pump {
    fn new(naming: super::super::Naming, sink: Sink, publish_imu: bool) -> Self {
        Pump {
            naming,
            sink,
            publish_imu,
            announced: Default::default(),
            latest_acceleration: [0.0; 3],
            clock: HostClock::default(),
            last_stamp: Default::default(),
        }
    }

    /// The stamp to publish for a stream: the mapped one, unless that would not
    /// be an advance on the stamp before it.
    fn increasing(&mut self, key: (StreamId, bool), stamp_nanos: u64) -> u64 {
        let previous = self.last_stamp.entry(key).or_default();
        *previous = stamp_nanos.max(previous.saturating_add(1));
        *previous
    }

    /// Waits for one composite frame and publishes everything inside it.
    ///
    /// # Safety
    /// `pipeline` must be a started pipeline; `queue` an aligner's output queue.
    unsafe fn step(
        &mut self,
        pipeline: *mut sys::rs2_pipeline,
        aligner: Option<&Aligner>,
    ) -> Result<()> {
        let mut error: *mut sys::rs2_error = ptr::null_mut();
        let composite =
            sys::rs2_pipeline_wait_for_frames(pipeline, FRAME_TIMEOUT_MS, &mut error);
        if !error.is_null() {
            sys::rs2_free_error(error);
            // A timeout is how a disengaged or unplugged camera looks, and the
            // caller's loop re-checks the stop flag, so it is not fatal here.
            return Ok(());
        }
        let composite = Owned::new(composite, sys::rs2_release_frame)?;

        let count = rs!(
            "counting frames in a composite",
            rs2_embedded_frames_count(composite.pointer())
        );
        for index in 0..count {
            let frame = Owned::new(
                rs!(
                    "extracting a frame",
                    rs2_extract_frame(composite.pointer(), index)
                ),
                sys::rs2_release_frame,
            )?;
            self.publish_frame(frame.pointer())?;
        }

        // The aligned depth is a second, reprojected copy. It is published on its
        // own topic so a reader can still get the sensor's native depth.
        if let Some(aligner) = aligner {
            // Processing consumes the reference it is handed, so the composite
            // this function still owns needs one added first.
            rs!("holding a frame for the aligner", rs2_frame_add_ref(composite.pointer()));
            rs!(
                "aligning depth to colour",
                rs2_process_frame(aligner.block.pointer(), composite.pointer())
            );
            let aligned = rs_ignoring_errors!(rs2_wait_for_frame(
                aligner.queue.pointer(),
                ALIGN_TIMEOUT_MS
            ));
            if !aligned.is_null() {
                let aligned = Owned::new(aligned, sys::rs2_release_frame)?;
                // The aligner hands back a whole frameset, colour included; only
                // the reprojected depth is new.
                let count = rs!(
                    "counting aligned frames",
                    rs2_embedded_frames_count(aligned.pointer())
                );
                for index in 0..count {
                    let frame = Owned::new(
                        rs!(
                            "extracting an aligned frame",
                            rs2_extract_frame(aligned.pointer(), index)
                        ),
                        sys::rs2_release_frame,
                    )?;
                    if self.describe(frame.pointer())?.0 == RS2_STREAM_DEPTH {
                        self.publish_aligned(frame.pointer())?;
                    }
                }
            }
        }
        Ok(())
    }

    /// Waits for one motion sample and publishes it.
    ///
    /// # Safety
    /// `queue` must be the queue a started motion sensor is writing into.
    unsafe fn step_motion(&mut self, queue: *mut sys::rs2_frame_queue) -> Result<()> {
        let mut error: *mut sys::rs2_error = ptr::null_mut();
        let frame = sys::rs2_wait_for_frame(queue, FRAME_TIMEOUT_MS, &mut error);
        if !error.is_null() {
            sys::rs2_free_error(error);
            return Ok(());
        }
        let frame = Owned::new(frame, sys::rs2_release_frame)?;
        let (kind, _, _, device_nanos) = self.describe(frame.pointer())?;
        let stamp_nanos = self.clock.host_nanos(device_nanos);
        self.publish_motion(frame.pointer(), kind, stamp_nanos)
    }

    /// The frame's stream kind, index, format and raw device timestamp.
    unsafe fn describe(&self, frame: *mut sys::rs2_frame) -> Result<(i32, i32, u32, u64)> {
        let profile = rs!("reading a frame's profile", rs2_get_frame_stream_profile(frame));
        let (kind, index, format, _) = stream_profile_data(profile)
            .ok_or_else(|| anyhow!("librealsense could not describe a frame's stream"))?;
        // librealsense reports milliseconds as a double; ROS wants nanoseconds.
        let milliseconds = rs!("reading a frame timestamp", rs2_get_frame_timestamp(frame));
        Ok((kind, index, format, (milliseconds * 1.0e6) as u64))
    }

    unsafe fn publish_frame(&mut self, frame: *mut sys::rs2_frame) -> Result<()> {
        let (kind, index, format, device_nanos) = self.describe(frame)?;
        let stamp_nanos = self.clock.host_nanos(device_nanos);
        let stream = match (kind, index) {
            (k, _) if k == RS2_STREAM_DEPTH => StreamId::Depth,
            (k, _) if k == RS2_STREAM_COLOR => StreamId::Color,
            (k, 1) if k == super::RS2_STREAM_INFRARED => StreamId::InfraLeft,
            (k, 2) if k == super::RS2_STREAM_INFRARED => StreamId::InfraRight,
            (k, _) if k == RS2_STREAM_GYRO || k == RS2_STREAM_ACCEL => {
                return self.publish_motion(frame, kind, stamp_nanos)
            }
            _ => return Ok(()),
        };
        self.publish_image(frame, stream, format, stamp_nanos, None)
    }

    unsafe fn publish_aligned(&mut self, frame: *mut sys::rs2_frame) -> Result<()> {
        let (_, _, format, device_nanos) = self.describe(frame)?;
        let stamp_nanos = self.clock.host_nanos(device_nanos);
        // Aligned depth lives in the colour optical frame, because that is the
        // camera it was reprojected into.
        let frame_id = self.naming.frame_id(StreamId::Color);
        self.publish_image(
            frame,
            StreamId::Depth,
            format,
            stamp_nanos,
            Some((self.naming.topic("aligned_depth_image"), frame_id)),
        )
    }

    unsafe fn publish_image(
        &mut self,
        frame: *mut sys::rs2_frame,
        stream: StreamId,
        format: u32,
        stamp_nanos: u64,
        override_naming: Option<(String, String)>,
    ) -> Result<()> {
        let stamp_nanos = self.increasing((stream, override_naming.is_some()), stamp_nanos);
        let Some((encoding, bytes_per_pixel)) = ros_encoding(format) else {
            // Publishing under a guessed encoding string silently corrupts every
            // reader, so an unmapped format is dropped and said out loud.
            anyhow::bail!("stream {} arrived in unmapped rs2_format {format}", stream.as_str());
        };
        let width = rs!("reading a frame width", rs2_get_frame_width(frame)) as usize;
        let height = rs!("reading a frame height", rs2_get_frame_height(frame)) as usize;
        let stride = rs!("reading a frame stride", rs2_get_frame_stride_in_bytes(frame)) as usize;
        let data = rs!("reading frame data", rs2_get_frame_data(frame)) as *const u8;
        if data.is_null() {
            anyhow::bail!("librealsense handed back a null frame buffer");
        }
        let step = if stride > 0 { stride } else { width * bytes_per_pixel };

        let (topic, frame_id) = match override_naming {
            Some(pair) => pair,
            None => (
                self.naming.image_topic(stream),
                self.naming.frame_id(stream),
            ),
        };
        let image = RawImage {
            header: Header::new(stamp_nanos, frame_id.clone()),
            width,
            height,
            step,
            is_bigendian: 0,
            encoding: encoding.to_string(),
            data: std::slice::from_raw_parts(data, step * height).to_vec(),
        };
        (self.sink)(Produced::Image {
            stream,
            topic,
            image,
        });

        if self.announced.insert(stream) {
            self.publish_camera_info(frame, stream, stamp_nanos, &frame_id)?;
        }
        Ok(())
    }

    /// Publishes the stream's intrinsics once, when it first delivers a frame.
    ///
    /// Once rather than per-frame because they are a property of the unit, not of
    /// the moment: republishing them at 30 Hz would add a whole channel's worth
    /// of identical messages to the file for nothing.
    unsafe fn publish_camera_info(
        &mut self,
        frame: *mut sys::rs2_frame,
        stream: StreamId,
        stamp_nanos: u64,
        frame_id: &str,
    ) -> Result<()> {
        let profile = rs!("reading a frame's profile", rs2_get_frame_stream_profile(frame));
        let mut intrinsics = std::mem::zeroed::<sys::rs2_intrinsics>();
        rs!(
            "reading intrinsics",
            rs2_get_video_stream_intrinsics(profile, &mut intrinsics)
        );
        let coefficients = [
            intrinsics.coeffs[0] as f64,
            intrinsics.coeffs[1] as f64,
            intrinsics.coeffs[2] as f64,
            intrinsics.coeffs[3] as f64,
            intrinsics.coeffs[4] as f64,
        ];
        let info = camera_info(
            stamp_nanos,
            frame_id,
            intrinsics.width as u32,
            intrinsics.height as u32,
            [intrinsics.fx as f64, intrinsics.fy as f64],
            [intrinsics.ppx as f64, intrinsics.ppy as f64],
            RsDistortion::from_raw(intrinsics.model),
            coefficients,
            0.0,
        );
        (self.sink)(Produced::CameraInfo {
            topic: self.naming.camera_info_topic(stream),
            info: Box::new(info),
        });
        Ok(())
    }

    unsafe fn publish_motion(
        &mut self,
        frame: *mut sys::rs2_frame,
        kind: i32,
        stamp_nanos: u64,
    ) -> Result<()> {
        if !self.publish_imu {
            return Ok(());
        }
        let data = rs!("reading motion data", rs2_get_frame_data(frame)) as *const f32;
        if data.is_null() {
            anyhow::bail!("librealsense handed back a null motion buffer");
        }
        let vector = std::slice::from_raw_parts(data, 3);
        let values = [vector[0] as f64, vector[1] as f64, vector[2] as f64];

        if kind == RS2_STREAM_ACCEL {
            // Accelerometer samples only update the pairing state. Publishing on
            // them too would double the message rate and emit an Imu whose
            // angular velocity is stale instead of whose acceleration is.
            self.latest_acceleration = values;
            return Ok(());
        }
        let stamp_nanos = self.increasing((StreamId::Imu, false), stamp_nanos);
        let imu = Imu::unoriented(
            Header::new(stamp_nanos, self.naming.frame_id(StreamId::Imu)),
            values,
            self.latest_acceleration,
        );
        (self.sink)(Produced::Imu {
            topic: self.naming.imu_topic(),
            imu: Box::new(imu),
        });
        Ok(())
    }
}

#[cfg(test)]
mod stamp_order_tests {
    use super::{Pump, StreamId};

    fn pump() -> Pump {
        Pump::new(
            super::super::super::Naming::for_kind(super::super::super::SensorKind::Realsense),
            std::sync::Arc::new(|_| true),
            true,
        )
    }

    /// The offset estimate improving mid-recording must not walk a stream's
    /// stamps backwards, and must not repeat one either -- a duplicate stamp is
    /// as unusable to a reader as an inverted one.
    #[test]
    fn a_better_offset_estimate_cannot_move_a_stream_backwards() {
        let mut pump = pump();
        let key = (StreamId::Imu, false);
        assert_eq!(pump.increasing(key, 1_000), 1_000);
        assert_eq!(pump.increasing(key, 995), 1_001);
        assert_eq!(pump.increasing(key, 1_000), 1_002);
        assert_eq!(pump.increasing(key, 5_000), 5_000);
    }

    /// Every stream carries its own history, so a slow one cannot drag another's
    /// stamps forward.
    #[test]
    fn streams_do_not_share_a_stamp_history() {
        let mut pump = pump();
        assert_eq!(pump.increasing((StreamId::Color, false), 9_000), 9_000);
        assert_eq!(pump.increasing((StreamId::Depth, false), 1_000), 1_000);
        // Native depth and the reprojected copy are separate series.
        assert_eq!(pump.increasing((StreamId::Depth, true), 1_000), 1_000);
    }
}

/// The factory extrinsics between the streams of one device, ready for
/// `/tf_static`. Read once at start, since they cannot change while streaming.
///
/// # Safety
/// `profile` must be a live pipeline profile.
pub unsafe fn body_extrinsics(
    profile: *mut sys::rs2_pipeline_profile,
    depth_to_imu: Option<BodyExtrinsic>,
    naming: &super::super::Naming,
    stamp_nanos: u64,
) -> Result<Vec<crate::msgs::TransformStamped>> {
    let streams = Owned::new(
        rs!(
            "listing the active streams",
            rs2_pipeline_profile_get_streams(profile)
        ),
        sys::rs2_delete_stream_profiles_list,
    )?;
    let count = rs!("counting active streams", rs2_get_stream_profiles_count(streams.pointer()));

    let mut depth_profile = ptr::null();
    let mut others = Vec::new();
    for index in 0..count {
        let profile = rs!(
            "reading an active stream",
            rs2_get_stream_profile(streams.pointer(), index)
        );
        let mut kind: sys::rs2_stream = 0;
        let mut format: sys::rs2_format = 0;
        let mut stream_index: i32 = 0;
        let mut unique_id: i32 = 0;
        let mut frame_rate: i32 = 0;
        rs!(
            "reading a stream profile",
            rs2_get_stream_profile_data(
                profile,
                &mut kind,
                &mut format,
                &mut stream_index,
                &mut unique_id,
                &mut frame_rate,
            )
        );
        let stream = match (kind as i32, stream_index) {
            (k, _) if k == RS2_STREAM_DEPTH => {
                depth_profile = profile;
                continue;
            }
            (k, _) if k == RS2_STREAM_COLOR => StreamId::Color,
            (k, 1) if k == super::RS2_STREAM_INFRARED => StreamId::InfraLeft,
            (k, 2) if k == super::RS2_STREAM_INFRARED => StreamId::InfraRight,
            _ => continue,
        };
        others.push((stream, profile));
    }
    if depth_profile.is_null() {
        // Every edge here is expressed relative to depth, so without it there is
        // nothing to hang the others off.
        return Ok(Vec::new());
    }

    // The IMU is opened outside the pipeline, so its edge is read there and
    // handed in rather than found among these profiles.
    let mut extrinsics: Vec<_> = depth_to_imu.into_iter().collect();
    for (stream, profile) in others {
        extrinsics.extend(extrinsic_between(depth_profile, profile, stream));
    }
    Ok(body_transforms(naming, stamp_nanos, &extrinsics))
}
