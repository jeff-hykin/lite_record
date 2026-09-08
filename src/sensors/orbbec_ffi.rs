//! The libobsensor half of the Orbbec backend, compiled only under the `orbbec`
//! feature.
//!
//! There is no maintained `-sys` crate for the Orbbec SDK, so the externs below
//! are hand-declared. Every signature and every constant was transcribed from
//! the headers of OrbbecSDK v2.9.3 rather than from documentation, and the
//! constants are pinned by tests in `orbbec.rs` so a mistake shows up as a test
//! failure instead of as a recording of the wrong stream.
//!
//! Two of libobsensor's habits shape this file, and both differ from
//! librealsense. Every call takes a trailing `ob_error**` out-param — *including
//! the deleters*, which is why `Owned`'s deleter is not a plain `fn(*mut T)`.
//! And frames are reference counted, so anything handed back by
//! `ob_frameset_get_frame`, `ob_filter_process` or `ob_frame_get_stream_profile`
//! is a new reference this file has to release.

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_double, c_int};
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Context, Result};

use super::super::realsense::body_transforms;
use super::super::{Backend, BackendStatus, CameraConfig, Produced, Sink, StreamId};
use super::{
    camera_info, extrinsic_to_body, ob_format_for, ob_frame_for, ob_stream_for, ros_encoding,
    ObDistortion, ObDistortionModel, OB_FRAME_ACCEL, OB_FRAME_DEPTH, OB_FRAME_GYRO, OB_STREAM_COLOR,
};
use crate::msgs::{Header, Imu, RawImage, TransformStamped};

/// How long to wait on a frameset before looking at the stop flag again. Long
/// enough that a 6 fps stream does not trip it, short enough that disengaging
/// feels immediate in the browser.
const FRAMESET_TIMEOUT_MS: u32 = 2_000;

/// `ob_property_id::OB_PROP_LASER_BOOL` — the IR projector.
const OB_PROP_LASER_BOOL: c_int = 3;

/// `ob_accel_full_scale_range::OB_ACCEL_FS_4g` and
/// `ob_gyro_full_scale_range::OB_GYRO_FS_1000dps`. Wide enough for a handheld
/// rig: 4 g covers a knock against a doorframe without clipping, and 1000 dps
/// covers a fast hand turn.
const OB_ACCEL_FS_4G: c_int = 2;
const OB_GYRO_FS_1000DPS: c_int = 7;

/// `ob_sample_rate::OB_SAMPLE_RATE_200_HZ`, matching the Mid-360's IMU rate so
/// the two inertial streams in one recording are directly comparable.
const OB_SAMPLE_RATE_200_HZ: c_int = 8;

// -- the C API ----------------------------------------------------------------

macro_rules! opaque {
    ($($name:ident),* $(,)?) => {
        $(
            #[repr(C)]
            pub struct $name {
                _private: [u8; 0],
            }
        )*
    };
}

opaque!(
    ob_context,
    ob_device,
    ob_device_list,
    ob_pipeline,
    ob_config,
    ob_frame,
    ob_stream_profile,
    ob_filter,
    ob_error,
);

/// `OBCameraIntrinsic`. `width`/`height` really are `int16_t`.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct ObCameraIntrinsic {
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
    width: i16,
    height: i16,
}

/// `OBCameraDistortion`. Unlike RealSense, the model travels with the
/// coefficients rather than inside the intrinsics.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct ObCameraDistortion {
    k1: f32,
    k2: f32,
    k3: f32,
    k4: f32,
    k5: f32,
    k6: f32,
    p1: f32,
    p2: f32,
    model: u32,
}

/// `OBExtrinsic`. `trans` is in **millimetres**.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct ObExtrinsic {
    rot: [f32; 9],
    trans: [f32; 3],
}

/// `OBFloat3D`, shared by `ob_accel_value` and `ob_gyro_value`.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct ObFloat3D {
    x: f32,
    y: f32,
    z: f32,
}

#[link(name = "OrbbecSDK")]
extern "C" {
    fn ob_create_context(error: *mut *mut ob_error) -> *mut ob_context;
    fn ob_delete_context(context: *mut ob_context, error: *mut *mut ob_error);
    fn ob_query_device_list(
        context: *mut ob_context,
        error: *mut *mut ob_error,
    ) -> *mut ob_device_list;
    fn ob_delete_device_list(list: *mut ob_device_list, error: *mut *mut ob_error);
    fn ob_device_list_get_count(list: *mut ob_device_list, error: *mut *mut ob_error) -> u32;
    fn ob_device_list_get_device_serial_number(
        list: *mut ob_device_list,
        index: u32,
        error: *mut *mut ob_error,
    ) -> *const c_char;
    fn ob_device_list_get_device(
        list: *mut ob_device_list,
        index: u32,
        error: *mut *mut ob_error,
    ) -> *mut ob_device;
    fn ob_delete_device(device: *mut ob_device, error: *mut *mut ob_error);
    fn ob_device_set_bool_property(
        device: *mut ob_device,
        property_id: c_int,
        value: bool,
        error: *mut *mut ob_error,
    );

    fn ob_create_pipeline_with_device(
        device: *mut ob_device,
        error: *mut *mut ob_error,
    ) -> *mut ob_pipeline;
    fn ob_delete_pipeline(pipeline: *mut ob_pipeline, error: *mut *mut ob_error);
    fn ob_pipeline_start_with_config(
        pipeline: *mut ob_pipeline,
        config: *mut ob_config,
        error: *mut *mut ob_error,
    );
    fn ob_pipeline_stop(pipeline: *mut ob_pipeline, error: *mut *mut ob_error);
    fn ob_pipeline_wait_for_frameset(
        pipeline: *mut ob_pipeline,
        timeout_ms: u32,
        error: *mut *mut ob_error,
    ) -> *mut ob_frame;

    fn ob_create_config(error: *mut *mut ob_error) -> *mut ob_config;
    fn ob_delete_config(config: *mut ob_config, error: *mut *mut ob_error);
    fn ob_config_enable_video_stream(
        config: *mut ob_config,
        stream_type: c_int,
        width: u32,
        height: u32,
        fps: u32,
        format: u32,
        error: *mut *mut ob_error,
    );
    fn ob_config_enable_accel_stream(
        config: *mut ob_config,
        full_scale_range: c_int,
        sample_rate: c_int,
        error: *mut *mut ob_error,
    );
    fn ob_config_enable_gyro_stream(
        config: *mut ob_config,
        full_scale_range: c_int,
        sample_rate: c_int,
        error: *mut *mut ob_error,
    );

    fn ob_delete_frame(frame: *mut ob_frame, error: *mut *mut ob_error);
    fn ob_frame_get_type(frame: *mut ob_frame, error: *mut *mut ob_error) -> c_int;
    fn ob_frame_get_format(frame: *mut ob_frame, error: *mut *mut ob_error) -> u32;
    fn ob_frame_get_timestamp_us(frame: *mut ob_frame, error: *mut *mut ob_error) -> u64;
    fn ob_frame_get_system_timestamp_us(frame: *mut ob_frame, error: *mut *mut ob_error) -> u64;
    fn ob_frame_get_global_timestamp_us(frame: *mut ob_frame, error: *mut *mut ob_error) -> u64;
    fn ob_frame_get_data(frame: *mut ob_frame, error: *mut *mut ob_error) -> *mut u8;
    fn ob_frame_get_data_size(frame: *mut ob_frame, error: *mut *mut ob_error) -> u32;
    fn ob_frame_get_stream_profile(
        frame: *mut ob_frame,
        error: *mut *mut ob_error,
    ) -> *mut ob_stream_profile;
    fn ob_video_frame_get_width(frame: *mut ob_frame, error: *mut *mut ob_error) -> u32;
    fn ob_video_frame_get_height(frame: *mut ob_frame, error: *mut *mut ob_error) -> u32;
    fn ob_accel_frame_get_value(frame: *mut ob_frame, error: *mut *mut ob_error) -> ObFloat3D;
    fn ob_gyro_frame_get_value(frame: *mut ob_frame, error: *mut *mut ob_error) -> ObFloat3D;
    fn ob_frameset_get_count(frameset: *mut ob_frame, error: *mut *mut ob_error) -> u32;
    fn ob_frameset_get_frame_by_index(
        frameset: *mut ob_frame,
        index: u32,
        error: *mut *mut ob_error,
    ) -> *mut ob_frame;
    fn ob_frameset_get_frame(
        frameset: *mut ob_frame,
        frame_type: c_int,
        error: *mut *mut ob_error,
    ) -> *mut ob_frame;

    fn ob_delete_stream_profile(profile: *mut ob_stream_profile, error: *mut *mut ob_error);
    fn ob_video_stream_profile_get_intrinsic(
        profile: *mut ob_stream_profile,
        error: *mut *mut ob_error,
    ) -> ObCameraIntrinsic;
    fn ob_video_stream_profile_get_distortion(
        profile: *mut ob_stream_profile,
        error: *mut *mut ob_error,
    ) -> ObCameraDistortion;
    fn ob_stream_profile_get_extrinsic_to(
        source: *mut ob_stream_profile,
        target: *mut ob_stream_profile,
        error: *mut *mut ob_error,
    ) -> ObExtrinsic;

    fn ob_create_filter(name: *const c_char, error: *mut *mut ob_error) -> *mut ob_filter;
    fn ob_delete_filter(filter: *mut ob_filter, error: *mut *mut ob_error);
    fn ob_filter_set_config_value(
        filter: *mut ob_filter,
        config_name: *const c_char,
        value: c_double,
        error: *mut *mut ob_error,
    );
    fn ob_filter_process(
        filter: *mut ob_filter,
        frame: *mut ob_frame,
        error: *mut *mut ob_error,
    ) -> *mut ob_frame;

    // The error accessors are the one family that takes no out-param, because
    // there would be nothing left to report it into.
    fn ob_error_get_message(error: *mut ob_error) -> *const c_char;
    fn ob_delete_error(error: *mut ob_error);
}

// -- error and handle plumbing ------------------------------------------------

/// Turns libobsensor's out-param error into a Rust one, freeing it either way.
///
/// # Safety
/// `error` must be the out-param of exactly one libobsensor call.
unsafe fn check(error: *mut ob_error, doing: &str) -> Result<()> {
    if error.is_null() {
        return Ok(());
    }
    let raw = ob_error_get_message(error);
    let message = if raw.is_null() {
        String::from("no message")
    } else {
        CStr::from_ptr(raw).to_string_lossy().into_owned()
    };
    ob_delete_error(error);
    Err(anyhow!("libobsensor failed {doing}: {message}"))
}

/// Runs one libobsensor call, checking and freeing its error out-param.
macro_rules! ob {
    ($doing:expr, $call:ident ( $($argument:expr),* $(,)? )) => {{
        let mut error: *mut ob_error = ptr::null_mut();
        let value = $call($($argument,)* &mut error);
        check(error, $doing)?;
        value
    }};
}

/// Runs a libobsensor call whose failure is not worth aborting for.
macro_rules! ob_ignoring_errors {
    ($call:ident ( $($argument:expr),* $(,)? )) => {{
        let mut error: *mut ob_error = ptr::null_mut();
        let value = $call($($argument,)* &mut error);
        if !error.is_null() {
            ob_delete_error(error);
        }
        value
    }};
}

/// An owned libobsensor handle.
///
/// The deleter takes an `ob_error**` like everything else in this SDK. Dropping
/// cannot return a failure, so the out-param is accepted and discarded — but it
/// still has to be *passed*, since a null there is what several of the deleters
/// dereference.
struct Owned<T> {
    raw: *mut T,
    delete: unsafe extern "C" fn(*mut T, *mut *mut ob_error),
}

impl<T> Owned<T> {
    /// # Safety
    /// `pointer` must be a live handle that `delete` is the correct deleter for,
    /// and must not be freed anywhere else.
    unsafe fn new(
        pointer: *mut T,
        delete: unsafe extern "C" fn(*mut T, *mut *mut ob_error),
    ) -> Result<Self> {
        if pointer.is_null() {
            return Err(anyhow!("libobsensor returned a null handle"));
        }
        Ok(Owned {
            raw: pointer,
            delete,
        })
    }

    /// Deliberately a method rather than a public field, for the same reason as
    /// in the RealSense backend: a closure reading a `raw` field would capture
    /// only the non-`Send` pointer and let the guard drop out from under the
    /// thread still using it. Going through `&self` captures the whole guard.
    fn pointer(&self) -> *mut T {
        self.raw
    }
}

impl<T> Drop for Owned<T> {
    fn drop(&mut self) {
        let mut error: *mut ob_error = ptr::null_mut();
        unsafe {
            (self.delete)(self.raw, &mut error);
            if !error.is_null() {
                ob_delete_error(error);
            }
        }
    }
}

// The pipeline and device handles are moved onto the capture thread. libobsensor
// is thread-safe for these objects; the raw pointer is what Rust objects to.
unsafe impl<T> Send for Owned<T> {}

// -- the streaming session ----------------------------------------------------

/// Everything created by one `start`, dropped as a unit by `stop`.
struct Session {
    running: Arc<AtomicBool>,
    worker: Option<std::thread::JoinHandle<()>>,
    /// Held so the emitter can be toggled while streaming.
    device: Arc<Mutex<Owned<ob_device>>>,
    /// The factory extrinsics, read once at start because they cannot change
    /// while the device streams.
    body_transforms: Vec<TransformStamped>,
}

pub struct OrbbecBackend {
    pub config: CameraConfig,
    session: Option<Session>,
    error: Option<String>,
    detail: String,
}

impl OrbbecBackend {
    pub fn new(config: CameraConfig) -> Self {
        OrbbecBackend {
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
/// a rig that carries a RealSense *and* an Orbbec, and on a two-Orbbec rig
/// picking "device 0" would silently swap which body a recording's frames belong
/// to between runs.
unsafe fn find_device(
    context: *mut ob_context,
    serial: Option<&str>,
) -> Result<(Owned<ob_device_list>, Owned<ob_device>, String)> {
    let list = Owned::new(
        ob!("listing devices", ob_query_device_list(context)),
        ob_delete_device_list,
    )?;
    let count = ob!("counting devices", ob_device_list_get_count(list.pointer()));
    if count == 0 {
        anyhow::bail!("no Orbbec device is attached");
    }

    let mut found = Vec::new();
    for index in 0..count {
        let raw = ob_ignoring_errors!(ob_device_list_get_device_serial_number(
            list.pointer(),
            index
        ));
        let number = if raw.is_null() {
            String::new()
        } else {
            CStr::from_ptr(raw).to_string_lossy().into_owned()
        };
        let matched = match serial {
            Some(wanted) => wanted == number,
            None => count == 1,
        };
        if matched {
            let device = Owned::new(
                ob!("opening a device", ob_device_list_get_device(list.pointer(), index)),
                ob_delete_device,
            )?;
            return Ok((list, device, number));
        }
        found.push(number);
    }

    match serial {
        Some(wanted) => anyhow::bail!("no Orbbec with serial {wanted}; attached: {found:?}"),
        None => anyhow::bail!(
            "{count} Orbbec devices are attached ({found:?}), so the serial setting must say which"
        ),
    }
}

/// Toggles the IR projector.
unsafe fn set_emitter(device: *mut ob_device, on: bool) -> Result<()> {
    ob!(
        "setting the emitter",
        ob_device_set_bool_property(device, OB_PROP_LASER_BOOL, on)
    );
    Ok(())
}

/// The stream a frame type belongs to, or `None` for anything we do not publish.
fn stream_of_frame(frame_type: c_int) -> Option<StreamId> {
    [
        StreamId::Depth,
        StreamId::Color,
        StreamId::InfraLeft,
        StreamId::InfraRight,
    ]
    .into_iter()
    .find(|stream| ob_frame_for(*stream) == Some(frame_type))
}

impl Backend for OrbbecBackend {
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
        self.detail = "released".into();
    }

    fn status(&self) -> BackendStatus {
        BackendStatus {
            running: self.session.is_some(),
            detail: self.detail.clone(),
            error: self.error.clone(),
        }
    }

    fn body_transforms(&self) -> Vec<TransformStamped> {
        self.session
            .as_ref()
            .map(|session| session.body_transforms.clone())
            .unwrap_or_default()
    }

    /// Only the emitter can change without a restart, matching the RealSense
    /// backend: resolution, frame rate and stream selection are fixed when the
    /// pipeline starts, and alignment changes what is published.
    fn apply_live(&mut self, config: &CameraConfig) -> bool {
        let restart_needed = config.width != self.config.width
            || config.height != self.config.height
            || config.frame_rate != self.config.frame_rate
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
            let device = session.device.lock().unwrap();
            if let Err(error) = unsafe { set_emitter(device.pointer(), config.emitter) } {
                self.error = Some(format!("{error:#}"));
                return false;
            }
        }
        self.config = config.clone();
        true
    }
}

impl OrbbecBackend {
    /// Builds a config for this camera's settings and starts a pipeline on it.
    ///
    /// # Safety
    /// `device` must be a live device handle, outliving the returned pipeline.
    unsafe fn start_pipeline(
        &self,
        device: *mut ob_device,
        with_motion: bool,
    ) -> Result<(Owned<ob_config>, Owned<ob_pipeline>)> {
        let config = Owned::new(ob!("creating a config", ob_create_config()), ob_delete_config)?;
        for stream in self.config.streams() {
            if let Some(kind) = ob_stream_for(stream) {
                ob!(
                    "enabling an image stream",
                    ob_config_enable_video_stream(
                        config.pointer(),
                        kind,
                        self.config.width,
                        self.config.height,
                        self.config.frame_rate,
                        ob_format_for(stream),
                    )
                );
            }
        }
        if with_motion {
            // The two inertial streams are enabled through their own calls
            // rather than `enable_video_stream`, because they carry a full-scale
            // range instead of a resolution.
            ob!(
                "enabling the accelerometer",
                ob_config_enable_accel_stream(
                    config.pointer(),
                    OB_ACCEL_FS_4G,
                    OB_SAMPLE_RATE_200_HZ,
                )
            );
            ob!(
                "enabling the gyroscope",
                ob_config_enable_gyro_stream(
                    config.pointer(),
                    OB_GYRO_FS_1000DPS,
                    OB_SAMPLE_RATE_200_HZ,
                )
            );
        }

        let pipeline = Owned::new(
            ob!("creating the pipeline", ob_create_pipeline_with_device(device)),
            ob_delete_pipeline,
        )?;
        ob!(
            "starting the pipeline",
            ob_pipeline_start_with_config(pipeline.pointer(), config.pointer())
        );
        Ok((config, pipeline))
    }

    /// # Safety
    /// Calls into libobsensor throughout. Every handle taken here is owned by an
    /// `Owned` or moved onto the worker thread, so none outlive this function
    /// uncontrolled.
    unsafe fn open(&mut self, sink: Sink) -> Result<()> {
        let context = Owned::new(ob!("creating the context", ob_create_context()), ob_delete_context)?;
        let (_list, device, serial) = find_device(context.pointer(), self.config.serial.as_deref())
            .context("selecting an Orbbec")?;
        set_emitter(device.pointer(), self.config.emitter)?;

        // Same reasoning as the RealSense backend: losing every image stream
        // because the host cannot reach the IMU is a worse outcome than
        // recording without one, and an unresolvable inertial request fails the
        // whole pipeline rather than just its own stream.
        let mut publish_imu = self.config.imu;
        let (config, pipeline) = match self.start_pipeline(device.pointer(), publish_imu) {
            Ok(started) => started,
            Err(with_imu) if publish_imu => {
                publish_imu = false;
                self.error = Some(format!("IMU unavailable, streaming video only: {with_imu:#}"));
                self.start_pipeline(device.pointer(), false)?
            }
            Err(error) => return Err(error),
        };

        let aligner = if self.config.align_depth_to_color {
            let name = CString::new("Align")?;
            let filter = Owned::new(
                ob!("creating the depth-to-colour aligner", ob_create_filter(name.as_ptr())),
                ob_delete_filter,
            )?;
            let key = CString::new("AlignType")?;
            ob!(
                "aiming the aligner at the colour stream",
                ob_filter_set_config_value(
                    filter.pointer(),
                    key.as_ptr(),
                    OB_STREAM_COLOR as c_double,
                )
            );
            Some(filter)
        } else {
            None
        };

        // Unlike librealsense, libobsensor has no handle that reports the active
        // profiles before a frame arrives — `ob_pipeline_get_stream_profile_list`
        // answers what the sensor *supports*. So the first frameset is pulled
        // here, both to read the extrinsics off its frames and to fail `start`
        // rather than the worker thread when a camera never delivers.
        let first = Owned::new(
            ob!(
                "waiting for the first frameset",
                ob_pipeline_wait_for_frameset(pipeline.pointer(), FRAMESET_TIMEOUT_MS)
            ),
            ob_delete_frame,
        )
        .context("the camera started but delivered no frames")?;
        let transforms = body_extrinsics(
            first.pointer(),
            &self.config.naming,
            crate::record::now_nanos(),
        )?;

        let running = Arc::new(AtomicBool::new(true));
        let device = Arc::new(Mutex::new(device));
        self.detail = format!("{serial} @ {}x{}", self.config.width, self.config.height);

        let worker = {
            let running = Arc::clone(&running);
            let naming = self.config.naming.clone();
            // Held for the thread's lifetime so the device is not released early.
            let held = (context, config, device.clone());
            std::thread::Builder::new()
                .name("orbbec".into())
                .spawn(move || {
                    let _held = held;
                    let mut pump = Pump::new(naming, sink, publish_imu);
                    // The frameset that paid for the extrinsics still carries a
                    // full set of images; dropping it would put a hole at the
                    // start of every recording.
                    unsafe {
                        if let Err(error) = pump.publish_frameset(first.pointer(), aligner.as_ref())
                        {
                            eprintln!("orbbec: {error:#}");
                        }
                    }
                    drop(first);
                    while running.load(Ordering::SeqCst) {
                        unsafe {
                            if let Err(error) = pump.step(pipeline.pointer(), aligner.as_ref()) {
                                eprintln!("orbbec: {error:#}");
                            }
                        }
                    }
                    unsafe {
                        ob_ignoring_errors!(ob_pipeline_stop(pipeline.pointer()));
                    }
                })?
        };

        self.session = Some(Session {
            running,
            worker: Some(worker),
            device,
            body_transforms: transforms,
        });
        Ok(())
    }
}

// -- the frame pump -----------------------------------------------------------

/// Turns framesets into `Produced` values.
///
/// It carries state for exactly two reasons, both shared with the RealSense
/// pump: CameraInfo is republished only when a stream first appears rather than
/// on every frame, and a gyro sample needs the most recent acceleration to
/// become one `sensor_msgs/Imu`.
struct Pump {
    naming: super::super::Naming,
    sink: Sink,
    publish_imu: bool,
    announced: std::collections::BTreeSet<StreamId>,
    latest_acceleration: [f64; 3],
}

impl Pump {
    fn new(naming: super::super::Naming, sink: Sink, publish_imu: bool) -> Self {
        Pump {
            naming,
            sink,
            publish_imu,
            announced: Default::default(),
            latest_acceleration: [0.0; 3],
        }
    }

    /// Waits for one frameset and publishes everything inside it.
    ///
    /// # Safety
    /// `pipeline` must be a started pipeline.
    unsafe fn step(&mut self, pipeline: *mut ob_pipeline, aligner: Option<&Owned<ob_filter>>) -> Result<()> {
        let mut error: *mut ob_error = ptr::null_mut();
        let frameset = ob_pipeline_wait_for_frameset(pipeline, FRAMESET_TIMEOUT_MS, &mut error);
        if !error.is_null() {
            ob_delete_error(error);
            return Ok(());
        }
        if frameset.is_null() {
            // A null without an error is how a timeout is reported, and the
            // caller's loop re-checks the stop flag, so it is not fatal here.
            return Ok(());
        }
        let frameset = Owned::new(frameset, ob_delete_frame)?;
        self.publish_frameset(frameset.pointer(), aligner)
    }

    unsafe fn publish_frameset(
        &mut self,
        frameset: *mut ob_frame,
        aligner: Option<&Owned<ob_filter>>,
    ) -> Result<()> {
        let count = ob!("counting frames in a frameset", ob_frameset_get_count(frameset));
        for index in 0..count {
            let frame = Owned::new(
                ob!(
                    "extracting a frame",
                    ob_frameset_get_frame_by_index(frameset, index)
                ),
                ob_delete_frame,
            )?;
            self.publish_frame(frame.pointer())?;
        }

        // The aligned depth is a second, reprojected copy. It is published on its
        // own topic so a reader can still get the sensor's native depth.
        if let Some(aligner) = aligner {
            let aligned = ob_ignoring_errors!(ob_filter_process(aligner.pointer(), frameset));
            if !aligned.is_null() {
                let aligned = Owned::new(aligned, ob_delete_frame)?;
                // The filter hands back a whole frameset, colour included; only
                // the reprojected depth is new.
                let depth = ob_ignoring_errors!(ob_frameset_get_frame(
                    aligned.pointer(),
                    OB_FRAME_DEPTH
                ));
                if !depth.is_null() {
                    let depth = Owned::new(depth, ob_delete_frame)?;
                    self.publish_aligned(depth.pointer())?;
                }
            }
        }
        Ok(())
    }

    /// A frame's stamp, in nanoseconds.
    ///
    /// The global timestamp is the one already on the host clock, so it is what
    /// makes an Orbbec frame comparable with a Mid-360 point. It reads zero on
    /// devices or firmware without global timestamping, hence the fallbacks.
    unsafe fn stamp_nanos(&self, frame: *mut ob_frame) -> u64 {
        for reader in [
            ob_frame_get_global_timestamp_us as unsafe extern "C" fn(_, _) -> u64,
            ob_frame_get_system_timestamp_us,
            ob_frame_get_timestamp_us,
        ] {
            let mut error: *mut ob_error = ptr::null_mut();
            let microseconds = reader(frame, &mut error);
            if !error.is_null() {
                ob_delete_error(error);
                continue;
            }
            if microseconds != 0 {
                return microseconds * 1_000;
            }
        }
        crate::record::now_nanos()
    }

    unsafe fn publish_frame(&mut self, frame: *mut ob_frame) -> Result<()> {
        let frame_type = ob!("reading a frame type", ob_frame_get_type(frame));
        let stamp_nanos = self.stamp_nanos(frame);
        if frame_type == OB_FRAME_ACCEL || frame_type == OB_FRAME_GYRO {
            return self.publish_motion(frame, frame_type, stamp_nanos);
        }
        let Some(stream) = stream_of_frame(frame_type) else {
            return Ok(());
        };
        let format = ob!("reading a frame format", ob_frame_get_format(frame));
        self.publish_image(frame, stream, format, stamp_nanos, None)
    }

    unsafe fn publish_aligned(&mut self, frame: *mut ob_frame) -> Result<()> {
        let format = ob!("reading a frame format", ob_frame_get_format(frame));
        let stamp_nanos = self.stamp_nanos(frame);
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
        frame: *mut ob_frame,
        stream: StreamId,
        format: u32,
        stamp_nanos: u64,
        override_naming: Option<(String, String)>,
    ) -> Result<()> {
        let Some((encoding, bytes_per_pixel)) = ros_encoding(format) else {
            // Publishing under a guessed encoding string silently corrupts every
            // reader, so an unmapped format is dropped and said out loud.
            anyhow::bail!(
                "stream {} arrived in unmapped ob_format {format}",
                stream.as_str()
            );
        };
        let width = ob!("reading a frame width", ob_video_frame_get_width(frame)) as usize;
        let height = ob!("reading a frame height", ob_video_frame_get_height(frame)) as usize;
        let size = ob!("reading a frame size", ob_frame_get_data_size(frame)) as usize;
        let data = ob!("reading frame data", ob_frame_get_data(frame)) as *const u8;
        if data.is_null() {
            anyhow::bail!("libobsensor handed back a null frame buffer");
        }
        // libobsensor has no stride getter, so the row length is recovered from
        // the buffer. Preferring the measured value over `width * bytes` is what
        // keeps a padded row from shearing the image diagonally.
        let step = if height > 0 && size / height >= width * bytes_per_pixel {
            size / height
        } else {
            width * bytes_per_pixel
        };
        if step * height > size {
            anyhow::bail!(
                "{} frame is {size} bytes, too small for {width}x{height} {encoding}",
                stream.as_str()
            );
        }

        let (topic, frame_id) = match override_naming {
            Some(pair) => pair,
            None => (self.naming.image_topic(stream), self.naming.frame_id(stream)),
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
        frame: *mut ob_frame,
        stream: StreamId,
        stamp_nanos: u64,
        frame_id: &str,
    ) -> Result<()> {
        let profile = Owned::new(
            ob!("reading a frame's profile", ob_frame_get_stream_profile(frame)),
            ob_delete_stream_profile,
        )?;
        let intrinsic = ob!(
            "reading intrinsics",
            ob_video_stream_profile_get_intrinsic(profile.pointer())
        );
        let distortion = ob!(
            "reading distortion",
            ob_video_stream_profile_get_distortion(profile.pointer())
        );
        let info = camera_info(
            stamp_nanos,
            frame_id,
            intrinsic.width as u32,
            intrinsic.height as u32,
            [intrinsic.fx as f64, intrinsic.fy as f64],
            [intrinsic.cx as f64, intrinsic.cy as f64],
            ObDistortionModel::from_raw(distortion.model),
            ObDistortion {
                k1: distortion.k1 as f64,
                k2: distortion.k2 as f64,
                k3: distortion.k3 as f64,
                k4: distortion.k4 as f64,
                k5: distortion.k5 as f64,
                k6: distortion.k6 as f64,
                p1: distortion.p1 as f64,
                p2: distortion.p2 as f64,
            },
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
        frame: *mut ob_frame,
        frame_type: c_int,
        stamp_nanos: u64,
    ) -> Result<()> {
        if !self.publish_imu {
            return Ok(());
        }
        if frame_type == OB_FRAME_ACCEL {
            // Accelerometer samples only update the pairing state. Publishing on
            // them too would double the message rate and emit an Imu whose
            // angular velocity is stale instead of whose acceleration is.
            let value = ob!("reading acceleration", ob_accel_frame_get_value(frame));
            self.latest_acceleration = [value.x as f64, value.y as f64, value.z as f64];
            return Ok(());
        }
        let value = ob!("reading angular velocity", ob_gyro_frame_get_value(frame));
        let imu = Imu::unoriented(
            Header::new(stamp_nanos, self.naming.frame_id(StreamId::Imu)),
            [value.x as f64, value.y as f64, value.z as f64],
            self.latest_acceleration,
        );
        (self.sink)(Produced::Imu {
            topic: self.naming.imu_topic(),
            imu: Box::new(imu),
        });
        Ok(())
    }
}

/// The factory extrinsics between the streams of one device, ready for
/// `/tf_static`, read off the frames of one frameset.
///
/// # Safety
/// `frameset` must be a live frameset.
unsafe fn body_extrinsics(
    frameset: *mut ob_frame,
    naming: &super::super::Naming,
    stamp_nanos: u64,
) -> Result<Vec<TransformStamped>> {
    let count = ob!("counting frames in a frameset", ob_frameset_get_count(frameset));

    let mut depth_profile: Option<Owned<ob_stream_profile>> = None;
    let mut others = Vec::new();
    for index in 0..count {
        let frame = Owned::new(
            ob!(
                "extracting a frame",
                ob_frameset_get_frame_by_index(frameset, index)
            ),
            ob_delete_frame,
        )?;
        let frame_type = ob!("reading a frame type", ob_frame_get_type(frame.pointer()));
        let stream = match frame_type {
            OB_FRAME_GYRO => StreamId::Imu,
            // Only the gyro carries the inertial frame; pairing means the accel
            // is reported in the same place, so it must not add a second edge.
            OB_FRAME_ACCEL => continue,
            other => match stream_of_frame(other) {
                Some(stream) => stream,
                None => continue,
            },
        };
        let profile = Owned::new(
            ob!(
                "reading a frame's profile",
                ob_frame_get_stream_profile(frame.pointer())
            ),
            ob_delete_stream_profile,
        )?;
        if frame_type == OB_FRAME_DEPTH {
            depth_profile = Some(profile);
        } else {
            others.push((stream, profile));
        }
    }

    // Every edge here is expressed relative to depth, so without it there is
    // nothing to hang the others off.
    let Some(depth_profile) = depth_profile else {
        return Ok(Vec::new());
    };

    let mut extrinsics = Vec::new();
    for (stream, profile) in &others {
        let raw = ob!(
            "reading an extrinsic",
            ob_stream_profile_get_extrinsic_to(depth_profile.pointer(), profile.pointer())
        );
        extrinsics.push(extrinsic_to_body(*stream, raw.rot, raw.trans));
    }
    Ok(body_transforms(naming, stamp_nanos, &extrinsics))
}
