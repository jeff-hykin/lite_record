//! The depthai half of the OAK-D backend, compiled only under the `oakd`
//! feature.
//!
//! depthai-core is C++17 and publishes no C API, so unlike the other two
//! backends there is nothing here to declare externs against directly. The
//! bindings below are to `oakd_shim.cpp`, which is built by `build.rs` and
//! linked into this crate; every function it exposes is listed here and nowhere
//! else.
//!
//! One structural difference from the RealSense and Orbbec backends. Those SDKs
//! hand back a whole frameset per call, so one thread drains everything. depthai
//! gives each output its own queue, and a 400 Hz IMU sharing a thread with a
//! 30 Hz colour stream would have its samples arrive in bursts of thirteen. So
//! each stream gets its own thread, which is also what makes the shim's
//! no-locking, one-held-message-per-stream contract sound.

use std::os::raw::{c_char, c_int};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Result};

use super::super::{Backend, BackendStatus, CameraConfig, Naming, Produced, Sink, StreamId};
use super::{
    body_transforms, camera_info, nearest_imu_rate, ros_encoding, shim_stream_index, socket_for,
    BodyExtrinsic, OakCameraModel, StreamCalibration, StrictlyIncreasing, CAM_A,
    CAM_B, CAM_C,
};
use crate::msgs::{Header, Imu, RawImage, TransformStamped};

/// How long a stream thread blocks before looking at the stop flag again. Short
/// enough that disengaging feels immediate in the browser, long enough that a
/// 5 fps stream does not spin.
const WAIT_TIMEOUT_MS: c_int = 1_000;

/// Room for one `std::exception::what()`. depthai's are single sentences.
const ERROR_CAPACITY: usize = 512;

// -- the shim's C API ---------------------------------------------------------

#[repr(C)]
pub struct LrOakDevice {
    _private: [u8; 0],
}

#[repr(C)]
struct LrOakConfig {
    color: i32,
    depth: i32,
    infrared: i32,
    imu: i32,
    width: i32,
    height: i32,
    frame_rate: i32,
    imu_rate: i32,
    emitter: i32,
    align_depth_to_color: i32,
    serial: *const c_char,
}

#[repr(C)]
#[derive(Default)]
struct LrOakSample {
    stream: i32,
    pixels: i32,
    width: i32,
    height: i32,
    data: *const u8,
    length: u64,
    device_stamp_nanos: u64,
    accelerometer: [f64; 3],
    gyroscope: [f64; 3],
}

impl LrOakSample {
    fn empty() -> Self {
        LrOakSample {
            data: std::ptr::null(),
            ..Default::default()
        }
    }
}

#[repr(C)]
#[derive(Default)]
struct LrOakCalibration {
    model: i32,
    width: i32,
    height: i32,
    intrinsics: [f64; 9],
    coefficients: [f64; 14],
    coefficient_count: i32,
    baseline_centimetres: f64,
}

extern "C" {
    fn lr_oak_steady_now_nanos() -> u64;
    fn lr_oak_open(
        config: *const LrOakConfig,
        error: *mut c_char,
        error_capacity: usize,
    ) -> *mut LrOakDevice;
    fn lr_oak_close(handle: *mut LrOakDevice);
    fn lr_oak_has_stream(handle: *const LrOakDevice, stream: i32) -> i32;
    fn lr_oak_wait(
        handle: *mut LrOakDevice,
        stream: i32,
        timeout_ms: c_int,
        out: *mut LrOakSample,
        error: *mut c_char,
        error_capacity: usize,
    ) -> i32;
    fn lr_oak_calibration(
        handle: *mut LrOakDevice,
        socket: i32,
        width: i32,
        height: i32,
        out: *mut LrOakCalibration,
    ) -> i32;
    fn lr_oak_extrinsics(
        handle: *mut LrOakDevice,
        socket: i32,
        rotation: *mut f64,
        translation: *mut f64,
    ) -> i32;
    fn lr_oak_imu_extrinsics(
        handle: *mut LrOakDevice,
        rotation: *mut f64,
        translation: *mut f64,
    ) -> i32;
    fn lr_oak_set_emitter(handle: *mut LrOakDevice, on: i32) -> i32;
}

/// The open device, closed when the last stream thread lets go of it.
///
/// `lr_oak_wait` touches only the slot for the stream it was asked about, so the
/// threads need no lock between them. Everything else the shim exposes reaches
/// the device object itself and goes through `elsewhere` first.
struct Device {
    raw: *mut LrOakDevice,
    elsewhere: Mutex<()>,
}

// The handle is shared by every stream thread. What Rust objects to is the raw
// pointer; the contract that makes sharing it sound is stated above.
unsafe impl Send for Device {}
unsafe impl Sync for Device {}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe { lr_oak_close(self.raw) };
    }
}

/// Reads back whatever the shim wrote into an error buffer.
fn message_from(buffer: &[c_char]) -> String {
    let text = unsafe { std::ffi::CStr::from_ptr(buffer.as_ptr()) };
    text.to_string_lossy().into_owned()
}

impl Device {
    fn open(config: &CameraConfig, imu_rate: u32) -> Result<Self> {
        let serial = config
            .serial
            .as_deref()
            .map(std::ffi::CString::new)
            .transpose()?;
        let request = LrOakConfig {
            color: config.color as i32,
            depth: config.depth as i32,
            infrared: config.infrared as i32,
            imu: config.imu as i32,
            width: config.width as i32,
            height: config.height as i32,
            frame_rate: config.frame_rate as i32,
            imu_rate: imu_rate as i32,
            emitter: config.emitter as i32,
            align_depth_to_color: config.align_depth_to_color as i32,
            serial: serial
                .as_ref()
                .map(|text| text.as_ptr())
                .unwrap_or(std::ptr::null()),
        };

        let mut error = [0 as c_char; ERROR_CAPACITY];
        let raw = unsafe { lr_oak_open(&request, error.as_mut_ptr(), error.len()) };
        if raw.is_null() {
            return Err(anyhow!("depthai could not open the camera: {}", message_from(&error)));
        }
        Ok(Device {
            raw,
            elsewhere: Mutex::new(()),
        })
    }

    fn has_stream(&self, stream: StreamId) -> bool {
        let Some(index) = shim_stream_index(stream) else {
            return false;
        };
        unsafe { lr_oak_has_stream(self.raw, index) == 1 }
    }

    fn set_emitter(&self, on: bool) -> Result<()> {
        let _guard = self.elsewhere.lock().unwrap();
        if unsafe { lr_oak_set_emitter(self.raw, on as i32) } != 0 {
            anyhow::bail!("the dot projector would not switch");
        }
        Ok(())
    }

    fn calibration(&self, socket: i32, width: u32, height: u32) -> Option<LrOakCalibration> {
        let _guard = self.elsewhere.lock().unwrap();
        let mut out = LrOakCalibration::default();
        let code =
            unsafe { lr_oak_calibration(self.raw, socket, width as i32, height as i32, &mut out) };
        (code == 0).then_some(out)
    }

    /// The factory transform from the left imager to `socket`, or from it to the
    /// inertial part when `socket` is `None`.
    fn extrinsic(&self, socket: Option<i32>) -> Option<([f64; 9], [f64; 3])> {
        let _guard = self.elsewhere.lock().unwrap();
        let mut rotation = [0.0f64; 9];
        let mut translation = [0.0f64; 3];
        let code = unsafe {
            match socket {
                Some(socket) => {
                    lr_oak_extrinsics(self.raw, socket, rotation.as_mut_ptr(), translation.as_mut_ptr())
                }
                None => lr_oak_imu_extrinsics(self.raw, rotation.as_mut_ptr(), translation.as_mut_ptr()),
            }
        };
        (code == 0).then_some((rotation, translation))
    }
}

// -- the backend --------------------------------------------------------------

struct Session {
    running: Arc<AtomicBool>,
    workers: Vec<std::thread::JoinHandle<()>>,
    device: Arc<Device>,
    body_transforms: Vec<TransformStamped>,
}

pub struct OakdBackend {
    pub config: CameraConfig,
    session: Option<Session>,
    error: Option<String>,
    detail: String,
}

impl OakdBackend {
    pub fn new(config: CameraConfig) -> Self {
        OakdBackend {
            config,
            session: None,
            error: None,
            detail: String::new(),
        }
    }
}

impl Backend for OakdBackend {
    fn start(&mut self, sink: Sink) -> Result<()> {
        if self.session.is_some() {
            return Ok(());
        }
        self.error = None;
        match self.open(sink) {
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
        for worker in session.workers {
            let _ = worker.join();
        }
        // Every thread held a clone; dropping the last one closes the pipeline
        // and releases the USB device for the next process to claim.
        drop(session.device);
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

    /// Only the emitter can change without a restart. Everything else is fixed
    /// when the pipeline's nodes are created, which is at open.
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
            if let Some(session) = self.session.as_ref() {
                if let Err(error) = session.device.set_emitter(config.emitter) {
                    self.error = Some(format!("{error:#}"));
                    return false;
                }
            }
        }
        self.config = config.clone();
        true
    }
}

impl OakdBackend {
    fn open(&mut self, sink: Sink) -> Result<()> {
        let imu_rate = nearest_imu_rate(self.config.imu_rate);
        let device = Arc::new(Device::open(&self.config, imu_rate)?);

        // Sampled per frame rather than once for the session. The device's own
        // stamp is what carries the spacing, but the offset onto the host clock
        // has to keep following it — a Pi's clock is corrected by NTP some way
        // into a run, and an offset taken before that leaves every later stamp
        // behind. See `crate::clock`.

        let transforms = self.read_body_transforms(&device, crate::record::now_nanos());

        let running = Arc::new(AtomicBool::new(true));
        let mut workers = Vec::new();
        let mut started = Vec::new();
        for stream in self.config.streams() {
            if !device.has_stream(stream) {
                continue;
            }
            let index = shim_stream_index(stream).expect("an enabled stream has a shim index");
            let mut pump = Pump {
                naming: self.config.naming.clone(),
                sink: Arc::clone(&sink),
                clock: crate::clock::HostClock::default(),
                stamps: StrictlyIncreasing::default(),
                stream,
                announced: false,
            };
            let running = Arc::clone(&running);
            let device = Arc::clone(&device);
            workers.push(
                std::thread::Builder::new()
                    .name(format!("oakd-{}", stream.as_str()))
                    .spawn(move || {
                        while running.load(Ordering::SeqCst) {
                            if let Err(error) = pump.step(&device, index) {
                                eprintln!("oakd: {error:#}");
                            }
                        }
                    })?,
            );
            started.push(stream.as_str());
        }

        if workers.is_empty() {
            anyhow::bail!("the camera opened but not one of the requested streams exists on it");
        }

        self.detail = format!(
            "{}x{} @ {}fps, imu {imu_rate}Hz, streaming {}",
            self.config.width,
            self.config.height,
            self.config.frame_rate,
            started.join(", ")
        );
        self.session = Some(Session {
            running,
            workers,
            device,
            body_transforms: transforms,
        });
        Ok(())
    }

    /// The factory extrinsics for `/tf_static`, read once because they cannot
    /// change while the device is open.
    ///
    /// Every edge is measured from the left mono imager, which is the frame
    /// depth is computed in. An edge the device declines to report sinks the
    /// whole set: a tree missing one camera is worse than the hub's identity
    /// fallback, which is at least uniformly wrong rather than wrong in one
    /// place a reader would have to notice.
    fn read_body_transforms(&self, device: &Device, stamp_nanos: u64) -> Vec<TransformStamped> {
        let depth_socket = if self.config.align_depth_to_color {
            CAM_A
        } else {
            CAM_B
        };
        let mut extrinsics = Vec::new();
        for stream in self.config.streams() {
            let source = match stream {
                // The root itself, already published as the edge off `_link`.
                StreamId::InfraLeft => continue,
                // Not addressed by a socket, so it has its own call.
                StreamId::Imu => None,
                StreamId::Depth if depth_socket == CAM_B => {
                    // Unaligned depth *is* the left imager's frame. Asking the
                    // device for CAM_B relative to CAM_B is not a question it
                    // answers, and identity here is the measurement, not a
                    // fallback.
                    extrinsics.push(BodyExtrinsic {
                        child: StreamId::Depth,
                        rotation: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        translation_centimetres: [0.0; 3],
                    });
                    continue;
                }
                StreamId::Depth => Some(depth_socket),
                StreamId::Color => Some(CAM_A),
                StreamId::InfraRight => Some(CAM_C),
                StreamId::PointCloud => continue,
            };
            let Some((rotation, translation_centimetres)) = device.extrinsic(source) else {
                return Vec::new();
            };
            extrinsics.push(BodyExtrinsic {
                child: stream,
                rotation,
                translation_centimetres,
            });
        }
        body_transforms(&self.config.naming, stamp_nanos, &extrinsics)
    }
}

// -- the frame pump -----------------------------------------------------------

/// Turns one stream's samples into `Produced` values. One per thread, so the
/// only state it carries is that stream's own.
struct Pump {
    naming: Naming,
    sink: Sink,
    clock: crate::clock::HostClock,
    stamps: StrictlyIncreasing,
    stream: StreamId,
    announced: bool,
}

impl Pump {
    fn step(&mut self, device: &Device, index: i32) -> Result<()> {
        let mut sample = LrOakSample::empty();
        let mut error = [0 as c_char; ERROR_CAPACITY];
        let code = unsafe {
            lr_oak_wait(
                device.raw,
                index,
                WAIT_TIMEOUT_MS,
                &mut sample,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        match code {
            0 => return Ok(()),
            1 => {}
            _ => {
                return Err(anyhow!(
                    "{} stream failed: {}",
                    self.stream.as_str(),
                    message_from(&error)
                ))
            }
        }

        // The device's own stamp for the sample, offset onto the epoch. Not the
        // moment it arrived here: that would carry this thread's scheduling
        // latency into the recording.
        let steady_now = unsafe { lr_oak_steady_now_nanos() };
        let host_now = crate::record::now_nanos();
        // The device stamp is on the same steady clock this reads, so the
        // estimate is offered the pair as it stands right now and the sample's
        // own stamp is what gets mapped.
        self.clock.map(steady_now, host_now);
        let stamp_nanos = self
            .stamps
            .next(self.clock.epoch_for(sample.device_stamp_nanos));
        if self.stream == StreamId::Imu {
            self.publish_imu(&sample, stamp_nanos);
            return Ok(());
        }
        self.publish_image(device, &sample, stamp_nanos)
    }

    fn publish_imu(&self, sample: &LrOakSample, stamp_nanos: u64) {
        let imu = Imu::unoriented(
            Header::new(stamp_nanos, self.naming.frame_id(StreamId::Imu)),
            sample.gyroscope,
            sample.accelerometer,
        );
        (self.sink)(Produced::Imu {
            topic: self.naming.imu_topic(),
            imu: Box::new(imu),
        });
    }

    fn publish_image(
        &mut self,
        device: &Device,
        sample: &LrOakSample,
        stamp_nanos: u64,
    ) -> Result<()> {
        let Some((encoding, bytes_per_pixel)) = ros_encoding(sample.pixels) else {
            // Guessing an encoding string silently corrupts every reader, so an
            // unmapped layout is dropped and said out loud.
            anyhow::bail!(
                "{} arrived in unmapped layout {}",
                self.stream.as_str(),
                sample.pixels
            );
        };
        let width = sample.width.max(0) as usize;
        let height = sample.height.max(0) as usize;
        let step = width * bytes_per_pixel;
        if sample.data.is_null() || (sample.length as usize) < step * height {
            anyhow::bail!(
                "{} frame is {} bytes, too small for {width}x{height} {encoding}",
                self.stream.as_str(),
                sample.length
            );
        }

        let frame_id = self.naming.frame_id(self.stream);
        let image = RawImage {
            header: Header::new(stamp_nanos, frame_id.clone()),
            width,
            height,
            step,
            is_bigendian: 0,
            encoding: encoding.to_string(),
            data: unsafe { std::slice::from_raw_parts(sample.data, step * height) }.to_vec(),
        };
        (self.sink)(Produced::Image {
            stream: self.stream,
            topic: self.naming.image_topic(self.stream),
            image,
        });

        // Once, when the stream first delivers, rather than per frame: the
        // intrinsics are a property of the unit, not of the moment.
        if !self.announced {
            self.announced = true;
            self.publish_camera_info(device, sample, stamp_nanos, &frame_id);
        }
        Ok(())
    }

    fn publish_camera_info(
        &self,
        device: &Device,
        sample: &LrOakSample,
        stamp_nanos: u64,
        frame_id: &str,
    ) {
        let Some(socket) = socket_for(self.stream) else {
            return;
        };
        let width = sample.width.max(0) as u32;
        let height = sample.height.max(0) as u32;
        let Some(calibration) = device.calibration(socket, width, height) else {
            eprintln!(
                "oakd: no factory calibration for {}, so no camera_info",
                self.stream.as_str()
            );
            return;
        };
        let count = (calibration.coefficient_count.max(0) as usize).min(14);
        let info = camera_info(
            stamp_nanos,
            frame_id,
            &StreamCalibration {
                width,
                height,
                intrinsics: calibration.intrinsics,
                model: OakCameraModel::from_raw(calibration.model),
                coefficients: &calibration.coefficients[..count],
                baseline_centimetres: calibration.baseline_centimetres,
            },
        );
        (self.sink)(Produced::CameraInfo {
            topic: self.naming.camera_info_topic(self.stream),
            info: Box::new(info),
        });
    }
}
