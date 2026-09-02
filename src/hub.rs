//! Central state: settings, the encode pipeline, the recorder, and the numbers
//! the monitor shows.
//!
//! The pipeline is deliberately three stages. Sensor threads only decode and
//! hand off; a pool of workers does the expensive image compression; one writer
//! thread owns the mcap file. Compressing on the capture thread was the obvious
//! shortcut and is the wrong one: a slow jpeg stalls the SDK's own frame queue
//! and costs frames on every other stream too.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use serde::{Deserialize, Serialize};

use crate::image::ImageFormat;
use crate::msgs::{CameraInfo, TransformStamped};
use crate::record::{self, Compression, Recorder, RecordingStatus};
use crate::sensors::{
    Backend, CameraConfig, LivoxConfig, Naming, Produced, SensorKind, Sink, StreamId,
};
use crate::sysmon;
use crate::urdf::{self, TreeProblem};

/// Bounded so a stalled encoder sheds frames instead of eating the Pi's memory.
const ENCODE_QUEUE_DEPTH: usize = 128;

/// Rates are averaged over this window rather than instantaneously, so the
/// number in the UI does not flicker.
const RATE_WINDOW: Duration = Duration::from_secs(2);

/// Nice value for the encode workers. Compression is the only work in this
/// process that holds a core for milliseconds at a stretch, and a frame the
/// capture thread is late to collect is gone for good, so the encoders sit
/// below everything else. Lowering a thread's own priority never needs
/// privileges, which is why the gap is opened downwards rather than by raising
/// the capture threads.
#[cfg(target_os = "linux")]
const ENCODE_NICE: libc::c_int = 5;

#[cfg(target_os = "linux")]
fn yield_to_capture_threads() {
    // Linux nice is per-task and `who = 0` means the calling task, so this
    // moves one worker rather than the whole process. Elsewhere the same call
    // would renice everything, which is why this is Linux-only.
    unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, ENCODE_NICE) };
}

#[cfg(not(target_os = "linux"))]
fn yield_to_capture_threads() {}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Settings {
    pub record_dir: PathBuf,
    pub compression: Compression,
    /// How image streams are stored. Depth and infrared fall back to raw when
    /// the chosen codec cannot hold their bit depth.
    pub color_format: ImageFormat,
    pub depth_format: ImageFormat,
    pub realsense: CameraConfig,
    pub orbbec: CameraConfig,
    pub livox: LivoxConfig,
    /// Whether the browser preview is running at all.
    pub preview_enabled: bool,
    /// Which image topic the preview shows.
    pub preview_topic: Option<String>,
    pub preview_quality: u8,
    pub preview_max_width: u32,
    /// The uploaded URDF, kept verbatim so the browser's three.js viewer and
    /// the tf_static writer see exactly the same bytes.
    #[serde(default)]
    pub urdf_xml: Option<String>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            record_dir: PathBuf::from("recordings"),
            compression: Compression::default(),
            color_format: ImageFormat::Jpeg,
            depth_format: ImageFormat::Png,
            realsense: CameraConfig::for_kind(SensorKind::Realsense),
            orbbec: CameraConfig::for_kind(SensorKind::Orbbec),
            livox: LivoxConfig::default(),
            preview_enabled: true,
            preview_topic: None,
            preview_quality: 60,
            preview_max_width: 640,
            urdf_xml: None,
        }
    }
}

impl Settings {
    /// Every frame id the recording will publish data on. These are what a URDF
    /// has to cover for the tree to be complete.
    pub fn sensor_frames(&self) -> Vec<String> {
        let mut frames = Vec::new();
        for config in [&self.realsense, &self.orbbec] {
            if config.enabled {
                frames.push(config.naming.root_frame_id());
            }
        }
        if self.livox.enabled {
            frames.push(self.livox.naming.frame_id(StreamId::PointCloud));
        }
        frames
    }

    /// The image format a given stream is stored in. The split is by bit depth,
    /// not by role: only depth is 16-bit, and a colour codec would silently
    /// truncate it. Infrared is plain 8-bit grey, so forcing it down the depth
    /// path made a Pi spend two cores deflating frames a jpeg would have
    /// handled in a tenth of the time.
    pub fn format_for(&self, stream: StreamId) -> ImageFormat {
        match stream {
            StreamId::Depth => self.depth_format,
            _ => self.color_format,
        }
    }
}

/// A rolling count, used for the per-stream Hz readout.
#[derive(Default)]
struct RateCounter {
    count: u64,
    window_started: Option<Instant>,
    window_count: u64,
    hz: f64,
}

impl RateCounter {
    fn tick(&mut self) {
        self.count += 1;
        self.window_count += 1;
        let started = *self.window_started.get_or_insert_with(Instant::now);
        let elapsed = started.elapsed();
        if elapsed >= RATE_WINDOW {
            self.hz = self.window_count as f64 / elapsed.as_secs_f64();
            self.window_count = 0;
            self.window_started = Some(Instant::now());
        }
    }

    /// A stream that stopped must decay to zero rather than showing its last
    /// good rate forever.
    fn hz(&self) -> f64 {
        match self.window_started {
            Some(started) if started.elapsed() > RATE_WINDOW * 2 => 0.0,
            _ => self.hz,
        }
    }
}

#[derive(Serialize, Clone)]
pub struct StreamStats {
    pub topic: String,
    pub hz: f64,
    pub total: u64,
    pub dropped: u64,
}

pub struct Hub {
    settings: RwLock<Settings>,
    settings_file: PathBuf,
    backends: Mutex<BTreeMap<SensorKind, Box<dyn Backend>>>,
    recorder: Mutex<Option<Recorder>>,
    /// Mirrors `recorder.is_some()` so the capture thread can ask "does anyone
    /// want this frame?" without contending for the recorder lock.
    recording_active: AtomicBool,
    last_status: Mutex<RecordingStatus>,
    rates: Mutex<BTreeMap<String, RateCounter>>,
    /// Frames the pipeline shed before they reached the encoder.
    pipeline_dropped: Mutex<BTreeMap<String, u64>>,
    /// The latest intrinsics seen on each camera_info topic.
    ///
    /// A backend announces these once when it opens, not on every frame, so a
    /// recording started afterwards would otherwise contain images that no
    /// reader can project. They are replayed into each new file the same way
    /// `/tf_static` is.
    latest_intrinsics: Mutex<BTreeMap<String, CameraInfo>>,
    encode_sender: Sender<Produced>,
    /// Latest preview frame, as jpeg bytes ready to push down the websocket.
    preview: Mutex<Option<PreviewFrame>>,
    /// Counts preview encodes so switching the preview off can be shown to
    /// actually stop the work rather than just hide the result.
    preview_encodes: AtomicU64,
    /// Counts frames that reached an encode worker, for the same reason
    /// `preview_encodes` exists: idling has to be demonstrably free, not just
    /// look free from the outside.
    record_encodes: AtomicU64,
    preview_wanted: AtomicBool,
    health: Mutex<sysmon::Sampler>,
    workers: Mutex<Vec<std::thread::JoinHandle<()>>>,
}

#[derive(Clone)]
pub struct PreviewFrame {
    pub topic: String,
    pub bytes: bytes::Bytes,
    pub width: u32,
    pub height: u32,
}

impl Hub {
    pub fn new(settings: Settings, settings_file: PathBuf) -> Arc<Self> {
        let (encode_sender, encode_receiver) = bounded(ENCODE_QUEUE_DEPTH);
        let hub = Arc::new(Hub {
            preview_wanted: AtomicBool::new(settings.preview_enabled),
            settings: RwLock::new(settings),
            settings_file,
            backends: Mutex::new(BTreeMap::new()),
            recorder: Mutex::new(None),
            recording_active: AtomicBool::new(false),
            last_status: Mutex::new(record::idle_status()),
            rates: Mutex::new(BTreeMap::new()),
            pipeline_dropped: Mutex::new(BTreeMap::new()),
            latest_intrinsics: Mutex::new(BTreeMap::new()),
            encode_sender,
            preview: Mutex::new(None),
            preview_encodes: AtomicU64::new(0),
            record_encodes: AtomicU64::new(0),
            health: Mutex::new(sysmon::Sampler::default()),
            workers: Mutex::new(Vec::new()),
        });
        hub.spawn_encoders(encode_receiver);
        hub
    }

    /// One worker per core beyond the first, so the capture threads and the
    /// writer still get a core to themselves on a four-core Pi.
    fn spawn_encoders(self: &Arc<Self>, receiver: Receiver<Produced>) {
        let workers = std::thread::available_parallelism()
            .map(|count| count.get().saturating_sub(1).max(1))
            .unwrap_or(1);
        let mut handles = self.workers.lock().unwrap();
        for index in 0..workers {
            let hub = Arc::clone(self);
            let receiver = receiver.clone();
            let handle = std::thread::Builder::new()
                .name(format!("encode-{index}"))
                .spawn(move || {
                    yield_to_capture_threads();
                    for produced in receiver {
                        hub.encode_and_store(produced);
                    }
                })
                .expect("failed to spawn encode worker");
            handles.push(handle);
        }
    }

    pub fn settings(&self) -> Settings {
        self.settings.read().unwrap().clone()
    }

    pub fn settings_file(&self) -> &std::path::Path {
        &self.settings_file
    }

    pub fn update_settings(&self, settings: Settings) -> Result<()> {
        self.preview_wanted
            .store(settings.preview_enabled, Ordering::Relaxed);
        *self.settings.write().unwrap() = settings;
        self.save_settings()
    }

    pub fn save_settings(&self) -> Result<()> {
        let settings = self.settings();
        if let Some(parent) = self.settings_file.parent() {
            std::fs::create_dir_all(parent).ok();
        }
        let text = serde_json::to_string_pretty(&settings)?;
        std::fs::write(&self.settings_file, text)
            .with_context(|| format!("writing {}", self.settings_file.display()))?;
        Ok(())
    }

    pub fn load_settings(path: &std::path::Path) -> Settings {
        std::fs::read_to_string(path)
            .ok()
            .and_then(|text| serde_json::from_str(&text).ok())
            .unwrap_or_default()
    }

    /// The callback handed to every backend. Runs on the capture thread, so it
    /// does the least possible work: count it, and post it.
    pub fn sink(self: &Arc<Self>) -> Sink {
        let hub = Arc::clone(self);
        Arc::new(move |produced: Produced| {
            hub.rates
                .lock()
                .unwrap()
                .entry(produced.topic().to_string())
                .or_default()
                .tick();
            // Compressing a frame costs a core, and with no recorder and no
            // preview the result is thrown away by `offer`. Shedding it here
            // is not a drop: nothing was going to keep it. Intrinsics are the
            // exception — the hub remembers those to replay into a recording
            // that has not started yet, so they are never shed.
            let announcement = matches!(produced, Produced::CameraInfo { .. });
            if !announcement
                && !hub.recording_active.load(Ordering::Relaxed)
                && !hub.preview_matches(produced.topic())
            {
                return true;
            }
            match hub.encode_sender.try_send(produced) {
                Ok(()) => true,
                Err(TrySendError::Full(produced)) => {
                    *hub.pipeline_dropped
                        .lock()
                        .unwrap()
                        .entry(produced.topic().to_string())
                        .or_default() += 1;
                    false
                }
                Err(TrySendError::Disconnected(_)) => false,
            }
        })
    }

    fn encode_and_store(&self, produced: Produced) {
        self.record_encodes.fetch_add(1, Ordering::Relaxed);
        let settings = self.settings();
        match produced {
            Produced::Image {
                stream,
                topic,
                image,
            } => {
                if self.preview_matches(&topic) {
                    self.render_preview(&topic, &image, &settings);
                }
                // A codec that cannot hold this stream's bit depth returns
                // nothing, and the frame is stored raw rather than truncated.
                match crate::image::compress(&image, settings.format_for(stream)) {
                    Some(compressed) => self.offer(
                        &format!("{topic}/compressed"),
                        crate::cdr::compressed_image(&compressed),
                    ),
                    None => self.offer(&topic, crate::cdr::raw_image(&image)),
                }
            }
            Produced::CameraInfo { topic, info } => {
                self.offer(&topic, crate::cdr::camera_info(&info));
                self.latest_intrinsics
                    .lock()
                    .unwrap()
                    .insert(topic, *info);
            }
            Produced::Imu { topic, imu } => {
                self.offer(&topic, crate::cdr::imu(&imu));
            }
            Produced::Cloud { topic, cloud } => {
                self.offer(&topic, crate::cdr::point_cloud2(&cloud));
            }
        }
    }

    fn preview_matches(&self, topic: &str) -> bool {
        if !self.preview_wanted.load(Ordering::Relaxed) {
            return false;
        }
        match &self.settings.read().unwrap().preview_topic {
            Some(wanted) => wanted == topic,
            None => false,
        }
    }

    fn render_preview(&self, topic: &str, image: &crate::msgs::RawImage, settings: &Settings) {
        let Ok(frame) = crate::image::encode(
            image,
            settings.preview_quality,
            settings.preview_max_width as usize,
        ) else {
            return;
        };
        self.preview_encodes.fetch_add(1, Ordering::Relaxed);
        *self.preview.lock().unwrap() = Some(PreviewFrame {
            topic: topic.to_string(),
            bytes: frame.jpeg,
            width: frame.width as u32,
            height: frame.height as u32,
        });
    }

    pub fn preview_encode_count(&self) -> u64 {
        self.preview_encodes.load(Ordering::Relaxed)
    }

    pub fn record_encode_count(&self) -> u64 {
        self.record_encodes.load(Ordering::Relaxed)
    }

    pub fn take_preview(&self) -> Option<PreviewFrame> {
        self.preview.lock().unwrap().take()
    }

    fn offer(&self, topic: &str, encoded: crate::cdr::Encoded) {
        if let Some(recorder) = self.recorder.lock().unwrap().as_ref() {
            recorder.offer(topic, encoded);
        }
    }

    pub fn stream_stats(&self) -> Vec<StreamStats> {
        let rates = self.rates.lock().unwrap();
        let pipeline_dropped = self.pipeline_dropped.lock().unwrap();
        let written = self
            .recorder
            .lock()
            .unwrap()
            .as_ref()
            .map(Recorder::status);
        rates
            .iter()
            .map(|(topic, counter)| {
                let writer_dropped = written
                    .as_ref()
                    .and_then(|status| status.topics.get(topic))
                    .map(|tally| tally.dropped)
                    .unwrap_or(0);
                StreamStats {
                    topic: topic.clone(),
                    hz: counter.hz(),
                    total: counter.count,
                    dropped: pipeline_dropped.get(topic).copied().unwrap_or(0) + writer_dropped,
                }
            })
            .collect()
    }

    /// Every image topic the current settings could preview, whether or not the
    /// camera is engaged yet. The browser needs this to fill the preview
    /// dropdown, and deriving it here keeps topic naming in one place.
    pub fn preview_topics(&self) -> Vec<String> {
        let settings = self.settings();
        let mut topics = Vec::new();
        for config in [&settings.realsense, &settings.orbbec] {
            for stream in config.streams() {
                if stream.is_image() {
                    topics.push(config.naming.image_topic(stream));
                }
            }
        }
        topics
    }

    pub fn health(&self) -> sysmon::Health {
        let record_dir = self.settings().record_dir;
        self.health.lock().unwrap().sample(&record_dir)
    }

    // -- sensors ----------------------------------------------------------

    /// Opens a sensor. Called on demand rather than at startup, so another
    /// process can hold the camera until the operator asks for it here.
    pub fn engage(self: &Arc<Self>, kind: SensorKind) -> Result<()> {
        self.disengage(kind);
        let settings = self.settings();
        let mut backend: Box<dyn Backend> = match kind {
            SensorKind::Realsense => Box::new(
                crate::sensors::realsense::RealsenseBackend::new(settings.realsense.clone()),
            ),
            SensorKind::Orbbec => Box::new(crate::sensors::orbbec::OrbbecBackend::new(
                settings.orbbec.clone(),
            )),
            SensorKind::Livox => Box::new(crate::sensors::livox::LivoxBackend::new(
                settings.livox.clone(),
            )),
        };
        backend.start(self.sink())?;
        self.backends.lock().unwrap().insert(kind, backend);
        Ok(())
    }

    /// Releases a sensor without stopping the process, so another program can
    /// claim the device. Recording continues on whatever else is engaged.
    pub fn disengage(&self, kind: SensorKind) {
        let removed = self.backends.lock().unwrap().remove(&kind);
        if let Some(mut backend) = removed {
            backend.stop();
        }
    }

    pub fn sensor_status(&self) -> BTreeMap<String, crate::sensors::BackendStatus> {
        let backends = self.backends.lock().unwrap();
        [SensorKind::Realsense, SensorKind::Orbbec, SensorKind::Livox]
            .into_iter()
            .map(|kind| {
                let status = backends.get(&kind).map(|backend| backend.status()).unwrap_or(
                    crate::sensors::BackendStatus {
                        running: false,
                        detail: if kind.compiled_in() {
                            "disengaged".into()
                        } else {
                            "not compiled in".into()
                        },
                        error: None,
                    },
                );
                (kind.as_str().to_string(), status)
            })
            .collect()
    }

    // -- urdf -------------------------------------------------------------

    pub fn set_urdf(&self, xml: Option<String>) -> Result<UrdfReport> {
        let report = self.inspect_urdf(xml.as_deref());
        self.settings.write().unwrap().urdf_xml = xml;
        self.save_settings()?;
        Ok(report)
    }

    pub fn inspect_urdf(&self, xml: Option<&str>) -> UrdfReport {
        let frames = self.settings().sensor_frames();
        let Some(xml) = xml else {
            return UrdfReport {
                present: false,
                robot_name: None,
                links: Vec::new(),
                joints: 0,
                problems: Vec::new(),
                parse_error: None,
            };
        };
        match urdf::parse(xml) {
            Ok(parsed) => UrdfReport {
                present: true,
                robot_name: Some(parsed.robot_name.clone()),
                links: parsed.links.clone(),
                joints: parsed.joints.len(),
                problems: parsed.problems(&frames),
                parse_error: None,
            },
            Err(error) => UrdfReport {
                present: true,
                robot_name: None,
                links: Vec::new(),
                joints: 0,
                problems: Vec::new(),
                parse_error: Some(format!("{error:#}")),
            },
        }
    }

    /// The full `/tf_static` payload: the URDF's joints, plus the edges that
    /// place each sensor's streams under that sensor's root frame.
    ///
    /// An engaged camera supplies those edges from its own factory calibration,
    /// which is the only place the millimetre offsets between its imagers exist.
    /// A sensor that is configured but not open falls back to identity edges —
    /// wrong by a couple of centimetres, but a named frame that exists beats a
    /// TF tree with a hole in it where readers expect a frame.
    pub fn static_transforms(&self, stamp_nanos: u64) -> Vec<TransformStamped> {
        let settings = self.settings();
        let mut transforms = settings
            .urdf_xml
            .as_deref()
            .and_then(|xml| urdf::parse(xml).ok())
            .map(|parsed| parsed.static_transforms(stamp_nanos))
            .unwrap_or_default();

        let backends = self.backends.lock().unwrap();
        // A sensor counts if it is configured on *or* currently engaged. `--engage`
        // opens a device without touching the settings file, so gating on the flag
        // alone left a running camera out and shipped recordings with no
        // `/tf_static` at all.
        let mut sensors: Vec<(SensorKind, &crate::sensors::Naming, Vec<crate::sensors::StreamId>)> =
            Vec::new();
        for (kind, config) in [
            (SensorKind::Realsense, &settings.realsense),
            (SensorKind::Orbbec, &settings.orbbec),
        ] {
            if config.enabled || backends.contains_key(&kind) {
                sensors.push((kind, &config.naming, config.streams()));
            }
        }
        if settings.livox.enabled || backends.contains_key(&SensorKind::Livox) {
            sensors.push((
                SensorKind::Livox,
                &settings.livox.naming,
                settings.livox.streams(),
            ));
        }

        for (kind, naming, streams) in sensors {
            let measured = backends
                .get(&kind)
                .map(|backend| backend.body_transforms())
                .unwrap_or_default();
            if !measured.is_empty() {
                transforms.extend(measured);
                continue;
            }
            for stream in streams {
                let mut edge =
                    TransformStamped::identity(&naming.root_frame_id(), &naming.frame_id(stream));
                edge.header = crate::msgs::Header::new(stamp_nanos, naming.root_frame_id());
                transforms.push(edge);
            }
        }
        transforms
    }

    // -- recording --------------------------------------------------------

    pub fn start_recording(&self, name: Option<&str>) -> Result<RecordingStatus> {
        let settings = self.settings();
        // Gathered before the recorder lock, not after. This reads the backend
        // map, and disengaging holds that map while it joins a sensor thread
        // that is itself trying to take the recorder lock — taking the two in
        // this order everywhere is what stops those meeting head-on.
        let transforms = self.static_transforms(record::now_nanos());
        let intrinsics: Vec<(String, CameraInfo)> = self
            .latest_intrinsics
            .lock()
            .unwrap()
            .iter()
            .map(|(topic, info)| (topic.clone(), info.clone()))
            .collect();

        let mut slot = self.recorder.lock().unwrap();
        if slot.is_some() {
            anyhow::bail!("already recording");
        }
        let file_name = match name {
            Some(name) => record::resolve(&settings.record_dir, name)?,
            None => settings.record_dir.join(record::default_name()),
        };
        let recorder = Recorder::start(&file_name, settings.compression)?;

        // tf_static goes in first so a reader that stops early still has the
        // frames it needs to place everything else.
        if !transforms.is_empty() {
            recorder.offer("/tf_static", crate::cdr::tf_message(&transforms));
        }
        for (topic, info) in &intrinsics {
            recorder.offer(topic, crate::cdr::camera_info(info));
        }
        let status = recorder.status();
        *slot = Some(recorder);
        self.recording_active.store(true, Ordering::Relaxed);
        Ok(status)
    }

    pub fn stop_recording(&self) -> Result<RecordingStatus> {
        let Some(recorder) = self.recorder.lock().unwrap().take() else {
            anyhow::bail!("not recording");
        };
        self.recording_active.store(false, Ordering::Relaxed);
        let status = recorder.finish()?;
        *self.last_status.lock().unwrap() = status.clone();
        Ok(status)
    }

    pub fn recording_status(&self) -> RecordingStatus {
        match self.recorder.lock().unwrap().as_ref() {
            Some(recorder) => recorder.status(),
            None => self.last_status.lock().unwrap().clone(),
        }
    }

    pub fn shutdown(&self) {
        for kind in [SensorKind::Realsense, SensorKind::Orbbec, SensorKind::Livox] {
            self.disengage(kind);
        }
        let _ = self.stop_recording();
    }
}

#[derive(Serialize)]
pub struct UrdfReport {
    pub present: bool,
    pub robot_name: Option<String>,
    pub links: Vec<String>,
    pub joints: usize,
    pub problems: Vec<TreeProblem>,
    pub parse_error: Option<String>,
}

impl UrdfReport {
    /// The single line the settings panel shows. Distinguishing "none uploaded"
    /// from "uploaded but broken" is the whole point.
    pub fn warning(&self) -> Option<String> {
        if !self.present {
            return Some(
                "no urdf uploaded: the recording will have sensor frames with nothing joining them"
                    .into(),
            );
        }
        if let Some(error) = &self.parse_error {
            return Some(format!("urdf could not be parsed: {error}"));
        }
        if self.problems.is_empty() {
            return None;
        }
        Some(format!(
            "urdf tree is broken: {}",
            self.problems
                .iter()
                .map(TreeProblem::message)
                .collect::<Vec<_>>()
                .join("; ")
        ))
    }
}

/// Used by the settings UI to offer sensible frame prefixes.
pub fn default_naming(kind: SensorKind) -> Naming {
    Naming::for_kind(kind)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::RawImage;

    fn scratch_hub() -> Arc<Hub> {
        let file = std::env::temp_dir().join(format!(
            "lite_record_settings_{}.json",
            record::now_nanos()
        ));
        Hub::new(Settings::default(), file)
    }

    fn an_image(width: usize, height: usize) -> RawImage {
        RawImage {
            header: crate::msgs::Header::new(1, "cam"),
            height,
            width,
            encoding: "rgb8".into(),
            is_bigendian: 0,
            step: width * 3,
            data: (0..(width * height * 3)).map(|index| index as u8).collect(),
        }
    }

    #[test]
    fn settings_round_trip_through_the_file_they_are_saved_to() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.compression = Compression::Zstd;
        settings.livox.frame_hz = 20.0;
        settings.realsense.naming.frame_prefix = "front".into();
        hub.update_settings(settings.clone()).unwrap();
        assert_eq!(Hub::load_settings(hub.settings_file()), settings);
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_missing_or_corrupt_settings_file_falls_back_to_defaults() {
        assert_eq!(
            Hub::load_settings(std::path::Path::new("/no/such/file.json")),
            Settings::default()
        );
        let path = std::env::temp_dir().join(format!("bad_{}.json", record::now_nanos()));
        std::fs::write(&path, "{not json").unwrap();
        assert_eq!(Hub::load_settings(&path), Settings::default());
        std::fs::remove_file(&path).ok();
    }

    /// Waits for the asynchronous encode pool to catch up.
    fn wait_for(condition: impl Fn() -> bool) -> bool {
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            if condition() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        condition()
    }

    #[test]
    fn turning_the_preview_off_stops_the_encode_rather_than_hiding_it() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.preview_enabled = true;
        settings.preview_topic = Some("/cam/color/image_raw".into());
        hub.update_settings(settings.clone()).unwrap();

        let sink = hub.sink();
        for _ in 0..3 {
            sink(Produced::Image {
                stream: StreamId::Color,
                topic: "/cam/color/image_raw".into(),
                image: an_image(32, 16),
            });
        }
        assert!(wait_for(|| hub.preview_encode_count() == 3));
        let with_preview = hub.preview_encode_count();
        assert!(hub.take_preview().is_some());

        settings.preview_enabled = false;
        hub.update_settings(settings).unwrap();
        for _ in 0..10 {
            sink(Produced::Image {
                stream: StreamId::Color,
                topic: "/cam/color/image_raw".into(),
                image: an_image(32, 16),
            });
        }
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(
            hub.preview_encode_count(),
            with_preview,
            "preview encoding kept running after being switched off"
        );
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_stream_that_is_not_the_previewed_one_is_never_encoded_for_preview() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.preview_topic = Some("/cam/color/image_raw".into());
        hub.update_settings(settings).unwrap();
        let sink = hub.sink();
        for _ in 0..5 {
            sink(Produced::Image {
                stream: StreamId::Depth,
                topic: "/cam/depth/image_raw".into(),
                image: an_image(32, 16),
            });
        }
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(hub.preview_encode_count(), 0);
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn frames_are_not_compressed_while_nothing_is_recording_or_previewing() {
        let hub = scratch_hub();
        let sink = hub.sink();
        for _ in 0..20 {
            sink(Produced::Image {
                stream: StreamId::Depth,
                topic: "/cam/depth/image_raw".into(),
                image: an_image(64, 48),
            });
        }
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(
            hub.record_encode_count(),
            0,
            "an idle recorder still paid to compress frames it then discarded"
        );

        let directory = hub.settings().record_dir;
        std::fs::create_dir_all(&directory).ok();
        hub.start_recording(Some("shed")).unwrap();
        for _ in 0..20 {
            sink(Produced::Image {
                stream: StreamId::Depth,
                topic: "/cam/depth/image_raw".into(),
                image: an_image(64, 48),
            });
        }
        assert!(wait_for(|| hub.record_encode_count() == 20));
        hub.stop_recording().unwrap();
        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn per_stream_rates_are_counted_whether_or_not_a_recording_is_running() {
        let hub = scratch_hub();
        let sink = hub.sink();
        for index in 0..40 {
            sink(Produced::Imu {
                topic: "/livox/imu".into(),
                imu: Box::new(crate::msgs::Imu::unoriented(
                    crate::msgs::Header::new(index * 5_000_000, "livox"),
                    [0.0; 3],
                    [0.0, 0.0, 9.81],
                )),
            });
        }
        let stats = hub.stream_stats();
        let imu = stats.iter().find(|s| s.topic == "/livox/imu").unwrap();
        assert_eq!(imu.total, 40);
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_recording_opens_with_tf_static_before_any_sensor_data() {
        let hub = scratch_hub();
        let directory =
            std::env::temp_dir().join(format!("lite_record_tf_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        settings.realsense.enabled = true;
        settings.urdf_xml = Some(
            r#"<robot name="rig">
                <link name="base_link"/><link name="realsense_link"/>
                <joint name="j" type="fixed">
                    <parent link="base_link"/><child link="realsense_link"/>
                    <origin xyz="0 0 0.1"/>
                </joint>
            </robot>"#
                .into(),
        );
        hub.update_settings(settings).unwrap();

        hub.start_recording(Some("tf.mcap")).unwrap();
        let status = hub.stop_recording().unwrap();
        assert!(status.messages >= 1);

        let bytes = std::fs::read(directory.join("tf.mcap")).unwrap();
        let first = mcap::MessageStream::new(&bytes)
            .unwrap()
            .next()
            .unwrap()
            .unwrap();
        assert_eq!(first.channel.topic, "/tf_static");
        assert_eq!(
            first.channel.schema.as_ref().unwrap().name,
            "tf2_msgs/msg/TFMessage"
        );
        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// A camera announces its intrinsics once, when it opens. Recording usually
    /// starts long after that, and images no reader can project are close to
    /// worthless, so the last CameraInfo per topic has to be replayed into every
    /// new file. Caught on a real D455: a 20 s recording came back with five
    /// image streams and no intrinsics at all.
    #[test]
    fn intrinsics_announced_before_recording_still_land_in_the_file() {
        let hub = scratch_hub();
        let directory =
            std::env::temp_dir().join(format!("lite_record_info_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        hub.update_settings(settings).unwrap();

        hub.sink()(Produced::CameraInfo {
            topic: "/realsense/color/camera_info".into(),
            info: Box::new(CameraInfo::pinhole(
                crate::msgs::Header::new(1, "realsense_color_optical_frame"),
                640,
                480,
                600.0,
                600.0,
                320.0,
                240.0,
                crate::msgs::DistortionModel::PlumbBob,
                vec![0.0; 5],
                0.0,
            )),
        });
        // The encoders are a thread pool, so the announcement lands a moment
        // after the sink returns.
        let deadline = Instant::now() + Duration::from_secs(5);
        while hub.latest_intrinsics.lock().unwrap().is_empty() {
            assert!(Instant::now() < deadline, "the announcement never arrived");
            std::thread::sleep(Duration::from_millis(10));
        }

        hub.start_recording(Some("info.mcap")).unwrap();
        hub.stop_recording().unwrap();

        let bytes = std::fs::read(directory.join("info.mcap")).unwrap();
        let topics: Vec<String> = mcap::MessageStream::new(&bytes)
            .unwrap()
            .map(|message| message.unwrap().channel.topic.clone())
            .collect();
        assert!(
            topics.iter().any(|t| t == "/realsense/color/camera_info"),
            "got {topics:?}"
        );
        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn the_tf_static_payload_joins_every_optical_frame_to_the_urdf() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.realsense.enabled = true;
        settings.urdf_xml = Some(
            r#"<robot name="rig"><link name="base_link"/><link name="realsense_link"/>
                <joint name="j" type="fixed"><parent link="base_link"/><child link="realsense_link"/></joint>
            </robot>"#
                .into(),
        );
        hub.update_settings(settings).unwrap();

        let transforms = hub.static_transforms(1);
        let children: Vec<&str> = transforms
            .iter()
            .map(|t| t.child_frame_id.as_str())
            .collect();
        assert!(children.contains(&"realsense_link"));
        assert!(children.contains(&"realsense_depth_optical_frame"));
        assert!(children.contains(&"realsense_color_optical_frame"));
        assert!(children.contains(&"realsense_infra1_optical_frame"));
        assert!(children.contains(&"realsense_imu_frame"));
        // Every optical frame must trace back to the urdf root, or the file
        // contains data no consumer can place.
        let parents: std::collections::BTreeSet<&str> =
            transforms.iter().map(|t| t.header.frame_id.as_str()).collect();
        assert!(parents.contains("base_link"));
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// `--engage` opens a device without writing the settings file, so a sensor
    /// can be streaming while its `enabled` flag is still false. Gating on that
    /// flag shipped recordings from a live camera with no `/tf_static` at all.
    #[test]
    fn an_engaged_sensor_gets_transforms_even_with_its_setting_off() {
        let hub = scratch_hub();
        assert!(!hub.settings().livox.enabled);

        assert!(hub.static_transforms(1).is_empty());
        hub.engage(SensorKind::Livox).unwrap();
        let children: Vec<String> = hub
            .static_transforms(1)
            .into_iter()
            .map(|transform| transform.child_frame_id)
            .collect();
        hub.disengage(SensorKind::Livox);

        assert!(
            children.iter().any(|child| child.contains("livox")),
            "an engaged lidar contributed no frames: {children:?}"
        );
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn no_urdf_and_a_broken_urdf_produce_different_warnings() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.realsense.enabled = true;
        hub.update_settings(settings).unwrap();

        let none = hub.inspect_urdf(None);
        assert!(none.warning().unwrap().contains("no urdf uploaded"));

        let broken = hub.inspect_urdf(Some(
            r#"<robot name="x"><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
                <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
                <joint name="cd" type="fixed"><parent link="c"/><child link="d"/></joint>
            </robot>"#,
        ));
        let warning = broken.warning().unwrap();
        assert!(warning.contains("tree is broken"), "{warning}");
        assert!(warning.contains("disconnected roots"), "{warning}");

        let unparseable = hub.inspect_urdf(Some("<<<not xml"));
        assert!(unparseable
            .warning()
            .unwrap()
            .contains("could not be parsed"));
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_urdf_missing_the_sensor_frame_is_reported_even_though_it_parses() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.livox.enabled = true;
        hub.update_settings(settings).unwrap();
        let report = hub.inspect_urdf(Some(r#"<robot name="x"><link name="base_link"/></robot>"#));
        assert!(report.parse_error.is_none());
        assert!(report.problems.contains(&TreeProblem::UncoveredFrame {
            frame: "livox_frame".into()
        }));
        assert!(report.warning().unwrap().contains("livox_frame"));
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_second_start_is_refused_rather_than_silently_replacing_the_file() {
        let hub = scratch_hub();
        let directory =
            std::env::temp_dir().join(format!("lite_record_twice_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        hub.update_settings(settings).unwrap();

        hub.start_recording(Some("first.mcap")).unwrap();
        assert!(hub.start_recording(Some("second.mcap")).is_err());
        hub.stop_recording().unwrap();
        assert!(hub.stop_recording().is_err());
        assert!(directory.join("first.mcap").exists());
        assert!(!directory.join("second.mcap").exists());
        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_backend_with_no_sdk_reports_its_absence_rather_than_appearing_idle() {
        let hub = scratch_hub();
        let status = hub.sensor_status();
        assert_eq!(status.len(), 3);
        assert!(!status["livox"].running);
        if !cfg!(feature = "realsense") {
            assert_eq!(status["realsense"].detail, "not compiled in");
            assert!(hub.engage(SensorKind::Realsense).is_err());
        }
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// Disengage has to survive being called on a sensor that was never
    /// engaged, because the UI button does not know the current state.
    #[test]
    fn disengaging_an_idle_sensor_is_a_no_op() {
        let hub = scratch_hub();
        hub.disengage(SensorKind::Livox);
        hub.disengage(SensorKind::Livox);
        assert!(!hub.sensor_status()["livox"].running);
        std::fs::remove_file(hub.settings_file()).ok();
    }
}
