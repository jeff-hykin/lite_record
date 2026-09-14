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
/// One queue per worker, so this is the depth of each.
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

/// Defaulted as a whole: a settings file written before a field existed is
/// still worth loading, and dropping every other setting because one key is
/// missing is far worse than filling that one key in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub record_dir: PathBuf,
    pub compression: Compression,
    /// How image streams are stored. Depth and infrared fall back to raw when
    /// the chosen codec cannot hold their bit depth.
    pub color_format: ImageFormat,
    pub depth_format: ImageFormat,
    pub realsense: CameraConfig,
    pub orbbec: CameraConfig,
    pub oakd: CameraConfig,
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
            depth_format: ImageFormat::Jpegxl,
            realsense: CameraConfig::for_kind(SensorKind::Realsense),
            orbbec: CameraConfig::for_kind(SensorKind::Orbbec),
            oakd: CameraConfig::for_kind(SensorKind::OakD),
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
    /// The root frame of every engaged sensor. These are what a URDF has to
    /// cover for the tree to be complete; the stream frames below each root are
    /// supplied by `static_transforms`, so asking a URDF for them would reject
    /// correct files and invite duplicate edges.
    pub fn sensor_frames(&self) -> Vec<String> {
        let mut frames = Vec::new();
        for config in [&self.realsense, &self.orbbec, &self.oakd] {
            if config.enabled {
                frames.push(config.naming.root_frame_id());
            }
        }
        if self.livox.enabled {
            frames.push(self.livox.naming.root_frame_id());
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

/// Every image topic the settings could preview, whether or not the camera is
/// engaged and whether or not the stream is switched on.
///
/// A switched-off stream stays in the list because the page's stream switch sits
/// beside this dropdown: dropping it would mean the last stream a camera has
/// could be switched off and never switched back on. Engaged cameras come first
/// and switched-on streams before switched-off ones, so both the automatic
/// choice and the operator's eye land on a stream that can produce a frame.
fn image_topics(settings: &Settings) -> Vec<String> {
    let cameras = [&settings.realsense, &settings.orbbec, &settings.oakd];
    let mut ranked = Vec::new();
    for config in cameras {
        let switched_on = config.streams();
        for stream in [
            StreamId::Depth,
            StreamId::Color,
            StreamId::InfraLeft,
            StreamId::InfraRight,
        ] {
            let rank = (!config.enabled, !switched_on.contains(&stream));
            ranked.push((rank, config.naming.image_topic(stream)));
        }
    }
    // Stable, so cameras and streams keep their declared order within a rank.
    ranked.sort_by_key(|entry| entry.0);
    ranked.into_iter().map(|(_, topic)| topic).collect()
}

/// The settings field that switches each topic off, as a dotted path the
/// browser can write straight back into the Settings struct it already holds.
///
/// Built from the configs rather than parsed out of the topic string, because
/// the prefixes are operator-editable: a rig that renames `/realsense` to
/// `/front_cam` still has to be able to turn its colour stream off.
fn topic_settings(settings: &Settings) -> BTreeMap<String, String> {
    let mut paths = BTreeMap::new();
    let cameras = [
        ("realsense", &settings.realsense),
        ("orbbec", &settings.orbbec),
        ("oakd", &settings.oakd),
    ];
    for (kind, config) in cameras {
        // Both infrared imagers are one switch, as they are on the device.
        for (stream, field) in [
            (StreamId::Depth, "depth"),
            (StreamId::Color, "color"),
            (StreamId::InfraLeft, "infrared"),
            (StreamId::InfraRight, "infrared"),
        ] {
            let path = format!("{kind}.{field}");
            paths.insert(config.naming.image_topic(stream), path.clone());
            // The intrinsics go silent with the imager they describe, so the one
            // switch has to claim both topics or camera_info looks untoggleable.
            paths.insert(config.naming.camera_info_topic(stream), path);
        }
        paths.insert(
            config.naming.topic("aligned_depth_image"),
            format!("{kind}.align_depth_to_color"),
        );
        paths.insert(config.naming.imu_topic(), format!("{kind}.imu"));
    }
    paths.insert(settings.livox.naming.imu_topic(), "livox.imu".to_owned());
    // The cloud is the lidar's reason for being open, so its only switch is the
    // lidar's own.
    paths.insert(
        settings.livox.naming.points_topic(),
        "livox.enabled".to_owned(),
    );
    paths
}

/// Which engaged sensors have to be cycled for a settings change to reach the
/// device. A backend that can absorb the change in place is asked to do so here,
/// because cycling a pipeline costs about a second of frames.
fn sensors_needing_restart(
    backends: &mut BTreeMap<SensorKind, Box<dyn Backend>>,
    previous: &Settings,
    settings: &Settings,
) -> Vec<SensorKind> {
    let cameras = [
        (SensorKind::Realsense, &previous.realsense, &settings.realsense),
        (SensorKind::Orbbec, &previous.orbbec, &settings.orbbec),
        (SensorKind::OakD, &previous.oakd, &settings.oakd),
    ];
    let mut restart = Vec::new();
    for (kind, was, now) in cameras {
        if was == now {
            continue;
        }
        if let Some(backend) = backends.get_mut(&kind) {
            if !backend.apply_live(now) {
                restart.push(kind);
            }
        }
    }
    // The lidar has no live knobs at all: its config is read once when the
    // sockets are opened and the work-mode handshake is sent.
    if previous.livox != settings.livox && backends.contains_key(&SensorKind::Livox) {
        restart.push(SensorKind::Livox);
    }
    restart
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

    /// Once carried a measured rate and has since decayed to nothing. A latched
    /// topic like `camera_info` publishes a single message and then legitimately
    /// stays quiet, so it never earns a rate and is not counted as stalled.
    fn stalled(&self) -> bool {
        self.hz > 0.0 && self.hz() == 0.0
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
    /// `pipeline_dropped` as it stood when the current recording started, so the
    /// shed count can be narrowed to that one file.
    shed_baseline: Mutex<BTreeMap<String, u64>>,
    /// Set by `start_recording` when the tf tree it wrote was broken.
    tree_warning: Mutex<Option<String>>,
    /// The latest intrinsics seen on each camera_info topic.
    ///
    /// A backend announces these once when it opens, not on every frame, so a
    /// recording started afterwards would otherwise contain images that no
    /// reader can project. They are replayed into each new file the same way
    /// `/tf_static` is.
    latest_intrinsics: Mutex<BTreeMap<String, CameraInfo>>,
    /// One queue per encode worker rather than one queue shared by all of them.
    /// A topic always goes to the same worker, so two messages from one stream
    /// cannot be encoded concurrently and reach the recorder in the wrong order.
    /// Sharing a queue let a 200 Hz IMU overtake itself roughly once every nine
    /// thousand samples, which put a backwards header stamp in the file.
    encode_senders: Vec<Sender<Produced>>,
    /// Latest preview frame, as jpeg bytes ready to push down the websocket.
    preview: Mutex<Option<PreviewFrame>>,
    /// Counts preview encodes so switching the preview off can be shown to
    /// actually stop the work rather than just hide the result.
    preview_encodes: AtomicU64,
    /// Counts messages the hub encoded and stored, wherever that ran, for the
    /// same reason `preview_encodes` exists: idling has to be demonstrably
    /// free, not just look free from the outside.
    record_encodes: AtomicU64,
    preview_wanted: AtomicBool,
    monitor: Mutex<sysmon::Monitor>,
    workers: Mutex<Vec<std::thread::JoinHandle<()>>>,
}

/// One worker per core beyond the first, so the capture threads and the writer
/// still get a core to themselves on a four-core Pi.
fn encode_worker_count() -> usize {
    std::thread::available_parallelism()
        .map(|count| count.get().saturating_sub(1).max(1))
        .unwrap_or(1)
}

/// Which encode worker owns a topic. Any stable mapping will do; what matters is
/// that a topic always lands on the same one.
fn worker_for(topic: &str, workers: usize) -> usize {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in topic.as_bytes() {
        hash = (hash ^ *byte as u64).wrapping_mul(0x0000_0100_0000_01b3);
    }
    (hash % workers as u64) as usize
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
        let (encode_senders, encode_receivers): (Vec<_>, Vec<_>) =
            (0..encode_worker_count()).map(|_| bounded(ENCODE_QUEUE_DEPTH)).unzip();
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
            shed_baseline: Mutex::new(BTreeMap::new()),
            tree_warning: Mutex::new(None),
            latest_intrinsics: Mutex::new(BTreeMap::new()),
            encode_senders,
            preview: Mutex::new(None),
            preview_encodes: AtomicU64::new(0),
            record_encodes: AtomicU64::new(0),
            monitor: Mutex::new(sysmon::Monitor::default()),
            workers: Mutex::new(Vec::new()),
        });
        hub.spawn_encoders(encode_receivers);
        hub.spawn_monitor();
        hub
    }

    /// Samples host health on its own cadence rather than whenever a browser
    /// asks. Two browsers polling used to each halve the other's /proc/stat
    /// measurement window, and a history that only advances while someone is
    /// watching would leave a freshly opened page with an empty chart.
    fn spawn_monitor(self: &Arc<Self>) {
        let hub = Arc::clone(self);
        std::thread::Builder::new()
            .name("monitor".into())
            .spawn(move || loop {
                std::thread::sleep(sysmon::SAMPLE_INTERVAL);
                let record_dir = hub.settings().record_dir;
                hub.monitor.lock().unwrap().tick(&record_dir, sysmon::SAMPLE_INTERVAL);
            })
            .expect("failed to spawn monitor thread");
    }

    fn spawn_encoders(self: &Arc<Self>, receivers: Vec<Receiver<Produced>>) {
        let mut handles = self.workers.lock().unwrap();
        for (index, receiver) in receivers.into_iter().enumerate() {
            let hub = Arc::clone(self);
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

    /// Saves the new settings and makes an engaged sensor obey them. Without
    /// this last part a knob like the IR emitter moves in the browser and
    /// nothing happens on the device, because a backend keeps the config it was
    /// started with.
    pub fn update_settings(self: &Arc<Self>, settings: Settings) -> Result<()> {
        let previous = self.settings();
        self.preview_wanted
            .store(settings.preview_enabled, Ordering::Relaxed);
        *self.settings.write().unwrap() = settings.clone();
        self.save_settings()?;

        let restart = sensors_needing_restart(
            &mut self.backends.lock().unwrap(),
            &previous,
            &settings,
        );
        for kind in restart {
            self.engage(kind)
                .with_context(|| format!("reopening the {} after a settings change", kind.as_str()))?;
        }
        Ok(())
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
        let Ok(text) = std::fs::read_to_string(path) else {
            return Settings::default();
        };
        match serde_json::from_str(&text) {
            Ok(settings) => settings,
            Err(error) => {
                eprintln!("ignoring {}: {error}", path.display());
                Settings::default()
            }
        }
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
            // An imu sample and an intrinsics announcement hold no image to
            // compress and serialise in microseconds, so they are written from
            // here rather than queued. Sharing a worker's queue with them cost
            // the colour stream a third of its frames at 720p30: a 200 Hz imu
            // fills 128 slots faster than one 720p jpeg encode returns, and the
            // frame that then finds the queue full is the one that is shed.
            if matches!(produced, Produced::Imu { .. } | Produced::CameraInfo { .. }) {
                hub.encode_and_store(produced);
                return true;
            }
            match hub.encode_senders[worker_for(produced.topic(), hub.encode_senders.len())]
                .try_send(produced)
            {
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
                // Both land on the one topic, as they do in dimos; the message
                // type is what says which arrived.
                match crate::image::compress(&image, settings.format_for(stream)) {
                    Some(compressed) => {
                        self.offer(&topic, crate::cdr::compressed_image(&compressed))
                    }
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
        self.preview_topic().is_some_and(|wanted| wanted == topic)
    }

    /// The topic the preview actually shows. Nothing chosen means the first
    /// image stream, so a box that has never been configured shows a picture
    /// instead of sitting on "no frames yet" with no hint a choice is needed.
    pub fn preview_topic(&self) -> Option<String> {
        let settings = self.settings.read().unwrap();
        let mut topics = image_topics(&settings);
        // A chosen topic no sensor publishes is ignored rather than honoured:
        // a settings file written before a topic was renamed would otherwise
        // hold the preview permanently blank with nothing on screen saying why.
        match &settings.preview_topic {
            Some(chosen) if topics.iter().any(|topic| topic == chosen) => Some(chosen.clone()),
            _ => topics.drain(..).next(),
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
        image_topics(&self.settings())
    }

    /// Topic to the settings path that switches it off. See [`topic_settings`].
    pub fn topic_settings(&self) -> BTreeMap<String, String> {
        topic_settings(&self.settings())
    }

    pub fn health(&self) -> sysmon::Health {
        self.monitor.lock().unwrap().health()
    }

    pub fn health_history(&self) -> sysmon::History {
        self.monitor.lock().unwrap().history()
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
            SensorKind::OakD => Box::new(crate::sensors::oakd::OakdBackend::new(
                settings.oakd.clone(),
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
        // Read the settings before taking the backends lock: `update_settings`
        // takes them the other way round, and meeting in the middle deadlocks.
        let settings = self.settings();
        let backends = self.backends.lock().unwrap();
        [
            SensorKind::Realsense,
            SensorKind::Orbbec,
            SensorKind::OakD,
            SensorKind::Livox,
        ]
            .into_iter()
            .map(|kind| {
                let mut status =
                    backends.get(&kind).map(|backend| backend.status()).unwrap_or(
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
                if status.running && status.error.is_none() {
                    let silent = self.silent_topics(naming_for(&settings, kind));
                    if !silent.is_empty() {
                        status.error = Some(format!("stopped producing: {}", silent.join(", ")))
                    }
                }
                (kind.as_str().to_string(), status)
            })
            .collect()
    }

    /// Topics under `prefix` that produced messages once and have since gone
    /// quiet. A camera that loses power re-enumerates and can come back with
    /// only some of its streams — the driver still reports itself as healthy, so
    /// the missing stream is invisible unless the rates are consulted.
    fn silent_topics(&self, prefix: &str) -> Vec<String> {
        let prefix = prefix.trim_end_matches('/');
        let mut silent: Vec<String> = self
            .rates
            .lock()
            .unwrap()
            .iter()
            .filter(|(topic, counter)| topic.starts_with(prefix) && counter.stalled())
            .map(|(topic, _)| topic.clone())
            .collect();
        silent.sort();
        silent
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
                tree_problems: self.tree_problems(None),
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
                tree_problems: self.tree_problems(Some(xml)),
                parse_error: None,
            },
            Err(error) => UrdfReport {
                present: true,
                robot_name: None,
                links: Vec::new(),
                joints: 0,
                problems: Vec::new(),
                tree_problems: Vec::new(),
                parse_error: Some(format!("{error:#}")),
            },
        }
    }

    /// The warning the last `start_recording` raised about the tf tree, if any.
    pub fn tree_warning(&self) -> Option<String> {
        self.tree_warning.lock().unwrap().clone()
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
        let xml = self.settings().urdf_xml.clone();
        self.static_transforms_with(xml.as_deref(), stamp_nanos)
    }

    /// The same payload for a URDF that is not (yet) the saved one, so an upload
    /// can be judged against the sensors' own edges before it is accepted.
    pub fn static_transforms_with(&self, urdf_xml: Option<&str>, stamp_nanos: u64) -> Vec<TransformStamped> {
        let settings = self.settings();
        let mut transforms = urdf_xml
            .and_then(|xml| urdf::parse(xml).ok())
            .map(|parsed| parsed.static_transforms(stamp_nanos))
            .unwrap_or_default();
        let placed_by_urdf: std::collections::BTreeSet<String> = transforms
            .iter()
            .map(|transform| transform.child_frame_id.clone())
            .collect();

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
            (SensorKind::OakD, &settings.oakd),
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
                // Only where nothing better is known. The identity is a
                // placeholder for a sensor that reports no extrinsics of its
                // own, and a URDF that places the frame is the operator saying
                // where it actually is — publishing both would put the same
                // child under two parents in one message. A Mid-360's point
                // origin sits 47 mm above the seat it bolts to, so the
                // placeholder is wrong by that much until a URDF says so.
                let child = naming.frame_id(stream);
                if placed_by_urdf.contains(&child) {
                    continue;
                }
                let mut edge = TransformStamped::identity(&naming.root_frame_id(), &child);
                edge.header = crate::msgs::Header::new(stamp_nanos, naming.root_frame_id());
                transforms.push(edge);
            }
        }
        transforms
    }

    /// What is wrong with the tree the recorder would actually write: the URDF's
    /// joints *and* the sensors' own edges together. `Urdf::problems` judges the
    /// file alone; this catches the URDF hanging a frame a sensor also places, a
    /// sensor frame the URDF never reaches, and a URDF-less rig whose sensors
    /// form separate trees.
    pub fn tree_problems(&self, urdf_xml: Option<&str>) -> Vec<TreeProblem> {
        let transforms = self.static_transforms_with(urdf_xml, 0);
        let mut parents_of: BTreeMap<String, Vec<String>> = BTreeMap::new();
        let mut tree = crate::tf::StaticTree::default();
        for transform in &transforms {
            let parents = parents_of.entry(transform.child_frame_id.clone()).or_default();
            if !parents.contains(&transform.header.frame_id) {
                parents.push(transform.header.frame_id.clone());
            }
            tree.insert_transform(transform);
        }
        let mut problems: Vec<TreeProblem> = parents_of
            .iter()
            .filter(|(_, parents)| parents.len() > 1)
            .map(|(child, parents)| TreeProblem::DoubleParent {
                child: child.clone(),
                parents: parents.clone(),
            })
            .collect();
        if transforms.is_empty() {
            return problems;
        }
        let roots = tree.roots();
        if roots.len() > 1 {
            problems.push(TreeProblem::MultipleRoots { roots });
        }
        let settings = self.settings();
        let mut data_frames: Vec<String> = Vec::new();
        for config in [&settings.realsense, &settings.orbbec, &settings.oakd] {
            if config.enabled {
                data_frames.extend(config.streams().into_iter().map(|stream| config.naming.frame_id(stream)));
            }
        }
        if settings.livox.enabled {
            data_frames.extend(settings.livox.streams().into_iter().map(|stream| settings.livox.naming.frame_id(stream)));
        }
        for frame in data_frames {
            if !tree.contains(&frame) {
                problems.push(TreeProblem::UncoveredFrame { frame });
            }
        }
        problems
    }

    // -- recording --------------------------------------------------------

    pub fn start_recording(&self, name: Option<&str>) -> Result<RecordingStatus> {
        let settings = self.settings();
        // Flagged, not refused: a rig with no URDF still records, but nobody
        // should find out the tree was broken from a reader weeks later.
        let broken = self.tree_problems(settings.urdf_xml.as_deref());
        if !broken.is_empty() {
            let text = broken.iter().map(TreeProblem::message).collect::<Vec<_>>().join("; ");
            eprintln!("warning: recording with a broken tf tree: {text}");
            *self.tree_warning.lock().unwrap() = Some(format!("tf tree is broken: {text}"));
        } else {
            *self.tree_warning.lock().unwrap() = None;
        }
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
        let mut recorder = Recorder::start(&file_name, settings.compression)?;

        // The transforms go in first so a reader that stops early still has the
        // frames it needs to place everything else, and keep going at 5 Hz so a
        // consumer joining mid-file (or dimos, which has no latched tf) sees them.
        recorder.repeat_transforms(transforms);
        // Re-stamped, not replayed as announced. A camera announces its
        // intrinsics when it opens, which on a Pi is at boot, before NTP has
        // corrected a clock that has no battery behind it. Keeping that stamp
        // put the calibration 2055 s before the images it belongs to in the
        // grocery recording. The content is what matters here; the stamp only
        // says when this file learned it.
        let announced_at = record::now_nanos();
        for (topic, info) in &intrinsics {
            let mut info = info.clone();
            info.header = crate::msgs::Header::new(announced_at, info.header.frame_id);
            recorder.offer(topic, crate::cdr::camera_info(&info));
        }
        *self.shed_baseline.lock().unwrap() = self.pipeline_dropped.lock().unwrap().clone();
        let status = recorder.status();
        *slot = Some(recorder);
        self.recording_active.store(true, Ordering::Relaxed);
        Ok(status)
    }

    /// Frames shed before the recorder ever saw them, per topic, since this
    /// recording started. `pipeline_dropped` counts for the life of the
    /// process, which is the wrong window for judging one file.
    fn shed_this_recording(&self) -> BTreeMap<String, u64> {
        let baseline = self.shed_baseline.lock().unwrap();
        self.pipeline_dropped
            .lock()
            .unwrap()
            .iter()
            .filter_map(|(topic, total)| {
                let shed = total - baseline.get(topic).copied().unwrap_or(0);
                (shed > 0).then(|| (topic.clone(), shed))
            })
            .collect()
    }

    pub fn stop_recording(&self) -> Result<RecordingStatus> {
        // Cleared before the lock is asked for, not after. `sink` sheds frames
        // on this flag, so lowering it first drains the capture and encode
        // threads that are otherwise taking the recorder lock hundreds of times
        // a second. Doing it in the other order let those threads starve this
        // one on a Pi saturated by encoding -- std's mutex is not fair, so
        // `stop` could wait indefinitely, and every `/api/status` piled up
        // behind it until the whole HTTP runtime stopped accepting.
        self.recording_active.store(false, Ordering::Relaxed);
        let Some(recorder) = self.recorder.lock().unwrap().take() else {
            anyhow::bail!("not recording");
        };
        let status = self.with_shed_frames(recorder.finish()?);
        *self.last_status.lock().unwrap() = status.clone();
        Ok(status)
    }

    /// A frame shed on the way to an encode worker never reaches the recorder,
    /// so the recorder's own tally called it zero while a third of the colour
    /// stream was going missing. The two counts are merged here, so one number
    /// answers "did this recording lose anything".
    fn with_shed_frames(&self, mut status: RecordingStatus) -> RecordingStatus {
        for (topic, shed) in self.shed_this_recording() {
            status.topics.entry(topic).or_default().dropped += shed;
            status.dropped += shed;
        }
        status
    }

    pub fn recording_status(&self) -> RecordingStatus {
        let status = match self.recorder.lock().unwrap().as_ref() {
            Some(recorder) => recorder.status(),
            None => return self.last_status.lock().unwrap().clone(),
        };
        self.with_shed_frames(status)
    }

    pub fn shutdown(&self) {
        for kind in [
            SensorKind::Realsense,
            SensorKind::Orbbec,
            SensorKind::OakD,
            SensorKind::Livox,
        ] {
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
    /// Problems of the tree the recorder would write with this URDF *and* the
    /// sensors' own edges combined, which is what a reader will actually see.
    pub tree_problems: Vec<TreeProblem>,
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
        if !self.problems.is_empty() {
            return Some(format!(
                "urdf tree is broken: {}",
                self.problems
                    .iter()
                    .map(TreeProblem::message)
                    .collect::<Vec<_>>()
                    .join("; ")
            ));
        }
        if !self.tree_problems.is_empty() {
            return Some(format!(
                "urdf and sensor frames together do not make one tree: {}",
                self.tree_problems
                    .iter()
                    .map(TreeProblem::message)
                    .collect::<Vec<_>>()
                    .join("; ")
            ));
        }
        None
    }
}

/// Used by the settings UI to offer sensible frame prefixes.
pub fn default_naming(kind: SensorKind) -> Naming {
    Naming::for_kind(kind)
}

fn naming_for(settings: &Settings, kind: SensorKind) -> &str {
    match kind {
        SensorKind::Realsense => &settings.realsense.naming.topic_prefix,
        SensorKind::Orbbec => &settings.orbbec.naming.topic_prefix,
        SensorKind::OakD => &settings.oakd.naming.topic_prefix,
        SensorKind::Livox => &settings.livox.naming.topic_prefix,
    }
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

    /// Stands in for a camera so the settings-to-device path can be exercised
    /// with no hardware. `absorbs_changes` is what a real backend answers when
    /// the knob that moved is one it can set on a live pipeline.
    struct SpyCamera {
        absorbs_changes: bool,
        applied: Arc<Mutex<Vec<CameraConfig>>>,
    }

    impl Backend for SpyCamera {
        fn start(&mut self, _sink: Sink) -> Result<()> {
            Ok(())
        }

        fn stop(&mut self) {}

        fn status(&self) -> crate::sensors::BackendStatus {
            crate::sensors::BackendStatus {
                running: true,
                detail: "spy".into(),
                error: None,
            }
        }

        fn apply_live(&mut self, config: &CameraConfig) -> bool {
            self.applied.lock().unwrap().push(config.clone());
            self.absorbs_changes
        }
    }

    fn spy(absorbs_changes: bool) -> (Box<dyn Backend>, Arc<Mutex<Vec<CameraConfig>>>) {
        let applied = Arc::new(Mutex::new(Vec::new()));
        let camera = SpyCamera {
            absorbs_changes,
            applied: Arc::clone(&applied),
        };
        (Box::new(camera), applied)
    }

    #[test]
    fn a_changed_setting_reaches_a_camera_that_is_already_streaming() {
        let mut backends: BTreeMap<SensorKind, Box<dyn Backend>> = BTreeMap::new();
        let (camera, applied) = spy(true);
        backends.insert(SensorKind::Realsense, camera);

        let previous = Settings::default();
        let mut settings = previous.clone();
        settings.realsense.emitter = !previous.realsense.emitter;

        let restart = sensors_needing_restart(&mut backends, &previous, &settings);
        assert!(restart.is_empty(), "an emitter flip must not cycle the device");
        assert_eq!(applied.lock().unwrap().len(), 1);
        assert_eq!(applied.lock().unwrap()[0], settings.realsense);
    }

    #[test]
    fn a_setting_the_camera_cannot_absorb_cycles_it_instead() {
        let mut backends: BTreeMap<SensorKind, Box<dyn Backend>> = BTreeMap::new();
        backends.insert(SensorKind::Realsense, spy(false).0);
        backends.insert(SensorKind::OakD, spy(true).0);

        let previous = Settings::default();
        let mut settings = previous.clone();
        settings.realsense.width = previous.realsense.width * 2;
        settings.oakd.emitter = !previous.oakd.emitter;

        assert_eq!(
            sensors_needing_restart(&mut backends, &previous, &settings),
            vec![SensorKind::Realsense],
        );
    }

    #[test]
    fn a_sensor_that_is_not_engaged_is_never_restarted_for_a_settings_change() {
        let mut backends: BTreeMap<SensorKind, Box<dyn Backend>> = BTreeMap::new();
        let previous = Settings::default();
        let mut settings = previous.clone();
        settings.realsense.width = 9999;
        settings.livox.frame_hz = 20.0;
        assert!(sensors_needing_restart(&mut backends, &previous, &settings).is_empty());
    }

    #[test]
    fn changing_the_lidar_cycles_it_because_it_has_no_live_knobs() {
        let mut backends: BTreeMap<SensorKind, Box<dyn Backend>> = BTreeMap::new();
        backends.insert(SensorKind::Livox, spy(true).0);
        let previous = Settings::default();
        let mut settings = previous.clone();
        settings.livox.frame_hz = previous.livox.frame_hz + 5.0;
        assert_eq!(
            sensors_needing_restart(&mut backends, &previous, &settings),
            vec![SensorKind::Livox],
        );
    }

    #[test]
    fn a_settings_write_that_touches_no_sensor_leaves_every_backend_alone() {
        let mut backends: BTreeMap<SensorKind, Box<dyn Backend>> = BTreeMap::new();
        let (camera, applied) = spy(false);
        backends.insert(SensorKind::Realsense, camera);
        let previous = Settings::default();
        let mut settings = previous.clone();
        settings.compression = Compression::Zstd;
        settings.preview_quality = 90;
        assert!(sensors_needing_restart(&mut backends, &previous, &settings).is_empty());
        assert!(applied.lock().unwrap().is_empty());
    }

    #[test]
    fn a_stream_that_went_quiet_is_reported_even_though_the_driver_looks_healthy() {
        let hub = scratch_hub();
        hub.backends
            .lock()
            .unwrap()
            .insert(SensorKind::Realsense, spy(true).0);

        let stale = Instant::now()
            .checked_sub(RATE_WINDOW * 4)
            .expect("the clock has not been running long enough");
        let mut rates = hub.rates.lock().unwrap();
        rates.insert(
            "/realsense/imu".into(),
            RateCounter {
                count: 1000,
                window_started: Some(stale),
                window_count: 0,
                hz: 200.0,
            },
        );
        rates.insert(
            "/realsense/color_image".into(),
            RateCounter {
                count: 500,
                window_started: Some(Instant::now()),
                window_count: 0,
                hz: 30.0,
            },
        );
        // Latched: one message, no rate, quiet ever since. Not a fault.
        rates.insert(
            "/realsense/camera_info".into(),
            RateCounter {
                count: 1,
                window_started: Some(stale),
                window_count: 1,
                hz: 0.0,
            },
        );
        drop(rates);

        let realsense = &hub.sensor_status()["realsense"];
        assert!(realsense.running);
        let error = realsense.error.as_deref().expect("a dead stream is an error");
        assert!(error.contains("/realsense/imu"), "{error}");
        assert!(!error.contains("color_image"), "{error}");
        assert!(!error.contains("camera_info"), "{error}");
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn with_no_preview_chosen_the_first_stream_of_an_enabled_camera_is_previewed() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.preview_topic = None;
        settings.realsense.enabled = false;
        settings.oakd.enabled = true;
        settings.oakd.color = true;
        hub.update_settings(settings.clone()).unwrap();

        let chosen = hub.preview_topic().expect("something has to be previewed");
        assert!(
            chosen.starts_with(&settings.oakd.naming.topic_prefix),
            "picked {chosen}, which is not on the camera that is switched on",
        );
        // The dropdown still offers every stream, so the operator can pick one
        // on a camera they are about to switch on.
        assert!(hub.preview_topics().len() > 1);
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn an_explicit_preview_choice_is_never_overridden_by_the_automatic_one() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        let wanted = settings.oakd.naming.image_topic(StreamId::InfraRight);
        settings.preview_topic = Some(wanted.clone());
        hub.update_settings(settings).unwrap();
        assert_eq!(hub.preview_topic(), Some(wanted));
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// The UI can only offer topics that exist, so a choice naming one that does
    /// not comes from a settings file written before a rename. Honouring it
    /// leaves the preview permanently blank *and* has the dropdown showing a
    /// different topic than the one the backend is matching on.
    #[test]
    fn a_preview_choice_no_sensor_publishes_falls_back_to_a_real_one() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.preview_topic = Some("/realsense/color/image_raw".into());
        hub.update_settings(settings).unwrap();
        let chosen = hub.preview_topic().expect("something has to be previewed");
        assert!(hub.preview_topics().contains(&chosen), "picked {chosen}");
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// Switching a camera's last stream off used to take the whole camera out of
    /// the preview dropdown, and the dropdown is where the switches live, so
    /// there was no way to switch anything back on without editing the settings
    /// file by hand.
    #[test]
    fn a_camera_with_every_stream_switched_off_is_still_listed() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.realsense.depth = false;
        settings.realsense.color = false;
        settings.realsense.infrared = false;
        settings.realsense.imu = false;
        hub.update_settings(settings).unwrap();

        let topics = hub.preview_topics();
        assert!(topics.contains(&"/realsense/color_image".to_owned()), "{topics:?}");
        // Still last, so the automatic choice lands on a stream that can produce.
        let chosen = hub.preview_topic().unwrap();
        assert!(!chosen.starts_with("/realsense"), "previewing a dead stream: {chosen}");
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// The monitor's per-stream switch is only as good as this map: a topic the
    /// map has no entry for gets no checkbox, so an operator who renamed a
    /// prefix would silently lose the ability to switch that stream off.
    #[test]
    fn every_stream_topic_names_the_setting_that_switches_it_off() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.realsense.naming.topic_prefix = "/front_cam".into();
        let paths = super::topic_settings(&settings);

        assert_eq!(paths.get("/front_cam/color_image").unwrap(), "realsense.color");
        assert_eq!(paths.get("/front_cam/camera_info").unwrap(), "realsense.color");
        assert_eq!(paths.get("/front_cam/depth_image").unwrap(), "realsense.depth");
        // One switch for the stereo pair, as on the device itself.
        assert_eq!(paths.get("/front_cam/infrared_left").unwrap(), "realsense.infrared");
        assert_eq!(paths.get("/front_cam/infrared_right").unwrap(), "realsense.infrared");
        assert_eq!(paths.get("/front_cam/imu").unwrap(), "realsense.imu");
        assert_eq!(paths.get("/livox/lidar").unwrap(), "livox.enabled");
        assert_eq!(paths.get("/livox/imu").unwrap(), "livox.imu");

        // Every path has to resolve in the Settings the browser writes back to,
        // or the checkbox saves a key the server then ignores.
        let json = serde_json::to_value(&settings).unwrap();
        for (topic, path) in &paths {
            let mut at = &json;
            for key in path.split('.') {
                at = at.get(key).unwrap_or_else(|| panic!("{topic} -> {path}"));
            }
            assert!(at.is_boolean(), "{path} is not a switch");
        }
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_settings_file_missing_a_whole_camera_block_keeps_its_other_settings() {
        let mut written = serde_json::to_value(Settings::default()).unwrap();
        written["record_dir"] = serde_json::json!("/somewhere/else");
        written.as_object_mut().unwrap().remove("oakd");

        let file = std::env::temp_dir().join(format!(
            "lite_record_partial_{}.json",
            record::now_nanos()
        ));
        std::fs::write(&file, written.to_string()).unwrap();
        let loaded = Hub::load_settings(&file);
        std::fs::remove_file(&file).ok();

        assert_eq!(loaded.record_dir, PathBuf::from("/somewhere/else"));
        assert_eq!(loaded.oakd, CameraConfig::for_kind(SensorKind::OakD));
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
        settings.preview_topic = Some("/realsense/color_image".into());
        hub.update_settings(settings.clone()).unwrap();

        let sink = hub.sink();
        for _ in 0..3 {
            sink(Produced::Image {
                stream: StreamId::Color,
                topic: "/realsense/color_image".into(),
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
                topic: "/realsense/color_image".into(),
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
        settings.preview_topic = Some("/realsense/color_image".into());
        hub.update_settings(settings).unwrap();
        let sink = hub.sink();
        for _ in 0..5 {
            sink(Produced::Image {
                stream: StreamId::Depth,
                topic: "/realsense/depth_image".into(),
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
        // Aimed away from depth: an unset preview falls back to the first image
        // stream, and that is depth.
        let mut settings = hub.settings();
        settings.preview_topic = Some("/realsense/color_image".into());
        hub.update_settings(settings).unwrap();
        let sink = hub.sink();
        for _ in 0..20 {
            sink(Produced::Image {
                stream: StreamId::Depth,
                topic: "/realsense/depth_image".into(),
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
                topic: "/realsense/depth_image".into(),
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

    /// The encode pool has several workers, and a stream that could be split
    /// across them arrives in the file in whatever order they happened to
    /// finish. At 200 Hz that showed up on the Pi as a header stamp landing
    /// behind the one before it, about once every nine thousand samples.
    #[test]
    fn a_stream_reaches_the_file_in_the_order_it_was_produced() {
        let hub = scratch_hub();
        let directory = std::env::temp_dir()
            .join(format!("lite_record_order_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        hub.update_settings(settings).unwrap();

        let sink = hub.sink();
        hub.start_recording(Some("order")).unwrap();
        let samples = 4000;
        for index in 1..=samples {
            // Re-offered rather than shed, so the file has to hold every one of
            // them and a gap would be this test failing rather than the queue
            // being short.
            while !sink(Produced::Imu {
                topic: "/cam/imu".into(),
                imu: Box::new(crate::msgs::Imu::unoriented(
                    crate::msgs::Header::new(index * 5_000_000, "cam"),
                    [0.0; 3],
                    [0.0, 0.0, 9.81],
                )),
            }) {
                std::thread::yield_now();
            }
        }
        assert!(wait_for(|| hub.record_encode_count() == samples));
        let status = hub.stop_recording().unwrap();
        let path = status.path.clone().unwrap();

        let bytes = std::fs::read(&path).unwrap();
        let stamps: Vec<u64> = mcap::MessageStream::new(&bytes)
            .unwrap()
            .map(|message| message.unwrap())
            .filter(|message| message.channel.topic == "/cam/imu")
            .map(|message| {
                let (seconds, nanos) = (
                    i32::from_le_bytes(message.data[4..8].try_into().unwrap()) as u64,
                    u32::from_le_bytes(message.data[8..12].try_into().unwrap()) as u64,
                );
                seconds * 1_000_000_000 + nanos
            })
            .collect();
        // The loop above only backpressures the encode pool. Past it the writer
        // queue sheds by design, and on a loaded machine it does, so a bare
        // count would be asserting the machine was idle. Every sample must be
        // written or counted, and the ones written must be in order.
        let tally = &status.topics["/cam/imu"];
        assert_eq!(stamps.len() as u64, tally.written);
        assert_eq!(tally.written + tally.dropped, samples);
        let mut sorted = stamps.clone();
        sorted.sort_unstable();
        assert_eq!(stamps, sorted, "the pool reordered a stream");

        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// The ordering guarantee above only holds for a topic the hash pins to one
    /// worker. Both newer backends were added after that hash existed, so this
    /// names their real topics rather than trusting that they inherited it.
    #[test]
    fn the_oakd_and_livox_topics_each_belong_to_exactly_one_encode_worker() {
        let oakd = Naming::for_kind(SensorKind::OakD);
        let livox = Naming::for_kind(SensorKind::Livox);
        let topics = [
            oakd.imu_topic(),
            oakd.image_topic(StreamId::Color),
            oakd.image_topic(StreamId::Depth),
            oakd.image_topic(StreamId::InfraLeft),
            oakd.image_topic(StreamId::InfraRight),
            livox.imu_topic(),
            livox.points_topic(),
        ];
        for workers in 1..=16 {
            for topic in &topics {
                let owner = worker_for(topic, workers);
                assert!(owner < workers, "{topic} hashed outside the pool");
                assert_eq!(
                    owner,
                    worker_for(topic, workers),
                    "{topic} did not land on a stable worker"
                );
            }
        }
    }

    /// Criterion 12: the OAK-D and Livox backends hand everything to `Hub::sink`,
    /// so their samples take the same hashed queue every other sensor does. Run
    /// together because interleaving is what would expose a stream being split.
    #[test]
    fn oakd_and_livox_streams_both_reach_the_file_in_the_order_produced() {
        let hub = scratch_hub();
        let directory =
            std::env::temp_dir().join(format!("lite_record_pool_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        hub.update_settings(settings).unwrap();

        let oakd_imu = Naming::for_kind(SensorKind::OakD).imu_topic();
        let livox_imu = Naming::for_kind(SensorKind::Livox).imu_topic();
        let sink = hub.sink();
        hub.start_recording(Some("pool")).unwrap();
        let per_topic = 2000;
        for index in 1..=per_topic {
            for topic in [&oakd_imu, &livox_imu] {
                while !sink(Produced::Imu {
                    topic: topic.clone(),
                    imu: Box::new(crate::msgs::Imu::unoriented(
                        crate::msgs::Header::new(index * 5_000_000, "rig"),
                        [0.0; 3],
                        [0.0, 0.0, 9.81],
                    )),
                }) {
                    std::thread::yield_now();
                }
            }
        }
        assert!(wait_for(|| hub.record_encode_count() == per_topic * 2));
        let status = hub.stop_recording().unwrap();
        let path = status.path.clone().unwrap();

        let bytes = std::fs::read(&path).unwrap();
        for topic in [&oakd_imu, &livox_imu] {
            let stamps: Vec<u64> = mcap::MessageStream::new(&bytes)
                .unwrap()
                .map(|message| message.unwrap())
                .filter(|message| &message.channel.topic == topic)
                .map(|message| {
                    let (seconds, nanos) = (
                        i32::from_le_bytes(message.data[4..8].try_into().unwrap()) as u64,
                        u32::from_le_bytes(message.data[8..12].try_into().unwrap()) as u64,
                    );
                    seconds * 1_000_000_000 + nanos
                })
                .collect();
            // The writer queue sheds by design past the encode pool, so this
            // asserts every sample was written or counted rather than that the
            // machine happened to be idle enough to keep up.
            let tally = &status.topics[topic];
            assert_eq!(stamps.len() as u64, tally.written, "{topic} lost samples");
            assert_eq!(tally.written + tally.dropped, per_topic, "{topic}");
            let mut sorted = stamps.clone();
            sorted.sort_unstable();
            assert_eq!(stamps, sorted, "the pool reordered {topic}");
        }

        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn a_recording_opens_with_tf_before_any_sensor_data_and_repeats_it() {
        let hub = scratch_hub();
        let directory =
            std::env::temp_dir().join(format!("lite_record_tf_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        settings.realsense.enabled = true;
        settings.urdf_xml = Some(
            r#"<robot name="rig">
                <link name="base_link"/><link name="camera_link"/>
                <joint name="j" type="fixed">
                    <parent link="base_link"/><child link="camera_link"/>
                    <origin xyz="0 0 0.1"/>
                </joint>
            </robot>"#
                .into(),
        );
        hub.update_settings(settings).unwrap();

        hub.start_recording(Some("tf.mcap")).unwrap();
        // Three copies at 5 Hz take 400 ms on an idle machine; a loaded one
        // schedules the repeater late, so wait for the count, not the clock.
        // Four, not three: the one-off /tf_static message is in this total too,
        // so waiting for three would stop after only two /tf repeats.
        let deadline = Instant::now() + Duration::from_secs(10);
        while hub.recording_status().messages < 4 {
            assert!(Instant::now() < deadline, "the static transforms were not repeated");
            std::thread::sleep(Duration::from_millis(20));
        }
        let status = hub.stop_recording().unwrap();
        assert!(status.messages >= 4, "{}", status.messages);

        let bytes = std::fs::read(directory.join("tf.mcap")).unwrap();
        let messages: Vec<_> = mcap::MessageStream::new(&bytes)
            .unwrap()
            .map(|message| message.unwrap())
            .collect();
        // The tree is on the wire before any sensor data, on both topics.
        let first = &messages[0];
        assert!(
            first.channel.topic == record::TF_STATIC_TOPIC || first.channel.topic == record::TF_TOPIC,
            "a recording must open with transforms, not {}",
            first.channel.topic
        );
        assert_eq!(
            first.channel.schema.as_ref().unwrap().name,
            "tf2_msgs/msg/TFMessage"
        );

        // Both topics carry the convention marker. Without it on /tf_static,
        // `convert` reads a fresh file as an old recorder's and inverts the edges.
        for topic in [record::TF_TOPIC, record::TF_STATIC_TOPIC] {
            let channel = messages
                .iter()
                .find(|message| message.channel.topic == topic)
                .unwrap_or_else(|| panic!("nothing was written to {topic}"))
                .channel
                .clone();
            assert_eq!(
                channel.metadata.get(record::TRANSFORM_CONVENTION_KEY).map(String::as_str),
                Some(record::TRANSFORM_CONVENTION_VALUE),
                "{topic} is missing the transform convention marker"
            );
        }

        // Repeated at 5 Hz with fresh stamps on /tf, the way dimos publishes them.
        let tf_stamps: Vec<u64> = messages
            .iter()
            .filter(|message| message.channel.topic == record::TF_TOPIC)
            .map(|message| message.log_time)
            .collect();
        assert!(tf_stamps.len() >= 3, "{tf_stamps:?}");
        assert!(tf_stamps.windows(2).all(|pair| pair[1] > pair[0]), "{tf_stamps:?}");

        // Written exactly once on /tf_static: a latched topic does not need repeating,
        // and a viewer that honours latching has the whole tree from t=0.
        let static_messages: Vec<_> = messages
            .iter()
            .filter(|message| message.channel.topic == record::TF_STATIC_TOPIC)
            .collect();
        assert_eq!(static_messages.len(), 1, "expected one /tf_static message");

        // Identical edges on both, so neither consumer sees a different rig.
        let first_tf = messages
            .iter()
            .find(|message| message.channel.topic == record::TF_TOPIC)
            .unwrap();
        assert_eq!(
            static_messages[0].data, first_tf.data,
            "/tf_static and /tf disagree about the rig"
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
            topic: "/realsense/camera_info".into(),
            info: Box::new(CameraInfo::pinhole(
                crate::msgs::Header::new(1, "camera_color_optical_frame"),
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
            topics.iter().any(|t| t == "/realsense/camera_info"),
            "got {topics:?}"
        );
        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// The calibration a camera announced at boot must not carry a boot-time
    /// stamp into a file recorded later — on a Pi that stamp predates NTP's
    /// The grocery recording's lesson: with no URDF the sensors' placeholder
    /// edges make separate trees, and a URDF must be judged together with
    /// those edges, not on its own.
    #[test]
    fn a_rig_without_a_urdf_is_flagged_and_a_joining_urdf_clears_it() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.realsense.enabled = true;
        settings.livox.enabled = true;
        hub.update_settings(settings).unwrap();

        let without = hub.tree_problems(None);
        assert!(
            without.iter().any(|problem| matches!(problem, TreeProblem::MultipleRoots { .. })),
            "two sensors with no urdf must be reported as separate trees: {without:?}"
        );
        assert!(hub.inspect_urdf(None).warning().is_some());

        let joined = r#"<robot name="rig">
            <link name="base_link"/><link name="camera_link"/><link name="livox_link"/>
            <link name="livox_frame"/><link name="livox_imu_frame"/>
            <joint name="c" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="0 0 0"/></joint>
            <joint name="l" type="fixed"><parent link="base_link"/><child link="livox_link"/><origin xyz="0 0 0.2"/></joint>
            <joint name="o" type="fixed"><parent link="livox_link"/><child link="livox_frame"/><origin xyz="0 0 0.047"/></joint>
            <joint name="i" type="fixed"><parent link="livox_frame"/><child link="livox_imu_frame"/><origin xyz="0.011 0.023 -0.044"/></joint>
        </robot>"#;
        let with = hub.tree_problems(Some(joined));
        assert!(with.is_empty(), "a urdf that joins every sensor frame must report nothing: {with:?}");
        let report = hub.inspect_urdf(Some(joined));
        assert!(report.warning().is_none(), "{:?}", report.warning());
        // And the published payload really has one parent per frame.
        let transforms = hub.static_transforms_with(Some(joined), 0);
        let imu_parents: Vec<_> = transforms
            .iter()
            .filter(|t| t.child_frame_id == "livox_imu_frame")
            .map(|t| t.header.frame_id.clone())
            .collect();
        assert_eq!(imu_parents, vec!["livox_frame".to_string()]);
    }

    /// correction and lands thousands of seconds before the images.
    #[test]
    fn a_replayed_camera_info_is_stamped_when_the_recording_started() {
        let hub = scratch_hub();
        let directory =
            std::env::temp_dir().join(format!("lite_record_stamp_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        hub.update_settings(settings).unwrap();

        // Announced with a stamp half an hour in the past, the way a camera
        // opened before the clock was corrected would have.
        let announced = record::now_nanos() - 2_005 * crate::msgs::NANOS_PER_SEC;
        hub.sink()(Produced::CameraInfo {
            topic: "/realsense/camera_info".into(),
            info: Box::new(CameraInfo::pinhole(
                crate::msgs::Header::new(announced, "camera_color_optical_frame"),
                640, 480, 600.0, 600.0, 320.0, 240.0,
                crate::msgs::DistortionModel::PlumbBob, vec![0.0; 5], 0.0,
            )),
        });
        let deadline = Instant::now() + Duration::from_secs(5);
        while hub.latest_intrinsics.lock().unwrap().is_empty() {
            assert!(Instant::now() < deadline, "the announcement never arrived");
            std::thread::sleep(Duration::from_millis(10));
        }

        let started = record::now_nanos();
        hub.start_recording(Some("stamp.mcap")).unwrap();
        hub.stop_recording().unwrap();

        let bytes = std::fs::read(directory.join("stamp.mcap")).unwrap();
        let info = mcap::MessageStream::new(&bytes)
            .unwrap()
            .map(Result::unwrap)
            .find(|message| message.channel.topic == "/realsense/camera_info")
            .expect("no camera_info in the file");
        let stamp = crate::cdr::decode_header(&info.data).unwrap().stamp_nanos();
        assert!(
            stamp >= started,
            "the calibration kept its announce stamp, {} s before the recording",
            (started - stamp) as f64 / crate::msgs::NANOS_PER_SEC as f64
        );
        assert!(stamp < started + 60 * crate::msgs::NANOS_PER_SEC);
        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    #[test]
    fn the_tf_static_payload_joins_every_optical_frame_to_the_urdf() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.realsense.enabled = true;
        settings.urdf_xml = Some(
            r#"<robot name="rig"><link name="base_link"/><link name="camera_link"/>
                <joint name="j" type="fixed"><parent link="base_link"/><child link="camera_link"/></joint>
            </robot>"#
                .into(),
        );
        hub.update_settings(settings).unwrap();

        let transforms = hub.static_transforms(1);
        let children: Vec<&str> = transforms
            .iter()
            .map(|t| t.child_frame_id.as_str())
            .collect();
        assert!(children.contains(&"camera_link"));
        assert!(children.contains(&"camera_depth_optical_frame"));
        assert!(children.contains(&"camera_color_optical_frame"));
        assert!(children.contains(&"camera_infra1_optical_frame"));
        assert!(children.contains(&"camera_imu_frame"));
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
    /// A URDF that states where a sensor's own frame really is must not be
    /// shadowed by the identity placeholder. The Mid-360's point origin is
    /// 47 mm above the seat it bolts to, and publishing both would give
    /// livox_frame two parents in a single message.
    #[test]
    fn a_frame_the_urdf_places_does_not_also_get_an_identity_placeholder() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.livox.enabled = true;
        settings.urdf_xml = Some(
            r#"<robot name="rig">
                <link name="base_link"/><link name="livox_link"/><link name="livox_frame"/>
                <joint name="mount" type="fixed"><parent link="base_link"/><child link="livox_link"/></joint>
                <joint name="origin" type="fixed">
                    <parent link="livox_link"/><child link="livox_frame"/>
                    <origin xyz="0 0 0.047"/>
                </joint>
            </robot>"#
                .into(),
        );
        hub.update_settings(settings).unwrap();

        let transforms = hub.static_transforms(1);
        let placements: Vec<&TransformStamped> = transforms
            .iter()
            .filter(|transform| transform.child_frame_id == "livox_frame")
            .collect();
        assert_eq!(placements.len(), 1, "livox_frame was published twice: {placements:?}");
        assert_eq!(placements[0].translation[2], 0.047, "the placeholder won");
        // A frame the urdf says nothing about still gets its placeholder.
        assert!(
            transforms.iter().any(|transform| transform.child_frame_id == "livox_imu_frame"),
            "an unplaced frame lost its fallback"
        );
        std::fs::remove_file(hub.settings_file()).ok();
    }

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
            frame: "livox_link".into()
        }));
        assert!(report.warning().unwrap().contains("livox_link"));
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// The colour stream lost a third of its frames at 720p30 while the
    /// recording reported no drops at all, because a frame shed on the way to
    /// an encode worker never reaches the writer, so only the hub knows it
    /// happened.
    #[test]
    fn a_frame_shed_before_the_recorder_is_still_the_recordings_drop() {
        let hub = scratch_hub();
        let directory = std::env::temp_dir().join(format!("lite_shed_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = hub.settings();
        settings.record_dir = directory.clone();
        hub.update_settings(settings).unwrap();

        // Shed before this recording, so it must not be charged to it.
        *hub.pipeline_dropped
            .lock()
            .unwrap()
            .entry("/realsense/color_image".into())
            .or_default() += 7;
        hub.start_recording(Some("shed")).unwrap();

        hub.sink()(Produced::Image {
            stream: StreamId::Color,
            topic: "/realsense/color_image".into(),
            image: an_image(4, 4),
        });
        assert!(wait_for(|| hub
            .recording_status()
            .topics
            .contains_key("/realsense/color_image")));
        *hub.pipeline_dropped
            .lock()
            .unwrap()
            .entry("/realsense/color_image".into())
            .or_default() += 3;

        let status = hub.recording_status();
        let tally = status
            .topics
            .get("/realsense/color_image")
            .expect("the frame is written under the stream topic");
        assert_eq!(tally.written, 1);
        assert_eq!(tally.dropped, 3);
        assert_eq!(status.dropped, 3);

        hub.stop_recording().unwrap();
        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(hub.settings_file()).ok();
    }

    /// A urdf that stops at the sensor's root frame is complete: the stream
    /// frames under it come from `static_transforms`, not the file.
    #[test]
    fn a_urdf_covering_only_the_livox_root_frame_is_accepted() {
        let hub = scratch_hub();
        let mut settings = hub.settings();
        settings.livox.enabled = true;
        hub.update_settings(settings).unwrap();
        let report = hub.inspect_urdf(Some(
            r#"<robot name="x"><link name="base_link"/><link name="livox_link"/>
                <joint name="j" type="fixed">
                    <parent link="base_link"/><child link="livox_link"/>
                </joint>
            </robot>"#,
        ));
        assert_eq!(report.problems, Vec::new());
        assert!(report.warning().is_none(), "{:?}", report.warning());
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
        assert_eq!(status.len(), 4);
        assert!(!status["livox"].running);
        if !cfg!(feature = "realsense") {
            assert_eq!(status["realsense"].detail, "not compiled in");
            assert!(hub.engage(SensorKind::Realsense).is_err());
        }
        if !cfg!(feature = "oakd") {
            assert_eq!(status["oakd"].detail, "not compiled in");
            assert!(hub.engage(SensorKind::OakD).is_err());
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
