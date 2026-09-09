//! Batched mcap writer.
//!
//! Sensor threads hand fully-encoded CDR messages to `offer`, which never
//! blocks: a full queue increments a drop counter instead of stalling the
//! capture thread, because a stalled RealSense callback backs up the SDK's own
//! frame queue and costs far more frames than the one we shed here.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use mcap::{WriteOptions, Writer};
use serde::{Deserialize, Serialize};

use crate::cdr::Encoded;

/// Deep enough to ride out a disk hiccup, shallow enough that a genuinely
/// overloaded machine sheds frames instead of growing an unbounded backlog and
/// running the Pi out of memory.
const QUEUE_DEPTH: usize = 1024;

/// How often the writer thread pushes its buffer at the filesystem. Chunks are
/// closed on this cadence too, so a hard power cut loses at most this much.
const FLUSH_INTERVAL: Duration = Duration::from_secs(2);

/// Messages pulled off the queue before checking the clock again. Draining in
/// bursts keeps the per-message cost down without letting the flush slip.
const BATCH_SIZE: usize = 256;

/// The mcap channel a sample is written to. A compressed frame takes a
/// `/compressed` suffix so that post-processing, which decodes depth to raw
/// pixels, can hand the plain name to the stream a dimos graph expects. It also
/// keeps the two schemas off one topic name, which a ROS consumer cannot make
/// sense of.
fn channel_topic(sample: &Sample) -> String {
    if sample.encoded.schema_name == crate::msgs::COMPRESSED_IMAGE_TYPE {
        format!("{}{}", sample.topic, crate::convert::COMPRESSED_SUFFIX)
    } else {
        sample.topic.clone()
    }
}

pub struct Sample {
    pub topic: String,
    pub encoded: Encoded,
    pub log_time: u64,
}

/// Chunk compression for the mcap file. Zstd at level 1 is the default: on real
/// 720p depth it beats lz4 on both size and CPU, because its entropy stage suits
/// run-structured 16-bit data. Level 1 rather than zstd's own default of 3,
/// which on a dense scene costs the writer thread three times as much for a few
/// percent of size.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug, Default)]
#[serde(rename_all = "lowercase")]
pub enum Compression {
    None,
    Lz4,
    #[default]
    Zstd,
}

impl Compression {
    fn to_mcap(self) -> Option<mcap::Compression> {
        match self {
            Compression::None => None,
            Compression::Lz4 => Some(mcap::Compression::Lz4),
            Compression::Zstd => Some(mcap::Compression::Zstd),
        }
    }

    fn level(self) -> u32 {
        match self {
            Compression::Zstd => 1,
            // lz4's own level scale starts at 0, and mcap reads 0 as "default".
            Compression::None | Compression::Lz4 => 0,
        }
    }
}

#[derive(Default)]
struct Counters {
    messages: AtomicU64,
    bytes: AtomicU64,
    dropped: AtomicU64,
}

#[derive(Default, Clone, Serialize)]
pub struct TopicTally {
    pub written: u64,
    pub dropped: u64,
}

impl TopicTally {
    pub fn drop_fraction(&self) -> f64 {
        let offered = self.written + self.dropped;
        if offered == 0 {
            return 0.0;
        }
        self.dropped as f64 / offered as f64
    }
}

#[derive(Serialize, Clone)]
pub struct RecordingStatus {
    pub active: bool,
    pub path: Option<String>,
    pub messages: u64,
    pub bytes: u64,
    pub dropped: u64,
    pub seconds: f64,
    pub topics: BTreeMap<String, TopicTally>,
}

pub fn idle_status() -> RecordingStatus {
    RecordingStatus {
        active: false,
        path: None,
        messages: 0,
        bytes: 0,
        dropped: 0,
        seconds: 0.0,
        topics: BTreeMap::new(),
    }
}

pub struct Recorder {
    path: PathBuf,
    started: SystemTime,
    counters: Arc<Counters>,
    tallies: Arc<Mutex<BTreeMap<String, TopicTally>>>,
    sender: Option<SyncSender<Sample>>,
    worker: Option<JoinHandle<Result<()>>>,
}

impl Recorder {
    pub fn start(path: &Path, compression: Compression) -> Result<Self> {
        if let Some(parent) = path.parent().filter(|parent| !parent.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("could not create {}", parent.display()))?;
        }
        let file =
            File::create(path).with_context(|| format!("could not create {}", path.display()))?;
        let writer = WriteOptions::new()
            .compression(compression.to_mcap())
            .compression_level(compression.level())
            .profile("ros2")
            .create(BufWriter::with_capacity(1 << 20, file))?;

        let (sender, receiver) = sync_channel(QUEUE_DEPTH);
        let counters = Arc::new(Counters::default());
        let tallies = Arc::new(Mutex::new(BTreeMap::new()));
        let worker = {
            let counters = Arc::clone(&counters);
            let tallies = Arc::clone(&tallies);
            std::thread::Builder::new()
                .name("mcap-writer".into())
                .spawn(move || drain(writer, receiver, counters, tallies))?
        };

        Ok(Recorder {
            path: path.to_path_buf(),
            started: SystemTime::now(),
            counters,
            tallies,
            sender: Some(sender),
            worker: Some(worker),
        })
    }

    /// Never blocks. Returns false when the message was shed.
    pub fn offer(&self, topic: &str, encoded: Encoded) -> bool {
        let Some(sender) = self.sender.as_ref() else {
            return false;
        };
        let sample = Sample {
            topic: topic.to_string(),
            encoded,
            log_time: now_nanos(),
        };
        match sender.try_send(sample) {
            Err(TrySendError::Full(sample)) => {
                self.counters.dropped.fetch_add(1, Ordering::Relaxed);
                self.tallies
                    .lock()
                    .unwrap()
                    .entry(sample.topic)
                    .or_default()
                    .dropped += 1;
                false
            }
            Err(TrySendError::Disconnected(_)) => false,
            Ok(()) => true,
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn status(&self) -> RecordingStatus {
        RecordingStatus {
            active: true,
            path: Some(self.path.display().to_string()),
            messages: self.counters.messages.load(Ordering::Relaxed),
            bytes: self.counters.bytes.load(Ordering::Relaxed),
            dropped: self.counters.dropped.load(Ordering::Relaxed),
            seconds: self
                .started
                .elapsed()
                .map(|age| age.as_secs_f64())
                .unwrap_or(0.0),
            topics: self.tallies.lock().unwrap().clone(),
        }
    }

    /// Flushes the queue and closes the file. The tally is read after the join,
    /// since the queued tail is still being written until then.
    pub fn finish(mut self) -> Result<RecordingStatus> {
        drop(self.sender.take());
        if let Some(worker) = self.worker.take() {
            worker
                .join()
                .map_err(|_| anyhow::anyhow!("mcap writer thread panicked"))??;
        }
        Ok(RecordingStatus {
            active: false,
            ..self.status()
        })
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        drop(self.sender.take());
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn drain(
    mut writer: Writer<BufWriter<File>>,
    receiver: Receiver<Sample>,
    counters: Arc<Counters>,
    tallies: Arc<Mutex<BTreeMap<String, TopicTally>>>,
) -> Result<()> {
    // Keyed by source topic *and* schema: an image stream carries
    // CompressedImage normally and falls back to Image when the codec cannot
    // hold the stream's bit depth, and an mcap channel binds to exactly one
    // schema. The two get different channel names, see `channel_topic`.
    let mut channels: HashMap<(String, &'static str), (u16, u32)> = HashMap::new();
    let mut schemas: HashMap<&'static str, u16> = HashMap::new();
    let mut written_since_flush = 0usize;
    let mut last_flush = Instant::now();
    let mut open = true;

    while open {
        let mut batch = Vec::with_capacity(BATCH_SIZE);
        match receiver.recv_timeout(FLUSH_INTERVAL) {
            Ok(sample) => batch.push(sample),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => open = false,
        }
        while batch.len() < BATCH_SIZE {
            match receiver.try_recv() {
                Ok(sample) => batch.push(sample),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    open = false;
                    break;
                }
            }
        }

        for sample in batch {
            let key = (sample.topic.clone(), sample.encoded.schema_name);
            let channel_id = match channels.get(&key) {
                Some((id, _)) => *id,
                None => {
                    let schema_id = match schemas.get(sample.encoded.schema_name) {
                        Some(id) => *id,
                        None => {
                            let id = writer.add_schema(
                                sample.encoded.schema_name,
                                "ros2msg",
                                sample.encoded.schema_text.as_bytes(),
                            )?;
                            schemas.insert(sample.encoded.schema_name, id);
                            id
                        }
                    };
                    let id = writer.add_channel(
                        schema_id,
                        &channel_topic(&sample),
                        "cdr",
                        &BTreeMap::new(),
                    )?;
                    channels.insert(key.clone(), (id, 0));
                    id
                }
            };

            let sequence = {
                let slot = channels.get_mut(&key).expect("just inserted");
                slot.1 = slot.1.wrapping_add(1);
                slot.1
            };

            writer.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id,
                    sequence,
                    log_time: sample.log_time,
                    publish_time: sample.log_time,
                },
                &sample.encoded.data,
            )?;
            counters.messages.fetch_add(1, Ordering::Relaxed);
            counters
                .bytes
                .fetch_add(sample.encoded.data.len() as u64, Ordering::Relaxed);
            tallies
                .lock()
                .unwrap()
                .entry(sample.topic)
                .or_default()
                .written += 1;
            written_since_flush += 1;
        }

        if written_since_flush > 0 && last_flush.elapsed() >= FLUSH_INTERVAL {
            writer.flush()?;
            written_since_flush = 0;
            last_flush = Instant::now();
        }
    }
    writer.finish()?;
    Ok(())
}

#[derive(Serialize)]
pub struct RecordingFile {
    pub name: String,
    pub path: String,
    pub bytes: u64,
    /// Unix seconds, so the browser can render it in the operator's own zone.
    /// The Pi has no battery-backed clock, so this can read 1970 until ntp
    /// catches up — the browser shows whatever the file says rather than hiding
    /// it, because a wrong-looking date is the symptom worth seeing.
    pub modified: i64,
}

pub fn list(directory: &Path) -> Vec<RecordingFile> {
    let Ok(entries) = std::fs::read_dir(directory) else {
        return Vec::new();
    };
    let mut files: Vec<RecordingFile> = entries
        .flatten()
        .filter(|entry| entry.path().extension().is_some_and(|end| end == "mcap"))
        .filter_map(|entry| {
            let metadata = entry.metadata().ok()?;
            Some(RecordingFile {
                name: entry.file_name().to_string_lossy().into_owned(),
                path: entry.path().display().to_string(),
                bytes: metadata.len(),
                modified: metadata
                    .modified()
                    .ok()
                    .and_then(|when| when.duration_since(UNIX_EPOCH).ok())
                    .map(|since| since.as_secs() as i64)
                    .unwrap_or(0),
            })
        })
        .collect();
    files.sort_by_key(|file| std::cmp::Reverse(file.modified));
    files
}

/// Resolves a client-supplied recording name against the recording directory.
/// Anything with a path separator is refused outright, so a browser cannot talk
/// the server into touching an unrelated file.
///
/// A bare name gains the `.mcap` extension rather than being rejected: this
/// program chooses the container it writes, so spelling the extension out is not
/// the operator's job. A name carrying some *other* extension is still refused,
/// because that is a mistake rather than an omission.
pub fn resolve(directory: &Path, name: &str) -> Result<PathBuf> {
    let mut parts = Path::new(name).components();
    let Some(std::path::Component::Normal(single)) = parts.next() else {
        anyhow::bail!("recording name must be a plain file name");
    };
    if parts.next().is_some() {
        anyhow::bail!("recording name must be a plain file name");
    }
    let path = directory.join(single);
    match path.extension() {
        None => Ok(path.with_extension("mcap")),
        Some(end) if end == "mcap" => Ok(path),
        Some(end) => anyhow::bail!("recording name must end in .mcap, not .{}", end.to_string_lossy()),
    }
}

pub fn default_name() -> String {
    let seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|age| age.as_secs())
        .unwrap_or(0);
    format!("lite_record_{seconds}.mcap")
}

pub fn now_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|age| age.as_nanos() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{Header, Imu};

    fn scratch(label: &str) -> PathBuf {
        let directory = std::env::temp_dir().join(format!("lite_record_{label}_{}", now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        directory
    }

    fn an_imu(index: u64) -> Encoded {
        crate::cdr::imu(&Imu::unoriented(
            Header::new(index * 5_000_000, "livox_frame"),
            [0.0, 0.0, 0.1],
            [0.0, 0.0, 9.81],
        ))
    }

    #[test]
    fn a_batched_run_reads_back_with_every_message_and_its_schema() {
        let directory = scratch("batched");
        let path = directory.join("out.mcap");
        let recorder = Recorder::start(&path, Compression::Lz4).unwrap();
        for index in 0..2000 {
            // 2000 messages into a queue of QUEUE_DEPTH, so on a machine busy
            // enough that the writer falls behind this legitimately sheds. That
            // is the subject of its own test; here the point is that everything
            // handed over comes back, so a shed sample is re-offered instead.
            while !recorder.offer("/livox/imu", an_imu(index)) {
                std::thread::yield_now();
            }
        }
        let status = recorder.finish().unwrap();
        assert_eq!(status.messages, 2000);
        assert_eq!(status.topics["/livox/imu"].written, 2000);

        let bytes = std::fs::read(&path).unwrap();
        let read: Vec<_> = mcap::MessageStream::new(&bytes)
            .unwrap()
            .map(|message| message.unwrap())
            .collect();
        assert_eq!(read.len(), 2000);
        assert_eq!(read[0].channel.message_encoding, "cdr");
        assert_eq!(
            read[0].channel.schema.as_ref().unwrap().name,
            "sensor_msgs/msg/Imu"
        );
        // Sequence numbers must be gapless or a reader cannot tell a dropped
        // message from a slow one.
        for (index, message) in read.iter().enumerate() {
            assert_eq!(message.sequence as usize, index + 1);
        }
        std::fs::remove_dir_all(&directory).unwrap();
    }

    #[test]
    fn one_schema_is_shared_by_every_topic_that_uses_it() {
        let directory = scratch("shared_schema");
        let path = directory.join("out.mcap");
        let recorder = Recorder::start(&path, Compression::None).unwrap();
        recorder.offer("/livox/imu", an_imu(0));
        recorder.offer("/camera/imu", an_imu(1));
        recorder.finish().unwrap();

        let bytes = std::fs::read(&path).unwrap();
        let mut schema_ids = std::collections::HashSet::new();
        let mut topics = std::collections::HashSet::new();
        for message in mcap::MessageStream::new(&bytes).unwrap() {
            let message = message.unwrap();
            schema_ids.insert(message.channel.schema.as_ref().unwrap().id);
            topics.insert(message.channel.topic.clone());
        }
        assert_eq!(topics.len(), 2);
        assert_eq!(schema_ids.len(), 1);
        std::fs::remove_dir_all(&directory).unwrap();
    }

    /// A wedged disk must cost frames, not stall the sensor thread: `offer`
    /// still returns, and every shed message is still accounted for.
    #[test]
    fn a_full_queue_sheds_frames_and_accounts_for_every_one() {
        let directory = scratch("shedding");
        let path = directory.join("out.mcap");
        let mut recorder = Recorder::start(&path, Compression::None).unwrap();

        // Simply flooding cannot fill the queue: the writer drains small
        // messages faster than one thread produces them, so on a quick machine
        // the flood ends with nothing ever shed and the test proves nothing.
        // Swapping in a queue nobody is reading is what a wedged disk looks
        // like from `offer`, and it fills at a known depth.
        let (stalled_sender, stalled_receiver) = sync_channel(QUEUE_DEPTH);
        let writer_sender = recorder.sender.replace(stalled_sender).unwrap();

        let mut accepted = 0u64;
        let mut refused = 0u64;
        for index in 0..(QUEUE_DEPTH as u64 * 8) {
            if recorder.offer("/flood", an_imu(index)) {
                accepted += 1;
            } else {
                refused += 1;
            }
        }
        assert_eq!(accepted, QUEUE_DEPTH as u64);
        assert_eq!(refused, QUEUE_DEPTH as u64 * 7);

        // Unwedged: hand the backlog to the real writer so the accounting can be
        // checked against the file it produces.
        recorder.sender = Some(writer_sender);
        for sample in stalled_receiver.try_iter() {
            recorder.sender.as_ref().unwrap().send(sample).unwrap();
        }

        let status = recorder.finish().unwrap();
        assert_eq!(status.dropped, refused);
        assert_eq!(status.messages, accepted);
        assert_eq!(status.topics["/flood"].written, accepted);
        assert_eq!(status.topics["/flood"].dropped, refused);
        assert!((status.topics["/flood"].drop_fraction() - refused as f64 / (accepted + refused) as f64).abs() < 1e-12);

        let bytes = std::fs::read(&path).unwrap();
        let count = mcap::MessageStream::new(&bytes).unwrap().count() as u64;
        assert_eq!(count, accepted, "every accepted message must reach the file");
        std::fs::remove_dir_all(&directory).unwrap();
    }

    #[test]
    fn each_compression_setting_still_reads_back() {
        for compression in [Compression::None, Compression::Lz4, Compression::Zstd] {
            let directory = scratch("compressed");
            let path = directory.join("out.mcap");
            let recorder = Recorder::start(&path, compression).unwrap();
            for index in 0..64 {
                recorder.offer("/livox/imu", an_imu(index));
            }
            recorder.finish().unwrap();
            let bytes = std::fs::read(&path).unwrap();
            let count = mcap::MessageStream::new(&bytes).unwrap().count();
            assert_eq!(count, 64, "{compression:?}");
            std::fs::remove_dir_all(&directory).unwrap();
        }
    }

    #[test]
    fn resolve_refuses_to_escape_the_recording_directory() {
        let directory = Path::new("/tmp/recordings");
        assert!(resolve(directory, "../../etc/passwd.mcap").is_err());
        assert!(resolve(directory, "nested/run.mcap").is_err());
        assert!(resolve(directory, "run.txt").is_err());
        assert_eq!(
            resolve(directory, "run.mcap").unwrap(),
            Path::new("/tmp/recordings/run.mcap")
        );
    }

    #[test]
    fn a_recording_name_typed_without_an_extension_gets_one() {
        // Typing "morning_run" in the browser is the common case, and this
        // program is the thing that decides it writes mcap.
        let directory = Path::new("/tmp/recordings");
        assert_eq!(
            resolve(directory, "morning_run").unwrap(),
            Path::new("/tmp/recordings/morning_run.mcap")
        );
        // Escaping still has to be refused whether or not an extension is given.
        assert!(resolve(directory, "../escape").is_err());
        assert!(resolve(directory, "nested/run").is_err());
    }

    #[test]
    fn listing_reports_size_and_ignores_other_files() {
        let directory = scratch("listing");
        std::fs::write(directory.join("a.mcap"), [0u8; 12]).unwrap();
        std::fs::write(directory.join("notes.txt"), [0u8; 3]).unwrap();
        let files = list(&directory);
        assert_eq!(files.len(), 1);
        assert_eq!(files[0].name, "a.mcap");
        assert_eq!(files[0].bytes, 12);
        // A clock that has not synced yet reads 0, which would make every
        // recording look equally old and hide the ordering bug behind it.
        assert!(files[0].modified > 1_700_000_000);
        std::fs::remove_dir_all(&directory).unwrap();
    }

    /// The recording you just finished is the one you want to download, so it
    /// has to be the top row rather than wherever the directory happens to
    /// hand it back.
    #[test]
    fn listing_puts_the_newest_recording_first() {
        let directory = scratch("listing_order");
        std::fs::write(directory.join("older.mcap"), [0u8; 1]).unwrap();
        std::thread::sleep(Duration::from_millis(1100));
        std::fs::write(directory.join("newer.mcap"), [0u8; 1]).unwrap();
        let names: Vec<_> = list(&directory).into_iter().map(|file| file.name).collect();
        assert_eq!(names, ["newer.mcap", "older.mcap"]);
        std::fs::remove_dir_all(&directory).unwrap();
    }
}
