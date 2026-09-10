//! Reading one topic back out of a finished mcap.
//!
//! A recording is mostly lidar and depth, and a tool that wants the colour
//! stream or the odometry should not have to decompress all of it. The summary
//! at the end of the file says which channels each chunk holds, so a chunk with
//! nothing wanted in it is never touched. A file without a summary — a
//! truncated one — still works, just by walking straight through.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, Result};
use mcap::{Channel, Message, Summary};

pub struct Recording {
    pub path: PathBuf,
    mapped: memmap2::Mmap,
    summary: Option<Summary>,
}

/// Messages of one channel, in file order.
pub type Messages<'a> = Box<dyn Iterator<Item = Result<Message<'a>>> + 'a>;

impl Recording {
    pub fn open(path: &Path) -> Result<Self> {
        let file = std::fs::File::open(path)
            .with_context(|| format!("could not open {}", path.display()))?;
        let mapped = unsafe { memmap2::Mmap::map(&file) }
            .with_context(|| format!("could not map {}", path.display()))?;
        let summary = Summary::read(&mapped).ok().flatten();
        Ok(Recording {
            path: path.to_path_buf(),
            mapped,
            summary,
        })
    }

    /// Every channel, sorted by topic. Read off the summary when there is one,
    /// otherwise collected from the messages themselves.
    pub fn channels(&self) -> Result<Vec<Arc<Channel<'_>>>> {
        let mut channels: Vec<Arc<Channel<'_>>> = match &self.summary {
            Some(summary) => summary.channels.values().cloned().collect(),
            None => {
                let mut seen = std::collections::BTreeMap::new();
                for message in mcap::MessageStream::new(&self.mapped)? {
                    let message = message?;
                    seen.entry(message.channel.id)
                        .or_insert_with(|| Arc::clone(&message.channel));
                }
                seen.into_values().collect()
            }
        };
        channels.sort_by(|left, right| left.topic.cmp(&right.topic));
        Ok(channels)
    }

    /// Finds a topic whether or not the caller typed its leading slash. The
    /// error lists what the file does hold, since a topic name is the usual
    /// thing to get wrong.
    pub fn channel(&self, topic: &str) -> Result<Arc<Channel<'_>>> {
        let channels = self.channels()?;
        let wanted = topic.trim_start_matches('/');
        channels
            .iter()
            .find(|channel| channel.topic.trim_start_matches('/') == wanted)
            .cloned()
            .with_context(|| {
                let topics: Vec<&str> = channels.iter().map(|channel| channel.topic.as_str()).collect();
                format!(
                    "no topic {topic} in {}; have: {}",
                    self.path.display(),
                    topics.join(", ")
                )
            })
    }

    /// How many messages the channel holds, from the statistics record, so it
    /// costs nothing to answer.
    pub fn message_count(&self, channel_id: u16) -> Option<u64> {
        self.summary
            .as_ref()?
            .stats
            .as_ref()?
            .channel_message_counts
            .get(&channel_id)
            .copied()
    }

    /// Every log time on the channel, ascending. Comes from the message indexes
    /// that follow each chunk, so the chunks themselves stay compressed.
    pub fn stamps(&self, channel_id: u16) -> Result<Vec<u64>> {
        let mut stamps = Vec::new();
        match self.summary.as_ref().filter(|summary| !summary.chunk_indexes.is_empty()) {
            Some(summary) => {
                for chunk in &summary.chunk_indexes {
                    if !chunk.message_index_offsets.contains_key(&channel_id) {
                        continue;
                    }
                    let indexes = summary.read_message_indexes(&self.mapped, chunk)?;
                    for (channel, entries) in indexes {
                        if channel.id == channel_id {
                            stamps.extend(entries.iter().map(|entry| entry.log_time));
                        }
                    }
                }
            }
            None => {
                for message in mcap::MessageStream::new(&self.mapped)? {
                    let message = message?;
                    if message.channel.id == channel_id {
                        stamps.push(message.log_time);
                    }
                }
            }
        }
        stamps.sort_unstable();
        Ok(stamps)
    }

    /// The channel's messages in file order — which is not time order: the
    /// recorder may flush one stream's chunks long after another's. Callers
    /// that care sort by stamp themselves. `window` limits by log time and lets
    /// whole chunks outside it be skipped unread.
    pub fn messages(&self, channel_id: u16, window: Option<(u64, u64)>) -> Result<Messages<'_>> {
        let inside = move |log_time: u64| window.is_none_or(|(low, high)| (low..=high).contains(&log_time));
        match self.summary.as_ref().filter(|summary| !summary.chunk_indexes.is_empty()) {
            Some(summary) => {
                let mut chunks: Vec<_> = summary
                    .chunk_indexes
                    .iter()
                    .filter(|chunk| {
                        // An empty offset map means the writer emitted no message
                        // indexes, which says nothing about what the chunk holds.
                        chunk.message_index_offsets.is_empty()
                            || chunk.message_index_offsets.contains_key(&channel_id)
                    })
                    .filter(|chunk| {
                        window.is_none_or(|(low, high)| {
                            chunk.message_end_time >= low && chunk.message_start_time <= high
                        })
                    })
                    .collect();
                chunks.sort_by_key(|chunk| chunk.chunk_start_offset);
                let mapped: &[u8] = &self.mapped;
                let stream = chunks.into_iter().flat_map(move |chunk| {
                    match summary.stream_chunk(mapped, chunk) {
                        Ok(messages) => Box::new(messages.map(|message| message.map_err(anyhow::Error::from)))
                            as Box<dyn Iterator<Item = Result<Message<'_>>>>,
                        Err(error) => Box::new(std::iter::once(Err(anyhow::Error::from(error)))),
                    }
                });
                Ok(Box::new(stream.filter(move |message| match message {
                    Ok(message) => message.channel.id == channel_id && inside(message.log_time),
                    Err(_) => true,
                })))
            }
            None => {
                let stream = mcap::MessageStream::new(&self.mapped)?;
                Ok(Box::new(
                    stream
                        .map(|message| message.map_err(anyhow::Error::from))
                        .filter(move |message| match message {
                            Ok(message) => message.channel.id == channel_id && inside(message.log_time),
                            Err(_) => true,
                        }),
                ))
            }
        }
    }
}

/// The schema name a channel was recorded under, or "" for one without.
pub fn schema_name<'a>(channel: &'a Channel<'_>) -> &'a str {
    channel.schema.as_ref().map_or("", |schema| schema.name.as_str())
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::*;
    use crate::cdr::Encoded;
    use std::collections::HashMap;

    /// A message bound for a test recording: its topic, payload and log time.
    pub struct Stamped {
        pub topic: String,
        pub encoded: Encoded,
        pub log_time: u64,
    }

    pub fn stamped(topic: &str, encoded: Encoded, log_time: u64) -> Stamped {
        Stamped {
            topic: topic.to_string(),
            encoded,
            log_time,
        }
    }

    /// Writes each group as its own chunk, in the order given, so a test can
    /// put a stream's chunks after everything else the way the recorder does.
    pub fn write_recording(path: &Path, groups: &[Vec<Stamped>]) {
        let file = std::fs::File::create(path).unwrap();
        let mut writer = mcap::WriteOptions::new()
            .compression(Some(mcap::Compression::Zstd))
            .profile("ros2")
            .create(std::io::BufWriter::new(file))
            .unwrap();
        let mut channels: HashMap<String, (u16, u32)> = HashMap::new();
        for group in groups {
            for message in group {
                let entry = channels.entry(message.topic.clone()).or_insert_with(|| {
                    let schema = writer
                        .add_schema(
                            message.encoded.schema_name,
                            "ros2msg",
                            message.encoded.schema_text.as_bytes(),
                        )
                        .unwrap();
                    (writer.add_channel(schema, &message.topic, "cdr", &Default::default()).unwrap(), 0)
                });
                entry.1 += 1;
                writer
                    .write_to_known_channel(
                        &mcap::records::MessageHeader {
                            channel_id: entry.0,
                            sequence: entry.1,
                            log_time: message.log_time,
                            publish_time: message.log_time,
                        },
                        &message.encoded.data,
                    )
                    .unwrap();
            }
            writer.flush().unwrap();
        }
        writer.finish().unwrap();
    }

    pub fn scratch(label: &str) -> PathBuf {
        let directory = std::env::temp_dir().join(format!("lite_record_{label}_{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        directory
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::*;
    use super::*;
    use crate::cdr::{self, Encoded};
    use crate::msgs::{Header, Imu};

    fn imu_at(stamp: u64) -> Encoded {
        cdr::imu(&Imu::unoriented(Header::new(stamp, "imu"), [0.0; 3], [0.0; 3]))
    }

    #[test]
    fn a_topic_is_found_with_or_without_its_leading_slash() {
        let path = scratch("topics_slash").join("slash.mcap");
        write_recording(&path, &[vec![stamped("/imu", imu_at(1), 1)]]);
        let recording = Recording::open(&path).unwrap();
        assert_eq!(recording.channel("imu").unwrap().topic, "/imu");
        assert_eq!(recording.channel("/imu").unwrap().topic, "/imu");
        let error = recording.channel("/nope").unwrap_err().to_string();
        assert!(error.contains("no topic /nope") && error.contains("/imu"), "{error}");
    }

    #[test]
    fn stamps_come_back_sorted_even_when_the_chunks_are_not() {
        let path = scratch("topics_order").join("order.mcap");
        write_recording(
            &path,
            &[
                vec![stamped("/imu", imu_at(30), 30), stamped("/imu", imu_at(40), 40)],
                vec![stamped("/other", imu_at(5), 5)],
                vec![stamped("/imu", imu_at(10), 10), stamped("/imu", imu_at(20), 20)],
            ],
        );
        let recording = Recording::open(&path).unwrap();
        let channel = recording.channel("imu").unwrap();
        assert_eq!(recording.stamps(channel.id).unwrap(), vec![10, 20, 30, 40]);
        assert_eq!(recording.message_count(channel.id), Some(4));
        let file_order: Vec<u64> = recording
            .messages(channel.id, None)
            .unwrap()
            .map(|message| message.unwrap().log_time)
            .collect();
        assert_eq!(file_order, vec![30, 40, 10, 20]);
    }

    #[test]
    fn a_window_keeps_only_the_messages_inside_it() {
        let path = scratch("topics_window").join("window.mcap");
        write_recording(
            &path,
            &[
                vec![stamped("/imu", imu_at(10), 10), stamped("/imu", imu_at(20), 20)],
                vec![stamped("/imu", imu_at(30), 30), stamped("/imu", imu_at(40), 40)],
            ],
        );
        let recording = Recording::open(&path).unwrap();
        let channel = recording.channel("imu").unwrap();
        let inside: Vec<u64> = recording
            .messages(channel.id, Some((20, 30)))
            .unwrap()
            .map(|message| message.unwrap().log_time)
            .collect();
        assert_eq!(inside, vec![20, 30]);
    }
}
