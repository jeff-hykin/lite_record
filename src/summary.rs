//! What `~/Commands/db_summary` prints for a `.db`, for an `.mcap`: per-topic
//! count, duration, rate, and how badly the stream stuttered.
//!
//! The timings come from the file's own message indexes, so a three-gigabyte
//! recording is summarised without decompressing a single payload. Only the
//! `/tf_static` chunk is ever decompressed, and only to name the frames.

use anyhow::{Context, Result};
use serde::Serialize;
use std::collections::BTreeMap;
use std::path::Path;

/// A Message record's fixed part: opcode, record length, channel id, sequence,
/// log time and publish time.
const MESSAGE_RECORD_BYTES: u64 = 1 + 8 + 2 + 4 + 8 + 8;

#[derive(Debug, Clone, Serialize, Default, PartialEq)]
pub struct Topic {
    pub topic: String,
    pub payload: String,
    pub count: u64,
    pub duration_seconds: f64,
    pub hz: f64,
    /// The 99th-percentile gap between consecutive messages. Next to
    /// `worst_gap_seconds` it separates a stream that stutters constantly from
    /// one that stalled once.
    pub p99_gap_seconds: f64,
    pub p99_ratio: f64,
    pub worst_gap_seconds: f64,
    /// The worst gap as a multiple of this stream's own average spacing, which
    /// is the only way to compare a stall on a 200 Hz stream against a 1 Hz one.
    pub worst_gap_ratio: f64,
    pub bytes: u64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Frame {
    pub parent: String,
    pub child: String,
}

#[derive(Debug, Clone, Serialize, Default, PartialEq)]
pub struct Summary {
    pub name: String,
    pub file_bytes: u64,
    pub start_unix_seconds: f64,
    pub duration_seconds: f64,
    pub message_count: u64,
    pub topics: Vec<Topic>,
    pub frames: Vec<Frame>,
    /// False when the file carried no usable index and every message had to be
    /// read. The numbers are the same either way; the wait is not.
    pub indexed: bool,
}

#[derive(Default)]
struct Track {
    payload: String,
    times: Vec<u64>,
    bytes: u64,
}

pub fn summarise(path: &Path) -> Result<Summary> {
    let file =
        std::fs::File::open(path).with_context(|| format!("could not open {}", path.display()))?;
    let file_bytes = file.metadata()?.len();
    let mapped = unsafe { memmap2::Mmap::map(&file) }
        .with_context(|| format!("could not map {}", path.display()))?;

    let indexed = mcap::Summary::read(&mapped)
        .ok()
        .flatten()
        .filter(|summary| !summary.chunk_indexes.is_empty());
    let (tracks, frames, indexed) = match indexed.as_ref().and_then(|summary| {
        from_indexes(&mapped, summary).map(|tracks| (tracks, frames_from_index(&mapped, summary)))
    }) {
        Some((tracks, frames)) => (tracks, frames, true),
        None => {
            let (tracks, frames) = from_messages(&mapped)?;
            (tracks, frames, false)
        }
    };

    let first = tracks
        .values()
        .filter_map(|track| track.times.iter().min())
        .min()
        .copied();
    let last = tracks
        .values()
        .filter_map(|track| track.times.iter().max())
        .max()
        .copied();
    let topics: Vec<Topic> = tracks
        .into_iter()
        .map(|(topic, track)| summarise_topic(topic, track))
        .collect();

    Ok(Summary {
        name: path
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned(),
        file_bytes,
        start_unix_seconds: first.map(|first| first as f64 / 1e9).unwrap_or(0.0),
        duration_seconds: match (first, last) {
            (Some(first), Some(last)) => (last - first) as f64 / 1e9,
            _ => 0.0,
        },
        message_count: topics.iter().map(|topic| topic.count).sum(),
        topics,
        frames,
        indexed,
    })
}

fn summarise_topic(topic: String, track: Track) -> Topic {
    let mut times = track.times;
    times.sort_unstable();
    let count = times.len() as u64;
    let duration_seconds = match (times.first(), times.last()) {
        (Some(first), Some(last)) => (last - first) as f64 / 1e9,
        _ => 0.0,
    };
    let hz = if duration_seconds > 0.0 {
        count as f64 / duration_seconds
    } else {
        0.0
    };
    let mean_gap = if count > 1 && duration_seconds > 0.0 {
        duration_seconds / (count - 1) as f64
    } else {
        0.0
    };

    let mut gaps: Vec<f64> = times
        .windows(2)
        .map(|pair| (pair[1] - pair[0]) as f64 / 1e9)
        .collect();
    gaps.sort_by(f64::total_cmp);
    let worst_gap_seconds = gaps.last().copied().unwrap_or(0.0);
    // Matches db_summary: the value at ceil(n * 0.99), so one stall in a hundred
    // shows up in `gap` but not here.
    let p99_gap_seconds = if gaps.is_empty() {
        0.0
    } else {
        let index = ((gaps.len() as f64 * 0.99).ceil() as usize)
            .saturating_sub(1)
            .min(gaps.len() - 1);
        gaps[index]
    };
    let ratio = |gap: f64| if mean_gap > 0.0 { gap / mean_gap } else { 0.0 };

    Topic {
        topic,
        payload: track.payload,
        count,
        duration_seconds,
        hz,
        p99_gap_seconds,
        p99_ratio: ratio(p99_gap_seconds),
        worst_gap_seconds,
        worst_gap_ratio: ratio(worst_gap_seconds),
        bytes: track.bytes,
    }
}

/// Per-message timestamps straight out of the message index records. Returns
/// `None` the moment any chunk lacks an index, so the caller falls back to a
/// full read rather than reporting half a recording.
fn from_indexes(mapped: &[u8], summary: &mcap::Summary) -> Option<BTreeMap<String, Track>> {
    let mut tracks: BTreeMap<String, Track> = BTreeMap::new();
    for chunk in &summary.chunk_indexes {
        let indexes = summary.read_message_indexes(mapped, chunk).ok()?;
        // Records sit back to back in the decompressed chunk, so the distance to
        // the next record is this record's size, whatever channel it belongs to.
        let mut spans: Vec<(u64, String)> = Vec::new();
        for (channel, entries) in &indexes {
            let track = tracks.entry(channel.topic.clone()).or_default();
            if track.payload.is_empty() {
                track.payload = payload_name(channel);
            }
            for entry in entries {
                track.times.push(entry.log_time);
                spans.push((entry.offset, channel.topic.clone()));
            }
        }
        spans.sort_unstable();
        for (position, (offset, topic)) in spans.iter().enumerate() {
            let next = spans
                .get(position + 1)
                .map(|(offset, _)| *offset)
                .unwrap_or(chunk.uncompressed_size);
            if let Some(track) = tracks.get_mut(topic) {
                track.bytes += next.saturating_sub(*offset);
            }
        }
    }
    (!tracks.is_empty()).then_some(tracks)
}

/// Every message read and its payload decompressed. Only used for a file with
/// no index, which in practice means one that was cut short.
fn from_messages(mapped: &[u8]) -> Result<(BTreeMap<String, Track>, Vec<Frame>)> {
    let mut tracks: BTreeMap<String, Track> = BTreeMap::new();
    let mut frames = Vec::new();
    for message in mcap::MessageStream::new(mapped)? {
        let message = message?;
        let track = tracks
            .entry(message.channel.topic.clone())
            .or_insert_with(|| Track {
                payload: payload_name(&message.channel),
                ..Track::default()
            });
        track.times.push(message.log_time);
        // The index path measures each record's span in the chunk, so it counts
        // the record header too. Adding it here keeps the column meaning the same
        // thing whichever path produced it.
        track.bytes += message.data.len() as u64 + MESSAGE_RECORD_BYTES;
        if frames.is_empty() && is_tf_static(&message.channel) {
            frames = decode_frames(&message.data);
        }
    }
    Ok((tracks, frames))
}

fn payload_name(channel: &mcap::Channel) -> String {
    channel
        .schema
        .as_ref()
        .map(|schema| schema.name.clone())
        .unwrap_or_else(|| channel.message_encoding.clone())
}

fn is_tf_static(channel: &mcap::Channel) -> bool {
    channel.topic == "/tf_static" && channel.message_encoding == "cdr"
}

/// Decompresses only the chunk holding `/tf_static` — it is written first, so
/// this is the first chunk and costs one chunk's worth of work.
fn frames_from_index(mapped: &[u8], summary: &mcap::Summary) -> Vec<Frame> {
    let Some((id, _)) = summary
        .channels
        .iter()
        .find(|(_, channel)| is_tf_static(channel))
    else {
        return Vec::new();
    };
    for chunk in &summary.chunk_indexes {
        if !chunk.message_index_offsets.contains_key(id) {
            continue;
        }
        let Ok(messages) = summary.stream_chunk(mapped, chunk) else {
            continue;
        };
        for message in messages.flatten() {
            if message.channel.topic == "/tf_static" {
                return decode_frames(&message.data);
            }
        }
    }
    Vec::new()
}

/// A `tf2_msgs/msg/TFMessage` is a count followed by that many transforms; only
/// the two frame names of each are wanted here.
fn decode_frames(data: &[u8]) -> Vec<Frame> {
    if data.len() < 8 || data[..4] != [0x00, 0x01, 0x00, 0x00] {
        return Vec::new();
    }
    let count = u32::from_le_bytes(data[4..8].try_into().unwrap()) as usize;
    // A rig with more joints than this is a corrupt length field, not a robot.
    if count > 4096 {
        return Vec::new();
    }
    let mut reader = crate::cdr::CdrReader::new(data);
    let _ = reader.u32();
    let mut frames = Vec::with_capacity(count);
    for _ in 0..count {
        let Some(header) = reader.try_header() else {
            break;
        };
        let Some(child) = reader.try_string() else {
            break;
        };
        // Not wanted, but the next transform starts after them.
        if reader.try_f64_array::<3>().is_none() || reader.try_f64_array::<4>().is_none() {
            break;
        }
        frames.push(Frame {
            parent: header.frame_id,
            child,
        });
    }
    frames
}

#[cfg(test)]
mod tests {
    use super::*;

    fn track(times: &[u64]) -> Track {
        Track {
            payload: "test/Msg".into(),
            times: times.to_vec(),
            bytes: 0,
        }
    }

    #[test]
    fn a_steady_stream_reports_its_rate_and_no_stall() {
        let times: Vec<u64> = (0..101).map(|step| step * 100_000_000).collect();
        let summary = summarise_topic("/camera".into(), track(&times));
        assert_eq!(summary.count, 101);
        assert!((summary.duration_seconds - 10.0).abs() < 1e-9);
        assert!((summary.hz - 10.1).abs() < 1e-9);
        assert!((summary.worst_gap_ratio - 1.0).abs() < 1e-6);
    }

    #[test]
    fn one_stall_shows_in_the_worst_gap_but_not_in_the_p99() {
        let mut times: Vec<u64> = (0..200).map(|step| step * 10_000_000).collect();
        let last = *times.last().unwrap();
        times.push(last + 2_000_000_000);
        let summary = summarise_topic("/lidar".into(), track(&times));
        assert!(summary.worst_gap_ratio > 100.0, "{summary:?}");
        assert!(summary.p99_ratio < 2.0, "{summary:?}");
    }

    #[test]
    fn a_single_message_has_no_rate_and_no_gap() {
        let summary = summarise_topic("/tf_static".into(), track(&[42]));
        assert_eq!(summary.count, 1);
        assert_eq!(summary.duration_seconds, 0.0);
        assert_eq!(summary.hz, 0.0);
        assert_eq!(summary.worst_gap_seconds, 0.0);
        assert_eq!(summary.worst_gap_ratio, 0.0);
    }

    #[test]
    fn timestamps_out_of_order_still_measure_the_span() {
        let summary = summarise_topic("/imu".into(), track(&[300, 100, 200]));
        assert_eq!(summary.count, 3);
        assert!((summary.duration_seconds - 200e-9).abs() < 1e-15);
    }

    #[test]
    fn a_tf_message_names_every_parent_and_child() {
        let encoded = crate::cdr::tf_message(&[
            crate::msgs::TransformStamped {
                header: crate::msgs::Header::new(0, "base_link"),
                child_frame_id: "camera_link".into(),
                translation: [0.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            crate::msgs::TransformStamped {
                header: crate::msgs::Header::new(0, "camera_link"),
                child_frame_id: "camera_depth_optical_frame".into(),
                translation: [0.0, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        ]);
        let frames = decode_frames(&encoded.data);
        assert_eq!(
            frames,
            vec![
                Frame {
                    parent: "base_link".into(),
                    child: "camera_link".into()
                },
                Frame {
                    parent: "camera_link".into(),
                    child: "camera_depth_optical_frame".into()
                },
            ]
        );
    }

    #[test]
    fn a_truncated_tf_message_names_the_frames_it_can_and_stops() {
        let encoded = crate::cdr::tf_message(&[crate::msgs::TransformStamped {
            header: crate::msgs::Header::new(0, "base_link"),
            child_frame_id: "camera_link".into(),
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }]);
        let cut = &encoded.data[..encoded.data.len() - 20];
        assert!(decode_frames(cut).is_empty());
    }

    #[test]
    fn something_that_is_not_a_tf_message_names_no_frames() {
        assert!(decode_frames(&[]).is_empty());
        assert!(decode_frames(&[0xFF; 64]).is_empty());
    }
}
