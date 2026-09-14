//! Reading a finished recording in log-time order, whatever order it is stored in.
//!
//! Two things go wrong with the obvious `mcap::MessageStream`, and this file
//! exists for both.
//!
//! **It is linear**, so it learns channels as it walks past their records and a
//! message naming a channel declared later in the file stops it dead with
//! `Message N referenced unknown channel M`. Nothing in the format forbids that
//! ordering — the summary is what a reader is meant to resolve channels from.
//! `~/datasets/lite_recorder/grocery.mcap` is such a file: channel 11
//! (`/realsense/infrared_left/compressed`) is declared at data-section record
//! 558,374 and first used at record 50. It opens fine in Foxglove and in
//! anything index-driven, and every linear reader that has met it has died.
//!
//! **And file order is not log order.** A tool that rewrites chunks in place
//! puts the rewritten ones where the old summary was, at the end. In that same
//! recording `mcap_edit --drop-tf-edge` moved every chunk that mentioned `/tf`,
//! and those chunks carried lidar and IMU too: walked in file order, the lidar
//! runs to t+802 s and then jumps back to t+0 for another 879 scans. Feeding
//! that to a state estimator does not fail, which is the dangerous part — it
//! rejects the scans that teleported and returns a plausible, wrong trajectory
//! (515 m of path and 250 rejections, against 367 m and 0 on the same recording
//! before the rewrite).
//!
//! So: merge the chunks by log time, activating each only once a message could
//! come out of it, and keep the linear walk for a file with no summary — one the
//! recorder was killed part way through, which is the case the straight-through
//! paths exist for.

use std::ops::ControlFlow;

use anyhow::Result;

/// Every channel id carrying `topic`.
///
/// A topic is not one channel. A recording that has been rewritten in place
/// carries a fresh channel record for each rewritten region, so `grocery.mcap`
/// has two ids for `/livox/lidar`, five for `/tf` and four for
/// `/pointlio_odometry`, with the messages split between them and some ids
/// left holding none at all. `channels.values().find(|c| c.topic == topic)`
/// therefore picks an arbitrary one, and picking the empty one reads as "this
/// topic has no messages".
pub fn channel_ids(summary: &mcap::Summary, topic: &str) -> std::collections::BTreeSet<u16> {
    summary
        .channels
        .values()
        .filter(|channel| channel.topic == topic)
        .map(|channel| channel.id)
        .collect()
}

/// How many messages a topic holds, across every channel that carries it.
pub fn message_count(summary: &mcap::Summary, topic: &str) -> u64 {
    let Some(stats) = summary.stats.as_ref() else {
        return 0;
    };
    channel_ids(summary, topic)
        .iter()
        .filter_map(|id| stats.channel_message_counts.get(id).copied())
        .sum()
}

/// Hands every message to `handle`, in log-time order. Returning
/// `ControlFlow::Break` stops the walk without an error.
pub fn for_each_message(
    mapped: &[u8],
    mut handle: impl FnMut(&mcap::Message<'_>) -> Result<ControlFlow<()>>,
) -> Result<()> {
    let indexed = match mcap::Summary::read(mapped) {
        Ok(Some(summary)) if !summary.chunk_indexes.is_empty() => Some(summary),
        // No summary, or messages outside chunks: there is no index to merge.
        _ => None,
    };
    let Some(summary) = indexed else {
        for message in mcap::MessageStream::new(mapped)? {
            if handle(&message?)?.is_break() {
                return Ok(());
            }
        }
        return Ok(());
    };

    let mut chunks = summary.chunk_indexes.clone();
    chunks.sort_by_key(|chunk| (chunk.message_start_time, chunk.chunk_start_offset));

    // Each open chunk, as the message it is holding plus the rest of it. Only
    // chunks whose time ranges overlap are ever open at once, which for a
    // recorder's own writing is one.
    type Rest<'a> = Box<dyn Iterator<Item = mcap::McapResult<mcap::Message<'a>>> + 'a>;
    let mut open: Vec<(mcap::Message<'_>, Rest<'_>)> = Vec::new();
    let mut unopened = 0;

    loop {
        // Open every chunk that could hold a message at or before the earliest
        // one already in hand — otherwise a later-starting chunk that begins
        // before that message would be skipped past.
        loop {
            let earliest = open.iter().map(|(message, _)| message.log_time).min();
            let should_open = match (unopened < chunks.len(), earliest) {
                (false, _) => false,
                (true, None) => true,
                (true, Some(time)) => chunks[unopened].message_start_time <= time,
            };
            if !should_open {
                break;
            }
            let mut rest = summary.stream_chunk(mapped, &chunks[unopened])?;
            unopened += 1;
            if let Some(first) = rest.next() {
                open.push((first?, Box::new(rest)));
            }
        }
        let Some(next) = open
            .iter()
            .enumerate()
            .min_by_key(|(_, (message, _))| message.log_time)
            .map(|(index, _)| index)
        else {
            return Ok(());
        };
        // Advance that chunk before handing the message over, so `handle` is not
        // holding a borrow of the slot being refilled.
        let following = open[next].1.next().transpose()?;
        let message = match following {
            Some(following) => std::mem::replace(&mut open[next].0, following),
            None => open.swap_remove(next).0,
        };
        if handle(&message)?.is_break() {
            return Ok(());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    /// `stamps` become one chunk each, written in the order given — so passing
    /// a descending list writes a file whose chunks are out of log order, the
    /// shape a chunk-rewriting tool leaves behind.
    fn recording(stamps: &[u64]) -> Vec<u8> {
        let mut buffer = Vec::new();
        {
            let mut writer = mcap::WriteOptions::new()
                .compression(None)
                .chunk_size(Some(1))
                .create(Cursor::new(&mut buffer))
                .unwrap();
            let schema = writer.add_schema("std_msgs/msg/Empty", "ros2msg", b"").unwrap();
            let channel = writer
                .add_channel(schema, "/a", "cdr", &std::collections::BTreeMap::new())
                .unwrap();
            for (index, stamp) in stamps.iter().enumerate() {
                writer
                    .write_to_known_channel(
                        &mcap::records::MessageHeader {
                            channel_id: channel,
                            sequence: index as u32,
                            log_time: *stamp,
                            publish_time: *stamp,
                        },
                        &[0u8; 8],
                    )
                    .unwrap();
            }
            writer.finish().unwrap();
        }
        buffer
    }

    fn stamps_from(bytes: &[u8]) -> Vec<u64> {
        let mut seen = Vec::new();
        for_each_message(bytes, |message| {
            seen.push(message.log_time);
            Ok(ControlFlow::Continue(()))
        })
        .unwrap();
        seen
    }

    #[test]
    fn messages_come_back_in_log_order_when_the_file_already_is() {
        assert_eq!(stamps_from(&recording(&[10, 20, 30, 40])), vec![10, 20, 30, 40]);
    }

    /// The grocery.mcap shape: chunks written out of order by a rewrite.
    #[test]
    fn chunks_stored_out_of_order_are_merged_back_into_log_order() {
        assert_eq!(stamps_from(&recording(&[30, 40, 10, 20])), vec![10, 20, 30, 40]);
        assert_eq!(stamps_from(&recording(&[40, 30, 20, 10])), vec![10, 20, 30, 40]);
        // The real one: a long run in order, then a tail belonging at the front.
        let mut stamps: Vec<u64> = (100..160).collect();
        stamps.extend(1..40);
        let mut expected = stamps.clone();
        expected.sort_unstable();
        assert_eq!(stamps_from(&recording(&stamps)), expected);
    }

    /// A chunk spanning the whole recording alongside the narrow ones, which is
    /// what an appended `/tf` chunk looks like. Written last, it must still
    /// interleave rather than land at the end.
    #[test]
    fn a_chunk_that_spans_the_others_interleaves_with_them() {
        let mut buffer = Vec::new();
        {
            let mut writer = mcap::WriteOptions::new()
                .compression(None)
                .chunk_size(Some(1 << 20))
                .create(Cursor::new(&mut buffer))
                .unwrap();
            let schema = writer.add_schema("std_msgs/msg/Empty", "ros2msg", b"").unwrap();
            let write = |writer: &mut mcap::Writer<Cursor<&mut Vec<u8>>>, topic: &str, stamps: &[u64]| {
                let channel = writer
                    .add_channel(schema, topic, "cdr", &std::collections::BTreeMap::new())
                    .unwrap();
                for stamp in stamps {
                    writer
                        .write_to_known_channel(
                            &mcap::records::MessageHeader {
                                channel_id: channel,
                                sequence: 0,
                                log_time: *stamp,
                                publish_time: *stamp,
                            },
                            &[0u8; 8],
                        )
                        .unwrap();
                }
            };
            write(&mut writer, "/narrow", &[10, 20, 30, 40, 50, 60]);
            writer.flush().unwrap();
            write(&mut writer, "/wide", &[15, 45]);
            writer.finish().unwrap();
        }
        assert_eq!(stamps_from(&buffer), vec![10, 15, 20, 30, 40, 45, 50, 60]);
    }

    #[test]
    fn breaking_stops_the_walk_without_an_error() {
        let bytes = recording(&[10, 20, 30, 40]);
        let mut seen = 0;
        for_each_message(&bytes, |_| {
            seen += 1;
            Ok(if seen == 2 { ControlFlow::Break(()) } else { ControlFlow::Continue(()) })
        })
        .unwrap();
        assert_eq!(seen, 2);
    }

    /// The fallback: a file cut off before its summary still reads.
    #[test]
    fn a_recording_with_no_summary_is_read_straight_through() {
        let bytes = recording(&[10, 20, 30, 40]);
        let mut seen = 0;
        let _ = for_each_message(&bytes[..bytes.len() / 2], |_| {
            seen += 1;
            Ok(ControlFlow::Continue(()))
        });
        assert!(seen > 0, "the linear fallback read nothing");
    }
}
