//! Appending messages to a finished mcap without rewriting it.
//!
//! An mcap ends with a summary section (schemas, channels, chunk index,
//! statistics) and a footer that points at it. Nothing in the data section
//! refers to file offsets after itself, so a recording can grow by cutting the
//! summary off, writing new chunks where it was, and putting a summary back
//! that covers the old chunks and the new ones. A 63 GB recording gains an
//! odometry topic for the cost of writing the odometry, not the cost of a copy.
//!
//! The chunks added here land at the end of the file, so their messages are
//! not in log-time order relative to the rest. Indexed readers (Foxglove, the
//! `mcap` CLI, this crate's own `Summary`) sort by the chunk index and never
//! notice; a linear walk sees them last. Anything here that consumes time
//! series therefore sorts by stamp rather than trusting file order.

use std::collections::{BTreeMap, HashMap};
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};
use mcap::records;

use crate::msgs::NANOS_PER_SEC;

/// Magic bytes at both ends of every mcap.
const MAGIC: [u8; 8] = [0x89, b'M', b'C', b'A', b'P', 0x30, b'\r', b'\n'];

mod op {
    pub const HEADER: u8 = 0x01;
    pub const FOOTER: u8 = 0x02;
    pub const SCHEMA: u8 = 0x03;
    pub const CHANNEL: u8 = 0x04;
    pub const MESSAGE: u8 = 0x05;
    pub const CHUNK: u8 = 0x06;
    pub const MESSAGE_INDEX: u8 = 0x07;
    pub const CHUNK_INDEX: u8 = 0x08;
    pub const ATTACHMENT_INDEX: u8 = 0x0A;
    pub const STATISTICS: u8 = 0x0B;
    pub const METADATA_INDEX: u8 = 0x0D;
    pub const SUMMARY_OFFSET: u8 = 0x0E;
    pub const DATA_END: u8 = 0x0F;
}

/// Uncompressed bytes per appended chunk. Small enough that a reader seeking
/// to one message decompresses little, large enough that zstd sees the
/// repetition in a stream of near-identical transforms.
const CHUNK_TARGET_BYTES: usize = 4 << 20;

/// Log time a single appended chunk may span.
///
/// Size alone is the wrong limit for what this appends. Transforms and odometry
/// are tiny, so 4 MB of them is the whole recording in one chunk — and a chunk
/// that spans the recording overlaps every other chunk in the file. A reader
/// that wants messages in log order can then no longer get there by sorting the
/// chunk index; it needs a merge. On the 58 GB grocery recording the appended
/// chunks left 2153 overlapping pairs. Capping the span keeps appended chunks
/// disjoint in time and roughly the width of the recorder's own, so sorting the
/// index is enough again.
const CHUNK_TARGET_NANOS: u64 = 5 * NANOS_PER_SEC;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Schema {
    pub name: String,
    pub encoding: String,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Channel {
    pub schema_id: u16,
    pub topic: String,
    pub message_encoding: String,
    pub metadata: BTreeMap<String, String>,
}

struct Pending {
    channel_id: u16,
    sequence: u32,
    log_time: u64,
    publish_time: u64,
    data: Vec<u8>,
}

/// A finished recording, open for adding messages to.
pub struct Appender {
    path: PathBuf,
    file: BufWriter<File>,
    schemas: BTreeMap<u16, Schema>,
    channels: BTreeMap<u16, Channel>,
    chunk_indexes: Vec<records::ChunkIndex>,
    attachment_indexes: Vec<records::AttachmentIndex>,
    metadata_indexes: Vec<records::MetadataIndex>,
    statistics: Option<records::Statistics>,
    sequences: HashMap<u16, u32>,
    pending: Vec<Pending>,
    appended: u64,
    appended_chunks: usize,
}

impl Appender {
    /// Opens `path`, reads its summary, and truncates it back to the end of its
    /// data section. From this point until [`Appender::finish`] the file has no
    /// summary; a crash in between leaves a recording that `mcap_recover` can
    /// re-index but that indexed readers refuse.
    pub fn open(path: &Path) -> Result<Self> {
        let existing = File::open(path).with_context(|| format!("could not open {}", path.display()))?;
        let mapped = unsafe { memmap2::Mmap::map(&existing)? };
        let (data_end_offset, summary) = locate_summary(&mapped)
            .with_context(|| format!("{} is not a finished, indexed mcap", path.display()))?;

        let mut schemas = BTreeMap::new();
        for schema in summary.schemas.values() {
            schemas.insert(
                schema.id,
                Schema {
                    name: schema.name.clone(),
                    encoding: schema.encoding.clone(),
                    data: schema.data.to_vec(),
                },
            );
        }
        let mut channels = BTreeMap::new();
        for channel in summary.channels.values() {
            channels.insert(
                channel.id,
                Channel {
                    schema_id: channel.schema.as_ref().map_or(0, |schema| schema.id),
                    topic: channel.topic.clone(),
                    message_encoding: channel.message_encoding.clone(),
                    metadata: channel.metadata.clone(),
                },
            );
        }
        let sequences = summary
            .stats
            .as_ref()
            .map(|stats| {
                stats
                    .channel_message_counts
                    .iter()
                    .map(|(channel, count)| (*channel, *count as u32))
                    .collect()
            })
            .unwrap_or_default();
        drop(mapped);

        let file = OpenOptions::new()
            .write(true)
            .open(path)
            .with_context(|| format!("could not open {} for writing", path.display()))?;
        file.set_len(data_end_offset)?;
        let mut file = BufWriter::with_capacity(1 << 20, file);
        file.seek(SeekFrom::Start(data_end_offset))?;

        Ok(Appender {
            path: path.to_path_buf(),
            file,
            schemas,
            channels,
            chunk_indexes: summary.chunk_indexes,
            attachment_indexes: summary.attachment_indexes,
            metadata_indexes: summary.metadata_indexes,
            statistics: summary.stats,
            sequences,
            pending: Vec::new(),
            appended: 0,
            appended_chunks: 0,
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn channels(&self) -> &BTreeMap<u16, Channel> {
        &self.channels
    }

    pub fn schemas(&self) -> &BTreeMap<u16, Schema> {
        &self.schemas
    }

    /// The id of the channel on `topic`, if the recording already has one.
    pub fn channel_id(&self, topic: &str) -> Option<u16> {
        self.channels
            .iter()
            .find(|(_, channel)| channel.topic == topic)
            .map(|(id, _)| *id)
    }

    /// Messages already on `channel_id` before this session, per the
    /// recording's own statistics.
    pub fn message_count(&self, channel_id: u16) -> u64 {
        self.statistics
            .as_ref()
            .and_then(|stats| stats.channel_message_counts.get(&channel_id).copied())
            .unwrap_or(0)
    }

    /// Reuses an identical schema already in the file, otherwise adds one.
    pub fn schema(&mut self, name: &str, encoding: &str, data: &[u8]) -> u16 {
        if let Some((id, _)) = self
            .schemas
            .iter()
            .find(|(_, schema)| schema.name == name && schema.encoding == encoding && schema.data == data)
        {
            return *id;
        }
        let id = self.schemas.keys().next_back().map_or(1, |last| last + 1);
        self.schemas.insert(
            id,
            Schema {
                name: name.into(),
                encoding: encoding.into(),
                data: data.to_vec(),
            },
        );
        id
    }

    /// The channel for `topic`, created if the file has none. An existing
    /// channel is reused as it is — its schema and metadata win — so the
    /// caller should check [`Appender::channel_id`] first when that matters.
    pub fn channel(
        &mut self,
        topic: &str,
        schema_id: u16,
        message_encoding: &str,
        metadata: &BTreeMap<String, String>,
    ) -> u16 {
        if let Some(id) = self.channel_id(topic) {
            return id;
        }
        let id = self.channels.keys().next_back().map_or(1, |last| last + 1);
        self.channels.insert(
            id,
            Channel {
                schema_id,
                topic: topic.into(),
                message_encoding: message_encoding.into(),
                metadata: metadata.clone(),
            },
        );
        id
    }

    /// Queues one message. Everything queued is held until [`Appender::finish`]
    /// and then written in log-time order, split into chunks of roughly
    /// `CHUNK_TARGET_BYTES`; the things appended here — odometry, transforms —
    /// are megabytes, and holding them is what lets the appended chunks be
    /// ordered among themselves.
    pub fn write(&mut self, channel_id: u16, log_time: u64, data: Vec<u8>) -> Result<()> {
        if !self.channels.contains_key(&channel_id) {
            bail!("no channel {channel_id}");
        }
        let sequence = self.sequences.entry(channel_id).or_insert(0);
        *sequence = sequence.wrapping_add(1);
        self.pending.push(Pending {
            channel_id,
            sequence: *sequence,
            log_time,
            publish_time: log_time,
            data,
        });
        Ok(())
    }

    fn flush_all(&mut self) -> Result<()> {
        let mut messages = std::mem::take(&mut self.pending);
        messages.sort_by_key(|message| message.log_time);
        let mut chunk: Vec<Pending> = Vec::new();
        let mut chunk_bytes = 0;
        for message in messages {
            // Cut on either limit. The span is measured from the chunk's first
            // message, so a burst that fits in the window still gets one chunk.
            let spans_too_long = chunk
                .first()
                .is_some_and(|first| message.log_time.saturating_sub(first.log_time) >= CHUNK_TARGET_NANOS);
            if spans_too_long {
                self.flush_chunk(std::mem::take(&mut chunk))?;
                chunk_bytes = 0;
            }
            chunk_bytes += message.data.len() + 22 + 9;
            chunk.push(message);
            if chunk_bytes >= CHUNK_TARGET_BYTES {
                self.flush_chunk(std::mem::take(&mut chunk))?;
                chunk_bytes = 0;
            }
        }
        self.flush_chunk(chunk)
    }

    pub fn appended(&self) -> u64 {
        self.appended
    }

    /// Writes `messages`, already in log-time order, as one chunk plus its
    /// message indexes.
    fn flush_chunk(&mut self, messages: Vec<Pending>) -> Result<()> {
        if messages.is_empty() {
            return Ok(());
        }

        // Every channel the chunk uses is declared inside it, ahead of its
        // first message, whether or not an earlier chunk declared it: a linear
        // reader needs the declaration, and a repeated identical record is
        // allowed. Schemas likewise.
        let mut body = Vec::with_capacity(CHUNK_TARGET_BYTES + 4096);
        let mut used: Vec<u16> = messages.iter().map(|message| message.channel_id).collect();
        used.sort_unstable();
        used.dedup();
        let mut declared_schemas = Vec::new();
        for channel_id in &used {
            let channel = &self.channels[channel_id];
            if channel.schema_id != 0 && !declared_schemas.contains(&channel.schema_id) {
                declared_schemas.push(channel.schema_id);
                let schema = &self.schemas[&channel.schema_id];
                write_record(&mut body, op::SCHEMA, &schema_record(channel.schema_id, schema));
            }
            write_record(&mut body, op::CHANNEL, &channel_record(*channel_id, channel));
        }

        let mut index_entries: BTreeMap<u16, Vec<(u64, u64)>> = BTreeMap::new();
        for message in &messages {
            index_entries
                .entry(message.channel_id)
                .or_default()
                .push((message.log_time, body.len() as u64));
            let mut record = Vec::with_capacity(message.data.len() + 22);
            record.extend_from_slice(&message.channel_id.to_le_bytes());
            record.extend_from_slice(&message.sequence.to_le_bytes());
            record.extend_from_slice(&message.log_time.to_le_bytes());
            record.extend_from_slice(&message.publish_time.to_le_bytes());
            record.extend_from_slice(&message.data);
            write_record(&mut body, op::MESSAGE, &record);
        }
        let message_start_time = messages.first().map_or(0, |message| message.log_time);
        let message_end_time = messages.last().map_or(0, |message| message.log_time);

        let compressed = zstd::bulk::compress(&body, 1).context("zstd failed on an appended chunk")?;
        let mut chunk = Vec::with_capacity(compressed.len() + 64);
        chunk.extend_from_slice(&message_start_time.to_le_bytes());
        chunk.extend_from_slice(&message_end_time.to_le_bytes());
        chunk.extend_from_slice(&(body.len() as u64).to_le_bytes());
        // A zero CRC means "not computed", which readers honour by skipping the
        // check; the chunk is verified by being read back at the end instead.
        chunk.extend_from_slice(&0u32.to_le_bytes());
        write_string(&mut chunk, "zstd");
        chunk.extend_from_slice(&(compressed.len() as u64).to_le_bytes());
        chunk.extend_from_slice(&compressed);

        let chunk_start_offset = self.file.stream_position()?;
        write_record(&mut self.file, op::CHUNK, &chunk);
        let chunk_length = 9 + chunk.len() as u64;

        let mut message_index_offsets = BTreeMap::new();
        let index_start = self.file.stream_position()?;
        for (channel_id, entries) in &index_entries {
            message_index_offsets.insert(*channel_id, self.file.stream_position()?);
            let mut record = Vec::with_capacity(entries.len() * 16 + 8);
            record.extend_from_slice(&channel_id.to_le_bytes());
            record.extend_from_slice(&((entries.len() * 16) as u32).to_le_bytes());
            for (log_time, offset) in entries {
                record.extend_from_slice(&log_time.to_le_bytes());
                record.extend_from_slice(&offset.to_le_bytes());
            }
            write_record(&mut self.file, op::MESSAGE_INDEX, &record);
        }
        let message_index_length = self.file.stream_position()? - index_start;

        self.chunk_indexes.push(records::ChunkIndex {
            message_start_time,
            message_end_time,
            chunk_start_offset,
            chunk_length,
            message_index_offsets,
            message_index_length,
            compression: "zstd".into(),
            compressed_size: compressed.len() as u64,
            uncompressed_size: body.len() as u64,
        });

        if let Some(stats) = self.statistics.as_mut() {
            stats.message_count += messages.len() as u64;
            stats.chunk_count += 1;
            if stats.message_count == messages.len() as u64 {
                stats.message_start_time = message_start_time;
                stats.message_end_time = message_end_time;
            } else {
                stats.message_start_time = stats.message_start_time.min(message_start_time);
                stats.message_end_time = stats.message_end_time.max(message_end_time);
            }
            for message in &messages {
                *stats.channel_message_counts.entry(message.channel_id).or_insert(0) += 1;
            }
        }
        self.appended += messages.len() as u64;
        self.appended_chunks += 1;
        Ok(())
    }

    /// Writes the last chunk and a summary covering every chunk, old and new,
    /// then reads the appended chunks back to prove they parse. Returns how
    /// many messages were appended.
    pub fn finish(mut self) -> Result<u64> {
        self.flush_all()?;
        write_record(&mut self.file, op::DATA_END, &0u32.to_le_bytes());

        let summary_start = self.file.stream_position()?;
        let mut groups: Vec<(u8, u64, u64)> = Vec::new();

        let mut group_start = summary_start;
        for (id, schema) in &self.schemas {
            write_record(&mut self.file, op::SCHEMA, &schema_record(*id, schema));
        }
        push_group(&mut groups, op::SCHEMA, group_start, self.file.stream_position()?);

        group_start = self.file.stream_position()?;
        for (id, channel) in &self.channels {
            write_record(&mut self.file, op::CHANNEL, &channel_record(*id, channel));
        }
        push_group(&mut groups, op::CHANNEL, group_start, self.file.stream_position()?);

        if let Some(stats) = self.statistics.as_mut() {
            stats.schema_count = self.schemas.len() as u16;
            stats.channel_count = self.channels.len() as u32;
            stats.chunk_count = self.chunk_indexes.len() as u32;
            group_start = self.file.stream_position()?;
            write_record(&mut self.file, op::STATISTICS, &statistics_record(stats));
            push_group(&mut groups, op::STATISTICS, group_start, self.file.stream_position()?);
        }

        group_start = self.file.stream_position()?;
        for chunk in &self.chunk_indexes {
            write_record(&mut self.file, op::CHUNK_INDEX, &chunk_index_record(chunk));
        }
        push_group(&mut groups, op::CHUNK_INDEX, group_start, self.file.stream_position()?);

        if !self.attachment_indexes.is_empty() {
            group_start = self.file.stream_position()?;
            for index in &self.attachment_indexes {
                write_record(&mut self.file, op::ATTACHMENT_INDEX, &attachment_index_record(index));
            }
            push_group(&mut groups, op::ATTACHMENT_INDEX, group_start, self.file.stream_position()?);
        }
        if !self.metadata_indexes.is_empty() {
            group_start = self.file.stream_position()?;
            for index in &self.metadata_indexes {
                write_record(&mut self.file, op::METADATA_INDEX, &metadata_index_record(index));
            }
            push_group(&mut groups, op::METADATA_INDEX, group_start, self.file.stream_position()?);
        }

        let summary_offset_start = self.file.stream_position()?;
        for (opcode, start, length) in &groups {
            let mut record = Vec::with_capacity(17);
            record.push(*opcode);
            record.extend_from_slice(&start.to_le_bytes());
            record.extend_from_slice(&length.to_le_bytes());
            write_record(&mut self.file, op::SUMMARY_OFFSET, &record);
        }

        let mut footer = Vec::with_capacity(20);
        footer.extend_from_slice(&summary_start.to_le_bytes());
        footer.extend_from_slice(&summary_offset_start.to_le_bytes());
        footer.extend_from_slice(&0u32.to_le_bytes());
        write_record(&mut self.file, op::FOOTER, &footer);
        self.file.write_all(&MAGIC)?;
        self.file.flush()?;
        self.file.get_ref().sync_all()?;

        let appended_from = self.chunk_indexes.len() - self.appended_chunks;
        verify_appended(&self.path, &self.chunk_indexes[appended_from..])?;
        Ok(self.appended)
    }
}

/// Reads each of `chunks` back through the real reader, so an appended chunk
/// that would not parse is caught before the file is handed back.
fn verify_appended(path: &Path, chunks: &[records::ChunkIndex]) -> Result<()> {
    let file = File::open(path)?;
    let mapped = unsafe { memmap2::Mmap::map(&file)? };
    let summary = mcap::Summary::read(&mapped)?.context("the summary written just now does not read back")?;
    for chunk in chunks {
        let mut count = 0u64;
        for message in summary.stream_chunk(&mapped, chunk)? {
            message.with_context(|| format!("the appended chunk at {} does not read back", chunk.chunk_start_offset))?;
            count += 1;
        }
        if count == 0 {
            bail!("the appended chunk at {} read back empty", chunk.chunk_start_offset);
        }
    }
    Ok(())
}

fn push_group(groups: &mut Vec<(u8, u64, u64)>, opcode: u8, start: u64, end: u64) {
    if end > start {
        groups.push((opcode, start, end - start));
    }
}

/// Finds the byte offset of the DataEnd record and reads the summary behind it.
fn locate_summary(mapped: &[u8]) -> Result<(u64, mcap::Summary)> {
    if mapped.len() < 8 + 9 + 29 + 8 || mapped[..8] != MAGIC || mapped[mapped.len() - 8..] != MAGIC {
        bail!("missing mcap magic");
    }
    let footer_at = mapped.len() - 8 - 29;
    if mapped[footer_at] != op::FOOTER {
        bail!("no footer record where one should be");
    }
    let summary_start = u64::from_le_bytes(mapped[footer_at + 9..footer_at + 17].try_into().unwrap());
    if summary_start == 0 {
        bail!("the recording has no summary section (was it cut short? try mcap_recover)");
    }
    let data_end = summary_start
        .checked_sub(13)
        .filter(|offset| mapped.get(*offset as usize) == Some(&op::DATA_END))
        .context("no DataEnd record in front of the summary")?;
    let summary = mcap::Summary::read(mapped)?.context("the summary section is empty")?;
    Ok((data_end, summary))
}

fn write_record(out: &mut impl Write, opcode: u8, body: &[u8]) {
    out.write_all(&[opcode]).expect("write failed");
    out.write_all(&(body.len() as u64).to_le_bytes()).expect("write failed");
    out.write_all(body).expect("write failed");
}

fn write_string(out: &mut Vec<u8>, value: &str) {
    out.extend_from_slice(&(value.len() as u32).to_le_bytes());
    out.extend_from_slice(value.as_bytes());
}

fn schema_record(id: u16, schema: &Schema) -> Vec<u8> {
    let mut record = Vec::with_capacity(schema.data.len() + 64);
    record.extend_from_slice(&id.to_le_bytes());
    write_string(&mut record, &schema.name);
    write_string(&mut record, &schema.encoding);
    record.extend_from_slice(&(schema.data.len() as u32).to_le_bytes());
    record.extend_from_slice(&schema.data);
    record
}

fn channel_record(id: u16, channel: &Channel) -> Vec<u8> {
    let mut record = Vec::with_capacity(128);
    record.extend_from_slice(&id.to_le_bytes());
    record.extend_from_slice(&channel.schema_id.to_le_bytes());
    write_string(&mut record, &channel.topic);
    write_string(&mut record, &channel.message_encoding);
    let mut entries = Vec::new();
    for (key, value) in &channel.metadata {
        write_string(&mut entries, key);
        write_string(&mut entries, value);
    }
    record.extend_from_slice(&(entries.len() as u32).to_le_bytes());
    record.extend_from_slice(&entries);
    record
}

fn statistics_record(stats: &records::Statistics) -> Vec<u8> {
    let mut record = Vec::with_capacity(64 + stats.channel_message_counts.len() * 10);
    record.extend_from_slice(&stats.message_count.to_le_bytes());
    record.extend_from_slice(&stats.schema_count.to_le_bytes());
    record.extend_from_slice(&stats.channel_count.to_le_bytes());
    record.extend_from_slice(&stats.attachment_count.to_le_bytes());
    record.extend_from_slice(&stats.metadata_count.to_le_bytes());
    record.extend_from_slice(&stats.chunk_count.to_le_bytes());
    record.extend_from_slice(&stats.message_start_time.to_le_bytes());
    record.extend_from_slice(&stats.message_end_time.to_le_bytes());
    record.extend_from_slice(&((stats.channel_message_counts.len() * 10) as u32).to_le_bytes());
    for (channel, count) in &stats.channel_message_counts {
        record.extend_from_slice(&channel.to_le_bytes());
        record.extend_from_slice(&count.to_le_bytes());
    }
    record
}

fn chunk_index_record(chunk: &records::ChunkIndex) -> Vec<u8> {
    let mut record = Vec::with_capacity(96 + chunk.message_index_offsets.len() * 10);
    record.extend_from_slice(&chunk.message_start_time.to_le_bytes());
    record.extend_from_slice(&chunk.message_end_time.to_le_bytes());
    record.extend_from_slice(&chunk.chunk_start_offset.to_le_bytes());
    record.extend_from_slice(&chunk.chunk_length.to_le_bytes());
    record.extend_from_slice(&((chunk.message_index_offsets.len() * 10) as u32).to_le_bytes());
    for (channel, offset) in &chunk.message_index_offsets {
        record.extend_from_slice(&channel.to_le_bytes());
        record.extend_from_slice(&offset.to_le_bytes());
    }
    record.extend_from_slice(&chunk.message_index_length.to_le_bytes());
    write_string(&mut record, &chunk.compression);
    record.extend_from_slice(&chunk.compressed_size.to_le_bytes());
    record.extend_from_slice(&chunk.uncompressed_size.to_le_bytes());
    record
}

fn attachment_index_record(index: &records::AttachmentIndex) -> Vec<u8> {
    let mut record = Vec::with_capacity(96);
    record.extend_from_slice(&index.offset.to_le_bytes());
    record.extend_from_slice(&index.length.to_le_bytes());
    record.extend_from_slice(&index.log_time.to_le_bytes());
    record.extend_from_slice(&index.create_time.to_le_bytes());
    record.extend_from_slice(&index.data_size.to_le_bytes());
    write_string(&mut record, &index.name);
    write_string(&mut record, &index.media_type);
    record
}

fn metadata_index_record(index: &records::MetadataIndex) -> Vec<u8> {
    let mut record = Vec::with_capacity(48);
    record.extend_from_slice(&index.offset.to_le_bytes());
    record.extend_from_slice(&index.length.to_le_bytes());
    write_string(&mut record, &index.name);
    record
}

/// The profile and library named in the file's header record.
pub fn header(mapped: &[u8]) -> Result<(String, String)> {
    if mapped.len() < 17 || mapped[..8] != MAGIC || mapped[8] != op::HEADER {
        bail!("missing mcap header");
    }
    let mut cursor = 17;
    let string = |cursor: &mut usize| -> Result<String> {
        let length = u32::from_le_bytes(mapped[*cursor..*cursor + 4].try_into()?) as usize;
        let text = std::str::from_utf8(&mapped[*cursor + 4..*cursor + 4 + length])?.to_string();
        *cursor += 4 + length;
        Ok(text)
    };
    let profile = string(&mut cursor)?;
    let library = string(&mut cursor)?;
    Ok((profile, library))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn scratch(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("lite_record_append_{name}_{}.mcap", crate::record::now_nanos()))
    }

    /// A small recording written by the ordinary writer: two channels, three
    /// chunks, a summary.
    fn write_recording(path: &Path, messages: u32) {
        let file = File::create(path).unwrap();
        let mut writer = mcap::WriteOptions::new()
            .compression(Some(mcap::Compression::Zstd))
            .chunk_size(Some(2048))
            .profile("ros2")
            .create(BufWriter::new(file))
            .unwrap();
        let schema = writer.add_schema("std_msgs/msg/String", "ros2msg", b"string data").unwrap();
        let first = writer.add_channel(schema, "/first", "cdr", &BTreeMap::new()).unwrap();
        let second = writer.add_channel(schema, "/second", "cdr", &BTreeMap::new()).unwrap();
        for index in 0..messages {
            let channel_id = if index % 2 == 0 { first } else { second };
            writer
                .write_to_known_channel(
                    &records::MessageHeader {
                        channel_id,
                        sequence: index,
                        log_time: 1_000 + index as u64 * 100,
                        publish_time: 1_000 + index as u64 * 100,
                    },
                    &[index as u8; 200],
                )
                .unwrap();
        }
        writer.finish().unwrap();
    }

    fn read_all(path: &Path) -> Vec<(String, u64, Vec<u8>)> {
        let bytes = std::fs::read(path).unwrap();
        mcap::MessageStream::new(&bytes)
            .unwrap()
            .map(|message| {
                let message = message.unwrap();
                (message.channel.topic.clone(), message.log_time, message.data.to_vec())
            })
            .collect()
    }

    #[test]
    fn appended_messages_join_the_index_and_the_originals_are_untouched() {
        let path = scratch("basic");
        write_recording(&path, 40);
        let before = read_all(&path);
        let size_before = std::fs::metadata(&path).unwrap().len();

        let mut appender = Appender::open(&path).unwrap();
        assert_eq!(appender.channel_id("/first"), Some(1));
        let schema = appender.schema("nav_msgs/msg/Odometry", "ros2msg", b"fake");
        let mut metadata = BTreeMap::new();
        metadata.insert("k".to_string(), "v".to_string());
        let odometry = appender.channel("/odom", schema, "cdr", &metadata);
        // Out of order on purpose: the chunk must sort them.
        for log_time in [2_500u64, 1_050, 3_000] {
            appender.write(odometry, log_time, log_time.to_le_bytes().to_vec()).unwrap();
        }
        assert_eq!(appender.finish().unwrap(), 3);

        let after = read_all(&path);
        assert_eq!(&after[..before.len()], &before[..]);
        let appended: Vec<u64> = after[before.len()..].iter().map(|(_, log_time, _)| *log_time).collect();
        assert_eq!(appended, vec![1_050, 2_500, 3_000]);
        assert!(after[before.len()..].iter().all(|(topic, _, _)| topic == "/odom"));
        assert!(std::fs::metadata(&path).unwrap().len() > size_before);

        let bytes = std::fs::read(&path).unwrap();
        let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
        let stats = summary.stats.clone().unwrap();
        assert_eq!(stats.message_count, 43);
        assert_eq!(stats.channel_count, 3);
        assert_eq!(stats.schema_count, 2);
        assert_eq!(stats.message_end_time, 1_000 + 39 * 100);
        assert_eq!(stats.channel_message_counts[&odometry], 3);
        let channel = &summary.channels[&odometry];
        assert_eq!(channel.topic, "/odom");
        assert_eq!(channel.metadata.get("k").map(String::as_str), Some("v"));
        assert_eq!(summary.chunk_indexes.len(), stats.chunk_count as usize);
        // The appended chunk is reachable through the index on its own.
        let last = summary.chunk_indexes.iter().max_by_key(|chunk| chunk.chunk_start_offset).unwrap();
        let indexed: Vec<u64> = summary
            .stream_chunk(&bytes, last)
            .unwrap()
            .map(|message| message.unwrap().log_time)
            .collect();
        assert_eq!(indexed, vec![1_050, 2_500, 3_000]);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn appending_to_an_existing_channel_continues_it() {
        let path = scratch("existing");
        write_recording(&path, 10);
        let mut appender = Appender::open(&path).unwrap();
        let first = appender.channel_id("/first").unwrap();
        assert_eq!(appender.message_count(first), 5);
        let schema = appender.schema("std_msgs/msg/String", "ros2msg", b"string data");
        assert_eq!(schema, appender.channels()[&first].schema_id, "an identical schema is reused");
        assert_eq!(appender.channel("/first", schema, "cdr", &BTreeMap::new()), first);
        appender.write(first, 9_000, vec![1]).unwrap();
        appender.finish().unwrap();

        let bytes = std::fs::read(&path).unwrap();
        let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
        assert_eq!(summary.channels.len(), 2);
        assert_eq!(summary.stats.unwrap().channel_message_counts[&first], 6);
        let last = read_all(&path).pop().unwrap();
        assert_eq!(last.0, "/first");
        assert_eq!(last.1, 9_000);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_large_append_spans_several_chunks_that_all_read_back() {
        let path = scratch("large");
        write_recording(&path, 4);
        let mut appender = Appender::open(&path).unwrap();
        let schema = appender.schema("x", "ros2msg", b"y");
        let channel = appender.channel("/bulk", schema, "cdr", &BTreeMap::new());
        let payload = vec![7u8; 100_000];
        for index in 0..100u64 {
            appender.write(channel, index, payload.clone()).unwrap();
        }
        appender.finish().unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
        assert!(summary.chunk_indexes.len() > 3, "{}", summary.chunk_indexes.len());
        assert_eq!(read_all(&path).len(), 104);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_recording_without_a_summary_is_refused_rather_than_damaged() {
        let path = scratch("truncated");
        write_recording(&path, 10);
        let bytes = std::fs::read(&path).unwrap();
        std::fs::write(&path, &bytes[..bytes.len() - 100]).unwrap();
        let error = match Appender::open(&path) {
            Ok(_) => panic!("a truncated file was accepted"),
            Err(error) => error,
        };
        assert!(format!("{error:#}").contains("not a finished, indexed mcap"), "{error:#}");
        assert_eq!(std::fs::read(&path).unwrap().len(), bytes.len() - 100, "the file was not touched");
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn the_header_names_the_profile_and_library() {
        let path = scratch("header");
        write_recording(&path, 2);
        let bytes = std::fs::read(&path).unwrap();
        let (profile, library) = header(&bytes).unwrap();
        assert_eq!(profile, "ros2");
        assert!(library.starts_with("mcap-rust"), "{library}");
        std::fs::remove_file(&path).ok();
    }
}
