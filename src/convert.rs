//! Turning a finished recording into one Foxglove will draw.
//!
//! Images record as lossless JPEG XL, which is the right choice for the card —
//! but Foxglove ships png, jpeg, webp and avif decoders and nothing for jxl, so
//! every image panel comes up empty. There is no extension to install for it the
//! way there was for RVL.
//!
//! So: decode every jxl frame and write it back in a format Foxglove has a
//! decoder for, picked per pixel layout by [`viewable_format`]. All of them are
//! lossless. The result is one file that is both the archive and the thing you
//! look at, which is the point — a viewing copy would mean carrying two.
//!
//! The same pass also mends calibrations: a camera_info recorded with
//! `distortion_model: "unknown"` — a RealSense inverse Brown-Conrady stream,
//! which has no ROS name — blanks any image panel its topic is attached to, so
//! it is refitted as genuine forward `plumb_bob` by [`crate::distortion`].

use std::collections::{BTreeMap, HashMap};
use std::fs::{File, OpenOptions};
use std::io::BufWriter;
use std::os::unix::fs::{FileExt, MetadataExt};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use anyhow::{Context, Result};
use mcap::sans_io::linear_reader::{LinearReadEvent, LinearReader, LinearReaderOptions};
use serde::Serialize;

use crate::cdr::{self, CdrReader};
use crate::image::{compress, decode_jpegxl, ImageFormat};
use crate::msgs::RawImage;

/// What `format` on a CompressedImage looks like when the payload is JPEG XL.
/// Matched case-insensitively because it is free text on the wire.
const JXL_FORMATS: [&str; 2] = ["jxl", "jpegxl"];

/// The suffix a compressed image topic carries, and which the decoded topic drops
/// so the two can coexist in one file. Applied when recording, see
/// [`crate::record`].
pub const COMPRESSED_SUFFIX: &str = "/compressed";

/// The recording is already in a form Foxglove can draw. Not a failure — the
/// CLI treats it as "nothing to do here" and carries on to the other stages.
#[derive(Debug)]
pub struct NothingToConvert;

impl std::fmt::Display for NothingToConvert {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "no jxl images or refittable camera infos — nothing to convert")
    }
}

impl std::error::Error for NothingToConvert {}

#[derive(Debug, Default, Serialize)]
pub struct Report {
    /// Frames re-encoded out of jxl into something Foxglove can decode.
    pub decoded: u64,
    /// Camera infos rewritten from an "unknown" inverse Brown-Conrady
    /// calibration to a fitted forward plumb_bob one.
    pub refitted: u64,
    /// `/tf_static` messages whose edges were written by an older recorder in
    /// the SDK's point-map direction and have been inverted into tf poses.
    pub inverted_transforms: u64,
    /// Messages whose header stamp was moved onto the file's clock.
    pub restamped: u64,
    /// Messages copied through untouched.
    pub copied: u64,
    /// Frames whose decode failed. They are dropped, not written broken.
    pub failed: u64,
    /// Size of the finished file. Not the sum of the payloads: raw depth is far
    /// larger than the jxl it replaces, and mcap's zstd takes most of that back.
    pub bytes: u64,
    /// Bytes handed back to the filesystem out of the source while the rewrite
    /// was still running. Zero unless the job was asked to reclaim.
    pub reclaimed: u64,
}

/// Live message count, so the browser can show a conversion moving rather than a
/// spinner that might be a hang. Shared with whoever kicked the job off.
#[derive(Default)]
pub struct Progress {
    pub messages: AtomicU64,
    pub bytes: AtomicU64,
}

fn file_size(path: &Path) -> u64 {
    std::fs::metadata(path).map(|data| data.len()).unwrap_or(0)
}

fn is_jxl(format: &str) -> bool {
    JXL_FORMATS.contains(&format.to_ascii_lowercase().as_str())
}

/// What a decoded frame should be written back as, chosen by its pixel layout.
///
/// Every option here is lossless, so this is only ever picking the smallest of
/// the formats Foxglove can decode. Ratios are of the jxl payload each replaces,
/// measured on frames off dimpi5 by `examples/recode_cost.rs`:
///
/// | layout            | raw   | png   | webp  |
/// |-------------------|-------|-------|-------|
/// | rgb8 colour       | 3.90x | 1.60x | 1.16x |
/// | mono8 infrared    | 2.65x | 1.05x | 1.17x |
/// | mono16 depth      | 6.20x | 1.36x |  n/a  |
///
/// Depth takes raw despite being the dearest of the three. A 16-bit png would be
/// smaller on disk but Foxglove hands compressed images to the browser's image
/// decoder, which returns 8 bits per channel, and its depth colormap only runs on
/// a raw `16UC1` image in the first place. Raw's 6.20x is of the payload and
/// mostly comes back out in the mcap chunk's zstd, which eats flat 16-bit runs.
///
/// The layout, not the topic name, is what decides this: an operator can rename a
/// topic prefix, and a stream that arrives in an unexpected layout should still
/// land on a format that can hold it rather than on whatever its name implied.
fn viewable_format(encoding: &str) -> ImageFormat {
    match encoding {
        "mono16" | "16UC1" => ImageFormat::Raw,
        "rgb8" | "bgr8" | "rgba8" | "bgra8" => ImageFormat::Webp,
        _ => ImageFormat::Png,
    }
}

/// Reads the CompressedImage far enough to answer "is this jxl", then decodes it.
/// Returns `None` for anything that is not a jxl frame, which is the signal to
/// copy the message through instead.
fn decoded_frame(payload: &[u8]) -> Option<Result<RawImage>> {
    let mut reader = CdrReader::new(payload);
    let header = reader.header();
    let format = reader.string();
    if !is_jxl(&format) {
        return None;
    }
    let compressed = reader.bytes();
    Some(decode_jpegxl(&compressed).map(|mut image| {
        // The decoder knows the pixel layout but not where the frame came from,
        // and Foxglove needs the frame_id to place it against the camera info.
        image.header = header;
        image.step = image.width * bytes_per_pixel(&image.encoding);
        image
    }))
}

fn bytes_per_pixel(encoding: &str) -> usize {
    match encoding {
        "mono16" => 2,
        "rgb8" => 3,
        _ => 1,
    }
}

/// Foxglove's depth colormap keys off `16UC1`; `mono16` is the same bytes under a
/// name its raw-image path treats as a greyscale photo instead.
fn depth_encoding(encoding: &str) -> &str {
    if encoding == "mono16" {
        "16UC1"
    } else {
        encoding
    }
}

/// Converts `input` in place. The decode lands in a temporary file beside it,
/// which replaces the original only when every single frame decoded — the
/// decode is exact, so the swap loses nothing, but a recording with even one
/// undecodable frame is left untouched rather than silently thinned.
pub fn in_place(
    input: &Path,
    progress: &Arc<Progress>,
    reclaim: Reclaim,
    shifts: &BTreeMap<u16, i64>,
) -> Result<Report> {
    let mut name = input.file_name().unwrap_or_default().to_os_string();
    name.push(".converting");
    let temp = input.with_file_name(name);
    // Once the source has had chunks punched out of it, the half-written temp
    // holds the only copy of everything converted so far. Deleting it on the way
    // out — which is the right thing to do for a conversion that kept its source
    // intact — would be the one action that actually loses messages.
    let discard = |temp: &Path| {
        if reclaim == Reclaim::No {
            let _ = std::fs::remove_file(temp);
        }
    };
    if shifts.is_empty() && !needs_conversion(input)? {
        return Err(NothingToConvert.into());
    }
    let report = match to_viewable(input, &temp, progress, reclaim, shifts) {
        Ok(report) => report,
        Err(error) => {
            discard(&temp);
            return Err(error);
        }
    };
    if report.failed > 0 {
        discard(&temp);
        anyhow::bail!(
            "{} frames would not decode, so {} was left untouched",
            report.failed,
            input.display()
        );
    }
    if report.decoded == 0 && report.refitted == 0 && report.restamped == 0 {
        // Nothing changed, so swapping in the rewrite would only churn the
        // file's compression. This is also what a second run hits. A
        // backwards `/tf_static` alone does not justify a rewrite either:
        // `fixup` corrects that by appending, which costs nothing.
        discard(&temp);
        return Err(NothingToConvert.into());
    }
    std::fs::rename(&temp, input)
        .with_context(|| format!("could not replace {}", input.display()))?;
    Ok(report)
}

/// Whether a rewrite would change anything: a jxl frame on any CompressedImage
/// channel, or an "unknown"-model calibration on any CameraInfo channel. Read
/// from the first message of each such channel rather than by walking the
/// file, so a second run — or a run that only wants the appended stages —
/// answers in a moment instead of rewriting 63 GB to find out nothing changed.
/// A file with no index is assumed to need it; the walk will find out.
pub fn needs_conversion(input: &Path) -> Result<bool> {
    let source = File::open(input).with_context(|| format!("could not open {}", input.display()))?;
    let mapped = unsafe { memmap2::Mmap::map(&source) }
        .with_context(|| format!("could not map {}", input.display()))?;
    let Some(summary) = mcap::Summary::read(&mapped).ok().flatten() else {
        return Ok(true);
    };
    let mut wanted: std::collections::BTreeSet<u16> = summary
        .channels
        .values()
        .filter(|channel| {
            channel.schema.as_ref().is_some_and(|schema| {
                schema.name == crate::msgs::COMPRESSED_IMAGE_TYPE
                    || schema.name == crate::msgs::CAMERA_INFO_TYPE
            })
        })
        .map(|channel| channel.id)
        .collect();
    let mut chunks = summary.chunk_indexes.clone();
    chunks.sort_by_key(|chunk| chunk.chunk_start_offset);
    for chunk in &chunks {
        if !chunk.message_index_offsets.keys().any(|id| wanted.contains(id)) {
            continue;
        }
        for message in summary.stream_chunk(&mapped, chunk)? {
            let message = message?;
            if !wanted.remove(&message.channel.id) {
                continue;
            }
            let schema_name = message.channel.schema.as_ref().map(|schema| schema.name.as_str());
            if schema_name == Some(crate::msgs::COMPRESSED_IMAGE_TYPE) {
                let mut reader = CdrReader::new(&message.data);
                let _ = reader.try_header();
                if reader.try_string().is_some_and(|format| is_jxl(&format)) {
                    return Ok(true);
                }
            } else if crate::distortion::parse_camera_info(&message.data).distortion_model
                == crate::msgs::DistortionModel::Unknown.as_str()
            {
                return Ok(true);
            }
        }
        if wanted.is_empty() {
            break;
        }
    }
    Ok(false)
}

/// The output file and the bookkeeping that maps each source channel onto its
/// replacement. Split out from the drivers because the whole-file walk and the
/// chunk-at-a-time walk differ only in how they find the next message.
struct Rewriter {
    writer: mcap::Writer<BufWriter<File>>,
    /// Keyed by source channel id, to the output channel id and its running
    /// sequence number. A decoded channel and a copied channel never share an
    /// id, so one map covers both.
    channels: HashMap<u16, (u16, u32)>,
    /// One distortion fit per camera_info channel, keyed by the coefficients
    /// it was made from — a calibration is constant over a recording, and a
    /// stream carries tens of thousands of copies of it, none of which should
    /// pay for the fit again. `None` remembers a fit that was rejected.
    fits: HashMap<u16, (Vec<f64>, Option<[f64; 5]>)>,
    /// Per source channel, the nanoseconds to add to every header stamp so the
    /// stream lands on the clock the file was logged on. See `crate::restamp`.
    shifts: BTreeMap<u16, i64>,
    report: Report,
}

impl Rewriter {
    fn new(output: &Path, shifts: &BTreeMap<u16, i64>) -> Result<Self> {
        let destination = File::create(output)
            .with_context(|| format!("could not create {}", output.display()))?;
        let writer = mcap::WriteOptions::new()
            .compression(Some(mcap::Compression::Zstd))
            .compression_level(1)
            .profile("ros2")
            .create(BufWriter::with_capacity(1 << 20, destination))?;
        Ok(Self {
            writer,
            channels: HashMap::new(),
            fits: HashMap::new(),
            shifts: shifts.clone(),
            report: Report::default(),
        })
    }

    /// The camera_info leg of the rewrite: an "unknown"-model calibration gets
    /// its distortion refitted as forward plumb_bob. `None` — copy the message
    /// through untouched — for every other model, and for a stream whose fit
    /// did not reproduce the recorded mapping.
    fn refit(&mut self, channel: u16, payload: &[u8]) -> Option<cdr::Encoded> {
        let info = crate::distortion::parse_camera_info(payload);
        if info.distortion_model != crate::msgs::DistortionModel::Unknown.as_str() {
            return None;
        }
        let cached = self
            .fits
            .get(&channel)
            .filter(|(source, _)| *source == info.distortion)
            .map(|(_, fit)| *fit);
        let fitted = cached.unwrap_or_else(|| {
            let fit = crate::distortion::fit_forward_plumb_bob(&info);
            self.fits.insert(channel, (info.distortion.clone(), fit));
            fit
        });
        fitted.map(|coefficients| {
            cdr::camera_info(&crate::msgs::CameraInfo {
                distortion_model: crate::msgs::DistortionModel::PlumbBob.as_str().to_string(),
                distortion: coefficients.to_vec(),
                ..info
            })
        })
    }

    fn write(&mut self, message: &mcap::Message) -> Result<()> {
        let channel = &message.channel;
        let schema_name = channel.schema.as_ref().map(|schema| schema.name.as_str());

        let rewritten = if schema_name == Some(crate::msgs::COMPRESSED_IMAGE_TYPE) {
            match decoded_frame(&message.data) {
                Some(Ok(image)) => match viewable_format(&image.encoding) {
                    ImageFormat::Raw => {
                        self.report.decoded += 1;
                        Some(cdr::raw_image(&RawImage {
                            encoding: depth_encoding(&image.encoding).to_string(),
                            ..image
                        }))
                    }
                    format => match compress(&image, format) {
                        Some(encoded) => {
                            self.report.decoded += 1;
                            Some(cdr::compressed_image(&encoded))
                        }
                        // Only reachable if a layout reaches a format that cannot
                        // hold it, which `viewable_format` exists to prevent. Failing
                        // here leaves the recording untouched rather than thinning it.
                        None => {
                            self.report.failed += 1;
                            return Ok(());
                        }
                    },
                },
                Some(Err(_)) => {
                    // A frame that will not decode is dropped rather than written as
                    // broken pixels, and shows up in the report.
                    self.report.failed += 1;
                    return Ok(());
                }
                None => {
                    self.report.copied += 1;
                    None
                }
            }
        } else if schema_name == Some(crate::msgs::CAMERA_INFO_TYPE) {
            match self.refit(channel.id, &message.data) {
                Some(encoded) => {
                    self.report.refitted += 1;
                    Some(encoded)
                }
                None => {
                    self.report.copied += 1;
                    None
                }
            }
        } else if schema_name == Some(crate::msgs::TF_TYPE)
            && channel.topic == "/tf_static"
            && !crate::fixup::is_marked(&channel.metadata)
        {
            match crate::cdr::decode_tf_message(&message.data).ok() {
                Some(transforms) => {
                    let inverted: Vec<_> = transforms
                        .iter()
                        .map(|edge| {
                            crate::tf::Pose::from_transform(edge).inverse().stamped(
                                edge.header.stamp_nanos(),
                                edge.parent(),
                                &edge.child_frame_id,
                            )
                        })
                        .collect();
                    self.report.inverted_transforms += 1;
                    Some(crate::cdr::tf_message(&inverted))
                }
                None => {
                    self.report.copied += 1;
                    None
                }
            }
        } else {
            self.report.copied += 1;
            None
        };

        if let std::collections::hash_map::Entry::Vacant(slot) = self.channels.entry(channel.id) {
            let (schema_id, topic) = match &rewritten {
                Some(encoded) => (
                    self.writer.add_schema(
                        encoded.schema_name,
                        "ros2msg",
                        encoded.schema_text.as_bytes(),
                    )?,
                    // Depth comes out as a raw Image and so takes the plain topic
                    // name a dimos graph expects. A stream that is still a
                    // CompressedImage, just in a decodable codec, keeps its
                    // suffix — the name describes the schema, not the codec.
                    match encoded.schema_name == crate::msgs::IMAGE_TYPE {
                        true => channel
                            .topic
                            .strip_suffix(COMPRESSED_SUFFIX)
                            .unwrap_or(&channel.topic),
                        false => channel.topic.as_str(),
                    },
                ),
                None => {
                    let schema = channel
                        .schema
                        .as_ref()
                        .context("a channel with no schema cannot be copied through")?;
                    (
                        self.writer
                            .add_schema(&schema.name, &schema.encoding, &schema.data)?,
                        channel.topic.as_str(),
                    )
                }
            };
            // A `/tf_static` that has just been inverted is marked as such, so
            // nothing downstream inverts it a second time.
            let mut metadata = channel.metadata.clone();
            if topic == "/tf_static" {
                metadata.extend(crate::record::channel_metadata(crate::record::TF_TOPIC));
            }
            let id =
                self.writer
                    .add_channel(schema_id, topic, &channel.message_encoding, &metadata)?;
            slot.insert((id, 0));
        }

        let entry = self.channels.get_mut(&channel.id).expect("just inserted");
        entry.1 = entry.1.wrapping_add(1);
        let (channel_id, sequence) = *entry;

        let payload = rewritten
            .as_ref()
            .map_or(message.data.as_ref(), |encoded| encoded.data.as_slice());
        // The stamp is moved last, so it applies whether the payload was
        // re-encoded on the way through or copied untouched.
        let shifted = self.shifts.get(&channel.id).map(|offset| {
            let mut moved = payload.to_vec();
            if crate::restamp::shift_header(&mut moved, *offset) {
                self.report.restamped += 1;
            }
            moved
        });
        let payload = shifted.as_deref().unwrap_or(payload);
        self.writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id,
                sequence,
                log_time: message.log_time,
                publish_time: message.publish_time,
            },
            payload,
        )?;
        Ok(())
    }

    fn written(&self) -> u64 {
        self.report.decoded + self.report.refitted + self.report.inverted_transforms + self.report.copied
    }
}

/// Whether the source may be eaten as it is read. See [`by_chunk`].
#[derive(Clone, Copy, PartialEq)]
pub enum Reclaim {
    No,
    /// Punch each source chunk out once its replacement is on disk and reads
    /// back. Peak space becomes the output alone rather than both files, at the
    /// cost of destroying the source progressively.
    AsItGoes,
}

pub fn to_viewable(
    input: &Path,
    output: &Path,
    progress: &Arc<Progress>,
    reclaim: Reclaim,
    shifts: &BTreeMap<u16, i64>,
) -> Result<Report> {
    // Write access only when we intend to punch: a plain conversion should not
    // be able to touch the source even by accident.
    let source = OpenOptions::new()
        .read(true)
        .write(reclaim == Reclaim::AsItGoes)
        .open(input)
        .with_context(|| format!("could not open {}", input.display()))?;
    // Mapped rather than read: a long recording is larger than the Pi's memory,
    // and the pages behind an mcap are touched once and never again.
    let mapped = unsafe { memmap2::Mmap::map(&source) }
        .with_context(|| format!("could not map {}", input.display()))?;

    // The summary sits at the end of the file and carries every chunk's offset,
    // which is what makes a chunk-at-a-time walk possible. A file without one —
    // a recovered or truncated recording — can still be converted, just only by
    // reading it straight through.
    let summary = mcap::Summary::read(&mapped).ok().flatten();
    match summary.filter(|summary| !summary.chunk_indexes.is_empty()) {
        Some(summary) => by_chunk(&mapped, &source, &summary, output, progress, reclaim, shifts),
        None if reclaim == Reclaim::AsItGoes => anyhow::bail!(
            "{} has no chunk index, so it cannot be converted a chunk at a time — \
             run it through mcap_recover first",
            input.display()
        ),
        None => whole_file(&mapped, output, progress, shifts),
    }
}

/// The straight-through walk, for a file whose index is missing.
fn whole_file(
    mapped: &[u8],
    output: &Path,
    progress: &Arc<Progress>,
    shifts: &BTreeMap<u16, i64>,
) -> Result<Report> {
    let mut rewriter = Rewriter::new(output, shifts)?;
    for message in mcap::MessageStream::new(mapped)? {
        rewriter.write(&message?)?;
        progress.messages.store(rewriter.written(), Ordering::Relaxed);
        // Stat rather than sum the payloads, so the browser shows room going off
        // the card. Occasionally, because it is a syscall in the message loop.
        if rewriter.written() % 256 == 0 {
            progress.bytes.store(file_size(output), Ordering::Relaxed);
        }
    }
    rewriter.writer.finish()?;
    let mut report = rewriter.report;
    report.bytes = file_size(output);
    progress.bytes.store(report.bytes, Ordering::Relaxed);
    Ok(report)
}

/// Converts one source chunk at a time, and — when asked — hands each source
/// chunk back to the filesystem as soon as its replacement is on disk.
///
/// The order matters and is the whole safety argument: a chunk is converted,
/// flushed so it is a complete chunk record rather than a half-written
/// compression stream, read back off the disk to prove it parses, and only then
/// is the source's copy punched out. Nothing is ever released on the strength of
/// a write that has not been verified.
///
/// What this cannot do is put the rewritten chunk back where the old one was.
/// It is bigger — that is the point of the conversion — and every offset after
/// it would shift, so the output is still a second file that gets renamed over
/// the source at the end. Reclaiming is what keeps the two from having to
/// coexist at full size: peak usage is the output alone, not both.
///
/// The cost, and it is a real one: once punching has started the source is no
/// longer a whole recording. If the job dies midway the messages all still
/// exist, but split across the partial output and the un-punched tail of the
/// source, and putting them back together is a manual job.
fn by_chunk(
    mapped: &[u8],
    source: &File,
    summary: &mcap::Summary,
    output: &Path,
    progress: &Arc<Progress>,
    reclaim: Reclaim,
    shifts: &BTreeMap<u16, i64>,
) -> Result<Report> {
    if !summary.attachment_indexes.is_empty() || !summary.metadata_indexes.is_empty() {
        anyhow::bail!(
            "this recording carries attachments or metadata, which a chunk-at-a-time \
             rewrite would drop"
        );
    }

    // Punching walks forward through the file, so the chunks have to be in file
    // order rather than whatever order the index happens to list them in.
    let mut chunks = summary.chunk_indexes.clone();
    chunks.sort_by_key(|chunk| chunk.chunk_start_offset);

    let block = source.metadata().map(|data| data.blksize()).unwrap_or(4096);
    let mut rewriter = Rewriter::new(output, shifts)?;
    // The header and the leading magic go out now, so that every range measured
    // from here on starts on a record boundary and can be parsed on its own.
    rewriter.writer.flush()?;
    let mut checked = file_size(output);
    let mut reclaimed = 0;

    for chunk in &chunks {
        let before = rewriter.written();
        for message in summary.stream_chunk(mapped, chunk)? {
            rewriter.write(&message?)?;
        }
        // Ends the output chunk and pushes it through the BufWriter, so what we
        // are about to read back is actually on the disk.
        rewriter.writer.flush()?;

        let grown = file_size(output);
        let found = verify(output, checked, grown).with_context(|| {
            format!(
                "the output written for the chunk at {} did not read back",
                chunk.chunk_start_offset
            )
        })?;
        let expected = rewriter.written() - before;
        if found != expected {
            anyhow::bail!(
                "the chunk at {} wrote {expected} messages but only {found} read back",
                chunk.chunk_start_offset
            );
        }
        checked = grown;

        if reclaim == Reclaim::AsItGoes {
            // The message index records sit right behind the chunk and are just
            // as dead once it has been converted, so they go too.
            let length = chunk.chunk_length + chunk.message_index_length;
            reclaimed += release(source, chunk.chunk_start_offset, length, block)?;
        }

        progress.messages.store(rewriter.written(), Ordering::Relaxed);
        progress.bytes.store(grown, Ordering::Relaxed);
    }

    rewriter.writer.finish()?;
    let mut report = rewriter.report;
    report.bytes = file_size(output);
    report.reclaimed = reclaimed;
    progress.bytes.store(report.bytes, Ordering::Relaxed);

    // A message living outside a chunk would never have been visited, and the
    // only way to notice is to count. Better to refuse than to hand back a
    // recording that is quietly missing messages.
    if let Some(stats) = &summary.stats {
        let seen = report.decoded + report.refitted + report.inverted_transforms + report.copied + report.failed;
        if stats.message_count != 0 && stats.message_count != seen {
            anyhow::bail!(
                "the index accounts for {} messages but the file says it holds {} — \
                 some of them are outside the chunks",
                seen,
                stats.message_count
            );
        }
    }
    Ok(report)
}

/// Reads back the bytes just appended to the output and parses every record in
/// them, returning how many messages they hold. This is what a chunk is
/// released on the strength of, so it reads from the file rather than trusting
/// the buffer it was written from.
fn verify(output: &Path, from: u64, to: u64) -> Result<u64> {
    if to <= from {
        return Ok(0);
    }
    let file = File::open(output)?;

    // The reader walks into chunks rather than handing them back whole, so a
    // message only turns up here if its chunk's zstd stream decompressed and
    // every record in it parsed — which is exactly what needs proving before the
    // source's copy is released.
    //
    // `read::LinearReader` cannot do this job: it caps record length at the
    // length of the buffer it is handed, and a decompressed frame is routinely
    // larger than the compressed chunk it came out of.
    let mut reader = LinearReader::new_with_options(
        LinearReaderOptions::default()
            .with_skip_start_magic(true)
            .with_skip_end_magic(true),
    );
    let mut cursor = from;
    let mut messages = 0;
    while let Some(event) = reader.next_event() {
        match event? {
            LinearReadEvent::ReadRequest(wanted) => {
                let taking = wanted.min((to - cursor) as usize);
                let read = file.read_at(&mut reader.insert(taking)[..taking], cursor)?;
                reader.notify_read(read);
                cursor += read as u64;
            }
            LinearReadEvent::Record { data, opcode } => {
                if matches!(
                    mcap::read::parse_record(opcode, data)?,
                    mcap::records::Record::Message { .. }
                ) {
                    messages += 1;
                }
            }
        }
    }
    Ok(messages)
}

/// Hands a byte range back to the filesystem, leaving a hole where it was. The
/// file keeps its length; reading the hole gives zeros.
///
/// Only whole blocks are released. A partial block at either end is left alone:
/// macOS refuses an unaligned punch outright, and on Linux it would only zero
/// the bytes without freeing anything. Returns how much was actually freed.
fn release(file: &File, offset: u64, length: u64, block: u64) -> Result<u64> {
    let start = offset.div_ceil(block) * block;
    let end = (offset + length) / block * block;
    if end <= start {
        return Ok(0);
    }
    punch(file, start, end - start)
        .with_context(|| format!("could not release {} bytes at {start}", end - start))?;
    Ok(end - start)
}

#[cfg(target_os = "linux")]
fn punch(file: &File, offset: u64, length: u64) -> std::io::Result<()> {
    use std::os::unix::io::AsRawFd;
    let result = unsafe {
        libc::fallocate(
            file.as_raw_fd(),
            libc::FALLOC_FL_PUNCH_HOLE | libc::FALLOC_FL_KEEP_SIZE,
            offset as libc::off_t,
            length as libc::off_t,
        )
    };
    match result {
        0 => Ok(()),
        _ => Err(std::io::Error::last_os_error()),
    }
}

#[cfg(target_os = "macos")]
fn punch(file: &File, offset: u64, length: u64) -> std::io::Result<()> {
    use std::os::unix::io::AsRawFd;
    let mut hole = libc::fpunchhole_t {
        fp_flags: 0,
        reserved: 0,
        fp_offset: offset as libc::off_t,
        fp_length: length as libc::off_t,
    };
    let result =
        unsafe { libc::fcntl(file.as_raw_fd(), libc::F_PUNCHHOLE, &mut hole as *mut _) };
    match result {
        -1 => Err(std::io::Error::last_os_error()),
        _ => Ok(()),
    }
}
