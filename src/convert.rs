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

use std::collections::HashMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use anyhow::{Context, Result};
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

#[derive(Debug, Default, Serialize)]
pub struct Report {
    /// Frames re-encoded out of jxl into something Foxglove can decode.
    pub decoded: u64,
    /// Messages copied through untouched.
    pub copied: u64,
    /// Frames whose decode failed. They are dropped, not written broken.
    pub failed: u64,
    /// Size of the finished file. Not the sum of the payloads: raw depth is far
    /// larger than the jxl it replaces, and mcap's zstd takes most of that back.
    pub bytes: u64,
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
pub fn in_place(input: &Path, progress: &Arc<Progress>) -> Result<Report> {
    let mut name = input.file_name().unwrap_or_default().to_os_string();
    name.push(".converting");
    let temp = input.with_file_name(name);
    let report = match to_viewable(input, &temp, progress) {
        Ok(report) => report,
        Err(error) => {
            let _ = std::fs::remove_file(&temp);
            return Err(error);
        }
    };
    if report.failed > 0 {
        let _ = std::fs::remove_file(&temp);
        anyhow::bail!(
            "{} frames would not decode, so {} was left untouched",
            report.failed,
            input.display()
        );
    }
    if report.decoded == 0 {
        // Nothing changed, so swapping in the rewrite would only churn the
        // file's compression. This is also what a second run hits.
        let _ = std::fs::remove_file(&temp);
        anyhow::bail!("no jxl images in the file — nothing to convert");
    }
    std::fs::rename(&temp, input)
        .with_context(|| format!("could not replace {}", input.display()))?;
    Ok(report)
}

pub fn to_viewable(input: &Path, output: &Path, progress: &Arc<Progress>) -> Result<Report> {
    let source = File::open(input).with_context(|| format!("could not open {}", input.display()))?;
    // Mapped rather than read: a long recording is larger than the Pi's memory,
    // and the pages behind an mcap are touched once and never again.
    let mapped = unsafe { memmap2::Mmap::map(&source) }
        .with_context(|| format!("could not map {}", input.display()))?;

    let destination =
        File::create(output).with_context(|| format!("could not create {}", output.display()))?;
    let mut writer = mcap::WriteOptions::new()
        .compression(Some(mcap::Compression::Zstd))
        .compression_level(1)
        .profile("ros2")
        .create(BufWriter::with_capacity(1 << 20, destination))?;

    // Keyed by source channel id. A decoded channel and a copied channel never
    // share an id, so one map covers both.
    let mut channels: HashMap<u16, (u16, u32)> = HashMap::new();
    let mut report = Report::default();

    for message in mcap::MessageStream::new(&mapped)? {
        let message = message?;
        let channel = &message.channel;
        let decoded = channel
            .schema
            .as_ref()
            .filter(|schema| schema.name == crate::msgs::COMPRESSED_IMAGE_TYPE)
            .and_then(|_| decoded_frame(&message.data));

        let rewritten = match decoded {
            Some(Ok(image)) => match viewable_format(&image.encoding) {
                ImageFormat::Raw => {
                    report.decoded += 1;
                    Some(cdr::raw_image(&RawImage {
                        encoding: depth_encoding(&image.encoding).to_string(),
                        ..image
                    }))
                }
                format => match compress(&image, format) {
                    Some(encoded) => {
                        report.decoded += 1;
                        Some(cdr::compressed_image(&encoded))
                    }
                    // Only reachable if a layout reaches a format that cannot
                    // hold it, which `viewable_format` exists to prevent. Failing
                    // here leaves the recording untouched rather than thinning it.
                    None => {
                        report.failed += 1;
                        continue;
                    }
                },
            },
            Some(Err(_)) => {
                // A frame that will not decode is dropped rather than written as
                // broken pixels, and shows up in the report.
                report.failed += 1;
                continue;
            }
            None => {
                report.copied += 1;
                None
            }
        };

        if let std::collections::hash_map::Entry::Vacant(slot) = channels.entry(channel.id) {
            let (schema_id, topic) = match &rewritten {
                Some(encoded) => (
                    writer.add_schema(
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
                        writer.add_schema(&schema.name, &schema.encoding, &schema.data)?,
                        channel.topic.as_str(),
                    )
                }
            };
            let id = writer.add_channel(
                schema_id,
                topic,
                &channel.message_encoding,
                &channel.metadata,
            )?;
            slot.insert((id, 0));
        }

        let entry = channels.get_mut(&channel.id).expect("just inserted");
        entry.1 = entry.1.wrapping_add(1);
        let (channel_id, sequence) = *entry;

        let payload = rewritten
            .as_ref()
            .map_or(message.data.as_ref(), |encoded| encoded.data.as_slice());
        writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id,
                sequence,
                log_time: message.log_time,
                publish_time: message.publish_time,
            },
            payload,
        )?;
        let written = report.decoded + report.copied;
        progress.messages.store(written, Ordering::Relaxed);
        // Stat rather than sum the payloads, so the browser shows room going off
        // the card. Occasionally, because it is a syscall in the message loop.
        if written % 256 == 0 {
            progress.bytes.store(file_size(output), Ordering::Relaxed);
        }
    }

    writer.finish()?;
    report.bytes = file_size(output);
    progress.bytes.store(report.bytes, Ordering::Relaxed);
    Ok(report)
}
