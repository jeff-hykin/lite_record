//! Turning an image topic in a recording into an mp4, by piping raw frames
//! into ffmpeg. A port of the `to_vid` Deno tool, reading mcap.
//!
//! These videos are a sanity check on a recording, not a deliverable, so the
//! defaults trade a lot of size for quality nobody is going to inspect.

use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};

use anyhow::{bail, Context, Result};
use clap::Args;

use crate::cdr::CdrReader;
use crate::image::{decode_jpeg, decode_jpegxl, decode_png, decode_webp};
use crate::msgs::{RawImage, COMPRESSED_IMAGE_TYPE, IMAGE_TYPE};
use crate::topics::{schema_name, Recording};

#[derive(Args, Debug, Clone)]
pub struct Options {
    pub recording: PathBuf,

    /// Image topic, with or without its leading slash.
    #[arg(required_unless_present = "list")]
    pub topic: Option<String>,

    /// Defaults to `<recording>_<topic>.mp4` beside the input.
    pub output: Option<PathBuf>,

    /// Override the frame rate (default: the stream's own, from its stamps).
    #[arg(long)]
    pub fps: Option<f64>,

    /// x264 quality, lower is better.
    #[arg(long, default_value_t = 28)]
    pub crf: u32,

    /// Scale the long edge down to this many pixels.
    #[arg(long)]
    pub scale: Option<u32>,

    /// Use every Nth frame.
    #[arg(long, default_value_t = 1)]
    pub stride: usize,

    /// Metres mapped to white for 16-bit depth.
    #[arg(long, default_value_t = 8.0)]
    pub depth_range: f64,

    /// List the image topics in the recording and exit.
    #[arg(long)]
    pub list: bool,

    /// Seconds into the stream to begin.
    #[arg(long)]
    pub start: Option<f64>,

    /// Seconds of the stream to encode.
    #[arg(long)]
    pub duration: Option<f64>,
}

/// One frame as ffmpeg will receive it.
#[derive(Debug, PartialEq)]
pub struct Frame {
    pub width: usize,
    pub height: usize,
    pub pixel_format: &'static str,
    pub data: Vec<u8>,
}

/// ffmpeg's pixel format for a ROS encoding, plus how many bytes a pixel takes.
fn pixel_format(encoding: &str) -> Option<(&'static str, usize)> {
    Some(match encoding {
        "mono8" | "8UC1" => ("gray", 1),
        "rgb8" => ("rgb24", 3),
        "bgr8" => ("bgr24", 3),
        "rgba8" => ("rgba", 4),
        "bgra8" => ("bgra", 4),
        "mono16" | "16UC1" => ("gray16le", 2),
        _ => return None,
    })
}

/// Sniffed from the bytes: the `format` field is free text and a mislabelled
/// payload should be decoded by what it is, not what it claims.
fn codec_of(data: &[u8]) -> Option<&'static str> {
    const JXL_CONTAINER: [u8; 12] = [0x00, 0x00, 0x00, 0x0C, b'J', b'X', b'L', b' ', 0x0D, 0x0A, 0x87, 0x0A];
    if data.starts_with(&[0xFF, 0x0A]) || data.starts_with(&JXL_CONTAINER) {
        Some("jxl")
    } else if data.len() >= 12 && &data[..4] == b"RIFF" && &data[8..12] == b"WEBP" {
        Some("webp")
    } else if data.starts_with(&[0x89, b'P', b'N', b'G']) {
        Some("png")
    } else if data.starts_with(&[0xFF, 0xD8]) {
        Some("jpeg")
    } else {
        None
    }
}

/// The pixels of a `sensor_msgs/Image` or `sensor_msgs/CompressedImage`.
pub fn decode_image(schema: &str, payload: &[u8]) -> Result<RawImage> {
    let mut reader = CdrReader::little_endian(payload)?;
    let short = || anyhow::anyhow!("{schema} is truncated");
    let header = reader.try_header().ok_or_else(short)?;
    match schema {
        IMAGE_TYPE => {
            let height = reader.try_u32().ok_or_else(short)? as usize;
            let width = reader.try_u32().ok_or_else(short)? as usize;
            let encoding = reader.try_string().ok_or_else(short)?;
            let is_bigendian = reader.try_u8().ok_or_else(short)?;
            let step = reader.try_u32().ok_or_else(short)? as usize;
            let data = reader.try_borrowed_bytes().ok_or_else(short)?.to_vec();
            Ok(RawImage {
                header,
                width,
                height,
                step,
                is_bigendian,
                encoding,
                data,
            })
        }
        COMPRESSED_IMAGE_TYPE => {
            let format = reader.try_string().ok_or_else(short)?;
            let data = reader.try_borrowed_bytes().ok_or_else(short)?;
            let mut image = match codec_of(data) {
                Some("jxl") => decode_jpegxl(data)?,
                Some("webp") => decode_webp(data)?,
                Some("png") => decode_png(data)?,
                Some("jpeg") => decode_jpeg(data)?,
                _ => bail!("payload labelled {format:?} is not jxl, webp, png or jpeg"),
            };
            image.header = header;
            Ok(image)
        }
        other => bail!("{other} is not an image type"),
    }
}

/// Depth is 16-bit millimetres; squash it to 8-bit so the mp4 is watchable.
fn depth_to_gray(data: &[u8], is_bigendian: bool, range_millimetres: f64) -> Vec<u8> {
    data.as_chunks::<2>()
        .0
        .iter()
        .map(|pair| {
            let pair = *pair;
            let millimetres = if is_bigendian { u16::from_be_bytes(pair) } else { u16::from_le_bytes(pair) };
            match millimetres {
                0 => 0,
                depth => ((depth as f64 / range_millimetres) * 255.0).round().min(255.0) as u8,
            }
        })
        .collect()
}

/// A decoded image as the bytes ffmpeg gets. A 16-bit depth frame is
/// converted to 8-bit greyscale on the way past, so the pixel format names
/// what ffmpeg will actually receive rather than what was stored.
pub fn to_frame(image: RawImage, depth_range_metres: f64) -> Result<Frame> {
    let (pixel_format, bytes_per_pixel) = pixel_format(&image.encoding)
        .with_context(|| format!("unsupported encoding {:?}", image.encoding))?;
    let (pixel_format, data) = match pixel_format {
        "gray16le" => (
            "gray",
            depth_to_gray(&image.data, image.is_bigendian != 0, depth_range_metres * 1000.0),
        ),
        _ => {
            if image.data.len() != image.width * image.height * bytes_per_pixel {
                bail!(
                    "a {}x{} {} frame should be {} bytes, not {}",
                    image.width,
                    image.height,
                    image.encoding,
                    image.width * image.height * bytes_per_pixel,
                    image.data.len()
                );
            }
            (pixel_format, image.data)
        }
    };
    Ok(Frame {
        width: image.width,
        height: image.height,
        pixel_format,
        data,
    })
}

fn is_image_type(schema: &str) -> bool {
    schema == IMAGE_TYPE || schema == COMPRESSED_IMAGE_TYPE
}

/// Frames per second from the median gap between stamps, as the Deno tool
/// measured it, so a stream with one dropped frame is not reported slow.
fn measured_fps(stamps: &[u64]) -> f64 {
    let mut gaps: Vec<u64> = stamps.windows(2).map(|pair| pair[1] - pair[0]).collect();
    gaps.sort_unstable();
    let median = gaps.get(gaps.len() / 2).copied().filter(|gap| *gap > 0).unwrap_or(1_000_000_000 / 30) as f64
        / 1e9;
    (10.0 / median).round() / 10.0
}

fn list(recording: &Recording) -> Result<()> {
    for channel in recording.channels()? {
        if !is_image_type(schema_name(&channel)) {
            continue;
        }
        let stamps = recording.stamps(channel.id)?;
        let rate = match (stamps.first(), stamps.last()) {
            (Some(first), Some(last)) if last > first => {
                (stamps.len() - 1) as f64 / ((last - first) as f64 / 1e9)
            }
            _ => 0.0,
        };
        let described = match recording.messages(channel.id, None)?.next() {
            Some(Ok(message)) => match decode_image(schema_name(&channel), &message.data) {
                Ok(image) => format!("{}x{} {}", image.width, image.height, image.encoding),
                Err(error) => format!("undecodable: {error}"),
            },
            _ => "empty".to_string(),
        };
        println!(
            "  {:<40} {:>8} frames  {:>5.1} fps  {}",
            channel.topic,
            stamps.len(),
            rate,
            described
        );
    }
    Ok(())
}

pub fn run(options: &Options) -> Result<()> {
    let recording = Recording::open(&options.recording)?;
    if options.list {
        return list(&recording);
    }
    let topic = options.topic.as_deref().context("a topic is required unless --list is given")?;
    let channel = recording.channel(topic)?;
    let schema = schema_name(&channel).to_string();
    if !is_image_type(&schema) {
        bail!("{} is {schema}, not an image topic. Try --list.", channel.topic);
    }

    let stamps = recording.stamps(channel.id)?;
    let Some(&first_stamp) = stamps.first() else {
        bail!("no frames in {}. Try --list.", channel.topic);
    };
    let window = match (options.start, options.duration) {
        (None, None) => None,
        (start, duration) => {
            let low = first_stamp + (start.unwrap_or(0.0) * 1e9) as u64;
            let high = duration.map_or(u64::MAX, |seconds| low + (seconds * 1e9) as u64);
            Some((low, high))
        }
    };
    let stamps: Vec<u64> = stamps
        .into_iter()
        .filter(|stamp| window.is_none_or(|(low, high)| (low..=high).contains(stamp)))
        .collect();
    if stamps.is_empty() {
        bail!("no frames of {} inside the requested window", channel.topic);
    }
    let stride = options.stride.max(1);
    let fps = options.fps.unwrap_or_else(|| measured_fps(&stamps) / stride as f64);

    let mut messages = recording.messages(channel.id, window)?;
    let first = messages
        .next()
        .context("no frames")??;
    let first = to_frame(decode_image(&schema, &first.data)?, options.depth_range)?;
    let target = options.output.clone().unwrap_or_else(|| {
        let stem = options.recording.file_stem().unwrap_or_default().to_string_lossy();
        let sanitized = channel.topic.trim_start_matches('/').replace('/', "_");
        options.recording.with_file_name(format!("{stem}_{sanitized}.mp4"))
    });
    println!(
        "to_video: {} {}x{} -> {}",
        channel.topic, first.width, first.height, first.pixel_format
    );
    println!(
        "to_video: {} frames over {:.1}s at {fps} fps",
        stamps.len(),
        (stamps[stamps.len() - 1] - stamps[0]) as f64 / 1e9
    );

    let mut arguments: Vec<String> = [
        "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo",
        "-pix_fmt", first.pixel_format,
        "-s", &format!("{}x{}", first.width, first.height),
        "-r", &fps.to_string(),
        "-i", "pipe:0",
    ]
    .iter()
    .map(|argument| argument.to_string())
    .collect();
    if let Some(scale) = options.scale {
        arguments.push("-vf".into());
        arguments.push(format!("scale={scale}:-2"));
    }
    arguments.extend(
        ["-c:v", "libx264", "-preset", "veryfast", "-crf", &options.crf.to_string(), "-pix_fmt", "yuv420p"]
            .iter()
            .map(|argument| argument.to_string()),
    );
    arguments.push(target.to_string_lossy().into_owned());
    let mut ffmpeg = Command::new("ffmpeg")
        .args(&arguments)
        .stdin(Stdio::piped())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .spawn()
        .context("could not start ffmpeg; is it on PATH?")?;
    let mut stdin = std::io::BufWriter::with_capacity(1 << 20, ffmpeg.stdin.take().expect("piped stdin"));

    let expected = first.data.len();
    let total = stamps.len().div_ceil(stride);
    let mut written = 0usize;
    let mut skipped = 0usize;
    let mut undecodable = 0usize;
    let mut index = 0usize;
    let mut pending = Some(first);
    loop {
        let frame = match pending.take() {
            Some(frame) => Some(frame),
            None => match messages.next() {
                None => break,
                Some(message) => {
                    let message = message?;
                    index += 1;
                    if !index.is_multiple_of(stride) {
                        continue;
                    }
                    match decode_image(&schema, &message.data).and_then(|image| to_frame(image, options.depth_range)) {
                        Ok(frame) => Some(frame),
                        Err(_) => {
                            undecodable += 1;
                            None
                        }
                    }
                }
            },
        };
        let Some(frame) = frame else { continue };
        if frame.data.len() != expected {
            // A frame of the wrong size would desync every frame after it.
            skipped += 1;
            continue;
        }
        stdin.write_all(&frame.data).context("ffmpeg stopped reading")?;
        written += 1;
        if written.is_multiple_of(500) {
            println!("to_video: {written}/{total} frames");
        }
    }
    stdin.flush()?;
    drop(stdin);
    let status = ffmpeg.wait()?;

    if skipped > 0 {
        eprintln!("to_video: skipped {skipped} frames whose size did not match the first");
    }
    if undecodable > 0 {
        eprintln!("to_video: skipped {undecodable} frames that would not decode");
    }
    if !status.success() {
        bail!("ffmpeg exited {}", status.code().unwrap_or(-1));
    }
    let size = std::fs::metadata(&target).map(|data| data.len()).unwrap_or(0);
    println!(
        "to_video: {written} frames, {:.1} MB -> {}",
        size as f64 / 1e6,
        target.display()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr;
    use crate::image::{compress, ImageFormat};
    use crate::msgs::{CompressedImage, Header};
    use crate::topics::test_support::{scratch, stamped, write_recording};

    fn raw(width: usize, height: usize, encoding: &str, bytes_per_pixel: usize) -> RawImage {
        RawImage {
            header: Header::new(1_000_000_000, "camera"),
            width,
            height,
            step: width * bytes_per_pixel,
            is_bigendian: 0,
            encoding: encoding.into(),
            data: (0..width * height * bytes_per_pixel).map(|index| (index % 251) as u8).collect(),
        }
    }

    #[test]
    fn raw_colour_and_mono_frames_pass_through_at_their_own_size() {
        let colour = cdr::raw_image(&raw(8, 4, "rgb8", 3));
        let frame = to_frame(decode_image(IMAGE_TYPE, &colour.data).unwrap(), 8.0).unwrap();
        assert_eq!((frame.width, frame.height, frame.pixel_format), (8, 4, "rgb24"));
        assert_eq!(frame.data.len(), 8 * 4 * 3);

        let mono = cdr::raw_image(&raw(8, 4, "mono8", 1));
        let frame = to_frame(decode_image(IMAGE_TYPE, &mono.data).unwrap(), 8.0).unwrap();
        assert_eq!((frame.pixel_format, frame.data.len()), ("gray", 32));
    }

    #[test]
    fn sixteen_bit_depth_is_squashed_to_eight_bit_grey_by_the_range() {
        let mut depth = raw(4, 1, "16UC1", 2);
        depth.data = [0u16, 4000, 8000, 60000]
            .iter()
            .flat_map(|millimetres| millimetres.to_le_bytes())
            .collect();
        let encoded = cdr::raw_image(&depth);
        let frame = to_frame(decode_image(IMAGE_TYPE, &encoded.data).unwrap(), 8.0).unwrap();
        assert_eq!(frame.pixel_format, "gray");
        assert_eq!(frame.data, vec![0, 128, 255, 255]);

        depth.is_bigendian = 1;
        depth.data = 4000u16.to_be_bytes().repeat(4);
        let frame = to_frame(decode_image(IMAGE_TYPE, &cdr::raw_image(&depth).data).unwrap(), 8.0).unwrap();
        assert_eq!(frame.data, vec![128; 4]);
    }

    #[test]
    fn compressed_webp_and_png_frames_decode_by_their_magic_not_their_label() {
        let source = raw(6, 5, "rgb8", 3);
        let webp = compress(&source, ImageFormat::Webp).unwrap();
        let mislabelled = CompressedImage {
            format: "jpeg".into(),
            ..webp
        };
        let frame = to_frame(
            decode_image(COMPRESSED_IMAGE_TYPE, &cdr::compressed_image(&mislabelled).data).unwrap(),
            8.0,
        )
        .unwrap();
        assert_eq!((frame.width, frame.height, frame.pixel_format), (6, 5, "rgb24"));
        assert_eq!(frame.data, source.data);

        let png = compress(&raw(6, 5, "mono8", 1), ImageFormat::Png).unwrap();
        let frame = to_frame(
            decode_image(COMPRESSED_IMAGE_TYPE, &cdr::compressed_image(&png).data).unwrap(),
            8.0,
        )
        .unwrap();
        assert_eq!((frame.pixel_format, frame.data.len()), ("gray", 30));
    }

    #[test]
    fn an_unknown_encoding_and_a_short_frame_are_refused() {
        let error = to_frame(raw(2, 2, "yuv422", 2), 8.0).unwrap_err().to_string();
        assert!(error.contains("unsupported encoding"), "{error}");
        let mut short = raw(2, 2, "rgb8", 3);
        short.data.pop();
        assert!(to_frame(short, 8.0).is_err());
    }

    #[test]
    fn the_frame_rate_is_the_median_gap_so_one_dropped_frame_does_not_slow_it() {
        let step = 1_000_000_000 / 30;
        let mut stamps: Vec<u64> = (0..20).map(|index| index * step).collect();
        stamps.remove(7);
        assert_eq!(measured_fps(&stamps), 30.0);
        assert_eq!(measured_fps(&[5]), 30.0);
    }

    #[test]
    fn a_short_stream_encodes_to_an_mp4_through_ffmpeg() {
        if Command::new("ffmpeg").arg("-version").output().is_err() {
            eprintln!("ffmpeg is not on PATH; skipping");
            return;
        }
        let directory = scratch("video_ffmpeg");
        let recording = directory.join("frames.mcap");
        let step = 1_000_000_000 / 10;
        let frames: Vec<_> = (0..12u64)
            .map(|index| {
                let mut image = raw(32, 16, "rgb8", 3);
                image.header = Header::new(index * step, "camera");
                stamped("/camera/image", cdr::raw_image(&image), index * step)
            })
            .collect();
        write_recording(&recording, &[frames]);
        let options = Options {
            recording: recording.clone(),
            topic: Some("camera/image".into()),
            output: None,
            fps: None,
            crf: 28,
            scale: None,
            stride: 2,
            depth_range: 8.0,
            list: false,
            start: None,
            duration: None,
        };
        run(&options).unwrap();
        let output = directory.join("frames_camera_image.mp4");
        assert!(std::fs::metadata(&output).unwrap().len() > 0, "{}", output.display());
    }

    #[test]
    fn listing_names_every_image_topic_with_its_layout() {
        let directory = scratch("video_list");
        let recording = directory.join("list.mcap");
        write_recording(
            &recording,
            &[vec![
                stamped("/camera/image", cdr::raw_image(&raw(4, 2, "rgb8", 3)), 0),
                stamped("/camera/image", cdr::raw_image(&raw(4, 2, "rgb8", 3)), 100_000_000),
                stamped("/imu", cdr::imu(&crate::msgs::Imu::unoriented(Header::new(0, "imu"), [0.0; 3], [0.0; 3])), 0),
            ]],
        );
        let opened = Recording::open(&recording).unwrap();
        let channel = opened.channel("/camera/image").unwrap();
        assert_eq!(opened.stamps(channel.id).unwrap().len(), 2);
        list(&opened).unwrap();
        assert!(opened.channels().unwrap().iter().filter(|channel| is_image_type(schema_name(channel))).count() == 1);
    }
}
