use crate::msgs::{CompressedImage, Header, RawImage};
use anyhow::{anyhow, bail, Context, Result};
use bytes::Bytes;
use jpeg_encoder::{ColorType, Encoder};
use serde::{Deserialize, Serialize};
use std::borrow::Cow;
use std::io::Cursor;

pub struct EncodedFrame {
    pub jpeg: Bytes,
    pub width: usize,
    pub height: usize,
    /// True when the browser is getting the publisher's own bytes rather than ours.
    pub passthrough: bool,
}

/// Recorded frames are archival, so this sits far above the streaming quality.
const RECORD_JPEG_QUALITY: u8 = 92;

/// How image topics are stored in a recording. `Raw` keeps the frame exactly as
/// it arrived; the rest re-encode it as a `sensor_msgs/CompressedImage`.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug, Default)]
#[serde(rename_all = "lowercase")]
pub enum ImageFormat {
    #[default]
    Raw,
    Jpeg,
    Png,
    Webp,
    Jpegxl,
}

impl ImageFormat {
    /// The `format` field a `CompressedImage` reader keys off.
    fn label(self) -> &'static str {
        match self {
            ImageFormat::Raw => "raw",
            ImageFormat::Jpeg => "jpeg",
            ImageFormat::Png => "png",
            ImageFormat::Webp => "webp",
            ImageFormat::Jpegxl => "jxl",
        }
    }
}

/// Pixel layouts the recording encoders share. Recording keeps the source
/// resolution and bit depth — a recording that quietly threw precision away
/// would be worse than a large one.
enum Surface {
    Gray8(Vec<u8>),
    Rgb8(Vec<u8>),
    Rgba8(Vec<u8>),
    Gray16(Vec<u16>),
}

/// Re-encodes a frame for recording. `None` means the format cannot hold these
/// pixels without losing precision — jpeg and webp are 8-bit only, and none of
/// them take 32-bit float depth — so the caller keeps the raw frame instead of
/// writing a degraded one.
pub fn compress(image: &RawImage, format: ImageFormat) -> Option<CompressedImage> {
    let surface = surface(image)?;
    let width = image.width as u32;
    let height = image.height as u32;
    let data = match format {
        ImageFormat::Raw => return None,
        ImageFormat::Jpeg => to_jpeg(&surface, width, height)?,
        ImageFormat::Png => to_png(&surface, width, height)?,
        ImageFormat::Webp => to_webp(&surface, width, height)?,
        ImageFormat::Jpegxl => to_jpegxl(&surface, width, height)?,
    };
    Some(CompressedImage {
        header: image.header.clone(),
        format: format.label().to_owned(),
        data,
    })
}

/// Repacks the rows into a tight buffer, dropping any `step` padding and
/// putting the channels in the order every encoder here expects.
fn surface(image: &RawImage) -> Option<Surface> {
    let bytes_per_pixel = match image.encoding.as_str() {
        "mono8" | "8UC1" => 1,
        "rgb8" | "8UC3" | "bgr8" => 3,
        "rgba8" | "8UC4" | "bgra8" => 4,
        "mono16" | "16UC1" | "depth16" => 2,
        _ => return None,
    };
    let tight = image.width.checked_mul(bytes_per_pixel)?;
    let step = if image.step > 0 { image.step } else { tight };
    if step < tight || image.data.len() < step.checked_mul(image.height)? {
        return None;
    }
    let rows = (0..image.height).map(|row| &image.data[row * step..row * step + tight]);

    if bytes_per_pixel == 2 {
        let big_endian = image.is_bigendian != 0;
        return Some(Surface::Gray16(
            rows.flat_map(|row| row.as_chunks::<2>().0.iter().copied())
                .map(|pair| {
                    if big_endian {
                        u16::from_be_bytes(pair)
                    } else {
                        u16::from_le_bytes(pair)
                    }
                })
                .collect(),
        ));
    }

    let mut packed: Vec<u8> = rows.flatten().copied().collect();
    if image.encoding.starts_with("bgr") {
        for pixel in packed.chunks_exact_mut(bytes_per_pixel) {
            pixel.swap(0, 2);
        }
    }
    match bytes_per_pixel {
        1 => Some(Surface::Gray8(packed)),
        3 => Some(Surface::Rgb8(packed)),
        _ => Some(Surface::Rgba8(packed)),
    }
}

fn to_jpeg(surface: &Surface, width: u32, height: u32) -> Option<Vec<u8>> {
    let (bytes, color) = match surface {
        Surface::Gray8(bytes) => (bytes.as_slice(), ColorType::Luma),
        Surface::Rgb8(bytes) => (bytes.as_slice(), ColorType::Rgb),
        Surface::Rgba8(bytes) => (bytes.as_slice(), ColorType::Rgba),
        Surface::Gray16(_) => return None,
    };
    let mut out = Vec::new();
    Encoder::new(&mut out, RECORD_JPEG_QUALITY)
        .encode(bytes, width as u16, height as u16, color)
        .ok()?;
    Some(out)
}

fn to_png(surface: &Surface, width: u32, height: u32) -> Option<Vec<u8>> {
    let (color, depth, bytes) = match surface {
        Surface::Gray8(b) => (png::ColorType::Grayscale, png::BitDepth::Eight, Cow::from(b)),
        Surface::Rgb8(b) => (png::ColorType::Rgb, png::BitDepth::Eight, Cow::from(b)),
        Surface::Rgba8(b) => (png::ColorType::Rgba, png::BitDepth::Eight, Cow::from(b)),
        // png stores 16-bit samples big-endian regardless of the host.
        Surface::Gray16(values) => (
            png::ColorType::Grayscale,
            png::BitDepth::Sixteen,
            Cow::from(values.iter().flat_map(|value| value.to_be_bytes()).collect::<Vec<u8>>()),
        ),
    };
    let mut out = Vec::new();
    let mut encoder = png::Encoder::new(&mut out, width, height);
    encoder.set_color(color);
    encoder.set_depth(depth);
    let mut writer = encoder.write_header().ok()?;
    writer.write_image_data(&bytes).ok()?;
    writer.finish().ok()?;
    Some(out)
}

fn to_webp(surface: &Surface, width: u32, height: u32) -> Option<Vec<u8>> {
    let (bytes, color) = match surface {
        Surface::Gray8(b) => (b.as_slice(), image_webp::ColorType::L8),
        Surface::Rgb8(b) => (b.as_slice(), image_webp::ColorType::Rgb8),
        Surface::Rgba8(b) => (b.as_slice(), image_webp::ColorType::Rgba8),
        Surface::Gray16(_) => return None,
    };
    let mut out = Vec::new();
    image_webp::WebPEncoder::new(&mut out)
        .encode(bytes, width, height, color)
        .ok()?;
    Some(out)
}

fn to_jpegxl(surface: &Surface, width: u32, height: u32) -> Option<Vec<u8>> {
    use zune_core::colorspace::ColorSpace;
    use zune_core::bit_depth::BitDepth;

    let (bytes, colorspace, depth) = match surface {
        Surface::Gray8(b) => (Cow::from(b), ColorSpace::Luma, BitDepth::Eight),
        Surface::Rgb8(b) => (Cow::from(b), ColorSpace::RGB, BitDepth::Eight),
        Surface::Rgba8(b) => (Cow::from(b), ColorSpace::RGBA, BitDepth::Eight),
        // zune reads 16-bit samples as native-endian byte pairs.
        Surface::Gray16(values) => (
            Cow::from(values.iter().flat_map(|value| value.to_ne_bytes()).collect::<Vec<u8>>()),
            ColorSpace::Luma,
            BitDepth::Sixteen,
        ),
    };
    let options =
        zune_core::options::EncoderOptions::new(width as usize, height as usize, colorspace, depth);
    let mut out = Vec::new();
    zune_jpegxl::JxlSimpleEncoder::new(&bytes, options)
        .encode(&mut out)
        .ok()?;
    Some(out)
}

#[derive(Clone, Copy, PartialEq)]
enum Pixels {
    Gray,
    Rgb,
}

const PNG_MAGIC: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];

#[derive(Clone, Copy, PartialEq, Debug)]
enum Codec {
    Jpeg,
    Png,
}

/// Sniffed rather than taken from the `format` or `encoding` string, because those
/// are free text and a mislabelled payload should be rejected, not served broken.
fn codec_of(data: &[u8]) -> Option<Codec> {
    if data.starts_with(&[0xFF, 0xD8]) {
        Some(Codec::Jpeg)
    } else if data.starts_with(&PNG_MAGIC) {
        Some(Codec::Png)
    } else {
        None
    }
}

/// dimos's `JpegLcmTransport` sends an ordinary `sensor_msgs/Image` whose `data`
/// is a whole compressed stream and whose `step` is 0, so the label is the only
/// hint that the payload is not pixels.
/// A conformant raw frame fills exactly `height * step` bytes and a codec stream
/// essentially never does, so this settles the question structurally. Magic bytes
/// alone would not: a bright mono8 row can genuinely open with ff d8 ff.
pub fn looks_like_pixels(height: usize, step: usize, length: usize) -> bool {
    step != 0 && height.checked_mul(step) == Some(length)
}

/// Neither the right size for its own dimensions nor a container we recognise, so
/// the frame is truncated, mis-measured, or in a codec we do not sniff. Counted
/// rather than logged per frame, since at 60 Hz a warn line would be unreadable.
pub fn frame_is_unclassifiable(height: usize, step: usize, data: &[u8]) -> bool {
    !looks_like_pixels(height, step, data.len()) && codec_of(data).is_none()
}

fn compressed_codec(image: &RawImage) -> Option<Codec> {
    if !matches!(image.encoding.as_str(), "jpeg" | "jpg" | "png") {
        return None;
    }
    if looks_like_pixels(image.height, image.step, image.data.len()) {
        return None;
    }
    codec_of(&image.data)
}

/// The container name to record such a frame under, since writing it back out as
/// an `Image` would claim a codec stream is a pixel layout and give it `step = 0`.
pub fn container_format(image: &RawImage) -> Option<&'static str> {
    match compressed_codec(image)? {
        Codec::Jpeg => Some("jpeg"),
        Codec::Png => Some("png"),
    }
}

fn decode_jpeg(data: &[u8]) -> Result<RawImage> {
    let mut decoder = zune_jpeg::JpegDecoder::new(Cursor::new(data));
    let pixels = decoder
        .decode()
        .map_err(|error| anyhow!("jpeg decode failed: {error}"))?;
    let (width, height) = decoder.dimensions().context("jpeg carries no frame header")?;
    let encoding = match decoder.output_colorspace() {
        Some(zune_core::colorspace::ColorSpace::Luma) => "mono8",
        Some(zune_core::colorspace::ColorSpace::RGB) => "rgb8",
        other => bail!("unsupported jpeg colorspace: {other:?}"),
    };
    Ok(decoded_image(width, height, encoding, pixels))
}

fn decode_png(data: &[u8]) -> Result<RawImage> {
    let mut decoder = png::Decoder::new(Cursor::new(data));
    // Turns a palette or a sub-byte bit depth into plain 8-bit samples, so the
    // match below only has to cover the layouts the raw path already knows.
    decoder.set_transformations(png::Transformations::EXPAND);
    let mut reader = decoder.read_info()?;
    let size = reader
        .output_buffer_size()
        .context("png dimensions overflow a buffer")?;
    let mut pixels = vec![0u8; size];
    let info = reader.next_frame(&mut pixels)?;
    pixels.truncate(info.buffer_size());
    let encoding = match (info.color_type, info.bit_depth) {
        (png::ColorType::Grayscale, png::BitDepth::Eight) => "mono8",
        (png::ColorType::Rgb, png::BitDepth::Eight) => "rgb8",
        (png::ColorType::Rgba, png::BitDepth::Eight) => "rgba8",
        (png::ColorType::Grayscale, png::BitDepth::Sixteen) => {
            // png always stores 16-bit samples big-endian, and `is_bigendian` is
            // set below rather than swapping a depth frame's worth of bytes here.
            return Ok(RawImage {
                is_bigendian: 1,
                ..decoded_image(info.width as usize, info.height as usize, "mono16", pixels)
            });
        }
        (color, depth) => bail!("unsupported png layout: {color:?} at {depth:?} bits"),
    };
    Ok(decoded_image(
        info.width as usize,
        info.height as usize,
        encoding,
        pixels,
    ))
}

fn decoded_image(width: usize, height: usize, encoding: &str, data: Vec<u8>) -> RawImage {
    RawImage {
        header: Header {
            stamp_sec: 0,
            stamp_nsec: 0,
            frame_id: String::new(),
        },
        width,
        height,
        step: 0,
        is_bigendian: 0,
        encoding: encoding.to_owned(),
        data,
    }
}

/// Reads just the header, so deciding whether a frame needs resizing costs a few
/// microseconds instead of a full decode.
fn compressed_size(codec: Codec, data: &[u8]) -> Result<(usize, usize)> {
    match codec {
        Codec::Jpeg => {
            let mut decoder = zune_jpeg::JpegDecoder::new(Cursor::new(data));
            decoder
                .decode_headers()
                .map_err(|error| anyhow!("jpeg header unreadable: {error}"))?;
            decoder.dimensions().context("jpeg carries no frame header")
        }
        Codec::Png => {
            let reader = png::Decoder::new(Cursor::new(data)).read_info()?;
            let info = reader.info();
            Ok((info.width as usize, info.height as usize))
        }
    }
}

fn kind_of(encoding: &str) -> Option<(Pixels, usize, bool)> {
    let (pixels, bytes_per_pixel, is_depth) = match encoding {
        "mono8" | "8UC1" => (Pixels::Gray, 1, false),
        "rgb8" | "8UC3" => (Pixels::Rgb, 3, false),
        "bgr8" => (Pixels::Rgb, 3, false),
        "rgba8" | "8UC4" => (Pixels::Rgb, 4, false),
        "bgra8" => (Pixels::Rgb, 4, false),
        "mono16" | "16UC1" | "depth16" => (Pixels::Rgb, 2, true),
        "32FC1" => (Pixels::Rgb, 4, true),
        _ => return None,
    };
    Some((pixels, bytes_per_pixel, is_depth))
}

/// Downscale and colour-convert in one pass, then JPEG encode.
///
/// Sampling is nearest neighbour: at the frame rates this streams, a cheap
/// resample that keeps up beats a pretty one that forces frames to be dropped.
pub fn encode(image: &RawImage, quality: u8, max_width: usize) -> Result<EncodedFrame> {
    match compressed_codec(image) {
        Some(codec) => encode_payload(codec, &image.data, quality, max_width),
        None => encode_raw(image, quality, max_width),
    }
}

/// A `sensor_msgs/CompressedImage`, whose whole point is that `data` is a codec
/// stream. The `format` field is ignored in favour of the magic bytes.
pub fn encode_compressed(
    image: &CompressedImage,
    quality: u8,
    max_width: usize,
) -> Result<EncodedFrame> {
    let Some(codec) = codec_of(&image.data) else {
        bail!("compressed format {} is not viewable", image.format);
    };
    encode_payload(codec, &image.data, quality, max_width)
}

/// A jpeg that already fits goes to the browser untouched, skipping a decode and a
/// re-encode per frame per viewer. Everything else is decoded to pixels and run
/// through the ordinary path, which is what makes png and oversized jpeg viewable.
fn encode_payload(
    codec: Codec,
    data: &[u8],
    quality: u8,
    max_width: usize,
) -> Result<EncodedFrame> {
    let (width, height) = compressed_size(codec, data)?;
    if codec == Codec::Jpeg && (max_width == 0 || width <= max_width) {
        return Ok(EncodedFrame {
            jpeg: Bytes::copy_from_slice(data),
            width,
            height,
            passthrough: true,
        });
    }
    let decoded = match codec {
        Codec::Jpeg => decode_jpeg(data)?,
        Codec::Png => decode_png(data)?,
    };
    encode_raw(&decoded, quality, max_width)
}

fn encode_raw(image: &RawImage, quality: u8, max_width: usize) -> Result<EncodedFrame> {
    let Some((pixels, bytes_per_pixel, is_depth)) = kind_of(&image.encoding) else {
        bail!("unsupported image encoding: {}", image.encoding);
    };
    let step = if image.step > 0 {
        image.step
    } else {
        image.width * bytes_per_pixel
    };
    if image.data.len() < step * image.height {
        bail!("image payload is shorter than height * step");
    }

    let (width, height) = fit(image.width, image.height, max_width);
    let channels = if pixels == Pixels::Gray { 1 } else { 3 };
    let mut out = vec![0u8; width * height * channels];

    if is_depth {
        let samples = sample_depth(image, step, bytes_per_pixel, width, height);
        let (low, high) = span(&samples);
        for (index, value) in samples.iter().enumerate() {
            let color = match value {
                Some(value) => turbo((value - low) / (high - low)),
                None => [0, 0, 0],
            };
            out[index * 3..index * 3 + 3].copy_from_slice(&color);
        }
    } else {
        let swap_red_blue = image.encoding.starts_with("bgr");
        for y in 0..height {
            let source_row = y * image.height / height;
            for x in 0..width {
                let source_column = x * image.width / width;
                let source = source_row * step + source_column * bytes_per_pixel;
                let target = (y * width + x) * channels;
                if channels == 1 {
                    out[target] = image.data[source];
                } else if swap_red_blue {
                    out[target] = image.data[source + 2];
                    out[target + 1] = image.data[source + 1];
                    out[target + 2] = image.data[source];
                } else {
                    out[target..target + 3].copy_from_slice(&image.data[source..source + 3]);
                }
            }
        }
    }

    let color_type = if channels == 1 {
        ColorType::Luma
    } else {
        ColorType::Rgb
    };
    let mut jpeg = Vec::with_capacity(width * height / 4);
    Encoder::new(&mut jpeg, quality).encode(&out, width as u16, height as u16, color_type)?;

    Ok(EncodedFrame {
        jpeg: Bytes::from(jpeg),
        width,
        height,
        passthrough: false,
    })
}

fn fit(width: usize, height: usize, max_width: usize) -> (usize, usize) {
    if width <= max_width || max_width == 0 {
        return (width.max(1), height.max(1));
    }
    let scaled_height = height * max_width / width.max(1);
    (max_width, scaled_height.max(1))
}

fn sample_depth(
    image: &RawImage,
    step: usize,
    bytes_per_pixel: usize,
    width: usize,
    height: usize,
) -> Vec<Option<f32>> {
    let mut samples = Vec::with_capacity(width * height);
    for y in 0..height {
        let source_row = y * image.height / height;
        for x in 0..width {
            let source_column = x * image.width / width;
            let source = source_row * step + source_column * bytes_per_pixel;
            let value = if bytes_per_pixel == 2 {
                let pair = [image.data[source], image.data[source + 1]];
                let raw = if image.is_bigendian != 0 {
                    u16::from_be_bytes(pair)
                } else {
                    u16::from_le_bytes(pair)
                };
                if raw == 0 {
                    None
                } else {
                    Some(raw as f32)
                }
            } else {
                let raw = f32::from_le_bytes(image.data[source..source + 4].try_into().unwrap());
                if raw.is_finite() && raw > 0.0 {
                    Some(raw)
                } else {
                    None
                }
            };
            samples.push(value);
        }
    }
    samples
}

fn span(samples: &[Option<f32>]) -> (f32, f32) {
    let mut low = f32::MAX;
    let mut high = f32::MIN;
    for value in samples.iter().flatten() {
        low = low.min(*value);
        high = high.max(*value);
    }
    if low >= high {
        return (0.0, 1.0);
    }
    (low, high)
}

fn turbo(position: f32) -> [u8; 3] {
    const STOPS: [[f32; 3]; 6] = [
        [0.19, 0.07, 0.23],
        [0.11, 0.53, 0.90],
        [0.14, 0.87, 0.68],
        [0.68, 0.98, 0.24],
        [0.98, 0.68, 0.12],
        [0.73, 0.09, 0.03],
    ];
    let position = position.clamp(0.0, 1.0) * (STOPS.len() - 1) as f32;
    let index = position.floor() as usize;
    let next = (index + 1).min(STOPS.len() - 1);
    let blend = position - index as f32;
    let mut color = [0u8; 3];
    for channel in 0..3 {
        let value = STOPS[index][channel] * (1.0 - blend) + STOPS[next][channel] * blend;
        color[channel] = (value * 255.0).clamp(0.0, 255.0) as u8;
    }
    color
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gray_image(width: usize, height: usize) -> RawImage {
        RawImage {
            header: crate::msgs::Header { stamp_sec: 0, stamp_nsec: 0, frame_id: String::new() },
            is_bigendian: 0,
            width,
            height,
            step: width,
            encoding: "mono8".to_owned(),
            data: (0..width * height).map(|index| index as u8).collect(),
        }
    }

    #[test]
    fn downscales_to_the_requested_width() {
        let frame = encode(&gray_image(640, 480), 70, 320).unwrap();
        assert_eq!((frame.width, frame.height), (320, 240));
        assert_eq!(&frame.jpeg[..2], &[0xff, 0xd8]);
    }

    #[test]
    fn keeps_small_images_at_native_size() {
        let frame = encode(&gray_image(64, 48), 70, 320).unwrap();
        assert_eq!((frame.width, frame.height), (64, 48));
    }

    #[test]
    fn lower_quality_produces_smaller_jpegs() {
        let mut noisy = gray_image(320, 240);
        for (index, byte) in noisy.data.iter_mut().enumerate() {
            *byte = ((index * 37) % 251) as u8;
        }
        let high = encode(&noisy, 90, 320).unwrap().jpeg.len();
        let low = encode(&noisy, 25, 320).unwrap().jpeg.len();
        assert!(low < high, "quality 25 ({low}) should beat quality 90 ({high})");
    }

    #[test]
    fn depth_frames_become_colour() {
        let image = RawImage {
            header: crate::msgs::Header { stamp_sec: 0, stamp_nsec: 0, frame_id: String::new() },
            is_bigendian: 0,
            width: 4,
            height: 2,
            step: 8,
            encoding: "16UC1".to_owned(),
            data: (0..8u16).flat_map(|value| (value * 100).to_le_bytes()).collect(),
        };
        let frame = encode(&image, 80, 640).unwrap();
        assert_eq!((frame.width, frame.height), (4, 2));
    }

    #[test]
    fn unknown_encodings_are_rejected() {
        let mut image = gray_image(8, 8);
        image.encoding = "yuv422".to_owned();
        assert!(encode(&image, 70, 320).is_err());
    }

    fn depth_image(width: usize, height: usize) -> RawImage {
        RawImage {
            header: crate::msgs::Header { stamp_sec: 0, stamp_nsec: 0, frame_id: String::new() },
            is_bigendian: 0,
            width,
            height,
            // Two bytes of row padding, so a decoder that trusts `step` blindly
            // would smear the depths sideways.
            step: width * 2 + 2,
            encoding: "16UC1".to_owned(),
            data: (0..height)
                .flat_map(|row| {
                    (0..width)
                        .flat_map(move |column| (((row * width + column) * 517) as u16).to_le_bytes())
                        .chain([0xff, 0xff])
                })
                .collect(),
        }
    }

    fn colour_image(width: usize, height: usize) -> RawImage {
        RawImage {
            header: crate::msgs::Header { stamp_sec: 0, stamp_nsec: 0, frame_id: String::new() },
            is_bigendian: 0,
            width,
            height,
            step: width * 3,
            encoding: "rgb8".to_owned(),
            data: (0..width * height * 3).map(|index| ((index * 7) % 251) as u8).collect(),
        }
    }

    /// Mirrors dimos's `JpegLcmTransport`: a plain Image whose data is a JFIF
    /// stream and whose step is 0.
    fn prejpeg_image(width: usize, height: usize) -> RawImage {
        let source = colour_image(width, height);
        let frame = encode(&source, 75, width).unwrap();
        RawImage {
            step: 0,
            encoding: "jpeg".to_owned(),
            data: frame.jpeg.to_vec(),
            ..source
        }
    }

    /// A plain Image whose data is a png stream, which is how frame-dumping tools
    /// tend to publish.
    fn prepng_image(width: usize, height: usize) -> RawImage {
        let source = colour_image(width, height);
        let compressed = compress(&source, ImageFormat::Png).unwrap();
        RawImage {
            step: 0,
            encoding: "png".to_owned(),
            data: compressed.data,
            ..source
        }
    }

    #[test]
    fn already_jpeg_frames_are_passed_through_without_re_encoding() {
        let image = prejpeg_image(32, 24);
        let frame = encode(&image, 40, 800).unwrap();
        assert_eq!(frame.jpeg.as_ref(), image.data.as_slice());
        assert!(frame.passthrough);
        // Read back from the stream rather than the message header, which is the
        // only field a `CompressedImage` does not carry at all.
        assert_eq!((frame.width, frame.height), (32, 24));
    }

    #[test]
    fn a_jpeg_wider_than_the_viewer_asked_for_is_decoded_and_scaled() {
        let image = prejpeg_image(32, 24);
        let frame = encode(&image, 40, 8).unwrap();
        assert_eq!((frame.width, frame.height), (8, 6));
        assert!(!frame.passthrough);
        assert_eq!(&frame.jpeg[..2], &[0xff, 0xd8]);
    }

    #[test]
    fn png_frames_are_decoded_and_served_as_jpeg() {
        let image = prepng_image(32, 24);
        let frame = encode(&image, 75, 800).unwrap();
        assert_eq!((frame.width, frame.height), (32, 24));
        // png never passes through: the tiles are served as jpeg.
        assert!(!frame.passthrough);
        assert_eq!(&frame.jpeg[..2], &[0xff, 0xd8]);
    }

    #[test]
    fn a_compressed_image_is_read_from_its_bytes_not_its_format_field() {
        let png = prepng_image(24, 16);
        let compressed = CompressedImage {
            header: png.header.clone(),
            format: "totally-wrong".to_owned(),
            data: png.data.clone(),
        };
        let frame = encode_compressed(&compressed, 75, 800).unwrap();
        assert_eq!((frame.width, frame.height), (24, 16));
    }

    #[test]
    fn a_compressed_image_that_is_not_a_known_codec_is_rejected() {
        let compressed = CompressedImage {
            header: crate::msgs::Header { stamp_sec: 0, stamp_nsec: 0, frame_id: String::new() },
            format: "jpeg".to_owned(),
            data: vec![0u8; 64],
        };
        assert!(encode_compressed(&compressed, 75, 800).is_err());
    }

    #[test]
    fn a_sixteen_bit_depth_png_keeps_its_samples_through_the_decoder() {
        let source = depth_image(9, 5);
        let compressed = compress(&source, ImageFormat::Png).unwrap();
        let decoded = decode_png(&compressed.data).unwrap();
        assert_eq!(decoded.encoding, "mono16");
        // png is big-endian on the wire, so a decoder that assumed the host order
        // would return byte-swapped depths here rather than the originals.
        assert_ne!(decoded.is_bigendian, 0);
        assert_eq!(depths(&decoded), depths(&source));
    }

    #[test]
    fn a_frame_labelled_jpeg_without_the_jfif_marker_is_rejected() {
        let mut image = prejpeg_image(16, 16);
        image.data[0] = 0x00;
        assert!(encode(&image, 75, 800).is_err());
    }

    #[test]
    fn recording_keeps_an_already_jpeg_frame_as_it_arrived() {
        let image = prejpeg_image(16, 16);
        assert!(compress(&image, ImageFormat::Png).is_none());
        assert!(compress(&image, ImageFormat::Jpeg).is_none());
    }

    fn depths(image: &RawImage) -> Vec<u16> {
        let Some(Surface::Gray16(values)) = surface(image) else {
            panic!("expected a 16-bit surface");
        };
        values
    }

    #[test]
    fn png_keeps_every_depth_sample_exact() {
        let image = depth_image(9, 5);
        let encoded = compress(&image, ImageFormat::Png).unwrap();
        assert_eq!(encoded.format, "png");

        let mut reader = png::Decoder::new(std::io::Cursor::new(&encoded.data))
            .read_info()
            .unwrap();
        assert_eq!(reader.info().bit_depth, png::BitDepth::Sixteen);
        let mut bytes = vec![0u8; reader.output_buffer_size().unwrap()];
        let info = reader.next_frame(&mut bytes).unwrap();
        let decoded: Vec<u16> = bytes[..info.buffer_size()]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| u16::from_be_bytes(*pair))
            .collect();
        assert_eq!(decoded, depths(&image));
    }

    #[test]
    fn jpegxl_keeps_every_depth_sample_exact() {
        let image = depth_image(9, 5);
        let encoded = compress(&image, ImageFormat::Jpegxl).unwrap();
        assert_eq!(encoded.format, "jxl");

        let render = jxl_oxide::JxlImage::builder()
            .read(std::io::Cursor::new(&encoded.data))
            .unwrap()
            .render_frame(0)
            .unwrap();
        let scale = f32::from(u16::MAX);
        let decoded: Vec<u16> = render
            .image_all_channels()
            .buf()
            .iter()
            .map(|value| (value * scale).round() as u16)
            .collect();
        assert_eq!(decoded, depths(&image));
    }

    #[test]
    fn webp_keeps_every_colour_pixel_exact() {
        let image = colour_image(11, 7);
        let encoded = compress(&image, ImageFormat::Webp).unwrap();
        assert_eq!(encoded.format, "webp");

        let mut decoder = image_webp::WebPDecoder::new(std::io::Cursor::new(&encoded.data)).unwrap();
        let mut bytes = vec![0u8; decoder.output_buffer_size().unwrap()];
        decoder.read_image(&mut bytes).unwrap();
        assert_eq!(decoder.dimensions(), (11, 7));
        assert_eq!(bytes, image.data);
    }

    #[test]
    fn lossy_and_colour_only_formats_refuse_depth() {
        let image = depth_image(9, 5);
        assert!(compress(&image, ImageFormat::Jpeg).is_none());
        assert!(compress(&image, ImageFormat::Webp).is_none());
        assert!(compress(&image, ImageFormat::Raw).is_none());
    }

    #[test]
    fn jpeg_still_encodes_colour() {
        let encoded = compress(&colour_image(16, 16), ImageFormat::Jpeg).unwrap();
        assert_eq!(encoded.format, "jpeg");
        assert_eq!(&encoded.data[..2], &[0xff, 0xd8]);
    }
}
