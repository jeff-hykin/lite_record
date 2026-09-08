//! Drives real frames through the hub's real sink and reads the compression
//! settings back out of the finished mcap.
//!
//! The unit tests in `image.rs` prove each codec round-trips. What they cannot
//! prove is that the *setting* reaches the file: that picking jpeg actually
//! renames the topic, swaps the schema to `CompressedImage` and stores a JFIF
//! stream, and that a codec which cannot hold a stream's bit depth silently
//! leaves the raw frame alone instead of truncating it. That is a property of
//! the wiring between settings, encode pool, recorder and writer, so it is
//! tested through all four.
//!
//! Only the pixels are synthetic — there is no RealSense or Orbbec on this
//! machine. Everything downstream of `hub.sink()` is the production path.

use std::collections::BTreeMap;
use std::time::Duration;

use lite_record::hub::{Hub, Settings};
use lite_record::image::ImageFormat;
use lite_record::livox::{self, parse_capture};
use lite_record::msgs::{Header, RawImage};
use lite_record::sensors::{Produced, StreamId};

/// Reads back the fields of a `sensor_msgs/CompressedImage`.
///
/// Written out rather than pulled from a library so the test checks the bytes
/// that were actually written, not this crate's own encoder run backwards.
fn read_compressed_image(message: &[u8]) -> (String, Vec<u8>) {
    // 4-byte encapsulation header, then all alignment is measured from the body.
    let body = &message[4..];
    let mut at = 0usize;

    let take = |count: usize, at: &mut usize| {
        let slice = body[*at..*at + count].to_vec();
        *at += count;
        slice
    };
    let u32_at = |at: &mut usize| {
        let padding = (4 - (*at % 4)) % 4;
        *at += padding;
        let bytes = take(4, at);
        u32::from_le_bytes(bytes.try_into().unwrap())
    };

    // header: stamp_sec, stamp_nsec, frame_id
    u32_at(&mut at);
    u32_at(&mut at);
    let frame_id_length = u32_at(&mut at) as usize;
    at += frame_id_length;

    let format_length = u32_at(&mut at) as usize;
    // The length includes the trailing nul that CDR strings carry.
    let format = String::from_utf8(take(format_length - 1, &mut at)).unwrap();
    at += 1;

    let data_length = u32_at(&mut at) as usize;
    let data = take(data_length, &mut at);
    (format, data)
}

fn colour_frame(width: usize, height: usize) -> RawImage {
    RawImage {
        header: Header::new(1_700_000_000_000_000_000, "camera_color_optical_frame"),
        width,
        height,
        step: width * 3,
        is_bigendian: 0,
        encoding: "rgb8".to_owned(),
        data: (0..width * height * 3)
            .map(|index| ((index * 7) % 251) as u8)
            .collect(),
    }
}

/// A 16-bit depth frame, whose samples are what the lossy codecs cannot hold.
fn depth_frame(width: usize, height: usize) -> RawImage {
    RawImage {
        header: Header::new(1_700_000_000_000_000_000, "camera_depth_optical_frame"),
        width,
        height,
        step: width * 2,
        is_bigendian: 0,
        encoding: "16UC1".to_owned(),
        data: (0..width * height)
            .flat_map(|index| ((index * 517) as u16).to_le_bytes())
            .collect(),
    }
}

fn quiet_settings(directory: &std::path::Path) -> Settings {
    let mut settings = Settings {
        record_dir: directory.to_path_buf(),
        ..Settings::default()
    };
    settings.livox.enabled = false;
    settings.realsense.enabled = false;
    settings.orbbec.enabled = false;
    settings.preview_enabled = false;
    settings
}

/// Every message in the file, keyed by topic, as (schema name, payloads).
fn read_back(path: &std::path::Path) -> BTreeMap<String, (String, Vec<Vec<u8>>)> {
    let bytes = std::fs::read(path).unwrap();
    let mut channels: BTreeMap<String, (String, Vec<Vec<u8>>)> = Default::default();
    for message in mcap::MessageStream::new(&bytes).unwrap() {
        let message = message.unwrap();
        let schema = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.clone())
            .unwrap_or_default();
        let entry = channels
            .entry(message.channel.topic.clone())
            .or_insert((schema, Vec::new()));
        entry.1.push(message.data.to_vec());
    }
    channels
}

/// Records one colour and one depth frame under the given formats, and returns
/// the finished file's channels.
fn record_one_frame_each(
    label: &str,
    color_format: ImageFormat,
    depth_format: ImageFormat,
) -> BTreeMap<String, (String, Vec<Vec<u8>>)> {
    let directory = std::env::temp_dir().join(format!(
        "lite_record_compress_{label}_{}",
        std::process::id()
    ));
    std::fs::create_dir_all(&directory).unwrap();

    let mut settings = quiet_settings(&directory);
    settings.color_format = color_format;
    settings.depth_format = depth_format;

    let hub = Hub::new(settings, directory.join("settings.json"));
    let status = hub.start_recording(Some(label)).unwrap();
    let path = status.path.clone().unwrap();

    let sink = hub.sink();
    sink(Produced::Image {
        stream: StreamId::Color,
        topic: "/camera/color_image".to_owned(),
        image: colour_frame(32, 24),
    });
    sink(Produced::Image {
        stream: StreamId::Depth,
        topic: "/camera/depth_image".to_owned(),
        image: depth_frame(32, 24),
    });

    // The sink hands off to the encode pool, which hands off to the writer.
    std::thread::sleep(Duration::from_millis(400));
    let finished = hub.stop_recording().unwrap();
    assert_eq!(finished.dropped, 0, "the pipeline dropped a two-frame recording");

    let channels = read_back(std::path::Path::new(&path));
    std::fs::remove_dir_all(&directory).ok();
    channels
}

#[test]
fn choosing_jpeg_and_png_rewrites_the_topics_schemas_and_payloads() {
    let channels = record_one_frame_each("jpeg_png", ImageFormat::Jpeg, ImageFormat::Png);

    let (schema, payloads) = channels
        .get("/camera/color_image")
        .expect("colour is missing from the file");
    assert_eq!(schema, "sensor_msgs/msg/CompressedImage");
    let (format, data) = read_compressed_image(&payloads[0]);
    assert_eq!(format, "jpeg");
    assert_eq!(&data[..2], &[0xff, 0xd8], "not a JFIF stream");
    // The whole point of asking for jpeg on a Pi is that it is smaller than the
    // 32*24*3 bytes the raw frame would have cost.
    assert!(data.len() < 32 * 24 * 3, "jpeg was not smaller than raw");

    let (schema, payloads) = channels
        .get("/camera/depth_image")
        .expect("depth is missing from the file");
    assert_eq!(schema, "sensor_msgs/msg/CompressedImage");
    let (format, data) = read_compressed_image(&payloads[0]);
    assert_eq!(format, "png");
    assert_eq!(&data[..8], &[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]);

    // Read the depth back out of the file and compare it with what went in.
    // Lossless is a claim about the samples, not about the file extension.
    let mut reader = png::Decoder::new(std::io::Cursor::new(&data))
        .read_info()
        .unwrap();
    assert_eq!(reader.info().bit_depth, png::BitDepth::Sixteen);
    let mut raw = vec![0u8; reader.output_buffer_size().unwrap()];
    let info = reader.next_frame(&mut raw).unwrap();
    let recovered: Vec<u16> = raw[..info.buffer_size()]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| u16::from_be_bytes(*pair))
        .collect();
    let expected: Vec<u16> = (0..32 * 24).map(|index| (index * 517) as u16).collect();
    assert_eq!(recovered, expected, "png depth did not survive the recording");
}

#[test]
fn webp_colour_survives_the_recording_pixel_for_pixel() {
    let channels = record_one_frame_each("webp", ImageFormat::Webp, ImageFormat::Png);
    let (_, payloads) = channels
        .get("/camera/color_image")
        .expect("colour is missing from the file");
    let (format, data) = read_compressed_image(&payloads[0]);
    assert_eq!(format, "webp");

    let mut decoder = image_webp::WebPDecoder::new(std::io::Cursor::new(&data)).unwrap();
    let mut pixels = vec![0u8; decoder.output_buffer_size().unwrap()];
    decoder.read_image(&mut pixels).unwrap();
    assert_eq!(decoder.dimensions(), (32, 24));
    assert_eq!(pixels, colour_frame(32, 24).data);
}

/// The failure this guards against is a silently degraded recording: jpeg is
/// 8-bit, so applying it to a 16-bit depth stream would either truncate every
/// sample or drop the frame. Neither is acceptable, so the raw frame is kept.
#[test]
fn a_codec_that_cannot_hold_depth_leaves_the_frame_raw_rather_than_truncating_it() {
    let channels = record_one_frame_each("lossy_depth", ImageFormat::Jpeg, ImageFormat::Jpeg);

    // Raw and compressed share the topic, so the schema is what says whether
    // the codec was applied or refused.
    let (schema, payloads) = channels
        .get("/camera/depth_image")
        .expect("depth was dropped entirely instead of falling back to raw");
    assert_eq!(
        schema, "sensor_msgs/msg/Image",
        "16-bit depth was written through an 8-bit codec"
    );
    assert!(!payloads.is_empty());

    // Colour still took the requested codec, so the fallback is per-stream.
    assert_eq!(
        channels["/camera/color_image"].0,
        "sensor_msgs/msg/CompressedImage"
    );
}

#[test]
fn the_raw_setting_records_an_image_message_untouched() {
    let channels = record_one_frame_each("raw", ImageFormat::Raw, ImageFormat::Raw);
    for topic in ["/camera/color_image", "/camera/depth_image"] {
        let (schema, payloads) = channels
            .get(topic)
            .unwrap_or_else(|| panic!("{topic} is missing"));
        assert_eq!(schema, "sensor_msgs/msg/Image");
        assert!(!payloads.is_empty());
    }
}

/// Voxel downsampling is the lidar's half of the compression settings. It is
/// checked on a real Mid-360 capture rather than on generated points, because
/// the thing being measured is how many distinct 20 cm cells a real room's
/// return pattern falls into.
#[test]
fn voxel_downsampling_thins_the_cloud_that_reaches_the_file() {
    let packets = parse_capture(livox::CAPTURE);
    assert!(!packets.is_empty(), "the capture fixture is empty");

    let full = livox::FrameAccumulator::new(10.0, "livox_frame");
    let thinned = livox::FrameAccumulator::new(10.0, "livox_frame").with_voxel_leaf_size(0.2);
    let (full, thinned) = (
        count_points(full, &packets),
        count_points(thinned, &packets),
    );

    assert!(full > 0, "the capture produced no points at all");
    assert!(
        thinned < full,
        "a 20 cm voxel grid did not thin the cloud: {thinned} of {full}"
    );
}

fn count_points(mut accumulator: livox::FrameAccumulator, packets: &[livox::CapturedPacket]) -> u32 {
    let mut points = 0;
    for packet in packets {
        if packet.port != livox::HOST_POINT_PORT {
            continue;
        }
        if let Some(cloud) = accumulator
            .push_packet(&packet.payload, packet.received_nanos)
            .unwrap()
        {
            points += cloud.width * cloud.height;
        }
    }
    // The last partial frame never crosses a boundary, so it is flushed by hand.
    if let Some(cloud) = accumulator.flush() {
        points += cloud.width * cloud.height;
    }
    points
}
