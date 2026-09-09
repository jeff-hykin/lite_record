//! Records the jxl streams the Pi does, then converts the finished file the way
//! the web UI's button does, and checks every one of them came out in something
//! Foxglove has a decoder for — without losing a pixel.
//!
//! The unit test in `image.rs` proves the decoder round-trips one buffer. This
//! proves the whole file survives: that each stream lands on the format its pixel
//! layout calls for, that a channel is rewritten rather than duplicated, and that
//! a converted file still reads back as an mcap.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use lite_record::convert;
use lite_record::hub::{Hub, Settings};
use lite_record::image::ImageFormat;
use lite_record::msgs::{Header, RawImage};
use lite_record::sensors::{Produced, StreamId};

const WIDTH: usize = 32;
const HEIGHT: usize = 24;
const STAMP: u64 = 1_700_000_000_000_000_000;

fn depth_samples() -> Vec<u16> {
    (0..WIDTH * HEIGHT).map(|index| (index * 517) as u16).collect()
}

/// Reads a `sensor_msgs/Image` back out of its CDR payload.
fn read_raw_image(message: &[u8]) -> (String, u32, u32, String, Vec<u8>) {
    let body = &message[4..];
    let mut at = 0usize;
    let u32_at = |at: &mut usize| {
        *at += (4 - (*at % 4)) % 4;
        let bytes: [u8; 4] = body[*at..*at + 4].try_into().unwrap();
        *at += 4;
        u32::from_le_bytes(bytes)
    };
    let string_at = |at: &mut usize| {
        let length = u32_at(at) as usize;
        let text = String::from_utf8(body[*at..*at + length - 1].to_vec()).unwrap();
        *at += length;
        text
    };

    u32_at(&mut at);
    u32_at(&mut at);
    let frame_id = string_at(&mut at);
    let height = u32_at(&mut at);
    let width = u32_at(&mut at);
    let encoding = string_at(&mut at);
    at += 1; // is_bigendian
    let _step = u32_at(&mut at);
    let length = u32_at(&mut at) as usize;
    (frame_id, height, width, encoding, body[at..at + length].to_vec())
}

/// The `format` string and payload of a `sensor_msgs/CompressedImage`.
fn read_compressed_image(message: &[u8]) -> (String, Vec<u8>) {
    let mut reader = lite_record::cdr::CdrReader::new(message);
    let _ = reader.header();
    (reader.string(), reader.bytes().to_vec())
}

fn decode_webp(encoded: &[u8]) -> Vec<u8> {
    let mut decoder = image_webp::WebPDecoder::new(std::io::Cursor::new(encoded)).unwrap();
    let mut pixels = vec![0u8; decoder.output_buffer_size().unwrap()];
    decoder.read_image(&mut pixels).unwrap();
    pixels
}

fn decode_png(encoded: &[u8]) -> Vec<u8> {
    let mut reader = png::Decoder::new(std::io::Cursor::new(encoded)).read_info().unwrap();
    let mut pixels = vec![0u8; reader.output_buffer_size().unwrap()];
    let info = reader.next_frame(&mut pixels).unwrap();
    pixels.truncate(info.buffer_size());
    pixels
}

/// Every message in a file, keyed by topic, as (schema name, payloads).
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
        channels
            .entry(message.channel.topic.clone())
            .or_insert((schema, Vec::new()))
            .1
            .push(message.data.to_vec());
    }
    channels
}

/// How many 512-byte blocks the filesystem has actually given the file. Unlike
/// its length this drops when a hole is punched, which is the only way to see
/// that reclaiming did anything.
fn allocated_blocks(path: &std::path::Path) -> u64 {
    use std::os::unix::fs::MetadataExt;
    std::fs::metadata(path).unwrap().blocks()
}

/// Big enough and noisy enough that lossless jxl cannot squeeze it, so a
/// handful of frames run past mcap's 768 KB chunk and the file ends up with
/// several chunks to walk.
const WIDE: usize = 320;
const TALL: usize = 240;

fn noisy_depth(frame: usize) -> Vec<u16> {
    (0..WIDE * TALL)
        .map(|index| ((index * 2_654_435_761usize + frame * 40_503) % 65_536) as u16)
        .collect()
}

#[test]
fn reclaiming_frees_each_source_chunk_once_its_replacement_is_verified() {
    let directory =
        std::env::temp_dir().join(format!("lite_record_reclaim_{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();

    let mut settings = Settings {
        record_dir: directory.clone(),
        depth_format: ImageFormat::Jpegxl,
        preview_enabled: false,
        ..Settings::default()
    };
    settings.livox.enabled = false;
    settings.realsense.enabled = false;
    settings.orbbec.enabled = false;

    let hub = Hub::new(settings, directory.join("settings.json"));
    let recorded = hub.start_recording(Some("reclaim")).unwrap().path.unwrap();
    let sink = hub.sink();
    const FRAMES: usize = 24;
    for frame in 0..FRAMES {
        sink(Produced::Image {
            stream: StreamId::Depth,
            topic: "/camera/depth_image".to_owned(),
            image: RawImage {
                header: Header::new(STAMP + frame as u64, "camera_depth_optical_frame"),
                width: WIDE,
                height: TALL,
                step: WIDE * 2,
                is_bigendian: 0,
                encoding: "16UC1".to_owned(),
                data: noisy_depth(frame).iter().flat_map(|s| s.to_le_bytes()).collect(),
            },
        });
    }
    std::thread::sleep(Duration::from_secs(3));
    assert_eq!(hub.stop_recording().unwrap().dropped, 0);

    let source = std::path::Path::new(&recorded);
    let bytes = std::fs::read(source).unwrap();
    let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
    assert!(
        summary.chunk_indexes.len() > 1,
        "the fixture landed in {} chunk(s), so it would not exercise chunk-at-a-time at all",
        summary.chunk_indexes.len()
    );
    drop(bytes);

    let before = allocated_blocks(source);
    let output = directory.join("reclaimed.mcap");
    let progress = Arc::new(convert::Progress::default());
    let report =
        convert::to_viewable(source, &output, &progress, convert::Reclaim::AsItGoes).unwrap();

    assert_eq!(report.decoded, FRAMES as u64);
    assert_eq!(report.failed, 0);
    assert!(report.reclaimed > 0, "nothing was handed back to the filesystem");

    // The source keeps its length — the holes are inside it — so length cannot
    // show this and only the block count can.
    let after = allocated_blocks(source);
    assert!(
        after < before / 2,
        "the source still holds {after} blocks of {before}, so it was not really freed"
    );

    // Every frame has to survive that, exactly. Reclaiming is only acceptable
    // because the data moved before the space was released.
    let channels = read_back(&output);
    let (schema, payloads) = &channels["/camera/depth_image"];
    assert_eq!(schema, "sensor_msgs/msg/Image");
    assert_eq!(payloads.len(), FRAMES, "reclaiming lost frames");
    for (frame, payload) in payloads.iter().enumerate() {
        let (_, height, width, encoding, pixels) = read_raw_image(payload);
        assert_eq!((width, height), (WIDE as u32, TALL as u32));
        assert_eq!(encoding, "16UC1");
        let recovered: Vec<u16> = pixels
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| u16::from_le_bytes(*pair))
            .collect();
        assert_eq!(recovered, noisy_depth(frame), "frame {frame} came back changed");
    }

    std::fs::remove_dir_all(&directory).ok();
}

#[test]
fn converting_a_recording_moves_every_jxl_stream_to_a_format_foxglove_can_draw() {
    let directory =
        std::env::temp_dir().join(format!("lite_record_convert_{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();

    let mut settings = Settings {
        record_dir: directory.clone(),
        // Colour and infrared in jxl too, as the rig actually records them.
        // Foxglove cannot draw any of the three, so all three have to move.
        color_format: ImageFormat::Jpegxl,
        depth_format: ImageFormat::Jpegxl,
        preview_enabled: false,
        ..Settings::default()
    };
    settings.livox.enabled = false;
    settings.realsense.enabled = false;
    settings.orbbec.enabled = false;

    let hub = Hub::new(settings, directory.join("settings.json"));
    let recorded = hub.start_recording(Some("convert")).unwrap().path.unwrap();

    let sink = hub.sink();
    sink(Produced::Image {
        stream: StreamId::Color,
        topic: "/camera/color_image".to_owned(),
        image: RawImage {
            header: Header::new(STAMP, "camera_color_optical_frame"),
            width: WIDTH,
            height: HEIGHT,
            step: WIDTH * 3,
            is_bigendian: 0,
            encoding: "rgb8".to_owned(),
            data: (0..WIDTH * HEIGHT * 3).map(|index| ((index * 7) % 251) as u8).collect(),
        },
    });
    let infrared: Vec<u8> = (0..WIDTH * HEIGHT).map(|index| ((index * 31) % 253) as u8).collect();
    sink(Produced::Image {
        stream: StreamId::InfraLeft,
        topic: "/camera/infrared_left".to_owned(),
        image: RawImage {
            header: Header::new(STAMP, "camera_infra1_optical_frame"),
            width: WIDTH,
            height: HEIGHT,
            step: WIDTH,
            is_bigendian: 0,
            encoding: "mono8".to_owned(),
            data: infrared.clone(),
        },
    });
    sink(Produced::Image {
        stream: StreamId::Depth,
        topic: "/camera/depth_image".to_owned(),
        image: RawImage {
            header: Header::new(STAMP, "camera_depth_optical_frame"),
            width: WIDTH,
            height: HEIGHT,
            step: WIDTH * 2,
            is_bigendian: 0,
            encoding: "16UC1".to_owned(),
            data: depth_samples().iter().flat_map(|sample| sample.to_le_bytes()).collect(),
        },
    });
    std::thread::sleep(Duration::from_millis(400));
    assert_eq!(hub.stop_recording().unwrap().dropped, 0);

    let source = std::path::Path::new(&recorded);
    assert_eq!(
        read_back(source)["/camera/depth_image/compressed"].0,
        "sensor_msgs/msg/CompressedImage",
        "depth did not record as jxl, so there is nothing to convert"
    );

    let color_pixels: Vec<u8> = (0..WIDTH * HEIGHT * 3).map(|index| ((index * 7) % 251) as u8).collect();

    let progress = Arc::new(convert::Progress::default());
    let report = convert::in_place(source, &progress, convert::Reclaim::No).unwrap();
    assert_eq!(report.decoded, 3, "a jxl stream was left in a format Foxglove cannot draw");
    assert_eq!(report.failed, 0);
    assert_eq!(
        report.bytes,
        std::fs::metadata(source).unwrap().len(),
        "the reported size is not what landed on the card"
    );
    let leftovers: Vec<_> = std::fs::read_dir(&directory)
        .unwrap()
        .flatten()
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .filter(|name| name != "convert.mcap")
        .collect();
    assert!(leftovers.is_empty(), "the conversion left {leftovers:?} beside the recording");

    let channels = read_back(source);
    let (schema, payloads) = &channels["/camera/depth_image"];
    assert_eq!(schema, "sensor_msgs/msg/Image");
    assert_eq!(payloads.len(), 1, "the depth frame was duplicated, not replaced");
    assert!(
        !channels.contains_key("/camera/depth_image/compressed"),
        "the decoded depth was added beside the jxl instead of taking over its name"
    );

    let (frame_id, height, width, encoding, pixels) = read_raw_image(&payloads[0]);
    assert_eq!(frame_id, "camera_depth_optical_frame");
    assert_eq!((width, height), (WIDTH as u32, HEIGHT as u32));
    assert_eq!(encoding, "16UC1", "Foxglove's depth colormap keys off this name");
    let recovered: Vec<u16> = pixels
        .as_chunks::<2>()
        .0
        .iter()
        .map(|pair| u16::from_le_bytes(*pair))
        .collect();
    assert_eq!(recovered, depth_samples(), "conversion changed the depths");

    // Colour and infrared stay compressed — just in a codec with a decoder behind
    // it — so they keep the `/compressed` name, which describes the schema.
    let (color_schema, color_payloads) = &channels["/camera/color_image/compressed"];
    assert_eq!(color_schema, "sensor_msgs/msg/CompressedImage");
    let (format, encoded) = read_compressed_image(&color_payloads[0]);
    assert_eq!(format, "webp", "Foxglove has no jxl decoder");
    assert_eq!(decode_webp(&encoded), color_pixels, "the webp lost colour pixels");

    let (infra_schema, infra_payloads) = &channels["/camera/infrared_left/compressed"];
    assert_eq!(infra_schema, "sensor_msgs/msg/CompressedImage");
    let (format, encoded) = read_compressed_image(&infra_payloads[0]);
    assert_eq!(format, "png", "Foxglove has no jxl decoder");
    assert_eq!(decode_png(&encoded), infrared, "the png lost infrared pixels");

    // A second run finds no jxl left. It must refuse and leave the file alone,
    // because a "conversion" that re-compresses an already-raw file in place
    // would churn every recording someone taps twice.
    let before = std::fs::read(source).unwrap();
    let error = convert::in_place(source, &progress, convert::Reclaim::No).unwrap_err();
    assert!(error.to_string().contains("nothing to convert"), "{error}");
    assert_eq!(std::fs::read(source).unwrap(), before, "a refused conversion changed the file");

    std::fs::remove_dir_all(&directory).ok();
}
