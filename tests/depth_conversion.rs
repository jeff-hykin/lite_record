//! Records a jxl depth stream the way the Pi does, then converts the finished
//! file the way the web UI's button does, and checks the result is what Foxglove
//! needs: raw `16UC1` pixels carrying the original samples, stamp and frame.
//!
//! The unit test in `image.rs` proves the decoder round-trips one buffer. This
//! proves the whole file survives: that colour is left alone, that the depth
//! channel is rewritten rather than duplicated, and that a converted file still
//! reads back as an mcap.

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

#[test]
fn converting_a_recording_replaces_its_jxl_depth_with_pixels_foxglove_can_draw() {
    let directory =
        std::env::temp_dir().join(format!("lite_record_convert_{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();

    let mut settings = Settings {
        record_dir: directory.clone(),
        color_format: ImageFormat::Jpeg,
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
        read_back(source)["/camera/depth_image"].0,
        "sensor_msgs/msg/CompressedImage",
        "depth did not record as jxl, so there is nothing to convert"
    );

    let original_color = read_back(source)["/camera/color_image"].1.clone();

    let progress = Arc::new(convert::Progress::default());
    let report = convert::depth_in_place(source, &progress).unwrap();
    assert_eq!(report.decoded, 1);
    assert_eq!(report.failed, 0);
    assert!(report.copied >= 1, "colour was not carried over");
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

    // Colour is not jxl, so it must come through as the bytes that were recorded.
    assert_eq!(
        channels["/camera/color_image"].1, original_color,
        "colour was rewritten instead of copied"
    );

    // A second run finds no jxl left. It must refuse and leave the file alone,
    // because a "conversion" that re-compresses an already-raw file in place
    // would churn every recording someone taps twice.
    let before = std::fs::read(source).unwrap();
    let error = convert::depth_in_place(source, &progress).unwrap_err();
    assert!(error.to_string().contains("nothing to convert"), "{error}");
    assert_eq!(std::fs::read(source).unwrap(), before, "a refused conversion changed the file");

    std::fs::remove_dir_all(&directory).ok();
}
