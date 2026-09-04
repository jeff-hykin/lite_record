//! What each stream costs to encode, as a share of one core at its frame rate.
//!
//! The recorder reaches full frame rate on a Pi 5, so this is not about
//! throughput any more: the box draws enough power to sit undervolted and
//! thermally throttled, and compression is where nearly all of the CPU goes.
//! Knowing which stream dominates is what decides whether an accelerated codec
//! is worth pulling in, so run this before reaching for one.
//!
//! The preview is measured alongside the streams because it is not free: it
//! runs on the same encode worker as the colour frame it is rendering, so its
//! cost lands squarely on the stream that is already the most expensive.
//!
//! Frames come from a real recording rather than being synthesised, because
//! codec cost tracks image content: random noise makes deflate give up early
//! and would have depth looking cheaper than it is.
//!
//!     cargo run --release --example encode_cost -- <directory of .bin frames> [width height]
//!
//! where the directory holds `color.bin` (rgb8), `infra_left.bin` and
//! `infra_right.bin` (mono8), and `depth.bin` (mono16, native endian), each a
//! tightly packed frame of the given size.

use lite_record::image::{compress, encode, ImageFormat};
use lite_record::msgs::{Header, RawImage};
use std::path::Path;
use std::time::Instant;

const FRAME_RATE: f64 = 30.0;
const ROUNDS: u32 = 60;
const PREVIEW_QUALITY: u8 = 60;
const PREVIEW_MAX_WIDTH: usize = 640;

fn main() {
    let mut args = std::env::args().skip(1);
    let directory = args.next().expect("pass a directory of frames");
    let directory = Path::new(&directory);
    let width: usize = args.next().map_or(640, |value| value.parse().unwrap());
    let height: usize = args.next().map_or(480, |value| value.parse().unwrap());

    println!(
        "{width}x{height} at {FRAME_RATE} Hz, {ROUNDS} frames per case, {}",
        std::env::consts::ARCH
    );
    println!(
        "{:<24} {:>10} {:>12} {:>10}",
        "stage", "ms/frame", "% of a core", "KiB"
    );

    let mut total = 0.0;
    let mut measure = |name: &str, work: &mut dyn FnMut() -> usize| {
        // One untimed pass so the allocator and caches are warm, otherwise the
        // first case pays for every later one.
        work();
        let started = Instant::now();
        let mut bytes = 0;
        for _ in 0..ROUNDS {
            bytes = work();
        }
        let per_frame = started.elapsed().as_secs_f64() / ROUNDS as f64;
        let share = per_frame * FRAME_RATE * 100.0;
        total += share;
        println!(
            "{name:<24} {:>10.2} {:>11.1}% {:>10.1}",
            per_frame * 1000.0,
            share,
            bytes as f64 / 1024.0
        );
        share
    };

    // bgr8 because that is the order every camera here delivers, and the byte
    // order decides whether the encoder gets a swap-free path.
    let color = frame(&directory.join("color.bin"), 3, "bgr8", width, height);
    let color_share = measure("color (bgr8 -> jpeg)", &mut || {
        compress(&color, ImageFormat::Jpeg).unwrap().data.len()
    });
    let preview_share = measure("preview (off the color)", &mut || {
        encode(&color, PREVIEW_QUALITY, PREVIEW_MAX_WIDTH)
            .unwrap()
            .jpeg
            .len()
    });

    for (name, file, channels, encoding, format) in [
        (
            "infra_left (mono8)",
            "infra_left.bin",
            1,
            "mono8",
            ImageFormat::Jpeg,
        ),
        (
            "infra_right (mono8)",
            "infra_right.bin",
            1,
            "mono8",
            ImageFormat::Jpeg,
        ),
        ("depth (mono16)", "depth.bin", 2, "mono16", ImageFormat::Png),
    ] {
        let image = frame(&directory.join(file), channels, encoding, width, height);
        measure(name, &mut || compress(&image, format).unwrap().data.len());
    }

    println!("{:<24} {:>10} {:>11.1}%", "everything", "", total);
    println!();
    // The colour worker runs both of these back to back, and one core is all it
    // has, so anything at or above 100% here is losing frames by construction.
    println!(
        "the color worker carries color + preview: {:.1}% of its one core",
        color_share + preview_share
    );
}

fn frame(
    path: &Path,
    bytes_per_pixel: usize,
    encoding: &str,
    width: usize,
    height: usize,
) -> RawImage {
    let data = std::fs::read(path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    assert_eq!(
        data.len(),
        width * height * bytes_per_pixel,
        "{}",
        path.display()
    );
    RawImage {
        header: Header {
            stamp_sec: 0,
            stamp_nsec: 0,
            frame_id: "cost".to_owned(),
        },
        width,
        height,
        step: width * bytes_per_pixel,
        is_bigendian: 0,
        encoding: encoding.to_owned(),
        data,
    }
}
