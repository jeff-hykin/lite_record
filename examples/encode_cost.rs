//! What each stream costs to encode, as a share of one core at its frame rate.
//!
//! The recorder reaches full frame rate on a Pi 5, so this is not about
//! throughput any more: the box draws enough power to sit undervolted and
//! thermally throttled, and compression is where nearly all of the CPU goes.
//! Knowing which stream dominates is what decides whether an accelerated codec
//! is worth pulling in, so run this before reaching for one.
//!
//! Frames come from a real recording rather than being synthesised, because
//! codec cost tracks image content: random noise makes deflate give up early
//! and would have depth looking cheaper than it is.
//!
//!     cargo run --release --example encode_cost -- <directory of .bin frames>
//!
//! where the directory holds `color.bin` (rgb8), `infra_left.bin` and
//! `infra_right.bin` (mono8), and `depth.bin` (mono16, native endian), each a
//! tightly packed 640x480 frame.

use lite_record::image::{compress, ImageFormat};
use lite_record::msgs::{Header, RawImage};
use std::path::Path;
use std::time::Instant;

const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const FRAME_RATE: f64 = 30.0;
const ROUNDS: u32 = 60;

fn main() {
    let directory = std::env::args().nth(1).expect("pass a directory of frames");
    let directory = Path::new(&directory);
    println!(
        "{WIDTH}x{HEIGHT} at {FRAME_RATE} Hz, {ROUNDS} frames per case, {}",
        std::env::consts::ARCH
    );
    println!(
        "{:<22} {:>10} {:>12} {:>10}",
        "stream", "ms/frame", "% of a core", "KiB"
    );

    let mut total = 0.0;
    for (name, file, channels, encoding, format) in [
        ("color (rgb8)", "color.bin", 3, "rgb8", ImageFormat::Jpeg),
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
        let image = frame(&directory.join(file), channels, encoding);
        // One untimed pass so the allocator and caches are warm, otherwise the
        // first case pays for every later one.
        compress(&image, format).expect("format cannot hold these pixels");

        let started = Instant::now();
        let mut bytes = 0;
        for _ in 0..ROUNDS {
            bytes = compress(&image, format).unwrap().data.len();
        }
        let per_frame = started.elapsed().as_secs_f64() / ROUNDS as f64;
        let share = per_frame * FRAME_RATE * 100.0;
        total += share;
        println!(
            "{name:<22} {:>10.2} {:>11.1}% {:>10.1}",
            per_frame * 1000.0,
            share,
            bytes as f64 / 1024.0
        );
    }
    println!("{:<22} {:>10} {:>11.1}%", "all four", "", total);
}

fn frame(path: &Path, bytes_per_pixel: usize, encoding: &str) -> RawImage {
    let data = std::fs::read(path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    assert_eq!(
        data.len(),
        WIDTH * HEIGHT * bytes_per_pixel,
        "{}",
        path.display()
    );
    RawImage {
        header: Header {
            stamp_sec: 0,
            stamp_nsec: 0,
            frame_id: "cost".to_owned(),
        },
        width: WIDTH,
        height: HEIGHT,
        step: WIDTH * bytes_per_pixel,
        is_bigendian: 0,
        encoding: encoding.to_owned(),
        data,
    }
}
