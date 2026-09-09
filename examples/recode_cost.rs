//! What each Foxglove-readable codec would cost, per stream, on real frames.
//!
//! Foxglove decodes png, jpeg, webp and avif, and nothing else, so a jxl
//! recording has to be re-encoded before it will draw. This says what that
//! trade costs: every ratio is against the jxl payload the codec would replace,
//! so 1.00 is break-even and jpeg being below it means the file shrinks.
//!
//! Ratios are of the *payload*, before the mcap chunk's zstd. Raw looks far
//! worse here than it lands on disk, because flat 16-bit depth runs are exactly
//! what zstd eats — measure the whole file for that number, not this.
//!
//! `cargo run --release --example recode_cost -- <recording.mcap>`

use std::collections::BTreeMap;

use lite_record::cdr::CdrReader;
use lite_record::image::{compress, decode_jpegxl, ImageFormat};

/// Decoding jxl and re-encoding it three ways costs about a second a frame, and
/// neighbouring frames of a handheld recording are nearly identical, so a sample
/// says the same thing as the whole stream for a fraction of the wait.
const SAMPLE_EVERY: usize = 10;

#[derive(Default)]
struct Tally {
    frames: usize,
    jxl: usize,
    candidates: BTreeMap<&'static str, usize>,
    nanos: BTreeMap<&'static str, u128>,
}

fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("usage: recode_cost <recording.mcap>"))?;
    let file = std::fs::File::open(&path)?;
    let mapped = unsafe { memmap2::Mmap::map(&file)? };

    let candidates = [
        ("raw", ImageFormat::Raw),
        ("png", ImageFormat::Png),
        ("webp", ImageFormat::Webp),
        ("jpeg", ImageFormat::Jpeg),
    ];

    let mut tallies: BTreeMap<String, Tally> = BTreeMap::new();
    let mut seen: BTreeMap<String, usize> = BTreeMap::new();

    for message in mcap::MessageStream::new(&mapped)? {
        let message = message?;
        let compressed_image = message
            .channel
            .schema
            .as_ref()
            .is_some_and(|schema| schema.name == lite_record::msgs::COMPRESSED_IMAGE_TYPE);
        if !compressed_image {
            continue;
        }

        let index = seen.entry(message.channel.topic.clone()).or_default();
        *index += 1;
        if *index % SAMPLE_EVERY != 1 {
            continue;
        }

        let mut reader = CdrReader::new(&message.data);
        let _ = reader.header();
        let format = reader.string().to_ascii_lowercase();
        if format != "jxl" && format != "jpegxl" {
            continue;
        }
        let image = decode_jpegxl(&reader.bytes())?;

        let tally = tallies.entry(message.channel.topic.clone()).or_default();
        tally.frames += 1;
        tally.jxl += message.data.len();
        for (name, format) in candidates {
            // `compress` returns None when the codec cannot hold the stream's
            // bit depth, which is the answer for jpeg and webp on 16-bit depth.
            let start = std::time::Instant::now();
            let size = match format {
                ImageFormat::Raw => image.data.len(),
                _ => compress(&image, format).map_or(0, |out| out.data.len()),
            };
            *tally.nanos.entry(name).or_default() += start.elapsed().as_nanos();
            *tally.candidates.entry(name).or_default() += size;
        }
    }

    println!("\nsize, as a ratio of the jxl payload it replaces");
    println!("{:38} {:>6} {:>9}  raw    png   webp   jpeg", "topic", "frames", "jxl KB/f");
    for (topic, tally) in &tallies {
        print!(
            "{topic:38} {:>6} {:>9.1}",
            tally.frames,
            tally.jxl as f64 / tally.frames as f64 / 1024.0
        );
        for (name, _) in candidates {
            match tally.candidates.get(name).copied().unwrap_or(0) {
                0 => print!("    —  "),
                total => print!("  {:.2}x", total as f64 / tally.jxl as f64),
            }
        }
        println!();
    }

    println!("\nencode time, ms per frame on this machine");
    println!("{:38} {:>7} {:>7} {:>7} {:>7}", "topic", "raw", "png", "webp", "jpeg");
    for (topic, tally) in &tallies {
        print!("{topic:38}");
        for (name, _) in candidates {
            match tally.nanos.get(name).copied().unwrap_or(0) {
                0 => print!("{:>7}", "—"),
                total => print!(
                    "{:>7.1}",
                    total as f64 / tally.frames as f64 / 1_000_000.0
                ),
            }
        }
        println!();
    }
    Ok(())
}
