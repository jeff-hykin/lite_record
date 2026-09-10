//! Repairing a recording whose sensor clocks disagree.
//!
//! Every stamp in a healthy file is the device's own hardware time put onto the
//! host clock — see [`crate::clock`]. A recorder that locked that offset once
//! writes a whole stream behind the rest of the file, and the grocery recording
//! is what that looks like: its Livox header stamps sit a flat 2005 s before
//! the RealSense stamps beside them, over all 874 s, with only delivery jitter
//! moving. Nothing in the file looks wrong. It simply cannot be drawn, because
//! no transform exists at the time a scan claims to have happened.
//!
//! Which clock moved is not recoverable from that recording — the host's, when
//! something corrected it, or the lidar's, which restarts when the sensor is
//! power-cycled or reconfigured while the offset it was pinned against is not.
//! The journal for that boot is gone. Both are cured by the same thing, an
//! offset that keeps following instead of one taken once, and both leave a file
//! that needs this.
//!
//! The repair is a shift, not a re-stamp from arrival time. Arrival time is
//! jittered by ethernet, USB and scheduling, and throwing the hardware spacing
//! away to escape a constant error would be the worse trade. So: measure how
//! far a channel is behind, and move the whole channel by that one number.
//!
//! **How the offset is measured.** Delivery latency can only make a message
//! arrive later than it was taken, so across a whole recording the *smallest*
//! `log_time - header_stamp` on a channel is the closest estimate of the true
//! offset — the same argument [`crate::clock::HostClock`] makes live, applied
//! offline where the whole file is available at once.

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result};

/// Below this, a channel is on the same clock as the file and is left alone.
/// Well above delivery jitter, well below any clock a person would notice.
pub const TOLERANCE_NANOS: i64 = 500_000_000;

/// What one channel's stamps say about the clock they were written on.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelClock {
    pub topic: String,
    /// The smallest `log_time - header_stamp` seen, in nanoseconds. Positive
    /// means the header stamps are behind the clock the file was logged on.
    pub offset_nanos: i64,
    pub messages: u64,
}

impl ChannelClock {
    pub fn needs_shift(&self) -> bool {
        self.offset_nanos.abs() > TOLERANCE_NANOS
    }

    pub fn seconds(&self) -> f64 {
        self.offset_nanos as f64 / 1e9
    }
}

/// Every channel's clock offset, by channel id.
///
/// Channels whose messages carry no `std_msgs/Header` first are skipped: there
/// is nothing to compare, and nothing this can repair. `/tf` is skipped too —
/// its stamps are written by this program, on the log clock by construction.
pub fn survey(mapped: &[u8]) -> Result<BTreeMap<u16, ChannelClock>> {
    let summary = mcap::Summary::read(mapped)?.context("the recording has no summary section")?;
    let mut clocks: BTreeMap<u16, ChannelClock> = BTreeMap::new();
    for message in mcap::MessageStream::new(mapped)? {
        let message = message?;
        if !repairable(&message.channel) {
            continue;
        }
        let Some(header) = crate::cdr::decode_header(&message.data) else {
            continue;
        };
        let offset = message.log_time as i64 - header.stamp_nanos() as i64;
        let entry = clocks.entry(message.channel.id).or_insert_with(|| ChannelClock {
            topic: message.channel.topic.clone(),
            offset_nanos: offset,
            messages: 0,
        });
        entry.offset_nanos = entry.offset_nanos.min(offset);
        entry.messages += 1;
    }
    let _ = summary;
    Ok(clocks)
}

/// Whether a channel's stamps can be compared and shifted: it carries a
/// `Header` as its first field, and it is not one this program writes.
pub fn repairable(channel: &mcap::Channel) -> bool {
    let Some(schema) = channel.schema.as_ref() else {
        return false;
    };
    if channel.topic == crate::record::TF_TOPIC || channel.topic == "/tf_static" {
        return false;
    }
    matches!(
        schema.name.as_str(),
        crate::msgs::IMAGE_TYPE
            | crate::msgs::COMPRESSED_IMAGE_TYPE
            | crate::msgs::POINT_CLOUD2_TYPE
            | crate::msgs::IMU_TYPE
            | crate::msgs::CAMERA_INFO_TYPE
            | crate::msgs::ODOMETRY_TYPE
    )
}

/// Adds `offset_nanos` to the `Header` at the front of a little-endian CDR
/// message, in place.
///
/// Only the eight bytes of `sec` and `nanosec` are touched, so a payload this
/// program cannot otherwise decode — a codec it does not know, a schema it has
/// never seen — still passes through byte for byte apart from its stamp.
pub fn shift_header(payload: &mut [u8], offset_nanos: i64) -> bool {
    if payload.len() < 12 || payload[..4] != [0x00, 0x01, 0x00, 0x00] {
        return false;
    }
    let seconds = i32::from_le_bytes(payload[4..8].try_into().expect("checked length"));
    let nanos = u32::from_le_bytes(payload[8..12].try_into().expect("checked length"));
    let stamp = seconds as i64 * crate::msgs::NANOS_PER_SEC as i64 + nanos as i64;
    let shifted = stamp.saturating_add(offset_nanos).max(0);
    payload[4..8].copy_from_slice(
        &((shifted / crate::msgs::NANOS_PER_SEC as i64) as i32).to_le_bytes(),
    );
    payload[8..12].copy_from_slice(
        &((shifted % crate::msgs::NANOS_PER_SEC as i64) as u32).to_le_bytes(),
    );
    true
}

/// The lines `post_process` prints about a recording's clocks.
pub fn describe(clocks: &BTreeMap<u16, ChannelClock>) -> String {
    let mut split: Vec<&ChannelClock> = clocks.values().filter(|clock| clock.needs_shift()).collect();
    if split.is_empty() {
        return String::new();
    }
    split.sort_by_key(|clock| std::cmp::Reverse(clock.offset_nanos));
    let mut out = String::from(
        "warning: this recording was written on more than one clock, so a viewer cannot place \
         the streams against each other:\n",
    );
    for clock in split {
        out.push_str(&format!(
            "  {:<40} {:+.3} s behind the log clock ({} messages)\n",
            clock.topic,
            clock.seconds(),
            clock.messages
        ));
    }
    out.push_str("  run post_process --fix-clocks to shift them onto it, keeping their spacing\n");
    out
}

/// A survey of `path`, for a caller that has not mapped the file itself.
pub fn survey_path(path: &Path) -> Result<BTreeMap<u16, ChannelClock>> {
    let file = std::fs::File::open(path).with_context(|| format!("could not open {}", path.display()))?;
    let mapped = unsafe { memmap2::Mmap::map(&file)? };
    survey(&mapped)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{Header, Imu, NANOS_PER_SEC};
    use std::io::BufWriter;

    fn scratch(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("lite_record_restamp_{name}_{}.mcap", crate::record::now_nanos()))
    }

    /// Two streams, one of them stamped a long way behind the clock the file is
    /// logged on, which is the grocery recording in miniature.
    fn split_clock_recording(path: &Path, behind_nanos: u64) {
        let file = std::fs::File::create(path).unwrap();
        let mut writer = mcap::WriteOptions::new()
            .compression(Some(mcap::Compression::Zstd))
            .chunk_size(Some(4096))
            .profile("ros2")
            .create(BufWriter::new(file))
            .unwrap();
        let sample = crate::cdr::imu(&Imu::unoriented(Header::new(0, "x"), [0.0; 3], [0.0; 3]));
        let schema = writer.add_schema(sample.schema_name, "ros2msg", sample.schema_text.as_bytes()).unwrap();
        let good = writer.add_channel(schema, "/camera/imu", "cdr", &Default::default()).unwrap();
        let late = writer.add_channel(schema, "/livox/imu", "cdr", &Default::default()).unwrap();
        let epoch = 1_700_000_000 * NANOS_PER_SEC;
        for index in 0..40u64 {
            let log_time = epoch + index * 5_000_000;
            // A little delivery jitter, so the survey has to take the minimum
            // rather than whatever the first message happened to show.
            let jitter = (index % 4) * 700_000;
            for (channel, stamp) in [
                (good, log_time - jitter),
                (late, log_time - behind_nanos - jitter),
            ] {
                let encoded = crate::cdr::imu(&Imu::unoriented(
                    Header::new(stamp, "frame"),
                    [0.0; 3],
                    [0.0; 3],
                ));
                writer
                    .write_to_known_channel(
                        &mcap::records::MessageHeader {
                            channel_id: channel,
                            sequence: index as u32,
                            log_time,
                            publish_time: log_time,
                        },
                        &encoded.data,
                    )
                    .unwrap();
            }
        }
        writer.finish().unwrap();
    }

    #[test]
    fn the_survey_finds_the_stream_that_is_behind_and_leaves_the_others_alone() {
        let path = scratch("survey");
        let behind = 2_005 * NANOS_PER_SEC;
        split_clock_recording(&path, behind);
        let bytes = std::fs::read(&path).unwrap();
        let clocks = survey(&bytes).unwrap();

        let late = clocks.values().find(|clock| clock.topic == "/livox/imu").unwrap();
        let good = clocks.values().find(|clock| clock.topic == "/camera/imu").unwrap();
        assert!(late.needs_shift());
        assert_eq!(late.offset_nanos, behind as i64, "the minimum, not a jittered sample");
        assert_eq!(late.messages, 40);
        assert!(!good.needs_shift(), "{} s", good.seconds());
        assert_eq!(good.offset_nanos, 0);

        let report = describe(&clocks);
        assert!(report.contains("/livox/imu"), "{report}");
        assert!(report.contains("2005.000 s behind"), "{report}");
        assert!(!report.contains("/camera/imu"), "{report}");
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_file_on_one_clock_reports_nothing() {
        let path = scratch("clean");
        split_clock_recording(&path, 0);
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(describe(&survey(&bytes).unwrap()), "");
        std::fs::remove_file(&path).ok();
    }

    /// The point of shifting rather than re-stamping from arrival: the device's
    /// own spacing is what a consumer integrates, and it has to come through
    /// untouched.
    #[test]
    fn shifting_moves_a_stamp_without_disturbing_its_spacing() {
        let first = crate::cdr::imu(&Imu::unoriented(
            Header::new(1_000_000_000, "frame"),
            [0.0; 3],
            [0.0; 3],
        ));
        let second = crate::cdr::imu(&Imu::unoriented(
            Header::new(1_033_333_333, "frame"),
            [0.0; 3],
            [0.0; 3],
        ));
        let offset = 2_005 * NANOS_PER_SEC as i64;
        let mut moved: Vec<Vec<u8>> = vec![first.data.clone(), second.data.clone()];
        for payload in &mut moved {
            assert!(shift_header(payload, offset));
        }
        let stamps: Vec<u64> = moved
            .iter()
            .map(|payload| crate::cdr::decode_header(payload).unwrap().stamp_nanos())
            .collect();
        assert_eq!(stamps[0], 1_000_000_000 + offset as u64);
        assert_eq!(stamps[1] - stamps[0], 33_333_333, "the spacing moved");
        // Nothing but the stamp changed.
        assert_eq!(&moved[0][12..], &first.data[12..]);
    }

    #[test]
    fn a_payload_that_is_not_little_endian_cdr_is_left_alone() {
        let mut big_endian = vec![0x00, 0x00, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 7, 8];
        assert!(!shift_header(&mut big_endian, 5));
        assert_eq!(big_endian[4..], [1, 2, 3, 4, 5, 6, 7, 8]);
        let mut too_short = vec![0x00, 0x01, 0x00, 0x00, 1, 2];
        assert!(!shift_header(&mut too_short, 5));
    }

    #[test]
    fn the_transforms_this_program_writes_are_never_shifted() {
        let path = scratch("tf");
        let file = std::fs::File::create(&path).unwrap();
        let mut writer = mcap::WriteOptions::new().profile("ros2").create(BufWriter::new(file)).unwrap();
        let encoded = crate::cdr::tf_message(&[]);
        let schema = writer.add_schema(encoded.schema_name, "ros2msg", encoded.schema_text.as_bytes()).unwrap();
        for topic in ["/tf", "/tf_static"] {
            let channel = writer.add_channel(schema, topic, "cdr", &Default::default()).unwrap();
            writer
                .write_to_known_channel(
                    &mcap::records::MessageHeader { channel_id: channel, sequence: 0, log_time: 9, publish_time: 9 },
                    &encoded.data,
                )
                .unwrap();
        }
        writer.finish().unwrap();
        let bytes = std::fs::read(&path).unwrap();
        assert!(survey(&bytes).unwrap().is_empty());
        std::fs::remove_file(&path).ok();
    }
}
