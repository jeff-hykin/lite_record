//! Livox SDK2 control frames: the commands that make a Mid-360 stream.
//!
//! A factory-fresh Mid-360 transmits nothing. It has to be told an address to
//! send points and IMU to, and then told to start sampling. Everything here is
//! the encode half of that conversation, kept free of sockets so the wire
//! format can be tested on a laptop with no lidar on the desk.
//!
//! The layout is transcribed from Livox-SDK2 (`sdk_core/comm/sdk_protocol.cpp`,
//! `sdk_core/command_handler/build_request.cpp`), not guessed, and the two CRCs
//! are pinned by their published check values in the tests below.

use anyhow::{bail, Result};
use std::net::Ipv4Addr;

/// Where the lidar listens for commands, and where it expects the answer to go.
pub const LIDAR_CMD_PORT: u16 = 56100;
pub const HOST_CMD_PORT: u16 = 56101;

/// Ports the lidar transmits *from*. The firmware refuses to use any others, so
/// they travel in the request purely so it can echo them back.
const LIDAR_PUSH_PORT: u16 = 56200;
const LIDAR_POINT_PORT: u16 = 56300;
const LIDAR_IMU_PORT: u16 = 56400;

/// Status pushes are unicast to whoever asked last, so this has to be set even
/// though nothing here reads them: leaving it alone points a shared lidar at
/// whichever machine configured it previously.
const HOST_PUSH_PORT: u16 = 56201;

/// Confusingly named in the SDK: this one command carries every settable key,
/// work mode among them, so it is the only command id this module sends.
pub const CMD_WORK_MODE_CONTROL: u16 = 0x0100;

const KEY_STATE_INFO_HOST_IP: u16 = 0x0005;
const KEY_POINT_DATA_HOST_IP: u16 = 0x0006;
const KEY_IMU_HOST_IP: u16 = 0x0007;
const KEY_WORK_MODE: u16 = 0x001a;
const KEY_IMU_DATA_EN: u16 = 0x001c;

pub const WORK_MODE_NORMAL: u8 = 0x01;

const SOF: u8 = 0xaa;
const VERSION: u8 = 0;
const COMMAND_TYPE_CMD: u8 = 0;
const COMMAND_TYPE_ACK: u8 = 1;
const SENDER_TYPE_HOST: u8 = 0;

/// Header up to and including both CRCs; payload follows.
pub const HEADER_LEN: usize = 24;

/// The header CRC covers everything before itself and nothing after.
const HEADER_CRC_COVERAGE: usize = 18;

/// CRC16/CCITT-FALSE, matching `FastCRC16::ccitt`.
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut remainder: u16 = 0xffff;
    for byte in data {
        remainder ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            remainder = if remainder & 0x8000 != 0 {
                (remainder << 1) ^ 0x1021
            } else {
                remainder << 1
            };
        }
    }
    remainder
}

/// CRC-32/ISO-HDLC, matching `FastCRC32::crc32`.
fn crc32(data: &[u8]) -> u32 {
    let mut remainder: u32 = 0xffff_ffff;
    for byte in data {
        remainder ^= u32::from(*byte);
        for _ in 0..8 {
            remainder = if remainder & 1 != 0 {
                (remainder >> 1) ^ 0xedb8_8320
            } else {
                remainder >> 1
            };
        }
    }
    !remainder
}

/// Wraps a payload in the 24-byte SDK2 header.
pub fn frame(command_id: u16, sequence_number: u32, payload: &[u8]) -> Vec<u8> {
    let mut buffer = Vec::with_capacity(HEADER_LEN + payload.len());
    buffer.push(SOF);
    buffer.push(VERSION);
    buffer.extend_from_slice(&((HEADER_LEN + payload.len()) as u16).to_le_bytes());
    // The field is 32 bits wide but the SDK only ever fills the low 16, and the
    // ack echoes back what it was sent, so matching that is what lets a reply be
    // paired with its request.
    buffer.extend_from_slice(&(sequence_number & 0xffff).to_le_bytes());
    buffer.extend_from_slice(&command_id.to_le_bytes());
    buffer.push(COMMAND_TYPE_CMD);
    buffer.push(SENDER_TYPE_HOST);
    buffer.extend_from_slice(&[0u8; 6]);
    debug_assert_eq!(buffer.len(), HEADER_CRC_COVERAGE);
    buffer.extend_from_slice(&crc16_ccitt(&buffer).to_le_bytes());
    let payload_crc = if payload.is_empty() { 0 } else { crc32(payload) };
    buffer.extend_from_slice(&payload_crc.to_le_bytes());
    buffer.extend_from_slice(payload);
    buffer
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ack {
    pub command_id: u16,
    pub sequence_number: u32,
    /// Zero means accepted.
    pub return_code: u8,
    /// Which key the lidar objected to, when it objected to one.
    pub error_key: u16,
}

pub fn parse_ack(buffer: &[u8]) -> Result<Ack> {
    if buffer.len() < HEADER_LEN {
        bail!("livox ack is {} bytes, shorter than a header", buffer.len());
    }
    if buffer[0] != SOF {
        bail!("livox ack does not start with 0x{SOF:02x}");
    }
    if buffer[1] != VERSION {
        bail!("livox ack speaks protocol version {}", buffer[1]);
    }
    let length = usize::from(u16::from_le_bytes([buffer[2], buffer[3]]));
    if length < HEADER_LEN || length > buffer.len() {
        bail!("livox ack claims {length} bytes but {} arrived", buffer.len());
    }
    let stated_header_crc = u16::from_le_bytes([buffer[18], buffer[19]]);
    if stated_header_crc != crc16_ccitt(&buffer[..HEADER_CRC_COVERAGE]) {
        bail!("livox ack header crc does not match; this is not an SDK2 reply");
    }
    if buffer[10] != COMMAND_TYPE_ACK {
        bail!("livox reply is a command, not an ack");
    }
    let payload = &buffer[HEADER_LEN..length];
    // Nothing is readable without the return code, but the error key is only
    // present on the config acks, so a short payload is not by itself a fault.
    let Some(return_code) = payload.first().copied() else {
        bail!("livox ack carries no return code");
    };
    let error_key = match payload.get(1..3) {
        Some(bytes) => u16::from_le_bytes([bytes[0], bytes[1]]),
        None => 0,
    };
    Ok(Ack {
        command_id: u16::from_le_bytes([buffer[8], buffer[9]]),
        sequence_number: u32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]),
        return_code,
        error_key,
    })
}

/// `key_num`, padded to four bytes, then `{u16 key, u16 length, value}` each.
fn key_value_payload(pairs: &[(u16, Vec<u8>)]) -> Vec<u8> {
    let mut buffer = Vec::new();
    buffer.extend_from_slice(&(pairs.len() as u16).to_le_bytes());
    buffer.extend_from_slice(&[0u8; 2]);
    for (key, value) in pairs {
        buffer.extend_from_slice(&key.to_le_bytes());
        buffer.extend_from_slice(&(value.len() as u16).to_le_bytes());
        buffer.extend_from_slice(value);
    }
    buffer
}

fn host_ip_value(host: Ipv4Addr, host_port: u16, lidar_port: u16) -> Vec<u8> {
    let mut value = host.octets().to_vec();
    value.extend_from_slice(&host_port.to_le_bytes());
    value.extend_from_slice(&lidar_port.to_le_bytes());
    value
}

/// Tells the lidar which address and ports to push status, points and IMU to.
///
/// The three destinations go in one command because the firmware applies a
/// config request atomically; splitting them lets a half-configured lidar
/// stream points to one host and IMU to another.
pub fn host_destination_payload(host: Ipv4Addr) -> Vec<u8> {
    key_value_payload(&[
        (
            KEY_STATE_INFO_HOST_IP,
            host_ip_value(host, HOST_PUSH_PORT, LIDAR_PUSH_PORT),
        ),
        (
            KEY_POINT_DATA_HOST_IP,
            host_ip_value(host, crate::livox::HOST_POINT_PORT, LIDAR_POINT_PORT),
        ),
        (
            KEY_IMU_HOST_IP,
            host_ip_value(host, crate::livox::HOST_IMU_PORT, LIDAR_IMU_PORT),
        ),
    ])
}

pub fn imu_data_payload(enabled: bool) -> Vec<u8> {
    key_value_payload(&[(KEY_IMU_DATA_EN, vec![u8::from(enabled)])])
}

pub fn work_mode_payload(mode: u8) -> Vec<u8> {
    key_value_payload(&[(KEY_WORK_MODE, vec![mode])])
}

/// Builds what a lidar would send back, so the command channel can be exercised
/// against a fake one. There is no lidar in CI, and the retry and rejection
/// paths are exactly the ones that are hard to provoke with a real one.
#[cfg(test)]
pub(crate) fn ack_frame(command_id: u16, sequence_number: u32, payload: &[u8]) -> Vec<u8> {
    let mut frame = frame(command_id, sequence_number, payload);
    frame[10] = COMMAND_TYPE_ACK;
    let header_crc = crc16_ccitt(&frame[..HEADER_CRC_COVERAGE]);
    frame[18..20].copy_from_slice(&header_crc.to_le_bytes());
    frame
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The published check values for both algorithms. Getting either wrong
    /// makes the lidar drop every frame in silence, which is indistinguishable
    /// from a cabling fault, so pin them rather than trust the transcription.
    #[test]
    fn both_crcs_match_their_published_check_values() {
        assert_eq!(crc16_ccitt(b"123456789"), 0x29b1);
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn a_frame_carries_the_sdk2_header_the_lidar_expects() {
        let payload = work_mode_payload(WORK_MODE_NORMAL);
        let frame = frame(CMD_WORK_MODE_CONTROL, 7, &payload);

        assert_eq!(frame.len(), HEADER_LEN + payload.len());
        assert_eq!(frame[0], 0xaa);
        assert_eq!(frame[1], 0);
        assert_eq!(
            u16::from_le_bytes([frame[2], frame[3]]) as usize,
            frame.len(),
            "length counts the header as well as the payload"
        );
        assert_eq!(u32::from_le_bytes([frame[4], frame[5], frame[6], frame[7]]), 7);
        assert_eq!(u16::from_le_bytes([frame[8], frame[9]]), 0x0100);
        assert_eq!(frame[10], COMMAND_TYPE_CMD);
        assert_eq!(frame[11], SENDER_TYPE_HOST);
        assert_eq!(&frame[12..18], &[0u8; 6]);
        assert_eq!(
            u16::from_le_bytes([frame[18], frame[19]]),
            crc16_ccitt(&frame[..18])
        );
        assert_eq!(
            u32::from_le_bytes([frame[20], frame[21], frame[22], frame[23]]),
            crc32(&payload)
        );
        assert_eq!(&frame[HEADER_LEN..], &payload[..]);
    }

    #[test]
    fn an_empty_payload_gets_a_zero_data_crc() {
        // The SDK special-cases this instead of taking the crc of nothing, and a
        // heartbeat is sent with no payload at all.
        let frame = frame(CMD_WORK_MODE_CONTROL, 1, &[]);
        assert_eq!(frame.len(), HEADER_LEN);
        assert_eq!(u32::from_le_bytes([frame[20], frame[21], frame[22], frame[23]]), 0);
    }

    #[test]
    fn a_sequence_number_past_16_bits_is_truncated_like_the_sdk_does() {
        // The lidar echoes the low 16 bits, so a host that kept the full 32
        // would never match a reply to its request.
        let frame = frame(CMD_WORK_MODE_CONTROL, 0x1_0005, &[]);
        assert_eq!(u32::from_le_bytes([frame[4], frame[5], frame[6], frame[7]]), 5);
    }

    #[test]
    fn the_destination_payload_names_all_three_streams() {
        let payload = host_destination_payload(Ipv4Addr::new(192, 168, 1, 50));
        assert_eq!(u16::from_le_bytes([payload[0], payload[1]]), 3);
        assert_eq!(&payload[2..4], &[0, 0], "key_num is padded out to four bytes");
        // Three keys, each four bytes of header and eight of value.
        assert_eq!(payload.len(), 4 + 3 * (4 + 8));

        let mut offset = 4;
        for (expected_key, expected_host_port, expected_lidar_port) in [
            (KEY_STATE_INFO_HOST_IP, HOST_PUSH_PORT, LIDAR_PUSH_PORT),
            (KEY_POINT_DATA_HOST_IP, crate::livox::HOST_POINT_PORT, LIDAR_POINT_PORT),
            (KEY_IMU_HOST_IP, crate::livox::HOST_IMU_PORT, LIDAR_IMU_PORT),
        ] {
            assert_eq!(
                u16::from_le_bytes([payload[offset], payload[offset + 1]]),
                expected_key
            );
            assert_eq!(
                u16::from_le_bytes([payload[offset + 2], payload[offset + 3]]),
                8,
                "the wire value is four address bytes and two ports, not the SDK's public struct"
            );
            assert_eq!(&payload[offset + 4..offset + 8], &[192, 168, 1, 50]);
            assert_eq!(
                u16::from_le_bytes([payload[offset + 8], payload[offset + 9]]),
                expected_host_port
            );
            assert_eq!(
                u16::from_le_bytes([payload[offset + 10], payload[offset + 11]]),
                expected_lidar_port
            );
            offset += 12;
        }
    }

    #[test]
    fn single_key_payloads_hold_one_byte_of_value() {
        assert_eq!(imu_data_payload(true), vec![1, 0, 0, 0, 0x1c, 0x00, 1, 0, 1]);
        assert_eq!(imu_data_payload(false), vec![1, 0, 0, 0, 0x1c, 0x00, 1, 0, 0]);
        assert_eq!(
            work_mode_payload(WORK_MODE_NORMAL),
            vec![1, 0, 0, 0, 0x1a, 0x00, 1, 0, 1]
        );
    }

    #[test]
    fn an_accepting_ack_round_trips() {
        let ack = parse_ack(&ack_frame(CMD_WORK_MODE_CONTROL, 42, &[0, 0, 0])).unwrap();
        assert_eq!(
            ack,
            Ack {
                command_id: CMD_WORK_MODE_CONTROL,
                sequence_number: 42,
                return_code: 0,
                error_key: 0,
            }
        );
    }

    #[test]
    fn a_rejecting_ack_says_which_key_was_refused() {
        let ack = parse_ack(&ack_frame(CMD_WORK_MODE_CONTROL, 1, &[1, 0x1a, 0x00])).unwrap();
        assert_eq!(ack.return_code, 1);
        assert_eq!(ack.error_key, KEY_WORK_MODE);
    }

    #[test]
    fn traffic_that_is_not_an_sdk2_ack_is_rejected_rather_than_misread() {
        // Port 56101 is a plain UDP port; a stray packet must not be read as an
        // acceptance, because that would report a lidar as configured when it
        // never heard the request.
        assert!(parse_ack(&[]).is_err());
        assert!(parse_ack(&[0u8; 32]).is_err(), "wrong start byte");

        let mut wrong_crc = ack_frame(CMD_WORK_MODE_CONTROL, 1, &[0, 0, 0]);
        wrong_crc[18] ^= 0xff;
        assert!(parse_ack(&wrong_crc).is_err(), "corrupt header crc");

        let command = frame(CMD_WORK_MODE_CONTROL, 1, &[0, 0, 0]);
        assert!(parse_ack(&command).is_err(), "our own command echoed back");

        let truncated = ack_frame(CMD_WORK_MODE_CONTROL, 1, &[0, 0, 0]);
        assert!(
            parse_ack(&truncated[..HEADER_LEN]).is_err(),
            "header says there is a payload but none arrived"
        );
    }
}
