//! Ingest a raw Livox Mid-360 SDK2 `.pcap` capture into [`SyncPackage`]s.
//!
//! The capture is classic little-endian microsecond libpcap. Livox SDK2 sends
//! two UDP streams that share a 36-byte `LivoxLidarEthernetPacket` header
//! carrying a device-nanosecond timestamp:
//!   * point cloud -> dst port 56301, data_type 1 (Cartesian-high, mm)
//!   * IMU         -> dst port 56401, data_type 0 (gyro rad/s, accel in g)
//!
//! Points are assembled into ~10 Hz frames; each frame plus the IMU samples up
//! to its end becomes one `SyncPackage`. This mirrors the proven ingestion in
//! the FAST-LIO `render` binary so both estimators see identical input.

use std::fs::File;
use std::io::{BufReader, Read};

use crate::config::Config;
use crate::types::{ImuData, Point, SyncPackage, V3D};

const CLOUD_PORT: u16 = 56301;
const IMU_PORT: u16 = 56401;
const LIVOX_HEADER: usize = 36;
const CLOUD_POINT_SIZE: usize = 14; // i32 x,y,z (mm) + u8 reflectivity + u8 tag
const GRAVITY: f64 = 9.81; // Livox IMU reports accel in g
const FRAME_SEC: f64 = 0.1; // assemble LiDAR frames at ~10 Hz

struct RawPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    abs_time: f64,
}

struct PcapReader<R: Read> {
    r: R,
}

impl<R: Read> PcapReader<R> {
    fn new(mut r: R) -> std::io::Result<Self> {
        let mut gh = [0u8; 24];
        r.read_exact(&mut gh)?;
        let magic = u32::from_le_bytes(gh[0..4].try_into().unwrap());
        if magic != 0xa1b2_c3d4 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("not a little-endian microsecond pcap (magic {magic:#x})"),
            ));
        }
        Ok(PcapReader { r })
    }

    fn next_packet<'a>(&mut self, buf: &'a mut Vec<u8>) -> Option<(u16, &'a [u8])> {
        let mut rh = [0u8; 16];
        if self.r.read_exact(&mut rh).is_err() {
            return None;
        }
        let incl = u32::from_le_bytes(rh[8..12].try_into().unwrap()) as usize;
        buf.resize(incl, 0);
        if self.r.read_exact(buf).is_err() {
            return None;
        }
        if incl < 42 {
            return Some((0, &[]));
        }
        let eth_type = u16::from_be_bytes([buf[12], buf[13]]);
        if eth_type != 0x0800 {
            return Some((0, &[]));
        }
        let ihl = ((buf[14] & 0x0f) as usize) * 4;
        let proto = buf[14 + 9];
        let udp = 14 + ihl;
        if proto != 17 || buf.len() < udp + 8 {
            return Some((0, &[]));
        }
        let dport = u16::from_be_bytes([buf[udp + 2], buf[udp + 3]]);
        let ulen = u16::from_be_bytes([buf[udp + 4], buf[udp + 5]]) as usize;
        let start = udp + 8;
        let end = (udp + ulen).min(buf.len());
        if end <= start {
            return Some((0, &[]));
        }
        Some((dport, &buf[start..end]))
    }
}

#[inline]
fn livox_timestamp_ns(payload: &[u8]) -> u64 {
    u64::from_le_bytes(payload[28..36].try_into().unwrap())
}

/// Load an entire Livox `.pcap` into a vector of `SyncPackage`s. `duration_s`
/// limits how much wall-clock of data to read (0 = whole file).
pub fn load_pcap(path: &str, cfg: &Config, duration_s: f64) -> std::io::Result<Vec<SyncPackage>> {
    let file = File::open(path)?;
    let mut reader = PcapReader::new(BufReader::with_capacity(1 << 20, file))?;
    let mut buf: Vec<u8> = Vec::with_capacity(2048);

    let filter_num = cfg.point_filter_num.max(1) as usize;
    let min_r2 = cfg.blind * cfg.blind;
    let max_r2 = cfg.max_range * cfg.max_range;

    let mut packages: Vec<SyncPackage> = Vec::new();
    let mut imu_buf: Vec<ImuData> = Vec::new();
    let mut frame_pts: Vec<RawPoint> = Vec::new();
    let mut frame_start: Option<f64> = None;
    let mut filter_ctr: usize = 0;
    let mut first_frame_time: Option<f64> = None;

    while let Some((dport, payload)) = reader.next_packet(&mut buf) {
        if payload.len() < LIVOX_HEADER {
            continue;
        }
        let ts = livox_timestamp_ns(payload) as f64 * 1e-9;
        let body = &payload[LIVOX_HEADER..];

        if dport == IMU_PORT {
            if body.len() < 24 {
                continue;
            }
            let g = |o: usize| f32::from_le_bytes(body[o..o + 4].try_into().unwrap()) as f64;
            imu_buf.push(ImuData {
                gyro: V3D::new(g(0), g(4), g(8)),
                acc: V3D::new(g(12), g(16), g(20)) * GRAVITY,
                time: ts,
            });
            continue;
        }
        if dport != CLOUD_PORT {
            continue;
        }

        let dot_num = u16::from_le_bytes([payload[5], payload[6]]) as usize;
        let interval_ns = u16::from_le_bytes([payload[3], payload[4]]) as f64 * 100.0;
        let pt_dt = if dot_num > 0 { interval_ns / dot_num as f64 * 1e-9 } else { 0.0 };

        for i in 0..dot_num {
            let o = i * CLOUD_POINT_SIZE;
            if o + CLOUD_POINT_SIZE > body.len() {
                break;
            }
            let xi = i32::from_le_bytes(body[o..o + 4].try_into().unwrap());
            let yi = i32::from_le_bytes(body[o + 4..o + 8].try_into().unwrap());
            let zi = i32::from_le_bytes(body[o + 8..o + 12].try_into().unwrap());
            if xi == 0 && yi == 0 && zi == 0 {
                continue;
            }
            let refl = body[o + 12];
            let x = xi as f64 / 1000.0;
            let y = yi as f64 / 1000.0;
            let z = zi as f64 / 1000.0;
            let r2 = x * x + y * y + z * z;
            if r2 < min_r2 || r2 > max_r2 {
                continue;
            }
            filter_ctr += 1;
            if filter_ctr % filter_num != 0 {
                continue;
            }
            let abs_time = ts + i as f64 * pt_dt;
            if frame_start.is_none() {
                frame_start = Some(abs_time);
            }
            frame_pts.push(RawPoint { x: x as f32, y: y as f32, z: z as f32, intensity: refl as f32, abs_time });
        }

        let fs = match frame_start {
            Some(v) => v,
            None => continue,
        };
        if ts - fs < FRAME_SEC {
            continue;
        }

        let frame_end = frame_pts.last().map(|p| p.abs_time).unwrap_or(ts);
        let cloud: Vec<Point> = frame_pts
            .iter()
            .map(|p| Point::new(p.x, p.y, p.z, p.intensity, ((p.abs_time - fs) * 1000.0) as f32))
            .collect();
        frame_pts.clear();
        frame_start = None;

        if imu_buf.is_empty() || cloud.is_empty() {
            continue;
        }

        let (this_imus, rest): (Vec<_>, Vec<_>) =
            imu_buf.drain(..).partition(|s| s.time <= frame_end);
        imu_buf = rest;
        if this_imus.is_empty() {
            continue;
        }

        if first_frame_time.is_none() {
            first_frame_time = Some(fs);
        }
        if duration_s > 0.0 && fs - first_frame_time.unwrap() > duration_s {
            break;
        }

        packages.push(SyncPackage {
            imus: this_imus,
            cloud,
            cloud_start_time: fs,
            cloud_end_time: frame_end,
        });
    }

    Ok(packages)
}
