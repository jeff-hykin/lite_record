//! NMEA 0183, the line protocol every USB GPS puck speaks (a BU-353N sends it
//! over a Prolific serial bridge at 4800 baud). Only GGA is turned into a fix:
//! it carries position, altitude, satellite count and HDOP in one sentence, once
//! per epoch, which is exactly one `NavSatFix`. Everything else is still
//! recorded verbatim on the raw topic, so nothing the receiver said is lost.

use crate::msgs::{Header, NavSatFix};

/// `sensor_msgs/NavSatStatus` values.
pub const STATUS_NO_FIX: i8 = -1;
pub const STATUS_FIX: i8 = 0;
pub const STATUS_SBAS_FIX: i8 = 1;
pub const STATUS_GBAS_FIX: i8 = 2;
/// GPS only. A multi-constellation receiver would OR in GLONASS/Galileo bits,
/// but GGA does not say which constellations fed the solution.
pub const SERVICE_GPS: u16 = 1;
pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;

/// The 1-sigma range error of a consumer receiver. HDOP scales it into a
/// horizontal error; this is the usual rule of thumb, hence "approximated".
const USER_EQUIVALENT_RANGE_ERROR_M: f64 = 5.0;

#[derive(Debug, Clone, PartialEq)]
pub struct Gga {
    /// Seconds since UTC midnight, as the receiver reports it.
    pub utc_seconds: Option<f64>,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    /// 0 invalid, 1 GPS, 2 DGPS/SBAS, 4 RTK fixed, 5 RTK float, 6 dead reckoning.
    pub quality: u8,
    pub satellites: u8,
    pub hdop: Option<f64>,
    /// Above mean sea level.
    pub altitude_msl: Option<f64>,
    /// Geoid height above the WGS84 ellipsoid.
    pub geoid_separation: Option<f64>,
}

impl Gga {
    pub fn has_fix(&self) -> bool {
        self.quality != 0 && self.latitude.is_some() && self.longitude.is_some()
    }

    /// NavSatFix wants altitude above the WGS84 ellipsoid, GGA gives it above
    /// the geoid; the two differ by ~25 m in San Francisco.
    pub fn ellipsoid_altitude(&self) -> Option<f64> {
        Some(self.altitude_msl? + self.geoid_separation.unwrap_or(0.0))
    }

    pub fn to_nav_sat_fix(&self, header: Header) -> NavSatFix {
        let status = if !self.has_fix() {
            STATUS_NO_FIX
        } else {
            match self.quality {
                2 => STATUS_SBAS_FIX,
                4 | 5 => STATUS_GBAS_FIX,
                _ => STATUS_FIX,
            }
        };
        let mut position_covariance = [0.0; 9];
        let covariance_type = match self.hdop {
            Some(hdop) if self.has_fix() => {
                let horizontal = (hdop * USER_EQUIVALENT_RANGE_ERROR_M).powi(2);
                position_covariance[0] = horizontal;
                position_covariance[4] = horizontal;
                // Vertical error runs about twice the horizontal.
                position_covariance[8] = 4.0 * horizontal;
                COVARIANCE_TYPE_APPROXIMATED
            }
            _ => COVARIANCE_TYPE_UNKNOWN,
        };
        NavSatFix {
            header,
            status,
            service: SERVICE_GPS,
            latitude: self.latitude.unwrap_or(f64::NAN),
            longitude: self.longitude.unwrap_or(f64::NAN),
            altitude: self.ellipsoid_altitude().unwrap_or(f64::NAN),
            position_covariance,
            position_covariance_type: covariance_type,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Sentence {
    Gga(Gga),
    /// Checksum-valid, but not a sentence this module interprets.
    Other,
}

/// Parses one line. `None` means it is not a checksum-valid NMEA sentence at
/// all, which is also how a serial port at the wrong baud rate is recognised.
pub fn parse(line: &str) -> Option<Sentence> {
    let body = checked_body(line)?;
    let fields: Vec<&str> = body.split(',').collect();
    let kind = fields.first()?;
    // The first two letters are the talker (GP, GN, GL, ...), the rest the type.
    if kind.len() == 5 && &kind[2..] == "GGA" {
        return parse_gga(&fields).map(Sentence::Gga);
    }
    Some(Sentence::Other)
}

/// The text between `$` and `*`, if the checksum matches.
pub fn checked_body(line: &str) -> Option<&str> {
    let line = line.trim();
    let rest = line.strip_prefix('$')?;
    let (body, checksum) = rest.rsplit_once('*')?;
    let expected = u8::from_str_radix(checksum.get(..2)?, 16).ok()?;
    let actual = body.bytes().fold(0u8, |sum, byte| sum ^ byte);
    (actual == expected).then_some(body)
}

fn parse_gga(fields: &[&str]) -> Option<Gga> {
    if fields.len() < 12 {
        return None;
    }
    Some(Gga {
        utc_seconds: parse_hhmmss(fields[1]),
        latitude: parse_coordinate(fields[2], fields[3], 2),
        longitude: parse_coordinate(fields[4], fields[5], 3),
        quality: fields[6].parse().unwrap_or(0),
        satellites: fields[7].parse().unwrap_or(0),
        hdop: fields[8].parse().ok(),
        altitude_msl: fields[9].parse().ok(),
        geoid_separation: fields[11].parse().ok(),
    })
}

fn parse_hhmmss(field: &str) -> Option<f64> {
    if field.len() < 6 {
        return None;
    }
    let hours: f64 = field[0..2].parse().ok()?;
    let minutes: f64 = field[2..4].parse().ok()?;
    let seconds: f64 = field[4..].parse().ok()?;
    Some(hours * 3600.0 + minutes * 60.0 + seconds)
}

/// `ddmm.mmmm` (or `dddmm.mmmm` for longitude) plus a hemisphere letter, to
/// signed decimal degrees.
fn parse_coordinate(value: &str, hemisphere: &str, degree_digits: usize) -> Option<f64> {
    if value.len() <= degree_digits {
        return None;
    }
    let degrees: f64 = value[..degree_digits].parse().ok()?;
    let minutes: f64 = value[degree_digits..].parse().ok()?;
    let magnitude = degrees + minutes / 60.0;
    match hemisphere {
        "N" | "E" => Some(magnitude),
        "S" | "W" => Some(-magnitude),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Captured from the BU-353N on dimpi5, 2026-09-26, indoors in San Francisco.
    const GGA: &str = "$GPGGA,175411.000,3745.7561,N,12229.6726,W,1,8,1.30,55.8,M,-25.4,M,,0000*5A";
    const RMC: &str = "$GPRMC,175411.000,A,3745.7561,N,12229.6726,W,0.09,179.20,260926,,,A*79";

    #[test]
    fn a_real_gga_parses_to_san_francisco() {
        let Some(Sentence::Gga(gga)) = parse(GGA) else {
            panic!("not a gga")
        };
        assert!(gga.has_fix());
        assert!((gga.latitude.unwrap() - 37.762_601_7).abs() < 1e-6);
        assert!((gga.longitude.unwrap() + 122.494_543_3).abs() < 1e-6);
        assert_eq!(gga.satellites, 8);
        assert_eq!(gga.hdop, Some(1.30));
        assert_eq!(gga.utc_seconds, Some(17.0 * 3600.0 + 54.0 * 60.0 + 11.0));
        assert!((gga.ellipsoid_altitude().unwrap() - 30.4).abs() < 1e-9);
    }

    #[test]
    fn other_valid_sentences_are_recognised_but_not_interpreted() {
        assert_eq!(parse(RMC), Some(Sentence::Other));
    }

    #[test]
    fn a_corrupted_sentence_is_rejected_by_its_checksum() {
        assert_eq!(parse(&GGA.replace("3745", "3746")), None);
        assert_eq!(parse(&GGA.replace("*5A", "*5B")), None);
        assert_eq!(parse("not nmea at all"), None);
        assert_eq!(parse("$GPGGA,no,checksum"), None);
        // What a 9600-baud read of a 4800-baud port looks like.
        assert_eq!(parse("`\u{fffd}` f`~f\u{fffd}\u{fffd}f"), None);
    }

    #[test]
    fn a_line_ending_and_other_talkers_are_accepted() {
        let multi = with_checksum("GNGGA,000001.00,0130.0000,S,00030.0000,E,2,12,0.8,10.0,M,5.0,M,,");
        let Some(Sentence::Gga(gga)) = parse(&format!("{multi}\r\n")) else {
            panic!("not a gga")
        };
        assert_eq!(gga.latitude, Some(-1.5));
        assert_eq!(gga.longitude, Some(0.5));
        assert_eq!(gga.quality, 2);
    }

    #[test]
    fn no_fix_is_reported_as_no_fix_with_nan_position() {
        // What the puck sends before it has seen a satellite.
        let line = with_checksum("GPGGA,175411.000,,,,,0,0,,,M,,M,,");
        let Some(Sentence::Gga(gga)) = parse(&line) else {
            panic!("not a gga")
        };
        assert!(!gga.has_fix());
        let fix = gga.to_nav_sat_fix(Header::new(1, "gps_link"));
        assert_eq!(fix.status, STATUS_NO_FIX);
        assert!(fix.latitude.is_nan() && fix.longitude.is_nan() && fix.altitude.is_nan());
        assert_eq!(fix.position_covariance_type, COVARIANCE_TYPE_UNKNOWN);
    }

    #[test]
    fn a_fix_carries_an_hdop_derived_covariance() {
        let Some(Sentence::Gga(gga)) = parse(GGA) else {
            panic!("not a gga")
        };
        let fix = gga.to_nav_sat_fix(Header::new(5, "gps_link"));
        assert_eq!(fix.status, STATUS_FIX);
        assert_eq!(fix.service, SERVICE_GPS);
        assert_eq!(fix.position_covariance_type, COVARIANCE_TYPE_APPROXIMATED);
        let horizontal = (1.3f64 * 5.0).powi(2);
        assert!((fix.position_covariance[0] - horizontal).abs() < 1e-9);
        assert!((fix.position_covariance[8] - 4.0 * horizontal).abs() < 1e-9);
    }

    fn with_checksum(body: &str) -> String {
        let sum = body.bytes().fold(0u8, |sum, byte| sum ^ byte);
        format!("${body}*{sum:02X}")
    }
}
