//! Cheap host health sampling for the web monitor.
//!
//! Everything here reads sysfs or /proc rather than shelling out, because the
//! monitor ticks at 5 Hz and forking `vcgencmd` twenty times a second would
//! itself be a measurable share of a Pi's CPU. The one exception is the
//! throttle word, which has no sysfs equivalent on older firmware.

use std::path::Path;
use std::time::Instant;

use serde::Serialize;

/// Raspberry Pi firmware throttle bits, as reported by `vcgencmd get_throttled`
/// and by the mailbox sysfs node. The low half is "now", the high half is
/// "since boot".
const UNDERVOLTAGE_NOW: u32 = 1 << 0;
const FREQ_CAPPED_NOW: u32 = 1 << 1;
const THROTTLED_NOW: u32 = 1 << 2;
const SOFT_TEMP_LIMIT_NOW: u32 = 1 << 3;
const UNDERVOLTAGE_EVER: u32 = 1 << 16;
const THROTTLED_EVER: u32 = 1 << 18;

#[derive(Debug, Clone, Serialize, Default)]
pub struct Health {
    /// Whole-machine busy fraction over the last sample interval, 0..1.
    pub cpu_busy: f64,
    /// Per-core busy fractions, so a pinned capture thread is visible even when
    /// the average looks idle.
    pub cpu_cores: Vec<f64>,
    pub load_one_minute: f64,
    pub memory_used_bytes: u64,
    pub memory_total_bytes: u64,
    pub temperature_celsius: Option<f64>,
    pub throttle: Option<Throttle>,
    /// Free space where recordings are being written.
    pub disk_free_bytes: Option<u64>,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq, Default)]
pub struct Throttle {
    pub raw: u32,
    pub undervoltage_now: bool,
    pub undervoltage_ever: bool,
    pub frequency_capped: bool,
    pub throttled_now: bool,
    pub throttled_ever: bool,
    pub soft_temperature_limit: bool,
}

impl Throttle {
    pub fn from_word(raw: u32) -> Self {
        Throttle {
            raw,
            undervoltage_now: raw & UNDERVOLTAGE_NOW != 0,
            undervoltage_ever: raw & UNDERVOLTAGE_EVER != 0,
            frequency_capped: raw & FREQ_CAPPED_NOW != 0,
            throttled_now: raw & THROTTLED_NOW != 0,
            throttled_ever: raw & THROTTLED_EVER != 0,
            soft_temperature_limit: raw & SOFT_TEMP_LIMIT_NOW != 0,
        }
    }

    /// The one line the UI shows. `None` means nothing is wrong.
    pub fn warning(&self) -> Option<String> {
        if self.undervoltage_now {
            return Some(
                "undervoltage right now: the power supply cannot hold 5V, frames will be dropped"
                    .into(),
            );
        }
        if self.throttled_now {
            return Some("cpu is being throttled right now".into());
        }
        if self.soft_temperature_limit {
            return Some("cpu is at its soft temperature limit".into());
        }
        if self.undervoltage_ever {
            return Some("undervoltage happened earlier in this boot".into());
        }
        if self.throttled_ever {
            return Some("cpu was throttled earlier in this boot".into());
        }
        None
    }
}

/// `vcgencmd get_throttled` prints exactly `throttled=0x50005`.
pub fn parse_throttled_output(text: &str) -> Option<u32> {
    let value = text.trim().strip_prefix("throttled=")?;
    let digits = value.strip_prefix("0x").unwrap_or(value);
    u32::from_str_radix(digits, 16).ok()
}

/// Sums the jiffy columns of a /proc/stat line into (busy, total).
fn parse_stat_line(line: &str) -> Option<(u64, u64)> {
    let mut fields = line.split_whitespace();
    let label = fields.next()?;
    if !label.starts_with("cpu") {
        return None;
    }
    let jiffies: Vec<u64> = fields.filter_map(|field| field.parse().ok()).collect();
    if jiffies.len() < 4 {
        return None;
    }
    let total: u64 = jiffies.iter().sum();
    // Column 3 is idle and column 4 is iowait; iowait still means the core is
    // available, so it does not count as busy.
    let idle = jiffies[3] + jiffies.get(4).copied().unwrap_or(0);
    Some((total.saturating_sub(idle), total))
}

/// Busy and total jiffies from one reading of one cpu line. A percentage is
/// the difference between two of these, never one on its own.
pub type CpuJiffies = (u64, u64);

/// Parses the whole of /proc/stat into the aggregate line plus one entry per
/// core, in core order.
pub fn parse_proc_stat(text: &str) -> (Option<CpuJiffies>, Vec<CpuJiffies>) {
    let mut total = None;
    let mut cores = Vec::new();
    for line in text.lines() {
        let Some(sample) = parse_stat_line(line) else {
            continue;
        };
        if line.starts_with("cpu ") {
            total = Some(sample);
        } else {
            cores.push(sample);
        }
    }
    (total, cores)
}

pub fn parse_meminfo(text: &str) -> (u64, u64) {
    let mut total_kb = 0u64;
    let mut available_kb = 0u64;
    for line in text.lines() {
        let Some((key, rest)) = line.split_once(':') else {
            continue;
        };
        let Some(value) = rest.split_whitespace().next().and_then(|v| v.parse().ok()) else {
            continue;
        };
        match key {
            "MemTotal" => total_kb = value,
            "MemAvailable" => available_kb = value,
            _ => {}
        }
    }
    let total = total_kb * 1024;
    (total.saturating_sub(available_kb * 1024), total)
}

/// CPU percentages are a difference between two readings, so the sampler holds
/// the previous one.
#[derive(Default)]
pub struct Sampler {
    previous_total: Option<CpuJiffies>,
    previous_cores: Vec<CpuJiffies>,
    /// Reading the throttle word costs a fork, so it is refreshed far more
    /// slowly than the 5 Hz the rest of the monitor runs at.
    last_throttle_check: Option<Instant>,
    cached_throttle: Option<Throttle>,
}

fn busy_fraction(previous: (u64, u64), current: (u64, u64)) -> f64 {
    let busy = current.0.saturating_sub(previous.0) as f64;
    let total = current.1.saturating_sub(previous.1) as f64;
    if total <= 0.0 {
        return 0.0;
    }
    (busy / total).clamp(0.0, 1.0)
}

impl Sampler {
    pub fn sample(&mut self, record_dir: &Path) -> Health {
        let mut health = Health::default();

        if let Ok(text) = std::fs::read_to_string("/proc/stat") {
            let (total, cores) = parse_proc_stat(&text);
            if let (Some(previous), Some(current)) = (self.previous_total, total) {
                health.cpu_busy = busy_fraction(previous, current);
            }
            if self.previous_cores.len() == cores.len() {
                health.cpu_cores = self
                    .previous_cores
                    .iter()
                    .zip(&cores)
                    .map(|(previous, current)| busy_fraction(*previous, *current))
                    .collect();
            }
            self.previous_total = total;
            self.previous_cores = cores;
        }

        if let Ok(text) = std::fs::read_to_string("/proc/loadavg") {
            health.load_one_minute = text
                .split_whitespace()
                .next()
                .and_then(|field| field.parse().ok())
                .unwrap_or(0.0);
        }

        if let Ok(text) = std::fs::read_to_string("/proc/meminfo") {
            let (used, total) = parse_meminfo(&text);
            health.memory_used_bytes = used;
            health.memory_total_bytes = total;
        }

        health.temperature_celsius = read_temperature();
        health.throttle = self.throttle();
        health.disk_free_bytes = free_bytes(record_dir);
        health
    }

    fn throttle(&mut self) -> Option<Throttle> {
        let stale = self
            .last_throttle_check
            .is_none_or(|when| when.elapsed().as_secs() >= 2);
        if stale {
            self.last_throttle_check = Some(Instant::now());
            self.cached_throttle = read_throttle();
        }
        self.cached_throttle
    }
}

/// Newer Pi kernels expose the word without a fork; fall back to `vcgencmd`
/// only when they do not.
fn read_throttle() -> Option<Throttle> {
    const SYSFS: &str = "/sys/devices/platform/soc/soc:firmware/get_throttled";
    if let Ok(text) = std::fs::read_to_string(SYSFS) {
        let digits = text.trim().trim_start_matches("0x");
        if let Ok(raw) = u32::from_str_radix(digits, 16) {
            return Some(Throttle::from_word(raw));
        }
    }
    let output = std::process::Command::new("vcgencmd")
        .arg("get_throttled")
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&output.stdout);
    parse_throttled_output(&text).map(Throttle::from_word)
}

fn read_temperature() -> Option<f64> {
    for zone in 0..8 {
        let path = format!("/sys/class/thermal/thermal_zone{zone}/temp");
        if let Ok(text) = std::fs::read_to_string(&path) {
            if let Ok(millidegrees) = text.trim().parse::<f64>() {
                return Some(millidegrees / 1000.0);
            }
        }
    }
    None
}

fn free_bytes(directory: &Path) -> Option<u64> {
    let path = std::ffi::CString::new(directory.as_os_str().as_encoded_bytes()).ok()?;
    let mut stats: libc::statvfs = unsafe { std::mem::zeroed() };
    // SAFETY: `path` is a valid NUL-terminated string and `stats` is a
    // correctly sized, writable statvfs.
    if unsafe { libc::statvfs(path.as_ptr(), &mut stats) } != 0 {
        return None;
    }
    Some(stats.f_bavail as u64 * stats.f_frsize as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_throttle_word_splits_into_now_and_ever() {
        let throttle = Throttle::from_word(0x50005);
        assert!(throttle.undervoltage_now);
        assert!(throttle.undervoltage_ever);
        assert!(throttle.throttled_now);
        assert!(throttle.throttled_ever);
        assert!(!throttle.frequency_capped);
        assert!(throttle
            .warning()
            .unwrap()
            .contains("undervoltage right now"));
    }

    #[test]
    fn a_past_undervoltage_warns_differently_from_a_present_one() {
        let past = Throttle::from_word(0x10000);
        assert!(!past.undervoltage_now);
        assert!(past.undervoltage_ever);
        assert!(past.warning().unwrap().contains("earlier in this boot"));
        assert_eq!(Throttle::from_word(0).warning(), None);
    }

    #[test]
    fn vcgencmd_output_parses_and_junk_does_not() {
        assert_eq!(parse_throttled_output("throttled=0x50005\n"), Some(0x50005));
        assert_eq!(parse_throttled_output("throttled=0\n"), Some(0));
        assert_eq!(parse_throttled_output("command not found"), None);
        assert_eq!(parse_throttled_output(""), None);
    }

    #[test]
    fn proc_stat_yields_an_aggregate_and_one_entry_per_core() {
        let text = "cpu  100 0 100 800 0 0 0 0 0 0\n\
                    cpu0 50 0 50 400 0 0 0 0 0 0\n\
                    cpu1 50 0 50 400 0 0 0 0 0 0\n\
                    intr 1 2 3\n";
        let (total, cores) = parse_proc_stat(text);
        assert_eq!(total, Some((200, 1000)));
        assert_eq!(cores, vec![(100, 500), (100, 500)]);
    }

    #[test]
    fn busy_fraction_is_the_difference_between_two_readings() {
        // 200 more busy jiffies out of 1000 more total is 20%.
        assert!((busy_fraction((200, 1000), (400, 2000)) - 0.2).abs() < 1e-12);
        // A repeated reading has no elapsed time and must not divide by zero.
        assert_eq!(busy_fraction((200, 1000), (200, 1000)), 0.0);
        // A counter reset must clamp rather than produce a negative percentage.
        assert_eq!(busy_fraction((999, 9999), (1, 10)), 0.0);
    }

    #[test]
    fn iowait_is_not_counted_as_busy() {
        // user=100 nice=0 system=0 idle=0 iowait=900: the core spent its whole
        // interval waiting on the disk, which is not cpu load.
        let (total, _) = parse_proc_stat("cpu  100 0 0 0 900 0 0\n");
        assert_eq!(total, Some((100, 1000)));
    }

    #[test]
    fn meminfo_reports_used_as_total_minus_available() {
        let text = "MemTotal:       8000000 kB\n\
                    MemFree:         100000 kB\n\
                    MemAvailable:   6000000 kB\n";
        let (used, total) = parse_meminfo(text);
        assert_eq!(total, 8_000_000 * 1024);
        assert_eq!(used, 2_000_000 * 1024);
    }

    #[test]
    fn free_space_is_reported_for_a_directory_that_exists() {
        let free = free_bytes(&std::env::temp_dir());
        assert!(free.is_some_and(|bytes| bytes > 0));
        assert_eq!(free_bytes(Path::new("/no/such/place/at/all")), None);
    }
}
