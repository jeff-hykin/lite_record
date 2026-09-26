//! A USB GPS puck (BU-353N and the like): NMEA 0183 over a serial bridge.
//!
//! Every GGA becomes a `sensor_msgs/NavSatFix` on `<prefix>/fix`, and every
//! checksum-valid sentence is also kept verbatim on `<prefix>/nmea`, because
//! the fix drops what the receiver knows about satellites, speed and course.
//! Stamps are host time at the end of the line, like every other sensor here;
//! the receiver's own UTC is still in the raw sentence.

use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};

use super::{Backend, BackendStatus, GpsConfig, Produced, Sink};
use crate::msgs::Header;
use crate::nmea::{self, Sentence};
use crate::record::now_nanos;

/// A puck emits once a second, so this is a couple of epochs of patience.
const PROBE_WINDOW: Duration = Duration::from_millis(2500);
const SILENCE_BEFORE_ERROR: Duration = Duration::from_secs(5);
const RETRY_INTERVAL: Duration = Duration::from_secs(2);
/// NMEA caps a sentence at 82 characters; anything longer is line noise.
const MAX_LINE: usize = 256;

pub struct GpsBackend {
    config: GpsConfig,
    running: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
    dropped: Arc<AtomicU64>,
    error: Arc<Mutex<Option<String>>>,
    detail: Arc<Mutex<String>>,
}

impl GpsBackend {
    pub fn new(config: GpsConfig) -> Self {
        GpsBackend {
            config,
            running: Arc::new(AtomicBool::new(false)),
            worker: None,
            dropped: Arc::new(AtomicU64::new(0)),
            error: Arc::new(Mutex::new(None)),
            detail: Arc::new(Mutex::new("searching for a gps".into())),
        }
    }
}

impl Backend for GpsBackend {
    fn start(&mut self, sink: Sink) -> Result<()> {
        if self.running.load(Ordering::SeqCst) {
            return Ok(());
        }
        speed_constant(self.config.baud)
            .with_context(|| format!("{} baud is not a rate a serial port can be set to", self.config.baud))?;
        *self.error.lock().unwrap() = None;
        self.running.store(true, Ordering::SeqCst);
        let running = Arc::clone(&self.running);
        let dropped = Arc::clone(&self.dropped);
        let error = Arc::clone(&self.error);
        let detail = Arc::clone(&self.detail);
        let config = self.config.clone();
        // Opening, probing and reopening after an unplug all happen on the
        // worker, so engaging never blocks on a device that is not there yet.
        self.worker = Some(
            std::thread::Builder::new()
                .name("gps".into())
                .spawn(move || {
                    let mut last_reported: Option<String> = None;
                    while running.load(Ordering::SeqCst) {
                        let outcome = match open_receiver(&config, &running) {
                            Ok((path, port)) => {
                                *error.lock().unwrap() = None;
                                eprintln!("gps: reading {} at {} baud", path.display(), config.baud);
                                last_reported = None;
                                let reason = stream(&config, &path, port, &running, &sink, &dropped, &error, &detail);
                                format!("{}: {reason}", path.display())
                            }
                            Err(failure) => format!("{failure:#}"),
                        };
                        if !running.load(Ordering::SeqCst) {
                            break;
                        }
                        *error.lock().unwrap() = Some(outcome.clone());
                        if last_reported.as_deref() != Some(outcome.as_str()) {
                            eprintln!("gps: {outcome}");
                            last_reported = Some(outcome);
                        }
                        let resume_at = Instant::now() + RETRY_INTERVAL;
                        while running.load(Ordering::SeqCst) && Instant::now() < resume_at {
                            std::thread::sleep(Duration::from_millis(100));
                        }
                    }
                })?,
        );
        Ok(())
    }

    fn stop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }

    fn status(&self) -> BackendStatus {
        BackendStatus {
            running: self.running.load(Ordering::SeqCst),
            detail: self.detail.lock().unwrap().clone(),
            error: self.error.lock().unwrap().clone(),
        }
    }
}

/// Reads until the port fails or goes quiet for good. Returns why it stopped.
#[allow(clippy::too_many_arguments)]
fn stream(
    config: &GpsConfig,
    path: &Path,
    mut port: File,
    running: &AtomicBool,
    sink: &Sink,
    dropped: &AtomicU64,
    error: &Mutex<Option<String>>,
    detail: &Mutex<String>,
) -> String {
    let fix_topic = config.fix_topic();
    let nmea_topic = config.nmea_topic();
    let frame = config.naming.root_frame_id();
    let mut lines = LineSplitter::default();
    let mut buffer = [0u8; 512];
    let mut last_sentence = Instant::now();
    *detail.lock().unwrap() = format!("{} @ {} baud, waiting for a sentence", path.display(), config.baud);
    while running.load(Ordering::SeqCst) {
        let count = match port.read(&mut buffer) {
            Ok(count) => count,
            Err(failure) if failure.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(failure) => return format!("read failed: {failure}"),
        };
        for line in lines.push(&buffer[..count]) {
            let Some(sentence) = nmea::parse(&line) else {
                continue;
            };
            last_sentence = Instant::now();
            if config.nmea
                && !sink(Produced::Nmea {
                    topic: nmea_topic.clone(),
                    sentence: line.clone(),
                })
            {
                dropped.fetch_add(1, Ordering::Relaxed);
            }
            if let Sentence::Gga(gga) = sentence {
                *detail.lock().unwrap() = describe(path, config.baud, &gga);
                let fix = gga.to_nav_sat_fix(Header::new(now_nanos(), frame.clone()));
                if !sink(Produced::NavSatFix {
                    topic: fix_topic.clone(),
                    fix: Box::new(fix),
                }) {
                    dropped.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        if last_sentence.elapsed() >= SILENCE_BEFORE_ERROR {
            // An unplugged usb-serial bridge on Linux reads as endless zero-byte
            // timeouts rather than an error, so the node vanishing is the tell.
            if !path.exists() {
                return "device went away".into();
            }
            *error.lock().unwrap() = Some(format!(
                "{}: no NMEA for {}s (wrong baud rate?)",
                path.display(),
                last_sentence.elapsed().as_secs()
            ));
        } else {
            *error.lock().unwrap() = None;
        }
    }
    "stopped".into()
}

fn describe(path: &Path, baud: u32, gga: &nmea::Gga) -> String {
    let fix = match (gga.has_fix(), gga.latitude, gga.longitude) {
        (true, Some(latitude), Some(longitude)) => format!(
            "fix {latitude:.5}, {longitude:.5}, hdop {}",
            gga.hdop.map_or("?".into(), |hdop| hdop.to_string())
        ),
        _ => "no fix".into(),
    };
    format!("{} @ {baud} baud · {fix} · {} sats", path.display(), gga.satellites)
}

/// The configured device, or the first serial port that answers in NMEA.
fn open_receiver(config: &GpsConfig, running: &AtomicBool) -> Result<(PathBuf, File)> {
    let device = config.device.trim();
    if !device.is_empty() {
        let path = PathBuf::from(device);
        let port = open_serial(&path, config.baud).with_context(|| format!("opening {device}"))?;
        return Ok((path, port));
    }
    let candidates = candidate_ports();
    if candidates.is_empty() {
        anyhow::bail!("no usb serial port found; is the gps plugged in?");
    }
    for path in &candidates {
        if !running.load(Ordering::SeqCst) {
            break;
        }
        let Ok(mut port) = open_serial(path, config.baud) else {
            continue;
        };
        if speaks_nmea(&mut port) {
            return Ok((path.clone(), port));
        }
    }
    anyhow::bail!(
        "none of {} sent NMEA at {} baud",
        candidates.iter().map(|path| path.display().to_string()).collect::<Vec<_>>().join(", "),
        config.baud
    )
}

fn speaks_nmea(port: &mut File) -> bool {
    let started = Instant::now();
    let mut lines = LineSplitter::default();
    let mut buffer = [0u8; 256];
    while started.elapsed() < PROBE_WINDOW {
        let Ok(count) = port.read(&mut buffer) else {
            return false;
        };
        if lines.push(&buffer[..count]).iter().any(|line| nmea::parse(line).is_some()) {
            return true;
        }
    }
    false
}

/// USB serial ports, stable names first. `/dev/serial/by-id` is Linux's
/// udev-maintained list of exactly the USB-attached ones, so on a Pi it never
/// includes the GPIO UART; the raw node names are the fallback for a system
/// without udev, and the `cu.` names are macOS's.
pub fn candidate_ports() -> Vec<PathBuf> {
    let mut found: Vec<PathBuf> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    let mut add = |path: PathBuf| {
        let canonical = std::fs::canonicalize(&path).unwrap_or_else(|_| path.clone());
        if seen.insert(canonical) {
            found.push(path);
        }
    };
    for entry in sorted_entries(Path::new("/dev/serial/by-id")) {
        add(entry);
    }
    for entry in sorted_entries(Path::new("/dev")) {
        let name = entry.file_name().unwrap_or_default().to_string_lossy().into_owned();
        if ["ttyUSB", "ttyACM", "cu.usbserial", "cu.usbmodem"]
            .iter()
            .any(|prefix| name.starts_with(prefix))
        {
            add(entry);
        }
    }
    found
}

fn sorted_entries(directory: &Path) -> Vec<PathBuf> {
    let mut entries: Vec<PathBuf> = std::fs::read_dir(directory)
        .map(|listing| listing.filter_map(|entry| entry.ok().map(|entry| entry.path())).collect())
        .unwrap_or_default();
    entries.sort();
    entries
}

/// Splits a byte stream into lines, tolerating the garbage a mid-sentence
/// open or a wrong baud rate produces.
#[derive(Default)]
struct LineSplitter {
    partial: Vec<u8>,
    /// The current line outgrew `MAX_LINE`; drop it through its line ending.
    overflowed: bool,
}

impl LineSplitter {
    fn push(&mut self, bytes: &[u8]) -> Vec<String> {
        let mut lines = Vec::new();
        for &byte in bytes {
            if byte == b'\n' || byte == b'\r' {
                if !self.partial.is_empty() && !self.overflowed {
                    lines.push(String::from_utf8_lossy(&self.partial).into_owned());
                }
                self.partial.clear();
                self.overflowed = false;
            } else if self.partial.len() < MAX_LINE {
                self.partial.push(byte);
            } else {
                self.partial.clear();
                self.overflowed = true;
            }
        }
        lines
    }
}

fn speed_constant(baud: u32) -> Option<libc::speed_t> {
    Some(match baud {
        4800 => libc::B4800,
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        _ => return None,
    })
}

/// Raw 8N1, reads that return after half a second of silence so the worker
/// can notice `stop`. Opened non-blocking so a port with no carrier cannot
/// hang the open itself, then switched back to blocking for the reads.
fn open_serial(path: &Path, baud: u32) -> std::io::Result<File> {
    use std::os::fd::AsRawFd;
    use std::os::unix::fs::OpenOptionsExt;

    let speed = speed_constant(baud).ok_or_else(|| {
        std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("unsupported baud {baud}"))
    })?;
    let file = std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
        .open(path)?;
    let fd = file.as_raw_fd();
    let check = |result: libc::c_int| {
        if result < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(())
        }
    };
    // SAFETY: fd is open for the lifetime of `file`, and termios is plain data
    // that tcgetattr fills in completely before anything reads it.
    unsafe {
        let mut settings: libc::termios = std::mem::zeroed();
        check(libc::tcgetattr(fd, &mut settings))?;
        libc::cfmakeraw(&mut settings);
        settings.c_cflag |= libc::CLOCAL | libc::CREAD;
        settings.c_cc[libc::VMIN] = 0;
        settings.c_cc[libc::VTIME] = 5;
        check(libc::cfsetispeed(&mut settings, speed))?;
        check(libc::cfsetospeed(&mut settings, speed))?;
        check(libc::tcsetattr(fd, libc::TCSANOW, &settings))?;
        check(libc::tcflush(fd, libc::TCIFLUSH))?;
        let flags = libc::fcntl(fd, libc::F_GETFL);
        check(flags)?;
        check(libc::fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK))?;
    }
    Ok(file)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines_split_across_reads_are_reassembled() {
        let mut lines = LineSplitter::default();
        assert!(lines.push(b"$GPGGA,1").is_empty());
        assert_eq!(lines.push(b"23*00\r\n$GPR"), vec!["$GPGGA,123*00".to_string()]);
        assert_eq!(lines.push(b"MC*00\r\n"), vec!["$GPRMC*00".to_string()]);
    }

    #[test]
    fn an_endless_line_of_noise_is_discarded_not_buffered() {
        let mut lines = LineSplitter::default();
        assert!(lines.push(&[b'x'; 10_000]).is_empty());
        assert!(lines.partial.len() <= MAX_LINE);
        assert_eq!(lines.push(b"\n$ok*00\n"), vec!["$ok*00".to_string()]);
    }

    #[test]
    fn only_real_serial_rates_are_accepted() {
        assert!(speed_constant(4800).is_some());
        assert!(speed_constant(115200).is_some());
        assert!(speed_constant(4801).is_none());
    }

    #[test]
    fn a_pseudo_terminal_speaking_nmea_is_read_into_fixes() {
        // A pty stands in for the serial bridge: the same termios calls, the
        // same read path, no hardware.
        let (controller, device) = pty_pair();
        let config = GpsConfig {
            device: device.display().to_string(),
            ..GpsConfig::default()
        };
        let received: Arc<Mutex<Vec<Produced>>> = Arc::default();
        let sink: Sink = {
            let received = Arc::clone(&received);
            Arc::new(move |produced| {
                received.lock().unwrap().push(produced);
                true
            })
        };
        let mut backend = GpsBackend::new(config);
        backend.start(sink).unwrap();
        std::thread::sleep(Duration::from_millis(200));
        let mut writer = &controller;
        use std::io::Write;
        writer
            .write_all(
                b"garbage\r\n$GPGGA,175411.000,3745.7561,N,12229.6726,W,1,8,1.30,55.8,M,-25.4,M,,0000*5A\r\n\
                  $GPRMC,175411.000,A,3745.7561,N,12229.6726,W,0.09,179.20,260926,,,A*79\r\n",
            )
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(3);
        while received.lock().unwrap().len() < 3 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(20));
        }
        backend.stop();
        let received = received.lock().unwrap();
        let topics: Vec<&str> = received.iter().map(Produced::topic).collect();
        assert_eq!(topics, vec!["/gps/nmea", "/gps/fix", "/gps/nmea"]);
        let Produced::NavSatFix { fix, .. } = &received[1] else {
            panic!("not a fix")
        };
        assert!((fix.latitude - 37.7626).abs() < 1e-3 && (fix.longitude + 122.4945).abs() < 1e-3);
        assert_eq!(fix.header.frame_id, "gps_link");
        assert!(backend.status().detail.contains("8 sats"));
    }

    fn pty_pair() -> (File, PathBuf) {
        use std::os::fd::FromRawFd;
        // SAFETY: standard posix_openpt sequence; the fd is owned by the File.
        unsafe {
            let fd = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
            assert!(fd >= 0);
            assert_eq!(libc::grantpt(fd), 0);
            assert_eq!(libc::unlockpt(fd), 0);
            let name = std::ffi::CStr::from_ptr(libc::ptsname(fd)).to_string_lossy().into_owned();
            (File::from_raw_fd(fd), PathBuf::from(name))
        }
    }
}
