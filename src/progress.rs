//! The terminal display for `post_process`: numbered steps, and for each one
//! a line that says how far along it is, when it will be done, and how fast it
//! is going relative to the recording itself.
//!
//! Every stage walks the recording in log-time order, so one measure of
//! progress serves them all: how far through the recording's own time span the
//! stage has got. That gives an ETA without knowing anything about the stage,
//! and a real-time factor -- recorded seconds per wall-clock second -- which is
//! the number that says whether a rig can afford to post-process in the field.
//!
//! Colour and in-place redrawing only happen on a terminal. Piped to a file or
//! journald the same information arrives as plain lines, one every
//! [`LOG_TICK`], so a log stays readable and never contains an escape code.

use std::io::{IsTerminal, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// How often a terminal line is redrawn.
const TICK: Duration = Duration::from_millis(250);
/// How often a run that is not on a terminal says it is still alive.
pub const LOG_TICK: Duration = Duration::from_secs(15);
/// An ETA extrapolated from the first moments of a stage is noise; hold it
/// back until this much has been done.
const ETA_AFTER: f64 = 0.02;

/// Where a running stage is, updated by the work and read by the ticker.
#[derive(Default)]
pub struct Gauge {
    /// Log time of the newest message the stage has dealt with.
    at_nanos: AtomicU64,
    /// Whether `at_nanos` has been set at all (zero is a legal log time).
    started: AtomicBool,
    detail: Mutex<String>,
}

impl Gauge {
    pub fn new() -> Arc<Gauge> {
        Arc::new(Gauge::default())
    }

    /// The stage has reached this point of the recording.
    pub fn at(&self, log_time_nanos: u64) {
        self.at_nanos.store(log_time_nanos, Ordering::Relaxed);
        self.started.store(true, Ordering::Relaxed);
    }

    /// A short note shown after the numbers: "1234 scans", "2.1 GB written".
    pub fn detail(&self, text: impl Into<String>) {
        *self.detail.lock().unwrap() = text.into();
    }

    fn position(&self) -> Option<u64> {
        self.started
            .load(Ordering::Relaxed)
            .then(|| self.at_nanos.load(Ordering::Relaxed))
    }
}

/// The recording's log-time span, which is what progress is measured against.
#[derive(Clone, Copy, Debug)]
pub struct Span {
    pub start_nanos: u64,
    pub end_nanos: u64,
}

impl Span {
    /// 0..=1 for a log time within the recording.
    pub fn fraction(&self, at_nanos: u64) -> f64 {
        let length = self.end_nanos.saturating_sub(self.start_nanos);
        if length == 0 {
            return 1.0;
        }
        (at_nanos.saturating_sub(self.start_nanos) as f64 / length as f64).clamp(0.0, 1.0)
    }

    /// Seconds of recording covered up to `at_nanos`.
    pub fn recorded_seconds(&self, at_nanos: u64) -> f64 {
        at_nanos.saturating_sub(self.start_nanos) as f64 / 1e9
    }
}

/// The figures one tick shows, computed apart from the drawing so they can be
/// tested.
#[derive(Debug, PartialEq)]
pub struct Reading {
    pub fraction: Option<f64>,
    pub eta: Option<Duration>,
    /// Recorded seconds per wall-clock second.
    pub realtime_factor: Option<f64>,
}

pub fn reading(span: Option<Span>, at_nanos: Option<u64>, elapsed: Duration) -> Reading {
    let (Some(span), Some(at)) = (span, at_nanos) else {
        return Reading { fraction: None, eta: None, realtime_factor: None };
    };
    let fraction = span.fraction(at);
    let wall = elapsed.as_secs_f64();
    let eta = (fraction >= ETA_AFTER && wall > 0.0)
        .then(|| Duration::from_secs_f64((wall * (1.0 - fraction) / fraction).max(0.0)));
    let realtime_factor = (wall > 0.5).then(|| span.recorded_seconds(at) / wall);
    Reading { fraction: Some(fraction), eta, realtime_factor }
}

/// `1m20s`, `45s`, `2h05m`.
pub fn clock(duration: Duration) -> String {
    let total = duration.as_secs();
    let (hours, minutes, seconds) = (total / 3600, (total % 3600) / 60, total % 60);
    if hours > 0 {
        format!("{hours}h{minutes:02}m")
    } else if minutes > 0 {
        format!("{minutes}m{seconds:02}s")
    } else {
        format!("{seconds}s")
    }
}

/// The step counter and the terminal it draws on.
pub struct Display {
    total_steps: usize,
    step: usize,
    interactive: bool,
    colour: bool,
}

const BOLD: &str = "\x1b[1m";
const DIM: &str = "\x1b[2m";
const CYAN: &str = "\x1b[36m";
const GREEN: &str = "\x1b[32m";
const YELLOW: &str = "\x1b[33m";
const RESET: &str = "\x1b[0m";
const CLEAR_LINE: &str = "\r\x1b[2K";

impl Display {
    /// `total_steps` is how many steps `step` will be called for; work that
    /// out before the first one, since the header of each says "of N".
    pub fn new(total_steps: usize) -> Display {
        let interactive = std::io::stdout().is_terminal();
        // NO_COLOR is the convention (no-color.org): set to anything, no colour.
        let colour = interactive && std::env::var_os("NO_COLOR").is_none_or(|value| value.is_empty());
        Display { total_steps, step: 0, interactive, colour }
    }

    pub fn total_steps(&self) -> usize {
        self.total_steps
    }

    fn paint(&self, code: &str, text: &str) -> String {
        if self.colour {
            format!("{code}{text}{RESET}")
        } else {
            text.to_owned()
        }
    }

    /// A line that is not part of a step: results, notes.
    pub fn note(&self, text: impl AsRef<str>) {
        println!("  {}", text.as_ref());
    }

    pub fn warn(&self, text: impl AsRef<str>) {
        println!("  {}", self.paint(YELLOW, &format!("warning: {}", text.as_ref())));
    }

    /// Announces the next step and runs it, redrawing its progress line until
    /// it returns. `span` is what the gauge's position is measured against;
    /// without one the line shows elapsed time and the detail only.
    pub fn step<T>(&mut self, name: &str, span: Option<Span>, gauge: &Arc<Gauge>, work: impl FnOnce() -> T) -> T {
        self.step += 1;
        let header = format!("[Step {} of {}] {name}", self.step, self.total_steps);
        println!("{}", self.paint(&format!("{BOLD}{CYAN}"), &header));
        let _ = std::io::stdout().flush();

        let running = Arc::new(AtomicBool::new(true));
        let stopping = Arc::clone(&running);
        let watched = Arc::clone(gauge);
        let (interactive, colour) = (self.interactive, self.colour);
        let started = Instant::now();
        let ticker = std::thread::spawn(move || {
            let gauge = watched;
            let mut last_log = Instant::now();
            while stopping.load(Ordering::Relaxed) {
                std::thread::sleep(TICK);
                let line = progress_line(span, &gauge, started.elapsed(), colour);
                if interactive {
                    print!("{CLEAR_LINE}  {line}");
                    let _ = std::io::stdout().flush();
                } else if last_log.elapsed() >= LOG_TICK {
                    println!("  {line}");
                    last_log = Instant::now();
                }
            }
        });

        let result = work();

        running.store(false, Ordering::Relaxed);
        let _ = ticker.join();
        let elapsed = started.elapsed();
        let done = match reading(span, gauge.position(), elapsed).realtime_factor {
            Some(factor) => format!("done in {} ({factor:.1}x rt)", clock(elapsed)),
            None => format!("done in {}", clock(elapsed)),
        };
        if interactive {
            print!("{CLEAR_LINE}");
        }
        println!("  {}", self.paint(GREEN, &done));
        result
    }
}

/// `42%  1m10s  eta 1m35s  1.4x rt  2.1 GB written`
fn progress_line(span: Option<Span>, gauge: &Gauge, elapsed: Duration, colour: bool) -> String {
    let reading = reading(span, gauge.position(), elapsed);
    let mut parts: Vec<String> = Vec::new();
    if let Some(fraction) = reading.fraction {
        parts.push(format!("{:>3.0}%", fraction * 100.0));
    }
    parts.push(clock(elapsed));
    match reading.eta {
        Some(eta) => parts.push(format!("eta {}", clock(eta))),
        None if reading.fraction.is_some() => parts.push("eta --".into()),
        None => {}
    }
    if let Some(factor) = reading.realtime_factor {
        parts.push(format!("{factor:.1}x rt"));
    }
    let detail = gauge.detail.lock().unwrap().clone();
    if !detail.is_empty() {
        parts.push(if colour { format!("{DIM}{detail}{RESET}") } else { detail });
    }
    parts.join("  ")
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPAN: Span = Span { start_nanos: 100_000_000_000, end_nanos: 200_000_000_000 };

    #[test]
    fn the_fraction_is_where_in_the_recording_the_stage_has_got() {
        assert_eq!(SPAN.fraction(100_000_000_000), 0.0);
        assert_eq!(SPAN.fraction(150_000_000_000), 0.5);
        assert_eq!(SPAN.fraction(250_000_000_000), 1.0);
        // Before the start (a stray stamp) is clamped, not negative.
        assert_eq!(SPAN.fraction(1), 0.0);
        let empty = Span { start_nanos: 5, end_nanos: 5 };
        assert_eq!(empty.fraction(5), 1.0);
    }

    #[test]
    fn the_eta_extrapolates_from_the_fraction_done() {
        // Half way after 30 s: another 30 s to go, at 100 s recorded / 30 s = 3.3x.
        let read = reading(Some(SPAN), Some(150_000_000_000), Duration::from_secs(30));
        assert_eq!(read.fraction, Some(0.5));
        assert_eq!(read.eta, Some(Duration::from_secs(30)));
        assert!((read.realtime_factor.unwrap() - 50.0 / 30.0).abs() < 1e-9);
    }

    #[test]
    fn no_eta_before_anything_meaningful_has_happened() {
        // 1% in: too early to extrapolate.
        let read = reading(Some(SPAN), Some(101_000_000_000), Duration::from_secs(2));
        assert_eq!(read.eta, None);
        // No position yet.
        let read = reading(Some(SPAN), None, Duration::from_secs(9));
        assert_eq!(read, Reading { fraction: None, eta: None, realtime_factor: None });
        // No span (a stage that cannot say where it is): elapsed only.
        let read = reading(None, Some(150_000_000_000), Duration::from_secs(9));
        assert_eq!(read.fraction, None);
    }

    #[test]
    fn durations_read_like_a_clock() {
        assert_eq!(clock(Duration::from_secs(7)), "7s");
        assert_eq!(clock(Duration::from_secs(80)), "1m20s");
        assert_eq!(clock(Duration::from_secs(3600 * 2 + 300)), "2h05m");
    }

    #[test]
    fn a_progress_line_carries_the_figures_and_the_detail() {
        let gauge = Gauge::new();
        gauge.at(150_000_000_000);
        gauge.detail("2.1 GB written");
        let line = progress_line(Some(SPAN), &gauge, Duration::from_secs(30), false);
        assert_eq!(line, " 50%  30s  eta 30s  1.7x rt  2.1 GB written");
        assert!(!line.contains('\x1b'));
        let coloured = progress_line(Some(SPAN), &gauge, Duration::from_secs(30), true);
        assert!(coloured.contains("\x1b[2m2.1 GB written\x1b[0m"));
    }

    #[test]
    fn a_step_with_no_span_still_shows_elapsed_and_detail() {
        let gauge = Gauge::new();
        gauge.detail("1234 scans");
        assert_eq!(progress_line(None, &gauge, Duration::from_secs(5), false), "5s  1234 scans");
    }
}
