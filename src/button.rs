//! A physical record button, and a light that is on while recording, for a
//! rig whose phone is in a pocket.
//!
//! The button is a momentary switch between a header GPIO and ground; the
//! pin's internal pull-up holds it high until it is pressed. One press starts
//! a recording, the next stops it. The light follows the recorder itself
//! rather than the button, so a recording started from the page lights it
//! too.
//!
//! The kernel's GPIO character device is spoken to directly (`linux/gpio.h`,
//! the v2 ABI). A crate for it would be tidier, but every layout below is a
//! few `#[repr(C)]` structs, and doing without keeps the cross builds free of
//! another dependency to vendor.

use crate::hub::Hub;
use std::fmt;
use std::str::FromStr;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Where the recording light is.
#[derive(Clone, Debug, PartialEq)]
pub enum Led {
    /// A LED the kernel already drives, by its `/sys/class/leds` name. `ACT`
    /// is the Pi's green one.
    Kernel(String),
    /// A plain LED on a header pin, through a resistor to ground.
    Gpio(u32),
}

impl FromStr for Led {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, String> {
        if let Some(pin) = text.strip_prefix("gpio:") {
            return pin
                .parse()
                .map(Led::Gpio)
                .map_err(|_| format!("{pin:?} is not a GPIO number"));
        }
        if text.is_empty() || text.contains('/') {
            return Err("expected a kernel LED name such as ACT, or gpio:<pin>".into());
        }
        Ok(Led::Kernel(text.to_owned()))
    }
}

impl fmt::Display for Led {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Led::Kernel(name) => f.write_str(name),
            Led::Gpio(pin) => write!(f, "gpio:{pin}"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ButtonConfig {
    /// BCM GPIO number, not the header pin number.
    pub pin: u32,
    pub led: Option<Led>,
}

#[derive(Debug, PartialEq)]
pub enum Action {
    Start,
    Stop,
}

/// A press toggles the recorder; a release does nothing.
pub fn action_for(event: u32, recording: bool) -> Option<Action> {
    (event == abi::EVENT_FALLING_EDGE).then_some(if recording {
        Action::Stop
    } else {
        Action::Start
    })
}

/// What the light is doing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Blink {
    Off,
    On,
    /// A recording is running without a stream it should have.
    Fast,
    /// The card is nearly full at the rate recordings are being written.
    Slow,
}

impl Blink {
    /// Half a period: how long the light stays on, then off.
    fn half_period(self) -> Option<Duration> {
        match self {
            Blink::Fast => Some(Duration::from_millis(100)),
            Blink::Slow => Some(Duration::from_millis(500)),
            Blink::Off | Blink::On => None,
        }
    }

    /// Whether the light is lit `elapsed` into the pattern, and how long until
    /// that changes.
    pub fn phase(self, elapsed: Duration) -> (bool, Duration) {
        match self.half_period() {
            None => (self == Blink::On, LIGHT_REFRESH),
            Some(half) => {
                let halves = elapsed.as_millis() / half.as_millis();
                let into = Duration::from_millis((elapsed.as_millis() % half.as_millis()) as u64);
                (halves.is_multiple_of(2), half - into)
            }
        }
    }
}

/// A missing stream outranks a filling card: the card can wait until the
/// recording ends, the missing camera cannot.
pub fn light_for(recording: bool, missing_stream: bool, storage_low: bool) -> Blink {
    match (recording, missing_stream, storage_low) {
        (true, true, _) => Blink::Fast,
        (_, _, true) => Blink::Slow,
        (true, false, false) => Blink::On,
        (false, _, false) => Blink::Off,
    }
}

/// Below this much recording time left, the light says so.
pub const STORAGE_WARNING: Duration = Duration::from_secs(5 * 60);

/// How often the write rate is re-measured. The free space itself costs
/// nothing here, the health monitor already reads it.
pub const STORAGE_CHECK: Duration = Duration::from_secs(5);

/// How long the card lasts at the pace recordings are written, measured from
/// the recorder's own byte count between checks rather than from any scan of
/// the disk. Between recordings the last measured pace stands, so a card that
/// is already too full for another take is flagged before the button is pressed.
#[derive(Default)]
pub struct StorageForecast {
    last: Option<(Instant, u64)>,
    bytes_per_second: Option<f64>,
}

impl StorageForecast {
    /// `bytes_written` is the current recording's total, or zero when idle.
    pub fn observe(&mut self, now: Instant, recording: bool, bytes_written: u64) {
        if !recording {
            self.last = None;
            return;
        }
        if let Some((then, before)) = self.last {
            let elapsed = now.saturating_duration_since(then);
            if elapsed < STORAGE_CHECK {
                return;
            }
            if bytes_written >= before {
                self.bytes_per_second =
                    Some((bytes_written - before) as f64 / elapsed.as_secs_f64());
            }
        }
        self.last = Some((now, bytes_written));
    }

    pub fn seconds_left(&self, free_bytes: Option<u64>) -> Option<f64> {
        let rate = self.bytes_per_second.filter(|rate| *rate > 0.0)?;
        Some(free_bytes? as f64 / rate)
    }

    pub fn low(&self, free_bytes: Option<u64>) -> bool {
        self.seconds_left(free_bytes)
            .is_some_and(|seconds| seconds < STORAGE_WARNING.as_secs_f64())
    }
}

/// Contact bounce on a tactile switch is over within a few milliseconds; this
/// is long enough to swallow it and short enough that no deliberate press is
/// lost.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
const DEBOUNCE: Duration = Duration::from_millis(30);

/// How often the light is compared against the recorder while the button is
/// quiet. This is the lag between a recording started from the page and its
/// light coming on.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
const LIGHT_REFRESH: Duration = Duration::from_millis(200);

/// `linux/gpio.h`, copied rather than bound so the cross builds need no kernel
/// headers. Kept out of the Linux-only code so the layouts are checked by the
/// tests on any machine.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(crate) mod abi {
    pub const MAX_NAME: usize = 32;
    pub const LINES_MAX: usize = 64;
    pub const ATTRS_MAX: usize = 10;

    pub const FLAG_INPUT: u64 = 1 << 2;
    pub const FLAG_OUTPUT: u64 = 1 << 3;
    pub const FLAG_EDGE_FALLING: u64 = 1 << 5;
    pub const FLAG_BIAS_PULL_UP: u64 = 1 << 8;

    pub const ATTR_ID_OUTPUT_VALUES: u32 = 2;
    pub const ATTR_ID_DEBOUNCE: u32 = 3;

    pub const EVENT_FALLING_EDGE: u32 = 2;

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct LineAttribute {
        pub id: u32,
        pub padding: u32,
        /// A union in C: `flags` or `values` as a u64, or `debounce_period_us`
        /// as a u32 in its first four bytes. Every target here is
        /// little-endian, so storing the u32 widened is the same bytes.
        pub value: u64,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct ConfigAttribute {
        pub attr: LineAttribute,
        pub mask: u64,
    }

    #[repr(C)]
    pub struct LineConfig {
        pub flags: u64,
        pub num_attrs: u32,
        pub padding: [u32; 5],
        pub attrs: [ConfigAttribute; ATTRS_MAX],
    }

    #[repr(C)]
    pub struct LineRequest {
        pub offsets: [u32; LINES_MAX],
        pub consumer: [u8; MAX_NAME],
        pub config: LineConfig,
        pub num_lines: u32,
        pub event_buffer_size: u32,
        pub padding: [u32; 5],
        pub fd: i32,
    }

    #[repr(C)]
    pub struct LineEvent {
        pub timestamp_ns: u64,
        pub id: u32,
        pub offset: u32,
        pub seqno: u32,
        pub line_seqno: u32,
        pub padding: [u32; 6],
    }

    #[repr(C)]
    pub struct LineValues {
        pub bits: u64,
        pub mask: u64,
    }

    #[repr(C)]
    pub struct ChipInfo {
        pub name: [u8; MAX_NAME],
        pub label: [u8; MAX_NAME],
        pub lines: u32,
    }

    /// `_IOC(dir, 0xB4, nr, size)`: the generic encoding, which arm64 and x86
    /// share. Read is 2, write is 1.
    pub const fn ioc(dir: u32, nr: u32, size: usize) -> u64 {
        ((dir << 30) | ((size as u32) << 16) | (0xB4 << 8) | nr) as u64
    }

    pub const GET_CHIPINFO: u64 = ioc(2, 0x01, std::mem::size_of::<ChipInfo>());
    pub const GET_LINE: u64 = ioc(3, 0x07, std::mem::size_of::<LineRequest>());
    pub const SET_VALUES: u64 = ioc(3, 0x0F, std::mem::size_of::<LineValues>());
}

#[cfg(target_os = "linux")]
pub use linux::spawn;

#[cfg(not(target_os = "linux"))]
pub fn spawn(_hub: Arc<Hub>, _config: &ButtonConfig) -> anyhow::Result<()> {
    anyhow::bail!("a GPIO button needs Linux")
}

#[cfg(target_os = "linux")]
mod linux {
    use super::abi::*;
    use super::{
        action_for, light_for, Action, Blink, ButtonConfig, Led, StorageForecast, DEBOUNCE,
        LIGHT_REFRESH,
    };
    use crate::hub::Hub;
    use anyhow::{bail, Context, Result};
    use std::fs::{File, OpenOptions};
    use std::io::Write;
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
    use std::path::PathBuf;
    use std::sync::Arc;
    use std::time::Instant;

    /// Opens the button (and the light) now, so a wrong pin or a permission
    /// problem is reported at startup rather than at the first press, then
    /// watches from its own thread.
    pub fn spawn(hub: Arc<Hub>, config: &ButtonConfig) -> Result<()> {
        let chip = open_header_chip()?;
        let button = request_input(&chip, config.pin, DEBOUNCE)
            .with_context(|| format!("requesting GPIO{} as the button", config.pin))?;
        let light = match &config.led {
            None => None,
            Some(Led::Kernel(name)) => Some(Light::Kernel(KernelLed::open(name)?)),
            Some(Led::Gpio(pin)) => Some(Light::Gpio(
                request_output(&chip, *pin)
                    .with_context(|| format!("requesting GPIO{pin} as the light"))?,
            )),
        };
        std::thread::Builder::new()
            .name("button".into())
            .spawn(move || watch(hub, button, light))
            .context("starting the button thread")?;
        Ok(())
    }

    fn watch(hub: Arc<Hub>, button: OwnedFd, mut light: Option<Light>) {
        let mut lit: Option<bool> = None;
        let mut pattern = Blink::Off;
        let mut pattern_started = Instant::now();
        let mut forecast = StorageForecast::default();
        loop {
            let now = Instant::now();
            let recording = hub.is_recording();
            let missing = if recording {
                hub.missing_streams()
            } else {
                Vec::new()
            };
            let status = hub.recording_status();
            forecast.observe(now, recording, status.bytes);
            let storage_low = forecast.low(hub.health().disk_free_bytes);

            let wanted = light_for(recording, !missing.is_empty(), storage_low);
            if wanted != pattern {
                pattern = wanted;
                pattern_started = now;
            }
            let (level, until_change) = pattern.phase(now.duration_since(pattern_started));
            if lit != Some(level) {
                if let Some(light) = &mut light {
                    if let Err(error) = light.set(level) {
                        eprintln!("button: could not set the light: {error:#}");
                    }
                }
                lit = Some(level);
            }
            let timeout = until_change.min(LIGHT_REFRESH).as_millis().max(1) as libc::c_int;

            let mut poll = libc::pollfd {
                fd: button.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let ready = unsafe { libc::poll(&mut poll, 1, timeout) };
            if ready < 0 {
                let error = std::io::Error::last_os_error();
                if error.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                eprintln!("button: giving up: {error}");
                return;
            }
            if ready == 0 {
                continue;
            }
            let mut event: LineEvent = unsafe { std::mem::zeroed() };
            let read = unsafe {
                libc::read(
                    button.as_raw_fd(),
                    (&mut event as *mut LineEvent).cast(),
                    std::mem::size_of::<LineEvent>(),
                )
            };
            if read != std::mem::size_of::<LineEvent>() as isize {
                eprintln!(
                    "button: short event read: {}",
                    std::io::Error::last_os_error()
                );
                continue;
            }
            match action_for(event.id, hub.is_recording()) {
                Some(Action::Start) => match hub.start_recording(None) {
                    Ok(status) => println!("button: recording {}", status.path.unwrap_or_default()),
                    Err(error) => eprintln!("button: could not start recording: {error:#}"),
                },
                Some(Action::Stop) => match hub.stop_recording() {
                    Ok(status) => println!("button: stopped {}", status.path.unwrap_or_default()),
                    Err(error) => eprintln!("button: could not stop recording: {error:#}"),
                },
                None => {}
            }
        }
    }

    enum Light {
        Kernel(KernelLed),
        Gpio(OwnedFd),
    }

    impl Light {
        fn set(&mut self, on: bool) -> Result<()> {
            match self {
                Light::Kernel(led) => led.set(on),
                Light::Gpio(line) => set_value(line, on),
            }
        }
    }

    /// `/sys/class/leds/<name>`. Its trigger is switched off so the kernel
    /// stops blinking it for disk activity; from then on it is ours.
    struct KernelLed {
        brightness: File,
        max: String,
    }

    impl KernelLed {
        fn open(name: &str) -> Result<Self> {
            let directory = PathBuf::from("/sys/class/leds").join(name);
            if !directory.is_dir() {
                let known: Vec<String> = std::fs::read_dir("/sys/class/leds")
                    .map(|entries| {
                        entries
                            .flatten()
                            .map(|entry| entry.file_name().to_string_lossy().into_owned())
                            .collect()
                    })
                    .unwrap_or_default();
                bail!("no LED named {name}; this machine has {known:?}");
            }
            let hint = "the service's user needs write access to it -- `lite_record survive_reboot` grants it";
            std::fs::write(directory.join("trigger"), "none")
                .with_context(|| format!("taking over LED {name}: {hint}"))?;
            let brightness = OpenOptions::new()
                .write(true)
                .open(directory.join("brightness"))
                .with_context(|| format!("opening LED {name}: {hint}"))?;
            let max = std::fs::read_to_string(directory.join("max_brightness"))
                .map(|text| text.trim().to_owned())
                .unwrap_or_else(|_| "1".into());
            Ok(Self { brightness, max })
        }

        fn set(&mut self, on: bool) -> Result<()> {
            let value = if on { self.max.as_str() } else { "0" };
            self.brightness
                .write_all(value.as_bytes())
                .context("writing brightness")?;
            Ok(())
        }
    }

    /// The header's controller: `pinctrl-rp1` on a Pi 5, `pinctrl-bcm2711` on
    /// a Pi 4. The other chips are expanders for the board's own buttons and
    /// power rails, so this is the only label that starts with `pinctrl-`.
    fn open_header_chip() -> Result<File> {
        let mut chips: Vec<PathBuf> = std::fs::read_dir("/dev")
            .context("listing /dev")?
            .flatten()
            .map(|entry| entry.path())
            .filter(|path| {
                path.file_name()
                    .is_some_and(|name| name.to_string_lossy().starts_with("gpiochip"))
            })
            .collect();
        chips.sort();
        let mut seen = Vec::new();
        for path in &chips {
            let chip = File::open(path).with_context(|| {
                format!(
                    "opening {}: the service's user needs to be in the gpio group -- `lite_record survive_reboot` does that",
                    path.display()
                )
            })?;
            let label = chip_label(&chip)?;
            if label.starts_with("pinctrl-") {
                return Ok(chip);
            }
            seen.push(format!("{} ({label})", path.display()));
        }
        bail!("no header GPIO controller: found {seen:?}")
    }

    fn chip_label(chip: &File) -> Result<String> {
        let mut info: ChipInfo = unsafe { std::mem::zeroed() };
        ioctl(
            chip.as_raw_fd(),
            GET_CHIPINFO,
            (&mut info as *mut ChipInfo).cast(),
        )
        .context("reading chip info")?;
        Ok(c_string(&info.label))
    }

    fn c_string(bytes: &[u8]) -> String {
        let end = bytes
            .iter()
            .position(|byte| *byte == 0)
            .unwrap_or(bytes.len());
        String::from_utf8_lossy(&bytes[..end]).into_owned()
    }

    fn request_input(chip: &File, pin: u32, debounce: std::time::Duration) -> Result<OwnedFd> {
        let attribute = ConfigAttribute {
            attr: LineAttribute {
                id: ATTR_ID_DEBOUNCE,
                padding: 0,
                value: debounce.as_micros() as u64,
            },
            mask: 1,
        };
        request_line(
            chip,
            pin,
            FLAG_INPUT | FLAG_BIAS_PULL_UP | FLAG_EDGE_FALLING,
            attribute,
        )
    }

    fn request_output(chip: &File, pin: u32) -> Result<OwnedFd> {
        let attribute = ConfigAttribute {
            attr: LineAttribute {
                id: ATTR_ID_OUTPUT_VALUES,
                padding: 0,
                value: 0,
            },
            mask: 1,
        };
        request_line(chip, pin, FLAG_OUTPUT, attribute)
    }

    fn request_line(
        chip: &File,
        pin: u32,
        flags: u64,
        attribute: ConfigAttribute,
    ) -> Result<OwnedFd> {
        let mut request: LineRequest = unsafe { std::mem::zeroed() };
        request.offsets[0] = pin;
        request.consumer[..b"lite_record".len()].copy_from_slice(b"lite_record");
        request.num_lines = 1;
        request.config.flags = flags;
        request.config.num_attrs = 1;
        request.config.attrs[0] = attribute;
        ioctl(
            chip.as_raw_fd(),
            GET_LINE,
            (&mut request as *mut LineRequest).cast(),
        )?;
        // The kernel handed us a new descriptor; from here it is owned like any other.
        Ok(unsafe { OwnedFd::from_raw_fd(request.fd) })
    }

    fn set_value(line: &OwnedFd, on: bool) -> Result<()> {
        let mut values = LineValues {
            bits: u64::from(on),
            mask: 1,
        };
        ioctl(
            line.as_raw_fd(),
            SET_VALUES,
            (&mut values as *mut LineValues).cast(),
        )
    }

    fn ioctl(fd: i32, request: u64, argument: *mut libc::c_void) -> Result<()> {
        // glibc takes the request as c_ulong, musl as c_int; the bits are the same.
        if unsafe { libc::ioctl(fd, request as _, argument) } < 0 {
            return Err(std::io::Error::last_os_error().into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::abi::*;
    use super::*;
    use std::mem::size_of;

    /// The kernel checks the size it finds in the ioctl number against its own
    /// struct, so a layout slip here is ENOTTY at runtime; these pin the
    /// numbers `linux/gpio.h` produces.
    #[test]
    fn the_layouts_match_the_kernel_headers() {
        assert_eq!(size_of::<LineAttribute>(), 16);
        assert_eq!(size_of::<ConfigAttribute>(), 24);
        assert_eq!(size_of::<LineConfig>(), 272);
        assert_eq!(size_of::<LineRequest>(), 592);
        assert_eq!(size_of::<LineEvent>(), 48);
        assert_eq!(size_of::<LineValues>(), 16);
        assert_eq!(size_of::<ChipInfo>(), 68);
        assert_eq!(GET_CHIPINFO, 0x8044_B401);
        assert_eq!(GET_LINE, 0xC250_B407);
        assert_eq!(SET_VALUES, 0xC010_B40F);
    }

    #[test]
    fn a_press_toggles_and_a_release_is_ignored() {
        assert_eq!(action_for(EVENT_FALLING_EDGE, false), Some(Action::Start));
        assert_eq!(action_for(EVENT_FALLING_EDGE, true), Some(Action::Stop));
        let rising = 1;
        assert_eq!(action_for(rising, false), None);
        assert_eq!(action_for(rising, true), None);
    }

    #[test]
    fn a_missing_stream_flashes_fast_and_a_full_card_slow() {
        assert_eq!(light_for(false, false, false), Blink::Off);
        assert_eq!(light_for(true, false, false), Blink::On);
        assert_eq!(light_for(true, true, false), Blink::Fast);
        assert_eq!(light_for(true, false, true), Blink::Slow);
        assert_eq!(light_for(true, true, true), Blink::Fast);
        // A card too full for another take is flagged before the press.
        assert_eq!(light_for(false, false, true), Blink::Slow);
        // Streams only matter while recording.
        assert_eq!(light_for(false, true, false), Blink::Off);
    }

    #[test]
    fn a_blink_alternates_and_says_when_it_next_changes() {
        let ms = Duration::from_millis;
        assert_eq!(Blink::Fast.phase(ms(0)), (true, ms(100)));
        assert_eq!(Blink::Fast.phase(ms(130)), (false, ms(70)));
        assert_eq!(Blink::Fast.phase(ms(200)), (true, ms(100)));
        assert_eq!(Blink::Slow.phase(ms(1_499)), (true, ms(1)));
        assert_eq!(Blink::Slow.phase(ms(1_500)), (false, ms(500)));
        assert_eq!(Blink::On.phase(ms(999)), (true, LIGHT_REFRESH));
        assert_eq!(Blink::Off.phase(ms(999)), (false, LIGHT_REFRESH));
    }

    #[test]
    fn the_forecast_extrapolates_from_the_recorders_byte_count() {
        let start = Instant::now();
        let mut forecast = StorageForecast::default();
        forecast.observe(start, true, 0);
        // Too soon to measure anything.
        forecast.observe(start + Duration::from_secs(1), true, 10_000_000);
        assert_eq!(forecast.seconds_left(Some(1 << 30)), None);
        // 50 MB in 5 s = 10 MB/s; a gigabyte lasts 100 s.
        forecast.observe(start + STORAGE_CHECK, true, 50_000_000);
        let left = forecast.seconds_left(Some(1_000_000_000)).unwrap();
        assert!((left - 100.0).abs() < 1.0, "{left}");
        assert!(forecast.low(Some(1_000_000_000)));
        assert!(!forecast.low(Some(10_000_000_000)));
        assert!(!forecast.low(None));
        // Between recordings the pace stands, so the warning can precede a press.
        forecast.observe(start + Duration::from_secs(60), false, 0);
        assert!(forecast.low(Some(1_000_000_000)));
        // A new recording starts its count from zero without reading as a
        // negative rate.
        forecast.observe(start + Duration::from_secs(70), true, 0);
        forecast.observe(
            start + Duration::from_secs(70) + STORAGE_CHECK,
            true,
            100_000_000,
        );
        assert!((forecast.seconds_left(Some(1_000_000_000)).unwrap() - 50.0).abs() < 1.0);
    }

    #[test]
    fn the_led_flag_names_a_kernel_led_or_a_pin() {
        assert_eq!("ACT".parse(), Ok(Led::Kernel("ACT".into())));
        assert_eq!("gpio:27".parse(), Ok(Led::Gpio(27)));
        assert!("gpio:x".parse::<Led>().is_err());
        assert!("".parse::<Led>().is_err());
        assert!("../etc".parse::<Led>().is_err());
        assert_eq!(Led::Gpio(27).to_string(), "gpio:27");
        assert_eq!(Led::Kernel("PWR".into()).to_string(), "PWR");
    }
}
