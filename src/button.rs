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
use std::time::Duration;

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
    use super::{action_for, Action, ButtonConfig, Led, DEBOUNCE, LIGHT_REFRESH};
    use crate::hub::Hub;
    use anyhow::{bail, Context, Result};
    use std::fs::{File, OpenOptions};
    use std::io::Write;
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
    use std::path::PathBuf;
    use std::sync::Arc;

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
        loop {
            let recording = hub.is_recording();
            if lit != Some(recording) {
                if let Some(light) = &mut light {
                    if let Err(error) = light.set(recording) {
                        eprintln!("button: could not set the light: {error:#}");
                    }
                }
                lit = Some(recording);
            }

            let mut poll = libc::pollfd {
                fd: button.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let ready =
                unsafe { libc::poll(&mut poll, 1, LIGHT_REFRESH.as_millis() as libc::c_int) };
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
