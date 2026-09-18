//! Interactive network setup for a headless rig: the wifi profiles it needs to
//! come back on its own, and the ethernet pair that lets one port serve both a
//! Mid-360 and an ordinary router.
//!
//! This exists because of a specific failure. On 2026-09-11 adding one wifi
//! network to dimpi5 left every `/etc/netplan/90-NM-*.yaml` at zero bytes,
//! which silently deleted every other saved network along with the ethernet
//! profile. From outside the Pi that is indistinguishable from dead hardware:
//! it boots, the radio is enabled, `wlan0` simply sits DOWN forever because it
//! has no profile for any SSID in range. It cost an afternoon and a cable
//! strung between a laptop and the rig to find. [`diagnose`] names that case in
//! one line, which is the whole reason this is a subcommand rather than a
//! paragraph in the README.
//!
//! Passwords are written straight into NetworkManager's keyfile rather than
//! passed to `nmcli ... wifi-sec.psk`, because argv is world-readable through
//! /proc for as long as the command runs — the same reason [`crate::privileged`]
//! feeds sudo its password on stdin. Nothing here ever puts a secret in an
//! argument vector, an environment variable it did not receive, or a log line.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};

use anyhow::{bail, Context, Result};

use crate::privileged::{is_root, Planned, Secret};

/// Where NetworkManager keeps saved profiles. Every file here is mode 0600 and
/// owned by root; NM refuses to load one that is group- or world-readable,
/// which is also why this module needs root rather than asking for a password.
pub const KEYFILE_DIR: &str = "/etc/NetworkManager/system-connections";

/// netplan's generated-profile directory. These files are not the source of
/// truth — NM's own keyfiles are — but netplan rewrites them, and when that
/// rewrite truncates them the profiles they described go with it.
pub const NETPLAN_DIR: &str = "/etc/netplan";

/// The ethernet profile that tries DHCP before the lidar's static address.
pub const ETHERNET_DHCP_PROFILE: &str = "eth0-dhcp";

/// Tried before the lidar profile, so a cable into a router wins; the lidar
/// profile sits at 0 and catches the fallthrough.
pub const ETHERNET_DHCP_PRIORITY: i32 = 10;

/// How long to wait for a DHCP server before giving up and letting the static
/// lidar profile take the port. Every boot with a lidar on the cable pays this
/// once, so it trades directly against how long the point stream takes to
/// appear.
pub const ETHERNET_DHCP_TIMEOUT_SECONDS: u32 = 15;

/// Below this much free space, rewriting a config file in place risks
/// truncating it to nothing. Sized to be roomy rather than exact: the point is
/// to shout well before the filesystem is actually at zero.
pub const DISK_HEADROOM_BYTES: u64 = 2 * 1024 * 1024 * 1024;

/// A wifi network as the operator described it.
pub struct WifiNetwork {
    pub ssid: String,
    pub password: Secret,
    /// Higher wins when more than one saved network is in range.
    pub priority: i32,
}

/// A saved NetworkManager profile, as `nmcli -t connection show` reports it.
#[derive(Debug, Clone, PartialEq)]
pub struct SavedConnection {
    pub name: String,
    pub kind: String,
    pub device: Option<String>,
    pub autoconnect: bool,
    pub priority: i32,
}

impl SavedConnection {
    pub fn is_wifi(&self) -> bool {
        self.kind.contains("wireless") || self.kind == "wifi"
    }

    pub fn is_ethernet(&self) -> bool {
        self.kind.contains("ethernet")
    }
}

/// An access point the radio can currently see.
#[derive(Debug, Clone, PartialEq)]
pub struct VisibleNetwork {
    pub ssid: String,
    /// 0-100 as NetworkManager scales it.
    pub signal: u8,
}

/// One interface, as `nmcli -t device status` reports it.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceStatus {
    pub device: String,
    pub kind: String,
    pub state: String,
    pub connection: Option<String>,
}

impl DeviceStatus {
    pub fn is_connected(&self) -> bool {
        self.state.starts_with("connected")
    }
}

/// Everything [`diagnose`] reads, gathered into plain data so the rules can be
/// tested without a NetworkManager, a radio, or a Pi.
#[derive(Debug, Default, Clone)]
pub struct Survey {
    /// Path and size of every file in [`NETPLAN_DIR`].
    pub netplan_files: Vec<(PathBuf, u64)>,
    pub saved: Vec<SavedConnection>,
    pub visible: Vec<VisibleNetwork>,
    pub devices: Vec<DeviceStatus>,
    pub wifi_radio_enabled: bool,
    /// Free and total bytes on the filesystem holding [`NETPLAN_DIR`]. A full
    /// card belongs in a network report because of how the two are connected;
    /// see the disk rule in [`diagnose`].
    pub root_free_bytes: u64,
    pub root_total_bytes: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Worth saying out loud, but nothing is wrong.
    Note,
    /// Works now, will bite later or in a different room.
    Warn,
    /// Something that was configured is gone, or cannot work as it stands.
    Broken,
}

impl Severity {
    fn marker(self) -> &'static str {
        match self {
            Severity::Note => "  ",
            Severity::Warn => " !",
            Severity::Broken => "!!",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    pub severity: Severity,
    pub headline: String,
    /// What to do about it, in one or two sentences.
    pub detail: String,
}

/// The checks, in the order an operator wants to read them: what is destroyed
/// first, then what is merely missing, then what is only surprising.
///
/// Every rule here is one that would have shortened a real debugging session.
pub fn diagnose(survey: &Survey) -> Vec<Finding> {
    let mut findings = Vec::new();

    // Upstream of every other rule here, and the reason a disk check lives in
    // a network tool at all. netplan rewrites its files in place rather than
    // writing a temp file and renaming, so on a full filesystem the truncate
    // succeeds, the write fails, and the profile is gone with nothing logged.
    // That is the most likely account of the dimpi5 failure below: a 114.8 GB
    // recording filled the card at 03:43 and the files were emptied at 04:49.
    if survey.root_total_bytes > 0 && survey.root_free_bytes < DISK_HEADROOM_BYTES {
        findings.push(Finding {
            severity: Severity::Broken,
            headline: format!(
                "only {} free on the recording filesystem, of {}",
                human_bytes(survey.root_free_bytes),
                human_bytes(survey.root_total_bytes)
            ),
            detail: "A full card does not just stop recordings. Config files rewritten in place \
                     are truncated to zero and never refilled, which is how a rig loses its saved \
                     wifi networks and comes back looking like dead hardware. Free space before \
                     trusting anything else in this report."
                .into(),
        });
    }

    let emptied: Vec<&PathBuf> = survey
        .netplan_files
        .iter()
        .filter(|(path, size)| *size == 0 && path.extension().is_some_and(|ext| ext == "yaml"))
        .map(|(path, _)| path)
        .collect();
    if !emptied.is_empty() {
        findings.push(Finding {
            severity: Severity::Broken,
            headline: format!(
                "{} netplan file(s) are 0 bytes: {}",
                emptied.len(),
                emptied
                    .iter()
                    .map(|path| path
                        .file_name()
                        .map(|name| name.to_string_lossy().into_owned())
                        .unwrap_or_default())
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
            detail: "Whatever profiles these described are gone. This is how dimpi5 lost every \
                     saved wifi network at once on 2026-09-11, minutes after a new network was \
                     added. Re-add the networks here; profiles written by this tool live in \
                     NetworkManager's own keyfiles and are not affected by a netplan rewrite."
                .into(),
        });
    }

    if !survey.wifi_radio_enabled {
        findings.push(Finding {
            severity: Severity::Broken,
            headline: "the wifi radio is switched off".into(),
            detail: "`nmcli radio wifi on` turns it back on. A soft block survives a reboot, so \
                     this looks exactly like a broken antenna until you check."
                .into(),
        });
    }

    let saved_wifi: Vec<&SavedConnection> = survey.saved.iter().filter(|c| c.is_wifi()).collect();
    let saved_ssids: BTreeSet<&str> = saved_wifi.iter().map(|c| c.name.as_str()).collect();

    if saved_wifi.is_empty() {
        findings.push(Finding {
            severity: Severity::Broken,
            headline: "no wifi network is saved at all".into(),
            detail: "This rig cannot join any wireless network, on this bench or anywhere else. \
                     Add one before it leaves the room — a headless Pi with no saved network and \
                     no cable has no way back in."
                .into(),
        });
    }

    let wifi_device = survey.devices.iter().find(|d| d.kind == "wifi");
    let wifi_connected = wifi_device.is_some_and(DeviceStatus::is_connected);

    // The exact shape of the 2026-09-11 failure: a strong access point sitting
    // right there with nothing saved for it.
    if !wifi_connected {
        let mut in_range: Vec<&VisibleNetwork> = survey
            .visible
            .iter()
            .filter(|ap| !ap.ssid.is_empty() && !saved_ssids.contains(ap.ssid.as_str()))
            .collect();
        in_range.sort_by_key(|ap| std::cmp::Reverse(ap.signal));
        in_range.dedup_by(|a, b| a.ssid == b.ssid);
        if let Some(best) = in_range.first().filter(|ap| ap.signal >= 50) {
            findings.push(Finding {
                severity: Severity::Broken,
                headline: format!(
                    "{:?} is in range at {}% and has no saved password",
                    best.ssid, best.signal
                ),
                detail: "The radio is fine and the network is right there; there is simply no \
                         profile for it. Add it below."
                    .into(),
            });
        }
    }

    // A rig whose only ethernet profile is the lidar's static address cannot
    // pick up an address from a router, which is the obvious thing to try when
    // wifi has failed.
    let ethernet: Vec<&SavedConnection> =
        survey.saved.iter().filter(|c| c.is_ethernet()).collect();
    if !ethernet.is_empty() && !ethernet.iter().any(|c| c.name == ETHERNET_DHCP_PROFILE) {
        findings.push(Finding {
            severity: Severity::Warn,
            headline: "the ethernet port has no DHCP profile".into(),
            detail: format!(
                "Only static profiles are saved ({}), so a cable into a router gets no address \
                 and the port is useless for recovery. Option 3 adds {ETHERNET_DHCP_PROFILE} \
                 above them, which falls back to the static profile when no DHCP server answers.",
                ethernet
                    .iter()
                    .map(|c| c.name.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        });
    }

    for connection in &saved_wifi {
        if connection.autoconnect {
            continue;
        }
        findings.push(Finding {
            severity: Severity::Warn,
            headline: format!("{:?} will not connect by itself", connection.name),
            detail: "autoconnect is off, so it only joins when somebody asks it to — which \
                     nobody can do on a rig with no screen."
                .into(),
        });
    }

    if findings.is_empty() {
        findings.push(Finding {
            severity: Severity::Note,
            headline: "nothing to report".into(),
            detail: format!(
                "{} wifi network(s) saved, radio on, netplan intact.",
                saved_wifi.len()
            ),
        });
    }

    findings
}

/// Renders a survey and its findings the way the subcommand prints them.
pub fn report(survey: &Survey) -> String {
    let mut out = String::new();
    for device in &survey.devices {
        if !is_physical_port(&device.kind) {
            continue;
        }
        let _ = writeln!(
            out,
            "  {:<11} {:<28} {}",
            device.device,
            device.state,
            device.connection.as_deref().unwrap_or("--")
        );
    }

    if survey.root_total_bytes > 0 {
        let _ = writeln!(
            out,
            "\n  disk     {} free of {}",
            human_bytes(survey.root_free_bytes),
            human_bytes(survey.root_total_bytes)
        );
    }

    let findings = diagnose(survey);
    let _ = writeln!(out);
    for finding in &findings {
        let _ = writeln!(out, "{} {}", finding.severity.marker(), finding.headline);
        for line in wrap(&finding.detail, 72) {
            let _ = writeln!(out, "     {line}");
        }
    }
    out
}

/// Whether this is a port somebody could plug a cable or an antenna into.
/// `p2p-dev-wlan0` is NetworkManager bookkeeping, `tailscale0` is a tunnel
/// that appears and disappears on its own, and neither belongs in a report
/// about how the rig reaches the network.
fn is_physical_port(kind: &str) -> bool {
    !matches!(kind, "wifi-p2p" | "loopback" | "tun" | "tap" | "bridge" | "vlan")
}

fn human_bytes(bytes: u64) -> String {
    match bytes {
        b if b >= 1 << 30 => format!("{:.1} GB", b as f64 / (1u64 << 30) as f64),
        b if b >= 1 << 20 => format!("{:.0} MB", b as f64 / (1u64 << 20) as f64),
        b => format!("{b} bytes"),
    }
}

/// Wraps on whitespace. Long enough for the terminal a Pi is set up from, and
/// it keeps the detail text readable without pulling in a formatting crate.
fn wrap(text: &str, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut current = String::new();
    for word in text.split_whitespace() {
        if !current.is_empty() && current.len() + 1 + word.len() > width {
            lines.push(std::mem::take(&mut current));
        }
        if !current.is_empty() {
            current.push(' ');
        }
        current.push_str(word);
    }
    if !current.is_empty() {
        lines.push(current);
    }
    lines
}

/// Splits one line of `nmcli -t` output. nmcli escapes a literal colon inside a
/// field as `\:` and a literal backslash as `\\`, so a naive `split(':')` tears
/// an SSID like `Guest:5G` in half and loses the second word.
pub fn split_terse(line: &str) -> Vec<String> {
    let mut fields = vec![String::new()];
    let mut characters = line.chars();
    while let Some(character) = characters.next() {
        match character {
            '\\' => {
                if let Some(escaped) = characters.next() {
                    fields
                        .last_mut()
                        .expect("there is always a current field")
                        .push(escaped);
                }
            }
            ':' => fields.push(String::new()),
            other => fields
                .last_mut()
                .expect("there is always a current field")
                .push(other),
        }
    }
    fields
}

/// Parses `nmcli -t -f NAME,TYPE,DEVICE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show`.
pub fn parse_connections(output: &str) -> Vec<SavedConnection> {
    output
        .lines()
        .filter(|line| !line.trim().is_empty())
        .filter_map(|line| {
            let fields = split_terse(line);
            if fields.len() < 5 {
                return None;
            }
            Some(SavedConnection {
                name: fields[0].clone(),
                kind: fields[1].clone(),
                device: Some(fields[2].clone()).filter(|d| !d.is_empty() && d != "--"),
                autoconnect: fields[3] == "yes",
                priority: fields[4].parse().unwrap_or(0),
            })
        })
        .collect()
}

/// Parses `nmcli -t -f SSID,SIGNAL device wifi list`.
///
/// A hidden network reports an empty SSID; those are dropped, since there is
/// nothing an operator could pick from the list.
pub fn parse_scan(output: &str) -> Vec<VisibleNetwork> {
    let mut networks: Vec<VisibleNetwork> = output
        .lines()
        .filter(|line| !line.trim().is_empty())
        .filter_map(|line| {
            let fields = split_terse(line);
            if fields.len() < 2 || fields[0].is_empty() {
                return None;
            }
            Some(VisibleNetwork {
                ssid: fields[0].clone(),
                signal: fields[1].parse().unwrap_or(0),
            })
        })
        .collect();
    // The same SSID comes back once per band and per access point. Keep the
    // strongest sighting of each.
    networks.sort_by(|a, b| a.ssid.cmp(&b.ssid).then(b.signal.cmp(&a.signal)));
    networks.dedup_by(|a, b| a.ssid == b.ssid);
    networks.sort_by_key(|network| std::cmp::Reverse(network.signal));
    networks
}

/// Parses `nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status`.
pub fn parse_devices(output: &str) -> Vec<DeviceStatus> {
    output
        .lines()
        .filter(|line| !line.trim().is_empty())
        .filter_map(|line| {
            let fields = split_terse(line);
            if fields.len() < 4 {
                return None;
            }
            Some(DeviceStatus {
                device: fields[0].clone(),
                kind: fields[1].clone(),
                state: fields[2].clone(),
                connection: Some(fields[3].clone()).filter(|c| !c.is_empty() && c != "--"),
            })
        })
        .collect()
}

/// An SSID is 1-32 bytes of anything, but a newline would let one close the
/// `[wifi]` section of the keyfile and open another, so the check is a
/// correctness requirement rather than tidiness.
fn validate_ssid(ssid: &str) -> Result<()> {
    if ssid.is_empty() {
        bail!("an ssid cannot be empty");
    }
    if ssid.len() > 32 {
        bail!("{ssid:?} is {} bytes; the limit is 32", ssid.len());
    }
    if ssid.chars().any(|c| c.is_control()) {
        bail!("{ssid:?} contains a control character");
    }
    Ok(())
}

/// WPA-PSK is 8-63 characters, or exactly 64 hex digits for a pre-hashed key.
/// Rejecting a short one here beats writing a profile that NetworkManager will
/// refuse at activation time, on a rig with nobody watching.
fn validate_psk(password: &Secret) -> Result<()> {
    let length = password.exposed_len();
    let hex = password.is_ascii_hex();
    if length == 64 && hex {
        return Ok(());
    }
    if !(8..=63).contains(&length) {
        bail!("a wpa password is 8 to 63 characters (or 64 hex digits); this one is {length}");
    }
    if password.contains_control() {
        bail!("that password contains a control character");
    }
    Ok(())
}

/// The filename a profile is stored under. The SSID is not usable directly: one
/// containing `/` would write outside [`KEYFILE_DIR`], and NetworkManager does
/// not require the filename to match the connection id anyway.
pub fn keyfile_name(ssid: &str) -> String {
    let safe: String = ssid
        .chars()
        .map(|c| match c.is_ascii_alphanumeric() || c == '-' || c == '_' {
            true => c,
            false => '_',
        })
        .collect();
    format!("{}.nmconnection", safe.trim_matches('_'))
}

/// Renders the NetworkManager keyfile for a wifi network.
///
/// The real SSID goes in the body even when the filename had to be sanitised,
/// so a network named `Cafe/Guest` still joins the right access point.
pub fn keyfile_for(network: &WifiNetwork, interface: &str, uuid: &str) -> Result<String> {
    validate_ssid(&network.ssid)?;
    validate_psk(&network.password)?;
    let mut file = String::new();
    let _ = write!(
        file,
        "[connection]\n\
         id={ssid}\n\
         uuid={uuid}\n\
         type=wifi\n\
         interface-name={interface}\n\
         autoconnect=true\n\
         autoconnect-priority={priority}\n\
         \n\
         [wifi]\n\
         mode=infrastructure\n\
         ssid={ssid}\n\
         \n\
         [wifi-security]\n\
         key-mgmt=wpa-psk\n\
         psk={psk}\n\
         \n\
         [ipv4]\n\
         method=auto\n\
         \n\
         [ipv6]\n\
         addr-gen-mode=default\n\
         method=auto\n",
        ssid = network.ssid,
        priority = network.priority,
        psk = network.password.expose(),
    );
    Ok(file)
}

/// A random v4 uuid, read from the kernel rather than a crate: this is the only
/// thing in the module that needs randomness, and a dependency for 16 bytes
/// would have to cross-compile for the Pi like everything else.
pub fn random_uuid() -> Result<String> {
    // read_exact, never `fs::read`: /dev/urandom is an endless stream with no
    // EOF, so reading "the file" spins forever allocating until the machine
    // runs out of memory. The uuid test caught this by hanging.
    use std::io::Read as _;
    let mut bytes = [0u8; 16];
    std::fs::File::open("/dev/urandom")
        .context("could not open /dev/urandom")?
        .read_exact(&mut bytes)
        .context("short read from /dev/urandom")?;
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    let hex: String = bytes.iter().map(|byte| format!("{byte:02x}")).collect();
    Ok(format!(
        "{}-{}-{}-{}-{}",
        &hex[0..8],
        &hex[8..12],
        &hex[12..16],
        &hex[16..20],
        &hex[20..32]
    ))
}

/// Writes the profile and hands NetworkManager the two commands that pick it
/// up. The file is created 0600 before any secret reaches it, so the password
/// is never briefly world-readable on disk.
pub fn save_wifi(network: &WifiNetwork, interface: &str, directory: &Path) -> Result<PathBuf> {
    let uuid = random_uuid()?;
    let contents = keyfile_for(network, interface, &uuid)?;
    let path = directory.join(keyfile_name(&network.ssid));
    write_private(&path, &contents)
        .with_context(|| format!("could not write {}", path.display()))?;
    Ok(path)
}

#[cfg(unix)]
fn write_private(path: &Path, contents: &str) -> Result<()> {
    use std::os::unix::fs::OpenOptionsExt;
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .mode(0o600)
        .open(path)?;
    file.write_all(contents.as_bytes())?;
    file.sync_all()?;
    Ok(())
}

/// The two commands that turn a freshly written keyfile into a live connection.
/// Neither carries a secret, so both are safe to show and to log.
pub fn apply_plan(profile: &str) -> Vec<Planned> {
    vec![
        Planned::root(
            "a keyfile written behind NetworkManager's back is invisible until it rereads the directory",
            &["nmcli", "connection", "reload"],
        ),
        Planned::root(
            "joining now means the rig is on the network in this session, not only after a reboot",
            &["nmcli", "connection", "up", profile],
        ),
    ]
}

/// Deleting goes through nmcli rather than unlinking the file, so NetworkManager
/// tears down a live connection instead of keeping it up until the next reboot.
pub fn forget_plan(profile: &str) -> Vec<Planned> {
    vec![Planned::root(
        "removes the saved profile and drops the connection if it is currently up",
        &["nmcli", "connection", "delete", profile],
    )]
}

/// The ethernet profile that makes one port serve both a router and a lidar.
///
/// It cannot be done with a single profile. A Mid-360 link has no DHCP server,
/// and NetworkManager treats `ipv4.method=auto` whose lease never arrives as a
/// failure of the whole connection — with `ipv6.method=disabled` there is no
/// second family to carry it, so the static addresses configured alongside go
/// down with it. Verified on dimpi5: activation ends with "IP configuration
/// could not be reserved" and the interface is left with no address at all.
///
/// So: two profiles. This one is tried first because of its priority, waits
/// [`ETHERNET_DHCP_TIMEOUT_SECONDS`], and on failure NetworkManager moves down
/// to the static lidar profile by itself.
pub fn ethernet_dhcp_plan(interface: &str) -> Result<Vec<Planned>> {
    validate_interface_name(interface)?;
    let timeout = ETHERNET_DHCP_TIMEOUT_SECONDS.to_string();
    let priority = ETHERNET_DHCP_PRIORITY.to_string();
    Ok(vec![
        Planned::root(
            "an older profile of the same name would keep its old settings and fight this one",
            &["nmcli", "connection", "delete", ETHERNET_DHCP_PROFILE],
        )
        .optional(),
        Planned::root(
            "tried ahead of the lidar's static profile, so a cable into a router gets a real address",
            &[
                "nmcli", "connection", "add",
                "type", "ethernet",
                "ifname", interface,
                "con-name", ETHERNET_DHCP_PROFILE,
                "ipv4.method", "auto",
                // may-fail=no is what makes a dead DHCP server fail the whole
                // connection rather than leaving the port up and address-less,
                // and failing is what triggers the fallback to the lidar.
                "ipv4.may-fail", "no",
                "ipv4.dhcp-timeout", &timeout,
                "ipv6.method", "disabled",
                "connection.autoconnect", "yes",
                "connection.autoconnect-priority", &priority,
                // One attempt, then hand the port over. Retrying would hold the
                // lidar off the interface for another timeout each time.
                "connection.autoconnect-retries", "1",
            ],
        ),
    ])
}

fn validate_interface_name(interface: &str) -> Result<()> {
    if interface.is_empty() || interface.len() > 15 {
        bail!("{interface:?} is not a valid interface name");
    }
    if !interface
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-')
    {
        bail!("{interface:?} is not a valid interface name");
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Talking to the running system
// ---------------------------------------------------------------------------

fn nmcli(arguments: &[&str]) -> Result<String> {
    let output = std::process::Command::new("nmcli")
        .args(arguments)
        .output()
        .context("could not run nmcli; this subcommand needs NetworkManager")?;
    if !output.status.success() {
        bail!(
            "nmcli {} failed: {}",
            arguments.join(" "),
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    Ok(String::from_utf8_lossy(&output.stdout).into_owned())
}

/// Reads the live state into the plain data [`diagnose`] works on.
pub fn survey(rescan: bool) -> Result<Survey> {
    let netplan_files = std::fs::read_dir(NETPLAN_DIR)
        .map(|entries| {
            entries
                .flatten()
                .map(|entry| {
                    let size = entry.metadata().map(|meta| meta.len()).unwrap_or(0);
                    (entry.path(), size)
                })
                .collect()
        })
        .unwrap_or_default();

    let saved = parse_connections(&nmcli(&[
        "-t",
        "-f",
        "NAME,TYPE,DEVICE,AUTOCONNECT,AUTOCONNECT-PRIORITY",
        "connection",
        "show",
    ])?);
    let devices = parse_devices(&nmcli(&["-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])?);
    let wifi_radio_enabled = nmcli(&["-t", "radio", "wifi"])
        .map(|value| value.trim() == "enabled")
        .unwrap_or(false);
    // A scan needs the radio on, and asking for one with it off is an error
    // rather than an empty list.
    let visible = match wifi_radio_enabled {
        true => {
            let arguments: &[&str] = match rescan {
                true => &["-t", "-f", "SSID,SIGNAL", "device", "wifi", "list", "--rescan", "yes"],
                false => &["-t", "-f", "SSID,SIGNAL", "device", "wifi", "list"],
            };
            parse_scan(&nmcli(arguments).unwrap_or_default())
        }
        false => Vec::new(),
    };

    let (root_free_bytes, root_total_bytes) = filesystem_space(Path::new(NETPLAN_DIR));

    Ok(Survey {
        netplan_files,
        saved,
        visible,
        devices,
        wifi_radio_enabled,
        root_free_bytes,
        root_total_bytes,
    })
}

/// Free and total bytes, via statvfs. `df` would need parsing and its output
/// is localised; the syscall is neither.
fn filesystem_space(path: &Path) -> (u64, u64) {
    use std::os::unix::ffi::OsStrExt;
    let Ok(c_path) = std::ffi::CString::new(path.as_os_str().as_bytes()) else {
        return (0, 0);
    };
    // SAFETY: statvfs writes into a struct we own and reads a NUL-terminated
    // path we just built.
    let mut stats: libc::statvfs = unsafe { std::mem::zeroed() };
    if unsafe { libc::statvfs(c_path.as_ptr(), &mut stats) } != 0 {
        return (0, 0);
    }
    let block = stats.f_frsize as u64;
    // f_bavail, not f_bfree: the reserved blocks are not available to the user
    // writing the recording, so counting them hides the problem.
    (stats.f_bavail as u64 * block, stats.f_blocks as u64 * block)
}

fn run_plan(plan: &[Planned]) -> Result<()> {
    for step in plan {
        let argv = step.full_argv(false);
        let output = std::process::Command::new(&argv[0])
            .args(&argv[1..])
            .output()
            .with_context(|| format!("could not run {}", argv[0]))?;
        if output.status.success() {
            println!("  ok  {}", step.display());
            continue;
        }
        let stderr = String::from_utf8_lossy(&output.stderr);
        if step.optional {
            println!("  --  {} ({})", step.display(), stderr.trim());
            continue;
        }
        bail!("{} failed: {}", step.display(), stderr.trim());
    }
    Ok(())
}

/// Everything here writes to /etc and asks NetworkManager to reload, so there is
/// no useful half-privileged mode to fall back to.
fn require_root() -> Result<()> {
    if is_root() {
        return Ok(());
    }
    bail!("network setup writes to {KEYFILE_DIR}, so it needs root: re-run with sudo")
}

// ---------------------------------------------------------------------------
// The interactive part
// ---------------------------------------------------------------------------

fn prompt(question: &str) -> Result<String> {
    print!("{question}");
    std::io::stdout().flush()?;
    let mut line = String::new();
    std::io::stdin().lock().read_line(&mut line)?;
    Ok(line.trim().to_string())
}

/// Reads a line with terminal echo off, so a password is not left on the screen
/// of a rig that is usually set up in front of other people.
///
/// Falls back to an echoing read when stdin is not a terminal — a pipe has no
/// echo to switch off, and refusing there would break `network wifi-add`.
fn prompt_secret(question: &str) -> Result<Secret> {
    print!("{question}");
    std::io::stdout().flush()?;

    // SAFETY: isatty takes a file descriptor and cannot fail in a way that
    // matters here; 0 is stdin.
    let interactive = unsafe { libc::isatty(0) } == 1;
    let mut saved: libc::termios = unsafe { std::mem::zeroed() };
    if interactive {
        // SAFETY: both calls take a descriptor and a termios we own.
        unsafe {
            libc::tcgetattr(0, &mut saved);
            let mut quiet = saved;
            quiet.c_lflag &= !libc::ECHO;
            libc::tcsetattr(0, libc::TCSAFLUSH, &quiet);
        }
    }
    let mut line = String::new();
    let read = std::io::stdin().lock().read_line(&mut line);
    if interactive {
        // SAFETY: restoring the termios we saved above.
        unsafe { libc::tcsetattr(0, libc::TCSAFLUSH, &saved) };
        println!();
    }
    read?;
    Ok(Secret::new(line.trim_end_matches(['\n', '\r']).to_string()))
}

fn wifi_interface(survey: &Survey) -> String {
    survey
        .devices
        .iter()
        .find(|device| device.kind == "wifi")
        .map(|device| device.device.clone())
        .unwrap_or_else(|| "wlan0".into())
}

fn ethernet_interface(survey: &Survey) -> String {
    survey
        .devices
        .iter()
        .find(|device| device.kind == "ethernet")
        .map(|device| device.device.clone())
        .unwrap_or_else(|| "eth0".into())
}

/// One higher than the strongest saved network, so the one just typed in wins
/// where the operator is standing. Networks added later still outrank it, which
/// is what somebody adding them in order of preference expects.
fn suggested_priority(survey: &Survey) -> i32 {
    survey
        .saved
        .iter()
        .filter(|connection| connection.is_wifi())
        .map(|connection| connection.priority)
        .max()
        .unwrap_or(0)
        + 10
}

fn add_wifi_interactively(survey: &Survey) -> Result<()> {
    if survey.visible.is_empty() {
        println!("  (nothing in range, or the radio is off — you can still type an ssid)");
    }
    for (index, network) in survey.visible.iter().take(15).enumerate() {
        let known = survey
            .saved
            .iter()
            .any(|saved| saved.is_wifi() && saved.name == network.ssid);
        println!(
            "  {:>2}  {:<32} {:>3}%  {}",
            index + 1,
            network.ssid,
            network.signal,
            if known { "saved" } else { "" }
        );
    }
    let answer = prompt("\nnumber, or an ssid to type by hand (blank to cancel): ")?;
    if answer.is_empty() {
        return Ok(());
    }
    let ssid = match answer.parse::<usize>() {
        Ok(index) if index >= 1 && index <= survey.visible.len() => {
            survey.visible[index - 1].ssid.clone()
        }
        _ => answer,
    };
    validate_ssid(&ssid)?;

    let password = prompt_secret(&format!("password for {ssid:?}: "))?;
    validate_psk(&password)?;

    let default_priority = suggested_priority(survey);
    let priority = match prompt(&format!("priority [{default_priority}]: "))? {
        empty if empty.is_empty() => default_priority,
        given => given
            .parse()
            .with_context(|| format!("{given:?} is not a number"))?,
    };

    let network = WifiNetwork {
        ssid: ssid.clone(),
        password,
        priority,
    };
    let interface = wifi_interface(survey);
    let path = save_wifi(&network, &interface, Path::new(KEYFILE_DIR))?;
    println!("  wrote {} (mode 0600)", path.display());
    run_plan(&apply_plan(&ssid))?;
    Ok(())
}

fn forget_wifi_interactively(survey: &Survey) -> Result<()> {
    let saved: Vec<&SavedConnection> = survey.saved.iter().filter(|c| c.is_wifi()).collect();
    if saved.is_empty() {
        println!("  nothing saved to forget");
        return Ok(());
    }
    for (index, connection) in saved.iter().enumerate() {
        println!(
            "  {:>2}  {:<32} priority {}",
            index + 1,
            connection.name,
            connection.priority
        );
    }
    let answer = prompt("\nnumber to forget (blank to cancel): ")?;
    if answer.is_empty() {
        return Ok(());
    }
    let index: usize = answer
        .parse()
        .with_context(|| format!("{answer:?} is not one of the numbers above"))?;
    let connection = saved
        .get(index.wrapping_sub(1))
        .context("that is not one of the numbers above")?;
    run_plan(&forget_plan(&connection.name))
}

fn set_up_ethernet(survey: &Survey) -> Result<()> {
    let interface = ethernet_interface(survey);
    println!(
        "  adding {ETHERNET_DHCP_PROFILE} on {interface}: DHCP first, \
         falling back to the static lidar profile after {ETHERNET_DHCP_TIMEOUT_SECONDS}s"
    );
    run_plan(&ethernet_dhcp_plan(&interface)?)
}

/// `lite_record network`, the menu.
pub fn interactive() -> Result<()> {
    require_root()?;
    let mut rescan = true;
    loop {
        let state = survey(rescan)?;
        rescan = false;
        println!();
        print!("{}", report(&state));
        println!(
            "\n  1  add a wifi network\n  \
               2  forget a wifi network\n  \
               3  set up the ethernet port (dhcp, falling back to the lidar)\n  \
               4  re-scan\n  \
               q  quit"
        );
        match prompt("\n> ")?.as_str() {
            "1" => add_wifi_interactively(&state)?,
            "2" => forget_wifi_interactively(&state)?,
            "3" => set_up_ethernet(&state)?,
            "4" => rescan = true,
            "q" | "quit" | "" => return Ok(()),
            other => println!("  {other:?}?"),
        }
    }
}

/// `lite_record network status`, for a rig with no terminal to sit at.
pub fn status() -> Result<()> {
    let state = survey(false)?;
    print!("{}", report(&state));
    let worst = diagnose(&state)
        .into_iter()
        .map(|finding| finding.severity)
        .max()
        .unwrap_or(Severity::Note);
    // Non-zero on a broken network so this is usable as a health check from a
    // script, which is the only way to ask a headless rig how it is.
    match worst {
        Severity::Broken => std::process::exit(2),
        _ => Ok(()),
    }
}

/// `lite_record network ethernet`, without the menu.
pub fn ethernet(interface: &str) -> Result<()> {
    require_root()?;
    run_plan(&ethernet_dhcp_plan(interface)?)
}

/// `lite_record network wifi-add --ssid X`, for scripting a fleet.
///
/// The password is read from stdin rather than taken as a flag: a flag would
/// put it in argv, which is world-readable through /proc, and in the shell
/// history of whoever set the rig up.
pub fn wifi_add(ssid: &str, priority: Option<i32>) -> Result<()> {
    require_root()?;
    let state = survey(false)?;
    let password = prompt_secret("password (from stdin): ")?;
    let network = WifiNetwork {
        ssid: ssid.to_string(),
        password,
        priority: priority.unwrap_or_else(|| suggested_priority(&state)),
    };
    let interface = wifi_interface(&state);
    let path = save_wifi(&network, &interface, Path::new(KEYFILE_DIR))?;
    println!("wrote {} (mode 0600)", path.display());
    run_plan(&apply_plan(ssid))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn secret(value: &str) -> Secret {
        Secret::new(value.to_string())
    }

    #[test]
    fn terse_parsing_keeps_a_colon_inside_a_field() {
        // An SSID containing a colon is the case a plain split(':') gets wrong,
        // and it silently produces a *different* valid-looking SSID.
        assert_eq!(
            split_terse(r"Guest\:5G:802-11-wireless:wlan0:yes:30"),
            vec!["Guest:5G", "802-11-wireless", "wlan0", "yes", "30"]
        );
        assert_eq!(split_terse(r"back\\slash:x"), vec![r"back\slash", "x"]);
    }

    #[test]
    fn connections_parse_into_kind_and_priority() {
        let output = "dimensional:802-11-wireless:wlan0:yes:50\n\
                      mid360:802-3-ethernet:eth0:yes:0\n\
                      lo:loopback:lo:no:0\n";
        let parsed = parse_connections(output);
        assert_eq!(parsed.len(), 3);
        assert!(parsed[0].is_wifi());
        assert_eq!(parsed[0].priority, 50);
        assert!(parsed[1].is_ethernet());
        assert_eq!(parsed[2].device.as_deref(), Some("lo"));
    }

    #[test]
    fn a_scan_keeps_the_strongest_sighting_of_each_ssid() {
        // Real output: one row per access point per band, so a mesh network
        // appears five times and would otherwise fill the whole menu.
        let output = "dimensional:85\ndimensional:100\nAdAstraLabs:55\n:72\ndimensional:92\n";
        let parsed = parse_scan(output);
        assert_eq!(
            parsed,
            vec![
                VisibleNetwork { ssid: "dimensional".into(), signal: 100 },
                VisibleNetwork { ssid: "AdAstraLabs".into(), signal: 55 },
            ],
            "hidden (empty) ssids drop out and duplicates collapse to the strongest"
        );
    }

    #[test]
    fn a_full_filesystem_is_reported_before_anything_else() {
        // The suspected first domino on dimpi5: the card filled, and the
        // config files rewritten after that came back empty.
        let survey = Survey {
            wifi_radio_enabled: true,
            root_free_bytes: 1_073_741_824,
            root_total_bytes: 125_000_000_000,
            saved: vec![SavedConnection {
                name: "dimensional".into(),
                kind: "802-11-wireless".into(),
                device: Some("wlan0".into()),
                autoconnect: true,
                priority: 50,
            }],
            devices: vec![DeviceStatus {
                device: "wlan0".into(),
                kind: "wifi".into(),
                state: "connected".into(),
                connection: Some("dimensional".into()),
            }],
            ..Default::default()
        };
        let findings = diagnose(&survey);
        assert_eq!(findings[0].severity, Severity::Broken);
        assert!(findings[0].headline.contains("free"), "{:?}", findings[0]);

        // An unknown filesystem (statvfs failed) must not be reported as full.
        let unknown = Survey { root_total_bytes: 0, root_free_bytes: 0, ..survey.clone() };
        assert!(!diagnose(&unknown).iter().any(|f| f.headline.contains("free of")));

        let roomy = Survey { root_free_bytes: 60_000_000_000, ..survey };
        assert!(!diagnose(&roomy).iter().any(|f| f.headline.contains("free")));
    }

    #[test]
    fn an_emptied_netplan_file_is_reported_as_broken() {
        // The 2026-09-11 dimpi5 failure, which looked like dead hardware.
        let survey = Survey {
            netplan_files: vec![
                (PathBuf::from("/etc/netplan/90-NM-abc.yaml"), 0),
                (PathBuf::from("/etc/netplan/90-NM-def.yaml"), 534),
            ],
            wifi_radio_enabled: true,
            root_free_bytes: 60_000_000_000,
            root_total_bytes: 125_000_000_000,
            saved: vec![SavedConnection {
                name: "somewhere-else".into(),
                kind: "802-11-wireless".into(),
                device: None,
                autoconnect: true,
                priority: 0,
            }],
            ..Default::default()
        };
        let findings = diagnose(&survey);
        let emptied = findings
            .iter()
            .find(|finding| finding.headline.contains("0 bytes"))
            .expect("the emptied file should be reported");
        assert_eq!(emptied.severity, Severity::Broken);
        assert!(emptied.headline.contains("90-NM-abc.yaml"));
        assert!(
            !emptied.headline.contains("90-NM-def.yaml"),
            "a file with content is not a problem"
        );
    }

    #[test]
    fn an_access_point_in_range_with_no_profile_is_reported() {
        let survey = Survey {
            wifi_radio_enabled: true,
            root_free_bytes: 60_000_000_000,
            root_total_bytes: 125_000_000_000,
            saved: vec![SavedConnection {
                name: "Hakus House".into(),
                kind: "802-11-wireless".into(),
                device: None,
                autoconnect: true,
                priority: 20,
            }],
            visible: vec![VisibleNetwork { ssid: "dimensional".into(), signal: 100 }],
            devices: vec![DeviceStatus {
                device: "wlan0".into(),
                kind: "wifi".into(),
                state: "disconnected".into(),
                connection: None,
            }],
            ..Default::default()
        };
        let findings = diagnose(&survey);
        assert!(
            findings.iter().any(|finding| finding.headline.contains("dimensional")
                && finding.severity == Severity::Broken),
            "got {findings:#?}"
        );
    }

    #[test]
    fn a_connected_radio_is_not_nagged_about_other_networks() {
        let survey = Survey {
            wifi_radio_enabled: true,
            root_free_bytes: 60_000_000_000,
            root_total_bytes: 125_000_000_000,
            saved: vec![SavedConnection {
                name: "dimensional".into(),
                kind: "802-11-wireless".into(),
                device: Some("wlan0".into()),
                autoconnect: true,
                priority: 50,
            }],
            visible: vec![VisibleNetwork { ssid: "SomeCafe".into(), signal: 90 }],
            devices: vec![DeviceStatus {
                device: "wlan0".into(),
                kind: "wifi".into(),
                state: "connected".into(),
                connection: Some("dimensional".into()),
            }],
            ..Default::default()
        };
        let findings = diagnose(&survey);
        assert!(
            !findings.iter().any(|f| f.headline.contains("SomeCafe")),
            "a working rig should not be told about every cafe it can see: {findings:#?}"
        );
    }

    #[test]
    fn a_static_only_ethernet_port_is_flagged() {
        let survey = Survey {
            wifi_radio_enabled: true,
            root_free_bytes: 60_000_000_000,
            root_total_bytes: 125_000_000_000,
            saved: vec![
                SavedConnection {
                    name: "mid360".into(),
                    kind: "802-3-ethernet".into(),
                    device: Some("eth0".into()),
                    autoconnect: true,
                    priority: 0,
                },
                SavedConnection {
                    name: "dimensional".into(),
                    kind: "802-11-wireless".into(),
                    device: Some("wlan0".into()),
                    autoconnect: true,
                    priority: 50,
                },
            ],
            devices: vec![DeviceStatus {
                device: "wlan0".into(),
                kind: "wifi".into(),
                state: "connected".into(),
                connection: Some("dimensional".into()),
            }],
            ..Default::default()
        };
        let findings = diagnose(&survey);
        let flagged = findings
            .iter()
            .find(|finding| finding.headline.contains("no DHCP profile"))
            .expect("a static-only ethernet port should be flagged");
        assert_eq!(flagged.severity, Severity::Warn);
        assert!(flagged.detail.contains("mid360"));
    }

    #[test]
    fn the_dhcp_profile_outranks_the_lidar_and_gives_up_quickly() {
        let plan = ethernet_dhcp_plan("eth0").unwrap();
        let add = plan
            .iter()
            .find(|step| step.argv.contains(&"add".to_string()))
            .expect("the plan adds a profile");
        let argv = add.argv.join(" ");
        // may-fail=no is the whole mechanism: without it a dead DHCP server
        // leaves eth0 up with no address instead of handing it to the lidar.
        assert!(argv.contains("ipv4.may-fail no"), "{argv}");
        assert!(argv.contains(&format!("connection.autoconnect-priority {ETHERNET_DHCP_PRIORITY}")));
        assert!(argv.contains(&format!("ipv4.dhcp-timeout {ETHERNET_DHCP_TIMEOUT_SECONDS}")));
        // A const block, so the lidar losing the port is checked when the
        // crate compiles rather than when somebody runs the tests.
        const { assert!(ETHERNET_DHCP_PRIORITY > 0, "the lidar profile sits at 0 and must lose") };
        assert!(ethernet_dhcp_plan("eth0; rm -rf /").is_err());
        assert!(ethernet_dhcp_plan("").is_err());
    }

    #[test]
    fn a_keyfile_carries_the_ssid_password_and_priority() {
        let network = WifiNetwork {
            ssid: "dimensional".into(),
            password: secret("daneel12020"),
            priority: 50,
        };
        let file = keyfile_for(&network, "wlan0", "1111-2222").unwrap();
        assert!(file.contains("\nssid=dimensional\n"));
        assert!(file.contains("\npsk=daneel12020\n"));
        assert!(file.contains("\nautoconnect-priority=50\n"));
        assert!(file.contains("\nkey-mgmt=wpa-psk\n"));
        assert!(file.contains("\ninterface-name=wlan0\n"));
    }

    #[test]
    fn an_ssid_cannot_inject_a_second_keyfile_section() {
        // A newline in an SSID would close [wifi] and open whatever the caller
        // wanted, which is a privilege escalation on a file NetworkManager
        // reads as root.
        let network = WifiNetwork {
            ssid: "evil\n[connection]\nid=other".into(),
            password: secret("abcdefgh"),
            priority: 0,
        };
        assert!(keyfile_for(&network, "wlan0", "x").is_err());
    }

    #[test]
    fn a_password_that_network_manager_would_reject_is_refused_here_first() {
        let short = WifiNetwork {
            ssid: "x".into(),
            password: secret("short"),
            priority: 0,
        };
        assert!(keyfile_for(&short, "wlan0", "x").is_err());

        // 64 hex digits is a pre-hashed key and is allowed despite being over
        // the 63-character limit for a passphrase.
        let hashed = WifiNetwork {
            ssid: "x".into(),
            password: secret(&"a1b2c3d4".repeat(8)),
            priority: 0,
        };
        assert!(keyfile_for(&hashed, "wlan0", "x").is_ok());
    }

    #[test]
    fn a_filename_cannot_escape_the_keyfile_directory() {
        assert_eq!(keyfile_name("../../etc/passwd"), "etc_passwd.nmconnection");
        assert_eq!(keyfile_name("Cafe/Guest"), "Cafe_Guest.nmconnection");
        assert_eq!(keyfile_name("dimensional"), "dimensional.nmconnection");
        for ssid in ["../../etc/passwd", "Cafe/Guest", "a b", "..."] {
            let name = keyfile_name(ssid);
            assert!(!name.contains('/'), "{name}");
            assert_eq!(
                Path::new(KEYFILE_DIR).join(&name).parent(),
                Some(Path::new(KEYFILE_DIR)),
                "{name} escaped the directory"
            );
        }
    }

    #[test]
    fn the_real_ssid_survives_a_sanitised_filename() {
        let network = WifiNetwork {
            ssid: "Cafe/Guest".into(),
            password: secret("abcdefgh"),
            priority: 0,
        };
        let file = keyfile_for(&network, "wlan0", "x").unwrap();
        assert!(
            file.contains("\nssid=Cafe/Guest\n"),
            "the file must name the access point, not the filename it was stored under"
        );
    }

    #[test]
    fn a_uuid_has_the_shape_network_manager_expects() {
        let uuid = random_uuid().unwrap();
        let parts: Vec<&str> = uuid.split('-').collect();
        assert_eq!(parts.iter().map(|p| p.len()).collect::<Vec<_>>(), vec![8, 4, 4, 4, 12]);
        assert!(uuid.chars().all(|c| c.is_ascii_hexdigit() || c == '-'));
        assert!(parts[2].starts_with('4'), "version nibble: {uuid}");
        assert_ne!(random_uuid().unwrap(), uuid, "two calls must differ");
    }

    #[test]
    fn a_new_network_outranks_the_ones_already_saved() {
        let survey = Survey {
            saved: vec![
                SavedConnection { name: "a".into(), kind: "802-11-wireless".into(), device: None, autoconnect: true, priority: 20 },
                SavedConnection { name: "b".into(), kind: "802-11-wireless".into(), device: None, autoconnect: true, priority: 30 },
            ],
            ..Default::default()
        };
        assert!(suggested_priority(&survey) > 30);
    }

    #[test]
    fn the_report_lists_only_ports_you_could_plug_something_into() {
        let survey = Survey {
            wifi_radio_enabled: true,
            root_free_bytes: 60_000_000_000,
            root_total_bytes: 125_000_000_000,
            saved: vec![SavedConnection {
                name: "dimensional".into(),
                kind: "802-11-wireless".into(),
                device: Some("wlan0".into()),
                autoconnect: true,
                priority: 50,
            }],
            devices: vec![
                DeviceStatus { device: "wlan0".into(), kind: "wifi".into(), state: "connected".into(), connection: Some("dimensional".into()) },
                DeviceStatus { device: "eth0".into(), kind: "ethernet".into(), state: "connected".into(), connection: Some("mid360".into()) },
                // Both of these showed up in the first run against the real Pi.
                DeviceStatus { device: "tailscale0".into(), kind: "tun".into(), state: "connected (externally)".into(), connection: Some("tailscale0".into()) },
                DeviceStatus { device: "p2p-dev-wlan0".into(), kind: "wifi-p2p".into(), state: "disconnected".into(), connection: None },
            ],
            ..Default::default()
        };
        let text = report(&survey);
        assert!(text.contains("wlan0"));
        assert!(text.contains("eth0"));
        assert!(!text.contains("tailscale0"), "a tunnel is not a port: {text}");
        assert!(!text.contains("p2p-dev"), "{text}");
    }

    #[test]
    fn detail_text_wraps_without_losing_a_word() {
        let text = "the quick brown fox jumps over the lazy dog";
        let lines = wrap(text, 12);
        assert!(lines.iter().all(|line| line.len() <= 12), "{lines:?}");
        assert_eq!(lines.join(" "), text);
    }
}
