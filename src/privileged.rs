//! Root-requiring helpers: the web terminal, USB auto-mount, and Mid-360
//! network setup.
//!
//! The sudo password lives only in this module's memory. It is fed to `sudo -S`
//! on stdin rather than passed as an argument, because argv is world-readable
//! through /proc for the lifetime of the process. It is never written to a file,
//! never printed, and never reaches the mcap.

use std::path::PathBuf;
use std::process::Stdio;

use anyhow::{bail, Context, Result};
use serde::Serialize;
use tokio::io::AsyncWriteExt;

/// A password that cannot be logged by accident. Every derive that could leak
/// it is deliberately absent, and the manual `Debug` prints a placeholder, so
/// a `{:?}` in a future error message still cannot spill it.
#[derive(Clone, Default)]
pub struct Secret(String);

impl Secret {
    pub fn new(value: String) -> Self {
        Secret(value)
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The only way out. Kept explicit and ugly so it is easy to grep for.
    fn expose(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Debug for Secret {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("Secret(<redacted>)")
    }
}

impl std::fmt::Display for Secret {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("<redacted>")
    }
}

/// A command the UI is about to run, shown to the operator before it happens.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Planned {
    pub argv: Vec<String>,
    pub needs_root: bool,
    /// One line explaining why this step exists, shown next to it in the UI.
    pub reason: String,
    /// A failure here is informational rather than fatal to the plan.
    pub optional: bool,
}

impl Planned {
    fn root(reason: &str, argv: &[&str]) -> Self {
        Planned {
            argv: argv.iter().map(|part| part.to_string()).collect(),
            needs_root: true,
            reason: reason.to_string(),
            optional: false,
        }
    }

    fn user(reason: &str, argv: &[&str]) -> Self {
        Planned {
            argv: argv.iter().map(|part| part.to_string()).collect(),
            needs_root: false,
            reason: reason.to_string(),
            optional: false,
        }
    }

    fn optional(mut self) -> Self {
        self.optional = true;
        self
    }

    /// The argv actually handed to the OS. Note the password is absent: `-S`
    /// makes sudo read it from stdin, and `-p ''` stops it printing a prompt
    /// into the output the browser sees.
    pub fn full_argv(&self, use_sudo: bool) -> Vec<String> {
        if self.needs_root && use_sudo {
            let mut argv = vec!["sudo".into(), "-S".into(), "-p".into(), String::new()];
            argv.extend(self.argv.iter().cloned());
            argv
        } else {
            self.argv.clone()
        }
    }

    pub fn display(&self) -> String {
        let prefix = if self.needs_root { "sudo " } else { "" };
        format!("{prefix}{}", self.argv.join(" "))
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct StepResult {
    pub command: String,
    pub exit_code: Option<i32>,
    pub stdout: String,
    pub stderr: String,
}

impl StepResult {
    pub fn succeeded(&self) -> bool {
        self.exit_code == Some(0)
    }
}

/// Mounts every unmounted partition on every removable block device.
///
/// `udisksctl` is preferred because it mounts as the calling user with sane
/// options and needs no root at all on a desktop-flavoured install; the plain
/// `mount` fallback is what actually works on a headless Pi image, which has no
/// udisks daemon.
pub fn usb_mount_plan(devices: &[UsbPartition]) -> Vec<Planned> {
    let mut plan = vec![Planned::user(
        "list block devices so the result can be checked afterwards",
        &["lsblk", "-o", "NAME,SIZE,FSTYPE,MOUNTPOINT,TRAN", "-J"],
    )];
    for device in devices {
        let mount_point = device.mount_point();
        plan.push(
            Planned::root(
                &format!("try udisks first for {}: it mounts as the login user and needs no root on images that ship the daemon", device.path),
                &["udisksctl", "mount", "-b", &device.path, "--no-user-interaction"],
            )
            .optional(),
        );
        plan.push(Planned::root(
            &format!("create the mount point for {}", device.path),
            &["mkdir", "-p", &mount_point],
        ));
        plan.push(Planned::root(
            &format!(
                "mount {} read-write with the web server's uid owning the files, so recordings can be written without root",
                device.path
            ),
            &["mount", "-o", "rw,uid=1000,gid=1000", &device.path, &mount_point],
        ));
    }
    plan
}

#[derive(Debug, Clone, PartialEq)]
pub struct UsbPartition {
    /// `/dev/sda1`
    pub path: String,
    pub label: Option<String>,
    pub size_bytes: u64,
}

impl UsbPartition {
    pub fn mount_point(&self) -> String {
        let leaf = self
            .label
            .as_deref()
            .filter(|label| {
                !label.is_empty()
                    && label
                        .chars()
                        .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
            })
            .map(str::to_string)
            .unwrap_or_else(|| {
                self.path
                    .rsplit('/')
                    .next()
                    .unwrap_or("usb")
                    .to_string()
            });
        format!("/media/{leaf}")
    }
}

/// Parses `lsblk -J` output into the removable partitions worth mounting.
pub fn removable_partitions(lsblk_json: &str) -> Result<Vec<UsbPartition>> {
    let parsed: serde_json::Value =
        serde_json::from_str(lsblk_json).context("lsblk did not return json")?;
    let mut found = Vec::new();
    let devices = parsed
        .get("blockdevices")
        .and_then(|value| value.as_array())
        .map(|array| array.as_slice())
        .unwrap_or(&[]);
    for device in devices {
        let is_usb = device.get("tran").and_then(|v| v.as_str()) == Some("usb");
        if !is_usb {
            continue;
        }
        let children = device
            .get("children")
            .and_then(|value| value.as_array())
            .cloned()
            .unwrap_or_default();
        // A drive with no partition table is written to whole-disk, so the disk
        // node itself is the candidate.
        let candidates = if children.is_empty() {
            vec![device.clone()]
        } else {
            children
        };
        for partition in candidates {
            let already_mounted = partition
                .get("mountpoint")
                .is_some_and(|value| !value.is_null());
            let has_filesystem = partition
                .get("fstype")
                .is_some_and(|value| !value.is_null());
            if already_mounted || !has_filesystem {
                continue;
            }
            let Some(name) = partition.get("name").and_then(|v| v.as_str()) else {
                continue;
            };
            found.push(UsbPartition {
                path: format!("/dev/{name}"),
                label: partition
                    .get("label")
                    .and_then(|v| v.as_str())
                    .map(str::to_string),
                size_bytes: partition
                    .get("size")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0),
            });
        }
    }
    Ok(found)
}

/// The Mid-360's IP is set at the factory from the last two digits of its
/// serial number, so the sticker on the unit gives it away; the factory default
/// is `192.168.1.155`. Whatever it is, it is always on `192.168.1.0/24`.
pub const LIDAR_SUBNET_PREFIX: &str = "192.168.1";
pub const LIDAR_FACTORY_DEFAULT_IP: &str = "192.168.1.155";
/// Data multicast group the lidar pushes point and IMU frames to.
pub const LIDAR_MULTICAST_GROUP: &str = "224.1.1.5";

/// Everything the button does, in order, with the history behind each step.
///
/// Every one of these steps exists because of a failure we actually hit, not
/// because a datasheet suggested it:
///
/// * The host had no address on the lidar's /24, so nothing arrived at all.
///   Fixed by `ip addr add`. DHCP is wrong here — the Mid-360 is not a DHCP
///   server and there is usually nothing else on the link.
/// * The kernel would not deliver the multicast stream because no route sent
///   `224.1.1.5` at the lidar's interface. With several NICs up, it picked the
///   wrong one and the SDK's join silently produced no traffic. Fixed by an
///   explicit host route, plus `255.255.255.255` for the SDK's discovery
///   broadcast.
/// * `multicast off` on a freshly-added interface drops the group join.
/// * A stale process still holding UDP 56100-56501 makes the next start fail
///   with "address already in use", or worse, two processes cross-feed each
///   other garbage. The check is listed so the operator sees the collision
///   rather than a mystery timeout.
pub fn mid360_network_plan(interface: &str, host_address: &str) -> Result<Vec<Planned>> {
    validate_interface(interface)?;
    let host_cidr = format!("{host_address}/24");
    if !host_address.starts_with(&format!("{LIDAR_SUBNET_PREFIX}.")) {
        bail!(
            "host address must be on {LIDAR_SUBNET_PREFIX}.0/24, the subnet the mid360 ships on; got {host_address}"
        );
    }
    validate_ipv4(host_address)?;
    let multicast_route = format!("{LIDAR_MULTICAST_GROUP}/32");

    Ok(vec![
        Planned::root(
            "bring the interface up; a link added but left down looks identical to a dead lidar",
            &["ip", "link", "set", interface, "up"],
        ),
        Planned::root(
            "the mid360 is not a dhcp server, so the host needs a static address on its /24 or no packet is ever delivered",
            &["ip", "addr", "add", &host_cidr, "dev", interface],
        )
        .optional(),
        Planned::root(
            "a freshly added interface can come up with multicast disabled, which silently swallows the group join",
            &["ip", "link", "set", interface, "multicast", "on"],
        ),
        Planned::root(
            "without an explicit route the kernel picks a different nic for 224.1.1.5 and the point stream never arrives",
            &["ip", "route", "add", &multicast_route, "dev", interface],
        )
        .optional(),
        Planned::root(
            "the sdk's discovery packet goes to the all-hosts broadcast address, which needs its own route on a multi-nic box",
            &["ip", "route", "add", "255.255.255.255/32", "dev", interface],
        )
        .optional(),
        Planned::user(
            "a leftover process still holding the 561xx ports makes the next start fail with address-already-in-use, or cross-feeds two readers garbage",
            &["ss", "-ulnp"],
        )
        .optional(),
    ])
}

fn validate_interface(interface: &str) -> Result<()> {
    if interface.is_empty() || interface.len() > 15 {
        bail!("{interface:?} is not a valid interface name");
    }
    if !interface
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '.' || c == '_' || c == '-' || c == ':')
    {
        bail!("{interface:?} is not a valid interface name");
    }
    Ok(())
}

fn validate_ipv4(address: &str) -> Result<()> {
    address
        .parse::<std::net::Ipv4Addr>()
        .with_context(|| format!("{address:?} is not an ipv4 address"))?;
    Ok(())
}

/// Picks a host address on the lidar's subnet that does not collide with the
/// lidar itself.
pub fn suggest_host_address(lidar_address: Option<&str>) -> String {
    let taken = lidar_address
        .and_then(|address| address.rsplit('.').next())
        .and_then(|octet| octet.parse::<u8>().ok());
    let preferred = if taken == Some(50) { 51 } else { 50 };
    format!("{LIDAR_SUBNET_PREFIX}.{preferred}")
}

/// Runs a planned step, feeding the password on stdin when the step needs root.
pub async fn run(planned: &Planned, password: Option<&Secret>) -> Result<StepResult> {
    let use_sudo = planned.needs_root && !is_root();
    if use_sudo && password.is_none_or(Secret::is_empty) {
        bail!(
            "{} needs root, and no sudo password has been entered",
            planned.display()
        );
    }
    let argv = planned.full_argv(use_sudo);
    let mut command = tokio::process::Command::new(&argv[0]);
    command
        .args(&argv[1..])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = command
        .spawn()
        .with_context(|| format!("could not run {}", argv[0]))?;

    let mut stdin = child.stdin.take().expect("stdin was piped");
    if use_sudo {
        let secret = password.expect("checked above");
        stdin
            .write_all(format!("{}\n", secret.expose()).as_bytes())
            .await?;
    }
    stdin.shutdown().await?;
    drop(stdin);

    let output = child.wait_with_output().await?;
    Ok(StepResult {
        // The displayed command is the plan, not `full_argv`, so a transcript
        // shown in the browser never even hints at the sudo pipeline.
        command: planned.display(),
        exit_code: output.status.code(),
        stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
    })
}

/// A free-form line from the web terminal. It goes through `sh -c` because an
/// operator typing into a terminal expects pipes and redirection to work; the
/// README documents that this makes the port a remote shell.
pub fn terminal_plan(line: &str, as_root: bool) -> Planned {
    Planned {
        argv: vec!["sh".into(), "-c".into(), line.to_string()],
        needs_root: as_root,
        reason: "web terminal".into(),
        optional: false,
    }
}

pub fn is_root() -> bool {
    // SAFETY: geteuid has no preconditions and cannot fail.
    unsafe { libc::geteuid() == 0 }
}

/// Where a mounted USB drive is likely to be, for the "save recordings here"
/// dropdown.
pub fn likely_removable_mounts() -> Vec<PathBuf> {
    let mut found = Vec::new();
    for root in ["/media", "/mnt", "/run/media"] {
        let Ok(entries) = std::fs::read_dir(root) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                found.push(path);
            }
        }
    }
    found.sort();
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_secret_cannot_be_printed_by_accident() {
        let secret = Secret::new("hunter2".into());
        assert_eq!(format!("{secret}"), "<redacted>");
        assert_eq!(format!("{secret:?}"), "Secret(<redacted>)");
        // The same guarantee has to survive being nested inside something else,
        // which is how a password usually escapes into a log.
        #[derive(Debug)]
        struct Settings {
            _password: Secret,
        }
        let printed = format!(
            "{:?}",
            Settings {
                _password: Secret::new("hunter2".into())
            }
        );
        assert!(!printed.contains("hunter2"), "{printed}");
    }

    #[test]
    fn the_password_is_never_an_argument() {
        let planned = Planned::root("test", &["mount", "/dev/sda1", "/media/usb"]);
        let argv = planned.full_argv(true);
        assert_eq!(
            argv,
            vec!["sudo", "-S", "-p", "", "mount", "/dev/sda1", "/media/usb"]
        );
        // `-S` is what makes sudo read stdin, and `-p ""` keeps the prompt out
        // of the output the browser is shown. Losing either would change how
        // the password is delivered, so both are asserted.
        assert!(argv.contains(&"-S".to_string()));
        assert_eq!(argv[3], "");
    }

    #[test]
    fn a_step_that_does_not_need_root_never_gets_a_sudo_prefix() {
        let planned = Planned::user("test", &["lsblk", "-J"]);
        assert_eq!(planned.full_argv(true), vec!["lsblk", "-J"]);
        assert_eq!(planned.display(), "lsblk -J");
    }

    #[test]
    fn already_being_root_means_no_sudo_pipeline_at_all() {
        let planned = Planned::root("test", &["ip", "link", "set", "eth0", "up"]);
        assert_eq!(
            planned.full_argv(false),
            vec!["ip", "link", "set", "eth0", "up"]
        );
    }

    #[test]
    fn the_mid360_plan_covers_address_multicast_and_routing() {
        let plan = mid360_network_plan("eth1", "192.168.1.50").unwrap();
        let commands: Vec<String> = plan.iter().map(Planned::display).collect();
        assert!(commands.contains(&"sudo ip addr add 192.168.1.50/24 dev eth1".to_string()));
        assert!(commands.contains(&"sudo ip link set eth1 multicast on".to_string()));
        assert!(commands.contains(&"sudo ip route add 224.1.1.5/32 dev eth1".to_string()));
        assert!(commands.contains(&"sudo ip route add 255.255.255.255/32 dev eth1".to_string()));
        // The port-collision check is the one that turns "mystery timeout" into
        // a diagnosis, so it must not be dropped from the plan.
        assert!(commands.iter().any(|command| command.contains("ss -ulnp")));
    }

    #[test]
    fn the_steps_that_fail_when_already_applied_are_marked_optional() {
        let plan = mid360_network_plan("eth1", "192.168.1.50").unwrap();
        for planned in &plan {
            let repeated = planned.argv.contains(&"add".to_string());
            if repeated {
                assert!(
                    planned.optional,
                    "{} fails with EEXIST on a second press",
                    planned.display()
                );
            }
        }
        // Bringing a link up is idempotent, so it is allowed to be fatal.
        assert!(!plan[0].optional);
    }

    #[test]
    fn an_address_off_the_lidar_subnet_is_refused_before_anything_runs() {
        let error = mid360_network_plan("eth1", "10.0.0.5").unwrap_err();
        assert!(format!("{error:#}").contains("192.168.1.0/24"));
        assert!(mid360_network_plan("eth1", "192.168.1.999").is_err());
    }

    #[test]
    fn an_interface_name_cannot_smuggle_in_another_command() {
        // argv is a list, so this could never inject anyway, but a name this
        // wrong is a bug worth failing on rather than passing to `ip`.
        assert!(mid360_network_plan("eth0; rm -rf /", "192.168.1.50").is_err());
        assert!(mid360_network_plan("", "192.168.1.50").is_err());
        assert!(mid360_network_plan("eth0", "192.168.1.50").is_ok());
    }

    #[test]
    fn the_suggested_host_address_never_collides_with_the_lidar() {
        assert_eq!(suggest_host_address(None), "192.168.1.50");
        assert_eq!(suggest_host_address(Some("192.168.1.155")), "192.168.1.50");
        assert_eq!(suggest_host_address(Some("192.168.1.50")), "192.168.1.51");
    }

    const LSBLK: &str = r#"{
        "blockdevices": [
            {"name":"mmcblk0","size":31914983424,"fstype":null,"mountpoint":null,"tran":null,
             "children":[{"name":"mmcblk0p1","size":268435456,"fstype":"vfat","mountpoint":"/boot"}]},
            {"name":"sda","size":1000204886016,"fstype":null,"mountpoint":null,"tran":"usb",
             "children":[{"name":"sda1","size":1000203837440,"fstype":"exfat","mountpoint":null,"label":"FIELD_DATA"}]},
            {"name":"sdb","size":8000000000,"fstype":null,"mountpoint":null,"tran":"usb",
             "children":[{"name":"sdb1","size":8000000000,"fstype":"ext4","mountpoint":"/media/already"}]}
        ]
    }"#;

    #[test]
    fn only_unmounted_usb_partitions_are_offered_for_mounting() {
        let partitions = removable_partitions(LSBLK).unwrap();
        assert_eq!(partitions.len(), 1);
        assert_eq!(partitions[0].path, "/dev/sda1");
        assert_eq!(partitions[0].label.as_deref(), Some("FIELD_DATA"));
        assert_eq!(partitions[0].size_bytes, 1_000_203_837_440);
    }

    #[test]
    fn a_whole_disk_with_no_partition_table_is_still_offered() {
        let partitions = removable_partitions(
            r#"{"blockdevices":[{"name":"sdc","size":64000000,"fstype":"ext4","mountpoint":null,"tran":"usb"}]}"#,
        )
        .unwrap();
        assert_eq!(partitions.len(), 1);
        assert_eq!(partitions[0].path, "/dev/sdc");
    }

    #[test]
    fn a_hostile_volume_label_cannot_steer_the_mount_point() {
        let partition = UsbPartition {
            path: "/dev/sda1".into(),
            label: Some("../../etc".into()),
            size_bytes: 0,
        };
        assert_eq!(partition.mount_point(), "/media/sda1");
        let spaced = UsbPartition {
            path: "/dev/sdb2".into(),
            label: Some("my drive".into()),
            size_bytes: 0,
        };
        assert_eq!(spaced.mount_point(), "/media/sdb2");
    }

    #[test]
    fn the_mount_plan_creates_the_directory_before_mounting_onto_it() {
        let plan = usb_mount_plan(&[UsbPartition {
            path: "/dev/sda1".into(),
            label: Some("FIELD_DATA".into()),
            size_bytes: 0,
        }]);
        let commands: Vec<String> = plan.iter().map(Planned::display).collect();
        let mkdir = commands
            .iter()
            .position(|c| c.contains("mkdir -p /media/FIELD_DATA"))
            .expect("mount point must be created");
        let mount = commands
            .iter()
            .position(|c| c.starts_with("sudo mount "))
            .expect("the drive must be mounted");
        assert!(mkdir < mount);
        assert!(commands[mount].contains("uid=1000"));
    }

    #[test]
    fn lsblk_that_is_not_json_is_an_error_not_an_empty_list() {
        assert!(removable_partitions("lsblk: command not found").is_err());
        assert_eq!(removable_partitions("{}").unwrap(), vec![]);
    }

    #[test]
    fn the_web_terminal_goes_through_a_shell_so_pipes_work() {
        let planned = terminal_plan("ls /dev | grep video", false);
        assert_eq!(planned.argv, vec!["sh", "-c", "ls /dev | grep video"]);
        assert!(!planned.needs_root);
        assert_eq!(
            terminal_plan("dmesg", true).full_argv(true),
            vec!["sudo", "-S", "-p", "", "sh", "-c", "dmesg"]
        );
    }

    #[tokio::test]
    async fn a_root_step_with_no_password_refuses_to_run() {
        if is_root() {
            return;
        }
        let planned = Planned::root("test", &["true"]);
        let error = run(&planned, None).await.unwrap_err();
        assert!(format!("{error:#}").contains("no sudo password"));
        let error = run(&planned, Some(&Secret::default())).await.unwrap_err();
        assert!(format!("{error:#}").contains("no sudo password"));
    }

    #[tokio::test]
    async fn an_unprivileged_step_runs_and_captures_both_streams() {
        let planned = terminal_plan("echo out; echo err 1>&2; exit 3", false);
        let result = run(&planned, None).await.unwrap();
        assert_eq!(result.exit_code, Some(3));
        assert!(!result.succeeded());
        assert_eq!(result.stdout.trim(), "out");
        assert_eq!(result.stderr.trim(), "err");
    }
}
