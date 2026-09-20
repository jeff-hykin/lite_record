//! Installs lite_record as a boot service: systemd on Linux, launchd on macOS.
//!
//! The point is a robot that comes back on its own after a power cut, so the
//! service is enabled and started immediately rather than only armed for the
//! next boot.

use anyhow::{bail, Context, Result};
use std::path::{Path, PathBuf};
use std::process::Command;

const LINUX_UNIT: &str = "/etc/systemd/system/lite_record.service";
const LINUX_AUTOMOUNT_UNIT: &str = "/etc/systemd/system/lite_record-usb-mount@.service";
const LINUX_AUTOMOUNT_RULE: &str = "/etc/udev/rules.d/99-lite_record-usb.rules";
const LINUX_AUTOMOUNT_HELPER: &str = "/usr/local/lib/lite_record/usb_mount";
const LINUX_GPIO_RULE: &str = "/etc/udev/rules.d/99-lite_record-gpio.rules";
const MACOS_LABEL: &str = "com.jeffhykin.lite_record";

pub fn install(arguments: &[String], working_directory: &Path) -> Result<()> {
    let binary = binary_path()?;
    println!("installing a boot service for {}", binary.display());
    println!("  {} {}", binary.display(), arguments.join(" "));

    if cfg!(target_os = "macos") {
        install_launchd(&binary, arguments, working_directory)
    } else if cfg!(target_os = "linux") {
        install_systemd(&binary, arguments, working_directory)
    } else {
        bail!("no boot service support for this platform")
    }
}

/// A `/nix/store` path pins one exact build forever, so a later
/// `nix profile upgrade` would leave the service running the old binary. The
/// profile symlink follows upgrades, so prefer it when it points at us.
fn binary_path() -> Result<PathBuf> {
    let executable = std::env::current_exe().context("locating the running binary")?;
    if executable.starts_with("/nix/store") {
        if let Some(home) = std::env::var_os("HOME") {
            let profile = PathBuf::from(home).join(".nix-profile/bin/lite_record");
            if profile.exists() {
                return Ok(profile);
            }
        }
    }
    Ok(executable)
}

fn current_user() -> Result<String> {
    std::env::var("SUDO_USER")
        .or_else(|_| std::env::var("USER"))
        .context("cannot tell which user the service should run as")
}

fn is_root() -> bool {
    Command::new("id")
        .arg("-u")
        .output()
        .ok()
        .and_then(|out| String::from_utf8(out.stdout).ok())
        .is_some_and(|uid| uid.trim() == "0")
}

/// Runs a step that needs root, prompting through sudo unless we already are.
fn privileged(program: &str, arguments: &[&str]) -> Result<()> {
    let mut command = if is_root() {
        Command::new(program)
    } else {
        let mut sudo = Command::new("sudo");
        sudo.arg(program);
        sudo
    };
    command.args(arguments);
    println!("  running: {program} {}", arguments.join(" "));
    let status = command
        .status()
        .with_context(|| format!("running {program}"))?;
    if !status.success() {
        bail!("{program} exited with {status}");
    }
    Ok(())
}

/// Writes a root-owned file by staging it in the user's temp directory first,
/// so the whole operation needs exactly one privileged primitive.
fn write_privileged(destination: &str, contents: &str) -> Result<()> {
    let staged = std::env::temp_dir().join("lite_record_service_staging");
    std::fs::write(&staged, contents).context("staging the service file")?;
    let staged = staged.to_string_lossy().into_owned();
    privileged("install", &["-m", "644", &staged, destination])?;
    let _ = std::fs::remove_file(&staged);
    Ok(())
}

/// systemd splits `ExecStart` on whitespace unless the argument is quoted.
fn systemd_quote(value: &str) -> String {
    format!("\"{}\"", value.replace('\\', "\\\\").replace('"', "\\\""))
}

fn install_systemd(binary: &Path, arguments: &[String], working_directory: &Path) -> Result<()> {
    let unit = systemd_unit(binary, arguments, working_directory, &current_user()?);
    write_privileged(LINUX_UNIT, &unit)?;
    install_usb_automount(&current_user()?)?;
    install_gpio_access(&current_user()?)?;
    privileged("systemctl", &["daemon-reload"])?;
    // `enable --now` leaves an already-running service on its old flags, so a
    // rerun with new options would look installed and change nothing.
    privileged("systemctl", &["enable", "lite_record"])?;
    privileged("systemctl", &["restart", "lite_record"])?;
    println!("\nlite_record now starts on boot.");
    println!("  usb drives mount themselves under /media, where the page looks.");
    println!("  status:  systemctl status lite_record");
    println!("  logs:    journalctl -u lite_record -f");
    println!("  disable: sudo systemctl disable --now lite_record");
    Ok(())
}

fn systemd_unit(binary: &Path, arguments: &[String], working_directory: &Path, user: &str) -> String {
    let mut exec_start = systemd_quote(&binary.to_string_lossy());
    for argument in arguments {
        exec_start.push(' ');
        exec_start.push_str(&systemd_quote(argument));
    }
    // `network-online` rather than `network`, because the lidar socket joins a
    // multicast group and that needs an interface that is actually up.
    //
    // The scheduling directives matter on a four-core Pi: a frame arrives every
    // 33 ms per stream and is unrecoverable if the reader is not there to take
    // it, so the recorder outranks anything else that happens to wake up.
    // `AmbientCapabilities` grants CAP_SYS_NICE so the negative nice survives
    // even when the service runs as an ordinary user. The IO priority is for the
    // writer thread, which competes with journald and apt for the SD card.
    format!(
        "[Unit]\n\
         Description=lite_record sensor recorder\n\
         After=network-online.target\n\
         Wants=network-online.target\n\
         \n\
         [Service]\n\
         Type=simple\n\
         ExecStart={exec_start}\n\
         WorkingDirectory={working}\n\
         User={user}\n\
         Restart=always\n\
         RestartSec=2\n\
         Nice=-10\n\
         AmbientCapabilities=CAP_SYS_NICE\n\
         IOSchedulingClass=best-effort\n\
         IOSchedulingPriority=0\n\
         OOMScoreAdjust=-500\n\
         \n\
         [Install]\n\
         WantedBy=multi-user.target\n",
        working = working_directory.display(),
    )
}

/// Makes a plugged-in USB drive appear under `/media` on its own.
///
/// Without this the page's storage dropdown shows nothing when a drive is
/// plugged in: `likely_removable_mounts` looks for *mounted* filesystems, and
/// raspios headless mounts none of them — there is no desktop session running
/// udisks to do it. The pieces are a udev rule that tags each new USB partition,
/// a templated unit udev pulls in, and a helper that picks the mount options the
/// filesystem will actually accept.
fn install_usb_automount(user: &str) -> Result<()> {
    write_privileged(LINUX_AUTOMOUNT_HELPER, &usb_mount_helper(user))?;
    privileged("chmod", &["755", LINUX_AUTOMOUNT_HELPER])?;
    write_privileged(LINUX_AUTOMOUNT_UNIT, AUTOMOUNT_UNIT)?;
    write_privileged(LINUX_AUTOMOUNT_RULE, AUTOMOUNT_RULE)?;
    privileged("udevadm", &["control", "--reload"])?;
    // Drives plugged in before this ran get picked up now, rather than only
    // after the next replug.
    privileged(
        "udevadm",
        &["trigger", "--subsystem-match=block", "--action=add"],
    )?;
    Ok(())
}

/// Lets the service reach a record button and the board's LED (`--button`,
/// `--led`) without running as root. The GPIO chips are already `gpio`-group
/// writable on raspios; the LEDs under `/sys/class/leds` are root-only, so a
/// rule hands them to the same group as they appear. Installed whether or not
/// a button is configured: it costs nothing, and the flags can be added later
/// without re-running this.
fn install_gpio_access(user: &str) -> Result<()> {
    privileged("groupadd", &["--force", "gpio"])?;
    // Group changes reach a systemd service when it next starts, which
    // `enable --now` does right after this.
    privileged("usermod", &["--append", "--groups", "gpio", user])?;
    write_privileged(LINUX_GPIO_RULE, GPIO_RULE)?;
    privileged("udevadm", &["control", "--reload"])?;
    // LEDs and chips that exist already get the rule applied now, not at the
    // next boot.
    privileged(
        "udevadm",
        &["trigger", "--subsystem-match=leds", "--subsystem-match=gpio", "--action=add"],
    )?;
    Ok(())
}

/// sysfs LEDs have no device node for udev to chmod, so the rule runs the
/// commands over the attribute directory instead, the same way raspios
/// grants its other hardware groups.
const GPIO_RULE: &str = "SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip*\", GROUP=\"gpio\", MODE=\"0660\"\n\
     SUBSYSTEM==\"leds\", ACTION==\"add\", \
     RUN+=\"/bin/chgrp -R gpio /sys%p\", RUN+=\"/bin/chmod -R g=u /sys%p\"\n";

/// `BindsTo` on the device is what unmounts the drive when it is yanked, so a
/// stale mount point never outlives the disk behind it.
const AUTOMOUNT_UNIT: &str = "[Unit]\n\
     Description=lite_record automount for /dev/%i\n\
     Requires=systemd-udevd.service\n\
     BindsTo=dev-%i.device\n\
     After=dev-%i.device\n\
     \n\
     [Service]\n\
     Type=oneshot\n\
     RemainAfterExit=yes\n\
     ExecStart=/usr/local/lib/lite_record/usb_mount add %i\n\
     ExecStop=/usr/local/lib/lite_record/usb_mount remove %i\n";

/// `ENV{SYSTEMD_WANTS}` rather than `RUN{program}`, because udev kills anything
/// slow it runs itself and mounting a cold spinning drive is not fast.
/// `ID_FS_USAGE` skips the whole-disk node and any partition with no filesystem.
const AUTOMOUNT_RULE: &str = "ACTION==\"add\", SUBSYSTEM==\"block\", ENV{ID_BUS}==\"usb\", \
     ENV{ID_FS_USAGE}==\"filesystem\", TAG+=\"systemd\", \
     ENV{SYSTEMD_WANTS}+=\"lite_record-usb-mount@%k.service\"\n";

/// The owning uid is baked in at install time: the recorder writes as its own
/// user, and exfat and vfat have no permission bits to inherit, so without
/// `uid=` a drive mounts root-owned and every recording fails to save.
fn usb_mount_helper(user: &str) -> String {
    format!(
        "#!/bin/bash\n\
         # Installed by `lite_record install-service`. Mounts a USB partition\n\
         # under /media, which is where the page's storage dropdown looks.\n\
         set -u\n\
         action=\"${{1:-}}\"\n\
         device=\"${{2:-}}\"\n\
         node=\"/dev/$device\"\n\
         [ -n \"$device\" ] || exit 1\n\
         \n\
         owner=$(id -u {user} 2>/dev/null || echo 0)\n\
         group=$(id -g {user} 2>/dev/null || echo 0)\n\
         \n\
         # The label is what a person recognises in the dropdown, but it is\n\
         # optional and may hold anything, so keep only characters that are\n\
         # unambiguous in a path and fall back to the kernel name.\n\
         label=$(lsblk -no LABEL \"$node\" 2>/dev/null | head -1)\n\
         label=$(printf '%s' \"$label\" | tr -c 'A-Za-z0-9._-' '_')\n\
         [ -n \"$label\" ] || label=\"$device\"\n\
         target=\"/media/$label\"\n\
         \n\
         if [ \"$action\" = remove ]; then\n\
         \x20   # The disk is already gone, so its label cannot be read back and\n\
         \x20   # the guess above may not be where it actually went. The kernel\n\
         \x20   # still lists the stale mount against the node, so ask it.\n\
         \x20   mounted=$(findmnt -n -o TARGET -S \"$node\" 2>/dev/null | head -1)\n\
         \x20   [ -n \"$mounted\" ] && target=\"$mounted\"\n\
         \x20   # A yanked drive can leave a plain umount blocking on dead IO,\n\
         \x20   # and the detach is what frees /media for the next one.\n\
         \x20   mountpoint -q \"$target\" && {{ umount \"$target\" || umount -l \"$target\"; }}\n\
         \x20   case \"$target\" in /media/*) rmdir \"$target\" 2>/dev/null ;; esac\n\
         \x20   exit 0\n\
         fi\n\
         \n\
         # Already mounted somewhere (an fstab entry, or a rerun of this) is a\n\
         # success, not a second mount point for the same disk.\n\
         findmnt -n -S \"$node\" >/dev/null 2>&1 && exit 0\n\
         \n\
         # Two unlabelled drives, or two labelled the same, must not collide.\n\
         suffix=2\n\
         while mountpoint -q \"$target\"; do\n\
         \x20   target=\"/media/$label-$suffix\"\n\
         \x20   suffix=$((suffix + 1))\n\
         done\n\
         mkdir -p \"$target\"\n\
         \n\
         # vfat, exfat and ntfs carry no ownership of their own and reject the\n\
         # permission options that the ones that do carry it require.\n\
         fstype=$(lsblk -no FSTYPE \"$node\" 2>/dev/null | head -1)\n\
         case \"$fstype\" in\n\
         \x20   vfat|exfat|ntfs|ntfs3)\n\
         \x20       options=\"uid=$owner,gid=$group,umask=022\" ;;\n\
         \x20   *)\n\
         \x20       options=\"\" ;;\n\
         esac\n\
         \n\
         if [ -n \"$options\" ]; then\n\
         \x20   mount -o \"$options\" \"$node\" \"$target\"\n\
         else\n\
         \x20   mount \"$node\" \"$target\"\n\
         fi || {{ rmdir \"$target\" 2>/dev/null; exit 1; }}\n\
         \n\
         # An ext4 stick formatted elsewhere is root-owned; the recorder has to\n\
         # be able to write to it without a password prompt nobody will see.\n\
         [ -n \"$options\" ] || chown \"$owner:$group\" \"$target\"\n\
         exit 0\n"
    )
}

fn xml_escape(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

fn install_launchd(binary: &Path, arguments: &[String], working_directory: &Path) -> Result<()> {
    let plist_path = format!("/Library/LaunchDaemons/{MACOS_LABEL}.plist");
    let plist = launchd_plist(binary, arguments, working_directory, &current_user()?);
    write_privileged(&plist_path, &plist)?;
    // Ignored on a first install; a reinstall needs the old one gone first.
    let _ = privileged("launchctl", &["bootout", &format!("system/{MACOS_LABEL}")]);
    privileged("launchctl", &["bootstrap", "system", &plist_path])?;
    println!("\nlite_record now starts on boot.");
    println!("  status:  sudo launchctl print system/{MACOS_LABEL}");
    println!("  disable: sudo launchctl bootout system/{MACOS_LABEL}");
    Ok(())
}

fn launchd_plist(
    binary: &Path,
    arguments: &[String],
    working_directory: &Path,
    user: &str,
) -> String {
    let mut program_arguments = String::new();
    for argument in std::iter::once(binary.to_string_lossy().into_owned())
        .chain(arguments.iter().cloned())
    {
        program_arguments.push_str(&format!(
            "        <string>{}</string>\n",
            xml_escape(&argument)
        ));
    }
    // A LaunchDaemon rather than a LaunchAgent, since an agent only starts once
    // somebody logs in, which is not what surviving a reboot means here.
    format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
         <!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">\n\
         <plist version=\"1.0\">\n\
         <dict>\n\
         \x20   <key>Label</key>\n\
         \x20   <string>{MACOS_LABEL}</string>\n\
         \x20   <key>ProgramArguments</key>\n\
         \x20   <array>\n\
         {program_arguments}\
         \x20   </array>\n\
         \x20   <key>UserName</key>\n\
         \x20   <string>{user}</string>\n\
         \x20   <key>WorkingDirectory</key>\n\
         \x20   <string>{working}</string>\n\
         \x20   <key>RunAtLoad</key>\n\
         \x20   <true/>\n\
         \x20   <key>KeepAlive</key>\n\
         \x20   <true/>\n\
         </dict>\n\
         </plist>\n",
        working = xml_escape(&working_directory.to_string_lossy()),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn usb_rule_pulls_in_the_unit_only_for_a_usb_partition_with_a_filesystem() {
        // Matching the whole-disk node too would try to mount /dev/sda and fail
        // on every plug, and a non-usb match would grab the boot sd card.
        assert!(AUTOMOUNT_RULE.contains("ENV{ID_BUS}==\"usb\""));
        assert!(AUTOMOUNT_RULE.contains("ENV{ID_FS_USAGE}==\"filesystem\""));
        assert!(AUTOMOUNT_RULE.contains("lite_record-usb-mount@%k.service"));
        // udev kills its own RUN children, which is why this hands off to systemd.
        assert!(!AUTOMOUNT_RULE.contains("RUN"));
    }

    #[test]
    fn usb_unit_unmounts_when_the_device_disappears() {
        assert!(AUTOMOUNT_UNIT.contains("BindsTo=dev-%i.device"));
        assert!(AUTOMOUNT_UNIT.contains("ExecStop=/usr/local/lib/lite_record/usb_mount remove %i"));
        assert!(AUTOMOUNT_UNIT.contains("RemainAfterExit=yes"));
    }

    #[test]
    fn usb_helper_gives_the_drive_to_the_service_user_and_mounts_under_media() {
        let helper = usb_mount_helper("dimos");
        assert!(helper.starts_with("#!/bin/bash\n"));
        // The uid is resolved on the pi, not baked in as a number, so an image
        // reused on a rig with a different user still writes as that user.
        assert!(helper.contains("owner=$(id -u dimos 2>/dev/null || echo 0)"));
        // `likely_removable_mounts` only scans /media, /mnt and /run/media.
        assert!(helper.contains("target=\"/media/$label\""));
        // exfat and vfat reject nothing-to-inherit ownership; ext4 rejects uid=.
        assert!(helper.contains("vfat|exfat|ntfs|ntfs3)"));
        assert!(helper.contains("uid=$owner,gid=$group,umask=022"));
    }

    #[test]
    fn usb_helper_keeps_a_label_from_escaping_its_mount_point() {
        let helper = usb_mount_helper("dimos");
        // A drive labelled `../../etc` must not become a mount point there.
        assert!(helper.contains("tr -c 'A-Za-z0-9._-' '_'"));
        // An unlabelled drive still needs a name.
        assert!(helper.contains("[ -n \"$label\" ] || label=\"$device\""));
        // Two drives sharing a label must not land on one point.
        assert!(helper.contains("while mountpoint -q \"$target\""));
    }

    #[test]
    fn usb_helper_unmounts_where_the_drive_actually_went() {
        let helper = usb_mount_helper("dimos");
        // On removal the label cannot be read back off a disk that is gone, so
        // deriving the mount point from it again would unmount nothing.
        assert!(helper.contains("findmnt -n -o TARGET -S \"$node\""));
        // Dead IO on a yanked drive blocks a plain umount forever.
        assert!(helper.contains("umount \"$target\" || umount -l \"$target\""));
        // Whatever findmnt reports, only our own mount points get removed.
        assert!(helper.contains("case \"$target\" in /media/*) rmdir"));
    }

    #[test]
    fn systemd_arguments_survive_spaces_and_quotes() {
        assert_eq!(systemd_quote("/opt/my robot"), "\"/opt/my robot\"");
        assert_eq!(systemd_quote("a\"b"), "\"a\\\"b\"");
        assert_eq!(systemd_quote("a\\b"), "\"a\\\\b\"");
    }

    #[test]
    fn plist_values_are_escaped() {
        assert_eq!(xml_escape("a&b<c>"), "a&amp;b&lt;c&gt;");
    }

    fn arguments() -> Vec<String> {
        ["--port", "8099", "--record-dir", "/data/my recordings"]
            .iter()
            .map(|value| (*value).to_owned())
            .collect()
    }

    #[test]
    fn the_gpio_rule_opens_the_chips_and_the_leds_to_the_group() {
        assert!(GPIO_RULE.contains("KERNEL==\"gpiochip*\", GROUP=\"gpio\", MODE=\"0660\""));
        assert!(GPIO_RULE.contains("SUBSYSTEM==\"leds\""));
        assert!(GPIO_RULE.contains("/bin/chgrp -R gpio /sys%p"));
        assert!(GPIO_RULE.contains("/bin/chmod -R g=u /sys%p"));
    }

    #[test]
    fn the_unit_launches_the_binary_with_every_flag() {
        let unit = systemd_unit(
            Path::new("/home/dimensional/.nix-profile/bin/lite_record"),
            &arguments(),
            Path::new("/home/dimensional"),
            "dimensional",
        );
        assert!(unit.contains(
            "ExecStart=\"/home/dimensional/.nix-profile/bin/lite_record\" \"--port\" \"8099\" \
             \"--record-dir\" \"/data/my recordings\"\n"
        ));
        assert!(unit.contains("User=dimensional\n"));
        assert!(unit.contains("WorkingDirectory=/home/dimensional\n"));
        assert!(unit.contains("Restart=always\n"));
        assert!(unit.contains("WantedBy=multi-user.target\n"));
    }

    #[test]
    fn the_unit_outranks_the_rest_of_the_system() {
        let unit = systemd_unit(
            Path::new("/usr/bin/lite_record"),
            &arguments(),
            Path::new("/home/dimensional"),
            "dimensional",
        );
        assert!(unit.contains("Nice=-10\n"));
        // Without the capability a non-root service silently keeps nice 0.
        assert!(unit.contains("AmbientCapabilities=CAP_SYS_NICE\n"));
        assert!(unit.contains("IOSchedulingPriority=0\n"));
        assert!(unit.contains("OOMScoreAdjust=-500\n"));
    }

    #[test]
    fn the_plist_lists_the_binary_first_then_each_flag() {
        let plist = launchd_plist(
            Path::new("/usr/local/bin/lite_record"),
            &arguments(),
            Path::new("/Users/jeff"),
            "jeff",
        );
        let strings: Vec<&str> = plist
            .split("<string>")
            .skip(1)
            .filter_map(|piece| piece.split("</string>").next())
            .collect();
        assert_eq!(
            strings,
            vec![
                MACOS_LABEL,
                "/usr/local/bin/lite_record",
                "--port",
                "8099",
                "--record-dir",
                "/data/my recordings",
                "jeff",
                "/Users/jeff",
            ]
        );
        assert!(plist.contains("<key>RunAtLoad</key>\n    <true/>"));
    }
}
