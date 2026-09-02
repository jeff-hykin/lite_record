//! Installs lite_record as a boot service: systemd on Linux, launchd on macOS.
//!
//! The point is a robot that comes back on its own after a power cut, so the
//! service is enabled and started immediately rather than only armed for the
//! next boot.

use anyhow::{bail, Context, Result};
use std::path::{Path, PathBuf};
use std::process::Command;

const LINUX_UNIT: &str = "/etc/systemd/system/lite_record.service";
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
    privileged("systemctl", &["daemon-reload"])?;
    privileged("systemctl", &["enable", "--now", "lite_record"])?;
    println!("\nlite_record now starts on boot.");
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
         \n\
         [Install]\n\
         WantedBy=multi-user.target\n",
        working = working_directory.display(),
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
