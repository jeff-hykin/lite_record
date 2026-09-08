//! The password that guards the system commands.
//!
//! Only the endpoints that run something on the machine are behind it — the
//! terminal, mounting a drive, configuring the lidar's interface, and handing the
//! server the sudo password. Recording, previewing and settings stay open,
//! because the person holding the rig should not have to type a password to press
//! record.
//!
//! The password itself is never stored. What is written to disk is a random salt
//! and a PBKDF2-HMAC-SHA256 derivation of the password, and the check compares in
//! constant time so an attacker cannot time their way to it byte by byte.

use anyhow::{bail, Context, Result};
use pbkdf2::pbkdf2_hmac;
use serde::{Deserialize, Serialize};
use sha2::Sha256;
use std::io::Read;
use std::path::{Path, PathBuf};
use subtle::ConstantTimeEq;

/// Enough to cost an attacker real time per guess, and about a tenth of a second
/// on a Pi 5 — which only ever happens on a system command, never on a preview
/// frame or a settings save.
const ROUNDS: u32 = 100_000;
const SALT_BYTES: usize = 16;
const KEY_BYTES: usize = 32;

/// The header a browser sends the password in. Not a cookie: a cookie would ride
/// along on every request including the websockets, and this is only ever wanted
/// on the four endpoints that run something.
pub const HEADER: &str = "x-lite-record-password";

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
struct Stored {
    rounds: u32,
    salt: String,
    key: String,
}

#[derive(Debug, Clone, Default)]
pub struct Access {
    path: PathBuf,
    stored: Option<Stored>,
}

impl Access {
    /// Reads whatever password was set before. A missing or unreadable file
    /// means no password, which is the state this program shipped in.
    pub fn load(path: &Path) -> Self {
        let stored = std::fs::read_to_string(path)
            .ok()
            .and_then(|text| match serde_json::from_str(&text) {
                Ok(stored) => Some(stored),
                Err(error) => {
                    eprintln!("ignoring {}: {error}", path.display());
                    None
                }
            });
        Access {
            path: path.to_path_buf(),
            stored,
        }
    }

    pub fn is_set(&self) -> bool {
        self.stored.is_some()
    }

    /// Whether `offered` may run a system command. With no password set every
    /// caller may, so an existing install does not lock its operator out the
    /// moment it is upgraded.
    pub fn allows(&self, offered: Option<&str>) -> bool {
        let Some(stored) = self.stored.as_ref() else {
            return true;
        };
        let Some(offered) = offered else {
            return false;
        };
        let (Ok(salt), Ok(key)) = (decode(&stored.salt), decode(&stored.key)) else {
            return false;
        };
        let mut derived = vec![0u8; key.len()];
        pbkdf2_hmac::<Sha256>(offered.as_bytes(), &salt, stored.rounds, &mut derived);
        derived.ct_eq(&key).into()
    }

    /// Sets or replaces the password. Replacing one requires the one in force,
    /// so someone who reaches the page while the operator is logged in cannot
    /// quietly take the rig over.
    pub fn set(&mut self, current: Option<&str>, next: &str) -> Result<()> {
        if self.is_set() && !self.allows(current) {
            bail!("that is not the current password");
        }
        if next.chars().count() < 6 {
            bail!("the password must be at least 6 characters");
        }
        let salt = random_bytes(SALT_BYTES)?;
        let mut key = vec![0u8; KEY_BYTES];
        pbkdf2_hmac::<Sha256>(next.as_bytes(), &salt, ROUNDS, &mut key);
        let stored = Stored {
            rounds: ROUNDS,
            salt: encode(&salt),
            key: encode(&key),
        };
        self.write(Some(stored))
    }

    /// Removes the password, leaving the system commands open again.
    pub fn clear(&mut self, current: Option<&str>) -> Result<()> {
        if self.is_set() && !self.allows(current) {
            bail!("that is not the current password");
        }
        let _ = std::fs::remove_file(&self.path);
        self.stored = None;
        Ok(())
    }

    fn write(&mut self, stored: Option<Stored>) -> Result<()> {
        if let Some(parent) = self.path.parent() {
            std::fs::create_dir_all(parent).ok();
        }
        match &stored {
            Some(stored) => {
                let text = serde_json::to_string_pretty(stored)?;
                std::fs::write(&self.path, text)
                    .with_context(|| format!("writing {}", self.path.display()))?;
                // The digest is salted and stretched, but a rig's operator
                // password is often reused, so it is not left world-readable.
                #[cfg(unix)]
                {
                    use std::os::unix::fs::PermissionsExt;
                    std::fs::set_permissions(&self.path, std::fs::Permissions::from_mode(0o600))
                        .with_context(|| format!("locking down {}", self.path.display()))?;
                }
            }
            None => {
                let _ = std::fs::remove_file(&self.path);
            }
        }
        self.stored = stored;
        Ok(())
    }
}

fn random_bytes(count: usize) -> Result<Vec<u8>> {
    let mut bytes = vec![0u8; count];
    std::fs::File::open("/dev/urandom")
        .context("no /dev/urandom to draw a salt from")?
        .read_exact(&mut bytes)?;
    Ok(bytes)
}

fn encode(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn decode(text: &str) -> Result<Vec<u8>> {
    if !text.len().is_multiple_of(2) {
        bail!("odd number of hex digits");
    }
    (0..text.len())
        .step_by(2)
        .map(|at| u8::from_str_radix(&text[at..at + 2], 16).context("not hex"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let path = std::env::temp_dir().join(format!("lite_record_access_{name}.json"));
        let _ = std::fs::remove_file(&path);
        path
    }

    #[test]
    fn with_no_password_set_every_caller_is_allowed() {
        let access = Access::load(&scratch("unset"));
        assert!(!access.is_set());
        assert!(access.allows(None));
        assert!(access.allows(Some("anything")));
    }

    #[test]
    fn once_a_password_is_set_only_that_password_is_allowed() {
        let path = scratch("set");
        let mut access = Access::load(&path);
        access.set(None, "field-rig").unwrap();
        assert!(access.is_set());
        assert!(access.allows(Some("field-rig")));
        assert!(!access.allows(Some("field-rig ")));
        assert!(!access.allows(Some("FIELD-RIG")));
        assert!(!access.allows(None));
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn the_password_itself_is_never_written_to_disk() {
        let path = scratch("plaintext");
        let mut access = Access::load(&path);
        access.set(None, "hunter2-hunter2").unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        assert!(!written.contains("hunter2"), "{written}");
        std::fs::remove_file(&path).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn the_digest_is_not_world_readable() {
        use std::os::unix::fs::PermissionsExt;
        let path = scratch("mode");
        Access::load(&path).set(None, "field-rig").unwrap();
        let mode = std::fs::metadata(&path).unwrap().permissions().mode();
        assert_eq!(mode & 0o077, 0, "{mode:o}");
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn a_password_survives_a_restart() {
        let path = scratch("restart");
        Access::load(&path).set(None, "field-rig").unwrap();
        let reloaded = Access::load(&path);
        assert!(reloaded.is_set());
        assert!(reloaded.allows(Some("field-rig")));
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn changing_a_password_needs_the_one_in_force() {
        let path = scratch("change");
        let mut access = Access::load(&path);
        access.set(None, "field-rig").unwrap();
        assert!(access.set(Some("guessing"), "new-password").is_err());
        assert!(access.allows(Some("field-rig")));
        access.set(Some("field-rig"), "new-password").unwrap();
        assert!(access.allows(Some("new-password")));
        assert!(!access.allows(Some("field-rig")));
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn clearing_a_password_needs_it_too_and_then_opens_up() {
        let path = scratch("clear");
        let mut access = Access::load(&path);
        access.set(None, "field-rig").unwrap();
        assert!(access.clear(Some("wrong")).is_err());
        access.clear(Some("field-rig")).unwrap();
        assert!(!access.is_set());
        assert!(access.allows(None));
        assert!(!path.exists());
    }

    #[test]
    fn a_password_too_short_to_be_worth_anything_is_refused() {
        let path = scratch("short");
        let mut access = Access::load(&path);
        assert!(access.set(None, "abc").is_err());
        assert!(!access.is_set());
    }

    #[test]
    fn two_installs_of_the_same_password_do_not_share_a_hash() {
        let (left, right) = (scratch("salt_a"), scratch("salt_b"));
        Access::load(&left).set(None, "field-rig").unwrap();
        Access::load(&right).set(None, "field-rig").unwrap();
        assert_ne!(
            std::fs::read_to_string(&left).unwrap(),
            std::fs::read_to_string(&right).unwrap()
        );
        std::fs::remove_file(&left).unwrap();
        std::fs::remove_file(&right).unwrap();
    }

    #[test]
    fn a_corrupt_password_file_locks_nobody_out_and_is_not_trusted() {
        let path = scratch("corrupt");
        std::fs::write(&path, "{ not json").unwrap();
        let access = Access::load(&path);
        assert!(!access.is_set());
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn hex_round_trips() {
        assert_eq!(encode(&[0x00, 0x0f, 0xff]), "000fff");
        assert_eq!(decode("000fff").unwrap(), vec![0x00, 0x0f, 0xff]);
        assert!(decode("abc").is_err());
        assert!(decode("zz").is_err());
    }
}
