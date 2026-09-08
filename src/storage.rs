//! Where a recording can be moved to: the volumes this machine has mounted, a
//! directory picker inside one, and the move itself.
//!
//! A move to a USB stick crosses filesystems, so it is a copy followed by a
//! delete, and a gigabyte over USB 2 takes long enough that the browser needs to
//! be told how far it has got.

use anyhow::{bail, Context, Result};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
pub struct Volume {
    pub path: PathBuf,
    pub label: String,
    pub filesystem: String,
    /// Mounted somewhere drives get mounted, rather than being part of the
    /// installed system. This is what sorts a USB stick to the top of the list.
    pub removable: bool,
    pub read_only: bool,
    pub free_bytes: Option<u64>,
    pub total_bytes: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mount {
    pub source: String,
    pub path: PathBuf,
    pub filesystem: String,
    pub read_only: bool,
}

/// Filesystems that hold files an operator would recognise. Everything else on a
/// running Linux box — proc, sysfs, cgroup, the dozens of tmpfs — is noise in a
/// "where do you want this?" list.
const REAL_FILESYSTEMS: &[&str] = &[
    "btrfs", "exfat", "ext2", "ext3", "ext4", "f2fs", "hfsplus", "iso9660", "msdos", "ntfs",
    "ntfs3", "udf", "vfat", "xfs",
];

const REMOVABLE_ROOTS: &[&str] = &["/media", "/mnt", "/run/media", "/Volumes"];

pub fn volumes() -> Vec<Volume> {
    let text = std::fs::read_to_string("/proc/mounts").unwrap_or_default();
    let mut volumes: Vec<Volume> = parse_mounts(&text)
        .into_iter()
        .filter(|mount| device_is_present(&mount.source))
        .map(|mount| {
            let capacity = capacity(&mount.path);
            Volume {
                label: label_for(&mount),
                removable: is_removable(&mount.path),
                read_only: mount.read_only,
                free_bytes: capacity.map(|capacity| capacity.0),
                total_bytes: capacity.map(|capacity| capacity.1),
                filesystem: mount.filesystem,
                path: mount.path,
            }
        })
        .collect();
    // Drives first, then alphabetically, so the stick someone just plugged in is
    // the first thing they see.
    volumes.sort_by(|left, right| {
        right
            .removable
            .cmp(&left.removable)
            .then_with(|| left.path.cmp(&right.path))
    });
    volumes
}

/// `/proc/mounts` is one line per mount: source, mount point, type, options.
/// Paths in it are octal-escaped, which matters because a USB stick's label is
/// very often "MY DRIVE".
pub fn parse_mounts(text: &str) -> Vec<Mount> {
    let mut mounts = Vec::new();
    for line in text.lines() {
        let mut fields = line.split_whitespace();
        let (Some(source), Some(path), Some(filesystem), Some(options)) =
            (fields.next(), fields.next(), fields.next(), fields.next())
        else {
            continue;
        };
        if !REAL_FILESYSTEMS.contains(&filesystem) {
            continue;
        }
        let path = PathBuf::from(unescape(path));
        // The boot partition is a real vfat filesystem and is never somewhere to
        // put twenty gigabytes of recordings.
        if path.starts_with("/boot") {
            continue;
        }
        mounts.push(Mount {
            source: unescape(source),
            path,
            filesystem: filesystem.to_string(),
            read_only: options.split(',').any(|option| option == "ro"),
        });
    }
    mounts.sort_by(|left, right| left.path.cmp(&right.path));
    mounts.dedup_by(|left, right| left.path == right.path);
    mounts
}

fn unescape(field: &str) -> String {
    let mut out = String::with_capacity(field.len());
    let mut characters = field.chars();
    while let Some(character) = characters.next() {
        if character != '\\' {
            out.push(character);
            continue;
        }
        let digits: String = characters.clone().take(3).collect();
        match u8::from_str_radix(&digits, 8) {
            Ok(byte) if digits.len() == 3 => {
                out.push(byte as char);
                characters.nth(2);
            }
            _ => out.push('\\'),
        }
    }
    out
}

/// Whether the block device behind a mount is still plugged in.
///
/// A drive pulled without being unmounted stays in `/proc/mounts` until
/// something unmounts it, so the mount table on its own goes on offering a stick
/// that left minutes ago. The device node under `/dev` disappears the moment the
/// hardware does, which is the signal that actually tracks the cable. Anything
/// not named by a path is left alone: only a device can be judged this way.
fn device_is_present(source: &str) -> bool {
    !source.starts_with("/dev/") || Path::new(source).exists()
}

fn is_removable(path: &Path) -> bool {
    REMOVABLE_ROOTS
        .iter()
        .any(|root| path.starts_with(root) && path != Path::new(root))
}

/// What to call the volume in a list. The last path component is the filesystem
/// label for anything auto-mounted, which is the name printed on the stick.
fn label_for(mount: &Mount) -> String {
    if mount.path == Path::new("/") {
        return "Internal storage".to_string();
    }
    mount
        .path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| mount.path.to_string_lossy().into_owned())
}

fn capacity(path: &Path) -> Option<(u64, u64)> {
    let path = std::ffi::CString::new(path.as_os_str().as_encoded_bytes()).ok()?;
    let mut stats: libc::statvfs = unsafe { std::mem::zeroed() };
    // SAFETY: `path` is NUL-terminated and `stats` is a writable statvfs.
    if unsafe { libc::statvfs(path.as_ptr(), &mut stats) } != 0 {
        return None;
    }
    let block = stats.f_frsize as u64;
    Some((stats.f_bavail as u64 * block, stats.f_blocks as u64 * block))
}

#[derive(Debug, Clone, Serialize)]
pub struct Listing {
    pub path: PathBuf,
    pub volume: PathBuf,
    /// `None` at the top of the volume, which is how the UI knows to stop
    /// offering "up".
    pub parent: Option<PathBuf>,
    pub directories: Vec<PathBuf>,
    pub writable: bool,
    pub free_bytes: Option<u64>,
}

/// Lists the directories inside `path`, which must be inside a mounted volume.
///
/// Both the volume and the requested path are resolved before they are compared,
/// so a request for `/media/stick/../../etc` is refused rather than followed —
/// this is a picker on an unauthenticated port, and it must not become a way to
/// read the filesystem.
pub fn browse(path: &Path) -> Result<Listing> {
    let resolved = path
        .canonicalize()
        .with_context(|| format!("there is no directory at {}", path.display()))?;
    if !resolved.is_dir() {
        bail!("{} is not a directory", resolved.display());
    }
    let volume = volumes()
        .into_iter()
        .filter_map(|volume| volume.path.canonicalize().ok())
        .filter(|mount| resolved.starts_with(mount))
        // The deepest matching mount, or everything under "/" would be allowed.
        .max_by_key(|mount| mount.components().count())
        .with_context(|| format!("{} is not on a mounted volume", resolved.display()))?;

    let mut directories: Vec<PathBuf> = std::fs::read_dir(&resolved)
        .with_context(|| format!("could not read {}", resolved.display()))?
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .filter(|path| {
            !path
                .file_name()
                .map(|name| name.to_string_lossy().starts_with('.'))
                .unwrap_or(false)
        })
        .collect();
    directories.sort();

    Ok(Listing {
        parent: (resolved != volume)
            .then(|| resolved.parent().map(Path::to_path_buf))
            .flatten(),
        writable: is_writable(&resolved),
        free_bytes: capacity(&resolved).map(|capacity| capacity.0),
        volume,
        path: resolved,
        directories,
    })
}

/// Creates `name` inside `parent` and returns the new directory.
///
/// The picker can only choose a folder that exists, so without this a freshly
/// mounted drive has nowhere to record into. `name` is a single component, not a
/// path: the checks in [`browse`] are what keep this endpoint from being a way
/// to write anywhere on the filesystem, and they only hold if the parent is the
/// only thing that decides where the directory lands.
pub fn create_folder(parent: &Path, name: &str) -> Result<PathBuf> {
    let name = name.trim();
    if name.is_empty() {
        bail!("name the folder");
    }
    if name == "." || name == ".." || name.contains('/') || name.contains('\0') {
        bail!("{name} is not a folder name");
    }
    // Resolves the parent and proves it is on a mounted volume, which is the
    // same gate the listing goes through.
    let parent = browse(parent)?.path;
    let target = parent.join(name);
    if target.exists() {
        bail!("{} already exists", target.display());
    }
    std::fs::create_dir(&target)
        .with_context(|| format!("could not create {}", target.display()))?;
    Ok(target)
}

fn is_writable(path: &Path) -> bool {
    let Ok(path) = std::ffi::CString::new(path.as_os_str().as_encoded_bytes()) else {
        return false;
    };
    // SAFETY: `path` is NUL-terminated.
    unsafe { libc::access(path.as_ptr(), libc::W_OK) == 0 }
}

#[derive(Debug, Default, Serialize)]
pub struct Progress {
    pub copied_bytes: AtomicU64,
    /// Bytes read back off the destination and checked against the original.
    pub verified_bytes: AtomicU64,
    pub total_bytes: AtomicU64,
    /// Set once the transfer commits to copying, so the page knows the bar has
    /// two passes to cover and not one. A move within one filesystem is a
    /// rename, which copies nothing and so has nothing to check.
    pub will_verify: AtomicBool,
    pub done: AtomicBool,
}

/// Moves `source` into the directory `destination`, keeping its name.
///
/// A rename is tried first: within one filesystem a move is instant, needs no
/// free space, and moves no bytes, so there is nothing to check. Across
/// filesystems the bytes are copied to a `.part` file which is read back off the
/// drive and checked against the original before it is renamed into place — an
/// interrupted or corrupted move leaves an obviously unfinished file rather than
/// a recording that looks whole and is not. The source is deleted last, after
/// both the check and the rename.
pub fn move_recording(source: &Path, destination: &Path, progress: &Progress) -> Result<PathBuf> {
    transfer(source, destination, progress, true)
}

/// Copies `source` into the directory `destination`, leaving the original alone.
///
/// The same `.part` staging as a move, so an interrupted copy cannot leave a
/// half-written file that looks like a whole recording.
pub fn copy_recording(source: &Path, destination: &Path, progress: &Progress) -> Result<PathBuf> {
    transfer(source, destination, progress, false)
}

fn transfer(
    source: &Path,
    destination: &Path,
    progress: &Progress,
    remove_source: bool,
) -> Result<PathBuf> {
    let name = source
        .file_name()
        .context("the recording has no file name")?;
    let target = destination.join(name);
    if target == source {
        bail!("that recording is already there");
    }
    if target.exists() {
        bail!("{} already exists", target.display());
    }
    if !destination.is_dir() {
        bail!("{} is not a directory", destination.display());
    }

    let size = source.metadata()?.len();
    progress.total_bytes.store(size, Ordering::Relaxed);

    if remove_source && std::fs::rename(source, &target).is_ok() {
        progress.copied_bytes.store(size, Ordering::Relaxed);
        return Ok(target);
    }

    if let Some(free) = capacity(destination).map(|capacity| capacity.0) {
        if free < size {
            bail!(
                "not enough room: the recording is {} MB and {} MB are free",
                size / 1_000_000,
                free / 1_000_000
            );
        }
    }

    progress.will_verify.store(true, Ordering::Relaxed);
    let partial = destination.join(format!("{}.part", name.to_string_lossy()));
    // Checked while it is still `.part`, so a recording that did not survive the
    // trip never appears under its real name — and, for a move, the original is
    // never deleted on the strength of a copy nobody read back.
    let checked = copy_with_progress(source, &partial, progress)
        .and_then(|written| verify_copy(&partial, &written, progress));
    if let Err(error) = checked {
        let _ = std::fs::remove_file(&partial);
        return Err(error);
    }
    std::fs::rename(&partial, &target)
        .with_context(|| format!("could not finish writing {}", target.display()))?;
    if remove_source {
        std::fs::remove_file(source).with_context(|| {
            format!(
                "copied to {} but could not remove the original",
                target.display()
            )
        })?;
    }
    Ok(target)
}

/// Copies `source` to `target`, returning the digest of what went past.
fn copy_with_progress(source: &Path, target: &Path, progress: &Progress) -> Result<[u8; 32]> {
    let mut reader = std::fs::File::open(source)
        .with_context(|| format!("could not open {}", source.display()))?;
    let mut writer = std::fs::File::create(target)
        .with_context(|| format!("could not write to {}", target.display()))?;
    let mut digest = Sha256::new();
    let mut buffer = vec![0u8; 4 << 20];
    loop {
        let read = reader.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        writer.write_all(&buffer[..read])?;
        digest.update(&buffer[..read]);
        progress
            .copied_bytes
            .fetch_add(read as u64, Ordering::Relaxed);
    }
    // A USB stick pulled out the moment the browser says "done" would otherwise
    // still have the tail of the file in the kernel's cache.
    writer.sync_all()?;
    forget_cached_pages(&writer);
    Ok(digest.finalize().into())
}

/// Reads `path` back off the drive and checks it against what was written.
///
/// This is the whole point of the exercise, so it must not be answered out of
/// the page cache: [`forget_cached_pages`] is called on the written file first,
/// which is what makes the read below come from the device.
fn verify_copy(path: &Path, expected: &[u8; 32], progress: &Progress) -> Result<()> {
    let mut file = std::fs::File::open(path)
        .with_context(|| format!("could not read {} back", path.display()))?;
    // Linux evicts on the written descriptor above; macOS only honours this on
    // the descriptor doing the reading, so both ends ask.
    forget_cached_pages(&file);
    let mut digest = Sha256::new();
    let mut buffer = vec![0u8; 4 << 20];
    loop {
        let read = file
            .read(&mut buffer)
            .with_context(|| format!("could not read {} back", path.display()))?;
        if read == 0 {
            break;
        }
        digest.update(&buffer[..read]);
        progress
            .verified_bytes
            .fetch_add(read as u64, Ordering::Relaxed);
    }
    let found: [u8; 32] = digest.finalize().into();
    if found != *expected {
        bail!(
            "what landed on the drive does not match the original, so nothing was kept — \
             the drive may be failing, full or unplugged"
        );
    }
    Ok(())
}

/// Drops a file's pages from the cache so the next read has to touch the media.
///
/// Without this the read-back would be answered out of RAM by the very bytes
/// that were just written, and would agree with itself no matter what reached
/// the drive.
fn forget_cached_pages(file: &std::fs::File) {
    use std::os::unix::io::AsRawFd;
    let descriptor = file.as_raw_fd();
    // SAFETY: `descriptor` is open for the lifetime of `file`, and both calls
    // are advisory — a failure only costs a warm cache.
    unsafe {
        #[cfg(target_os = "linux")]
        libc::posix_fadvise(descriptor, 0, 0, libc::POSIX_FADV_DONTNEED);
        #[cfg(target_os = "macos")]
        libc::fcntl(descriptor, libc::F_NOCACHE, 1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_filesystems_holding_real_files_are_offered() {
        let mounts = parse_mounts(
            "proc /proc proc rw,relatime 0 0\n\
             /dev/mmcblk0p2 / ext4 rw,noatime 0 0\n\
             tmpfs /run tmpfs rw,nosuid 0 0\n\
             /dev/sda1 /media/dimos/FIELD\\040DRIVE exfat rw,noatime 0 0\n\
             /dev/mmcblk0p1 /boot/firmware vfat ro,relatime 0 0\n",
        );
        assert_eq!(
            mounts.iter().map(|mount| &mount.path).collect::<Vec<_>>(),
            vec![
                &PathBuf::from("/"),
                &PathBuf::from("/media/dimos/FIELD DRIVE"),
            ]
        );
    }

    #[test]
    fn a_read_only_mount_says_so() {
        let mounts = parse_mounts("/dev/sda1 /mnt/backup ext4 ro,relatime 0 0\n");
        assert!(mounts[0].read_only);
        assert!(!parse_mounts("/dev/sda1 /mnt/backup ext4 rw,ro_something 0 0\n")[0].read_only);
    }

    #[test]
    fn a_drive_label_with_a_space_survives_proc_mounts_escaping() {
        let mounts = parse_mounts("/dev/sda1 /media/dimos/FIELD\\040DRIVE vfat rw 0 0\n");
        assert_eq!(mounts[0].path, PathBuf::from("/media/dimos/FIELD DRIVE"));
        assert_eq!(label_for(&mounts[0]), "FIELD DRIVE");
    }

    #[test]
    fn the_system_disk_is_named_rather_than_shown_as_a_slash() {
        let mounts = parse_mounts("/dev/mmcblk0p2 / ext4 rw 0 0\n");
        assert_eq!(label_for(&mounts[0]), "Internal storage");
        assert!(!is_removable(&mounts[0].path));
    }

    #[test]
    fn a_mount_under_media_counts_as_removable_but_media_itself_does_not() {
        assert!(is_removable(Path::new("/media/dimos/STICK")));
        assert!(is_removable(Path::new("/run/media/dimos/STICK")));
        assert!(!is_removable(Path::new("/media")));
        assert!(!is_removable(Path::new("/home/dimos")));
    }

    #[test]
    fn browsing_a_path_that_is_on_no_mounted_volume_is_refused() {
        // The test machine's /proc/mounts is either absent (macOS) or does not
        // list a temporary directory, so this stands in for "outside the picker".
        let directory = std::env::temp_dir().join("lite_record_browse_test");
        std::fs::create_dir_all(&directory).unwrap();
        let refused = browse(&directory);
        let listed = volumes()
            .into_iter()
            .any(|volume| directory.starts_with(&volume.path));
        assert_eq!(refused.is_err(), !listed, "{refused:?}");
        std::fs::remove_dir_all(&directory).unwrap();
    }

    #[test]
    fn a_climbing_path_is_resolved_before_it_is_checked_and_never_echoed_back() {
        // The picker is on an unauthenticated port, so the mount check has to run
        // against the resolved path. If `..` were kept, the returned path would
        // be the caller's string and every later step would act on the escape.
        let root = std::env::temp_dir().join("lite_record_climb");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(root.join("here")).unwrap();
        let climbing = root.join("here").join("..").join("here");
        match browse(&climbing) {
            Ok(listing) => {
                assert_eq!(listing.path, root.join("here").canonicalize().unwrap());
                assert!(listing.path.starts_with(&listing.volume), "{listing:?}");
            }
            // A machine whose temp dir is on no listed mount refuses it outright,
            // which is the same guarantee reached sooner.
            Err(error) => assert!(error.to_string().contains("mounted volume"), "{error}"),
        }
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn browsing_something_that_is_not_there_is_an_error_not_a_panic() {
        assert!(browse(Path::new("/definitely/not/here")).is_err());
    }

    #[test]
    fn a_move_within_one_filesystem_leaves_nothing_behind() {
        let root = std::env::temp_dir().join("lite_record_move_same");
        let _ = std::fs::remove_dir_all(&root);
        let destination = root.join("out");
        std::fs::create_dir_all(&destination).unwrap();
        let source = root.join("take.mcap");
        std::fs::write(&source, b"recording").unwrap();

        let progress = Progress::default();
        let moved = move_recording(&source, &destination, &progress).unwrap();
        assert_eq!(moved, destination.join("take.mcap"));
        assert!(!source.exists());
        assert_eq!(std::fs::read(&moved).unwrap(), b"recording");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_copy_within_one_filesystem_keeps_the_original() {
        let root = std::env::temp_dir().join("lite_record_copy_same");
        let _ = std::fs::remove_dir_all(&root);
        let destination = root.join("out");
        std::fs::create_dir_all(&destination).unwrap();
        let source = root.join("take.mcap");
        std::fs::write(&source, b"recording").unwrap();

        let progress = Progress::default();
        let copied = copy_recording(&source, &destination, &progress).unwrap();
        assert_eq!(copied, destination.join("take.mcap"));
        assert_eq!(std::fs::read(&source).unwrap(), b"recording");
        assert_eq!(std::fs::read(&copied).unwrap(), b"recording");
        assert_eq!(progress.copied_bytes.load(Ordering::Relaxed), 9);
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn every_byte_that_lands_is_read_back_off_the_drive() {
        let root = std::env::temp_dir().join("lite_record_verify");
        let _ = std::fs::remove_dir_all(&root);
        let destination = root.join("out");
        std::fs::create_dir_all(&destination).unwrap();
        let source = root.join("take.mcap");
        let contents = vec![7u8; 5 << 20];
        std::fs::write(&source, &contents).unwrap();

        let progress = Progress::default();
        copy_recording(&source, &destination, &progress).unwrap();
        assert!(progress.will_verify.load(Ordering::Relaxed));
        assert_eq!(
            progress.verified_bytes.load(Ordering::Relaxed),
            contents.len() as u64
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_destination_that_does_not_match_the_original_is_refused() {
        let root = std::env::temp_dir().join("lite_record_verify_mismatch");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let landed = root.join("take.mcap.part");
        std::fs::write(&landed, b"what actually landed").unwrap();

        let expected = {
            let mut digest = Sha256::new();
            digest.update(b"what was sent");
            digest.finalize().into()
        };
        let error = verify_copy(&landed, &expected, &Progress::default()).unwrap_err();
        assert!(error.to_string().contains("does not match"), "{error}");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn bytes_that_did_not_survive_the_trip_leave_nothing_behind() {
        // The staging file is pointed at /dev/null: a destination that takes
        // the writes and does not keep them. Which step notices differs by
        // platform — macOS refuses the flush, Linux gets as far as reading back
        // an empty file — so what is asserted here is the guarantee rather than
        // the message: the original survives and no wreckage is left on the
        // drive. Exercised through a copy because a move within one filesystem
        // is a rename, which moves no bytes and so has nothing to check.
        let root = std::env::temp_dir().join("lite_record_lost_bytes");
        let _ = std::fs::remove_dir_all(&root);
        let destination = root.join("out");
        std::fs::create_dir_all(&destination).unwrap();
        let source = root.join("take.mcap");
        std::fs::write(&source, b"the only copy of a recording").unwrap();
        std::os::unix::fs::symlink("/dev/null", destination.join("take.mcap.part")).unwrap();

        copy_recording(&source, &destination, &Progress::default()).unwrap_err();
        assert_eq!(
            std::fs::read(&source).unwrap(),
            b"the only copy of a recording"
        );
        assert!(!destination.join("take.mcap").exists());
        assert!(!destination.join("take.mcap.part").exists());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_drive_that_was_pulled_stops_being_offered() {
        // A yanked stick keeps its /proc/mounts line until something unmounts
        // it, so presence is judged by the device node, which goes at once.
        assert!(device_is_present("/dev/null"));
        assert!(!device_is_present("/dev/lite-record-no-such-device"));
        // A mount named by something other than a device path cannot be judged
        // this way and must not be dropped on a guess.
        assert!(device_is_present("overlay"));
    }

    #[test]
    fn a_copy_leaves_no_part_file_when_it_fails() {
        let root = std::env::temp_dir().join("lite_record_copy_failure");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let source = root.join("take.mcap");
        std::fs::write(&source, b"recording").unwrap();

        let error =
            copy_recording(&source, &root.join("no-such-dir"), &Progress::default()).unwrap_err();
        assert!(error.to_string().contains("not a directory"), "{error}");
        assert!(source.exists());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_move_onto_an_existing_file_refuses_rather_than_overwriting() {
        let root = std::env::temp_dir().join("lite_record_move_clash");
        let _ = std::fs::remove_dir_all(&root);
        let destination = root.join("out");
        std::fs::create_dir_all(&destination).unwrap();
        let source = root.join("take.mcap");
        std::fs::write(&source, b"new").unwrap();
        std::fs::write(destination.join("take.mcap"), b"old").unwrap();

        let error = move_recording(&source, &destination, &Progress::default()).unwrap_err();
        assert!(error.to_string().contains("already exists"), "{error}");
        assert_eq!(std::fs::read(destination.join("take.mcap")).unwrap(), b"old");
        assert!(source.exists());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_copy_reports_every_byte_it_moved() {
        let root = std::env::temp_dir().join("lite_record_copy_progress");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let source = root.join("big");
        std::fs::write(&source, vec![7u8; 9 << 20]).unwrap();

        let progress = Progress::default();
        copy_with_progress(&source, &root.join("copy"), &progress).unwrap();
        assert_eq!(progress.copied_bytes.load(Ordering::Relaxed), 9 << 20);
        assert_eq!(std::fs::read(root.join("copy")).unwrap().len(), 9 << 20);
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_failed_copy_leaves_no_half_written_recording() {
        let root = std::env::temp_dir().join("lite_record_move_failure");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let source = root.join("take.mcap");
        std::fs::write(&source, b"recording").unwrap();

        let error = move_recording(&source, &root.join("no-such-dir"), &Progress::default())
            .unwrap_err();
        assert!(error.to_string().contains("not a directory"), "{error}");
        assert!(source.exists());
        std::fs::remove_dir_all(&root).unwrap();
    }
}
