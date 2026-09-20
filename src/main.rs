use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use lite_record::hub::{Hub, Settings};
use lite_record::sensors::SensorKind;
use lite_record::button::{self, Led};
use lite_record::{convert, heatmap, network, service, video, web};
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::Arc;

#[derive(Parser)]
#[command(
    name = "lite_record",
    about = "Sensor recorder for handheld rigs",
    version = concat!(env!("CARGO_PKG_VERSION"), " ", env!("LR_BUILD_ID"))
)]
struct Args {
    #[arg(long, global = true, default_value_t = 8099)]
    port: u16,

    /// Defaults to every interface so a phone on the same wifi can reach it.
    /// The UI exposes a shell, so see the README before leaving this open.
    #[arg(long, global = true, default_value = "0.0.0.0")]
    bind: IpAddr,

    /// Where mcap recordings are written and listed from.
    #[arg(long, global = true, default_value = "recordings")]
    record_dir: PathBuf,

    /// Where settings are persisted, so a reboot keeps the rig's naming,
    /// URDF and per-sensor configuration.
    #[arg(long, global = true)]
    settings_file: Option<PathBuf>,

    /// Open these sensors at startup instead of waiting for a button press.
    #[arg(long, global = true, value_delimiter = ',')]
    engage: Vec<String>,

    /// A push button between this GPIO (BCM number: 17 is header pin 11) and
    /// ground: one press starts a recording, the next stops it. Linux only.
    #[arg(long, global = true)]
    button: Option<u32>,

    /// A light that is on while recording: a kernel LED name such as ACT (the
    /// Pi's green one) or gpio:<pin> for an LED wired to a header pin.
    #[arg(long, global = true, requires = "button")]
    led: Option<Led>,

    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand)]
enum Command {
    /// Install lite_record as a boot service (systemd or launchd) with these
    /// same options, and start it now. Asks for sudo.
    #[command(name = "survive_reboot", alias = "survive-reboot")]
    SurviveReboot,

    /// Rewrite a recording in place, moving every image stream into a format
    /// Foxglove and rerun can both decode. The same work the UI's Post process
    /// button does, for a rig with no browser pointed at it.
    #[command(name = "post_process", alias = "post-process")]
    PostProcess(PostProcessArgs),

    /// Complete a recording's frame tree: correct the camera extrinsics an
    /// older recorder wrote backwards, add the URDF's joints, and append the
    /// result to /tf at 5 Hz. Prints the tree and exits non-zero if it is
    /// still disconnected.
    #[command(name = "tf_fixup", alias = "tf-fixup")]
    TfFixup {
        recording: PathBuf,

        #[arg(long)]
        urdf: Option<PathBuf>,
    },

    /// Top-down density render of a point cloud topic with the odometry path
    /// drawn over it, blue at the start and red at the end.
    Heatmap(heatmap::Options),

    /// Encode an image topic as an mp4 by piping its frames through ffmpeg.
    #[command(name = "to_video", alias = "to-video")]
    ToVideo(video::Options),

    /// Set up the wifi and ethernet a headless rig needs to come back on its
    /// own, and report what is wrong with what it has. Interactive with no
    /// argument. Needs root, because it writes NetworkManager's profiles.
    Network {
        #[command(subcommand)]
        what: Option<NetworkCommand>,
    },
}

#[derive(Subcommand)]
enum NetworkCommand {
    /// Print the interfaces, the free space, and every problem found, then
    /// exit non-zero if anything is broken. Safe to run as a health check.
    Status,

    /// Save one wifi network without a menu, for setting up a fleet.
    ///
    /// The password is read from stdin, never taken as a flag: an argument
    /// would put it in argv, which any local user can read out of /proc, and
    /// in the shell history of whoever ran it.
    #[command(name = "wifi_add", alias = "wifi-add")]
    WifiAdd {
        #[arg(long)]
        ssid: String,

        /// Higher wins where more than one saved network is in range.
        /// Defaults to just above the highest already saved.
        #[arg(long)]
        priority: Option<i32>,
    },

    /// Add the ethernet profile that tries DHCP first and falls back to the
    /// lidar's static address when no DHCP server answers.
    Ethernet {
        #[arg(long, default_value = "eth0")]
        interface: String,
    },
}

#[derive(clap::Args)]
struct PostProcessArgs {
    recording: PathBuf,

    /// Punch each source chunk out once its replacement has been verified,
    /// so the card only has to hold the output rather than both files. This
    /// destroys the original as it goes: an interrupted run leaves the
    /// messages split across two files, and putting them back is manual.
    #[arg(long)]
    reclaim: bool,

    /// Skip the Point-LIO pass that appends /pointlio_odometry and the
    /// odom -> rig edges on /tf.
    #[arg(long)]
    no_odom: bool,

    /// A URDF whose joints join the sensors' own frame trees into one, the
    /// same as running tf_fixup first.
    #[arg(long)]
    urdf: Option<PathBuf>,

    /// The PointCloud2 topic to estimate odometry from. Found automatically
    /// when there is only one lidar.
    #[arg(long)]
    lidar_topic: Option<String>,

    /// The Imu topic that goes with --lidar-topic.
    #[arg(long)]
    imu_topic: Option<String>,

    /// Do everything except write: convert nothing, append nothing, but
    /// print the tree, its problems, and the odometry the file would get.
    #[arg(long)]
    dry_run: bool,

    /// Also write the estimated trajectory here in TUM format
    /// (`time x y z qx qy qz qw`, sensor clock), for comparing runs.
    #[arg(long)]
    trajectory: Option<PathBuf>,

    /// Shift any stream whose header stamps sit on a different clock from the
    /// rest of the file onto the file's clock, keeping the device's own
    /// spacing. A split clock is reported either way; this repairs it.
    #[arg(long)]
    fix_clocks: bool,

    /// Metres per second above which the odometry rolls a scan back as a
    /// mismatch. The default suits a rig somebody carries; a bike needs more.
    #[arg(long, default_value_t = lite_record::odometry::HANDHELD_MAX_VELOCITY)]
    max_speed: f64,

    /// Rewrite a `/tf_static` an older recorder wrote in the SDK's direction.
    /// The appended `/tf` already supersedes it for anything that keeps a
    /// history per frame, so this is only worth the rewrite for a consumer
    /// that reads `/tf_static` on its own.
    #[arg(long)]
    fix_static_tf: bool,

    /// Skip the motion-compensated copy of the lidar the odometry pass
    /// otherwise appends as /pointlio_lidar. It is roughly the size of the
    /// lidar stream again.
    #[arg(long)]
    no_deskew: bool,

    /// Run the estimator over a recording that already has odometry, and append
    /// only the corrected /pointlio_lidar clouds from it.
    ///
    /// This exists because a recording post-processed before /pointlio_lidar
    /// was a thing has no way to gain one: the corrected clouds need the states
    /// the estimator holds inside each scan, and the estimator is skipped once
    /// /pointlio_odometry is there. It appends *only* the clouds, because
    /// appending cannot remove anything — a second odometry pass would leave
    /// two full sets on the topic rather than replacing the first. The run is
    /// deterministic given the same input and --max-speed, so the clouds agree
    /// with the odometry already in the file.
    #[arg(long)]
    deskew_only: bool,

    /// Skip the raycast voxel map that otherwise appends /global_map and writes
    /// a .pc2.lcm beside the recording.
    #[arg(long)]
    no_raytrace: bool,

    /// Append a transform even when the recording already places that frame.
    ///
    /// Off by default, because appending cannot remove: writing a second value
    /// for an edge the file already publishes leaves it with two answers and
    /// nothing saying which is meant. A consumer then has to guess, and a tf
    /// tree that interpolates slerps between them — which is what happened to
    /// `sensor_mount_link -> livox_link` in the grocery recording. Cut the old
    /// value out first instead, then run this.
    #[arg(long)]
    allow_tf_conflict: bool,
}

impl Args {
    /// The flags the installed service should be launched with. Every path is
    /// made absolute, since a service does not inherit this shell's directory.
    fn service_arguments(&self, record_dir: &Path, settings_file: &Path) -> Vec<String> {
        let mut arguments = vec![
            "--port".into(),
            self.port.to_string(),
            "--bind".into(),
            self.bind.to_string(),
            "--record-dir".into(),
            record_dir.to_string_lossy().into_owned(),
            "--settings-file".into(),
            settings_file.to_string_lossy().into_owned(),
        ];
        if !self.engage.is_empty() {
            arguments.push("--engage".into());
            arguments.push(self.engage.join(","));
        }
        if let Some(pin) = self.button {
            arguments.push("--button".into());
            arguments.push(pin.to_string());
        }
        if let Some(led) = &self.led {
            arguments.push("--led".into());
            arguments.push(led.to_string());
        }
        arguments
    }
}

fn default_settings_file() -> PathBuf {
    match std::env::var_os("HOME") {
        Some(home) => PathBuf::from(home).join(".dimos/lite_record.json"),
        None => PathBuf::from("lite_record.json"),
    }
}

/// `canonicalize` only works on paths that already exist, and the recordings
/// directory is created lazily, so fall back to joining the current directory.
fn absolute(path: &Path) -> PathBuf {
    std::fs::canonicalize(path).unwrap_or_else(|_| match std::env::current_dir() {
        Ok(directory) => directory.join(path),
        Err(_) => path.to_path_buf(),
    })
}

/// Runs `work` while printing `describe()` once a minute, so a stage that runs
/// for an hour on a large recording never looks hung.
/// How often a run that is not on a terminal says it is still alive. A log wants
/// a readable trail, not a line a second, but a minute of silence reads as hung.
const LOG_TICK_SECONDS: u64 = 15;

/// Runs `work`, saying what stage is in progress from the moment it starts and
/// then how far in it is, so a long recording never looks like a hung process.
///
/// On a terminal that is one line redrawn every second. Piped to a file or to
/// journald a carriage return would produce one enormous line, so there it is a
/// fresh line every `LOG_TICK_SECONDS`.
fn with_ticker<T>(
    label: &str,
    describe: impl Fn() -> String + Send + 'static,
    work: impl FnOnce() -> T,
) -> T {
    use std::io::{IsTerminal, Write};

    let interactive = std::io::stdout().is_terminal();
    // Printed before the work starts rather than at the first tick: the point is
    // that something appears the instant the stage begins.
    println!("{label}...");
    let _ = std::io::stdout().flush();

    let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let stopping = Arc::clone(&running);
    let ticker = std::thread::spawn(move || {
        let mut seconds = 0;
        while stopping.load(std::sync::atomic::Ordering::Relaxed) {
            std::thread::sleep(std::time::Duration::from_secs(1));
            seconds += 1;
            if interactive {
                // \r and no newline, so the count updates in place. The trailing
                // spaces wipe whatever a longer previous line left behind.
                print!("\r  {seconds:>6}s  {}     ", describe());
                let _ = std::io::stdout().flush();
            } else if seconds % LOG_TICK_SECONDS == 0 {
                println!("  {seconds:>6}s  {}", describe());
            }
        }
    });
    let result = work();
    running.store(false, std::sync::atomic::Ordering::Relaxed);
    let _ = ticker.join();
    if interactive {
        // The redrawn line is progress, not a result; leave the scrollback to
        // the summary the caller prints next.
        print!("\r\x1b[2K");
        let _ = std::io::stdout().flush();
    }
    result
}

#[cfg(test)]
mod ticker_tests {
    use super::*;

    #[test]
    fn the_label_appears_before_the_work_runs() {
        // The complaint this answers was silence at the start, so the ordering
        // is the point: the stage announces itself, then the work begins.
        let order = Arc::new(std::sync::Mutex::new(Vec::new()));
        let seen = Arc::clone(&order);
        with_ticker("stage", String::new, || seen.lock().unwrap().push("work"));
        assert_eq!(*order.lock().unwrap(), vec!["work"]);
    }

    #[test]
    fn a_log_tick_is_frequent_enough_to_not_look_hung() {
        // A minute of nothing is what made a working run look dead.
        let seconds = LOG_TICK_SECONDS;
        assert!(seconds <= 30, "{seconds}s between log lines is too quiet");
    }

    #[test]
    fn the_work_result_is_handed_back_untouched() {
        assert_eq!(with_ticker("stage", String::new, || 7), 7);
    }
}

fn load_urdf(path: Option<&Path>) -> Result<Option<lite_record::urdf::Urdf>> {
    let Some(path) = path else {
        return Ok(None);
    };
    let xml = std::fs::read_to_string(path).with_context(|| format!("could not read {}", path.display()))?;
    Ok(Some(lite_record::urdf::parse(&xml).with_context(|| format!("{} is not a urdf", path.display()))?))
}

/// The three stages that turn a fresh recording into one that is viewable,
/// placed and localised: recode the images (a rewrite, skipped when there is none),
/// then complete the frame tree and estimate odometry (both appended).
fn post_process(args: &PostProcessArgs) -> Result<()> {
    let PostProcessArgs {
        recording, reclaim, no_odom, urdf, lidar_topic, imu_topic, dry_run, trajectory, fix_clocks,
        no_raytrace,
        fix_static_tf, max_speed, no_deskew, deskew_only, allow_tf_conflict,
    } = args;
    let (recording, reclaim, no_odom, dry_run) = (recording.as_path(), *reclaim, *no_odom, *dry_run);
    let deskew_only = *deskew_only;
    // Already there is a reason to skip, not to duplicate: appending cannot
    // remove the first set.
    // These two survey the whole file before anything is written, which on a
    // multi-gigabyte recording is a long time to show nothing at all -- the
    // stage tickers below do not start until this is finished.
    println!("reading {} ...", recording.display());
    let already_deskewed = lite_record::deskew::already_present(recording)?;
    let wants_deskew = !no_deskew && !dry_run && already_deskewed == 0;
    if already_deskewed > 0 && !no_deskew {
        println!(
            "{} already carries {already_deskewed} messages on {}; not correcting again",
            recording.display(),
            lite_record::deskew::DESKEWED_TOPIC
        );
    }
    if deskew_only && !wants_deskew {
        anyhow::bail!(
            "--deskew-only has nothing to do: {}",
            match (no_deskew, dry_run, already_deskewed) {
                (true, _, _) => "--no-deskew was also given".to_string(),
                (_, true, _) => "--dry-run writes nothing".to_string(),
                (_, _, count) => format!("the recording already has {count} corrected clouds"),
            }
        );
    }
    // Surveyed before anything is written, so the report describes the file as
    // it was handed over rather than as this run leaves it.
    let clocks = lite_record::restamp::survey_path(recording)?;
    let size = std::fs::metadata(recording).map(|at| at.len()).unwrap_or(0);
    println!("  {:.2} GB, {} streams", size as f64 / 1e9, clocks.len());
    print!("{}", lite_record::restamp::describe(&clocks));
    let shifts: std::collections::BTreeMap<u16, i64> = match *fix_clocks && !dry_run {
        true => clocks
            .iter()
            .filter(|(_, clock)| clock.needs_shift())
            .map(|(id, clock)| (*id, clock.offset_nanos))
            .collect(),
        false => Default::default(),
    };
    let (lidar_topic, imu_topic, trajectory) = (lidar_topic.as_deref(), imu_topic.as_deref(), trajectory.as_deref());
    let started = std::time::Instant::now();
    let urdf = load_urdf(urdf.as_deref())?;

    let reclaim = match reclaim {
        true => convert::Reclaim::AsItGoes,
        false => convert::Reclaim::No,
    };
    let progress = Arc::new(convert::Progress::default());
    let watched = Arc::clone(&progress);
    let converted = with_ticker(
        "recoding images and refitting camera infos",
        move || {
            format!(
                "{} messages  {:.2} GB written",
                watched.messages.load(std::sync::atomic::Ordering::Relaxed),
                watched.bytes.load(std::sync::atomic::Ordering::Relaxed) as f64 / 1e9,
            )
        },
        || match (dry_run, convert::needs_conversion(recording).map(|needed| needed || *fix_static_tf)) {
            (true, Ok(true)) => {
                println!("{}: would recode images / refit camera infos (dry run)", recording.display());
                Err(convert::NothingToConvert.into())
            }
            (true, Ok(false)) => Err(convert::NothingToConvert.into()),
            (true, Err(error)) => Err(error),
            (false, Ok(false)) if shifts.is_empty() => Err(convert::NothingToConvert.into()),
            (false, _) => convert::in_place(recording, &progress, reclaim, &shifts),
        },
    );
    match converted {
        Ok(report) => println!(
            "{}: {} decoded, {} refitted, {} restamped, {} transforms inverted, {} copied, {} failed, {:.2} GB, {:.2} GB reclaimed, {}s",
            recording.display(),
            report.decoded,
            report.refitted,
            report.restamped,
            report.inverted_transforms,
            report.copied,
            report.failed,
            report.bytes as f64 / 1e9,
            report.reclaimed as f64 / 1e9,
            started.elapsed().as_secs(),
        ),
        Err(error) if error.downcast_ref::<convert::NothingToConvert>().is_some() => {
            println!("{}: already viewable, nothing to convert", recording.display());
        }
        Err(error) => return Err(error),
    }

    // The frame tree first, since the odometry describes its root.
    let file = std::fs::File::open(recording)?;
    let mapped = unsafe { memmap2::Mmap::map(&file)? };
    let inspected = lite_record::fixup::inspect(&mapped)?;
    let plan = lite_record::fixup::plan(&inspected, urdf.as_ref());
    if !no_odom && urdf.is_none() && plan.tree.roots().len() > 1 {
        println!(
            "note: the frame tree has {} roots and no --urdf was given, so the odometry will describe \
             the lidar's own link; pass --urdf now if the rig has one, since odometry cannot be re-rooted later",
            plan.tree.roots().len()
        );
    }

    // Filled by the estimator's walk when a corrected lidar was asked for, and
    // emptied into the file after the appender is open.
    let mut spool = None;
    let estimate = if no_odom {
        None
    } else if let (Some(existing), false) = (lite_record::odometry::already_present(recording)?, deskew_only) {
        println!(
            "{} already carries {} messages on {}; not estimating again",
            recording.display(),
            existing.messages,
            lite_record::odometry::ODOMETRY_TOPIC
        );
        // Odometry describes the tree's root, computed with the lidar-to-root
        // transform of the day it was written. A urdf that has since moved the
        // lidar makes those poses describe a rig that does not exist, and
        // nothing about the file looks wrong.
        if let Some((metres, radians)) = existing.geometry.as_deref().and_then(|marker| {
            let lidar = inspected
                .frame_of_topic
                .values()
                .find(|frame| plan.tree.contains(frame) && frame.contains("livox"))
                .or_else(|| inspected.frame_of_topic.values().next())?;
            lite_record::odometry::geometry_drift(marker, &plan.tree, lidar)
        }) {
            if metres > 1e-3 || radians > 1e-3 {
                println!(
                    "warning: that odometry was estimated with the lidar {metres:.3} m and {:.2} deg \
                     from where this urdf puts it, so it describes a different rig — re-estimate on \
                     a copy that has no odometry yet",
                    radians.to_degrees()
                );
            }
        }
        None
    } else {
        let summary = mcap::Summary::read(&mapped)?.context("no summary")?;
        let found = lite_record::odometry::find_lidar_and_imu(&summary);
        // Either topic given on the command line replaces the found one.
        let topics = found
            .map(|(lidar, imu)| {
                (
                    lidar_topic.map_or(lidar, str::to_string),
                    imu_topic.map_or(imu, str::to_string),
                )
            })
            .or_else(|| Some((lidar_topic?.to_string(), imu_topic?.to_string())));
        match topics {
            None => {
                println!("no lidar + imu pair in the recording, so no odometry to estimate");
                None
            }
            Some((lidar, imu)) => {
                if wants_deskew {
                    spool = Some(lite_record::deskew::Spool::beside(recording)?);
                }
                let scans = Arc::new(std::sync::atomic::AtomicU64::new(0));
                let watched = Arc::clone(&scans);
                let estimate = with_ticker(
                    &format!("estimating odometry from {lidar} + {imu}"),
                    move || format!("{} scans", watched.load(std::sync::atomic::Ordering::Relaxed)),
                    || lite_record::odometry::estimate(&mapped, &lidar, &imu, &scans, *max_speed, spool.as_mut()),
                )?;
                println!(
                    "  {} poses, {:.1} m of path, {} scans rejected by the {} m/s cap, first scan reached the recorder {:.3} s after it began",
                    estimate.poses.len(),
                    estimate.path_length_metres,
                    estimate.rejected_scans,
                    max_speed,
                    estimate.delivery_latency_seconds
                );
                if let Some(spool) = spool.as_ref() {
                    println!(
                        "  {} motion-compensated scans -> {} ({:.2} GB){}",
                        spool.clouds(),
                        lite_record::deskew::DESKEWED_TOPIC,
                        spool.bytes() as f64 / 1e9,
                        match spool.passed_through() {
                            0 => String::new(),
                            skipped => format!(", {skipped} scan(s) the estimator could not place left out"),
                        }
                    );
                }
                if let Some(path) = trajectory {
                    lite_record::odometry::write_tum(&estimate, path)?;
                    println!("  trajectory -> {}", path.display());
                }
                Some(estimate)
            }
        }
    };
    drop(mapped);

    if dry_run || (plan.new_edges.is_empty() && estimate.is_none()) {
        if let Some(spool) = spool.take() {
            spool.discard();
        }
        print!("{}", lite_record::fixup::describe(&plan, 0));
        if dry_run {
            println!(
                "dry run: would append {} static edge(s) and {} odometry poses; {}s",
                plan.new_edges.len(),
                estimate.as_ref().map_or(0, |estimate| estimate.poses.len()),
                started.elapsed().as_secs()
            );
        } else {
            println!("nothing to append; {}s", started.elapsed().as_secs());
        }
        // Having nothing to append says nothing about the map: a recording that
        // already carries its odometry and clouds reaches here every time, and
        // it is exactly the one a second run is meant to add a map to.
        if !dry_run && !*no_raytrace {
            raytrace_stage(recording)?;
        }
        return Ok(());
    }
    if !plan.conflicting.is_empty() && !*allow_tf_conflict {
        print!("{}", lite_record::fixup::describe(&plan, 0));
        anyhow::bail!(
            "refusing to write {} conflicting transform(s).\nCut the old value out first (`mcap_edit --drop-tf-edge <parent>:<child>`), then run this\nagain — or pass --allow-tf-conflict to write it anyway and leave the file ambiguous.",
            plan.conflicting.len()
        );
    }
    let mut appender = lite_record::mcap_append::Appender::open(recording)?;
    let static_messages = lite_record::fixup::append_static_transforms(
        &mut appender,
        &plan.new_edges,
        inspected.start_nanos,
        inspected.end_nanos,
    )?;
    let appended = match (&estimate, deskew_only) {
        (Some(estimate), false) => Some(lite_record::odometry::append(&mut appender, estimate, &plan.tree)?),
        _ => None,
    };
    let deskewed = match (spool, &estimate) {
        (Some(spool), Some(estimate)) => spool.drain_into(&mut appender, &estimate.lidar_frame)?,
        (Some(spool), None) => {
            spool.discard();
            0
        }
        (None, _) => 0,
    };
    let total = appender.finish()?;
    print!("{}", lite_record::fixup::describe(&plan, static_messages));
    if let Some(appended) = appended {
        println!(
            "appended {} {} messages (odom -> {}) and as many /tf edges",
            appended.odometry_messages,
            lite_record::odometry::ODOMETRY_TOPIC,
            appended.child_frame
        );
    }
    if deskewed > 0 {
        println!("appended {deskewed} {} clouds", lite_record::deskew::DESKEWED_TOPIC);
    }
    println!("{total} messages appended to {}; {}s", recording.display(), started.elapsed().as_secs());
    if !*no_raytrace {
        raytrace_stage(recording)?;
    }
    Ok(())
}

/// Builds the raycast voxel map and writes it back, once the clouds and poses
/// it reads are actually in the file.
///
/// This runs as its own pass rather than riding along with the estimator,
/// because it reads the motion-compensated clouds the estimator only finishes
/// writing at the append above.
fn raytrace_stage(recording: &Path) -> Result<()> {
    let already = lite_record::raytrace::already_present(recording)?;
    if already > 0 {
        println!(
            "{} already carries {already} {} message(s); not mapping again",
            recording.display(),
            lite_record::raytrace::GLOBAL_MAP_TOPIC,
        );
        return Ok(());
    }
    let opened = lite_record::topics::Recording::open(recording)?;
    // A recording with no odometry has nothing to place scans by. That is a
    // reason to say so and move on, not to fail a run whose other stages worked.
    if opened.channel(lite_record::odometry::ODOMETRY_TOPIC).is_err() {
        println!(
            "no {} in the recording, so there is nothing to build a map from",
            lite_record::odometry::ODOMETRY_TOPIC,
        );
        return Ok(());
    }
    let started = std::time::Instant::now();
    let scans = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let watched = Arc::clone(&scans);
    let map = with_ticker(
        "building the voxel map",
        move || format!("{} scans", watched.load(std::sync::atomic::Ordering::Relaxed)),
        || {
            lite_record::raytrace::build(
                &opened,
                lite_record::deskew::DESKEWED_TOPIC,
                lite_record::record::TF_TOPIC,
                lite_record::odometry::ODOM_FRAME,
                &scans,
            )
        },
    )?;
    let beside = lite_record::raytrace::write(recording, &map)?;
    println!(
        "  {} voxels at {} m from {} scans{} -> {} and {}",
        map.points(),
        map.voxel_size,
        map.scans,
        match map.unplaced {
            0 => String::new(),
            n => format!(", {n} scan(s) with no pose left out"),
        },
        lite_record::raytrace::GLOBAL_MAP_TOPIC,
        beside.display(),
    );
    println!("  voxel map in {}s", started.elapsed().as_secs());
    Ok(())
}

fn tf_fixup(recording: &Path, urdf: Option<&Path>) -> Result<()> {
    let urdf = load_urdf(urdf)?;
    let (plan, appended) = lite_record::fixup::fix(recording, urdf.as_ref())?;
    print!("{}", lite_record::fixup::describe(&plan, appended));
    if !plan.problems.is_empty() {
        std::process::exit(1);
    }
    Ok(())
}

fn sensor_named(name: &str) -> Result<SensorKind> {
    match name.trim().to_ascii_lowercase().as_str() {
        "realsense" => Ok(SensorKind::Realsense),
        "orbbec" => Ok(SensorKind::Orbbec),
        "oakd" | "oak-d" | "oak" => Ok(SensorKind::OakD),
        "livox" | "mid360" => Ok(SensorKind::Livox),
        other => anyhow::bail!(
            "{other:?} is not a sensor; expected realsense, orbbec, oakd or livox"
        ),
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let record_dir = absolute(&args.record_dir);
    let settings_file = absolute(
        &args
            .settings_file
            .clone()
            .unwrap_or_else(default_settings_file),
    );

    match args.command {
        Some(Command::SurviveReboot) => {
            let arguments = args.service_arguments(&record_dir, &settings_file);
            let working_directory = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/"));
            return service::install(&arguments, &working_directory);
        }
        Some(Command::PostProcess(args)) => {
            return post_process(&args);
        }
        Some(Command::TfFixup { recording, urdf }) => {
            return tf_fixup(&recording, urdf.as_deref());
        }
        Some(Command::Heatmap(options)) => return heatmap::run(&options),
        Some(Command::ToVideo(options)) => return video::run(&options),
        Some(Command::Network { what }) => {
            return match what {
                None => network::interactive(),
                Some(NetworkCommand::Status) => network::status(),
                Some(NetworkCommand::WifiAdd { ssid, priority }) => {
                    network::wifi_add(&ssid, priority)
                }
                Some(NetworkCommand::Ethernet { interface }) => network::ethernet(&interface),
            };
        }
        None => {}
    }

    // The saved file wins on everything except the recording directory, which
    // the command line is allowed to override for a one-off run onto a USB drive.
    let settings = Settings {
        record_dir,
        ..Hub::load_settings(&settings_file)
    };
    let hub = Hub::new(settings, settings_file);

    for name in &args.engage {
        let kind = sensor_named(name)?;
        if let Err(error) = hub.engage(kind) {
            eprintln!("could not engage {}: {error:#}", kind.as_str());
        }
    }

    // Not fatal: a wrong pin or a missing permission would otherwise make
    // the service crash-loop and take the recorder down with it.
    if let Some(pin) = args.button {
        let config = button::ButtonConfig { pin, led: args.led.clone() };
        match button::spawn(Arc::clone(&hub), &config) {
            Ok(()) => println!(
                "  record button on GPIO{pin}{}",
                config.led.as_ref().map(|led| format!(", light on {led}")).unwrap_or_default()
            ),
            Err(error) => eprintln!("warning: no record button: {error:#}"),
        }
    }

    let state = web::AppState::new(Arc::clone(&hub));
    let address = SocketAddr::new(args.bind, args.port);
    let listener = tokio::net::TcpListener::bind(address)
        .await
        .with_context(|| format!("binding {address}"))?;
    println!("lite_record on http://{}:{}", local_address(), args.port);
    println!("  recordings -> {}", hub.settings().record_dir.display());

    let serving = axum::serve(listener, web::router(state))
        .with_graceful_shutdown(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await;
    // An mcap left unfinalised has no index and no summary, so every reader
    // rejects it. Closing on the way out is what makes ctrl-c safe.
    hub.shutdown();
    serving?;
    Ok(())
}

fn local_address() -> IpAddr {
    let probe = UdpSocket::bind("0.0.0.0:0")
        .and_then(|socket| {
            socket.connect("8.8.8.8:80")?;
            socket.local_addr()
        })
        .map(|address| address.ip());
    probe.unwrap_or(IpAddr::from([127, 0, 0, 1]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_names_are_accepted_the_way_an_operator_would_type_them() {
        assert_eq!(sensor_named("realsense").unwrap(), SensorKind::Realsense);
        assert_eq!(sensor_named(" Livox ").unwrap(), SensorKind::Livox);
        assert_eq!(sensor_named("mid360").unwrap(), SensorKind::Livox);
        assert_eq!(sensor_named("OAK-D").unwrap(), SensorKind::OakD);
        assert!(sensor_named("velodyne").is_err());
    }

    #[test]
    fn the_installed_service_gets_absolute_paths_for_every_file_it_touches() {
        let args = Args::parse_from(["lite_record"]);
        let arguments = args.service_arguments(
            Path::new("/data/recordings"),
            Path::new("/home/pi/.dimos/lite_record.json"),
        );
        assert!(arguments.contains(&"/data/recordings".to_string()));
        assert!(arguments.contains(&"/home/pi/.dimos/lite_record.json".to_string()));
        for argument in &arguments {
            if argument.starts_with('/') {
                assert!(Path::new(argument).is_absolute());
            }
        }
    }
}
