use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use lite_record::hub::{Hub, Settings};
use lite_record::sensors::SensorKind;
use lite_record::{convert, heatmap, service, video, web};
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::Arc;

#[derive(Parser)]
#[command(name = "lite_record", about = "Sensor recorder for handheld rigs")]
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

    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand)]
enum Command {
    /// Install lite_record as a boot service (systemd or launchd) with these
    /// same options, and start it now. Asks for sudo.
    #[command(name = "survive_reboot", alias = "survive-reboot")]
    SurviveReboot,

    /// Rewrite a recording in place, moving every jxl stream into a format
    /// Foxglove can decode. The same work the UI's Post process button does,
    /// for a rig with no browser pointed at it.
    #[command(name = "post_process", alias = "post-process")]
    PostProcess {
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
    },

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
fn with_ticker<T>(describe: impl Fn() -> String + Send + 'static, work: impl FnOnce() -> T) -> T {
    let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let stopping = Arc::clone(&running);
    let ticker = std::thread::spawn(move || {
        let mut seconds = 0;
        while stopping.load(std::sync::atomic::Ordering::Relaxed) {
            std::thread::sleep(std::time::Duration::from_secs(1));
            seconds += 1;
            if seconds % 60 == 0 {
                println!("  {:>6}s  {}", seconds, describe());
            }
        }
    });
    let result = work();
    running.store(false, std::sync::atomic::Ordering::Relaxed);
    let _ = ticker.join();
    result
}

fn load_urdf(path: Option<&Path>) -> Result<Option<lite_record::urdf::Urdf>> {
    let Some(path) = path else {
        return Ok(None);
    };
    let xml = std::fs::read_to_string(path).with_context(|| format!("could not read {}", path.display()))?;
    Ok(Some(lite_record::urdf::parse(&xml).with_context(|| format!("{} is not a urdf", path.display()))?))
}

/// The three stages that turn a fresh recording into one that is viewable,
/// placed and localised: decode jxl (a rewrite, skipped when there is none),
/// then complete the frame tree and estimate odometry (both appended).
fn post_process(
    recording: &Path,
    reclaim: bool,
    no_odom: bool,
    urdf: Option<&Path>,
    lidar_topic: Option<&str>,
    imu_topic: Option<&str>,
    dry_run: bool,
) -> Result<()> {
    let started = std::time::Instant::now();
    let urdf = load_urdf(urdf)?;

    let reclaim = match reclaim {
        true => convert::Reclaim::AsItGoes,
        false => convert::Reclaim::No,
    };
    let progress = Arc::new(convert::Progress::default());
    let watched = Arc::clone(&progress);
    let converted = with_ticker(
        move || {
            format!(
                "{} messages  {:.2} GB written",
                watched.messages.load(std::sync::atomic::Ordering::Relaxed),
                watched.bytes.load(std::sync::atomic::Ordering::Relaxed) as f64 / 1e9,
            )
        },
        || match (dry_run, convert::needs_conversion(recording)) {
            (true, Ok(true)) => {
                println!("{}: would convert jxl / refit camera infos (dry run)", recording.display());
                Err(convert::NothingToConvert.into())
            }
            (true, Ok(false)) => Err(convert::NothingToConvert.into()),
            (true, Err(error)) => Err(error),
            (false, _) => convert::in_place(recording, &progress, reclaim),
        },
    );
    match converted {
        Ok(report) => println!(
            "{}: {} decoded, {} refitted, {} transforms inverted, {} copied, {} failed, {:.2} GB, {:.2} GB reclaimed, {}s",
            recording.display(),
            report.decoded,
            report.refitted,
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

    let estimate = if no_odom {
        None
    } else if let Some(count) = lite_record::odometry::already_present(recording)? {
        println!("{} already carries {count} messages on {}; not estimating again", recording.display(), lite_record::odometry::ODOMETRY_TOPIC);
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
                println!("estimating odometry from {lidar} + {imu} (this takes a while)");
                let scans = Arc::new(std::sync::atomic::AtomicU64::new(0));
                let watched = Arc::clone(&scans);
                let estimate = with_ticker(
                    move || format!("{} scans", watched.load(std::sync::atomic::Ordering::Relaxed)),
                    || lite_record::odometry::estimate(&mapped, &lidar, &imu, &scans),
                )?;
                println!(
                    "  {} poses, {:.1} m of path, log clock {:+.3} s from the lidar's stamps",
                    estimate.poses.len(),
                    estimate.path_length_metres,
                    estimate.log_offset_seconds
                );
                Some(estimate)
            }
        }
    };
    drop(mapped);

    if dry_run || (plan.new_edges.is_empty() && estimate.is_none()) {
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
        return Ok(());
    }
    let mut appender = lite_record::mcap_append::Appender::open(recording)?;
    let static_messages = lite_record::fixup::append_static_transforms(
        &mut appender,
        &plan.new_edges,
        inspected.start_nanos,
        inspected.end_nanos,
    )?;
    let appended = match &estimate {
        Some(estimate) => Some(lite_record::odometry::append(&mut appender, estimate, &plan.tree)?),
        None => None,
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
    println!("{total} messages appended to {}; {}s", recording.display(), started.elapsed().as_secs());
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
        Some(Command::PostProcess { recording, reclaim, no_odom, urdf, lidar_topic, imu_topic, dry_run }) => {
            return post_process(
                &recording,
                reclaim,
                no_odom,
                urdf.as_deref(),
                lidar_topic.as_deref(),
                imu_topic.as_deref(),
                dry_run,
            );
        }
        Some(Command::TfFixup { recording, urdf }) => {
            return tf_fixup(&recording, urdf.as_deref());
        }
        Some(Command::Heatmap(options)) => return heatmap::run(&options),
        Some(Command::ToVideo(options)) => return video::run(&options),
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
