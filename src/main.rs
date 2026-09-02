use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use lite_record::hub::{Hub, Settings};
use lite_record::sensors::SensorKind;
use lite_record::{service, web};
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

fn sensor_named(name: &str) -> Result<SensorKind> {
    match name.trim().to_ascii_lowercase().as_str() {
        "realsense" => Ok(SensorKind::Realsense),
        "orbbec" => Ok(SensorKind::Orbbec),
        "livox" | "mid360" => Ok(SensorKind::Livox),
        other => anyhow::bail!("{other:?} is not a sensor; expected realsense, orbbec or livox"),
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

    if let Some(Command::SurviveReboot) = args.command {
        let arguments = args.service_arguments(&record_dir, &settings_file);
        let working_directory = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/"));
        return service::install(&arguments, &working_directory);
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
