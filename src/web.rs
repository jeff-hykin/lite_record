//! The HTTP and websocket surface.
//!
//! Two websockets rather than one: the monitor ticks at 5 Hz whether or not
//! anyone is watching a camera, and the preview pushes binary jpeg frames. Mixing
//! them meant a slow image send delayed the numbers that tell the operator the
//! Pi is browning out, which is exactly when they need to be quick.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Path, State};
use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{delete, get, post, put};
use axum::Router;
use bytes::Bytes;
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use serde_json::json;

use crate::hub::{Hub, Settings};
use crate::privileged::{self, Secret};
use crate::record;
use crate::sensors::SensorKind;

/// The monitor's tick. Jeff asked for about 5 Hz specifically so the readout
/// does not itself become a measurable load on a Pi.
const MONITOR_INTERVAL: Duration = Duration::from_millis(200);

/// The preview is polled rather than pushed, because the hub keeps only the
/// newest frame and a client that fell behind should skip to it.
const PREVIEW_POLL: Duration = Duration::from_millis(20);

#[derive(Clone)]
pub struct AppState {
    pub hub: Arc<Hub>,
    /// The sudo password, held in memory for this process only. Never
    /// serialised, never written to the settings file, never logged.
    password: Arc<Mutex<Secret>>,
}

impl AppState {
    pub fn new(hub: Arc<Hub>) -> Self {
        AppState {
            hub,
            password: Arc::new(Mutex::new(Secret::default())),
        }
    }

    fn password(&self) -> Secret {
        self.password.lock().unwrap().clone()
    }
}

pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/app.js", get(script))
        .route("/style.css", get(stylesheet))
        .route("/healthz", get(|| async { "ok" }))
        .route("/api/status", get(status))
        .route("/api/settings", put(put_settings))
        .route("/api/recordings", get(recordings))
        .route("/api/recordings/{name}", delete(remove_recording))
        .route("/api/recordings/{name}/download", get(download_recording))
        .route("/api/record/start", post(start_recording))
        .route("/api/record/stop", post(stop_recording))
        .route("/api/sensors/{kind}/{action}", post(sensor_action))
        .route("/api/urdf", get(get_urdf).put(put_urdf))
        .route("/api/urdf/inspect", post(inspect_urdf))
        .route("/api/password", post(set_password))
        .route("/api/terminal", post(terminal))
        .route("/api/usb/mount", post(mount_usb))
        .route("/api/network/mid360", post(configure_lidar_network))
        .route("/ws/monitor", get(monitor_socket))
        .route("/ws/preview", get(preview_socket))
        .with_state(state)
}

async fn index() -> Response {
    (
        [(header::CONTENT_TYPE, "text/html; charset=utf-8")],
        include_str!("../web/index.html"),
    )
        .into_response()
}

async fn script() -> Response {
    (
        [(header::CONTENT_TYPE, "text/javascript; charset=utf-8")],
        include_str!("../web/app.js"),
    )
        .into_response()
}

async fn stylesheet() -> Response {
    (
        [(header::CONTENT_TYPE, "text/css; charset=utf-8")],
        include_str!("../web/style.css"),
    )
        .into_response()
}

fn bad_request(error: impl std::fmt::Display) -> Response {
    (
        StatusCode::BAD_REQUEST,
        axum::Json(json!({ "error": error.to_string() })),
    )
        .into_response()
}

/// A URDF report plus its prose. The problem messages live on the Rust enum, so
/// rendering them here rather than in the browser keeps one wording of each.
fn urdf_payload(report: &crate::hub::UrdfReport) -> serde_json::Value {
    json!({
        "report": report,
        "warning": report.warning(),
        "problems": report
            .problems
            .iter()
            .map(crate::urdf::TreeProblem::message)
            .collect::<Vec<_>>(),
    })
}

/// Everything the page needs on load, and everything a slow poller needs to
/// recover after a websocket drop.
fn status_payload(state: &AppState) -> serde_json::Value {
    let settings = state.hub.settings();
    let urdf = state.hub.inspect_urdf(settings.urdf_xml.as_deref());
    json!({
        "settings": settings,
        "sensors": state.hub.sensor_status(),
        "recording": state.hub.recording_status(),
        "streams": state.hub.stream_stats(),
        "preview_topics": state.hub.preview_topics(),
        // Not the same as settings.preview_topic: with nothing chosen the hub
        // picks one, and the dropdown has to show what is really being encoded.
        "preview_topic": state.hub.preview_topic(),
        "urdf": urdf_payload(&urdf),
        "removable_mounts": privileged::likely_removable_mounts(),
        "is_root": privileged::is_root(),
        "has_password": !state.password().is_empty(),
    })
}

async fn status(State(state): State<AppState>) -> Response {
    axum::Json(status_payload(&state)).into_response()
}

/// A change to resolution or frame rate reopens the camera, and opening a USB3
/// pipeline takes seconds of blocking work, so this cannot run on a runtime
/// worker or the monitor socket stops ticking exactly when the operator is
/// watching to see whether the change took.
async fn put_settings(
    State(state): State<AppState>,
    axum::Json(change): axum::Json<serde_json::Value>,
) -> Response {
    // `Settings` carries `#[serde(default)]` so an old settings file missing a
    // key still loads, which means deserialising the request on its own turns
    // every key the caller left out into its *default* rather than leaving it
    // alone. Sending `{"preview_enabled": true}` that way disabled the lidar,
    // dropped the camera to 640x480 and moved the recording directory. So merge
    // over the settings in force and deserialise the result.
    let mut merged = match serde_json::to_value(state.hub.settings()) {
        Ok(value) => value,
        Err(error) => return bad_request(format!("{error:#}")),
    };
    merge_into(&mut merged, change);
    let settings: Settings = match serde_json::from_value(merged) {
        Ok(settings) => settings,
        Err(error) => return bad_request(format!("{error:#}")),
    };
    let hub = Arc::clone(&state.hub);
    let applied = tokio::task::spawn_blocking(move || hub.update_settings(settings)).await;
    match applied {
        Ok(Ok(())) => axum::Json(status_payload(&state)).into_response(),
        Ok(Err(error)) => bad_request(format!("{error:#}")),
        Err(error) => bad_request(error),
    }
}

/// Overlays `change` onto `target`, descending into objects so that naming a
/// single camera field leaves that camera's other fields standing. Anything that
/// is not an object replaces wholesale, which is what an array of one setting
/// should do.
fn merge_into(target: &mut serde_json::Value, change: serde_json::Value) {
    match (target, change) {
        (serde_json::Value::Object(existing), serde_json::Value::Object(incoming)) => {
            for (key, value) in incoming {
                merge_into(existing.entry(key).or_insert(serde_json::Value::Null), value)
            }
        }
        (target, change) => *target = change,
    }
}

/// Whether `path` is the file a recording is writing *now*. The status keeps the
/// last path after a recording stops so the UI can still name it, so the active
/// flag has to be checked too — without it a finished recording could never be
/// deleted or downloaded.
fn is_being_recorded(state: &AppState, path: &std::path::Path) -> bool {
    let status = state.hub.recording_status();
    status.active
        && status
            .path
            .as_deref()
            .is_some_and(|writing| writing == path.to_string_lossy())
}

async fn recordings(State(state): State<AppState>) -> Response {
    let directory = state.hub.settings().record_dir;
    axum::Json(record::list(&directory)).into_response()
}

async fn remove_recording(Path(name): Path<String>, State(state): State<AppState>) -> Response {
    let directory = state.hub.settings().record_dir;
    let path = match record::resolve(&directory, &name) {
        Ok(path) => path,
        Err(error) => return bad_request(error),
    };
    if is_being_recorded(&state, &path) {
        return bad_request("that file is being recorded right now");
    }
    match std::fs::remove_file(&path) {
        Ok(()) => axum::Json(json!({ "ok": true })).into_response(),
        Err(error) => bad_request(error),
    }
}

/// The byte range a `Range` header asks for, as an inclusive pair clamped to the
/// file. An unparseable or unsatisfiable header answers `None`, which sends the
/// whole file: a server is allowed to ignore `Range`, and a download that is
/// merely not resumable beats one that fails.
fn requested_range(header: Option<&str>, length: u64) -> Option<(u64, u64)> {
    let spec = header?.trim().strip_prefix("bytes=")?;
    if spec.contains(',') || length == 0 {
        return None;
    }
    let (start, end) = spec.split_once('-')?;
    let start: u64 = start.trim().parse().ok()?;
    let end = match end.trim() {
        "" => length - 1,
        text => text.parse::<u64>().ok()?.min(length - 1),
    };
    (start <= end).then_some((start, end))
}

/// Streams a recording back over HTTP, so pulling one off the Pi needs nothing
/// but a URL — no ssh, no scp, no account on the box.
async fn download_recording(
    Path(name): Path<String>,
    headers_in: axum::http::HeaderMap,
    State(state): State<AppState>,
) -> Response {
    let directory = state.hub.settings().record_dir;
    let path = match record::resolve(&directory, &name) {
        Ok(path) => path,
        Err(error) => return bad_request(error),
    };
    if is_being_recorded(&state, &path) {
        // An mcap grows its index and footer at the end, so a copy taken now is
        // one no indexed reader will open. Stopping first is the fix.
        return bad_request("that file is being recorded right now; stop the recording first");
    }
    let file = match tokio::fs::File::open(&path).await {
        Ok(file) => file,
        Err(error) => return bad_request(format!("{}: {error}", path.display())),
    };
    let length = match file.metadata().await {
        Ok(data) => data.len(),
        Err(error) => return bad_request(error),
    };

    let file_name = path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| name.clone());
    let mut headers = axum::http::HeaderMap::new();
    let mut set = |name: header::HeaderName, value: String| {
        if let Ok(value) = value.parse() {
            headers.insert(name, value);
        }
    };
    set(header::CONTENT_TYPE, "application/octet-stream".into());
    set(header::ACCEPT_RANGES, "bytes".into());
    set(
        header::CONTENT_DISPOSITION,
        format!("attachment; filename=\"{file_name}\""),
    );

    let asked = headers_in
        .get(header::RANGE)
        .and_then(|value| value.to_str().ok());
    let (status, start, span) = match requested_range(asked, length) {
        Some((start, end)) => {
            set(
                header::CONTENT_RANGE,
                format!("bytes {start}-{end}/{length}"),
            );
            (StatusCode::PARTIAL_CONTENT, start, end + 1 - start)
        }
        None => (StatusCode::OK, 0, length),
    };
    set(header::CONTENT_LENGTH, span.to_string());

    let mut file = file;
    if start > 0 {
        use tokio::io::AsyncSeekExt;
        if let Err(error) = file.seek(std::io::SeekFrom::Start(start)).await {
            return bad_request(error);
        }
    }
    let body = axum::body::Body::from_stream(tokio_util::io::ReaderStream::new(
        tokio::io::AsyncReadExt::take(file, span),
    ));
    (status, headers, body).into_response()
}

#[derive(Deserialize, Default)]
struct StartRecording {
    #[serde(default)]
    name: Option<String>,
}

async fn start_recording(
    State(state): State<AppState>,
    body: Option<axum::Json<StartRecording>>,
) -> Response {
    let name = body.and_then(|axum::Json(body)| body.name);
    match state.hub.start_recording(name.as_deref()) {
        Ok(status) => axum::Json(status).into_response(),
        Err(error) => bad_request(error),
    }
}

async fn stop_recording(State(state): State<AppState>) -> Response {
    // Finalising an mcap drains the writer queue and writes the index, which on
    // a multi-gigabyte file takes long enough to block the async runtime.
    let hub = Arc::clone(&state.hub);
    let finished = tokio::task::spawn_blocking(move || hub.stop_recording()).await;
    match finished {
        Ok(Ok(status)) => axum::Json(status).into_response(),
        Ok(Err(error)) => bad_request(error),
        Err(error) => bad_request(error),
    }
}

async fn sensor_action(
    Path((kind, action)): Path<(String, String)>,
    State(state): State<AppState>,
) -> Response {
    let kind = match kind.as_str() {
        "realsense" => SensorKind::Realsense,
        "orbbec" => SensorKind::Orbbec,
        "oakd" => SensorKind::OakD,
        "livox" => SensorKind::Livox,
        other => return bad_request(format!("{other:?} is not a sensor")),
    };
    // Opening or releasing a device blocks for as long as the driver takes, so
    // it is kept off the runtime workers that serve the monitor socket.
    let hub = Arc::clone(&state.hub);
    let outcome = tokio::task::spawn_blocking(move || match action.as_str() {
        // `engage` already cycles the device, so restart is the same call under
        // the name an operator reaches for when a camera has wedged.
        "engage" | "restart" => hub.engage(kind).map_err(|error| format!("{error:#}")),
        "disengage" => {
            hub.disengage(kind);
            Ok(())
        }
        other => Err(format!("{other:?} is not engage, restart or disengage")),
    })
    .await;
    match outcome {
        Ok(Ok(())) => axum::Json(state.hub.sensor_status()).into_response(),
        Ok(Err(error)) => bad_request(error),
        Err(error) => bad_request(error),
    }
}

async fn get_urdf(State(state): State<AppState>) -> Response {
    match state.hub.settings().urdf_xml {
        Some(xml) => ([(header::CONTENT_TYPE, "application/xml")], xml).into_response(),
        None => (StatusCode::NOT_FOUND, "no urdf uploaded").into_response(),
    }
}

/// The body is the raw URDF, not json, so a browser can hand over the file it
/// was given without re-encoding a multi-megabyte string.
async fn put_urdf(State(state): State<AppState>, body: String) -> Response {
    let xml = if body.trim().is_empty() {
        None
    } else {
        Some(body)
    };
    match state.hub.set_urdf(xml) {
        Ok(report) => axum::Json(urdf_payload(&report)).into_response(),
        Err(error) => bad_request(error),
    }
}

/// Checks a URDF without saving it, so the upload dialog can warn before the
/// operator commits to a file that would break the tree.
async fn inspect_urdf(State(state): State<AppState>, body: String) -> Response {
    let report = state.hub.inspect_urdf(Some(&body));
    axum::Json(urdf_payload(&report)).into_response()
}

#[derive(Deserialize)]
struct PasswordBody {
    password: String,
}

/// Stores the sudo password for this process only. The response deliberately
/// echoes nothing back but a boolean.
async fn set_password(
    State(state): State<AppState>,
    axum::Json(body): axum::Json<PasswordBody>,
) -> Response {
    *state.password.lock().unwrap() = Secret::new(body.password);
    axum::Json(json!({ "has_password": !state.password().is_empty() })).into_response()
}

#[derive(Deserialize)]
struct TerminalBody {
    line: String,
    #[serde(default)]
    as_root: bool,
}

async fn terminal(
    State(state): State<AppState>,
    axum::Json(body): axum::Json<TerminalBody>,
) -> Response {
    let planned = privileged::terminal_plan(&body.line, body.as_root);
    let password = state.password();
    match privileged::run(&planned, Some(&password)).await {
        Ok(result) => axum::Json(result).into_response(),
        Err(error) => bad_request(format!("{error:#}")),
    }
}

/// Runs a whole plan, stopping at the first fatal failure but carrying on past
/// the steps marked optional, and returns the transcript either way.
async fn run_plan(plan: &[privileged::Planned], password: &Secret) -> serde_json::Value {
    let mut transcript = Vec::new();
    let mut failed = None;
    for planned in plan {
        match privileged::run(planned, Some(password)).await {
            Ok(result) => {
                let fatal = !result.succeeded() && !planned.optional;
                transcript.push(json!({
                    "reason": planned.reason,
                    "optional": planned.optional,
                    "result": result,
                }));
                if fatal {
                    failed = Some(planned.display());
                    break;
                }
            }
            Err(error) => {
                transcript.push(json!({
                    "reason": planned.reason,
                    "optional": planned.optional,
                    "error": format!("{error:#}"),
                }));
                if !planned.optional {
                    failed = Some(planned.display());
                    break;
                }
            }
        }
    }
    json!({ "steps": transcript, "failed": failed })
}

async fn mount_usb(State(state): State<AppState>) -> Response {
    let password = state.password();
    let listing = privileged::terminal_plan("lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINT,LABEL,TRAN -J", false);
    let found = match privileged::run(&listing, None).await {
        Ok(result) => result.stdout,
        Err(error) => return bad_request(format!("{error:#}")),
    };
    let partitions = match privileged::removable_partitions(&found) {
        Ok(partitions) => partitions,
        Err(error) => return bad_request(format!("{error:#}")),
    };
    if partitions.is_empty() {
        return axum::Json(json!({
            "steps": [],
            "failed": null,
            "note": "no unmounted usb partition found",
        }))
        .into_response();
    }
    let plan = privileged::usb_mount_plan(&partitions);
    let mut outcome = run_plan(&plan, &password).await;
    outcome["mounts"] = json!(privileged::likely_removable_mounts());
    axum::Json(outcome).into_response()
}

#[derive(Deserialize)]
struct NetworkBody {
    interface: String,
    #[serde(default)]
    host_address: Option<String>,
}

async fn configure_lidar_network(
    State(state): State<AppState>,
    axum::Json(body): axum::Json<NetworkBody>,
) -> Response {
    let settings = state.hub.settings();
    let host_address = body
        .host_address
        .filter(|address| !address.trim().is_empty())
        .unwrap_or_else(|| {
            privileged::suggest_host_address(settings.livox.lidar_address.as_deref())
        });
    let plan = match privileged::mid360_network_plan(&body.interface, &host_address) {
        Ok(plan) => plan,
        Err(error) => return bad_request(format!("{error:#}")),
    };
    let mut outcome = run_plan(&plan, &state.password()).await;
    outcome["host_address"] = json!(host_address);
    axum::Json(outcome).into_response()
}

// -- websockets -----------------------------------------------------------

async fn monitor_socket(upgrade: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    upgrade.on_upgrade(move |socket| run_monitor_socket(socket, state))
}

/// Reading and writing run as separate tasks so a payload that cannot drain
/// into a congested phone does not also hold up a Stop Recording press.
async fn run_monitor_socket(socket: WebSocket, state: AppState) {
    let (mut sink, mut stream) = socket.split();
    let writer_state = state.clone();
    let writer = tokio::spawn(async move {
        let mut ticker = tokio::time::interval(MONITOR_INTERVAL);
        // A client that fell behind should get the next tick, not a burst of
        // the ones it missed, which would only push it further behind.
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            ticker.tick().await;
            let health = writer_state.hub.health();
            let warning = health.throttle.as_ref().and_then(crate::sysmon::Throttle::warning);
            let payload = json!({
                "health": health,
                "warning": warning,
                "streams": writer_state.hub.stream_stats(),
                "recording": writer_state.hub.recording_status(),
                "sensors": writer_state.hub.sensor_status(),
            })
            .to_string();
            if sink.send(Message::Text(payload.into())).await.is_err() {
                break;
            }
        }
    });

    // Nothing is expected from the browser here; the read half exists so a
    // closed socket is noticed promptly rather than at the next failed send.
    while let Some(Ok(_)) = stream.next().await {}
    writer.abort();
}

async fn preview_socket(upgrade: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    upgrade.on_upgrade(move |socket| run_preview_socket(socket, state))
}

/// Sends the newest frame available at the moment the socket is free. A client
/// that cannot keep up simply misses the frames it slept through.
async fn run_preview_socket(mut socket: WebSocket, state: AppState) {
    let mut since_frame = Duration::ZERO;
    loop {
        tokio::time::sleep(PREVIEW_POLL).await;
        match state.hub.take_preview() {
            Some(frame) => {
                since_frame = Duration::ZERO;
                if socket.send(Message::Binary(frame.bytes)).await.is_err() {
                    break;
                }
            }
            None => {
                // A preview that is switched off, or a camera that is not
                // engaged, must still let us notice a browser that walked away.
                since_frame += PREVIEW_POLL;
                if since_frame >= Duration::from_secs(5) {
                    since_frame = Duration::ZERO;
                    if socket.send(Message::Ping(Bytes::new())).await.is_err() {
                        break;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch_state() -> AppState {
        let file = std::env::temp_dir().join(format!("lite_record_web_{}.json", record::now_nanos()));
        AppState::new(Hub::new(Settings::default(), file))
    }

    #[tokio::test]
    async fn naming_one_setting_leaves_every_other_setting_alone() {
        let state = scratch_state();
        let mut wanted = state.hub.settings();
        wanted.realsense.width = 1280;
        wanted.realsense.height = 720;
        wanted.livox.enabled = true;
        wanted.record_dir = "/tmp/somewhere".into();
        state.hub.update_settings(wanted).unwrap();

        let response = put_settings(
            State(state.clone()),
            axum::Json(json!({"preview_enabled": true, "realsense": {"frame_rate": 15}})),
        )
        .await;
        assert_eq!(response.status(), StatusCode::OK);

        let after = state.hub.settings();
        assert!(after.preview_enabled);
        assert_eq!(after.realsense.frame_rate, 15);
        // The keys the request never mentioned.
        assert_eq!(after.realsense.width, 1280);
        assert_eq!(after.realsense.height, 720);
        assert!(after.livox.enabled);
        assert_eq!(after.record_dir, std::path::PathBuf::from("/tmp/somewhere"));
        std::fs::remove_file(state.hub.settings_file()).ok();
    }

    #[test]
    fn the_status_payload_never_carries_the_password() {
        let state = scratch_state();
        *state.password.lock().unwrap() = Secret::new("hunter2".into());
        let payload = status_payload(&state).to_string();
        assert!(!payload.contains("hunter2"), "{payload}");
        // The UI still has to know whether it needs to ask for one.
        assert!(payload.contains("\"has_password\":true"));
        std::fs::remove_file(state.hub.settings_file()).ok();
    }

    #[test]
    fn a_range_header_is_read_as_an_inclusive_pair_clamped_to_the_file() {
        assert_eq!(requested_range(Some("bytes=0-9"), 100), Some((0, 9)));
        // An open end, which is what a resumed download sends.
        assert_eq!(requested_range(Some("bytes=40-"), 100), Some((40, 99)));
        // Past the end is clamped rather than refused.
        assert_eq!(requested_range(Some("bytes=90-500"), 100), Some((90, 99)));
    }

    #[test]
    fn a_range_that_cannot_be_honoured_sends_the_whole_file() {
        assert_eq!(requested_range(None, 100), None);
        assert_eq!(requested_range(Some("bytes=-20"), 100), None);
        assert_eq!(requested_range(Some("bytes=0-9,20-29"), 100), None);
        assert_eq!(requested_range(Some("items=0-9"), 100), None);
        assert_eq!(requested_range(Some("bytes=80-40"), 100), None);
        assert_eq!(requested_range(Some("bytes=0-9"), 0), None);
    }

    #[tokio::test]
    async fn a_recording_downloads_whole_and_by_range() {
        let state = scratch_state();
        let directory = std::env::temp_dir().join(format!("lite_web_dl_{}", record::now_nanos()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut settings = state.hub.settings();
        settings.record_dir = directory.clone();
        state.hub.update_settings(settings).unwrap();
        let body: Vec<u8> = (0..=255u8).collect();
        std::fs::write(directory.join("clip.mcap"), &body).unwrap();

        let fetch = |range: Option<&'static str>| {
            let mut headers = axum::http::HeaderMap::new();
            if let Some(range) = range {
                headers.insert(header::RANGE, range.parse().unwrap());
            }
            download_recording(
                Path("clip.mcap".to_string()),
                headers,
                State(state.clone()),
            )
        };

        let whole = fetch(None).await;
        assert_eq!(whole.status(), StatusCode::OK);
        assert_eq!(whole.headers()[header::ACCEPT_RANGES], "bytes");
        let bytes = axum::body::to_bytes(whole.into_body(), 1 << 20).await.unwrap();
        assert_eq!(bytes.as_ref(), body.as_slice());

        let part = fetch(Some("bytes=250-")).await;
        assert_eq!(part.status(), StatusCode::PARTIAL_CONTENT);
        assert_eq!(part.headers()[header::CONTENT_RANGE], "bytes 250-255/256");
        let bytes = axum::body::to_bytes(part.into_body(), 1 << 20).await.unwrap();
        assert_eq!(bytes.as_ref(), &body[250..]);

        std::fs::remove_dir_all(&directory).ok();
        std::fs::remove_file(state.hub.settings_file()).ok();
    }

    #[tokio::test]
    async fn a_failed_optional_step_does_not_abort_the_plan() {
        let plan = vec![
            privileged::terminal_plan("exit 1", false),
            privileged::terminal_plan("echo second", false),
        ];
        let mut plan = plan;
        plan[0].optional = true;
        let outcome = run_plan(&plan, &Secret::default()).await;
        assert!(outcome["failed"].is_null());
        assert_eq!(outcome["steps"].as_array().unwrap().len(), 2);
    }

    #[tokio::test]
    async fn a_failed_required_step_stops_the_plan_and_names_itself() {
        let plan = vec![
            privileged::terminal_plan("exit 1", false),
            privileged::terminal_plan("echo never", false),
        ];
        let outcome = run_plan(&plan, &Secret::default()).await;
        assert_eq!(outcome["failed"], json!("sh -c exit 1"));
        assert_eq!(outcome["steps"].as_array().unwrap().len(), 1);
    }
}
