//! The server binds `0.0.0.0` and one of its endpoints runs an arbitrary shell
//! line, so the gate in front of it is the only thing between a shared network
//! and a root shell. This drives the real router over a real socket, because a
//! guard that is right in a unit test and wired up wrong in the router is worth
//! nothing.

use std::io::{Read, Write};
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use lite_record::access::HEADER;
use lite_record::hub::{Hub, Settings};
use lite_record::web;

const PASSWORD: &str = "field-rig-2026";

/// Every endpoint that runs something on the machine. Read-only endpoints and
/// the recording controls are deliberately not in this list.
const GUARDED: &[&str] = &[
    "/api/terminal",
    "/api/usb/mount",
    "/api/network/mid360",
    "/api/password",
];

/// Enough keys to satisfy every guarded endpoint's body, so a rejected body can
/// never be mistaken for the gate doing its job. The interface name is
/// deliberately not a real one: an authorised call should get as far as refusing
/// it and no further, rather than reconfiguring the machine running the tests.
const BODY: &str = r#"{"line":"true","password":"x","interface":"not an interface"}"#;

struct Server {
    address: SocketAddr,
    directory: PathBuf,
}

impl Drop for Server {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.directory);
    }
}

/// Each server gets its own directory: the password file sits beside the
/// settings under a fixed name, so two of these sharing a directory would share
/// a password and race each other.
async fn serve(name: &str) -> Server {
    let directory = std::env::temp_dir().join(format!("lite_record_gate_{name}"));
    let _ = std::fs::remove_dir_all(&directory);
    std::fs::create_dir_all(&directory).unwrap();

    let hub = Hub::new(
        Settings {
            record_dir: directory.join("recordings"),
            ..Settings::default()
        },
        directory.join("settings.json"),
    );
    let state = web::AppState::new(Arc::clone(&hub));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap();
    tokio::spawn(async move {
        let _ = axum::serve(listener, web::router(state)).await;
    });
    Server { address, directory }
}

/// One HTTP/1.1 request over a plain socket. A test that only needs a status
/// code and a small JSON body does not need an HTTP client library, and this
/// keeps the dependency out of the build.
fn request(
    address: SocketAddr,
    method: &str,
    path: &str,
    password: Option<&str>,
    body: Option<&str>,
) -> (u16, String) {
    let mut socket = std::net::TcpStream::connect(address).unwrap();
    let mut head = format!(
        "{method} {path} HTTP/1.1\r\nHost: {address}\r\nConnection: close\r\n\
         Content-Type: application/json\r\nContent-Length: {}\r\n",
        body.unwrap_or("").len()
    );
    if let Some(password) = password {
        head.push_str(&format!("{HEADER}: {password}\r\n"));
    }
    head.push_str("\r\n");
    socket.write_all(head.as_bytes()).unwrap();
    socket.write_all(body.unwrap_or("").as_bytes()).unwrap();

    // A 401 is decided from the headers, so the server answers and closes while
    // the body written above is still sitting unread in its receive buffer —
    // which makes the close an RST, and turns a perfectly good response already
    // in our buffer into a ConnectionReset. Keep what arrived.
    let mut answer = Vec::new();
    let mut chunk = [0u8; 4096];
    loop {
        match socket.read(&mut chunk) {
            Ok(0) => break,
            Ok(read) => answer.extend_from_slice(&chunk[..read]),
            Err(error) if error.kind() == std::io::ErrorKind::ConnectionReset => break,
            Err(error) => panic!("reading {path}: {error}"),
        }
    }
    assert!(!answer.is_empty(), "{path} closed without answering");
    let answer = String::from_utf8_lossy(&answer).into_owned();
    let status = answer
        .split_whitespace()
        .nth(1)
        .and_then(|code| code.parse().ok())
        .unwrap_or(0);
    let body = answer.split_once("\r\n\r\n").map(|split| split.1).unwrap_or("");
    (status, body.to_string())
}

fn post(address: SocketAddr, path: &str, password: Option<&str>) -> u16 {
    request(address, "POST", path, password, Some(BODY)).0
}

fn set_password(address: SocketAddr, current: Option<&str>, next: Option<&str>) -> u16 {
    let body = serde_json::json!({ "current": current, "next": next }).to_string();
    request(address, "POST", "/api/access", None, Some(&body)).0
}

fn get_json(address: SocketAddr, path: &str, password: Option<&str>) -> serde_json::Value {
    let (status, body) = request(address, "GET", path, password, None);
    assert_eq!(status, 200, "{path} answered {status}: {body}");
    // The body is chunked, so the JSON is what sits between the first brace and
    // the last.
    let start = body.find('{').unwrap();
    let end = body.rfind('}').unwrap();
    serde_json::from_str(&body[start..=end]).unwrap()
}

#[tokio::test(flavor = "multi_thread")]
async fn a_system_command_needs_the_password_and_a_recording_does_not() {
    let server = serve("mixed").await;
    let address = server.address;

    // Before a password is set the rig works exactly as it did.
    for path in GUARDED {
        assert_ne!(post(address, path, None), 401, "{path} before setup");
    }

    assert_eq!(set_password(address, None, Some(PASSWORD)), 200);

    for path in GUARDED {
        assert_eq!(post(address, path, None), 401, "{path} with no password");
        assert_eq!(
            post(address, path, Some("wrong-password")),
            401,
            "{path} with the wrong password"
        );
        assert_ne!(
            post(address, path, Some(PASSWORD)),
            401,
            "{path} with the right password"
        );
    }

    // Reading the state of the rig, and recording with it, stay open: the person
    // holding it should not have to type a password to press record.
    for path in ["/api/status", "/api/recordings", "/api/storage/volumes"] {
        let (status, _) = request(address, "GET", path, None, None);
        assert_eq!(status, 200, "{path} answered {status}");
    }
    assert_ne!(post(address, "/api/record/stop", None), 401);
}

#[tokio::test(flavor = "multi_thread")]
async fn the_password_cannot_be_changed_by_someone_who_does_not_have_it() {
    let server = serve("change").await;
    let address = server.address;
    assert_eq!(set_password(address, None, Some(PASSWORD)), 200);

    assert_eq!(set_password(address, None, Some("taken-over")), 400);
    assert_eq!(set_password(address, Some("guess"), Some("taken-over")), 400);
    assert_eq!(post(address, "/api/password", Some(PASSWORD)), 200);

    assert_eq!(set_password(address, Some(PASSWORD), Some("second-password")), 200);
    assert_eq!(post(address, "/api/password", Some(PASSWORD)), 401);
    assert_eq!(post(address, "/api/password", Some("second-password")), 200);
}

#[tokio::test(flavor = "multi_thread")]
async fn the_page_can_ask_whether_the_password_it_cached_is_still_right() {
    let server = serve("cached").await;
    let address = server.address;

    let asked = get_json(address, "/api/access", None);
    assert_eq!(asked["password_required"], false);
    assert_eq!(asked["allowed"], true);

    assert_eq!(set_password(address, None, Some(PASSWORD)), 200);

    let stale = get_json(address, "/api/access", Some("what-it-had-cached"));
    assert_eq!(stale["password_required"], true);
    assert_eq!(stale["allowed"], false);

    let good = get_json(address, "/api/access", Some(PASSWORD));
    assert_eq!(good["allowed"], true);
}
