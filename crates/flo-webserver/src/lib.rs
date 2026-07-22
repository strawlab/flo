use axum::{
    Json,
    extract::{Path, State},
    response::{
        IntoResponse,
        sse::{Event, Sse},
    },
    routing::{get, post},
};
use base64::Engine;
use eyre as anyhow;
use futures_util::stream::Stream;
use http::{HeaderValue, StatusCode, header::ACCEPT, request::Parts};
use preferences_serde1::{AppInfo, Preferences};
use std::{collections::BTreeMap, convert::Infallible, io::Write, net::SocketAddr};
use tokio::sync::watch;
use tower_http::trace::TraceLayer;

use flo_core::{
    BuiEventData, DeviceState, EVENT_NAME, FLO_QUIT_EVENT_NAME, FloCommand, FloControllerConfig,
    FloEvent,
};

pub const APP_INFO: AppInfo = AppInfo {
    name: "flo",
    author: "AndrewStraw",
};
const COOKIE_SECRET_KEY: &str = "cookie-secret-base64";

/// Authenticated Strand Camera sessions, keyed by the camera name reported by
/// each server. The names are exposed as the first path component below
/// `/camera`.
pub type StrandCamSessions = BTreeMap<String, strand_bui_backend_session::HttpSession>;

#[cfg(not(any(feature = "bundle_files", feature = "serve_files")))]
compile_error!("Need cargo feature \"bundle_files\" or \"serve_files\"");

#[cfg(all(feature = "bundle_files", feature = "serve_files"))]
compile_error!(
    "Need exactly one of cargo features \"bundle_files\" or \"serve_files\", but both given."
);

/// header extractor for "Accept: text/event-stream" --------------------------
pub struct AcceptsEventStream;

impl<S> axum::extract::FromRequestParts<S> for AcceptsEventStream
where
    S: Send + Sync,
{
    type Rejection = (StatusCode, &'static str);
    async fn from_request_parts(p: &mut Parts, _: &S) -> Result<Self, Self::Rejection> {
        const ES: &[u8] = b"text/event-stream";
        if p.headers.get_all(ACCEPT).iter().any(|v| v.as_bytes() == ES) {
            Ok(AcceptsEventStream)
        } else {
            Err((
                StatusCode::BAD_REQUEST,
                "Bad request: It is required that you have an \
                HTTP Header \"Accept: text/event-stream\"",
            ))
        }
    }
}

async fn callback_handler(
    State(app_state): State<AppState>,
    session_key: axum_token_auth::SessionKey,
    Json(payload): Json<FloCommand>,
) -> impl IntoResponse {
    session_key.is_present();
    tracing::trace!("got HTTP message {:?}", payload);
    app_state
        .user_commands_tx
        .send(FloEvent::Command(payload, flo_core::CommandSource::Bui))
        .unwrap();
}

async fn events_handler(
    State(app_state): State<AppState>,
    session_key: axum_token_auth::SessionKey,
    _: AcceptsEventStream,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    session_key.is_present();
    use futures::stream;
    use tokio_stream::StreamExt;
    use tokio_stream::wrappers::WatchStream;
    let from_device_rx = app_state.from_device_rx.clone();
    let cfg = app_state.cfg.clone();
    let strand_cameras = app_state.strand_cam_proxy_info.clone();

    let stream1 = stream::iter(vec![
        BuiEventData::Config(cfg),
        BuiEventData::StrandCameras(strand_cameras),
    ]);
    let stream2 = WatchStream::from(from_device_rx).map(BuiEventData::DeviceState);

    let data_stream = stream1
        .chain(stream2)
        .map(|msg| Ok(Event::default().event(EVENT_NAME).json_data(msg).unwrap()));

    // When the server is shutting down it flips this watch channel to `true`.
    // Emit a dedicated SSE event so every connected browser (not only the one
    // that triggered the quit) shows the "FLO has quit" screen and stops
    // reconnecting. The initial `false` value is filtered out.
    let quit_stream = WatchStream::new(app_state.quit_rx.clone())
        .filter(|quitting| *quitting)
        .map(|_| Ok(Event::default().event(FLO_QUIT_EVENT_NAME).data("quit")));

    let stream = data_stream.merge(quit_stream);
    Sse::new(stream)
}

/// Returns the URLs (one per reachable network interface) at which this web UI
/// can be reached, each carrying a freshly minted short-lived access token. The
/// frontend turns these into QR codes for connecting another device.
async fn device_connect_urls_handler(
    State(app_state): State<AppState>,
    session_key: axum_token_auth::SessionKey,
) -> impl IntoResponse {
    session_key.is_present();
    match build_connect_urls(
        app_state.bound_addr,
        app_state.token_required,
        &app_state.persistent_secret,
    ) {
        Ok(urls) => Json(urls).into_response(),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            format!("failed to enumerate network interfaces: {e}"),
        )
            .into_response(),
    }
}

/// Forward one request through the authenticated HTTP session for a Strand
/// Camera.
///
/// This intentionally follows `braid-run`'s camera proxy: only the `Accept`
/// headers, HTTP method, and body need explicit forwarding. `HttpSession`
/// supplies the camera's session cookie and the JSON content type.
async fn cam_proxy_handler_inner(
    app_state: AppState,
    raw_cam_name: String,
    cam_path: String,
    req: axum::extract::Request,
) -> impl IntoResponse {
    tracing::debug!(
        raw_cam_name,
        cam_path,
        ?req,
        "proxying request to Strand Cam"
    );
    let accepts: Vec<HeaderValue> = req
        .headers()
        .get_all(http::header::ACCEPT)
        .iter()
        .cloned()
        .collect();

    let Some(mut session) = app_state.strand_cam_sessions.get(&raw_cam_name).cloned() else {
        let message = format!("Unknown camera {raw_cam_name:?}");
        tracing::warn!("{message}");
        return Err((StatusCode::NOT_FOUND, message));
    };

    session
        .req_accepts(&cam_path, &accepts, req.method().clone(), req.into_body())
        .await
        .map_err(|e| {
            let message = format!("Failed request to Strand Cam {raw_cam_name:?}: {e}");
            tracing::error!("{message}");
            (StatusCode::BAD_GATEWAY, message)
        })
}

async fn cam_proxy_handler_root(
    State(app_state): State<AppState>,
    session_key: axum_token_auth::SessionKey,
    Path(raw_cam_name): Path<String>,
    req: axum::extract::Request,
) -> impl IntoResponse {
    session_key.is_present();
    cam_proxy_handler_inner(app_state, raw_cam_name, String::new(), req).await
}

async fn cam_proxy_handler(
    State(app_state): State<AppState>,
    session_key: axum_token_auth::SessionKey,
    Path((raw_cam_name, cam_path)): Path<(String, String)>,
    req: axum::extract::Request,
) -> impl IntoResponse {
    session_key.is_present();
    cam_proxy_handler_inner(app_state, raw_cam_name, cam_path, req).await
}

/// Whether `uri`'s host is a loopback address (so unreachable from another
/// device). Used to decide which connection URLs are worth offering to scan.
fn is_loopback_uri(uri: &http::Uri) -> bool {
    matches!(uri.host(), Some("127.0.0.1") | Some("[::1]"))
}

async fn handle_auth_error(err: tower::BoxError) -> (StatusCode, &'static str) {
    match err.downcast::<axum_token_auth::ValidationErrors>() {
        Ok(err) => {
            tracing::error!(
                "Validation error(s): {:?}",
                err.errors().collect::<Vec<_>>()
            );
            (StatusCode::UNAUTHORIZED, "Request is not authorized")
        }
        Err(orig_err) => {
            tracing::error!("Unhandled internal error: {orig_err}");
            (StatusCode::INTERNAL_SERVER_ERROR, "internal server error")
        }
    }
}

#[derive(Clone, Debug)]
struct AppState {
    from_device_rx: watch::Receiver<DeviceState>,
    user_commands_tx: tokio::sync::broadcast::Sender<FloEvent>,
    cfg: FloControllerConfig,
    /// Flips to `true` once, just before shutdown, so the SSE handler can tell
    /// every connected browser the server is quitting.
    quit_rx: watch::Receiver<bool>,
    /// The address the HTTP server is bound to (possibly unspecified, e.g.
    /// `0.0.0.0`), used to enumerate device-connection URLs on demand.
    bound_addr: SocketAddr,
    /// Whether reaching this server requires an access token (true unless bound
    /// to loopback). Mirrors the policy used when the startup URL is minted.
    token_required: bool,
    /// The cookie/token secret, used to mint a fresh short-lived access token
    /// when a device-connection QR code is requested.
    persistent_secret: cookie::Key,
    strand_cam_sessions: StrandCamSessions,
    strand_cam_proxy_info: Vec<flo_core::StrandCamProxyInfo>,
}

fn display_qr_url(url: &str) {
    use qrcodegen::{QrCode, QrCodeEcc};
    use std::io::stdout;

    let qr = QrCode::encode_text(url, QrCodeEcc::Low).unwrap();

    let stdout = stdout();
    let mut stdout_handle = stdout.lock();
    writeln!(stdout_handle).expect("write failed");
    for y in 0..qr.size() {
        write!(stdout_handle, " ").expect("write failed");
        for x in 0..qr.size() {
            write!(
                stdout_handle,
                "{}",
                if qr.get_module(x, y) { "██" } else { "  " }
            )
            .expect("write failed");
        }
        writeln!(stdout_handle).expect("write failed");
    }
    writeln!(stdout_handle).expect("write failed");
}

/// Describe the bound server for [`strand_bui_backend_session::build_urls`],
/// minting a fresh short-lived access token when one is required (i.e. the
/// server is not bound to loopback).
///
/// The token is self-expiring and signed with the persistent secret; the auth
/// layer (same secret) validates it by signature and expiry, so nothing is
/// stored. A single token is embedded in every interface URL, exactly as the
/// startup URL is minted.
fn make_bui_server_info(
    bound_addr: SocketAddr,
    token_required: bool,
    persistent_secret: &cookie::Key,
) -> strand_bui_backend_session_types::BuiServerAddrInfo {
    use strand_bui_backend_session_types::{AccessToken, BuiServerAddrInfo};
    let token = if token_required {
        AccessToken::PreSharedToken(axum_token_auth::generate_token(
            persistent_secret,
            ACCESS_TOKEN_TTL,
        ))
    } else {
        AccessToken::NoToken
    };
    BuiServerAddrInfo::new(bound_addr, token)
}

/// Enumerate the interfaces the server is reachable on and build the
/// [`DeviceConnectUrls`] describing them, minting a fresh access token when one
/// is required.
///
/// [`DeviceConnectUrls`]: strand_bui_backend_session_types::DeviceConnectUrls
fn build_connect_urls(
    bound_addr: SocketAddr,
    token_required: bool,
    persistent_secret: &cookie::Key,
) -> std::io::Result<strand_bui_backend_session_types::DeviceConnectUrls> {
    let info = make_bui_server_info(bound_addr, token_required, persistent_secret);
    let uris = strand_bui_backend_session::build_urls(&info)?;
    let loopback_only = uris.iter().all(is_loopback_uri);
    let urls = uris.into_iter().map(|u| u.to_string()).collect();
    Ok(strand_bui_backend_session_types::DeviceConnectUrls {
        urls,
        loopback_only,
    })
}

/// Duration for which a freshly minted access token remains valid.
///
/// A token is only needed for a client's first request: a successful auth hands
/// back a session cookie that carries the session afterward. Keeping the token
/// short-lived bounds the window in which a token leaked via a URL (terminal
/// scrollback, log files, a photographed QR code) can be replayed.
pub const ACCESS_TOKEN_TTL: std::time::Duration = std::time::Duration::from_secs(30 * 60);

/// Load the persistent cookie/token secret, generating and saving a fresh one
/// if none exists.
///
/// This secret signs both the session cookies and the self-expiring access
/// tokens, so it must be loaded once and shared between [start_listener] (which
/// mints the token printed in the URL) and [main_loop] (which validates it).
/// Keeping the secret stable across restarts keeps already-issued browser
/// cookies valid through an upgrade.
pub fn load_persistent_secret() -> anyhow::Result<cookie::Key> {
    let persistent_secret_base64 = match String::load(&APP_INFO, COOKIE_SECRET_KEY) {
        Ok(secret_base64) => secret_base64,
        Err(_) => {
            tracing::debug!("No secret loaded from preferences file, generating new.");
            let persistent_secret = cookie::Key::generate();
            let persistent_secret_base64 =
                base64::engine::general_purpose::STANDARD.encode(persistent_secret.master());
            persistent_secret_base64.save(&APP_INFO, COOKIE_SECRET_KEY)?;
            persistent_secret_base64
        }
    };

    // The secret can forge any session cookie and mint any token, so ensure its
    // on-disk file is owner-only.
    harden_prefs_file(&APP_INFO, COOKIE_SECRET_KEY);

    let persistent_secret =
        base64::engine::general_purpose::STANDARD.decode(persistent_secret_base64)?;
    Ok(cookie::Key::try_from(persistent_secret.as_slice())?)
}

/// Restrict the on-disk `preferences_serde1` file backing `key` to owner-only
/// access (Unix mode 0600), warning if it was previously reachable by other
/// local users.
///
/// The cookie/token secret is effectively a master credential and the persisted
/// cookie jar holds live session cookies, so neither should be group- or
/// world-readable. `preferences_serde1` creates the file with the process umask
/// (typically 0644); this tightens it after the fact. No-op on non-Unix
/// platforms, whose permission model differs.
pub fn harden_prefs_file(app: &AppInfo, key: &str) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;

        // Mirror `preferences_serde1`'s path layout: <config>/<app>/<key>.prefs.json
        let Some(mut path) = preferences_serde1::prefs_base_dir() else {
            return;
        };
        path.push(app.name);
        path.push(key);
        let Some(mut name) = path.file_name().map(|n| n.to_os_string()) else {
            return;
        };
        name.push(".prefs.json");
        path.set_file_name(name);

        let Ok(md) = std::fs::metadata(&path) else {
            // No file on disk: nothing to do.
            return;
        };
        let mode = md.permissions().mode() & 0o777;
        if mode & 0o077 != 0 {
            tracing::warn!(
                "Restricting permissions on sensitive file {} from {mode:o} to 600 \
                 (it was accessible to other local users).",
                path.display(),
            );
        }
        if let Err(e) = std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600)) {
            tracing::warn!("Could not restrict permissions on {}: {e}", path.display());
        }
    }
    #[cfg(not(unix))]
    {
        let _ = (app, key);
    }
}

pub async fn start_listener(
    address_string: &str,
    persistent_secret: &cookie::Key,
) -> anyhow::Result<(
    tokio::net::TcpListener,
    Option<axum_token_auth::TokenConfig>,
)> {
    let socket_addr = std::net::ToSocketAddrs::to_socket_addrs(&address_string)?
        .next()
        .ok_or_else(|| anyhow::anyhow!("no address found for HTTP server"))?;

    let listener = tokio::net::TcpListener::bind(socket_addr).await?;
    let listener_local_addr = listener.local_addr()?;

    // With self-expiring signed tokens the `TokenConfig` carries no value; it
    // only signals that a token is required. The token string handed to the
    // user is minted separately below from the persistent secret.
    let token_config: Option<axum_token_auth::TokenConfig> =
        if !listener_local_addr.ip().is_loopback() {
            Some(axum_token_auth::TokenConfig::new("token"))
        } else {
            None
        };

    let info = make_bui_server_info(
        listener_local_addr,
        token_config.is_some(),
        persistent_secret,
    );
    for url in strand_bui_backend_session::build_urls(&info)? {
        tracing::info!("FLO listener at {listener_local_addr}, predicted URL: {url}");

        if !is_loopback_uri(&url) {
            println!("QR code for {url}");
            display_qr_url(&format!("{url}"));
        }
    }

    Ok((listener, token_config))
}

/// Parse a list of CIDR strings (e.g. `"100.64.0.0/10"`) into the network type
/// expected by [`main_loop`]'s `trusted_networks` argument, returning a
/// descriptive error for the first one that fails to parse.
pub fn parse_trusted_networks(nets: &[String]) -> anyhow::Result<Vec<axum_token_auth::CidrBlock>> {
    nets.iter()
        .map(|s| {
            s.parse::<axum_token_auth::CidrBlock>()
                .map_err(|e| anyhow::Error::msg(format!("invalid trusted network CIDR {s:?}: {e}")))
        })
        .collect()
}

#[expect(
    clippy::too_many_arguments,
    reason = "the web server wiring genuinely needs all of these inputs"
)]
pub async fn main_loop(
    tcp_listener: tokio::net::TcpListener,
    token_config: Option<axum_token_auth::TokenConfig>,
    persistent_secret: cookie::Key,
    trusted_networks: Vec<axum_token_auth::CidrBlock>,
    strand_cam_sessions: StrandCamSessions,
    strand_cam_proxy_info: Vec<flo_core::StrandCamProxyInfo>,
    from_device_rx: watch::Receiver<DeviceState>,
    cfg: FloControllerConfig,
    user_commands_tx: tokio::sync::broadcast::Sender<FloEvent>,
    quit_rx: watch::Receiver<bool>,
) -> Result<(), anyhow::Error> {
    let bound_addr = tcp_listener.local_addr()?;
    let token_required = token_config.is_some();
    let app_state = AppState {
        from_device_rx,
        user_commands_tx,
        cfg,
        quit_rx,
        bound_addr,
        token_required,
        persistent_secret: persistent_secret.clone(),
        strand_cam_sessions,
        strand_cam_proxy_info,
    };

    // `AuthConfig` is `#[non_exhaustive]`, so build it via `new` and set fields.
    let mut cfg = axum_token_auth::AuthConfig::new(persistent_secret);
    cfg.token_config = token_config;
    cfg.cookie_name = "braid-bui-session";
    // Sessions slide forward on use and survive up to 400 days of absence,
    // enforced server-side via the signed cookie. Existing cookies that
    // predate this field carry no embedded expiry and are treated as
    // non-expiring until renewed, so they stay valid across the upgrade.
    cfg.session_expires = Some(std::time::Duration::from_secs(60 * 60 * 24 * 400)); // 400 days
    // Clients on a trusted overlay network (e.g. Tailscale/WireGuard) are
    // accepted without a token; the overlay has already authenticated them.
    cfg.trusted_networks = trusted_networks;
    let auth_layer = cfg.into_layer();

    #[cfg(feature = "bundle_files")]
    static ASSETS_DIR: include_dir::Dir<'static> = include_dir::include_dir!("$DIST_DIR"); // Built by build script in `flo-bui`

    #[cfg(feature = "bundle_files")]
    let serve_dir = tower_serve_static::ServeDir::new(&ASSETS_DIR);

    #[cfg(feature = "serve_files")]
    let serve_dir = tower_http::services::fs::ServeDir::new(
        camino::Utf8PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("flo-bui")
            .join("pkg"),
    );

    // Create axum router.
    let router = axum::Router::new()
        .route(&format!("/{}", flo_core::EVENTS_PATH), get(events_handler))
        .route("/callback", post(callback_handler))
        .route("/device-connect-urls", get(device_connect_urls_handler))
        .route(
            &format!("/{}/{{encoded_cam_name}}/", flo_core::CAM_PROXY_PATH),
            axum::routing::method_routing::any(cam_proxy_handler_root),
        )
        .route(
            &format!(
                "/{}/{{encoded_cam_name}}/{{*path}}",
                flo_core::CAM_PROXY_PATH
            ),
            axum::routing::method_routing::any(cam_proxy_handler),
        )
        .fallback_service(serve_dir)
        .layer(
            tower::ServiceBuilder::new()
                .layer(TraceLayer::new_for_http())
                // Auth layer will produce an error if the request cannot be
                // authorized so we must handle that.
                .layer(axum::error_handling::HandleErrorLayer::new(
                    handle_auth_error,
                ))
                .layer(auth_layer),
        )
        .with_state(app_state);

    axum::serve(
        tcp_listener,
        router.into_make_service_with_connect_info::<SocketAddr>(),
    )
    .await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, RwLock};

    use axum::{Router, body::Body, response::IntoResponse, routing::any};
    use http::{Method, Request, StatusCode, header::ACCEPT};

    use super::{AppState, StrandCamSessions, build_connect_urls, cam_proxy_handler_inner};

    fn test_app_state(strand_cam_sessions: StrandCamSessions) -> AppState {
        let state =
            flo_core::DeviceState::new(flo_core::DeviceId::new([0; flo_core::DEVICE_ID_LEN]));
        let (_, from_device_rx) = tokio::sync::watch::channel(state);
        let (user_commands_tx, _) = tokio::sync::broadcast::channel(1);
        let (_, quit_rx) = tokio::sync::watch::channel(false);
        AppState {
            from_device_rx,
            user_commands_tx,
            cfg: Default::default(),
            quit_rx,
            bound_addr: "127.0.0.1:0".parse().unwrap(),
            token_required: false,
            persistent_secret: cookie::Key::generate(),
            strand_cam_sessions,
            strand_cam_proxy_info: Vec::new(),
        }
    }

    #[test]
    fn loopback_addr_has_no_token_and_is_loopback_only() {
        let secret = cookie::Key::generate();
        // A specified (non-wildcard) address yields exactly that one URL.
        let addr = "127.0.0.1:3440".parse().unwrap();
        let info = build_connect_urls(addr, false, &secret).unwrap();
        assert_eq!(info.urls, vec!["http://127.0.0.1:3440/".to_string()]);
        assert!(info.loopback_only);
    }

    #[test]
    fn routable_addr_carries_a_token_and_is_not_loopback() {
        let secret = cookie::Key::generate();
        let addr = "192.168.1.5:3440".parse().unwrap();
        let info = build_connect_urls(addr, true, &secret).unwrap();
        assert!(!info.loopback_only);
        assert_eq!(info.urls.len(), 1);
        let url = &info.urls[0];
        assert!(
            url.starts_with("http://192.168.1.5:3440/?token="),
            "unexpected url: {url}"
        );
        // A token must actually be present after `token=`.
        assert!(url.len() > "http://192.168.1.5:3440/?token=".len());
    }

    #[test]
    fn camera_proxy_forwards_path_method_accept_and_body() {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        runtime.block_on(async {
            async fn echo_request(req: Request<Body>) -> String {
                let method = req.method().clone();
                let path = req.uri().path().to_string();
                let accept = req
                    .headers()
                    .get(ACCEPT)
                    .and_then(|value| value.to_str().ok())
                    .unwrap_or_default()
                    .to_string();
                let body = axum::body::to_bytes(req.into_body(), usize::MAX)
                    .await
                    .unwrap();
                format!(
                    "{method} {path} {accept} {}",
                    String::from_utf8_lossy(&body)
                )
            }

            let upstream_listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let upstream_addr = upstream_listener.local_addr().unwrap();
            let upstream_task = tokio::spawn(async move {
                axum::serve(
                    upstream_listener,
                    Router::new()
                        .route("/", any(echo_request))
                        .route("/{*path}", any(echo_request)),
                )
                .await
                .unwrap();
            });

            let info = strand_bui_backend_session_types::BuiServerAddrInfo::new(
                upstream_addr,
                strand_bui_backend_session_types::AccessToken::NoToken,
            );
            let jar = Arc::new(RwLock::new(cookie_store::CookieStore::new(None)));
            let session = strand_bui_backend_session::create_session(&info, jar)
                .await
                .unwrap();
            let mut sessions = StrandCamSessions::new();
            sessions.insert("camera one".to_string(), session);

            let request = Request::builder()
                .method(Method::POST)
                .uri("/ignored-by-inner-handler")
                .header(ACCEPT, "text/event-stream")
                .body(Body::from("hello camera"))
                .unwrap();
            let response = cam_proxy_handler_inner(
                test_app_state(sessions),
                "camera one".to_string(),
                "nested/path".to_string(),
                request,
            )
            .await
            .into_response();

            assert_eq!(response.status(), StatusCode::OK);
            let body = axum::body::to_bytes(response.into_body(), usize::MAX)
                .await
                .unwrap();
            assert_eq!(
                body.as_ref(),
                b"POST /nested/path text/event-stream hello camera"
            );

            upstream_task.abort();
        });
    }

    #[test]
    fn camera_proxy_returns_not_found_for_unknown_camera() {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        runtime.block_on(async {
            let request = Request::builder().body(Body::empty()).unwrap();
            let response = cam_proxy_handler_inner(
                test_app_state(StrandCamSessions::new()),
                "missing camera".to_string(),
                String::new(),
                request,
            )
            .await
            .into_response();
            assert_eq!(response.status(), StatusCode::NOT_FOUND);
        });
    }
}
