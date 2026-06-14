use axum::{
    Json,
    extract::State,
    response::{
        IntoResponse,
        sse::{Event, Sse},
    },
    routing::{get, post},
};
use base64::Engine;
use eyre as anyhow;
use futures_util::stream::Stream;
use http::{StatusCode, header::ACCEPT, request::Parts};
use preferences_serde1::{AppInfo, Preferences};
use std::{
    convert::Infallible,
    io::Write,
    net::{IpAddr, SocketAddr},
};
use tokio::sync::watch;
use tower_http::trace::TraceLayer;

use flo_core::{BuiEventData, DeviceState, EVENT_NAME, FloCommand, FloControllerConfig, FloEvent};

pub const APP_INFO: AppInfo = AppInfo {
    name: "flo",
    author: "AndrewStraw",
};
const COOKIE_SECRET_KEY: &str = "cookie-secret-base64";

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
    // use futures::stream::StreamExt;
    use futures::stream::{self, StreamExt};
    let from_device_rx = app_state.from_device_rx.clone();
    let cfg = app_state.cfg.clone();

    let stream1 = stream::iter(vec![BuiEventData::Config(cfg)]);
    let stream2 =
        tokio_stream::wrappers::WatchStream::from(from_device_rx).map(BuiEventData::DeviceState);

    let stream = stream1.chain(stream2);

    let stream = stream.map(|msg| Ok(Event::default().event(EVENT_NAME).json_data(msg).unwrap()));
    Sse::new(stream)
}

fn expand_unspecified_addr(addr: &SocketAddr) -> std::io::Result<Vec<SocketAddr>> {
    if addr.ip().is_unspecified() {
        Ok(expand_unspecified_ip(addr.ip())?
            .into_iter()
            .map(|ip| SocketAddr::new(ip, addr.port()))
            .collect())
    } else {
        Ok(vec![*addr])
    }
}

fn expand_unspecified_ip(ip: IpAddr) -> std::io::Result<Vec<IpAddr>> {
    if ip.is_unspecified() {
        // Get all interfaces if IP is unspecified.
        Ok(if_addrs::get_if_addrs()?
            .iter()
            .filter_map(|x| {
                let this_ip = x.addr.ip();
                // Take only IP addresses from correct family.
                if ip.is_ipv4() == this_ip.is_ipv4() {
                    Some(this_ip)
                } else {
                    None
                }
            })
            .collect())
    } else {
        Ok(vec![ip])
    }
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
    let all_addrs = expand_unspecified_addr(&listener_local_addr)?;

    // With self-expiring signed tokens the `TokenConfig` carries no value; it
    // only signals that a token is required. The token string handed to the
    // user is minted separately below from the persistent secret.
    let token_config: Option<axum_token_auth::TokenConfig> =
        if !listener_local_addr.ip().is_loopback() {
            Some(axum_token_auth::TokenConfig::new("token"))
        } else {
            None
        };

    for addr in all_addrs.iter() {
        let url = {
            let query = if token_config.is_some() {
                // Mint a short-lived, self-expiring token signed with the
                // persistent secret. The auth layer (same secret) validates it
                // by signature and expiry; nothing is stored.
                let token = axum_token_auth::generate_token(&persistent_secret, ACCESS_TOKEN_TTL);
                format!("token={token}")
            } else {
                "".to_string()
            };
            http::uri::Builder::new()
                .scheme("http")
                .authority(format!("{}:{}", addr.ip(), addr.port()))
                .path_and_query(format!("/?{query}"))
                .build()
                .unwrap()
        };
        tracing::info!("FLO listener at {listener_local_addr}, predicted URL: {url}");

        if !addr.ip().is_loopback() {
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

pub async fn main_loop(
    tcp_listener: tokio::net::TcpListener,
    token_config: Option<axum_token_auth::TokenConfig>,
    persistent_secret: cookie::Key,
    trusted_networks: Vec<axum_token_auth::CidrBlock>,
    from_device_rx: watch::Receiver<DeviceState>,
    cfg: FloControllerConfig,
    user_commands_tx: tokio::sync::broadcast::Sender<FloEvent>,
) -> Result<(), anyhow::Error> {
    let app_state = AppState {
        from_device_rx,
        user_commands_tx,
        cfg,
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
