use axum::{
    extract::State,
    response::{
        sse::{Event, Sse},
        IntoResponse,
    },
    routing::{get, post},
    Json,
};
use eyre as anyhow;
use futures_util::stream::Stream;
use http::{header::ACCEPT, request::Parts, StatusCode};
use preferences_serde1::{AppInfo, Preferences};
use std::{
    convert::Infallible,
    io::Write,
    net::{IpAddr, SocketAddr},
};
use tokio::sync::watch;
use tower_http::trace::TraceLayer;

use flo_core::{BuiEventData, DeviceState, FloCommand, FloControllerConfig, FloEvent, EVENT_NAME};

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

#[axum::async_trait]
impl<S> axum::extract::FromRequestParts<S> for AcceptsEventStream {
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

pub async fn start_listener(
    address_string: &str,
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

    let token_config: Option<axum_token_auth::TokenConfig> =
        if !listener_local_addr.ip().is_loopback() {
            Some(axum_token_auth::TokenConfig::new_token("token"))
        } else {
            None
        };

    for addr in all_addrs.iter() {
        let url = {
            let query = match &token_config {
                None => "".to_string(),
                Some(tok) => format!("token={}", tok.value),
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

pub async fn main_loop(
    tcp_listener: tokio::net::TcpListener,
    token_config: Option<axum_token_auth::TokenConfig>,
    from_device_rx: watch::Receiver<DeviceState>,
    cfg: FloControllerConfig,
    user_commands_tx: tokio::sync::broadcast::Sender<FloEvent>,
) -> Result<(), anyhow::Error> {
    let app_state = AppState {
        from_device_rx,
        user_commands_tx,
        cfg,
    };

    let persistent_secret_base64 = match String::load(&APP_INFO, COOKIE_SECRET_KEY) {
        Ok(secret_base64) => secret_base64,
        Err(_) => {
            tracing::debug!("No secret loaded from preferences file, generating new.");
            let persistent_secret = cookie::Key::generate();
            let persistent_secret_base64 = base64::encode(persistent_secret.master());
            persistent_secret_base64.save(&APP_INFO, COOKIE_SECRET_KEY)?;
            persistent_secret_base64
        }
    };

    let persistent_secret = base64::decode(persistent_secret_base64)?;
    let persistent_secret = cookie::Key::try_from(persistent_secret.as_slice())?;

    let cfg = axum_token_auth::AuthConfig {
        token_config,
        persistent_secret,
        cookie_name: "braid-bui-session",
        cookie_expires: Some(std::time::Duration::from_secs(60 * 60 * 24 * 400)), // 400 days
    };
    let auth_layer = cfg.into_layer();

    #[cfg(feature = "bundle_files")]
    static ASSETS_DIR: include_dir::Dir<'static> =
        include_dir::include_dir!("$CARGO_MANIFEST_DIR/../flo-bui/pkg");

    #[cfg(feature = "bundle_files")]
    let serve_dir = tower_serve_static::ServeDir::new(&ASSETS_DIR);

    #[cfg(feature = "serve_files")]
    let serve_dir = tower_http::services::fs::ServeDir::new(
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("flo-bui")
            .join("pkg"),
    );

    // Create axum router.
    let router = axum::Router::new()
        .route(&format!("/{}", flo_core::EVENTS_PATH), get(events_handler))
        .route("/callback", post(callback_handler))
        .nest_service("/", serve_dir)
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
