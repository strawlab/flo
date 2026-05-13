//! `camshow` — webcam viewer with OSD overlay.
//!
//! A deliberately small process that:
//! - Captures frames from a USB webcam.
//! - Renders them in an egui window.
//! - Overlays an OSD canvas pushed from `flo` over a localhost TCP connection.
//! - Records the raw frames + OSD log to disk on flo's command. The output
//!   directory and codec configuration arrive in the recording-start
//!   message itself, so camshow holds no opinion of its own about either.
//!
//! Robustness goal: this process must keep displaying the camera even if
//! every other piece of the system (flo, motors, mavlink, tracking cameras)
//! is dead or restarting. Failures in any non-camera subsystem are logged
//! and contained.

use clap::Parser;
use color_eyre::eyre::{Result, WrapErr};
use tokio::{net::TcpListener, sync::watch};

mod camera;
mod gui;
mod server;
mod state;

use camera::CameraTask;
use state::{DisplayFrame, OsdSnapshot};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// TCP address to listen on for flo connections.
    #[arg(long, default_value = camshow_protocol::DEFAULT_CAMSHOW_ADDR)]
    listen: String,

    /// Show in a window instead of fullscreen.
    #[arg(long)]
    windowed: bool,

    /// Preferred webcam human name. If unset, the first available camera
    /// is used.
    #[arg(long)]
    fpv_cam: Option<String>,

    /// Display a fixed test-pattern OSD (every glyph kind). Shown whenever
    /// flo isn't pushing a live canvas; useful for verifying the rendering
    /// pipeline without the rest of the stack running.
    #[arg(long)]
    test_pattern: bool,

    /// Directory for log files. Defaults to the home directory.
    #[arg(long)]
    log_dir: Option<camino::Utf8PathBuf>,
}

fn main() -> Result<()> {
    color_eyre::install()?;

    if std::env::var_os("RUST_LOG").is_none() {
        // SAFETY: set in single-threaded code at startup, before any other
        // thread that might read RUST_LOG has been spawned.
        unsafe { std::env::set_var("RUST_LOG", "camshow=info,info") };
    }

    let cli = Cli::parse();

    let log_dir = if let Some(d) = cli.log_dir.as_ref() {
        d.clone()
    } else {
        let home =
            home::home_dir().ok_or_else(|| eyre::eyre!("cannot determine home directory"))?;
        camino::Utf8PathBuf::from_path_buf(home)
            .map_err(|p| eyre::eyre!("home directory is not valid UTF-8: {}", p.display()))?
    };
    std::fs::create_dir_all(&log_dir)
        .with_context(|| format!("creating log directory {log_dir}"))?;

    init_tracing(&log_dir)?;

    let (display_tx, display_rx) = watch::channel::<Option<DisplayFrame>>(None);
    let (osd_tx, osd_rx) = watch::channel::<Option<OsdSnapshot>>(None);
    let (recording_tx, recording_rx) = tokio::sync::mpsc::unbounded_channel();
    let (egui_ctx_tx, egui_ctx_rx) = std::sync::mpsc::channel();
    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel();

    // Tokio runtime runs the TCP server. eframe owns the main thread.
    let listen_addr = cli.listen.clone();
    let tokio_handle = std::thread::Builder::new()
        .name("camshow-tokio".into())
        .spawn(move || -> Result<()> {
            let rt = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()?;
            rt.block_on(async move {
                let listener = TcpListener::bind(&listen_addr)
                    .await
                    .with_context(|| format!("binding TCP listener at {listen_addr}"))?;
                server::run(listener, osd_tx, recording_tx).await
            })
        })?;

    let test_pattern = cli.test_pattern.then(osd_utils::test_pattern);
    let camera_handle = std::thread::Builder::new()
        .name("camshow-camera".into())
        .spawn(move || {
            camera::run(CameraTask {
                fpv_cam_human_name: cli.fpv_cam,
                test_pattern,
                egui_ctx_rx,
                display_tx,
                osd_rx,
                recording_rx,
                shutdown_rx,
            })
        })?;

    let windowed = cli.windowed;
    let eframe_opts = eframe::NativeOptions {
        window_builder: Some(Box::new(move |mut vb| {
            vb.fullscreen = Some(!windowed);
            vb.decorations = Some(true);
            vb
        })),
        ..Default::default()
    };

    let run_result = eframe::run_native(
        env!("CARGO_PKG_NAME"),
        eframe_opts,
        Box::new(move |_cc| Ok(Box::new(gui::CamshowApp::new(display_rx, egui_ctx_tx)))),
    );

    // Tell helper threads to shut down. The camera thread exits when its
    // sender side (`display_tx`) is dropped along with the GUI's `display_rx`.
    let _ = shutdown_tx.send(());

    if let Err(e) = run_result {
        return Err(eyre::eyre!("eframe failed: {e}"));
    }

    match camera_handle.join() {
        Ok(Ok(())) => {}
        Ok(Err(e)) => tracing::error!("camera thread error: {e:?}"),
        Err(_) => tracing::error!("camera thread panicked"),
    }
    // The tokio thread's outer loop never returns under normal use, so we
    // don't block on it here.
    drop(tokio_handle);
    Ok(())
}

fn init_tracing(log_dir: &camino::Utf8Path) -> Result<()> {
    use std::sync::Mutex;
    use time::{UtcOffset, format_description::well_known::Iso8601};
    use tracing_subscriber::{
        fmt::{self, time::OffsetTime},
        layer::SubscriberExt,
    };

    let log_file_name = chrono::Local::now()
        .format(".camshow-%Y%m%d_%H%M%S.%f.log")
        .to_string();
    let full_log_path = log_dir.join(&log_file_name);

    let timer = OffsetTime::new(
        UtcOffset::from_whole_seconds(chrono::Local::now().offset().local_minus_utc())?,
        Iso8601::DEFAULT,
    );

    #[cfg(target_os = "windows")]
    ansi_term::enable_ansi_support()
        .map_err(|code| eyre::eyre!("Failed setting windows ansi: {code}"))?;

    let file = std::fs::File::create(&full_log_path)
        .with_context(|| format!("creating log file {full_log_path}"))?;
    let file_writer = Mutex::new(file);
    let file_layer = fmt::layer()
        .with_timer(timer.clone())
        .with_writer(file_writer)
        .with_ansi(false)
        .with_file(true)
        .with_line_number(true);
    let console_layer = fmt::layer()
        .with_timer(timer)
        .with_file(true)
        .with_line_number(true);
    let collector = tracing_subscriber::registry()
        .with(file_layer)
        .with(console_layer)
        .with(tracing_subscriber::filter::EnvFilter::from_default_env());
    tracing::subscriber::set_global_default(collector)?;
    std::panic::set_hook(Box::new(tracing_panic::panic_hook));
    tracing::info!("logging to {full_log_path}");
    Ok(())
}
