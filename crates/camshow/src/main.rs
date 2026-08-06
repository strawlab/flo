//! `camshow` — webcam capture with an OSD overlay pushed from `flo`.
//!
//! One process, one binary; which outputs run is decided entirely at runtime
//! by CLI arguments. It always:
//! - Captures frames from a USB webcam.
//! - Overlays an OSD canvas pushed from `flo` over a localhost TCP connection.
//! - Records the clean (no-OSD) frames + OSD log to disk on flo's command. The
//!   output directory and codec configuration arrive in the recording-start
//!   message itself, so camshow holds no opinion of its own about either.
//!
//! What is *displayed* is selectable: the webcam with its OSD, or frames from a
//! strand-cam tracking camera relayed over a second TCP connection
//! (`--video-listen`), which is how the drone operator switches the live view
//! mid-flight. That selection never reaches the recording.
//!
//! It then hands each displayed frame to whichever outputs were enabled:
//! - `--gui`: a local egui video display.
//! - `--rtp-dest`: an H.264/RTP/UDP stream, e.g. for an OpenIPC-style
//!   groundstation.
//!
//! Both can run at the same time; with neither, camshow is a recorder that
//! waits for flo to tell it to write a file.
//!
//! Robustness goal: this process must keep capturing (displaying, recording,
//! streaming) even if every other piece of the system (flo, motors, mavlink,
//! tracking cameras) is dead or restarting. Failures in any non-camera
//! subsystem are logged and contained.

use std::net::SocketAddr;

use clap::Parser;
use color_eyre::eyre::Result;
use flo_core::DisplaySource;
use tokio::sync::{mpsc, watch};
use tracing::{error, info, warn};

mod bootstrap;
mod camera;
mod gui;
mod rtp;
mod server;
mod sink;
mod source;
mod state;
mod video_link;

use camera::CameraTask;
use rtp::{EncoderChoice, RtpSinkConfig, RtpStreamParams};
use sink::SinkConfig;
use state::OsdSnapshot;

/// Used for the log file name and to name the threads this spawns (for
/// `ps`/thread-dump readability).
const APP_NAME: &str = "camshow";

/// How many raw frames may sit between the capture loop and the RTP encoder
/// before frames get dropped rather than stalling capture.
const RTP_QUEUE_SIZE: usize = 8;

/// CLI spelling of [`DisplaySource`], which lives in flo-core and so cannot
/// derive clap's traits itself.
#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum DisplaySourceArg {
    Webcam,
    StrandCamMain,
    StrandCamSecondary,
}

impl From<DisplaySourceArg> for DisplaySource {
    fn from(arg: DisplaySourceArg) -> Self {
        match arg {
            DisplaySourceArg::Webcam => Self::Webcam,
            DisplaySourceArg::StrandCamMain => Self::StrandCamMain,
            DisplaySourceArg::StrandCamSecondary => Self::StrandCamSecondary,
        }
    }
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// TCP address to listen on for flo connections.
    #[arg(long, default_value = camshow_protocol::DEFAULT_CAMSHOW_ADDR)]
    listen: String,

    /// TCP address to listen on for the video link, over which flo relays
    /// tracking-camera frames for the non-webcam display sources.
    #[arg(long, default_value = camshow_protocol::video::DEFAULT_CAMSHOW_VIDEO_ADDR)]
    video_listen: String,

    /// What the display and stream show. The recording is always the clean
    /// webcam whatever this is set to. Non-webcam sources need frames relayed
    /// over the video link; without them the webcam is shown instead.
    #[arg(long, value_enum, default_value_t = DisplaySourceArg::Webcam)]
    display_source: DisplaySourceArg,

    /// Preferred webcam human name. If unset, the first available camera is
    /// used.
    #[arg(long)]
    fpv_cam: Option<String>,

    /// Overlay a fixed test-pattern OSD (every glyph kind). Shown whenever
    /// flo isn't pushing a live canvas; useful for verifying the rendering
    /// pipeline without the rest of the stack running.
    #[arg(long)]
    test_pattern: bool,

    /// Directory for log files. Defaults to the home directory.
    #[arg(long)]
    log_dir: Option<camino::Utf8PathBuf>,

    /// Show the video in a local window (fullscreen unless `--windowed`).
    #[arg(long)]
    gui: bool,

    /// Show the GUI in a window instead of fullscreen.
    #[arg(long, requires = "gui")]
    windowed: bool,

    /// Stream H.264/RTP/UDP to this host:port (e.g. an OpenIPC groundstation
    /// receiver). Enables the RTP output.
    #[arg(long)]
    rtp_dest: Option<SocketAddr>,

    /// RTP H.264 encoder backend.
    #[arg(long, value_enum, default_value_t = EncoderChoice::Ffmpeg, requires = "rtp_dest")]
    rtp_encoder: EncoderChoice,

    /// Initial target bitrate for the RTP stream, in kbps.
    #[arg(long, default_value_t = 4000, requires = "rtp_dest")]
    rtp_bitrate_kbps: u32,

    /// Nominal frame rate, used by the ffmpeg backend's timestamp/VBV
    /// arithmetic. Actual capture cadence need not match exactly.
    #[arg(long, default_value_t = 30.0, requires = "rtp_dest")]
    rtp_fps: f32,

    /// Path MTU in bytes for the RTP/UDP datagrams.
    #[arg(long, default_value_t = 1400, requires = "rtp_dest")]
    rtp_mtu: usize,

    /// Interval, in frames, between forced RTP keyframes (IDR).
    #[arg(long, default_value_t = 30, requires = "rtp_dest")]
    rtp_idr_interval: u32,

    /// Tee the raw Annex-B elementary stream sent to the RTP encoder to this
    /// file, for offline `ffprobe`/decoder verification.
    #[arg(long, requires = "rtp_dest")]
    rtp_dump_annexb: Option<std::path::PathBuf>,
}

fn main() -> Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        // SAFETY: We ensure that this only happens in single-threaded code
        // because this is immediately at the start of main() and no other
        // threads have started.
        unsafe { std::env::set_var("RUST_LOG", "info") };
    }

    color_eyre::install()?;

    let cli = Cli::parse();
    bootstrap::init(cli.log_dir.clone())?;

    if !cli.gui && cli.rtp_dest.is_none() {
        warn!(
            "neither --gui nor --rtp-dest given; capturing only to record on flo's command \
             (nothing will be displayed or streamed)"
        );
    }

    let (osd_tx, osd_rx) = watch::channel::<Option<OsdSnapshot>>(None);
    let (recording_tx, recording_rx) = mpsc::unbounded_channel();
    let (rtp_bitrate_tx, rtp_bitrate_rx) = mpsc::unbounded_channel();
    // An unbounded channel rather than a oneshot because shutdown can be
    // requested from more than one place: the signal handler in the tokio
    // runtime below, and (in GUI mode) this thread once eframe returns. The
    // camera thread treats a send *or* all senders being dropped as "shut
    // down".
    let (shutdown_tx, shutdown_rx) = mpsc::unbounded_channel::<()>();

    let rtp_sink_cfg = cli.rtp_dest.map(|dest| RtpSinkConfig {
        params: RtpStreamParams {
            dest,
            encoder: cli.rtp_encoder,
            fps: cli.rtp_fps,
            idr_interval_frames: cli.rtp_idr_interval,
            mtu: cli.rtp_mtu,
            queue_size: RTP_QUEUE_SIZE,
            dump_annexb: cli.rtp_dump_annexb.clone(),
        },
        initial_bitrate_kbps: cli.rtp_bitrate_kbps,
    });

    let listen_addr = cli.listen.clone();
    let video_listen_addr = cli.video_listen.clone();
    let windowed = cli.windowed;
    let test_pattern = cli.test_pattern.then(osd_utils::test_pattern);
    let display_source = DisplaySource::from(cli.display_source);
    if display_source != DisplaySource::Webcam {
        info!("displaying {display_source:?} rather than the webcam (the recording is unaffected)");
    }
    // Shared between the video-link server on the tokio runtime and the
    // capture thread that displays what it receives.
    let relayed = video_link::LatestRelayedFrame::new();
    // The CLI value is the starting point; flo overrides it whenever the
    // operator switches.
    let (display_source_tx, display_source_rx) = watch::channel(display_source);
    // This carries local GUI requests back to FLO. It is separate from
    // `display_source_tx`, which carries FLO's authoritative state to the
    // camera thread.
    let (gui_display_source_tx, mut gui_display_source_rx) = watch::channel(display_source);
    gui_display_source_rx.mark_unchanged();
    // The GUI half of the wiring only exists in GUI mode: the sink side goes
    // to the camera thread, the handle side to eframe on this thread.
    let (gui_sink_cfg, gui_handles) = cli
        .gui
        .then(|| gui::channels(gui_display_source_tx.clone()))
        .unzip();

    let relayed_for_camera = relayed.clone();
    let camera_handle = std::thread::Builder::new()
        .name(format!("{APP_NAME}-camera"))
        .spawn(move || {
            camera::run(CameraTask {
                fpv_cam_human_name: cli.fpv_cam,
                test_pattern,
                sinks: SinkConfig {
                    gui: gui_sink_cfg,
                    rtp: rtp_sink_cfg,
                },
                osd_rx,
                recording_rx,
                rtp_bitrate_rx,
                display_source_rx,
                relayed: relayed_for_camera,
                shutdown_rx,
            })
        })?;

    let server = server::Server {
        listen_addr,
        video_listen_addr,
        relayed,
        osd_tx,
        recording_tx,
        rtp_bitrate_tx,
        display_source_tx,
        gui_display_source_rx,
    };

    let run_result = match gui_handles {
        // eframe insists on the main thread, so the tokio runtime gets its
        // own. We never join that thread: it may be parked in `accept`, and
        // the process is exiting anyway.
        Some(handles) => {
            let tokio_shutdown_tx = shutdown_tx.clone();
            std::thread::Builder::new()
                .name(format!("{APP_NAME}-tokio"))
                .spawn(move || serve_until_shutdown(server, tokio_shutdown_tx))?;
            gui::run(handles, windowed)
        }
        // Headless: this thread has nothing else to do, so it runs the
        // runtime directly and returns once flo's server ends or a signal
        // arrives.
        None => serve_until_shutdown(server, shutdown_tx.clone()),
    };

    // Tell the camera thread (and, through it, the GUI viewport) to stop.
    let _ = shutdown_tx.send(());

    match camera_handle.join() {
        Ok(Ok(())) => {}
        Ok(Err(e)) => error!("camera thread error: {e:?}"),
        Err(_) => error!("camera thread panicked"),
    }
    run_result
}

/// Runs the TCP server on a fresh tokio runtime until it ends or a shutdown
/// signal arrives, then requests process shutdown.
///
/// [`server::Server::run`] only returns on a bind failure — accept errors are
/// retried — so an error here means flo will never be able to connect. That is
/// worth shutting down for (a supervisor can restart us, or the operator can
/// fix the address) rather than capturing forever with no way to be told to
/// record.
fn serve_until_shutdown(
    server: server::Server,
    shutdown_tx: mpsc::UnboundedSender<()>,
) -> Result<()> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    rt.block_on(async move {
        tokio::select! {
            result = server.run() => {
                if let Err(e) = result {
                    error!("TCP server task ended: {e:?}");
                }
            }
            () = shutdown_signal() => {
                info!("shutdown signal received");
            }
        }
    });
    let _ = shutdown_tx.send(());
    Ok(())
}

/// Resolves on Ctrl-C or, on Unix, SIGTERM (as sent by systemd on stop).
async fn shutdown_signal() {
    let ctrl_c = async {
        let _ = tokio::signal::ctrl_c().await;
    };

    #[cfg(unix)]
    let terminate = async {
        use tokio::signal::unix::{SignalKind, signal};
        let mut sigterm = signal(SignalKind::terminate()).expect("install SIGTERM handler");
        sigterm.recv().await;
    };
    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        () = ctrl_c => {},
        () = terminate => {},
    }
}
