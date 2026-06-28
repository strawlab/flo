// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! CLI tool to replay recorded centroid data from a `.floz` file as UDP packets
//! to a running FLO controller.
//!
//! Live FLO receives centroids over UDP from Strand Camera's `imops` module
//! (see `on_image_centroid` in the `flo` crate). The wire payload is a
//! [`flo_core::UdpMsg`] encoded as CBOR. This tool reads the `centroid.csv`
//! table already saved inside a `.floz` archive and re-emits each row as the
//! exact same UDP message, reproducing the original arrival cadence. That lets a
//! developer exercise the full controller pipeline (Kalman filter, tracking
//! state machine, `.floz` recording, and the web UI) with no camera or motor
//! hardware attached: start `flo` with a config that omits the `strand_cam_*`
//! blocks and no motor flags, then point this tool at its `--udp-addr`.
//!
//! Rows where nothing was detected (`mu00 == 0.0`) are replayed too, so the
//! controller's "stale camera" / "lost target" logic is exercised, not just the
//! happy path.

use std::net::UdpSocket;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use chrono::{DateTime, Local};
use clap::Parser;
use color_eyre::eyre::{Context, Result, bail};

use flo_core::{MomentCentroid, StampedMomentCentroid, UdpMsg};

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Opt {
    /// Input .floz filename (a ZIP archive or an extracted directory).
    input: PathBuf,

    /// UDP address of the running FLO controller (its `--udp-addr`).
    #[arg(long, default_value = "127.0.0.1:8080")]
    target: String,

    /// Playback speed multiplier. 2.0 plays twice as fast; 0.5 half speed.
    #[arg(long, default_value_t = 1.0)]
    speed: f64,

    /// Repeat the recording indefinitely (until interrupted with Ctrl-C).
    #[arg(long)]
    r#loop: bool,

    /// Skip this many seconds into the recording before starting playback.
    #[arg(long, default_value_t = 0.0)]
    start: f64,

    /// Stop after replaying this many seconds of the recording (measured in
    /// recording time, before the `--speed` multiplier). Plays to the end if
    /// unset.
    #[arg(long)]
    duration: Option<f64>,

    /// Relabel the first camera seen in the recording to this name. Use when the
    /// controller config expects a specific primary camera name.
    #[arg(long)]
    primary_cam: Option<String>,

    /// Relabel the second camera seen in the recording to this name (the stereo
    /// camera). Use when the controller config expects a specific secondary
    /// camera name.
    #[arg(long)]
    secondary_cam: Option<String>,
}

fn init_tracing() -> Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        let envstr = format!("{}=info,info", env!("CARGO_PKG_NAME")).replace('-', "_");
        // SAFETY: called once at startup before any threads are spawned.
        unsafe {
            std::env::set_var("RUST_LOG", envstr);
        }
    }
    use tracing_subscriber::{fmt, layer::SubscriberExt};
    let console_layer = fmt::layer().with_file(true).with_line_number(true);
    let collector = tracing_subscriber::registry()
        .with(console_layer)
        .with(tracing_subscriber::filter::EnvFilter::from_default_env());
    tracing::subscriber::set_global_default(collector)?;
    std::panic::set_hook(Box::new(tracing_panic::panic_hook));
    Ok(())
}

/// Convert a recorded [`StampedMomentCentroid`] back into the [`MomentCentroid`]
/// that originally arrived over UDP, applying any camera-name relabeling.
fn to_moment_centroid(row: &StampedMomentCentroid, new_cam_name: &str) -> MomentCentroid {
    MomentCentroid {
        schema_version: row.schema_version,
        framenumber: row.framenumber,
        timestamp_source: row.timestamp_source.clone(),
        timestamp: row.timestamp,
        mu00: row.mu00,
        mu01: row.mu01,
        mu10: row.mu10,
        center_x: row.center_x,
        center_y: row.center_y,
        cam_name: new_cam_name.into(),
    }
}

fn main() -> Result<()> {
    color_eyre::install()?;
    init_tracing()?;
    let opt = Opt::parse();

    if opt.speed <= 0.0 {
        bail!("--speed must be positive, got {}", opt.speed);
    }

    let mut floz = floz_parser::floz_parse_path(&opt.input)
        .with_context(|| format!("Parsing file {}", opt.input.display()))?;
    let mut centroids = std::mem::take(&mut floz.centroids);

    if centroids.is_empty() {
        bail!("No centroids found in {}", opt.input.display());
    }

    // Replay in receive order. The recorder writes rows as they arrive, but sort
    // defensively so the inter-row timing is always non-negative.
    centroids.sort_by_key(|c| c.received_timestamp);

    // Build the camera-name relabeling map from first-seen order: the first
    // distinct camera becomes `--primary-cam`, the second `--secondary-cam`.
    // Cameras without an override keep their recorded name.
    let cam_remap = build_cam_remap(&centroids, &opt);
    for (orig, new) in &cam_remap {
        if orig != new {
            tracing::info!("Relabeling camera {orig:?} -> {new:?}");
        }
    }

    // Window the recording to [start, start+duration) in recording time.
    let base: DateTime<Local> = centroids[0].received_timestamp;
    let start_off = Duration::from_secs_f64(opt.start);
    let end_off = opt.duration.map(|d| start_off + Duration::from_secs_f64(d));
    let scheduled: Vec<(Duration, MomentCentroid)> = centroids
        .iter()
        .filter_map(|row| {
            let off = (row.received_timestamp - base).to_std().ok()?;
            if off < start_off {
                return None;
            }
            if let Some(end) = end_off
                && off >= end
            {
                return None;
            }
            let cam = cam_remap
                .iter()
                .find(|(orig, _)| orig.as_str() == row.cam_name.as_str())
                .map(|(_, new)| new.as_str())
                .unwrap_or_else(|| row.cam_name.as_str());
            // Re-base offsets so playback starts immediately at `--start`.
            Some((off - start_off, to_moment_centroid(row, cam)))
        })
        .collect();

    if scheduled.is_empty() {
        bail!("No centroids fall within the requested --start/--duration window");
    }

    let socket = UdpSocket::bind("0.0.0.0:0").context("Binding local UDP socket")?;
    socket
        .connect(&opt.target)
        .with_context(|| format!("Connecting UDP socket to {}", opt.target))?;

    let span = scheduled.last().unwrap().0;
    tracing::info!(
        "Replaying {} centroids ({:.1}s of recording at {}x speed) to {}",
        scheduled.len(),
        span.as_secs_f64(),
        opt.speed,
        opt.target,
    );

    loop {
        replay_once(&socket, &scheduled, opt.speed)?;
        if !opt.r#loop {
            break;
        }
        tracing::info!("Looping.");
    }

    tracing::info!("Done.");
    Ok(())
}

/// Emit every scheduled centroid once, sleeping between sends to reproduce the
/// recorded cadence divided by `speed`.
fn replay_once(
    socket: &UdpSocket,
    scheduled: &[(Duration, MomentCentroid)],
    speed: f64,
) -> Result<()> {
    let wall_start = Instant::now();
    let mut sent = 0usize;
    for (off, centroid) in scheduled {
        let target = wall_start + off.div_f64(speed);
        let now = Instant::now();
        if target > now {
            std::thread::sleep(target - now);
        }
        let bytes = flo_core::encode_udp_msg(&UdpMsg::Centroid(centroid.clone()))
            .context("Encoding centroid")?;
        socket.send(&bytes).context("Sending UDP packet")?;
        sent += 1;
    }
    tracing::info!("Sent {sent} centroids.");
    Ok(())
}

/// Map each distinct recorded camera name to the name it should be sent as.
///
/// Cameras are numbered by first appearance: the first distinct name maps to
/// `--primary-cam` (if given), the second to `--secondary-cam` (if given), and
/// any further or un-overridden cameras keep their recorded names.
fn build_cam_remap(centroids: &[StampedMomentCentroid], opt: &Opt) -> Vec<(String, String)> {
    let mut order: Vec<String> = Vec::new();
    for c in centroids {
        let name = c.cam_name.as_str();
        if !order.iter().any(|n| n == name) {
            order.push(name.to_string());
        }
    }
    order
        .into_iter()
        .enumerate()
        .map(|(i, orig)| {
            let new = match i {
                0 => opt.primary_cam.clone(),
                1 => opt.secondary_cam.clone(),
                _ => None,
            }
            .unwrap_or_else(|| orig.clone());
            (orig, new)
        })
        .collect()
}
