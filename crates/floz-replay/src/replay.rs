// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! `replay` subcommand: re-emit the centroids recorded in a `.floz` file.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use chrono::{DateTime, Local};
use clap::Args;
use color_eyre::eyre::{Context, Result, bail};

use flo_core::{MomentCentroid, StampedMomentCentroid};

#[derive(Debug, Args)]
pub struct ReplayArgs {
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

/// Replay recorded centroids into `send`. The cadence and camera-name mapping
/// are independent of how observations reach FLO.
pub fn run_with_sink(
    opt: &ReplayArgs,
    mut send: impl FnMut(MomentCentroid) -> Result<()>,
) -> Result<()> {
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

    let span = scheduled.last().unwrap().0;
    tracing::info!(
        "Replaying {} centroids ({:.1}s of recording at {}x speed)",
        scheduled.len(),
        span.as_secs_f64(),
        opt.speed,
    );

    loop {
        replay_once(&scheduled, opt.speed, &mut send)?;
        if !opt.r#loop {
            break;
        }
        tracing::info!("Looping.");
    }

    tracing::info!("Done.");
    Ok(())
}

/// Run the legacy UDP transport adapter.
pub fn run(opt: ReplayArgs) -> Result<()> {
    let socket = crate::synth::connect_udp(&opt.target)?;
    run_with_sink(&opt, |centroid| {
        crate::synth::send_centroid(&socket, &centroid)
    })
}

/// Emit every scheduled centroid once, sleeping between sends to reproduce the
/// recorded cadence divided by `speed`.
fn replay_once(
    scheduled: &[(Duration, MomentCentroid)],
    speed: f64,
    send: &mut impl FnMut(MomentCentroid) -> Result<()>,
) -> Result<()> {
    let wall_start = Instant::now();
    let mut sent = 0usize;
    for (off, centroid) in scheduled {
        let target = wall_start + off.div_f64(speed);
        let now = Instant::now();
        if target > now {
            std::thread::sleep(target - now);
        }
        send(centroid.clone())?;
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
fn build_cam_remap(centroids: &[StampedMomentCentroid], opt: &ReplayArgs) -> Vec<(String, String)> {
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
