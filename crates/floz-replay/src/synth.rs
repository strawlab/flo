// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! `synth` subcommand: generate centroids from a parametric virtual-insect
//! trajectory.
//!
//! The trajectory is defined in angular coordinates relative to the main
//! camera's bore: an azimuth/elevation Lissajous oscillation at a configurable
//! range. Each sample is projected into a [`MomentCentroid`] by *inverting* the
//! controller's own calibration:
//!
//! * The azimuth/elevation angles are inverted through the
//!   `centroid_to_sensor_{x,y}_angle_func` 2x2 map to recover the pixel offset
//!   `(dx, dy)` from the image center, giving the main camera's centroid
//!   position.
//! * The range is inverted through `stereopsis_calib` to compute the
//!   main-vs-stereo x disparity, giving the secondary camera's centroid x.
//!
//! Because the projection uses the controller's own calibration, the angles and
//! stereopsis distance the controller recovers match the trajectory that was
//! requested. Stereo pairs are sent with identical framenumbers and timestamps
//! so the controller's pairing (constant frame offset, <5 ms timestamp skew)
//! locks immediately.

use std::net::UdpSocket;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Args;
use color_eyre::eyre::{Context, Result, bail};

use flo_core::{
    CentroidToAngleCalibration, FloControllerConfig, MomentCentroid, StereopsisCalibration,
    TimestampSource, UdpMsg,
};

#[derive(Debug, Args)]
pub struct SynthArgs {
    /// FLO config (YAML). Provides the angle calibration and (for distance) the
    /// `stereopsis_calib`, so the synthesized centroids invert the same model
    /// the controller uses. Use the same config you pass to `flo`.
    #[arg(long)]
    config: PathBuf,

    /// UDP address of the running FLO controller (its `--udp-addr`).
    #[arg(long, default_value = "127.0.0.1:8080")]
    target: String,

    /// Centroid output rate, in Hz (per camera).
    #[arg(long, default_value_t = 100.0)]
    rate: f64,

    /// Total duration to generate, in seconds.
    #[arg(long, default_value_t = 30.0)]
    duration: f64,

    /// Repeat indefinitely (until interrupted with Ctrl-C).
    #[arg(long)]
    r#loop: bool,

    /// Period of the azimuth/elevation oscillation, in seconds.
    #[arg(long, default_value_t = 8.0)]
    period: f64,

    /// Azimuth (horizontal) oscillation amplitude, in degrees.
    #[arg(long, default_value_t = 8.0)]
    azimuth_deg: f64,

    /// Elevation (vertical) oscillation amplitude, in degrees.
    #[arg(long, default_value_t = 5.0)]
    elevation_deg: f64,

    /// Base target range, in meters.
    #[arg(long, default_value_t = 2.0)]
    distance: f64,

    /// Sinusoidal range variation amplitude, in meters (0 = constant range).
    #[arg(long, default_value_t = 0.0)]
    distance_amplitude: f64,

    /// Period of the range variation, in seconds. Defaults to `--period`.
    #[arg(long)]
    distance_period: Option<f64>,

    /// Image-center x the controller expects (the `SetImOpsCenterX` value).
    #[arg(long, default_value_t = 960)]
    center_x: u32,

    /// Image-center y the controller expects (the `SetImOpsCenterY` value).
    #[arg(long, default_value_t = 600)]
    center_y: u32,

    /// Centroid zeroth moment (`mu00`, the detection "mass"). Must be non-zero
    /// for the controller to treat it as a detection.
    #[arg(long, default_value_t = 1000.0)]
    mass: f64,

    /// Primary (main) camera name. Should match the controller's main camera.
    #[arg(long, default_value = "synth-primary")]
    primary_cam: String,

    /// Secondary (stereo) camera name. Should match the controller's
    /// `strand_cam_secondary.cam_name`.
    #[arg(long, default_value = "synth-secondary")]
    secondary_cam: String,

    /// Generate only the primary camera (no stereo / no distance), even if the
    /// config has a `stereopsis_calib`.
    #[arg(long)]
    no_stereo: bool,
}

/// Inverse of [`flo_core::StereopsisCalibration`] in pixel units.
///
/// The forward model recovers range from disparity as `r = f / |disp - d_inf|`,
/// where disparity is `x_main - x_stereo`. Inverting: `disp = d_inf + f / r`.
/// The `pixel_size_um` factor cancels, so we work in raw pixels.
struct StereoInverse {
    f_px: f64,
    d_inf_px: f64,
}

impl StereoInverse {
    fn new(cal: &StereopsisCalibration) -> Self {
        let (r1, r2) = (cal.r1_m, cal.r2_m);
        let (d1, d2) = (cal.x_offset_1_px, cal.x_offset_2_px);
        let f_px = r1 * r2 * (d1 - d2) / (r2 - r1);
        let d_inf_px = d1 - f_px / r1;
        Self { f_px, d_inf_px }
    }

    /// Main-minus-stereo x disparity (pixels) that encodes range `r` meters.
    fn disparity_px(&self, r: f64) -> f64 {
        self.d_inf_px + self.f_px / r
    }
}

/// Solve the controller's centroid->angle 2x2 map for the pixel offset
/// `(dx, dy)` from the image center that produces the requested
/// `(sensor_x_angle, sensor_y_angle)`.
///
/// Forward (see `centroid_to_angle` in the `flo` crate):
///   sx = dx*xf.dx_gain + dy*xf.dy_gain + xf.offset
///   sy = dx*yf.dx_gain + dy*yf.dy_gain + yf.offset
fn angles_to_pixel_offset(
    xf: &CentroidToAngleCalibration,
    yf: &CentroidToAngleCalibration,
    sx: f64,
    sy: f64,
) -> Result<(f64, f64)> {
    let (a, b) = (xf.dx_gain, xf.dy_gain);
    let (c, d) = (yf.dx_gain, yf.dy_gain);
    let det = a * d - b * c;
    if det.abs() < 1e-12 {
        bail!(
            "centroid_to_sensor angle calibration is singular (det={det:.3e}); \
             cannot invert angles to pixels"
        );
    }
    let (px, py) = (sx - xf.offset, sy - yf.offset);
    let dx = (px * d - py * b) / det;
    let dy = (a * py - c * px) / det;
    Ok((dx, dy))
}

/// One projected stereo (or mono) sample at trajectory time `t` seconds.
struct Sample {
    primary: MomentCentroid,
    secondary: Option<MomentCentroid>,
}

fn project(
    opt: &SynthArgs,
    cfg: &FloControllerConfig,
    stereo: Option<&StereoInverse>,
    t: f64,
    framenumber: u32,
    timestamp: chrono::DateTime<chrono::Utc>,
) -> Result<Sample> {
    let w = std::f64::consts::TAU / opt.period;
    let az = opt.azimuth_deg.to_radians() * (w * t).sin();
    // Use cosine for elevation so the motion traces an ellipse rather than a
    // line, exercising both axes simultaneously.
    let el = opt.elevation_deg.to_radians() * (w * t).cos();
    let dist_period = opt.distance_period.unwrap_or(opt.period);
    let r = opt.distance + opt.distance_amplitude * (std::f64::consts::TAU / dist_period * t).sin();

    let (dx, dy) = angles_to_pixel_offset(
        &cfg.centroid_to_sensor_x_angle_func,
        &cfg.centroid_to_sensor_y_angle_func,
        az,
        el,
    )?;
    let x_main = opt.center_x as f64 + dx;
    let y_main = opt.center_y as f64 + dy;

    let primary = moment_centroid(
        opt,
        &opt.primary_cam,
        framenumber,
        timestamp,
        x_main,
        y_main,
    );

    let secondary = match stereo {
        Some(inv) => {
            if r <= 0.0 {
                bail!("--distance/--distance-amplitude produced non-positive range {r}");
            }
            let x_stereo = x_main - inv.disparity_px(r);
            Some(moment_centroid(
                opt,
                &opt.secondary_cam,
                framenumber,
                timestamp,
                x_stereo,
                y_main,
            ))
        }
        None => None,
    };

    Ok(Sample { primary, secondary })
}

/// Build a centroid whose `x()`/`y()` are the requested pixel coordinates.
///
/// `x() = mu10/mu00` and `y() = mu01/mu00`, so we scale the requested
/// coordinates by the chosen `mass` (mu00).
fn moment_centroid(
    opt: &SynthArgs,
    cam_name: &str,
    framenumber: u32,
    timestamp: chrono::DateTime<chrono::Utc>,
    x: f64,
    y: f64,
) -> MomentCentroid {
    MomentCentroid {
        schema_version: 2,
        framenumber,
        timestamp_source: TimestampSource::HostAcquiredTimestamp,
        timestamp,
        mu00: opt.mass,
        mu10: opt.mass * x,
        mu01: opt.mass * y,
        center_x: opt.center_x,
        center_y: opt.center_y,
        cam_name: cam_name.into(),
    }
}

/// Produce synthetic centroids into `send` using an already-loaded FLO
/// configuration. This is the transport-independent entry point used by both
/// the legacy UDP CLI and the in-process FLO extension.
pub fn run_with_sink(
    opt: &SynthArgs,
    cfg: &FloControllerConfig,
    mut send: impl FnMut(MomentCentroid) -> Result<()>,
) -> Result<()> {
    if opt.rate <= 0.0 {
        bail!("--rate must be positive, got {}", opt.rate);
    }
    if opt.duration <= 0.0 {
        bail!("--duration must be positive, got {}", opt.duration);
    }

    let stereo = if opt.no_stereo {
        None
    } else {
        cfg.stereopsis_calib.as_ref().map(StereoInverse::new)
    };
    if stereo.is_some() {
        tracing::info!(
            "Stereo enabled: sending primary {:?} + secondary {:?} (distance tracking)",
            opt.primary_cam,
            opt.secondary_cam,
        );
    } else if opt.no_stereo {
        tracing::info!("Stereo disabled by --no-stereo: sending primary only");
    } else {
        tracing::info!(
            "No stereopsis_calib in config: sending primary only (no distance). \
             Add a stereopsis_calib for distance tracking."
        );
    }

    let n_samples = (opt.duration * opt.rate).round() as u64;
    let dt = Duration::from_secs_f64(1.0 / opt.rate);
    tracing::info!(
        "Synthesizing {} samples at {} Hz ({:.1}s)",
        n_samples,
        opt.rate,
        opt.duration,
    );

    let mut framenumber: u32 = 0;
    loop {
        let wall_start = Instant::now();
        // Base the synthetic acquisition timestamps on a single `now` per pass so
        // a stereo pair always shares an identical timestamp.
        for i in 0..n_samples {
            let t = i as f64 / opt.rate;
            let target = wall_start + dt.mul_f64(i as f64);
            let now = Instant::now();
            if target > now {
                std::thread::sleep(target - now);
            }
            let timestamp = chrono::Utc::now();
            let sample = project(opt, cfg, stereo.as_ref(), t, framenumber, timestamp)?;
            send(sample.primary)?;
            if let Some(secondary) = &sample.secondary {
                send(secondary.clone())?;
            }
            framenumber = framenumber.wrapping_add(1);
        }
        tracing::info!("Sent {n_samples} samples.");
        if !opt.r#loop {
            break;
        }
        tracing::info!("Looping.");
    }

    tracing::info!("Done.");
    Ok(())
}

/// Run the legacy UDP transport adapter.
pub fn run(opt: SynthArgs) -> Result<()> {
    let cfg: FloControllerConfig = {
        let rdr = std::fs::File::open(&opt.config)
            .with_context(|| format!("Opening config {}", opt.config.display()))?;
        serde_yaml::from_reader(rdr)
            .with_context(|| format!("Parsing config {}", opt.config.display()))?
    };
    let socket = connect_udp(&opt.target)?;
    run_with_sink(&opt, &cfg, |centroid| send_centroid(&socket, &centroid))
}

/// Bind a local UDP socket and connect it to `target` so `send` can be used.
pub fn connect_udp(target: &str) -> Result<UdpSocket> {
    let socket = UdpSocket::bind("0.0.0.0:0").context("Binding local UDP socket")?;
    socket
        .connect(target)
        .with_context(|| format!("Connecting UDP socket to {target}"))?;
    Ok(socket)
}

/// CBOR-encode a centroid as the controller's [`UdpMsg`] and send it.
pub fn send_centroid(socket: &UdpSocket, centroid: &MomentCentroid) -> Result<()> {
    let bytes = flo_core::encode_udp_msg(&UdpMsg::Centroid(centroid.clone()))
        .context("Encoding centroid")?;
    socket.send(&bytes).context("Sending UDP packet")?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn in_process_source_validates_before_using_a_transport() {
        let args = SynthArgs {
            config: PathBuf::from("unused.yaml"),
            target: "127.0.0.1:8080".into(),
            rate: 0.0,
            duration: 1.0,
            r#loop: false,
            period: 1.0,
            azimuth_deg: 1.0,
            elevation_deg: 1.0,
            distance: 1.0,
            distance_amplitude: 0.0,
            distance_period: None,
            center_x: 0,
            center_y: 0,
            mass: 1.0,
            primary_cam: "primary".into(),
            secondary_cam: "secondary".into(),
            no_stereo: true,
        };

        let error = run_with_sink(&args, &FloControllerConfig::default(), |_| Ok(())).unwrap_err();
        assert!(error.to_string().contains("--rate must be positive"));
    }
}
