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
//! requested.
//!
//! The two tracking cameras are hardware triggered from one pulse, so both see
//! every subject at the same instant and both count every trigger. What they do
//! *not* share is where their hardware counters started, so the two
//! framenumbers for one trigger differ by an arbitrary fixed integer.
//! `--secondary-frame-offset` reproduces that; `--secondary-lag-frames`,
//! `--secondary-skew-ms` and `--{primary,secondary}-drop-every` reproduce what
//! else the controller has to cope with (see the "Stereo pairing" section of
//! the top-level README).

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use clap::Args;
use color_eyre::eyre::{Result, bail};

use flo_core::{
    CentroidToAngleCalibration, FloControllerConfig, MomentCentroid, StereopsisCalibration,
    TimestampSource,
};

#[derive(Debug, Args)]
pub struct SynthArgs {
    /// FLO config (YAML). Provides the angle calibration and (for distance) the
    /// `stereopsis_calib`, so the synthesized centroids invert the same model
    /// the controller uses. Use the same config you pass to `flo`.
    #[arg(long)]
    config: PathBuf,

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

    /// Primary (main) camera name. Defaults to the `flo-strand-cam.main`
    /// camera name in `--config`, so the synthesized observations are
    /// attributed to the camera the controller registered.
    #[arg(long)]
    primary_cam: Option<String>,

    /// Secondary (stereo) camera name. Defaults to the
    /// `flo-strand-cam.secondary` camera name in `--config`.
    #[arg(long)]
    secondary_cam: Option<String>,

    /// Generate only the primary camera (no stereo / no distance), even if the
    /// config has a `stereopsis_calib`.
    #[arg(long)]
    no_stereo: bool,

    /// Difference between the two cameras' hardware framenumbers at the first
    /// trigger (`secondary - primary`). May be negative.
    ///
    /// Both cameras count triggers in their own hardware counter, and those
    /// counters start wherever each camera was powered up, so a deployed pair
    /// is essentially never at the same count.
    #[arg(long, default_value_t = 0, allow_negative_numbers = true)]
    secondary_frame_offset: i32,

    /// Timestamp difference between the two cameras, in milliseconds.
    ///
    /// The cameras are triggered by one pulse, so this is host stamping jitter
    /// rather than a real difference in exposure time. Negative means the
    /// secondary is stamped earlier.
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    secondary_skew_ms: f64,

    /// Withhold every Nth secondary observation.
    ///
    /// The trigger still fired and the camera still counted it — this is the
    /// subject going undetected in that camera, or its frame going missing on
    /// the way — so framenumbers are unaffected and the trigger simply has no
    /// pair to be made from. 0 (the default) withholds nothing.
    #[arg(long, default_value_t = 0)]
    secondary_drop_every: u64,

    /// Withhold every Nth primary observation, the mirror image of
    /// `--secondary-drop-every`.
    #[arg(long, default_value_t = 0)]
    primary_drop_every: u64,

    /// Deliver secondary observations this many triggers late, leaving their
    /// framenumber and timestamp untouched.
    ///
    /// This models frames stuck somewhere on the way to the controller rather
    /// than lost: the data is correct but arrives after the primary has moved
    /// on.
    #[arg(long, default_value_t = 0)]
    secondary_lag_frames: u64,
}

/// Camera names to fall back on when neither the command line nor a
/// configuration supplies them. Only reachable from the standalone CLI, where
/// there is no embedded camera host to take names from.
const DEFAULT_PRIMARY_CAM: &str = "synth-primary";
const DEFAULT_SECONDARY_CAM: &str = "synth-secondary";

impl SynthArgs {
    /// The FLO configuration these centroids are projected through.
    pub fn config_path(&self) -> &Path {
        &self.config
    }

    /// Whether stereo observations will be generated for `cfg`.
    pub fn is_stereo(&self, cfg: &FloControllerConfig) -> bool {
        !self.no_stereo && cfg.stereopsis_calib.is_some()
    }

    /// Whether any of the camera-desynchronization options is in use.
    fn is_desynchronized(&self) -> bool {
        self.secondary_frame_offset != 0
            || self.secondary_skew_ms != 0.0
            || self.secondary_drop_every != 0
            || self.primary_drop_every != 0
            || self.secondary_lag_frames != 0
    }

    /// Adopt the embedded camera host's names for any not given on the command
    /// line. An explicit `--primary-cam`/`--secondary-cam` always wins.
    pub fn adopt_camera_names(&mut self, names: &flo_app::CameraNames) {
        if self.primary_cam.is_none() {
            self.primary_cam = Some(names.main.clone());
        }
        if self.secondary_cam.is_none() {
            self.secondary_cam.clone_from(&names.secondary);
        }
    }

    fn primary_cam(&self) -> &str {
        self.primary_cam.as_deref().unwrap_or(DEFAULT_PRIMARY_CAM)
    }

    fn secondary_cam(&self) -> &str {
        self.secondary_cam
            .as_deref()
            .unwrap_or(DEFAULT_SECONDARY_CAM)
    }
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

/// How one camera labels an observation: its own framenumber and its own
/// acquisition time. The two cameras get separate stamps because free-running
/// cameras neither expose at the same instant nor stay on the same count.
#[derive(Clone, Copy)]
struct Stamp {
    framenumber: u32,
    timestamp: chrono::DateTime<chrono::Utc>,
}

fn project(
    opt: &SynthArgs,
    cfg: &FloControllerConfig,
    stereo: Option<&StereoInverse>,
    t: f64,
    primary_stamp: Stamp,
    secondary_stamp: Stamp,
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

    let primary = moment_centroid(opt, opt.primary_cam(), primary_stamp, x_main, y_main);

    let secondary = match stereo {
        Some(inv) => {
            if r <= 0.0 {
                bail!("--distance/--distance-amplitude produced non-positive range {r}");
            }
            let x_stereo = x_main - inv.disparity_px(r);
            Some(moment_centroid(
                opt,
                opt.secondary_cam(),
                secondary_stamp,
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
    stamp: Stamp,
    x: f64,
    y: f64,
) -> MomentCentroid {
    MomentCentroid {
        schema_version: 2,
        framenumber: stamp.framenumber,
        timestamp_source: TimestampSource::HostAcquiredTimestamp,
        timestamp: stamp.timestamp,
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
            opt.primary_cam(),
            opt.secondary_cam(),
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

    if opt.is_desynchronized() {
        tracing::info!(
            "Cameras desynchronized on purpose: secondary framenumber offset {}, skew {} ms, \
             every {}th secondary and every {}th primary observation withheld, secondary \
             delivered {} frames late (0 = none)",
            opt.secondary_frame_offset,
            opt.secondary_skew_ms,
            opt.secondary_drop_every,
            opt.primary_drop_every,
            opt.secondary_lag_frames,
        );
    }
    let secondary_skew =
        chrono::TimeDelta::nanoseconds((opt.secondary_skew_ms * 1e6).round() as i64);

    // Both cameras are on the same trigger and count every one of them, so the
    // two counters keep whatever difference they started with, whatever happens
    // to the observations afterwards.
    let mut primary_framenumber: u32 = 0;
    let mut secondary_framenumber: u32 = 0u32.wrapping_add_signed(opt.secondary_frame_offset);
    let mut sample_index: u64 = 0;
    // Secondary observations still in transit, oldest first.
    let mut in_transit: std::collections::VecDeque<MomentCentroid> =
        std::collections::VecDeque::new();
    loop {
        let wall_start = Instant::now();
        // Both cameras are triggered by the same pulse, so one `now` per trigger
        // stamps both; --secondary-skew-ms is host stamping jitter on top.
        for i in 0..n_samples {
            let t = i as f64 / opt.rate;
            let target = wall_start + dt.mul_f64(i as f64);
            let now = Instant::now();
            if target > now {
                std::thread::sleep(target - now);
            }
            let timestamp = chrono::Utc::now();
            let withholds = |every: u64| every != 0 && (sample_index + 1).is_multiple_of(every);
            let withhold_primary = withholds(opt.primary_drop_every);
            let withhold_secondary = withholds(opt.secondary_drop_every);
            let sample = project(
                opt,
                cfg,
                stereo.as_ref(),
                t,
                Stamp {
                    framenumber: primary_framenumber,
                    timestamp,
                },
                Stamp {
                    framenumber: secondary_framenumber,
                    timestamp: timestamp + secondary_skew,
                },
            )?;
            if !withhold_primary {
                send(sample.primary)?;
            }
            primary_framenumber = primary_framenumber.wrapping_add(1);
            if let Some(secondary) = &sample.secondary
                && !withhold_secondary
            {
                in_transit.push_back(secondary.clone());
            }
            secondary_framenumber = secondary_framenumber.wrapping_add(1);
            // Anything that has been in transit long enough now arrives, keeping
            // its original framenumber and timestamp.
            while in_transit.len() > opt.secondary_lag_frames as usize {
                send(in_transit.pop_front().expect("length checked"))?;
            }
            sample_index += 1;
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

#[cfg(test)]
mod tests {
    use super::*;

    fn args() -> SynthArgs {
        SynthArgs {
            config: PathBuf::from("unused.yaml"),
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
            primary_cam: Some("primary".into()),
            secondary_cam: Some("secondary".into()),
            no_stereo: true,
            secondary_frame_offset: 0,
            secondary_skew_ms: 0.0,
            secondary_drop_every: 0,
            primary_drop_every: 0,
            secondary_lag_frames: 0,
        }
    }

    /// A stereo config, with the calibrations `--azimuth-deg` and `--distance`
    /// are inverted through.
    fn stereo_config() -> FloControllerConfig {
        FloControllerConfig {
            centroid_to_sensor_x_angle_func: CentroidToAngleCalibration {
                dx_gain: 0.00035,
                dy_gain: 0.0,
                offset: 0.0,
            },
            centroid_to_sensor_y_angle_func: CentroidToAngleCalibration {
                dx_gain: 0.0,
                dy_gain: -0.00035,
                offset: 0.0,
            },
            stereopsis_calib: Some(StereopsisCalibration {
                r1_m: 2.0,
                x_offset_1_px: 20.0,
                r2_m: 5.0,
                x_offset_2_px: 10.0,
                pixel_size_um: 3.4,
            }),
            ..Default::default()
        }
    }

    /// Run a short synthesis, collecting what each camera sent.
    fn collect(opt: &SynthArgs) -> (Vec<MomentCentroid>, Vec<MomentCentroid>) {
        let cfg = stereo_config();
        let mut sent = Vec::new();
        run_with_sink(opt, &cfg, |c| {
            sent.push(c);
            Ok(())
        })
        .unwrap();
        let (primary, secondary): (Vec<_>, Vec<_>) = sent
            .into_iter()
            .partition(|c| c.cam_name.as_str() == opt.primary_cam());
        (primary, secondary)
    }

    #[test]
    fn in_process_source_validates_before_using_a_transport() {
        let error =
            run_with_sink(&args(), &FloControllerConfig::default(), |_| Ok(())).unwrap_err();
        assert!(error.to_string().contains("--rate must be positive"));
    }

    #[test]
    fn synchronized_by_default() {
        let opt = SynthArgs {
            rate: 1000.0,
            duration: 0.005,
            no_stereo: false,
            ..args()
        };
        let (primary, secondary) = collect(&opt);
        assert_eq!(primary.len(), 5);
        assert_eq!(secondary.len(), 5);
        for (p, s) in primary.iter().zip(&secondary) {
            assert_eq!(p.framenumber, s.framenumber);
            assert_eq!(p.timestamp, s.timestamp);
        }
    }

    #[test]
    fn skew_shifts_only_the_secondary_timestamps() {
        let opt = SynthArgs {
            rate: 1000.0,
            duration: 0.005,
            no_stereo: false,
            secondary_skew_ms: 4.0,
            ..args()
        };
        let (primary, secondary) = collect(&opt);
        assert_eq!(primary.len(), secondary.len());
        for (p, s) in primary.iter().zip(&secondary) {
            assert_eq!(
                s.timestamp - p.timestamp,
                chrono::TimeDelta::milliseconds(4)
            );
            assert_eq!(p.framenumber, s.framenumber);
        }
    }

    /// Framenumber offset (`secondary - primary`) of the two observations that
    /// share an acquisition instant, for every trigger both cameras captured.
    ///
    /// The subtraction wraps, as it must for counters that are `u32` on the
    /// wire: a secondary that counts below the primary is a large positive
    /// difference, not a negative one.
    fn offsets(primary: &[MomentCentroid], secondary: &[MomentCentroid]) -> Vec<i64> {
        secondary
            .iter()
            .filter_map(|s| {
                let p = primary.iter().find(|p| p.timestamp == s.timestamp)?;
                Some(i64::from(s.framenumber.wrapping_sub(p.framenumber) as i32))
            })
            .collect()
    }

    #[test]
    fn a_starting_offset_stays_constant() {
        let opt = SynthArgs {
            rate: 1000.0,
            duration: 0.005,
            no_stereo: false,
            secondary_frame_offset: -7,
            ..args()
        };
        let (primary, secondary) = collect(&opt);
        assert_eq!(offsets(&primary, &secondary), vec![-7; 5]);
    }

    #[test]
    fn a_withheld_observation_leaves_the_framenumbers_alone() {
        let opt = SynthArgs {
            rate: 1000.0,
            duration: 0.010,
            no_stereo: false,
            secondary_frame_offset: 12,
            secondary_drop_every: 3,
            ..args()
        };
        let (primary, secondary) = collect(&opt);
        assert_eq!(primary.len(), 10);
        // Triggers 2, 5 and 8 (0-based) have no secondary observation.
        assert_eq!(secondary.len(), 7);
        // The camera counted those triggers even though nothing came of them,
        // so the offset between the counters is the one it started with.
        assert_eq!(offsets(&primary, &secondary), vec![12; 7]);
    }

    #[test]
    fn a_withheld_primary_observation_leaves_the_framenumbers_alone() {
        let opt = SynthArgs {
            rate: 1000.0,
            duration: 0.010,
            no_stereo: false,
            secondary_frame_offset: -12,
            primary_drop_every: 3,
            ..args()
        };
        let (primary, secondary) = collect(&opt);
        assert_eq!(primary.len(), 7);
        assert_eq!(secondary.len(), 10);
        assert_eq!(offsets(&primary, &secondary), vec![-12; 7]);
    }

    #[test]
    fn lagged_secondary_frames_keep_their_stamps_but_arrive_late() {
        let opt = SynthArgs {
            rate: 1000.0,
            duration: 0.005,
            no_stereo: false,
            secondary_lag_frames: 2,
            ..args()
        };
        let cfg = stereo_config();
        let mut sent = Vec::new();
        run_with_sink(&opt, &cfg, |c| {
            sent.push(c);
            Ok(())
        })
        .unwrap();
        // Stamps are untouched, so the pairing still has everything it needs.
        let (primary, secondary): (Vec<_>, Vec<_>) =
            sent.iter().cloned().partition(|c| c.cam_name == "primary");
        assert_eq!(offsets(&primary, &secondary), vec![0; 3]);
        // But each secondary is delivered two triggers after its partner, and
        // the last two never arrive at all before the run ends.
        assert_eq!(secondary.len(), 3);
        let order: Vec<&str> = sent.iter().map(|c| c.cam_name.as_str()).collect();
        assert_eq!(
            order,
            vec![
                "primary",
                "primary",
                "primary",
                "secondary",
                "primary",
                "secondary",
                "primary",
                "secondary",
            ]
        );
    }
}
