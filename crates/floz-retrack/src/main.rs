// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! CLI tool to re-estimate target distance offline from the centroid data
//! already saved in a `.floz` file.
//!
//! The live FLO controller computes distance by stereopsis: pairing
//! simultaneous centroids from the primary and secondary cameras and applying
//! the `stereopsis_calib` calibration. When that live pairing fails (see
//! `floz-cli --distance` for diagnosis), the recorded `dist_obs`/`est_dist`
//! columns are frozen or empty even though the raw centroids from both cameras
//! are present and usable. This tool re-pairs those centroids offline and fills
//! in the distance, writing a new `.floz` that downstream analysis can consume
//! unchanged.
//!
//! This v1 computes only the raw per-pair stereopsis distance (no Kalman
//! smoothing) and writes it into both the `dist_obs` and `est_dist` columns of
//! the recomputed `tracking_state` rows.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::io::Write;
use std::path::{Path, PathBuf};

use chrono::{DateTime, Utc};
use clap::Parser;
use color_eyre::eyre::{Context, Result, bail, eyre};

use flo_core::{
    BMsg, FloControllerConfig, FloDetectionEvent, MomentCentroid, SaveTrackingState,
    StampedMomentCentroid, StereopsisCalibration, TRACKING_STATE_FNAME,
};

/// Text written before the ZIP data in a `.floz` file (the format is a ZIP with
/// a small identifying prefix). Mirrors what the FLO recorder writes.
const FLOZ_HEADER: &str = "FLOZ file. This is a standard ZIP file with a specific schema.\n";
const README_FNAME: &str = "README.md";

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Opt {
    /// Input .floz filename (a ZIP archive or an extracted directory).
    input: PathBuf,

    /// Name of the primary (main) camera. If omitted, it is auto-detected from
    /// the `broadway.jsonl` event log.
    #[arg(long)]
    primary_cam: Option<String>,

    /// Name of the secondary (stereo) camera. If omitted, it is taken as the
    /// other camera present in the data (or auto-detected from `broadway.jsonl`).
    #[arg(long)]
    secondary_cam: Option<String>,

    /// Maximum difference (milliseconds) between the two cameras' acquisition
    /// timestamps for centroids to be paired. Defaults to the 5 ms used by the
    /// live controller.
    #[arg(long, default_value_t = 5)]
    window_ms: i64,

    /// Output .floz filename. Defaults to `<input-stem>_retracked.floz`
    /// alongside the input.
    #[arg(short, long)]
    output: Option<PathBuf>,
}

/// Project a [`StampedMomentCentroid`] back to the [`MomentCentroid`] expected
/// by the stereopsis calibration.
fn to_moment_centroid(c: &StampedMomentCentroid) -> MomentCentroid {
    MomentCentroid {
        schema_version: c.schema_version,
        framenumber: c.framenumber,
        timestamp_source: c.timestamp_source.clone(),
        timestamp: c.timestamp,
        mu00: c.mu00,
        mu01: c.mu01,
        mu10: c.mu10,
        center_x: c.center_x,
        center_y: c.center_y,
        cam_name: c.cam_name.clone(),
    }
}

/// Resolve which camera is primary and which is secondary.
///
/// Priority: explicit CLI flags, then the `is_primary` flag recorded in
/// `broadway.jsonl`, then — if exactly two cameras are present and one role is
/// known — the remaining camera.
fn resolve_roles(
    opt: &Opt,
    cams_present: &BTreeSet<String>,
    broadway: &[flo_core::StampedBMsg],
) -> Result<(String, String)> {
    // Roles recorded live, if any.
    let mut detected_primary = None;
    let mut detected_secondary = None;
    for ev in broadway {
        if let BMsg::FloDetectionEvent(FloDetectionEvent::Centroid(ce)) = &ev.msg {
            let slot = if ce.is_primary {
                &mut detected_primary
            } else {
                &mut detected_secondary
            };
            if slot.is_none() {
                *slot = Some(ce.centroid.cam_name.clone());
            }
        }
        if detected_primary.is_some() && detected_secondary.is_some() {
            break;
        }
    }

    let primary = opt
        .primary_cam
        .clone()
        .or(detected_primary)
        .ok_or_else(|| {
            eyre!(
                "Could not determine the primary camera. Pass --primary-cam \
                 (cameras present: {cams_present:?})."
            )
        })?;

    let secondary = opt
        .secondary_cam
        .clone()
        .or(detected_secondary)
        .or_else(|| {
            // The other camera, if exactly two are present.
            let others: Vec<&String> = cams_present.iter().filter(|c| **c != primary).collect();
            if others.len() == 1 {
                Some(others[0].clone())
            } else {
                None
            }
        })
        .ok_or_else(|| {
            eyre!(
                "Could not determine the secondary camera. Pass --secondary-cam \
                 (cameras present: {cams_present:?})."
            )
        })?;

    if primary == secondary {
        bail!("Primary and secondary cameras must differ (both are `{primary}`).");
    }
    for cam in [&primary, &secondary] {
        if !cams_present.contains(cam) {
            bail!("Camera `{cam}` has no centroids in this file (present: {cams_present:?}).");
        }
    }
    Ok((primary, secondary))
}

/// A valid centroid reduced to what pairing and distance need.
struct Det {
    acquired: DateTime<Utc>,
    framenumber: u32,
    moment: MomentCentroid,
}

/// Greedily pair primary and secondary detections by acquisition time within
/// `window`, then keep only the pairs sharing the most common framenumber
/// offset (mirroring how the controller locks one offset and rejects the rest).
/// Returns `(primary_acquisition_time, distance_meters)` for each kept pair.
fn compute_distances(
    primary: &[Det],
    secondary: &[Det],
    calib: &StereopsisCalibration,
    window: chrono::TimeDelta,
) -> Vec<(DateTime<Utc>, f32)> {
    // First pass: greedy time-match, recording the framenumber offset of each
    // candidate pair.
    let mut candidates: Vec<(usize, usize, i64)> = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < primary.len() && j < secondary.len() {
        let delta = primary[i].acquired - secondary[j].acquired;
        if delta.abs() <= window {
            let offset = i64::from(primary[i].framenumber) - i64::from(secondary[j].framenumber);
            candidates.push((i, j, offset));
            i += 1;
            j += 1;
        } else if primary[i].acquired < secondary[j].acquired {
            i += 1;
        } else {
            j += 1;
        }
    }

    // Determine the modal framenumber offset.
    let mut offset_counts: BTreeMap<i64, usize> = BTreeMap::new();
    for (_, _, off) in &candidates {
        *offset_counts.entry(*off).or_default() += 1;
    }
    let modal_offset = offset_counts.iter().max_by_key(|(_, n)| **n).map(|(o, _)| *o);

    let mut out = Vec::new();
    for (pi, si, off) in candidates {
        if Some(off) != modal_offset {
            continue;
        }
        let state = calib.centroids_to_distance((
            primary[pi].moment.clone(),
            secondary[si].moment.clone(),
        ));
        let dist = state.as_radial_distance().0 as f32;
        if dist.is_finite() && dist > 0.0 {
            out.push((primary[pi].acquired, dist));
        }
    }
    out
}

/// Write a new `.floz`: copy every entry from `src` verbatim except
/// `tracking_state.csv`, which is regenerated from `tracking_states`.
fn write_retracked_floz<R: std::io::Read + std::io::Seek>(
    src: &mut zip_or_dir::ZipDirArchive<R>,
    tracking_states: &[SaveTrackingState],
    output: &Path,
) -> Result<()> {
    let mut names: Vec<String> = src
        .list_paths::<PathBuf>(None)
        .context("Listing source archive contents")?
        .into_iter()
        .map(|p| p.to_string_lossy().into_owned())
        .collect();
    // Keep README first so the first bytes identify the file, matching the
    // recorder's convention.
    names.sort_by_key(|n| (n != README_FNAME, n.clone()));

    let file = std::fs::File::create(output)
        .with_context(|| format!("Creating output {}", output.display()))?;
    let mut file = std::io::BufWriter::new(file);
    file.write_all(FLOZ_HEADER.as_bytes())?;

    let mut zip = zip::ZipWriter::new(file);
    // Data tables are compressed. The README is stored uncompressed and first
    // (see the sort above) so the start of the file is human-readable, matching
    // the `.braidz` convention.
    let compressed = zip::write::SimpleFileOptions::default()
        .large_file(true)
        .compression_method(zip::CompressionMethod::Deflated)
        .unix_permissions(0o755);
    let stored = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Stored)
        .unix_permissions(0o755);

    for name in &names {
        let options = if name == README_FNAME { stored } else { compressed };
        zip.start_file(name, options)?;
        if name == TRACKING_STATE_FNAME {
            let mut wtr = csv::Writer::from_writer(&mut zip);
            for row in tracking_states {
                wtr.serialize(row)?;
            }
            wtr.flush()?;
        } else {
            let mut rdr = src
                .open(name)
                .with_context(|| format!("Opening `{name}` in source archive"))?;
            std::io::copy(&mut rdr, &mut zip)?;
        }
    }
    zip.finish()?;
    Ok(())
}

fn default_output(input: &Path) -> PathBuf {
    let stem = input
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "output".to_string());
    let fname = format!("{stem}_retracked.floz");
    match input.parent() {
        Some(p) if !p.as_os_str().is_empty() => p.join(fname),
        _ => PathBuf::from(fname),
    }
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

fn main() -> Result<()> {
    color_eyre::install()?;
    init_tracing()?;
    let opt = Opt::parse();

    let mut floz = floz_parser::floz_parse_path(&opt.input)
        .with_context(|| format!("Parsing file {}", opt.input.display()))?;

    let mut tracking_states = std::mem::take(&mut floz.tracking_states);
    let centroids = std::mem::take(&mut floz.centroids);
    let config: Option<FloControllerConfig> = floz.config.take();

    let calib = config
        .as_ref()
        .and_then(|c| c.stereopsis_calib.clone())
        .ok_or_else(|| {
            eyre!(
                "No `stereopsis_calib` in this file's flo-config.yaml; distance cannot be \
                 computed from the cameras."
            )
        })?;

    let mut archive = floz.into_inner();
    let broadway = floz_parser::read_broadway_log(&mut archive)
        .context("Reading broadway.jsonl for camera roles")?;

    // Which cameras have valid detections?
    let cams_present: BTreeSet<String> = centroids
        .iter()
        .filter(|c| c.mu00 != 0.0)
        .map(|c| c.cam_name.clone())
        .collect();
    if cams_present.len() < 2 {
        bail!(
            "Stereopsis needs two cameras; only {} produced valid centroids ({cams_present:?}).",
            cams_present.len()
        );
    }

    let (primary_cam, secondary_cam) = resolve_roles(&opt, &cams_present, &broadway)?;
    tracing::info!("primary camera: {primary_cam}, secondary camera: {secondary_cam}");

    // Collect and time-sort valid detections per role.
    let collect = |cam: &str| -> Vec<Det> {
        let mut v: Vec<Det> = centroids
            .iter()
            .filter(|c| c.mu00 != 0.0 && c.cam_name == cam)
            .map(|c| Det {
                acquired: c.timestamp,
                framenumber: c.framenumber,
                moment: to_moment_centroid(c),
            })
            .collect();
        v.sort_by_key(|d| d.acquired);
        v
    };
    let primary = collect(&primary_cam);
    let secondary = collect(&secondary_cam);

    let window = chrono::TimeDelta::milliseconds(opt.window_ms);
    let distances = compute_distances(&primary, &secondary, &calib, window);
    if distances.is_empty() {
        bail!(
            "No valid stereo pairs found within {} ms; nothing to retrack.",
            opt.window_ms
        );
    }
    let dist_by_time: HashMap<DateTime<Utc>, f32> = distances.into_iter().collect();
    tracing::info!(
        "computed distance for {} stereo pairs (of {} primary / {} secondary detections)",
        dist_by_time.len(),
        primary.len(),
        secondary.len(),
    );

    // Fill the recomputed distance into tracking_state rows, matched by the
    // primary centroid's acquisition timestamp.
    let mut num_filled = 0usize;
    for row in &mut tracking_states {
        if let Some(ts) = row.centroid_timestamp
            && let Some(&dist) = dist_by_time.get(&ts)
        {
            row.dist_obs = dist;
            row.est_dist = Some(dist);
            num_filled += 1;
        }
    }
    tracing::info!("filled distance into {num_filled} tracking_state rows");

    let output = opt.output.clone().unwrap_or_else(|| default_output(&opt.input));
    write_retracked_floz(&mut archive, &tracking_states, &output)?;
    tracing::info!("wrote {}", output.display());
    println!("{}", output.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn det(acq_ms: i64, framenumber: u32) -> Det {
        let acquired = DateTime::<Utc>::UNIX_EPOCH + chrono::TimeDelta::milliseconds(acq_ms);
        let moment = MomentCentroid {
            framenumber,
            // x() = mu10/mu00; vary x with framenumber so pairs differ.
            mu00: 1.0,
            mu10: f64::from(framenumber),
            ..Default::default()
        };
        Det {
            acquired,
            framenumber,
            moment,
        }
    }

    fn test_calib() -> StereopsisCalibration {
        // Values from a real flo-config.yaml.
        StereopsisCalibration {
            r1_m: 4.35,
            x_offset_1_px: 218.8,
            r2_m: 22.25,
            x_offset_2_px: 85.8,
            pixel_size_um: 3.45,
        }
    }

    #[test]
    fn pairs_within_window_and_produces_distance() {
        let calib = test_calib();
        let window = chrono::TimeDelta::milliseconds(5);
        // Two aligned pairs (consistent framenumber offset of -100), one
        // primary detection with no partner in range.
        let primary = vec![det(0, 10), det(50, 11), det(1000, 12)];
        let secondary = vec![det(1, 110), det(52, 111)];
        let out = compute_distances(&primary, &secondary, &calib, window);
        assert_eq!(out.len(), 2);
        for (_, d) in &out {
            assert!(d.is_finite() && *d > 0.0);
        }
    }

    #[test]
    fn rejects_off_modal_framenumber_offset() {
        let calib = test_calib();
        let window = chrono::TimeDelta::milliseconds(5);
        // Three time-aligned pairs: offsets -100, -100, and -7 (the outlier,
        // e.g. a dropped frame). Only the two modal-offset pairs are kept.
        let primary = vec![det(0, 10), det(50, 11), det(100, 12)];
        let secondary = vec![det(1, 110), det(52, 111), det(101, 19)];
        let out = compute_distances(&primary, &secondary, &calib, window);
        assert_eq!(out.len(), 2);
    }

    #[test]
    fn no_pairs_when_disjoint_in_time() {
        let calib = test_calib();
        let window = chrono::TimeDelta::milliseconds(5);
        let primary = vec![det(0, 1), det(10, 2)];
        let secondary = vec![det(100, 1), det(110, 2)];
        assert!(compute_distances(&primary, &secondary, &calib, window).is_empty());
    }

    #[test]
    fn resolves_secondary_from_two_cameras_and_flag() {
        let opt = Opt {
            input: PathBuf::new(),
            primary_cam: Some("camA".to_string()),
            secondary_cam: None,
            window_ms: 5,
            output: None,
        };
        let cams: BTreeSet<String> = ["camA", "camB"].iter().map(|s| s.to_string()).collect();
        let (p, s) = resolve_roles(&opt, &cams, &[]).unwrap();
        assert_eq!((p.as_str(), s.as_str()), ("camA", "camB"));
    }

    #[test]
    fn errors_when_roles_undeterminable() {
        let opt = Opt {
            input: PathBuf::new(),
            primary_cam: None,
            secondary_cam: None,
            window_ms: 5,
            output: None,
        };
        let cams: BTreeSet<String> = ["camA", "camB"].iter().map(|s| s.to_string()).collect();
        assert!(resolve_roles(&opt, &cams, &[]).is_err());
    }

    #[test]
    fn default_output_appends_retracked() {
        let out = default_output(Path::new("/data/flo20260519_181751.289609672.floz"));
        assert_eq!(
            out,
            PathBuf::from("/data/flo20260519_181751.289609672_retracked.floz")
        );
    }
}
