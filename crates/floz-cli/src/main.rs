// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! CLI tool to inspect and query `.floz` tracking data files.
//!
//! This is modeled on the `braidz-cli` program from strand-braid.

use std::collections::BTreeMap;
use std::path::PathBuf;

use chrono::{DateTime, Local, Utc};
use clap::Parser;
use color_eyre::eyre::{Result, WrapErr};
use serde::Serialize;

/// Time window (milliseconds) within which centroids from the two cameras are
/// considered a simultaneous pair eligible for stereopsis. This mirrors the
/// 5 ms threshold used in the FLO controller when establishing the stereo
/// camera frame offset (see `on_fast_tick` in the `flo` crate).
const STEREO_WINDOW_MILLIS: i64 = 5;

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Opt {
    /// Input .floz filename (a ZIP archive or an extracted directory).
    input: PathBuf,

    /// Print all rows in the `motor_positions` table.
    #[arg(short, long)]
    motor_positions: bool,

    /// Print all rows in the `tracking_state` table.
    #[arg(short, long)]
    tracking_states: bool,

    /// Print all rows in the `centroid` table.
    #[arg(short, long)]
    centroids: bool,

    /// Print all rows in the `encoder_data` table.
    #[arg(short, long)]
    encoder_data: bool,

    /// Print all rows in the `encoder_offsets` table.
    #[arg(short = 'o', long)]
    encoder_offsets: bool,

    /// Diagnose why the tracked distance is (or is not) changing.
    ///
    /// Reports how often the raw stereopsis distance observation (`dist_obs`)
    /// updated, whether the distance Kalman estimate (`est_dist`, renamed
    /// `online_distance` in downstream analysis) was ever produced, and the
    /// stereo centroid coverage that determines whether distance can be
    /// measured at all.
    #[arg(short = 'd', long)]
    distance: bool,
}

/// Summary of a single time-series table within a `.floz` file.
#[derive(Debug, Serialize)]
struct TableSummary {
    num_rows: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    start_time: Option<chrono::DateTime<chrono::Local>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    end_time: Option<chrono::DateTime<chrono::Local>>,
}

/// Top-level summary of the contents of a `.floz` file.
#[derive(Debug, Serialize)]
struct FlozSummary {
    filename: String,
    filesize: u64,
    /// Top-level entries contained in the archive.
    files: Vec<String>,
    motor_positions: TableSummary,
    tracking_states: TableSummary,
    centroids: TableSummary,
    encoder_data: TableSummary,
    encoder_offsets: TableSummary,
}

/// Diagnostic report explaining the behavior of the tracked distance.
#[derive(Debug, Serialize)]
struct DistanceReport {
    tracking_states: TrackingStateDistanceSummary,
    stereopsis: StereopsisSummary,
    /// Human-readable interpretation of the numbers above.
    diagnosis: Vec<String>,
}

#[derive(Debug, Serialize)]
struct TrackingStateDistanceSummary {
    num_rows: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    start_time: Option<DateTime<Local>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    end_time: Option<DateTime<Local>>,
    /// The distance Kalman estimate (`est_dist`); this is what downstream
    /// analysis renames to `online_distance`.
    online_distance_kalman_estimate: EstDistSummary,
    /// The raw stereopsis distance observation (`dist_obs`). This field is
    /// "sticky": it is only overwritten when a fresh, valid stereopsis
    /// measurement arrives, so a constant value means no new measurements.
    raw_distance_observation: DistObsSummary,
}

#[derive(Debug, Serialize)]
struct EstDistSummary {
    /// Rows where the Kalman estimate exists (the filter had been initialized).
    num_present: usize,
    /// Rows where the estimate is absent (the filter was never initialized,
    /// because no valid distance observation had yet been incorporated).
    num_absent: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    min: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max: Option<f32>,
}

#[derive(Debug, Serialize)]
struct DistObsSummary {
    #[serde(skip_serializing_if = "Option::is_none")]
    first: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    last: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    min: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max: Option<f32>,
    /// Count of distinct values seen across all rows.
    num_distinct_values: usize,
    /// Number of rows whose value differs from the immediately preceding row,
    /// i.e. how many times a fresh stereopsis measurement was incorporated.
    num_updates: usize,
    /// True if the value never changed for the whole recording.
    frozen: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    longest_unchanging_run: Option<UnchangingRun>,
}

#[derive(Debug, Serialize)]
struct UnchangingRun {
    value: f32,
    num_rows: usize,
    fraction_of_rows: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    start_time: Option<DateTime<Local>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    end_time: Option<DateTime<Local>>,
    duration_secs: f64,
}

#[derive(Debug, Serialize)]
struct StereopsisSummary {
    config: StereopsisConfigSummary,
    /// Per-camera centroid counts from the `centroid` table.
    cameras: Vec<CameraCentroidSummary>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stereo_pairs: Option<StereoPairSummary>,
}

#[derive(Debug, Serialize)]
struct StereopsisConfigSummary {
    /// Whether `flo-config.yaml` was present and parsed.
    config_parsed: bool,
    /// Whether `stereopsis_calib` is set (required to compute distance at all).
    stereopsis_calib_present: bool,
    /// Whether `kalman_filter_dist_parameters` is set (required for `est_dist`).
    distance_kalman_params_present: bool,
    /// Whether a secondary camera is configured (required for stereopsis).
    secondary_camera_configured: bool,
}

#[derive(Debug, Serialize)]
struct CameraCentroidSummary {
    cam_name: String,
    num_centroids: usize,
    /// Centroids with a real detection (`mu00 != 0`).
    num_valid_detections: usize,
}

#[derive(Debug, Serialize)]
struct StereoPairSummary {
    window_millis: i64,
    camera_a: String,
    camera_b: String,
    num_valid_a: usize,
    num_valid_b: usize,
    /// Greedy count of detection pairs (one per camera) within `window_millis`
    /// by camera *acquisition* time. This is the upper bound on how many
    /// stereopsis distances could have been computed.
    num_pairs_within_window: usize,
    /// Of those acquisition-aligned pairs, how many also arrived at the PC
    /// (`received_timestamp`) within `window_millis`. The live controller pairs
    /// by arrival, so a count well below `num_pairs_within_window` would mean
    /// the cameras' data reached the host too far apart to pair.
    num_pairs_received_within_window: usize,
    /// The controller locks onto the framenumber offset of the first valid pair
    /// and then rejects any pair whose offset differs. This reports how
    /// consistent the time-aligned pairs are by that criterion.
    #[serde(skip_serializing_if = "Option::is_none")]
    framenumber_offset: Option<FramenumberOffsetSummary>,
}

#[derive(Debug, Serialize)]
struct FramenumberOffsetSummary {
    /// The most common framenumber offset (`framenumber_a - framenumber_b`)
    /// among time-aligned pairs — what the controller would most likely lock.
    modal_offset: i64,
    /// Pairs whose offset equals `modal_offset` (the controller would accept).
    num_pairs_at_modal_offset: usize,
    /// Pairs whose offset differs (the controller would reject these).
    num_pairs_off_modal_offset: usize,
    /// Distinct framenumber offsets observed across the time-aligned pairs.
    num_distinct_offsets: usize,
}

/// True if a centroid carries a real detection (nonzero zeroth moment).
fn is_valid_centroid(c: &flo_core::StampedMomentCentroid) -> bool {
    c.mu00 != 0.0
}

/// One valid centroid as used for stereo pairing: acquisition time, received
/// time, and framenumber.
#[derive(Clone, Copy)]
struct CentroidStamp {
    acquired: DateTime<Utc>,
    received: DateTime<Local>,
    framenumber: u32,
}

/// A matched stereo pair, by camera acquisition time.
struct MatchedPair {
    /// `framenumber_a - framenumber_b`.
    framenumber_offset: i64,
    /// Difference of the two cameras' `received_timestamp`s.
    received_skew: chrono::TimeDelta,
}

/// Greedily match detection pairs (one centroid from each camera) whose
/// acquisition timestamps fall within `window` of each other. Both inputs must
/// be sorted ascending by acquisition time.
fn match_stereo_pairs(
    a: &[CentroidStamp],
    b: &[CentroidStamp],
    window: chrono::TimeDelta,
) -> Vec<MatchedPair> {
    let (mut i, mut j) = (0, 0);
    let mut pairs = Vec::new();
    while i < a.len() && j < b.len() {
        let delta = a[i].acquired - b[j].acquired;
        if delta.abs() <= window {
            pairs.push(MatchedPair {
                framenumber_offset: i64::from(a[i].framenumber) - i64::from(b[j].framenumber),
                received_skew: a[i].received - b[j].received,
            });
            i += 1;
            j += 1;
        } else if a[i].acquired < b[j].acquired {
            i += 1;
        } else {
            j += 1;
        }
    }
    pairs
}

fn build_distance_report(
    tracking_states: &[flo_core::SaveTrackingState],
    centroids: &[flo_core::StampedMomentCentroid],
    config: Option<&flo_core::FloControllerConfig>,
) -> DistanceReport {
    // --- est_dist (online distance) ---
    let est_present: Vec<f32> = tracking_states.iter().filter_map(|r| r.est_dist).collect();
    let est = EstDistSummary {
        num_present: est_present.len(),
        num_absent: tracking_states.len() - est_present.len(),
        min: est_present.iter().copied().reduce(f32::min),
        max: est_present.iter().copied().reduce(f32::max),
    };

    // --- dist_obs (raw, sticky stereopsis observation) ---
    let obs: Vec<f32> = tracking_states.iter().map(|r| r.dist_obs).collect();
    let mut distinct = std::collections::HashSet::new();
    for v in &obs {
        distinct.insert(v.to_bits());
    }
    let mut num_updates = 0usize;
    // Track the longest run of identical consecutive values.
    let (mut best_start, mut best_len) = (0usize, if obs.is_empty() { 0 } else { 1 });
    let mut cur_start = 0usize;
    for idx in 1..obs.len() {
        if obs[idx].to_bits() != obs[idx - 1].to_bits() {
            num_updates += 1;
            if idx - cur_start > best_len {
                best_len = idx - cur_start;
                best_start = cur_start;
            }
            cur_start = idx;
        }
    }
    if !obs.is_empty() && obs.len() - cur_start > best_len {
        best_len = obs.len() - cur_start;
        best_start = cur_start;
    }

    let longest_unchanging_run = if obs.is_empty() {
        None
    } else {
        let end_idx = best_start + best_len - 1;
        let start_time = tracking_states
            .get(best_start)
            .map(|r| r.processed_timestamp);
        let end_time = tracking_states.get(end_idx).map(|r| r.processed_timestamp);
        let duration_secs = match (start_time, end_time) {
            (Some(s), Some(e)) => (e - s).num_microseconds().unwrap_or(0) as f64 / 1e6,
            _ => 0.0,
        };
        Some(UnchangingRun {
            value: obs[best_start],
            num_rows: best_len,
            fraction_of_rows: best_len as f64 / obs.len() as f64,
            start_time,
            end_time,
            duration_secs,
        })
    };

    let dist_obs = DistObsSummary {
        first: obs.first().copied(),
        last: obs.last().copied(),
        min: obs.iter().copied().reduce(f32::min),
        max: obs.iter().copied().reduce(f32::max),
        num_distinct_values: distinct.len(),
        num_updates,
        frozen: num_updates == 0 && !obs.is_empty(),
        longest_unchanging_run,
    };

    // --- per-camera centroid coverage ---
    // Each entry: (total centroids, valid count, stamps of valid detections).
    let mut by_cam: BTreeMap<String, (usize, usize, Vec<CentroidStamp>)> = BTreeMap::new();
    for c in centroids {
        let entry = by_cam.entry(c.cam_name.clone()).or_default();
        entry.0 += 1;
        if is_valid_centroid(c) {
            entry.1 += 1;
            entry.2.push(CentroidStamp {
                acquired: c.timestamp,
                received: c.received_timestamp,
                framenumber: c.framenumber,
            });
        }
    }
    let cameras: Vec<CameraCentroidSummary> = by_cam
        .iter()
        .map(|(name, (total, valid, _))| CameraCentroidSummary {
            cam_name: name.clone(),
            num_centroids: *total,
            num_valid_detections: *valid,
        })
        .collect();

    // --- stereo pairing (only meaningful with exactly two cameras) ---
    let stereo_pairs = if by_cam.len() == 2 {
        let mut iter = by_cam.iter();
        let (name_a, (_, valid_a, ts_a)) = iter.next().unwrap();
        let (name_b, (_, valid_b, ts_b)) = iter.next().unwrap();
        let mut ts_a = ts_a.clone();
        let mut ts_b = ts_b.clone();
        ts_a.sort_unstable_by_key(|c| c.acquired);
        ts_b.sort_unstable_by_key(|c| c.acquired);
        let window = chrono::TimeDelta::milliseconds(STEREO_WINDOW_MILLIS);
        let matched = match_stereo_pairs(&ts_a, &ts_b, window);

        let num_pairs_received_within_window = matched
            .iter()
            .filter(|p| p.received_skew.abs() <= window)
            .count();

        // Summarize how consistent the framenumber offsets are: the controller
        // locks the first offset and rejects pairs that disagree with it.
        let framenumber_offset = if matched.is_empty() {
            None
        } else {
            let mut counts: BTreeMap<i64, usize> = BTreeMap::new();
            for p in &matched {
                *counts.entry(p.framenumber_offset).or_default() += 1;
            }
            let (&modal_offset, &modal_count) = counts.iter().max_by_key(|(_, n)| **n).unwrap();
            Some(FramenumberOffsetSummary {
                modal_offset,
                num_pairs_at_modal_offset: modal_count,
                num_pairs_off_modal_offset: matched.len() - modal_count,
                num_distinct_offsets: counts.len(),
            })
        };

        Some(StereoPairSummary {
            window_millis: STEREO_WINDOW_MILLIS,
            camera_a: name_a.clone(),
            camera_b: name_b.clone(),
            num_valid_a: *valid_a,
            num_valid_b: *valid_b,
            num_pairs_within_window: matched.len(),
            num_pairs_received_within_window,
            framenumber_offset,
        })
    } else {
        None
    };

    let config_summary = StereopsisConfigSummary {
        config_parsed: config.is_some(),
        stereopsis_calib_present: config.is_some_and(|c| c.stereopsis_calib.is_some()),
        distance_kalman_params_present: config
            .is_some_and(|c| c.kalman_filter_dist_parameters.is_some()),
        secondary_camera_configured: config
            .is_some_and(|c| c.strand_cam_secondary.is_some() || c.secondary_cam_name.is_some()),
    };

    let diagnosis = build_diagnosis(&est, &dist_obs, &config_summary, &cameras, &stereo_pairs);

    DistanceReport {
        tracking_states: TrackingStateDistanceSummary {
            num_rows: tracking_states.len(),
            start_time: tracking_states.first().map(|r| r.processed_timestamp),
            end_time: tracking_states.last().map(|r| r.processed_timestamp),
            online_distance_kalman_estimate: est,
            raw_distance_observation: dist_obs,
        },
        stereopsis: StereopsisSummary {
            config: config_summary,
            cameras,
            stereo_pairs,
        },
        diagnosis,
    }
}

/// Translate the computed numbers into plain-language observations.
fn build_diagnosis(
    est: &EstDistSummary,
    dist_obs: &DistObsSummary,
    config: &StereopsisConfigSummary,
    cameras: &[CameraCentroidSummary],
    stereo_pairs: &Option<StereoPairSummary>,
) -> Vec<String> {
    let mut out = Vec::new();

    // Configuration prerequisites.
    if config.config_parsed {
        if !config.stereopsis_calib_present {
            out.push(
                "`stereopsis_calib` is not configured, so distance can never be computed from \
                 the two cameras. `dist_obs` stays at its default and `est_dist` is never set."
                    .to_string(),
            );
        }
        if !config.distance_kalman_params_present {
            out.push(
                "`kalman_filter_dist_parameters` is not configured, so the distance Kalman filter \
                 never runs and `est_dist`/`online_distance` is always empty."
                    .to_string(),
            );
        }
        if !config.secondary_camera_configured {
            out.push(
                "No secondary camera is configured, so no stereopsis (and hence no distance \
                 measurement) is possible."
                    .to_string(),
            );
        }
    } else {
        out.push(
            "`flo-config.yaml` was missing or could not be parsed, so configuration-based checks \
             were skipped."
                .to_string(),
        );
    }

    // The estimate itself.
    if est.num_present == 0 {
        out.push(format!(
            "The distance Kalman estimate (`est_dist`/`online_distance`) is empty in all {} rows: \
             no valid stereopsis distance observation was ever incorporated, so the filter never \
             initialized.",
            est.num_absent
        ));
    } else if est.num_absent > 0 {
        out.push(format!(
            "The distance Kalman estimate is present in {} rows and absent in {} (the filter \
             initializes only once the first valid distance observation arrives).",
            est.num_present, est.num_absent
        ));
    }

    // The raw observation.
    if dist_obs.frozen {
        out.push(format!(
            "`dist_obs` never changed (held {} for the whole recording). It only updates when a \
             fresh stereopsis measurement arrives, so no new measurement was ever incorporated.",
            dist_obs.first.unwrap_or_default()
        ));
    } else if let Some(run) = &dist_obs.longest_unchanging_run {
        if run.fraction_of_rows >= 0.5 {
            out.push(format!(
                "`dist_obs` updated {} times total, but its longest unchanging stretch covers {} \
                 rows ({:.1}% of the recording, {:.1} s): during that stretch no fresh stereopsis \
                 measurement arrived.",
                dist_obs.num_updates,
                run.num_rows,
                run.fraction_of_rows * 100.0,
                run.duration_secs,
            ));
        } else {
            out.push(format!(
                "`dist_obs` updated {} times across the recording (longest unchanging stretch: {} \
                 rows, {:.1} s).",
                dist_obs.num_updates, run.num_rows, run.duration_secs,
            ));
        }
    }

    // Stereo coverage — the usual reason measurements are sparse.
    if let Some(sp) = stereo_pairs {
        if sp.num_pairs_within_window == 0 {
            out.push(format!(
                "No detections from `{}` and `{}` fell within {} ms of each other, so no \
                 stereopsis distance could be computed. Distance only updates when both cameras \
                 detect the target simultaneously.",
                sp.camera_a, sp.camera_b, sp.window_millis
            ));
        } else {
            out.push(format!(
                "{} of {}/{} valid detections from `{}`/`{}` could be paired within {} ms — these \
                 are the only opportunities to measure distance.",
                sp.num_pairs_within_window,
                sp.num_valid_a,
                sp.num_valid_b,
                sp.camera_a,
                sp.camera_b,
                sp.window_millis,
            ));
            if let Some(fo) = &sp.framenumber_offset
                && fo.num_pairs_off_modal_offset > 0
            {
                out.push(format!(
                    "Of those pairs, {} match the modal framenumber offset ({}) but {} do not \
                     ({} distinct offsets seen). The controller locks the first offset and \
                     rejects mismatched pairs, so framenumber drift between the cameras (e.g. \
                     dropped frames) suppresses distance updates.",
                    fo.num_pairs_at_modal_offset,
                    fo.modal_offset,
                    fo.num_pairs_off_modal_offset,
                    fo.num_distinct_offsets,
                ));
            }
            // Arrival skew: the live controller pairs by arrival order, so if
            // far fewer pairs arrived together than were acquired together, the
            // host received the two streams too far apart to pair them live.
            if sp.num_pairs_received_within_window * 2 < sp.num_pairs_within_window {
                out.push(format!(
                    "Although {} pairs were acquired within {} ms, only {} also *arrived* at the \
                     host within {} ms. The live controller pairs by arrival, so this arrival \
                     skew (e.g. one camera's data lagging) prevents most pairs from forming.",
                    sp.num_pairs_within_window,
                    sp.window_millis,
                    sp.num_pairs_received_within_window,
                    sp.window_millis,
                ));
            }

            // The decisive case: stereo opportunities existed and the inputs are
            // clean, yet no distance was ever produced — the failure is in the
            // live system, not the recorded data.
            if dist_obs.frozen
                && est.num_present == 0
                && sp.num_pairs_received_within_window * 2 >= sp.num_pairs_within_window
                && sp
                    .framenumber_offset
                    .as_ref()
                    .is_none_or(|fo| fo.num_pairs_off_modal_offset == 0)
            {
                out.push(
                    "Despite clean, well-aligned stereo opportunities (consistent framenumber \
                     offset, matching arrival times), no distance was ever incorporated. The \
                     recorded inputs are sufficient, so the cause is in the live stereo pipeline \
                     at record time (e.g. the two centroids were never buffered together within a \
                     single fast tick) rather than in the data — investigate the controller."
                        .to_string(),
                );
            }
        }
    } else if cameras.len() < 2 {
        out.push(format!(
            "Only {} camera(s) produced centroids; stereopsis needs two, so distance cannot be \
             measured.",
            cameras.len()
        ));
    } else {
        out.push(format!(
            "{} cameras produced centroids; stereo pairing analysis assumes exactly two, so it \
             was skipped.",
            cameras.len()
        ));
    }

    out
}

fn init_tracing() -> Result<()> {
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
    if std::env::var_os("RUST_LOG").is_none() {
        // SAFETY: We ensure that this only happens in single-threaded code
        // because this is immediately at the start of main() and no other
        // threads have started.
        unsafe { std::env::set_var("RUST_LOG", "info") };
    }

    color_eyre::install()?;
    init_tracing()?;

    let opt = Opt::parse();

    let attr = std::fs::metadata(&opt.input)
        .with_context(|| format!("Getting file metadata for {}", opt.input.display()))?;

    let mut floz = floz_parser::floz_parse_path(&opt.input)
        .with_context(|| format!("Parsing file {}", opt.input.display()))?;

    // Move the parsed tables out of the archive so we can both summarize them
    // and (below) consume the archive to list its raw contents.
    let motor_positions = std::mem::take(&mut floz.motor_positions);
    let tracking_states = std::mem::take(&mut floz.tracking_states);
    let centroids = std::mem::take(&mut floz.centroids);
    let encoder_data = std::mem::take(&mut floz.encoder_data);
    let encoder_offsets = std::mem::take(&mut floz.encoder_offsets);
    let config = floz.config.take();

    let motor_summary = TableSummary {
        num_rows: motor_positions.len(),
        start_time: motor_positions.first().map(|row| row.local),
        end_time: motor_positions.last().map(|row| row.local),
    };

    let tracking_summary = TableSummary {
        num_rows: tracking_states.len(),
        start_time: tracking_states.first().map(|row| row.processed_timestamp),
        end_time: tracking_states.last().map(|row| row.processed_timestamp),
    };

    let centroid_summary = TableSummary {
        num_rows: centroids.len(),
        start_time: centroids.first().map(|row| row.received_timestamp),
        end_time: centroids.last().map(|row| row.received_timestamp),
    };

    let encoder_data_summary = TableSummary {
        num_rows: encoder_data.len(),
        start_time: encoder_data.first().map(|row| row.local),
        end_time: encoder_data.last().map(|row| row.local),
    };

    // `encoder_offsets` is calibration data with no per-row timestamp.
    let encoder_offsets_summary = TableSummary {
        num_rows: encoder_offsets.len(),
        start_time: None,
        end_time: None,
    };

    // Consume the archive to enumerate its top-level contents.
    let archive = floz.into_inner();
    let files = archive
        .list_paths::<PathBuf>(None)
        .context("Listing archive contents")?
        .into_iter()
        .map(|p| p.display().to_string())
        .collect();

    let summary = FlozSummary {
        filename: opt.input.display().to_string(),
        filesize: attr.len(),
        files,
        motor_positions: motor_summary,
        tracking_states: tracking_summary,
        centroids: centroid_summary,
        encoder_data: encoder_data_summary,
        encoder_offsets: encoder_offsets_summary,
    };

    let yaml_buf = serde_yaml::to_string(&summary)?;
    println!("{}", yaml_buf);

    if opt.distance {
        let report = build_distance_report(&tracking_states, &centroids, config.as_ref());
        let report_buf = serde_yaml::to_string(&report)?;
        println!("distance_report:");
        // Indent the report under the `distance_report:` key.
        for line in report_buf.lines() {
            println!("  {line}");
        }
    }

    if opt.motor_positions {
        println!("motor_positions table: --------------");
        for row in &motor_positions {
            println!("{:?}", row);
        }
    }

    if opt.tracking_states {
        println!("tracking_state table: --------------");
        for row in &tracking_states {
            println!("{:?}", row);
        }
    }

    if opt.centroids {
        println!("centroid table: --------------");
        for row in &centroids {
            println!("{:?}", row);
        }
    }

    if opt.encoder_data {
        println!("encoder_data table: --------------");
        for row in &encoder_data {
            println!("{:?}", row);
        }
    }

    if opt.encoder_offsets {
        println!("encoder_offsets table: --------------");
        for row in &encoder_offsets {
            println!("{:?}", row);
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp(acq_ms: i64, recv_ms: i64, framenumber: u32) -> CentroidStamp {
        let epoch = DateTime::<Utc>::UNIX_EPOCH;
        CentroidStamp {
            acquired: epoch + chrono::TimeDelta::milliseconds(acq_ms),
            received: DateTime::<Local>::from(epoch + chrono::TimeDelta::milliseconds(recv_ms)),
            framenumber,
        }
    }

    #[test]
    fn pairs_only_within_window() {
        let window = chrono::TimeDelta::milliseconds(5);
        // a[1]/b[1] are aligned; the others are far apart in time.
        let a = [stamp(0, 0, 100), stamp(50, 50, 101), stamp(1000, 1000, 102)];
        let b = [stamp(52, 52, 200), stamp(2000, 2000, 201)];
        let matched = match_stereo_pairs(&a, &b, window);
        assert_eq!(matched.len(), 1);
        assert_eq!(matched[0].framenumber_offset, 101 - 200);
        assert_eq!(
            matched[0].received_skew,
            chrono::TimeDelta::milliseconds(-2)
        );
    }

    #[test]
    fn no_pairs_when_streams_disjoint_in_time() {
        let window = chrono::TimeDelta::milliseconds(5);
        let a = [stamp(0, 0, 1), stamp(10, 10, 2)];
        let b = [stamp(100, 100, 1), stamp(110, 110, 2)];
        assert!(match_stereo_pairs(&a, &b, window).is_empty());
    }

    #[test]
    fn received_skew_detects_arrival_lag() {
        let window = chrono::TimeDelta::milliseconds(5);
        // Acquired together, but b arrives 20 ms late: pairs by acquisition but
        // not by arrival.
        let a = [stamp(0, 0, 1)];
        let b = [stamp(1, 21, 1)];
        let matched = match_stereo_pairs(&a, &b, window);
        assert_eq!(matched.len(), 1);
        assert!(matched[0].received_skew.abs() > window);
    }
}
