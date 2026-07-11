use std::io::{Seek, Write};

use color_eyre::eyre::Result;
use flo_core::{
    FloControllerConfig, GimbalEncoderData, GimbalEncoderOffsets, MomentCentroid, SaveToDiskMsg,
    StampedBMsg, StampedJson, StampedMomentCentroid,
};
use serde::{Deserialize, Serialize};

use crate::json_lines_writer::JsonLinesWriter;

/// Test that StampedMomentCentroid contains all fields in MomentCentroid.
#[test]
fn test_stamped_is_superset() -> Result<()> {
    // This deserializes `MomentCentroid`, adds a field, and then serializes to
    // `StampedMomentCentroid`. Due to the `deny_unknown_fields` attribute on
    // `StampedMomentCentroid`, this ensures that new fields added to
    // `MomentCentroid` during development remain part of
    // `StampedMomentCentroid`. We could theoretically write a macro to
    // auto-generated `StampedMomentCentroid` from `MomentCentroid` but writing
    // macros gets ugly.

    let mc = MomentCentroid::default();
    let received_timestamp = chrono::Local::now();
    let received_timestamp_value = serde_json::to_value(received_timestamp)?;

    let mut json_value = serde_json::to_value(mc)?;
    let obj = json_value.as_object_mut().unwrap();
    if obj
        .insert("received_timestamp".to_string(), received_timestamp_value)
        .is_some()
    {
        color_eyre::eyre::bail!("'received_timestamp' field already in MomentCentroid");
    }
    let _smc: StampedMomentCentroid = serde_json::from_value(json_value)?;
    Ok(())
}

#[test]
fn test_writer_task_creates_floz_on_toggle_off() -> Result<()> {
    use chrono::TimeZone;

    let base_dir = tempfile::tempdir()?;
    let output_dir = camino::Utf8PathBuf::from_path_buf(base_dir.path().join("session.flo"))
        .expect("tempdir path must be valid utf-8");

    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let config = FloControllerConfig::default();
    let (buffered_secs_tx, _buffered_secs_rx) = tokio::sync::watch::channel(0.0);
    let handle = std::thread::spawn(move || writer_task_main(rx, &config, buffered_secs_tx));

    let creation_time = chrono::FixedOffset::east_opt(0)
        .expect("valid fixed offset")
        .with_ymd_and_hms(2026, 5, 28, 12, 0, 0)
        .single()
        .expect("valid datetime");

    tx.send(SaveToDiskMsg::ToggleSavingFloz(Some((
        creation_time.into(),
        output_dir.clone(),
        false,
    ))))?;

    tx.send(SaveToDiskMsg::ToggleSavingFloz(None))?;
    tx.send(SaveToDiskMsg::Quit)?;
    drop(tx);

    handle.join().expect("writer thread panicked")?;

    let floz_path = output_dir.with_extension("floz");
    assert!(
        floz_path.exists(),
        "expected floz file at {}",
        floz_path.as_std_path().display()
    );
    assert!(
        !output_dir.exists(),
        "expected output dir removed: {}",
        output_dir.as_std_path().display()
    );

    floz_parser::floz_parse_path(&floz_path)?;

    Ok(())
}

#[test]
fn test_precapture_buffer_is_written_to_recording() -> Result<()> {
    // Data sent *before* recording starts, while a pre-capture window is set,
    // must end up in the recording (the "time travel" capture).
    let base_dir = tempfile::tempdir()?;
    let output_dir = camino::Utf8PathBuf::from_path_buf(base_dir.path().join("session.flo"))
        .expect("tempdir path must be valid utf-8");

    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let config = FloControllerConfig::default();
    let (buffered_secs_tx, _buffered_secs_rx) = tokio::sync::watch::channel(0.0);
    let handle = std::thread::spawn(move || writer_task_main(rx, &config, buffered_secs_tx));

    // Enable a generous pre-capture window, then send a centroid while NOT
    // recording. It should be buffered in RAM.
    tx.send(SaveToDiskMsg::SetPreCaptureSeconds(60.0))?;
    let buffered_centroid = MomentCentroid {
        framenumber: 42,
        ..Default::default()
    };
    tx.send(SaveToDiskMsg::CentroidData((
        chrono::Local::now(),
        buffered_centroid,
    )))?;

    // Now start a pre-capture recording. The buffered centroid must be
    // flushed into it.
    tx.send(SaveToDiskMsg::ToggleSavingFloz(Some((
        chrono::Local::now(),
        output_dir.clone(),
        true,
    ))))?;

    // A live centroid after recording started.
    let live_centroid = MomentCentroid {
        framenumber: 43,
        ..Default::default()
    };
    tx.send(SaveToDiskMsg::CentroidData((
        chrono::Local::now(),
        live_centroid,
    )))?;

    tx.send(SaveToDiskMsg::ToggleSavingFloz(None))?;
    tx.send(SaveToDiskMsg::Quit)?;
    drop(tx);

    handle.join().expect("writer thread panicked")?;

    let floz_path = output_dir.with_extension("floz");
    let archive = floz_parser::floz_parse_path(&floz_path)?;
    let framenumbers: Vec<u32> = archive.centroids.iter().map(|c| c.framenumber).collect();
    assert_eq!(
        framenumbers,
        vec![42, 43],
        "pre-captured centroid (42) must precede the live one (43)"
    );

    Ok(())
}

#[test]
fn test_normal_recording_excludes_precapture_buffer() -> Result<()> {
    // A normal recording (include_precapture = false) must NOT pull in data
    // buffered before it started.
    let base_dir = tempfile::tempdir()?;
    let output_dir = camino::Utf8PathBuf::from_path_buf(base_dir.path().join("session.flo"))
        .expect("tempdir path must be valid utf-8");

    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
    let config = FloControllerConfig::default();
    let (buffered_secs_tx, _buffered_secs_rx) = tokio::sync::watch::channel(0.0);
    let handle = std::thread::spawn(move || writer_task_main(rx, &config, buffered_secs_tx));

    tx.send(SaveToDiskMsg::SetPreCaptureSeconds(60.0))?;
    let buffered_centroid = MomentCentroid {
        framenumber: 42,
        ..Default::default()
    };
    tx.send(SaveToDiskMsg::CentroidData((
        chrono::Local::now(),
        buffered_centroid,
    )))?;

    // Normal recording: include_precapture = false.
    tx.send(SaveToDiskMsg::ToggleSavingFloz(Some((
        chrono::Local::now(),
        output_dir.clone(),
        false,
    ))))?;

    let live_centroid = MomentCentroid {
        framenumber: 43,
        ..Default::default()
    };
    tx.send(SaveToDiskMsg::CentroidData((
        chrono::Local::now(),
        live_centroid,
    )))?;

    tx.send(SaveToDiskMsg::ToggleSavingFloz(None))?;
    tx.send(SaveToDiskMsg::Quit)?;
    drop(tx);

    handle.join().expect("writer thread panicked")?;

    let floz_path = output_dir.with_extension("floz");
    let archive = floz_parser::floz_parse_path(&floz_path)?;
    let framenumbers: Vec<u32> = archive.centroids.iter().map(|c| c.framenumber).collect();
    assert_eq!(
        framenumbers,
        vec![43],
        "normal recording must contain only the live centroid (43)"
    );

    Ok(())
}

fn with_received_timestamp(
    mc: MomentCentroid,
    received_timestamp: chrono::DateTime<chrono::Local>,
) -> StampedMomentCentroid {
    StampedMomentCentroid {
        received_timestamp,
        schema_version: mc.schema_version,
        framenumber: mc.framenumber,
        timestamp_source: mc.timestamp_source.clone(),
        timestamp: mc.timestamp,
        mu00: mc.mu00,
        mu01: mc.mu01,
        mu10: mc.mu10,
        center_x: mc.center_x,
        center_y: mc.center_y,
        cam_name: mc.cam_name,
    }
}

/// Attempt to shutdown writer.
///
/// This will not necessarily always work as drop is not guaranteed to be called.
pub(crate) struct WriteCloser {
    flo_write_tx: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
}

impl WriteCloser {
    pub(crate) fn new(flo_write_tx: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>) -> Self {
        Self { flo_write_tx }
    }
}

impl Drop for WriteCloser {
    fn drop(&mut self) {
        tracing::trace!("quitting writer because WriteCloser dropped");
        match self.flo_write_tx.send(SaveToDiskMsg::Quit) {
            Ok(_) => {}
            Err(e) => {
                tracing::info!("WriteCloser could not send quit message: {:?}", e);
            }
        }
    }
}

/// Listen to a Receiver for messages and save the data to disk.
///
/// This function only exits upon error or when the Sender counterpart to the
/// Receiver has closed. It blocks and does not use an async context and thus
/// should be spawned with `tokio::task::spawn_blocking`. There would be little
/// benefit to making this fully async because writing to files with tokio
/// anyway uses `tokio::task::spawn_blocking`.
#[tracing::instrument(skip_all)]
pub(crate) fn writer_task_main(
    mut flo_write_rx: tokio::sync::mpsc::UnboundedReceiver<SaveToDiskMsg>,
    config: &FloControllerConfig,
    buffered_secs_tx: tokio::sync::watch::Sender<f64>,
) -> Result<()> {
    use SaveToDiskMsg::*;
    use std::time::{Duration, Instant};

    let mut writing_state: Option<WritingState> = None;
    const FLUSH_INTERVAL: u64 = 1;
    let flush_interval = Duration::from_secs(FLUSH_INTERVAL);

    let mut last_flushed = Instant::now();

    tracing::debug!("Starting floz writer task. {}:{}", file!(), line!());

    let mut encoder_offsets = None;

    // Pre-capture ("post-trigger") RAM buffer. Holds recent data messages
    // while not recording so that starting a recording also writes the
    // preceding window to disk.
    let mut precapture = PreCaptureBuffer::new();

    // Ideally we would have a timeout here, but this is not available.

    while let Some(msg) = flo_write_rx.blocking_recv() {
        tracing::trace!("processing message {msg:?}");
        match msg {
            Quit => {
                break;
            }
            SetPreCaptureSeconds(secs) => {
                precapture.set_window_secs(secs);
                let _ = buffered_secs_tx.send(precapture.buffered_secs());
            }
            ToggleSavingFloz(values) => {
                if let Some((creation_time, output_dirname, include_precapture)) = values {
                    tracing::info!("Saving FLO data to {output_dirname}");
                    if writing_state.is_none() {
                        let mut ws = WritingState::new(
                            creation_time,
                            output_dirname,
                            encoder_offsets.as_ref(),
                            config,
                        )?;
                        // For a pre-capture ("post-trigger") recording, flush
                        // the buffered window into the new recording first so
                        // it begins in the past. A normal recording leaves the
                        // buffer untouched and starts from now.
                        if include_precapture {
                            let buffered = precapture.drain();
                            if !buffered.is_empty() {
                                tracing::info!(
                                    "Writing {} pre-captured messages to recording",
                                    buffered.len()
                                );
                            }
                            for (_stamp, buffered_msg) in buffered {
                                ws.save_data_msg(buffered_msg)?;
                            }
                            let _ = buffered_secs_tx.send(precapture.buffered_secs());
                        }
                        writing_state = Some(ws);
                    }
                } else {
                    tracing::info!("Done saving FLO data");
                    // This will drop the writers and thus close them.
                    writing_state = None;
                }
            }
            GimbalEncoderOffsets(offsets) => {
                if let Some(ref mut ws) = writing_state.as_mut() {
                    ws.save_encoder_offsets(&offsets)?;
                }
                // Cache value.
                encoder_offsets = Some(offsets);
            }
            // All remaining variants carry per-event data. While recording,
            // write them straight to disk; otherwise feed the pre-capture
            // buffer (which itself ignores them when the window is zero).
            data_msg => {
                if let Some(ref mut ws) = writing_state.as_mut() {
                    ws.save_data_msg(data_msg)?;
                } else if precapture.push(data_msg) {
                    let _ = buffered_secs_tx.send(precapture.buffered_secs());
                }
            }
        }
        tracing::trace!("processing message done");

        // after processing message, check if we should flush data.
        if last_flushed.elapsed() > flush_interval {
            // flush all writers
            if let Some(ref mut ws) = writing_state {
                ws.flush_all()?;
            }

            last_flushed = Instant::now();
        }
    }
    tracing::info!("Done with floz writer task.");
    Ok(())
}

/// RAM ring buffer of recent data messages for pre-capture recording.
///
/// Messages are retained with the wall-clock time they were dequeued. The
/// buffer keeps at most `window_secs` worth of data, trimming the oldest
/// messages as new ones arrive. A zero window disables buffering entirely.
struct PreCaptureBuffer {
    window_secs: f64,
    inner: std::collections::VecDeque<(chrono::DateTime<chrono::Local>, SaveToDiskMsg)>,
}

impl PreCaptureBuffer {
    fn new() -> Self {
        Self {
            window_secs: 0.0,
            inner: std::collections::VecDeque::new(),
        }
    }

    /// Update the buffer window. Zero (or negative) disables buffering and
    /// discards anything currently held.
    fn set_window_secs(&mut self, secs: f64) {
        if secs > 0.0 {
            self.window_secs = secs;
            self.trim(chrono::Local::now());
        } else {
            self.window_secs = 0.0;
            self.inner.clear();
        }
    }

    /// Buffer a data message. Returns `true` if it was retained (i.e. the
    /// buffer is enabled), so callers can avoid recomputing the buffered span
    /// when nothing changed.
    fn push(&mut self, msg: SaveToDiskMsg) -> bool {
        if self.window_secs <= 0.0 {
            return false;
        }
        let now = chrono::Local::now();
        self.inner.push_back((now, msg));
        self.trim(now);
        true
    }

    /// Drop messages older than the window relative to `now`.
    fn trim(&mut self, now: chrono::DateTime<chrono::Local>) {
        while let Some((stamp, _)) = self.inner.front() {
            if secs_between(*stamp, now) > self.window_secs {
                self.inner.pop_front();
            } else {
                break;
            }
        }
    }

    /// Number of seconds of data currently buffered (span between the oldest
    /// and newest retained message).
    fn buffered_secs(&self) -> f64 {
        match (self.inner.front(), self.inner.back()) {
            (Some((first, _)), Some((last, _))) => secs_between(*first, *last),
            _ => 0.0,
        }
    }

    /// Take all buffered messages, leaving the buffer empty.
    fn drain(&mut self) -> std::collections::VecDeque<(chrono::DateTime<chrono::Local>, SaveToDiskMsg)>
    {
        std::mem::take(&mut self.inner)
    }
}

fn secs_between(
    earlier: chrono::DateTime<chrono::Local>,
    later: chrono::DateTime<chrono::Local>,
) -> f64 {
    (later - earlier).num_milliseconds() as f64 / 1000.0
}

const README_MD_FNAME: &str = "README.md";

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
struct FloMetadata {
    git_revision: String,
    creation_time: chrono::DateTime<chrono::FixedOffset>,
    timezone: String,
}

struct WritingState {
    output_dirname: camino::Utf8PathBuf,
    /// The readme file in the output directory.
    ///
    /// We keep this file open to establish locking on the open directory.
    ///
    /// In theory, we might prefer an open reference to the directory itself,
    /// but this does not seem possible. So we have a potential slight race
    /// condition when we have our directory but not yet the file handle on
    /// readme.
    readme_fd: Option<std::fs::File>,
    centroid_wtr: csv::Writer<Box<dyn Write + Send>>,
    tracking_state_wtr: csv::Writer<Box<dyn Write + Send>>,
    motor_position_wtr: csv::Writer<Box<dyn Write + Send>>,
    encoder_offsets_wtr: Option<csv::Writer<Box<dyn Write + Send>>>,
    encoder_data_wtr: Option<csv::Writer<Box<dyn Write + Send>>>,
    mavlink_data_wtr: Option<JsonLinesWriter<Box<dyn Write + Send>>>,
    broadway_data_wtr: Option<JsonLinesWriter<Box<dyn Write + Send>>>,
    /// One JSONL file per registered extension, opened lazily on the first
    /// record. Keyed by extension name (e.g. "extension" → `extension.jsonl`).
    extension_wtrs:
        std::collections::BTreeMap<&'static str, JsonLinesWriter<Box<dyn Write + Send>>>,
}

fn _test_writing_state_is_send() {
    // Compile-time test to ensure WritingState implements Send trait.
    fn implements<T: Send>() {}
    implements::<WritingState>();
}

fn readme_contents() -> String {
    // This text is stored uncompressed and as the first entry in the `.floz`
    // archive, so its leading bytes also serve to identify the file format.
    format!(
        r#"# FLO data file (`.floz`)

This is a data file saved by `{name}` (version {version}), part of the FLO
(Fast Lock-On) project for high-resolution videography of moving subjects using
a camera system that automatically tracks the subject.

Project home page: <https://github.com/strawlab/flo>

FLO is described in:

> Vo-Doan TT, Titov VV, Harrap MJM, Lochner S, Straw AD. High Resolution
> Outdoor Videography of Insects Using Fast Lock-On Tracking. Science Robotics
> 9(95), eadm7689 (2024). DOI: 10.1126/scirobotics.adm7689

## File format

A `.floz` file is a standard ZIP archive. You can open it with any ZIP tool, or
analyze it with the tools at <https://github.com/strawlab/flo-data-analysis>.
The archive typically contains:

- `README.md` — this file (stored uncompressed and first).
- `flo-metadata.yaml` — recording metadata (FLO git revision, creation time,
  timezone).
- `flo-config.yaml` — the FLO controller configuration in effect for this
  recording (geometry, Kalman filter and PID gains, calibration functions).
- `tracking_state.csv` — per-update tracking estimates and motor commands
  (estimated and observed pan/tilt/distance, motor commands, predictions).
- `centroid.csv` — subject image coordinates received from the camera.
- `motor_positions.csv` — measured motor/gimbal positions over time.
- `encoder_data.csv` — raw gimbal IMU and encoder readings.
- `encoder_offsets.csv` — gimbal encoder offsets used during the recording.
- `broadway.jsonl` — line-delimited JSON log of FLO events and commands.

Not every file is present in every recording; the exact set depends on the
hardware and configuration used.
"#,
        name = env!("CARGO_PKG_NAME"),
        version = env!("CARGO_PKG_VERSION"),
    )
}

impl WritingState {
    fn new(
        creation_time_local: chrono::DateTime<chrono::Local>,
        output_dirname: camino::Utf8PathBuf,
        encoder_offsets: Option<&GimbalEncoderOffsets>,
        config: &FloControllerConfig,
    ) -> Result<Self> {
        let creation_time = creation_time_local.with_timezone(creation_time_local.offset());
        let git_revision = env!("GIT_HASH").to_string();

        // create output dir
        std::fs::create_dir_all(&output_dirname)?;

        // Until we obtain the readme file handle, we have a small race
        // condition where another process could also open this directory.

        let readme_fd = {
            let readme_path = output_dirname.join(README_MD_FNAME);

            let mut fd = std::fs::File::create_new(readme_path)?;

            // Start and end it with some newlines so the text is more
            // readable.
            fd.write_all(readme_contents().as_bytes())?;
            Some(fd)
        };

        {
            let flo_metadata_path = output_dirname.join("flo-metadata.yaml");

            let metadata = FloMetadata {
                git_revision,
                creation_time,
                timezone: iana_time_zone::get_timezone()?,
            };
            let metadata_buf = serde_yaml::to_string(&metadata)?;

            let mut fd = std::fs::File::create(flo_metadata_path)?;
            fd.write_all(metadata_buf.as_bytes())?;
        }

        {
            let flo_config_path = output_dirname.join("flo-config.yaml");

            let config_buf = serde_yaml::to_string(config)?;

            let mut fd = std::fs::File::create(flo_config_path)?;
            fd.write_all(config_buf.as_bytes())?;
        }

        let centroid_wtr = {
            let mut csv_path = output_dirname.clone();
            csv_path.push(flo_core::CENTROID_FNAME);
            let wtr = Box::new(bufwriter(csv_path)?);
            csv::Writer::from_writer(wtr as Box<dyn Write + Send>)
        };

        let tracking_state_wtr = {
            let mut csv_path = output_dirname.clone();
            csv_path.push(flo_core::TRACKING_STATE_FNAME);
            let wtr = Box::new(bufwriter(csv_path)?);
            csv::Writer::from_writer(wtr as Box<dyn Write + Send>)
        };

        let motor_position_wtr = {
            let mut csv_path = output_dirname.clone();
            csv_path.push(flo_core::MOTOR_POSITIONS_FNAME);
            let wtr = Box::new(bufwriter(csv_path)?);
            csv::Writer::from_writer(wtr as Box<dyn Write + Send>)
        };

        let mut result = WritingState {
            output_dirname,
            readme_fd,
            centroid_wtr,
            tracking_state_wtr,
            motor_position_wtr,
            encoder_offsets_wtr: None,
            encoder_data_wtr: None,
            mavlink_data_wtr: None,
            broadway_data_wtr: None,
            extension_wtrs: std::collections::BTreeMap::new(),
        };

        if let Some(encoder_offsets) = encoder_offsets {
            result.save_encoder_offsets(encoder_offsets)?;
        }

        Ok(result)
    }

    /// Write a single per-event data message to the appropriate file.
    ///
    /// Used both for live messages while recording and for replaying the
    /// pre-capture buffer. Control messages (`Quit`, `ToggleSavingFloz`,
    /// `SetPreCaptureSeconds`, `GimbalEncoderOffsets`) are handled by the
    /// caller and never reach here.
    fn save_data_msg(&mut self, msg: SaveToDiskMsg) -> Result<()> {
        use SaveToDiskMsg::*;
        match msg {
            CentroidData((stamp, centroid_data)) => {
                let centroid_data = with_received_timestamp(centroid_data, stamp);
                self.save_centroid(centroid_data)?;
            }
            StampedTrackingState(stamped_tracking_state) => {
                self.save_stamped_tracking_state(*stamped_tracking_state)?;
            }
            MotorPosition(motor_position) => {
                self.save_motor_position(*motor_position)?;
            }
            GimbalEncoderData(encoder_data) => {
                self.save_encoder_data(&encoder_data)?;
            }
            MavlinkData(v) => {
                self.save_mavlink_data(&v)?;
            }
            BroadwaySaveToDiskMsg(msg) => {
                self.save_broadway_msg(msg)?;
            }
            ExtensionRecord {
                file_name,
                stamp,
                record,
            } => {
                self.save_extension_record(file_name, stamp, record)?;
            }
            other => {
                tracing::warn!("unexpected control message in save_data_msg: {other:?}");
            }
        }
        Ok(())
    }

    fn save_centroid(&mut self, centroid_data: StampedMomentCentroid) -> Result<()> {
        self.centroid_wtr.serialize(centroid_data)?;
        Ok(())
    }

    fn save_encoder_offsets(&mut self, encoder_offsets: &GimbalEncoderOffsets) -> Result<()> {
        if self.encoder_offsets_wtr.is_none() {
            let mut csv_path = self.output_dirname.clone();
            csv_path.push(flo_core::ENCODER_OFFSETS_FNAME);
            let wtr = Box::new(bufwriter(csv_path)?);
            self.encoder_offsets_wtr = Some(csv::Writer::from_writer(wtr as Box<dyn Write + Send>));
        }
        let encoder_offsets_wtr = self.encoder_offsets_wtr.as_mut().unwrap();
        encoder_offsets_wtr.serialize(encoder_offsets)?;
        Ok(())
    }

    fn save_encoder_data(&mut self, encoder_data: &GimbalEncoderData) -> Result<()> {
        if self.encoder_data_wtr.is_none() {
            let mut csv_path = self.output_dirname.clone();
            csv_path.push(flo_core::ENCODER_DATA_FNAME);
            let wtr = Box::new(bufwriter(csv_path)?);
            self.encoder_data_wtr = Some(csv::Writer::from_writer(wtr as Box<dyn Write + Send>));
        }
        let encoder_data_wtr = self.encoder_data_wtr.as_mut().unwrap();
        encoder_data_wtr.serialize(encoder_data)?;
        Ok(())
    }

    fn save_mavlink_data(&mut self, mavlink_data: &StampedJson) -> Result<()> {
        if self.mavlink_data_wtr.is_none() {
            let mut jsonl_path = self.output_dirname.clone();
            jsonl_path.push("mavlink.jsonl");
            let wtr = Box::new(bufwriter(jsonl_path)?);
            self.mavlink_data_wtr = Some(JsonLinesWriter::from_writer(wtr));
        }
        let mavlink_data_wtr = self.mavlink_data_wtr.as_mut().unwrap();
        mavlink_data_wtr.serialize(mavlink_data)?;
        Ok(())
    }

    fn save_broadway_msg(&mut self, msg: StampedBMsg) -> Result<()> {
        if self.broadway_data_wtr.is_none() {
            let mut jsonl_path = self.output_dirname.clone();
            jsonl_path.push("broadway.jsonl");
            let wtr = Box::new(bufwriter(jsonl_path)?);
            self.broadway_data_wtr = Some(JsonLinesWriter::from_writer(wtr));
        }
        let broadway_data_wtr = self.broadway_data_wtr.as_mut().unwrap();
        broadway_data_wtr.serialize(&msg)?;
        Ok(())
    }

    fn save_extension_record(
        &mut self,
        file_name: &'static str,
        stamp: chrono::DateTime<chrono::Local>,
        record: serde_json::Value,
    ) -> Result<()> {
        let wtr = match self.extension_wtrs.entry(file_name) {
            std::collections::btree_map::Entry::Occupied(e) => e.into_mut(),
            std::collections::btree_map::Entry::Vacant(e) => {
                let mut jsonl_path = self.output_dirname.clone();
                jsonl_path.push(format!("{file_name}.jsonl"));
                let buf: Box<dyn Write + Send> = Box::new(bufwriter(jsonl_path)?);
                e.insert(JsonLinesWriter::from_writer(buf))
            }
        };
        // Wrap the record so each line carries the local timestamp alongside
        // the extension's payload. Equivalent to StampedJson but the
        // extension-side type is opaque to this crate.
        let stamped = serde_json::json!({ "stamp": stamp, "record": record });
        wtr.serialize(&stamped)?;
        Ok(())
    }

    fn save_stamped_tracking_state(
        &mut self,
        stamped_tracking_state: flo_core::StampedTrackingState,
    ) -> Result<()> {
        self.tracking_state_wtr
            .serialize(flo_core::SaveTrackingState::from(stamped_tracking_state))?;
        Ok(())
    }

    fn save_motor_position(&mut self, motor_position: flo_core::MotorPositionResult) -> Result<()> {
        self.motor_position_wtr.serialize(motor_position)?;
        Ok(())
    }

    fn flush_all(&mut self) -> Result<()> {
        self.centroid_wtr.flush()?;
        self.tracking_state_wtr.flush()?;
        self.motor_position_wtr.flush()?;
        self.encoder_offsets_wtr
            .as_mut()
            .map(|x| x.flush())
            .transpose()?;
        self.encoder_data_wtr
            .as_mut()
            .map(|x| x.flush())
            .transpose()?;
        self.mavlink_data_wtr
            .as_mut()
            .map(|x| x.flush())
            .transpose()?;
        self.broadway_data_wtr
            .as_mut()
            .map(|x| x.flush())
            .transpose()?;
        for wtr in self.extension_wtrs.values_mut() {
            wtr.flush()?;
        }
        Ok(())
    }
}

fn bufwriter<P: AsRef<std::path::Path>>(path: P) -> std::io::Result<impl Write + Seek + Send> {
    Ok(std::io::BufWriter::new(std::fs::File::create(path)?))
}

fn dummy_csv() -> csv::Writer<Box<dyn Write + Send>> {
    let fd = Box::new(Vec::with_capacity(0));
    csv::Writer::from_writer(fd)
}

fn recording_zip_options(is_readme: bool) -> zip::write::SimpleFileOptions {
    let compression_method = if is_readme {
        zip::CompressionMethod::Stored
    } else {
        zip::CompressionMethod::Deflated
    };

    zip::write::SimpleFileOptions::default()
        .large_file(true)
        .unix_permissions(0o755)
        .compression_method(compression_method)
}

impl Drop for WritingState {
    #[tracing::instrument(skip_all)]
    fn drop(&mut self) {
        // Drop all files, which flushes and closes them. Unfortunately errors
        // will get swallowed, but lots of boilerplate would be required to
        // bubble them up. And still we cannot guarantee that   drop is called.
        {
            self.centroid_wtr = dummy_csv();
            self.tracking_state_wtr = dummy_csv();
            self.motor_position_wtr = dummy_csv();
            self.encoder_offsets_wtr = None;
            self.encoder_data_wtr = None;
            self.mavlink_data_wtr = None;
            self.broadway_data_wtr = None;
            self.extension_wtrs.clear();
        }

        // Move out original output name so that a subsequent call to `drop()`
        // doesn't accidentally overwrite our real data.
        let output_dirname = std::mem::take(&mut self.output_dirname);
        // Compress the saved directory into a .floz file.
        {
            // TODO: read all the (forward) kalman estimates and smooth them to
            // an additional file. If we do it here, it is done after the
            // realtime tracking and thus does not interfere with recording
            // data. On the other hand, if we smooth at the end of each
            // trajectory, those smoothing costs are amortized throughout the
            // experiment.

            let replace_extension = match output_dirname.extension() {
                Some(ext) => ext == "flo",
                None => false,
            };

            // compute the name of the zip file.
            let output_zipfile: camino::Utf8PathBuf = if replace_extension {
                output_dirname.with_extension("floz")
            } else {
                let mut tmp = output_dirname.clone().into_os_string();
                tmp.push(".floz");
                camino::Utf8PathBuf::from_os_string(tmp).unwrap()
            };

            tracing::info!("creating zip file {output_zipfile}");
            // zip the output_dirname directory
            {
                let mut file = bufwriter(output_zipfile).unwrap();

                let header = "FLOZ file. This is a standard ZIP file with a \
            specific schema.\n";
                file.write_all(header.as_bytes()).unwrap();

                let walkdir = walkdir::WalkDir::new(&output_dirname);

                // Store README.md first and without compression so that its
                // contents remain visible near the beginning of the archive.
                // Deflate the remaining files to keep normal recordings small.
                let mut readme_entry: Option<walkdir::DirEntry> = None;

                let mut files = Vec::new();
                for entry in walkdir.into_iter().filter_map(|e| e.ok()) {
                    if entry.file_name() == README_MD_FNAME {
                        readme_entry = Some(entry);
                    } else {
                        files.push(entry);
                    }
                }
                let mut zip_wtr = zip::ZipWriter::new(file);

                if let Some(entry) = readme_entry {
                    crate::zip_dir::zip_dir(
                        &mut std::iter::once(entry),
                        &output_dirname,
                        &mut zip_wtr,
                        recording_zip_options(true),
                    )
                    .expect("zip README.md");
                }

                crate::zip_dir::zip_dir(
                    &mut files.into_iter(),
                    &output_dirname,
                    &mut zip_wtr,
                    recording_zip_options(false),
                )
                .expect("zip remaining recording files");
                zip_wtr.finish().unwrap();
            }

            // Release the file so we no longer have exclusive access to the
            // directory. (Until we remove the directory, we have a small race
            // condition where another process could open the directory without
            // obtaining the readme file handle.)
            self.readme_fd = None;

            // Once the original directory is written successfully to a zip
            // file, we remove it.
            tracing::info!("done creating zip file, removing {output_dirname}");
            std::fs::remove_dir_all(&output_dirname).unwrap();
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn recording_zip_compression_keeps_readme_uncompressed() -> zip::result::ZipResult<()> {
        use std::io::Write as _;

        let mut zip_wtr = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
        zip_wtr.start_file("README.md", super::recording_zip_options(true))?;
        zip_wtr.write_all(b"read me")?;
        zip_wtr.start_file("data.csv", super::recording_zip_options(false))?;
        zip_wtr.write_all(b"some recorded data")?;

        let buf = zip_wtr.finish()?.into_inner();
        let mut archive = zip::ZipArchive::new(std::io::Cursor::new(buf))?;
        assert_eq!(
            archive.by_name("README.md")?.compression(),
            zip::CompressionMethod::Stored
        );
        assert_eq!(
            archive.by_name("data.csv")?.compression(),
            zip::CompressionMethod::Deflated
        );
        Ok(())
    }
}
