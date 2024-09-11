use std::{
    io::{Read, Seek, Write},
    path::PathBuf,
};

use color_eyre::eyre::{self, Result};
use flo_core::{
    FloatType, GimbalEncoderData, GimbalEncoderOffsets, MomentCentroid, RadialDistance,
    SaveToDiskMsg, StampedBMsg, StampedJson, StampedTrackingState, TimestampSource,
};
use serde::{Deserialize, Serialize};

use crate::json_lines_writer::JsonLinesWriter;

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
struct StampedMomentCentroid {
    received_timestamp: chrono::DateTime<chrono::Local>,
    schema_version: u8,
    framenumber: u32,
    timestamp_source: TimestampSource,
    timestamp: chrono::DateTime<chrono::Utc>,
    mu00: FloatType,
    mu01: FloatType,
    mu10: FloatType,
    center_x: u32,
    center_y: u32,
    cam_name: String,
}

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
        eyre::bail!("'received_timestamp' field already in MomentCentroid");
    }
    let _smc: StampedMomentCentroid = serde_json::from_value(json_value)?;
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
        self.flo_write_tx.send(SaveToDiskMsg::Quit).unwrap();
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
) -> Result<()> {
    use std::time::{Duration, Instant};
    use SaveToDiskMsg::*;

    let mut writing_state: Option<WritingState> = None;
    const FLUSH_INTERVAL: u64 = 1;
    let flush_interval = Duration::from_secs(FLUSH_INTERVAL);

    let mut last_flushed = Instant::now();

    tracing::debug!("Starting floz writer task. {}:{}", file!(), line!());

    let mut encoder_offsets = None;

    // Ideally we would have a timeout here, but this is not available.

    while let Some(msg) = flo_write_rx.blocking_recv() {
        tracing::debug!("processing message {msg:?}");
        match msg {
            Quit => {
                break;
            }
            CentroidData((stamp, centroid_data)) => {
                if let Some(ref mut ws) = writing_state {
                    let centroid_data = with_received_timestamp(centroid_data, stamp);
                    ws.save_centroid(centroid_data)?;
                }
            }
            StampedTrackingState(stamped_tracking_state) => {
                if let Some(ref mut ws) = writing_state {
                    ws.save_stamped_tracking_state(*stamped_tracking_state)?;
                }
            }
            MotorPosition(motor_position) => {
                if let Some(ref mut ws) = writing_state {
                    ws.save_motor_position(*motor_position)?;
                }
            }
            ToggleSavingCsv(values) => {
                if let Some((creation_time, output_dirname)) = values {
                    tracing::info!("Saving FLO data to {}", output_dirname.display());
                    if writing_state.is_none() {
                        writing_state = Some(WritingState::new(
                            creation_time,
                            output_dirname,
                            encoder_offsets.as_ref(),
                        )?);
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
            GimbalEncoderData(encoder_data) => {
                if let Some(ref mut ws) = writing_state.as_mut() {
                    ws.save_encoder_data(&encoder_data)?;
                }
            }
            MavlinkData(v) => {
                if let Some(ref mut ws) = writing_state.as_mut() {
                    ws.save_mavlink_data(&v)?;
                }
            }
            BroadwaySaveToDiskMsg(msg) => {
                if let Some(ref mut ws) = writing_state.as_mut() {
                    ws.save_broadway_msg(msg)?;
                }
            }
        }
        tracing::debug!("processing message done");

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

trait AsFloat {
    fn as_float(&self) -> f64;
}

impl AsFloat for Option<RadialDistance> {
    fn as_float(&self) -> f64 {
        match self {
            None => f64::NAN,
            Some(v) => v.0,
        }
    }
}

const README_MD_FNAME: &str = "README.md";

#[derive(Debug, PartialEq, Eq, Clone, Serialize, Deserialize)]
struct FloMetadata {
    git_revision: String,
    creation_time: chrono::DateTime<chrono::FixedOffset>,
    timezone: String,
}

// These are all f32 to keep file size down.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
struct SaveTrackingState {
    centroid_timestamp: Option<chrono::DateTime<chrono::Utc>>,
    est_pan: Option<f32>,
    est_tilt: Option<f32>,
    est_dist: Option<f32>,
    pan_obs: f32,
    tilt_obs: f32,
    dist_obs: f32,
    cam_pan: f32,
    cam_tilt: f32,
    motor_cmd_pan: f32,
    motor_cmd_tilt: f32,
    motor_cmd_focus: f32,
    pan_predicted: f32,
    tilt_predicted: f32,
    pan_error_integral: f32,
    tilt_error_integral: f32,
    processed_timestamp: chrono::DateTime<chrono::Local>,
}

impl From<StampedTrackingState> for SaveTrackingState {
    fn from(orig: StampedTrackingState) -> Self {
        let (est_pan, est_tilt) = if let Some(ke) = orig.tracking_state.kalman_estimates.as_ref() {
            (Some(ke.0.state()[0]), Some(ke.0.state()[1]))
        } else {
            (None, None)
        };

        let est_dist = orig
            .tracking_state
            .kalman_estimates_distance
            .as_ref()
            .map(|ke| ke.0.state()[0]);

        let to_f32 = |x: FloatType| x as f32;
        let (cam_pan, cam_tilt) = if let Some(pos) = orig.tracking_state.last_motor_readout {
            (to_f32(pos.pan_imu.0), to_f32(pos.tilt_imu.0))
        } else {
            (0.0, 0.0)
        };

        Self {
            centroid_timestamp: orig.centroid_timestamp,
            est_pan: est_pan.map(to_f32),
            est_tilt: est_tilt.map(to_f32),
            est_dist: est_dist.map(to_f32),
            pan_obs: to_f32(orig.tracking_state.pan_obs),
            tilt_obs: to_f32(orig.tracking_state.tilt_obs),
            dist_obs: to_f32(orig.tracking_state.dist_obs),
            cam_pan,
            cam_tilt,
            motor_cmd_pan: to_f32(orig.tracking_state.pan_command.as_float()),
            motor_cmd_tilt: to_f32(orig.tracking_state.tilt_command.as_float()),
            motor_cmd_focus: to_f32(orig.tracking_state.focus_command.as_float()),
            pan_predicted: to_f32(orig.tracking_state.pan.as_float()),
            tilt_predicted: to_f32(orig.tracking_state.tilt.as_float()),
            pan_error_integral: to_f32(orig.tracking_state.pan_err_integral),
            tilt_error_integral: to_f32(orig.tracking_state.tilt_err_integral),
            processed_timestamp: orig.processed_timestamp,
        }
    }
}

struct WritingState {
    output_dirname: std::path::PathBuf,
    /// The readme file in the output directory.
    ///
    /// We keep this file open to establish locking on the open directory.
    ///
    /// In theory, we might prefer an open reference to the directory itself,
    /// but this does not seem possible. So we have a potential slight race
    /// condition when we have our directory but not yet the file handle on
    /// readme.
    #[allow(dead_code)]
    readme_fd: Option<std::fs::File>,
    centroid_wtr: csv::Writer<Box<dyn Write + Send>>,
    tracking_state_wtr: csv::Writer<Box<dyn Write + Send>>,
    motor_position_wtr: csv::Writer<Box<dyn Write + Send>>,
    encoder_offsets_wtr: Option<csv::Writer<Box<dyn Write + Send>>>,
    encoder_data_wtr: Option<csv::Writer<Box<dyn Write + Send>>>,
    mavlink_data_wtr: Option<JsonLinesWriter<Box<dyn Write + Send>>>,
    broadway_data_wtr: Option<JsonLinesWriter<Box<dyn Write + Send>>>,
}

fn _test_writing_state_is_send() {
    // Compile-time test to ensure WritingState implements Send trait.
    fn implements<T: Send>() {}
    implements::<WritingState>();
}

fn readme_contents() -> String {
    format!("\n\nThis is data saved by {}.\n\n", env!("CARGO_PKG_NAME"))
}

impl WritingState {
    fn new(
        creation_time_local: chrono::DateTime<chrono::Local>,
        output_dirname: PathBuf,
        encoder_offsets: Option<&GimbalEncoderOffsets>,
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
            fd.write_all(readme_contents().as_bytes()).unwrap();
            Some(fd)
        };

        {
            let flo_metadata_path = output_dirname.join("flo-metadata.yaml");

            let metadata = FloMetadata {
                git_revision,
                creation_time,
                timezone: iana_time_zone::get_timezone()?,
            };
            let metadata_buf = serde_yaml::to_string(&metadata).unwrap();

            let mut fd = std::fs::File::create(flo_metadata_path)?;
            fd.write_all(metadata_buf.as_bytes()).unwrap();
        }

        let centroid_wtr = {
            let mut csv_path = output_dirname.clone();
            csv_path.push("centroid.csv");
            let wtr = Box::new(bufwriter(csv_path)?);
            csv::Writer::from_writer(wtr as Box<dyn Write + Send>)
        };

        let tracking_state_wtr = {
            let mut csv_path = output_dirname.clone();
            csv_path.push("tracking_state.csv");
            let wtr = Box::new(bufwriter(csv_path)?);
            csv::Writer::from_writer(wtr as Box<dyn Write + Send>)
        };

        let motor_position_wtr = {
            let mut csv_path = output_dirname.clone();
            csv_path.push("motor_positions.csv");
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
        };

        if let Some(encoder_offsets) = encoder_offsets {
            result.save_encoder_offsets(encoder_offsets)?;
        }

        Ok(result)
    }

    fn save_centroid(&mut self, centroid_data: StampedMomentCentroid) -> Result<()> {
        self.centroid_wtr.serialize(centroid_data)?;
        Ok(())
    }

    fn save_encoder_offsets(&mut self, encoder_offsets: &GimbalEncoderOffsets) -> Result<()> {
        if self.encoder_offsets_wtr.is_none() {
            let mut csv_path = self.output_dirname.clone();
            csv_path.push("encoder_offsets.csv");
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
            csv_path.push("encoder_data.csv");
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

    fn save_stamped_tracking_state(
        &mut self,
        stamped_tracking_state: flo_core::StampedTrackingState,
    ) -> Result<()> {
        self.tracking_state_wtr
            .serialize(SaveTrackingState::from(stamped_tracking_state))?;
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
        Ok(())
    }

    /// Open an existing directory
    ///
    /// This solely exists to close the directory and save it as a .floz file.
    fn from_existing_dir<P: AsRef<std::path::Path>>(existing_dir: P) -> Result<Self> {
        let existing_dir: std::path::PathBuf = std::path::PathBuf::from(existing_dir.as_ref());

        // Until we obtain the readme file handle, we have a small race
        // condition where another process could also open this directory.
        let readme_fd = {
            let readme_path = existing_dir.join(README_MD_FNAME);

            let mut fd = std::fs::File::open(readme_path)?;
            let mut actual_readme_contents = String::new();
            fd.read_to_string(&mut actual_readme_contents)?;
            if actual_readme_contents != readme_contents() {
                eyre::bail!("unexpected readme contents");
            }

            Some(fd)
        };

        Ok(Self {
            readme_fd,
            output_dirname: existing_dir,
            centroid_wtr: dummy_csv(),
            tracking_state_wtr: dummy_csv(),
            motor_position_wtr: dummy_csv(),
            encoder_offsets_wtr: None,
            encoder_data_wtr: None,
            mavlink_data_wtr: None,
            broadway_data_wtr: None,
        })
    }
}

fn bufwriter<P: AsRef<std::path::Path>>(path: P) -> std::io::Result<impl Write + Seek + Send> {
    Ok(std::io::BufWriter::new(std::fs::File::create(path)?))
}

pub(crate) fn fix_csv_file<P: AsRef<std::path::Path>>(csv_path: P) -> Result<()> {
    let newlen = {
        // Checks if CSV files are complete. If not, delete any final unfinished line.
        let raw_rdr = std::fs::File::open(&csv_path)?;
        let mut rdr = csv::Reader::from_reader(raw_rdr);
        let mut last_good_position = None;
        for record in rdr.byte_records() {
            let record = match record {
                Ok(record) => record,
                Err(csv_err) => {
                    last_good_position = csv_err.position().map(|pos| pos.byte());
                    break;
                }
            };
            last_good_position = record.position().map(|pos| pos.byte());
        }
        if let Some(last_good_position) = last_good_position {
            last_good_position - 1
        } else {
            // file may be empty
            return Ok(());
        }
    };
    let fd = std::fs::File::options()
        .read(true)
        .write(true)
        .open(&csv_path)?;
    fd.set_len(newlen)?;
    Ok(())
}

pub(crate) fn repair_unfinished_flo<P: AsRef<std::path::Path>>(existing_dir: P) -> Result<()> {
    // find CSV files
    let pattern = format!("{}/{}", existing_dir.as_ref().display(), "*.csv");
    for csv_path in glob::glob(&pattern)? {
        let csv_path = csv_path?;
        tracing::info!("fixing CSV file: {}", csv_path.display());
        fix_csv_file(&csv_path)?;
    }

    let ws = WritingState::from_existing_dir(existing_dir)?;
    std::mem::drop(ws);
    Ok(())
}

fn dummy_csv() -> csv::Writer<Box<dyn Write + Send>> {
    let fd = Box::new(Vec::with_capacity(0));
    csv::Writer::from_writer(fd)
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
            let output_zipfile: std::path::PathBuf = if replace_extension {
                output_dirname.with_extension("floz")
            } else {
                let mut tmp = output_dirname.clone().into_os_string();
                tmp.push(".floz");
                tmp.into()
            };

            tracing::info!("creating zip file {}", output_zipfile.display());
            // zip the output_dirname directory
            {
                let mut file = bufwriter(output_zipfile).unwrap();

                let header = "FLOZ file. This is a standard ZIP file with a \
            specific schema.\n";
                file.write_all(header.as_bytes()).unwrap();

                let walkdir = walkdir::WalkDir::new(&output_dirname);

                // Reorder the results to save the README_MD_FNAME file first
                // so that the first bytes of the file have it. This is why we
                // special-case the file here.
                let mut readme_entry: Option<walkdir::DirEntry> = None;

                let mut files = Vec::new();
                for entry in walkdir.into_iter().filter_map(|e| e.ok()) {
                    if entry.file_name() == README_MD_FNAME {
                        readme_entry = Some(entry);
                    } else {
                        files.push(entry);
                    }
                }
                if let Some(entry) = readme_entry {
                    files.insert(0, entry);
                }

                let mut zip_wtr = zip::ZipWriter::new(file);
                let options = zip::write::FileOptions::default()
                    .large_file(true)
                    .unix_permissions(0o755);

                crate::zip_dir::zip_dir(
                    &mut files.into_iter(),
                    &output_dirname,
                    &mut zip_wtr,
                    options,
                )
                .expect("zip_dir");
                zip_wtr.finish().unwrap();
            }

            // Release the file so we no longer have exclusive access to the
            // directory. (Until we remove the directory, we have a small race
            // condition where another process could open the directory without
            // obtaining the readme file handle.)
            self.readme_fd = None;

            // Once the original directory is written successfully to a zip
            // file, we remove it.
            tracing::info!(
                "done creating zip file, removing {}",
                output_dirname.display()
            );
            std::fs::remove_dir_all(&output_dirname).unwrap();
        }
    }
}
