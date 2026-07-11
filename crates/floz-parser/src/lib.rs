use color_eyre::eyre::Result;

use std::{
    fs::File,
    io::{BufRead, BufReader, Read, Seek},
};

pub struct FlozArchive<R: Read + Seek> {
    archive: zip_or_dir::ZipDirArchive<R>,
    pub motor_positions: Vec<flo_core::MotorPositionResult>,
    pub tracking_states: Vec<flo_core::SaveTrackingState>,
    pub centroids: Vec<flo_core::StampedMomentCentroid>,
    /// Raw gimbal encoder data. Only present for gimbal recordings.
    pub encoder_data: Vec<flo_core::GimbalEncoderData>,
    /// Gimbal encoder offset calibration. Only present for gimbal recordings.
    pub encoder_offsets: Vec<flo_core::GimbalEncoderOffsets>,
    /// The controller configuration in effect during the recording.
    ///
    /// `None` if the archive has no `flo-config.yaml`, or if it could not be
    /// deserialized (e.g. it was written by an incompatible version). A failure
    /// to parse the config is logged but not treated as fatal, since the
    /// recorded data tables are still usable on their own.
    pub config: Option<flo_core::FloControllerConfig>,
}

impl<R: Read + Seek> FlozArchive<R> {
    /// Consume and return the raw storage archive.
    pub fn into_inner(self) -> zip_or_dir::ZipDirArchive<R> {
        self.archive
    }

    /// Display the path to the archive.
    pub fn display(&self) -> std::path::Display<'_> {
        self.archive.display()
    }

    /// Get the path to the archive.
    pub fn path(&self) -> &std::path::Path {
        self.archive.path()
    }
}

pub fn floz_parse_reader<R: Read + Seek>(rdr: R, display_name: String) -> Result<FlozArchive<R>> {
    let zs = zip_or_dir::ZipDirArchive::from_zip(rdr, display_name)?;
    let parsed = floz_parse(zs)?;
    Ok(parsed)
}

pub fn floz_parse_path<P: AsRef<std::path::Path>>(path: P) -> Result<FlozArchive<BufReader<File>>> {
    let zs = zip_or_dir::ZipDirArchive::auto_from_path(&path)?;
    let parsed = floz_parse(zs)?;
    Ok(parsed)
}

/// Read and deserialize every row of a CSV file within the archive.
fn read_csv_table<R, T>(archive: &mut zip_or_dir::ZipDirArchive<R>, fname: &str) -> Result<Vec<T>>
where
    R: Read + Seek,
    T: serde::de::DeserializeOwned,
{
    let path = archive.path_starter().join(fname);
    let rdr = path.open()?;
    let mut rows = Vec::new();
    for row in csv::Reader::from_reader(rdr).into_deserialize() {
        rows.push(row?);
    }
    Ok(rows)
}

/// Like [read_csv_table], but returns an empty `Vec` if the file is absent.
///
/// Some tables (e.g. the gimbal encoder data) are only written for certain
/// recording setups, so their absence is not an error.
fn read_optional_csv_table<R, T>(
    archive: &mut zip_or_dir::ZipDirArchive<R>,
    fname: &str,
) -> Result<Vec<T>>
where
    R: Read + Seek,
    T: serde::de::DeserializeOwned,
{
    if archive.is_file(fname) {
        read_csv_table(archive, fname)
    } else {
        Ok(Vec::new())
    }
}

pub fn floz_parse<R: Read + Seek>(
    mut archive: zip_or_dir::ZipDirArchive<R>,
) -> Result<FlozArchive<R>> {
    let motor_positions = read_csv_table(&mut archive, flo_core::MOTOR_POSITIONS_FNAME)?;
    let tracking_states = read_csv_table(&mut archive, flo_core::TRACKING_STATE_FNAME)?;
    let centroids = read_csv_table(&mut archive, flo_core::CENTROID_FNAME)?;
    let encoder_data = read_optional_csv_table(&mut archive, flo_core::ENCODER_DATA_FNAME)?;
    let encoder_offsets = read_optional_csv_table(&mut archive, flo_core::ENCODER_OFFSETS_FNAME)?;
    let config = read_optional_config(&mut archive, flo_core::FLO_CONFIG_FNAME);

    Ok(FlozArchive {
        archive,
        motor_positions,
        tracking_states,
        centroids,
        encoder_data,
        encoder_offsets,
        config,
    })
}

/// Read and deserialize the controller config, returning `None` if it is absent
/// or cannot be parsed.
///
/// We deliberately swallow parse errors (after logging) rather than propagate
/// them: the config is supplementary diagnostic information and an unparseable
/// config from a different software version should not prevent inspecting the
/// recorded data tables.
fn read_optional_config<R: Read + Seek>(
    archive: &mut zip_or_dir::ZipDirArchive<R>,
    fname: &str,
) -> Option<flo_core::FloControllerConfig> {
    if !archive.is_file(fname) {
        return None;
    }
    let result = (|| -> Result<flo_core::FloControllerConfig> {
        let rdr = archive.path_starter().join(fname).open()?;
        let value: serde_yaml::Value = serde_yaml::from_reader(rdr)?;
        Ok(deserialize_config_compat(value)?)
    })();
    match result {
        Ok(config) => Some(config),
        Err(e) => {
            tracing::warn!("Could not parse {fname}: {e}");
            None
        }
    }
}

/// Deserialize a recorded config, accepting fields written by known older FLO
/// versions without weakening strict parsing for current configs.
fn deserialize_config_compat(
    mut value: serde_yaml::Value,
) -> Result<flo_core::FloControllerConfig, serde_yaml::Error> {
    match serde_yaml::from_value(value.clone()) {
        Ok(config) => Ok(config),
        Err(original_error) => {
            let Some(osd_config) = value
                .as_mapping_mut()
                .and_then(|root| root.get_mut("osd_config"))
                .and_then(serde_yaml::Value::as_mapping_mut)
            else {
                return Err(original_error);
            };
            if osd_config.remove("display_webcam").is_none() {
                return Err(original_error);
            }
            serde_yaml::from_value(value)
        }
    }
}

#[cfg(test)]
mod config_compat_tests {
    #[test]
    fn config_compat_accepts_legacy_display_webcam() {
        let config = flo_core::FloControllerConfig::default();
        let mut value = serde_yaml::to_value(&config).unwrap();
        value["osd_config"] = serde_yaml::from_str(
            "display_webcam:\n  windowed: false\n  fpv_cam_human_name: legacy camera\n",
        )
        .unwrap();

        assert!(super::deserialize_config_compat(value).is_ok());
    }

    #[test]
    fn config_compat_still_rejects_unknown_osd_fields() {
        let config = flo_core::FloControllerConfig::default();
        let mut value = serde_yaml::to_value(&config).unwrap();
        value["osd_config"] = serde_yaml::from_str("unexpected_field: true\n").unwrap();

        assert!(super::deserialize_config_compat(value).is_err());
    }
}

/// Read the `broadway.jsonl` event log from the archive, if present.
///
/// Returns the successfully-parsed [`flo_core::StampedBMsg`] events. Lines that
/// fail to deserialize (e.g. from a newer schema) are skipped with a logged
/// warning rather than aborting, since callers typically need only a subset of
/// event types. Returns an empty `Vec` if the log is absent.
pub fn read_broadway_log<R: Read + Seek>(
    archive: &mut zip_or_dir::ZipDirArchive<R>,
) -> Result<Vec<flo_core::StampedBMsg>> {
    if !archive.is_file(flo_core::BROADWAY_FNAME) {
        return Ok(Vec::new());
    }
    let rdr = archive
        .path_starter()
        .join(flo_core::BROADWAY_FNAME)
        .open()?;
    let rdr = BufReader::new(rdr);
    let mut events = Vec::new();
    let mut num_skipped = 0usize;
    for line in rdr.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        match serde_json::from_str::<flo_core::StampedBMsg>(&line) {
            Ok(ev) => events.push(ev),
            Err(_) => num_skipped += 1,
        }
    }
    if num_skipped > 0 {
        tracing::warn!(
            "Skipped {num_skipped} unparseable line(s) in {}",
            flo_core::BROADWAY_FNAME
        );
    }
    Ok(events)
}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
