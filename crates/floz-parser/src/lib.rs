use color_eyre::eyre::Result;

use std::{
    fs::File,
    io::{BufReader, Read, Seek},
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
fn read_csv_table<R, T>(
    archive: &mut zip_or_dir::ZipDirArchive<R>,
    fname: &str,
) -> Result<Vec<T>>
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

    Ok(FlozArchive {
        archive,
        motor_positions,
        tracking_states,
        centroids,
        encoder_data,
        encoder_offsets,
    })
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
