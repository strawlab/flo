use color_eyre::eyre::Result;

use std::{
    fs::File,
    io::{BufReader, Read, Seek},
};

pub struct FlozArchive<R: Read + Seek> {
    archive: zip_or_dir::ZipDirArchive<R>,
    pub motor_positions: Vec<flo_core::MotorPositionResult>,
    pub tracking_states: Vec<flo_core::SaveTrackingState>,
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

pub fn floz_parse<R: Read + Seek>(
    mut archive: zip_or_dir::ZipDirArchive<R>,
) -> Result<FlozArchive<R>> {
    // Open main motor positions data.
    let motor_positions = {
        let motor_positions_fname = archive.path_starter().join(flo_core::MOTOR_POSITIONS_FNAME);

        let rdr = motor_positions_fname.open()?;
        let motor_positions_rdr = csv::Reader::from_reader(rdr);

        let mut motor_positions = Vec::new();
        for row in motor_positions_rdr.into_deserialize() {
            let row: flo_core::MotorPositionResult = row?;
            motor_positions.push(row);
        }
        motor_positions
    };

    let tracking_states = {
        let tracking_states_fname = archive.path_starter().join(flo_core::TRACKING_STATE_FNAME);

        let rdr = tracking_states_fname.open()?;
        let tracking_states_rdr = csv::Reader::from_reader(rdr);

        let mut tracking_states = Vec::new();
        for row in tracking_states_rdr.into_deserialize() {
            let row: flo_core::SaveTrackingState = row?;
            tracking_states.push(row);
        }
        tracking_states
    };

    Ok(FlozArchive {
        archive,
        motor_positions,
        tracking_states,
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
