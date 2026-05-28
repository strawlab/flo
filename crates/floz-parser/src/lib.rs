use color_eyre::eyre::Result;

use std::{
    fs::File,
    io::{BufReader, Read, Seek},
};

pub struct FlozArchive<R: Read + Seek> {
    archive: zip_or_dir::ZipDirArchive<R>,
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

pub fn floz_parse_reader<R: Read + Seek>(
    rdr: R,
    display_name: String,
) -> Result<FlozArchive<R>> {
    let zs = zip_or_dir::ZipDirArchive::from_zip(rdr, display_name)?;
    let parsed = floz_parse(zs)?;
    Ok(parsed)
}

pub fn floz_parse_path<P: AsRef<std::path::Path>>(
    path: P,
) -> Result<FlozArchive<BufReader<File>>> {
    let zs = zip_or_dir::ZipDirArchive::auto_from_path(&path)?;
    let parsed = floz_parse(zs)?;
    Ok(parsed)
}

pub fn floz_parse<R: Read + Seek>(
    archive: zip_or_dir::ZipDirArchive<R>,
) -> Result<FlozArchive<R>> {
    Ok(FlozArchive {
        archive,
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
