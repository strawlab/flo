//! Shared writer for FLOZ archives.

use std::io::{Seek, Write};

pub const FLOZ_HEADER: &str = "FLOZ file. This is a standard ZIP file with a specific schema.\n";
pub const README_FNAME: &str = "README.md";

pub struct FlozWriter<W: Write + Seek> {
    inner: zip::ZipWriter<W>,
    compression_level: Option<i64>,
    entry_count: usize,
}

impl<W: Write + Seek> FlozWriter<W> {
    pub fn new(mut writer: W, compression_level: Option<i64>) -> zip::result::ZipResult<Self> {
        writer.write_all(FLOZ_HEADER.as_bytes())?;
        Ok(Self {
            inner: zip::ZipWriter::new(writer),
            compression_level,
            entry_count: 0,
        })
    }

    pub fn start_file(&mut self, name: &str) -> zip::result::ZipResult<()> {
        if self.entry_count == 0 && name != README_FNAME {
            return Err(zip::result::ZipError::InvalidArchive(
                "README.md must be the first FLOZ entry".into(),
            ));
        }
        if self.entry_count != 0 && name == README_FNAME {
            return Err(zip::result::ZipError::InvalidArchive(
                "README.md must only occur as the first FLOZ entry".into(),
            ));
        }

        let options = self.options(name == README_FNAME);
        self.inner.start_file(name, options)?;
        self.entry_count += 1;
        Ok(())
    }

    pub fn add_directory(&mut self, name: &str) -> zip::result::ZipResult<()> {
        self.inner.add_directory(name, self.options(false))
    }

    pub fn finish(self) -> zip::result::ZipResult<W> {
        self.inner.finish()
    }

    fn options(&self, is_readme: bool) -> zip::write::SimpleFileOptions {
        let method = if is_readme {
            zip::CompressionMethod::Stored
        } else {
            zip::CompressionMethod::Deflated
        };
        let level = if is_readme {
            None
        } else {
            self.compression_level
        };

        zip::write::SimpleFileOptions::default()
            .large_file(true)
            .unix_permissions(0o755)
            .compression_method(method)
            .compression_level(level)
    }
}

impl<W: Write + Seek> Write for FlozWriter<W> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.inner.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn readme_is_first_and_stored() -> zip::result::ZipResult<()> {
        let mut writer = FlozWriter::new(std::io::Cursor::new(Vec::new()), Some(6))?;
        writer.start_file(README_FNAME)?;
        writer.write_all(b"read me")?;
        writer.start_file("data.csv")?;
        writer.write_all(b"some repeated data; some repeated data")?;

        let buf = writer.finish()?.into_inner();
        assert!(buf.starts_with(FLOZ_HEADER.as_bytes()));
        let mut archive = zip::ZipArchive::new(std::io::Cursor::new(buf))?;
        assert_eq!(archive.by_index(0)?.name(), README_FNAME);
        assert_eq!(
            archive.by_name(README_FNAME)?.compression(),
            zip::CompressionMethod::Stored
        );
        assert_eq!(
            archive.by_name("data.csv")?.compression(),
            zip::CompressionMethod::Deflated
        );
        Ok(())
    }

    #[test]
    fn rejects_data_before_readme() {
        let mut writer = FlozWriter::new(std::io::Cursor::new(Vec::new()), None).unwrap();
        assert!(writer.start_file("data.csv").is_err());
    }
}
