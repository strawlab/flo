use serde::Serialize;
use std::io::{Result, Write};

pub(crate) struct JsonLinesWriter<W> {
    wtr: W,
}

impl<W> JsonLinesWriter<W>
where
    W: Write,
{
    pub(crate) fn from_writer(wtr: W) -> Self {
        Self { wtr }
    }

    pub(crate) fn serialize<S: Serialize>(&mut self, value: S) -> Result<()> {
        let buf = serde_json::to_vec(&value)?;
        if buf.contains(&b'\n') {
            return Err(std::io::Error::other("JSON lines cannot have newline"));
        }
        self.wtr.write_all(&buf)?;
        self.wtr.write_all(b"\n")
    }

    pub(crate) fn flush(&mut self) -> Result<()> {
        self.wtr.flush()
    }
}
