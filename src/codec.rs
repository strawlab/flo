use bytes::{buf::Buf, BytesMut};
use tokio_util::codec::{Decoder, Encoder};

use super::*;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("serde JSON error {0}")]
    SerdeJson(#[from] serde_json::Error),
    #[error("io error {0}")]
    Io(#[from] std::io::Error),
    #[error("JSON representation contained newline")]
    NewlineInData,
}

// -------------------------

/// JSON Lines text format, also called newline-delimited JSON.
#[derive(Default)]
pub struct JsonLinesCodec {}

impl JsonLinesCodec {
    pub fn new() -> Self {
        Self {}
    }
}

impl Decoder for JsonLinesCodec {
    type Item = PwmSerial;
    type Error = Error;

    fn decode(&mut self, buf: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if buf.is_empty() {
            Ok(None)
        } else {
            let msg = match serde_json::from_slice(&buf[..]) {
                Ok(msg) => msg,
                Err(e) => {
                    // If decode fails, we should still advance the buffer.
                    //
                    // In case of error, we want to advance our place in the buffer so that
                    // we don't attempt to re-parse this bad data again.
                    buf.advance(buf.remaining());
                    return Err(e.into());
                }
            };
            buf.advance(buf.len());
            Ok(Some(msg))
        }
    }
}

// We encode `T` and not `&T` because we do not want to deal with
// the lifetime issues (this is used in async contexts.)
impl Encoder<PwmSerial> for JsonLinesCodec {
    type Error = Error;
    fn encode(
        &mut self,
        msg: PwmSerial,
        final_buf: &mut bytes::BytesMut,
    ) -> Result<(), Self::Error> {
        let mut v = serde_json::to_vec(&msg)?;
        if memchr::memchr2(b'\n', b'\r', &v).is_some() {
            return Err(Error::NewlineInData);
        }
        v.push(b'\n');
        final_buf.extend_from_slice(v.as_slice());
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use color_eyre::eyre as anyhow;

    #[test]
    fn test_roundtrip() -> anyhow::Result<()> {
        for orig in &[
            PwmSerial::VersionRequest,
            PwmSerial::VersionResponse(flo_core::DATATYPES_VERSION),
        ] {
            let mut codec = JsonLinesCodec {};
            let mut buf = bytes::BytesMut::new();
            codec.encode(orig.clone(), &mut buf)?;
            dbg!((&orig, String::from_utf8_lossy(buf.chunk())));
            let decoded = codec.decode(&mut buf)?.unwrap();
            assert_eq!(orig, &decoded);
        }
        Ok(())
    }

    #[test]
    fn test_hardcoded() -> anyhow::Result<()> {
        use bytes::BufMut;
        let mut codec = JsonLinesCodec {};
        let mut buf = bytes::BytesMut::new();
        buf.put(flo_core::VERSION_RESPONSE_JSON_NEWLINE);
        let decoded = codec.decode(&mut buf)?.unwrap();
        assert_eq!(
            PwmSerial::VersionResponse(flo_core::DATATYPES_VERSION),
            decoded
        );
        Ok(())
    }
}
