use thiserror::Error;

use bytes::{buf::Buf, BytesMut};
use tokio_util::codec::{Decoder, Encoder};

use crate::UdpMsg;

use serde::Serialize;

#[derive(Error, Debug)]
pub(crate) enum Error {
    #[error("cbor error {0}")]
    Cbor(#[from] serde_cbor::Error),
    #[error("io error {0}")]
    Io(#[from] std::io::Error),
}

// We encode `T` and not `&T` because we do not want to deal with
// the lifetime issues (this is used in async contexts.)
fn do_encode_cbor<T: Serialize>(msg: T, final_buf: &mut bytes::BytesMut) -> Result<(), Error> {
    // TODO: why doesn't this work
    // serde_cbor::to_writer(&mut final_buf,&msg)?;
    let v = serde_cbor::to_vec(&msg)?;
    final_buf.extend_from_slice(v.as_slice());
    Ok(())
}

// -------------------------

/// This codec runs on the device (or emulated devices).
///
/// This codec decodes messages to the device and encodes messages to the host.
#[derive(Default)]
pub(crate) struct FloControllerUdpCodec {}

impl Decoder for FloControllerUdpCodec {
    type Item = UdpMsg;
    type Error = Error;

    fn decode(&mut self, buf: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if buf.is_empty() {
            Ok(None)
        } else {
            let msg = match serde_cbor::from_slice(&buf[..]) {
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
impl Encoder<UdpMsg> for FloControllerUdpCodec {
    type Error = Error;
    fn encode(&mut self, msg: UdpMsg, final_buf: &mut bytes::BytesMut) -> Result<(), Self::Error> {
        do_encode_cbor(msg, final_buf)
    }
}
