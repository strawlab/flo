use thiserror::Error;

use bytes::{BytesMut, buf::Buf};
use tokio_util::codec::{Decoder, Encoder};

use flo_core::UdpMsg;

#[derive(Error, Debug)]
pub(crate) enum Error {
    #[error("cbor error {0}")]
    Cbor(#[from] serde_cbor::Error),
    #[error("io error {0}")]
    Io(#[from] std::io::Error),
}

// -------------------------

/// This codec runs on the device (or emulated devices).
///
/// This codec decodes messages to the device and encodes messages to the host.
/// The wire format ([UdpMsg] as CBOR) lives in `flo-core` so that any sender
/// (e.g. `floz-replay`) stays byte-compatible with this decoder.
#[derive(Default)]
pub(crate) struct FloControllerUdpCodec {}

impl Decoder for FloControllerUdpCodec {
    type Item = UdpMsg;
    type Error = Error;

    fn decode(&mut self, buf: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if buf.is_empty() {
            Ok(None)
        } else {
            let msg = match flo_core::decode_udp_msg(&buf[..]) {
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

impl Encoder<UdpMsg> for FloControllerUdpCodec {
    type Error = Error;
    fn encode(&mut self, msg: UdpMsg, final_buf: &mut bytes::BytesMut) -> Result<(), Self::Error> {
        final_buf.extend_from_slice(&flo_core::encode_udp_msg(&msg)?);
        Ok(())
    }
}
