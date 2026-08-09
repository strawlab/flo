//! The preview link: webcam frames sent from `camshow` back to `flo`.
//!
//! A *third* TCP connection, alongside the JSON-lines control link in the
//! parent module and the [`video`](crate::video) relay. It runs in the opposite
//! direction to everything else: camshow owns the webcam, so camshow is the
//! only process that can show flo's browser UI what the FPV camera sees.
//!
//! As with the video relay, camshow listens and flo connects, so all the
//! reconnect logic stays on flo's side and camshow's configuration remains a
//! list of ports it serves.
//!
//! Design constraints, which differ from the relay's:
//!
//! - **Nothing depends on this.** The preview is a convenience for whoever has
//!   the browser open. camshow decimates hard, drops rather than blocking, and
//!   carries on when no one is connected; flo shows no preview panel when no
//!   frames are arriving.
//! - **Downscaled at the source.** flo re-encodes these to JPEG for the
//!   browser, and a preview pane is a few hundred pixels wide. Sending full
//!   sensor resolution would spend bandwidth and CPU on detail that is thrown
//!   away twice over.
//! - **Self-describing per frame,** for the same reason the relay is: the
//!   camera's geometry can change while the link is up.

use crate::video::{MAX_VIDEO_PAYLOAD_LEN, WirePixelFormat};

/// Marks the start of every frame on the preview link. Distinct from the
/// relay's [`VIDEO_MAGIC`](crate::video::VIDEO_MAGIC) so connecting to the
/// wrong port fails immediately and says so.
pub const PREVIEW_MAGIC: [u8; 4] = *b"CSPF";

/// Bumped when this framing changes incompatibly. Independent of both the
/// control link's and the relay's versions.
pub const PREVIEW_WIRE_VERSION: u16 = 1;

/// Size of the fixed header preceding each frame's pixel data.
pub const PREVIEW_HEADER_LEN: usize = 32;

/// Default address camshow listens on, and flo connects to, for the preview
/// link. One above the video relay's port.
pub const DEFAULT_CAMSHOW_PREVIEW_ADDR: &str = "127.0.0.1:2226";

/// Errors decoding a preview frame header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum PreviewWireError {
    #[error("not a preview stream (magic {0:?})")]
    BadMagic([u8; 4]),
    #[error("preview wire version mismatch (peer={peer}, us={PREVIEW_WIRE_VERSION})")]
    VersionMismatch { peer: u16 },
    #[error("unknown pixel format {0}")]
    UnknownPixelFormat(u8),
    #[error("degenerate geometry {width}x{height}")]
    DegenerateGeometry { width: u32, height: u32 },
    #[error("stride {stride} is too small for {width} px of {pixel_format:?}")]
    StrideTooSmall {
        stride: u32,
        width: u32,
        pixel_format: WirePixelFormat,
    },
    #[error("payload length {payload_len} does not match stride {stride} * height {height}")]
    PayloadLengthMismatch {
        payload_len: u32,
        stride: u32,
        height: u32,
    },
    #[error("payload length {0} exceeds the maximum")]
    PayloadTooLarge(u32),
}

/// The fixed header preceding one preview frame's pixel data.
///
/// Encoded little-endian, pinned so a capture of the stream stays
/// interpretable even though both ends are normally on one machine.
///
/// No source field: a preview frame is always the webcam, which is the whole
/// point of this link. No frame number either — the firehose on flo's side
/// numbers what it sends to each browser, and nothing here needs to correlate
/// a preview frame with a recorded one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreviewFrameHeader {
    pub pixel_format: WirePixelFormat,
    pub width: u32,
    pub height: u32,
    /// Bytes per image row, which may exceed `width * bytes_per_pixel`.
    pub stride: u32,
    /// Capture time, in nanoseconds since the Unix epoch (UTC).
    pub timestamp_nanos: i64,
    /// Length of the pixel data following this header. Always
    /// `stride * height`; carried explicitly so a receiver can reject a
    /// nonsensical header before allocating.
    pub payload_len: u32,
}

impl PreviewFrameHeader {
    /// Builds a header for a frame of the given geometry, computing
    /// `payload_len` from it, and checks that the geometry is sane.
    pub fn new(
        pixel_format: WirePixelFormat,
        width: u32,
        height: u32,
        stride: u32,
        timestamp_nanos: i64,
    ) -> Result<Self, PreviewWireError> {
        let header = Self {
            pixel_format,
            width,
            height,
            stride,
            timestamp_nanos,
            payload_len: stride.saturating_mul(height),
        };
        header.validate()?;
        Ok(header)
    }

    fn validate(&self) -> Result<(), PreviewWireError> {
        if self.width == 0 || self.height == 0 {
            return Err(PreviewWireError::DegenerateGeometry {
                width: self.width,
                height: self.height,
            });
        }
        let min_stride = self.width * self.pixel_format.bytes_per_pixel();
        if self.stride < min_stride {
            return Err(PreviewWireError::StrideTooSmall {
                stride: self.stride,
                width: self.width,
                pixel_format: self.pixel_format,
            });
        }
        if self.payload_len > MAX_VIDEO_PAYLOAD_LEN {
            return Err(PreviewWireError::PayloadTooLarge(self.payload_len));
        }
        // Checked against the u64 product so a `stride * height` that overflows
        // u32 is rejected rather than wrapping to a plausible-looking length.
        if u64::from(self.payload_len) != u64::from(self.stride) * u64::from(self.height) {
            return Err(PreviewWireError::PayloadLengthMismatch {
                payload_len: self.payload_len,
                stride: self.stride,
                height: self.height,
            });
        }
        Ok(())
    }

    pub fn encode(&self) -> [u8; PREVIEW_HEADER_LEN] {
        let mut out = [0u8; PREVIEW_HEADER_LEN];
        out[0..4].copy_from_slice(&PREVIEW_MAGIC);
        out[4..6].copy_from_slice(&PREVIEW_WIRE_VERSION.to_le_bytes());
        out[6] = self.pixel_format as u8;
        // out[7] is padding, keeping the u32s that follow 4-byte aligned.
        out[8..12].copy_from_slice(&self.width.to_le_bytes());
        out[12..16].copy_from_slice(&self.height.to_le_bytes());
        out[16..20].copy_from_slice(&self.stride.to_le_bytes());
        out[20..28].copy_from_slice(&self.timestamp_nanos.to_le_bytes());
        out[28..32].copy_from_slice(&self.payload_len.to_le_bytes());
        out
    }

    pub fn decode(bytes: &[u8; PREVIEW_HEADER_LEN]) -> Result<Self, PreviewWireError> {
        let magic: [u8; 4] = bytes[0..4].try_into().expect("4 bytes");
        if magic != PREVIEW_MAGIC {
            return Err(PreviewWireError::BadMagic(magic));
        }
        let version = u16::from_le_bytes(bytes[4..6].try_into().expect("2 bytes"));
        if version != PREVIEW_WIRE_VERSION {
            return Err(PreviewWireError::VersionMismatch { peer: version });
        }
        let header = Self {
            pixel_format: WirePixelFormat::from_wire(bytes[6])
                .ok_or(PreviewWireError::UnknownPixelFormat(bytes[6]))?,
            width: u32::from_le_bytes(bytes[8..12].try_into().expect("4 bytes")),
            height: u32::from_le_bytes(bytes[12..16].try_into().expect("4 bytes")),
            stride: u32::from_le_bytes(bytes[16..20].try_into().expect("4 bytes")),
            timestamp_nanos: i64::from_le_bytes(bytes[20..28].try_into().expect("8 bytes")),
            payload_len: u32::from_le_bytes(bytes[28..32].try_into().expect("4 bytes")),
        };
        header.validate()?;
        Ok(header)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header() -> PreviewFrameHeader {
        PreviewFrameHeader::new(
            WirePixelFormat::Rgb8,
            640,
            360,
            640 * 3,
            1_760_000_000_123_456_789,
        )
        .unwrap()
    }

    #[test]
    fn header_roundtrips() {
        let original = header();
        let encoded = original.encode();
        assert_eq!(encoded.len(), PREVIEW_HEADER_LEN);
        assert_eq!(PreviewFrameHeader::decode(&encoded).unwrap(), original);
        assert_eq!(original.payload_len, 640 * 360 * 3);
    }

    #[test]
    fn payload_len_follows_a_padded_stride() {
        let padded = PreviewFrameHeader::new(WirePixelFormat::Mono8, 100, 10, 128, 0).unwrap();
        assert_eq!(padded.payload_len, 1280, "stride, not width, sets the size");
        assert_eq!(
            PreviewFrameHeader::decode(&padded.encode()).unwrap(),
            padded
        );
    }

    #[test]
    fn the_relays_framing_is_not_mistaken_for_a_preview() {
        let mut encoded = header().encode();
        encoded[0..4].copy_from_slice(&crate::video::VIDEO_MAGIC);
        assert!(matches!(
            PreviewFrameHeader::decode(&encoded),
            Err(PreviewWireError::BadMagic(_))
        ));
    }

    #[test]
    fn a_future_version_is_refused_rather_than_misread() {
        let mut encoded = header().encode();
        encoded[4..6].copy_from_slice(&(PREVIEW_WIRE_VERSION + 1).to_le_bytes());
        assert!(matches!(
            PreviewFrameHeader::decode(&encoded),
            Err(PreviewWireError::VersionMismatch { .. })
        ));
    }

    #[test]
    fn a_stride_too_small_for_the_pixel_format_is_refused() {
        // 3 bytes per pixel needs 300, not 100.
        assert!(matches!(
            PreviewFrameHeader::new(WirePixelFormat::Rgb8, 100, 10, 100, 0),
            Err(PreviewWireError::StrideTooSmall { .. })
        ));
    }

    #[test]
    fn a_degenerate_frame_is_refused() {
        assert!(matches!(
            PreviewFrameHeader::new(WirePixelFormat::Rgb8, 0, 10, 0, 0),
            Err(PreviewWireError::DegenerateGeometry { .. })
        ));
    }
}
