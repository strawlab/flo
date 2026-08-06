//! The video link: raw frames relayed from `flo-strand-cam` to `camshow`.
//!
//! This is a *second*, separate TCP connection from the JSON-lines control
//! link in the parent module. Multi-megabyte frames have no business being
//! base64'd into JSON, so they get their own socket with a fixed binary header.
//! camshow listens on both; flo connects to both and reconnects to each
//! independently.
//!
//! The frames carried here are for the operator's live view only. What camshow
//! records to disk is always its own clean FPV webcam image.
//!
//! Design constraints:
//!
//! - **Mono stays mono.** A tracking camera's Mono8 frame is sent at one byte
//!   per pixel and replicated to RGB8 by the receiver. Converting before
//!   sending would triple the bandwidth for no added information.
//! - **Lossy, latest-frame-wins.** The sender decimates to display rate and
//!   drops rather than blocking; the receiver keeps only the newest frame. The
//!   relayed camera is flight-critical, so nothing on this path may exert
//!   backpressure on acquisition.
//! - **Self-describing per frame.** Geometry and pixel format are on every
//!   frame, not negotiated once, because a camera can be reconfigured while the
//!   link is up.

use flo_core::DisplaySource;

/// Marks the start of every frame on the video link. Lets a receiver fail
/// loudly if something other than this protocol connects to the port.
pub const VIDEO_MAGIC: [u8; 4] = *b"CSVF";

/// Bumped when the binary video framing changes incompatibly. Independent of
/// the control link's [`PROTOCOL_VERSION`](crate::PROTOCOL_VERSION): the two
/// links can version separately.
pub const VIDEO_WIRE_VERSION: u16 = 1;

/// Size of the fixed header preceding each frame's pixel data.
pub const VIDEO_HEADER_LEN: usize = 40;

/// Default address camshow listens on, and flo connects to, for the video link.
/// One above the control link's port.
pub const DEFAULT_CAMSHOW_VIDEO_ADDR: &str = "127.0.0.1:2225";

/// Refuse a payload larger than this. A corrupt or hostile header must not make
/// the receiver allocate an arbitrary amount of memory. Comfortably above any
/// real frame: 8K RGB8 is about 100 MB.
pub const MAX_VIDEO_PAYLOAD_LEN: u32 = 256 * 1024 * 1024;

/// Pixel formats the video link carries.
///
/// Deliberately a short, closed list rather than strand-braid's full `PixFmt`:
/// the sender converts anything exotic to RGB8, so the receiver needs no
/// knowledge of Bayer patterns or packed YUV.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WirePixelFormat {
    /// One byte per pixel, as a tracking camera delivers it.
    Mono8 = 0,
    /// Three bytes per pixel, R then G then B.
    Rgb8 = 1,
}

impl WirePixelFormat {
    pub fn bytes_per_pixel(self) -> u32 {
        match self {
            Self::Mono8 => 1,
            Self::Rgb8 => 3,
        }
    }

    fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Mono8),
            1 => Some(Self::Rgb8),
            _ => None,
        }
    }
}

/// Why a received header was rejected.
///
/// Every variant is fatal for the connection: the framing is fixed-size, so
/// there is no way to resynchronize within a stream once a header does not
/// parse. The receiver drops the connection and waits for flo to reconnect.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum VideoWireError {
    #[error("not a camshow video stream (magic was {0:02x?}, expected {VIDEO_MAGIC:02x?})")]
    BadMagic([u8; 4]),
    #[error("video wire version mismatch (peer={peer}, us={VIDEO_WIRE_VERSION})")]
    VersionMismatch { peer: u16 },
    #[error("unknown video source id {0}")]
    UnknownSource(u8),
    #[error("the webcam is never relayed over the video link; camshow captures it itself")]
    WebcamSource,
    #[error("unknown video pixel format id {0}")]
    UnknownPixelFormat(u8),
    #[error("frame geometry is degenerate ({width}x{height})")]
    DegenerateGeometry { width: u32, height: u32 },
    #[error("stride {stride} is too small for {width} pixels of {pixel_format:?}")]
    StrideTooSmall {
        stride: u32,
        width: u32,
        pixel_format: WirePixelFormat,
    },
    #[error("payload length {payload_len} does not match stride {stride} x height {height}")]
    PayloadLengthMismatch {
        payload_len: u32,
        stride: u32,
        height: u32,
    },
    #[error("payload length {0} exceeds the {MAX_VIDEO_PAYLOAD_LEN} byte limit")]
    PayloadTooLarge(u32),
}

/// The fixed header preceding one frame's pixel data.
///
/// Encoded little-endian. Both ends are on the same machine (the link is
/// loopback by design), but the byte order is pinned anyway so a capture of the
/// stream stays interpretable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VideoFrameHeader {
    /// Which camera produced this frame. Never [`DisplaySource::Webcam`].
    pub source: DisplaySource,
    pub pixel_format: WirePixelFormat,
    pub width: u32,
    pub height: u32,
    /// Bytes per image row, which may exceed `width * bytes_per_pixel` when the
    /// camera pads its rows.
    pub stride: u32,
    /// The sending camera's host frame number, for logging and gap detection.
    /// Not contiguous: the sender decimates.
    pub frame_number: u64,
    /// Acquisition time, in nanoseconds since the Unix epoch (UTC).
    pub timestamp_nanos: i64,
    /// Length of the pixel data following this header. Always
    /// `stride * height`; carried explicitly so a receiver can read the payload
    /// without re-deriving it, and reject a nonsensical header before
    /// allocating.
    pub payload_len: u32,
}

fn source_to_wire(source: DisplaySource) -> u8 {
    match source {
        DisplaySource::Webcam => 0,
        DisplaySource::StrandCamMain => 1,
        DisplaySource::StrandCamSecondary => 2,
    }
}

fn source_from_wire(byte: u8) -> Result<DisplaySource, VideoWireError> {
    match byte {
        0 => Err(VideoWireError::WebcamSource),
        1 => Ok(DisplaySource::StrandCamMain),
        2 => Ok(DisplaySource::StrandCamSecondary),
        other => Err(VideoWireError::UnknownSource(other)),
    }
}

impl VideoFrameHeader {
    /// Builds a header for a frame of the given geometry, computing
    /// `payload_len` from it, and checks that the geometry is sane.
    pub fn new(
        source: DisplaySource,
        pixel_format: WirePixelFormat,
        width: u32,
        height: u32,
        stride: u32,
        frame_number: u64,
        timestamp_nanos: i64,
    ) -> Result<Self, VideoWireError> {
        let header = Self {
            source,
            pixel_format,
            width,
            height,
            stride,
            frame_number,
            timestamp_nanos,
            payload_len: stride.saturating_mul(height),
        };
        header.validate()?;
        Ok(header)
    }

    fn validate(&self) -> Result<(), VideoWireError> {
        if self.source == DisplaySource::Webcam {
            return Err(VideoWireError::WebcamSource);
        }
        if self.width == 0 || self.height == 0 {
            return Err(VideoWireError::DegenerateGeometry {
                width: self.width,
                height: self.height,
            });
        }
        let min_stride = self.width * self.pixel_format.bytes_per_pixel();
        if self.stride < min_stride {
            return Err(VideoWireError::StrideTooSmall {
                stride: self.stride,
                width: self.width,
                pixel_format: self.pixel_format,
            });
        }
        if self.payload_len > MAX_VIDEO_PAYLOAD_LEN {
            return Err(VideoWireError::PayloadTooLarge(self.payload_len));
        }
        // Checked against the u64 product so a `stride * height` that overflows
        // u32 is rejected rather than wrapping to a plausible-looking length.
        if u64::from(self.payload_len) != u64::from(self.stride) * u64::from(self.height) {
            return Err(VideoWireError::PayloadLengthMismatch {
                payload_len: self.payload_len,
                stride: self.stride,
                height: self.height,
            });
        }
        Ok(())
    }

    pub fn encode(&self) -> [u8; VIDEO_HEADER_LEN] {
        let mut out = [0u8; VIDEO_HEADER_LEN];
        out[0..4].copy_from_slice(&VIDEO_MAGIC);
        out[4..6].copy_from_slice(&VIDEO_WIRE_VERSION.to_le_bytes());
        out[6] = source_to_wire(self.source);
        out[7] = self.pixel_format as u8;
        out[8..12].copy_from_slice(&self.width.to_le_bytes());
        out[12..16].copy_from_slice(&self.height.to_le_bytes());
        out[16..20].copy_from_slice(&self.stride.to_le_bytes());
        out[20..28].copy_from_slice(&self.frame_number.to_le_bytes());
        out[28..36].copy_from_slice(&self.timestamp_nanos.to_le_bytes());
        out[36..40].copy_from_slice(&self.payload_len.to_le_bytes());
        out
    }

    pub fn decode(bytes: &[u8; VIDEO_HEADER_LEN]) -> Result<Self, VideoWireError> {
        let magic: [u8; 4] = bytes[0..4].try_into().expect("4 bytes");
        if magic != VIDEO_MAGIC {
            return Err(VideoWireError::BadMagic(magic));
        }
        let version = u16::from_le_bytes(bytes[4..6].try_into().expect("2 bytes"));
        if version != VIDEO_WIRE_VERSION {
            return Err(VideoWireError::VersionMismatch { peer: version });
        }
        let header = Self {
            source: source_from_wire(bytes[6])?,
            pixel_format: WirePixelFormat::from_wire(bytes[7])
                .ok_or(VideoWireError::UnknownPixelFormat(bytes[7]))?,
            width: u32::from_le_bytes(bytes[8..12].try_into().expect("4 bytes")),
            height: u32::from_le_bytes(bytes[12..16].try_into().expect("4 bytes")),
            stride: u32::from_le_bytes(bytes[16..20].try_into().expect("4 bytes")),
            frame_number: u64::from_le_bytes(bytes[20..28].try_into().expect("8 bytes")),
            timestamp_nanos: i64::from_le_bytes(bytes[28..36].try_into().expect("8 bytes")),
            payload_len: u32::from_le_bytes(bytes[36..40].try_into().expect("4 bytes")),
        };
        header.validate()?;
        Ok(header)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header() -> VideoFrameHeader {
        VideoFrameHeader::new(
            DisplaySource::StrandCamMain,
            WirePixelFormat::Mono8,
            1440,
            1080,
            1440,
            9876,
            1_760_000_000_123_456_789,
        )
        .unwrap()
    }

    #[test]
    fn header_roundtrips() {
        let original = header();
        let encoded = original.encode();
        assert_eq!(encoded.len(), VIDEO_HEADER_LEN);
        assert_eq!(VideoFrameHeader::decode(&encoded).unwrap(), original);
        assert_eq!(original.payload_len, 1440 * 1080);
    }

    #[test]
    fn payload_len_follows_a_padded_stride() {
        let padded = VideoFrameHeader::new(
            DisplaySource::StrandCamSecondary,
            WirePixelFormat::Rgb8,
            100,
            10,
            320,
            0,
            0,
        )
        .unwrap();
        assert_eq!(padded.payload_len, 3200, "stride, not width, sets the size");
        assert_eq!(VideoFrameHeader::decode(&padded.encode()).unwrap(), padded);
    }

    #[test]
    fn a_foreign_stream_is_rejected_rather_than_misread() {
        let mut encoded = header().encode();
        encoded[0..4].copy_from_slice(b"HTTP");
        assert_eq!(
            VideoFrameHeader::decode(&encoded),
            Err(VideoWireError::BadMagic(*b"HTTP"))
        );
    }

    #[test]
    fn a_version_mismatch_is_rejected_loudly() {
        let mut encoded = header().encode();
        encoded[4..6].copy_from_slice(&(VIDEO_WIRE_VERSION + 1).to_le_bytes());
        assert_eq!(
            VideoFrameHeader::decode(&encoded),
            Err(VideoWireError::VersionMismatch {
                peer: VIDEO_WIRE_VERSION + 1
            })
        );
    }

    #[test]
    fn the_webcam_is_not_a_relayable_source() {
        assert_eq!(
            VideoFrameHeader::new(DisplaySource::Webcam, WirePixelFormat::Mono8, 4, 4, 4, 0, 0),
            Err(VideoWireError::WebcamSource)
        );

        let mut encoded = header().encode();
        encoded[6] = 0;
        assert_eq!(
            VideoFrameHeader::decode(&encoded),
            Err(VideoWireError::WebcamSource)
        );
    }

    #[test]
    fn unknown_ids_are_rejected() {
        let mut encoded = header().encode();
        encoded[6] = 7;
        assert_eq!(
            VideoFrameHeader::decode(&encoded),
            Err(VideoWireError::UnknownSource(7))
        );

        let mut encoded = header().encode();
        encoded[7] = 9;
        assert_eq!(
            VideoFrameHeader::decode(&encoded),
            Err(VideoWireError::UnknownPixelFormat(9))
        );
    }

    #[test]
    fn a_stride_too_small_for_the_pixel_format_is_rejected() {
        assert_eq!(
            VideoFrameHeader::new(
                DisplaySource::StrandCamMain,
                WirePixelFormat::Rgb8,
                100,
                10,
                100,
                0,
                0
            ),
            Err(VideoWireError::StrideTooSmall {
                stride: 100,
                width: 100,
                pixel_format: WirePixelFormat::Rgb8,
            })
        );
    }

    #[test]
    fn degenerate_geometry_is_rejected() {
        assert_eq!(
            VideoFrameHeader::new(
                DisplaySource::StrandCamMain,
                WirePixelFormat::Mono8,
                0,
                10,
                0,
                0,
                0
            ),
            Err(VideoWireError::DegenerateGeometry {
                width: 0,
                height: 10
            })
        );
    }

    /// A corrupt or hostile header must not be able to make the receiver
    /// allocate a huge buffer, so `payload_len` is checked against the
    /// geometry it claims rather than trusted.
    #[test]
    fn a_payload_length_disagreeing_with_the_geometry_is_rejected() {
        let mut encoded = header().encode();
        encoded[36..40].copy_from_slice(&99u32.to_le_bytes());
        assert_eq!(
            VideoFrameHeader::decode(&encoded),
            Err(VideoWireError::PayloadLengthMismatch {
                payload_len: 99,
                stride: 1440,
                height: 1080,
            })
        );
    }

    #[test]
    fn an_oversized_frame_is_rejected_before_it_is_allocated() {
        let too_big = VideoFrameHeader::new(
            DisplaySource::StrandCamMain,
            WirePixelFormat::Rgb8,
            40_000,
            40_000,
            120_000,
            0,
            0,
        );
        assert!(
            matches!(too_big, Err(VideoWireError::PayloadTooLarge(_))),
            "got {too_big:?}"
        );
    }
}
