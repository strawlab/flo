//! Wire protocol between the `flo` controller process and the `camshow`
//! webcam viewer.
//!
//! `camshow` listens on a TCP port. `flo` connects (and reconnects after any
//! disconnect) and pushes a stream of JSON-lines messages: OSD canvas updates
//! plus recording start/stop commands. The link is one-way (flo → camshow).
//! camshow keeps showing the camera regardless of whether flo is connected,
//! so the link is best-effort.

use osd_utils::OsdCache;
use serde::{Deserialize, Serialize};

/// Default address camshow listens on and flo connects to.
pub const DEFAULT_CAMSHOW_ADDR: &str = "127.0.0.1:2224";

/// Bumped when the wire protocol changes incompatibly. camshow rejects
/// connections from flo if the version does not match.
pub const PROTOCOL_VERSION: u32 = 1;

/// Parameters for a recording-start: the codec config, the output
/// directory, and the timestamp used to derive the file name. Defined as a
/// standalone struct so the same payload can be reused by the in-process
/// channel that flo and camshow each maintain to bridge their async tasks.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecordingStart {
    pub creation_time: chrono::DateTime<chrono::Local>,
    pub data_dir: camino::Utf8PathBuf,
    pub mp4_cfg: strand_cam_remote_control::RecordingConfig,
}

/// Messages flo sends to camshow.
///
/// `StartRecording` is boxed so the enum size stays bounded by the small
/// variants — `RecordingConfig` carries several `Vec<String>` fields and
/// would otherwise dominate the layout.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum FloToCamshow {
    /// Always the first message. Lets camshow drop incompatible peers.
    Hello { protocol_version: u32 },
    /// New OSD canvas to overlay on the webcam frame.
    Osd(OsdCache),
    /// Begin recording webcam to disk. flo provides the output directory
    /// and codec configuration so camshow itself need not know either.
    StartRecording(Box<RecordingStart>),
    /// Stop recording, finalize the file.
    StopRecording,
}

pub type CamshowFramedCodec = json_lines::codec::JsonLinesCodec<FloToCamshow, FloToCamshow>;

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;

    fn roundtrip(msg: &FloToCamshow) -> FloToCamshow {
        let s = serde_json::to_string(msg).expect("serialize");
        serde_json::from_str(&s).expect("deserialize")
    }

    #[test]
    fn hello_roundtrip() {
        let msg = FloToCamshow::Hello {
            protocol_version: PROTOCOL_VERSION,
        };
        let s = serde_json::to_string(&msg).unwrap();
        assert!(s.contains(r#""kind":"hello""#), "got {s}");
        assert!(matches!(roundtrip(&msg), FloToCamshow::Hello { .. }));
    }

    #[test]
    fn stop_roundtrip() {
        let msg = FloToCamshow::StopRecording;
        let s = serde_json::to_string(&msg).unwrap();
        assert_eq!(s, r#"{"kind":"stop_recording"}"#);
        assert!(matches!(roundtrip(&msg), FloToCamshow::StopRecording));
    }

    #[test]
    fn start_recording_boxed_struct_roundtrip() {
        let start = RecordingStart {
            creation_time: chrono::Local
                .with_ymd_and_hms(2026, 5, 7, 12, 0, 0)
                .unwrap(),
            data_dir: camino::Utf8PathBuf::from("/tmp/flo"),
            mp4_cfg: strand_cam_remote_control::RecordingConfig::default(),
        };
        let msg = FloToCamshow::StartRecording(Box::new(start));
        let s = serde_json::to_string(&msg).unwrap();
        assert!(s.contains(r#""kind":"start_recording""#), "got {s}");
        assert!(s.contains(r#""data_dir":"/tmp/flo""#), "got {s}");
        match roundtrip(&msg) {
            FloToCamshow::StartRecording(rt) => {
                assert_eq!(rt.data_dir, camino::Utf8PathBuf::from("/tmp/flo"));
            }
            other => panic!("unexpected variant: {other:?}"),
        }
    }

    #[test]
    fn osd_roundtrip() {
        let canvas = OsdCache::new(30, 16);
        let msg = FloToCamshow::Osd(canvas);
        let s = serde_json::to_string(&msg).unwrap();
        assert!(s.contains(r#""kind":"osd""#), "got {s}");
        assert!(matches!(roundtrip(&msg), FloToCamshow::Osd(_)));
    }
}
