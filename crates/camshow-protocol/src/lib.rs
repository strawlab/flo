//! Wire protocol between the `flo` controller process and the `camshow`
//! webcam viewer.
//!
//! `camshow` listens on a TCP port. `flo` connects (and reconnects after any
//! disconnect) and pushes a stream of JSON-lines messages: OSD canvas updates
//! plus recording start/stop commands. `camshow` can send an operator's local
//! display-selection request back on that same connection. camshow keeps
//! showing the camera regardless of whether flo is connected, so the link is
//! best-effort.
//!
//! Two further connections carry pixels, each with its own binary framing and
//! its own port: [`video`] relays tracking-camera frames from flo to camshow
//! for the live view, and [`preview`] sends webcam frames back the other way
//! so flo's browser UI can show what the FPV camera sees.

use osd_utils::OsdCache;
use serde::{Deserialize, Serialize};

pub mod preview;
pub mod video;

/// Default address camshow listens on and flo connects to.
pub const DEFAULT_CAMSHOW_ADDR: &str = "127.0.0.1:2224";

/// Bumped when the wire protocol changes incompatibly. camshow rejects
/// connections from flo if the version does not match.
///
/// 2: added `SetDisplaySource`.
/// 3: made the control link bidirectional for GUI display-selection requests.
/// 4: added dynamic H.264/RTP destination management.
/// 5: made the bitrate independently configurable for every RTP target.
/// 6: added pre-capture (`SetPreCaptureSeconds` and
///    `RecordingStart::include_precapture`).
/// 7: camshow advertises its preview port on the control link.
pub const PROTOCOL_VERSION: u32 = 7;

/// Parameters for a recording-start: the codec config, the output
/// directory, and the timestamp used to derive the file name. Defined as a
/// standalone struct so the same payload can be reused by the in-process
/// channel that flo and camshow each maintain to bridge their async tasks.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecordingStart {
    pub creation_time: chrono::DateTime<chrono::Local>,
    pub data_dir: camino::Utf8PathBuf,
    pub mp4_cfg: strand_cam_remote_control::RecordingConfig,
    /// Whether to begin the file with camshow's pre-capture buffer, so the
    /// recording starts before the operator triggered it. When false the
    /// buffer is left alone and recording starts from the next frame.
    ///
    /// `creation_time` is flo's estimate of where such a recording begins:
    /// camshow's buffer is its own, so the first frame written lands near
    /// that time rather than exactly on it.
    pub include_precapture: bool,
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
    /// Size camshow's pre-capture ring buffer, in seconds of video. Zero
    /// disables it and frees whatever it holds.
    ///
    /// State, not an event: camshow forgets it when the link drops, so flo
    /// resends this on every connection the way it does the display source.
    SetPreCaptureSeconds { secs: f64 },
    /// Replace the complete set of independently configured H.264/RTP streams.
    SetRtpTargets { targets: Vec<flo_core::RtpTarget> },
    /// Choose what the display and RTP stream show. A non-webcam source needs
    /// frames arriving on the video link (see [`video`]); without them camshow
    /// shows the webcam instead. Never affects the recording.
    ///
    /// A named field rather than a newtype: serde's internally-tagged
    /// representation flattens a nested unit-variant enum into a bare key
    /// (`{"strand_cam_main":null}`), which is not a shape worth putting on a
    /// wire.
    SetDisplaySource { source: flo_core::DisplaySource },
}

/// Messages camshow sends back to FLO.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum CamshowToFlo {
    /// Request that FLO select this source. FLO remains authoritative: it
    /// republishes the resulting state to camshow after accepting the request.
    SetDisplaySource { source: flo_core::DisplaySource },
    /// The H.264/RTP destinations camshow is currently configured to send to.
    RtpTargets { targets: Vec<flo_core::RtpTarget> },
    /// The TCP port on the control peer that serves FPV webcam previews. FLO
    /// combines this with the control connection's peer IP, rather than using
    /// camshow's bind address, which may be an unspecified address.
    PreviewPort { port: u16 },
}

/// Codec as used by FLO: decode requests from camshow and encode commands to it.
pub type FloCamshowCodec = json_lines::codec::JsonLinesCodec<CamshowToFlo, FloToCamshow>;

/// Codec as used by camshow: decode commands from FLO and encode requests to it.
pub type CamshowFloCodec = json_lines::codec::JsonLinesCodec<FloToCamshow, CamshowToFlo>;

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
            include_precapture: true,
        };
        let msg = FloToCamshow::StartRecording(Box::new(start));
        let s = serde_json::to_string(&msg).unwrap();
        assert!(s.contains(r#""kind":"start_recording""#), "got {s}");
        assert!(s.contains(r#""data_dir":"/tmp/flo""#), "got {s}");
        match roundtrip(&msg) {
            FloToCamshow::StartRecording(rt) => {
                assert_eq!(rt.data_dir, camino::Utf8PathBuf::from("/tmp/flo"));
                assert!(rt.include_precapture);
            }
            other => panic!("unexpected variant: {other:?}"),
        }
    }

    #[test]
    fn set_precapture_seconds_roundtrip() {
        let msg = FloToCamshow::SetPreCaptureSeconds { secs: 12.5 };
        let s = serde_json::to_string(&msg).unwrap();
        assert_eq!(s, r#"{"kind":"set_pre_capture_seconds","secs":12.5}"#);
        assert!(matches!(
            roundtrip(&msg),
            FloToCamshow::SetPreCaptureSeconds { secs } if secs == 12.5
        ));
    }

    #[test]
    fn set_display_source_roundtrip() {
        let msg = FloToCamshow::SetDisplaySource {
            source: flo_core::DisplaySource::StrandCamMain,
        };
        let s = serde_json::to_string(&msg).unwrap();
        assert_eq!(
            s, r#"{"kind":"set_display_source","source":"strand_cam_main"}"#,
            "got {s}"
        );
        assert!(matches!(
            roundtrip(&msg),
            FloToCamshow::SetDisplaySource {
                source: flo_core::DisplaySource::StrandCamMain
            }
        ));
    }

    #[test]
    fn gui_display_source_request_roundtrip() {
        let msg = CamshowToFlo::SetDisplaySource {
            source: flo_core::DisplaySource::StrandCamSecondary,
        };
        let s = serde_json::to_string(&msg).unwrap();
        assert_eq!(
            s, r#"{"kind":"set_display_source","source":"strand_cam_secondary"}"#,
            "got {s}"
        );
        assert!(matches!(
            roundtrip_camshow_to_flo(&msg),
            CamshowToFlo::SetDisplaySource {
                source: flo_core::DisplaySource::StrandCamSecondary
            }
        ));
    }

    #[test]
    fn rtp_targets_roundtrip_in_both_directions() {
        let targets = vec![flo_core::RtpTarget {
            addr: "192.168.1.20:5600".parse().unwrap(),
            bitrate_kbps: 2500,
        }];
        let to_camshow = FloToCamshow::SetRtpTargets {
            targets: targets.clone(),
        };
        assert!(matches!(
            roundtrip(&to_camshow),
            FloToCamshow::SetRtpTargets { targets: received } if received == targets
        ));

        let to_flo = CamshowToFlo::RtpTargets { targets };
        assert!(matches!(
            roundtrip_camshow_to_flo(&to_flo),
            CamshowToFlo::RtpTargets { targets } if targets.len() == 1
        ));
    }

    #[test]
    fn preview_port_roundtrips() {
        let message = CamshowToFlo::PreviewPort { port: 2226 };
        assert!(matches!(
            roundtrip_camshow_to_flo(&message),
            CamshowToFlo::PreviewPort { port: 2226 }
        ));
    }

    #[test]
    fn osd_roundtrip() {
        let canvas = OsdCache::new(30, 16);
        let msg = FloToCamshow::Osd(canvas);
        let s = serde_json::to_string(&msg).unwrap();
        assert!(s.contains(r#""kind":"osd""#), "got {s}");
        assert!(matches!(roundtrip(&msg), FloToCamshow::Osd(_)));
    }

    fn roundtrip_camshow_to_flo(msg: &CamshowToFlo) -> CamshowToFlo {
        let s = serde_json::to_string(msg).expect("serialize");
        serde_json::from_str(&s).expect("deserialize")
    }
}
