//! Relays tracking-camera frames to `camshow` so the operator's live view can
//! show them instead of the FPV webcam.
//!
//! One task owns the whole path: every camera's frame tap, the current display
//! selection, and the socket. That is deliberate — only one camera is ever
//! selected, and a single writer means a frame can never be interleaved into
//! another frame's bytes on the wire.
//!
//! Nothing here may push back on acquisition. The tracking camera is
//! flight-critical, so the display path is lossy end to end:
//!
//! - Strand Camera delivers every frame to FLO through a lossless sink. FLO's
//!   detector consumes that sink first, then uses `try_send` to offer a frame
//!   here. A busy relay therefore drops only display frames, never detections.
//! - Frames for a camera that is not selected are dropped immediately. The taps
//!   are still drained, so the cost of nobody watching is a channel round trip
//!   and a refcount bump.
//! - Selected frames are decimated to display rate. A tracker may acquire at
//!   60 fps or more; nobody can see that on a video link.
//! - A socket write that stalls times out and the connection is dropped rather
//!   than waited on, because a wedged camshow must not wedge the relay.

use std::time::{Duration, Instant};

use camshow_protocol::video::{VideoFrameHeader, WirePixelFormat};
use color_eyre::eyre::{Result, WrapErr};
use flo_core::DisplaySource;
use machine_vision_formats::{
    ImageData, PixFmt, PixelFormat, Stride,
    pixel_format::{Mono8, RGB8},
};
use serde::Deserialize;
use strand_cam::HostFrame;
use tokio::{
    io::AsyncWriteExt,
    net::TcpStream,
    sync::{mpsc, watch},
};
use tracing::{debug, info, warn};

/// How long to wait before trying camshow again. camshow may not be running at
/// all, so a failed connect is logged at debug level only.
const RECONNECT_DELAY: Duration = Duration::from_secs(1);

/// A write that takes longer than this means camshow has stopped reading. The
/// connection is dropped: it may have a partial frame on it, so it cannot be
/// reused, and reconnecting is how the relay recovers.
const WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// How often to repeat a per-frame conversion failure. It would otherwise be
/// one line per frame for as long as the camera stays in that pixel format.
const CONVERT_WARN_INTERVAL: Duration = Duration::from_secs(10);

/// Relay settings, from the `flo-strand-cam.video_relay` config section. The
/// defaults are the working configuration; the section only needs to be present
/// to change something.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct VideoRelayConfig {
    /// Turn the relay off entirely. Frame taps are then not even registered, so
    /// strand-cam does no tap work at all.
    #[serde(default = "default_enabled")]
    pub(crate) enabled: bool,
    /// camshow's video-link address, matching its `--video-listen`.
    #[serde(default = "default_camshow_video_addr")]
    pub(crate) camshow_video_addr: String,
    /// Upper bound on frames sent per second, per camera.
    #[serde(default = "default_max_fps")]
    pub(crate) max_fps: f64,
}

fn default_enabled() -> bool {
    true
}

fn default_camshow_video_addr() -> String {
    camshow_protocol::video::DEFAULT_CAMSHOW_VIDEO_ADDR.to_owned()
}

fn default_max_fps() -> f64 {
    30.0
}

impl Default for VideoRelayConfig {
    fn default() -> Self {
        Self {
            enabled: default_enabled(),
            camshow_video_addr: default_camshow_video_addr(),
            max_fps: default_max_fps(),
        }
    }
}

impl VideoRelayConfig {
    /// Minimum spacing between sent frames, or `None` if `max_fps` is not a
    /// usable rate (in which case every selected frame is sent).
    fn min_send_interval(&self) -> Option<Duration> {
        (self.max_fps.is_finite() && self.max_fps > 0.0)
            .then(|| Duration::from_secs_f64(1.0 / self.max_fps))
    }
}

/// Everything the relay task needs. Taps are `Option` because the secondary
/// camera is optional and because the relay may be configured off.
pub(crate) struct VideoRelay {
    pub(crate) config: VideoRelayConfig,
    pub(crate) main_tap: Option<mpsc::Receiver<HostFrame>>,
    pub(crate) secondary_tap: Option<mpsc::Receiver<HostFrame>>,
    pub(crate) display_source: watch::Receiver<DisplaySource>,
}

/// Rate limiter for one camera.
struct Decimator {
    min_interval: Option<Duration>,
    last_sent: Option<Instant>,
}

impl Decimator {
    fn new(min_interval: Option<Duration>) -> Self {
        Self {
            min_interval,
            last_sent: None,
        }
    }

    fn allows_now(&mut self) -> bool {
        let Some(min_interval) = self.min_interval else {
            return true;
        };
        let now = Instant::now();
        if self.last_sent.is_some_and(|at| now - at < min_interval) {
            return false;
        }
        self.last_sent = Some(now);
        true
    }

    /// Forgets the last send, so the first frame after a switch or a reconnect
    /// goes out immediately instead of waiting out an interval.
    fn reset(&mut self) {
        self.last_sent = None;
    }
}

/// Runs the relay until every tap has closed.
///
/// Never returns an error for a transport problem: camshow being absent,
/// restarting, or hanging up is expected and is handled by reconnecting. flo
/// must keep flying regardless.
pub(crate) async fn run(relay: VideoRelay) -> Result<()> {
    let VideoRelay {
        config,
        mut main_tap,
        mut secondary_tap,
        mut display_source,
    } = relay;

    if !config.enabled {
        debug!("video relay disabled by configuration");
        return Ok(());
    }
    if main_tap.is_none() && secondary_tap.is_none() {
        debug!("video relay has no camera taps; nothing to relay");
        return Ok(());
    }
    info!(
        "video relay target: {} (cap {} fps per camera)",
        config.camshow_video_addr, config.max_fps
    );

    let min_interval = config.min_send_interval();
    let mut main_decimator = Decimator::new(min_interval);
    let mut secondary_decimator = Decimator::new(min_interval);
    // Reused across frames so a steady stream does not allocate per frame.
    let mut payload: Vec<u8> = Vec::new();
    let mut connection: Option<TcpStream> = None;
    let mut last_convert_warn: Option<Instant> = None;
    let mut selected = *display_source.borrow_and_update();

    loop {
        // Every tap is drained whatever is selected: leaving frames queued would
        // only make strand-cam's tap look permanently full for no benefit.
        let tapped = tokio::select! {
            changed = display_source.changed() => {
                if changed.is_err() {
                    debug!("display source channel closed; ending the video relay");
                    return Ok(());
                }
                let new_selection = *display_source.borrow_and_update();
                if new_selection != selected {
                    selected = new_selection;
                    main_decimator.reset();
                    secondary_decimator.reset();
                    debug!("video relay now sending for {selected:?}");
                }
                continue;
            }
            frame = recv_tap(&mut main_tap) => (DisplaySource::StrandCamMain, frame),
            frame = recv_tap(&mut secondary_tap) => (DisplaySource::StrandCamSecondary, frame),
        };

        let (source, frame) = tapped;
        let Some(frame) = frame else {
            // That camera has stopped. Drop its tap so `recv_tap` stops
            // offering it, and finish once no camera is left.
            match source {
                DisplaySource::StrandCamSecondary => secondary_tap = None,
                _ => main_tap = None,
            }
            if main_tap.is_none() && secondary_tap.is_none() {
                info!("every camera tap has closed; ending the video relay");
                return Ok(());
            }
            continue;
        };

        // A frame and a source selection can become ready in the same poll.
        // Read the latest value again so the frame immediately following a
        // switch is not accidentally discarded because `select!` chose the
        // frame arm first.
        let current_selection = *display_source.borrow_and_update();
        if current_selection != selected {
            selected = current_selection;
            main_decimator.reset();
            secondary_decimator.reset();
            debug!("video relay now sending for {selected:?}");
        }
        if source != selected {
            continue;
        }
        let decimator = match source {
            DisplaySource::StrandCamSecondary => &mut secondary_decimator,
            _ => &mut main_decimator,
        };
        if !decimator.allows_now() {
            continue;
        }

        let header = match encode(source, &frame, &mut payload) {
            Ok(header) => header,
            Err(e) => {
                if last_convert_warn.is_none_or(|at| at.elapsed() >= CONVERT_WARN_INTERVAL) {
                    warn!("cannot relay frames from {source:?}: {e:?}");
                    last_convert_warn = Some(Instant::now());
                }
                continue;
            }
        };

        if connection.is_none() {
            match connect(&config.camshow_video_addr).await {
                Some(stream) => connection = Some(stream),
                None => {
                    // No frame went out, so this one must not count against the
                    // rate cap; the next should go as soon as it arrives.
                    decimator.reset();
                    tokio::time::sleep(RECONNECT_DELAY).await;
                    continue;
                }
            }
        }

        let stream = connection.as_mut().expect("just connected");
        if let Err(e) = send_frame(stream, &header, &payload).await {
            warn!("video link write failed, reconnecting: {e:?}");
            connection = None;
            decimator.reset();
        }
    }
}

/// Awaits the next frame from a tap, or never resolves if that camera has no
/// tap, so it can sit in a `select!` arm unconditionally.
async fn recv_tap(tap: &mut Option<mpsc::Receiver<HostFrame>>) -> Option<HostFrame> {
    match tap {
        Some(tap) => tap.recv().await,
        None => std::future::pending().await,
    }
}

async fn connect(addr: &str) -> Option<TcpStream> {
    match TcpStream::connect(addr).await {
        Ok(stream) => {
            stream.set_nodelay(true).ok();
            info!("video link connected to camshow at {addr}");
            Some(stream)
        }
        Err(e) => {
            // camshow may simply not be running; that is not an error here.
            debug!("video link connect to {addr} failed: {e}");
            None
        }
    }
}

/// Writes one frame, header then pixels, giving up rather than blocking
/// indefinitely on a camshow that has stopped reading.
async fn send_frame(
    stream: &mut TcpStream,
    header: &VideoFrameHeader,
    payload: &[u8],
) -> Result<()> {
    let write = async {
        stream.write_all(&header.encode()).await?;
        stream.write_all(payload).await?;
        Ok::<(), std::io::Error>(())
    };
    tokio::time::timeout(WRITE_TIMEOUT, write)
        .await
        .map_err(|_| {
            color_eyre::eyre::eyre!("camshow did not read a frame within {WRITE_TIMEOUT:?}")
        })?
        .wrap_err("writing to the video link")?;
    Ok(())
}

/// Builds the wire header for a tapped frame and fills `payload` with its
/// pixels, tightly packed.
///
/// Mono8 is sent as-is at one byte per pixel — camshow replicates it to RGB8 —
/// because converting here would triple the bandwidth for no new information.
/// Anything else is converted to RGB8, which costs three times the bytes but
/// keeps the wire format to two cases the receiver has to know about.
fn encode(
    source: DisplaySource,
    frame: &HostFrame,
    payload: &mut Vec<u8>,
) -> Result<VideoFrameHeader> {
    let image = frame.image.borrow();
    let pixel_format = if image.pixel_format() == PixFmt::Mono8 {
        let mono = image
            .as_static::<Mono8>()
            .ok_or_else(|| color_eyre::eyre::eyre!("Mono8 frame did not borrow as Mono8"))?;
        pack_rows(&mono, WirePixelFormat::Mono8, payload)?;
        WirePixelFormat::Mono8
    } else {
        let rgb = image
            .into_pixel_format::<RGB8>()
            .wrap_err_with(|| format!("converting {:?} to RGB8", image.pixel_format()))?;
        pack_rows(&rgb, WirePixelFormat::Rgb8, payload)?;
        WirePixelFormat::Rgb8
    };

    let stride = image.width() * pixel_format.bytes_per_pixel();
    Ok(VideoFrameHeader::new(
        source,
        pixel_format,
        image.width(),
        image.height(),
        stride,
        frame.frame_number,
        frame.timestamp.timestamp_nanos_opt().unwrap_or(0),
    )?)
}

/// Copies the image's rows into `out` with no padding between them.
///
/// The camera's own stride may include padding, and the wire format's
/// `payload_len` is `stride * height` exactly, so repacking here is what keeps
/// the two consistent for every camera.
fn pack_rows<FMT, IMG>(image: &IMG, pixel_format: WirePixelFormat, out: &mut Vec<u8>) -> Result<()>
where
    FMT: PixelFormat,
    IMG: ImageData<FMT> + Stride,
{
    let row_len = image.width() as usize * pixel_format.bytes_per_pixel() as usize;
    let stride = image.stride();
    let data = image.image_data();

    out.clear();
    out.reserve(row_len * image.height() as usize);
    for row in 0..image.height() as usize {
        let start = row * stride;
        let bytes = data
            .get(start..start + row_len)
            .ok_or_else(|| color_eyre::eyre::eyre!("frame buffer is shorter than its geometry"))?;
        out.extend_from_slice(bytes);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use machine_vision_formats::owned::OImage;
    use strand_dynamic_frame::DynamicFrameOwned;

    use super::*;

    fn tapped(image: DynamicFrameOwned) -> HostFrame {
        HostFrame {
            image: Arc::new(image),
            frame_number: 42,
            timestamp: chrono::DateTime::UNIX_EPOCH,
            timestamp_source: strand_cam::TimestampSource::HostAcquiredTimestamp,
        }
    }

    #[test]
    fn mono8_is_sent_at_one_byte_per_pixel() {
        let image = OImage::<Mono8>::new(2, 2, 2, vec![1, 2, 3, 4]).unwrap();
        let mut payload = Vec::new();

        let header = encode(
            DisplaySource::StrandCamMain,
            &tapped(DynamicFrameOwned::from_static(image)),
            &mut payload,
        )
        .unwrap();

        assert_eq!(header.pixel_format, WirePixelFormat::Mono8);
        assert_eq!(header.stride, 2);
        assert_eq!(header.payload_len, 4);
        assert_eq!(payload, vec![1, 2, 3, 4]);
        assert_eq!(header.frame_number, 42);
    }

    #[test]
    fn a_padded_camera_stride_is_repacked_to_match_the_header() {
        // 2x2 Mono8 with two padding bytes per row. The header says stride 2, so
        // the payload must be 4 bytes with the padding removed.
        let image =
            OImage::<Mono8>::new(2, 2, 4, vec![1, 2, 0xff, 0xff, 3, 4, 0xff, 0xff]).unwrap();
        let mut payload = Vec::new();

        let header = encode(
            DisplaySource::StrandCamMain,
            &tapped(DynamicFrameOwned::from_static(image)),
            &mut payload,
        )
        .unwrap();

        assert_eq!(header.stride, 2);
        assert_eq!(header.payload_len, 4);
        assert_eq!(payload, vec![1, 2, 3, 4]);
    }

    #[test]
    fn a_colour_camera_is_converted_to_rgb8() {
        let image = OImage::<RGB8>::new(2, 1, 6, vec![1, 2, 3, 4, 5, 6]).unwrap();
        let mut payload = Vec::new();

        let header = encode(
            DisplaySource::StrandCamSecondary,
            &tapped(DynamicFrameOwned::from_static(image)),
            &mut payload,
        )
        .unwrap();

        assert_eq!(header.pixel_format, WirePixelFormat::Rgb8);
        assert_eq!(header.stride, 6);
        assert_eq!(payload, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn the_payload_buffer_is_reused_without_leaving_earlier_bytes_behind() {
        let mut payload = vec![0xaa; 1000];
        let image = OImage::<Mono8>::new(2, 1, 2, vec![7, 8]).unwrap();

        let header = encode(
            DisplaySource::StrandCamMain,
            &tapped(DynamicFrameOwned::from_static(image)),
            &mut payload,
        )
        .unwrap();

        assert_eq!(payload.len(), header.payload_len as usize);
        assert_eq!(payload, vec![7, 8]);
    }

    #[test]
    fn the_decimator_caps_the_rate_and_lets_the_first_frame_through() {
        let mut decimator = Decimator::new(Some(Duration::from_secs(3600)));

        assert!(decimator.allows_now(), "the first frame is always sent");
        assert!(!decimator.allows_now(), "and the next one is too soon");

        // A switch or a reconnect must not make the operator wait out an
        // interval before seeing anything.
        decimator.reset();
        assert!(decimator.allows_now());
    }

    #[test]
    fn an_unusable_rate_cap_sends_every_frame_rather_than_none() {
        for max_fps in [0.0, -1.0, f64::NAN] {
            let config = VideoRelayConfig {
                max_fps,
                ..Default::default()
            };
            assert!(
                config.min_send_interval().is_none(),
                "max_fps {max_fps} should not produce an interval"
            );
            let mut decimator = Decimator::new(config.min_send_interval());
            assert!(decimator.allows_now());
            assert!(decimator.allows_now());
        }
    }

    #[test]
    fn a_normal_rate_cap_becomes_the_expected_interval() {
        let config = VideoRelayConfig {
            max_fps: 30.0,
            ..Default::default()
        };
        assert_eq!(
            config.min_send_interval(),
            Some(Duration::from_secs_f64(1.0 / 30.0))
        );
    }

    fn mono_frame(value: u8) -> HostFrame {
        let image = OImage::<Mono8>::new(2, 2, 2, vec![value; 4]).unwrap();
        tapped(DynamicFrameOwned::from_static(image))
    }

    /// Reads one whole frame, or gives up if none arrives.
    async fn read_frame(stream: &mut tokio::net::TcpStream) -> Option<(VideoFrameHeader, Vec<u8>)> {
        use tokio::io::AsyncReadExt;

        let mut header_bytes = [0u8; camshow_protocol::video::VIDEO_HEADER_LEN];
        tokio::time::timeout(Duration::from_secs(5), stream.read_exact(&mut header_bytes))
            .await
            .ok()?
            .ok()?;
        let header = VideoFrameHeader::decode(&header_bytes).unwrap();
        let mut payload = vec![0u8; header.payload_len as usize];
        stream.read_exact(&mut payload).await.ok()?;
        Some((header, payload))
    }

    /// The relay must drain its taps whatever is selected, but put nothing on
    /// the wire — and not even connect — until the operator asks for that
    /// camera. Sending frames nobody is watching is wasted bandwidth on the
    /// flight computer.
    #[tokio::test]
    async fn frames_reach_the_wire_only_while_their_camera_is_selected() {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap().to_string();

        let (tap_tx, tap_rx) = mpsc::channel(4);
        let (source_tx, source_rx) = watch::channel(DisplaySource::Webcam);

        let relay = tokio::spawn(run(VideoRelay {
            config: VideoRelayConfig {
                enabled: true,
                camshow_video_addr: addr,
                // No decimation, so the test does not depend on timing.
                max_fps: 0.0,
            },
            main_tap: Some(tap_rx),
            secondary_tap: None,
            display_source: source_rx,
        }));

        // Webcam selected: these are drained and dropped.
        for value in [1, 2, 3] {
            tap_tx.send(mono_frame(value)).await.unwrap();
        }
        // Nothing should be connecting. Being generous with the wait would only
        // slow the test down; a connect to a bound listener is immediate.
        assert!(
            tokio::time::timeout(Duration::from_millis(250), listener.accept())
                .await
                .is_err(),
            "the relay must not even connect while nobody is watching this camera"
        );

        source_tx.send(DisplaySource::StrandCamMain).unwrap();
        tap_tx.send(mono_frame(9)).await.unwrap();

        let (mut stream, _) = tokio::time::timeout(Duration::from_secs(5), listener.accept())
            .await
            .expect("the relay connects once its camera is selected")
            .unwrap();
        let (header, payload) = read_frame(&mut stream).await.expect("a frame arrives");
        assert_eq!(header.source, DisplaySource::StrandCamMain);
        assert_eq!(header.pixel_format, WirePixelFormat::Mono8);
        assert_eq!(
            payload,
            vec![9; 4],
            "the frame sent is one from after the switch, not a stale queued one"
        );

        source_tx.send(DisplaySource::Webcam).unwrap();
        // Give the relay a chance to observe the switch before the next frame.
        tokio::time::sleep(Duration::from_millis(50)).await;
        tap_tx.send(mono_frame(11)).await.unwrap();
        assert!(
            read_frame(&mut stream).await.is_none(),
            "switching back to the webcam stops the traffic"
        );

        drop(tap_tx);
        tokio::time::timeout(Duration::from_secs(5), relay)
            .await
            .expect("the relay ends once its last tap closes")
            .unwrap()
            .unwrap();
    }

    #[tokio::test]
    async fn a_disabled_relay_does_nothing_at_all() {
        let (_tap_tx, tap_rx) = mpsc::channel(1);
        let (_source_tx, source_rx) = watch::channel(DisplaySource::StrandCamMain);

        run(VideoRelay {
            config: VideoRelayConfig {
                enabled: false,
                // Nothing listens here; the relay must not try.
                camshow_video_addr: "127.0.0.1:1".to_owned(),
                max_fps: 30.0,
            },
            main_tap: Some(tap_rx),
            secondary_tap: None,
            display_source: source_rx,
        })
        .await
        .unwrap();
    }
}
