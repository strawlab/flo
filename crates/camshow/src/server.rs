//! The two servers flo connects to: the JSON-lines control link, which is
//! everything below, and the binary video link in [`crate::video_link`]. They
//! are separate connections on separate ports and fail independently — a live
//! control link with a dead video link means the display falls back to the
//! webcam, and vice versa.

use camshow_protocol::{CamshowFloCodec, CamshowToFlo, FloToCamshow, PROTOCOL_VERSION};
use eyre::{Result, WrapErr};
use flo_core::DisplaySource;
use futures::{SinkExt, StreamExt};
use tokio::{net::TcpListener, sync::watch};
use tokio_util::codec::{FramedRead, FramedWrite};
use tracing::{debug, info, warn};

use crate::{
    state::{OsdSnapshot, RecordingCommand},
    video_link::LatestRelayedFrame,
};

/// The listen addresses plus the channels flo's messages are forwarded on.
pub(crate) struct Server {
    pub(crate) listen_addr: String,
    pub(crate) video_listen_addr: String,
    /// Where the video link deposits the newest relayed frame.
    pub(crate) relayed: LatestRelayedFrame,
    pub(crate) osd_tx: watch::Sender<Option<OsdSnapshot>>,
    pub(crate) recording_tx: tokio::sync::mpsc::UnboundedSender<RecordingCommand>,
    /// The current complete RTP destination set, consumed by the capture
    /// thread and reported to FLO whenever its control link connects.
    pub(crate) rtp_targets_tx: watch::Sender<Vec<flo_core::RtpTarget>>,
    /// What the capture loop should display. A watch rather than a queue: only
    /// the operator's latest choice matters.
    pub(crate) display_source_tx: watch::Sender<DisplaySource>,
    /// Local GUI requests that need to be forwarded to FLO. The initial value
    /// is marked seen by `main`, so FLO's selection is never overwritten when
    /// a connection first opens.
    pub(crate) gui_display_source_rx: watch::Receiver<DisplaySource>,
}

impl Server {
    /// Serves both links until either one's listener fails to bind.
    ///
    /// A bind failure is fatal because it cannot be recovered from in place:
    /// the caller shuts the process down so a supervisor can restart it or the
    /// operator can fix the address. Accept and per-connection errors are
    /// retried inside each server.
    pub(crate) async fn run(mut self) -> Result<()> {
        let video = crate::video_link::serve(self.video_listen_addr.clone(), self.relayed.clone());
        tokio::select! {
            result = self.serve_control() => result,
            result = video => result,
        }
    }

    /// Binds `listen_addr` and serves flo connections, forwarding OSD updates
    /// and recording commands to the channels, until a bind or accept error
    /// ends the loop.
    async fn serve_control(&mut self) -> Result<()> {
        let listener = TcpListener::bind(&self.listen_addr)
            .await
            .with_context(|| format!("binding TCP listener at {}", self.listen_addr))?;
        info!("listening on {}", listener.local_addr()?);
        loop {
            let (stream, peer) = match listener.accept().await {
                Ok(pair) => pair,
                Err(e) => {
                    warn!("accept error: {e}; retrying");
                    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
                    continue;
                }
            };
            info!("flo connected from {peer}");

            let osd_tx = self.osd_tx.clone();
            let recording_tx = self.recording_tx.clone();
            let rtp_targets_tx = self.rtp_targets_tx.clone();
            let display_source_tx = self.display_source_tx.clone();
            if let Err(e) = Self::handle_client(
                stream,
                &mut self.gui_display_source_rx,
                &osd_tx,
                &recording_tx,
                &rtp_targets_tx,
                &display_source_tx,
            )
            .await
            {
                warn!("flo connection {peer} ended: {e}");
            } else {
                info!("flo {peer} disconnected cleanly");
            }

            // Always stop any in-flight recording when the connection drops so
            // we never accumulate unbounded webcam files behind a dead flo.
            let _ = self.recording_tx.send(RecordingCommand::Stop);
            // Clear any stale OSD overlay so the next connection starts clean.
            let _ = self.osd_tx.send(None);
            // Go back to the webcam: with flo gone there is nothing feeding the
            // video link either, so leaving a tracking camera selected would
            // just mean the fallback path warning until flo returns.
            let _ = self.display_source_tx.send(DisplaySource::Webcam);
        }
    }

    async fn handle_client(
        stream: tokio::net::TcpStream,
        gui_display_source_rx: &mut watch::Receiver<DisplaySource>,
        osd_tx: &watch::Sender<Option<OsdSnapshot>>,
        recording_tx: &tokio::sync::mpsc::UnboundedSender<RecordingCommand>,
        rtp_targets_tx: &watch::Sender<Vec<flo_core::RtpTarget>>,
        display_source_tx: &watch::Sender<DisplaySource>,
    ) -> Result<()> {
        stream.set_nodelay(true).ok();
        let (read_half, write_half) = stream.into_split();
        let mut frames = FramedRead::new(read_half, CamshowFloCodec::default());
        let mut requests = FramedWrite::new(write_half, CamshowFloCodec::default());

        let mut handshake_done = false;
        loop {
            let msg = tokio::select! {
                msg = frames.next() => match msg {
                    Some(msg) => msg?,
                    None => return Ok(()),
                },
                changed = gui_display_source_rx.changed() => {
                    changed?;
                    let source = *gui_display_source_rx.borrow_and_update();
                    requests
                        .send(CamshowToFlo::SetDisplaySource { source })
                        .await?;
                    continue;
                }
            };
            if !handshake_done {
                match msg {
                    FloToCamshow::Hello { protocol_version } => {
                        if protocol_version != PROTOCOL_VERSION {
                            eyre::bail!(
                                "protocol mismatch (peer={protocol_version}, us={PROTOCOL_VERSION})"
                            );
                        }
                        handshake_done = true;
                        debug!("handshake complete");
                        requests
                            .send(CamshowToFlo::RtpTargets {
                                targets: rtp_targets_tx.borrow().clone(),
                            })
                            .await?;
                    }
                    other => eyre::bail!("client sent {other:?} before Hello"),
                }
                continue;
            }
            match msg {
                FloToCamshow::Hello { .. } => debug!("repeated Hello; ignoring"),
                FloToCamshow::Osd(canvas) => {
                    let _ = osd_tx.send(Some(OsdSnapshot {
                        canvas,
                        received_at: std::time::Instant::now(),
                    }));
                }
                FloToCamshow::StartRecording(start) => {
                    recording_tx.send(RecordingCommand::Start(start))?;
                }
                FloToCamshow::StopRecording => {
                    recording_tx.send(RecordingCommand::Stop)?;
                }
                FloToCamshow::SetPreCaptureSeconds { secs } => {
                    recording_tx.send(RecordingCommand::SetPreCaptureSeconds(secs))?;
                }
                FloToCamshow::SetRtpTargets { targets } => {
                    rtp_targets_tx.send(targets)?;
                }
                FloToCamshow::SetDisplaySource { source } => {
                    // An error here means the capture loop is gone, which the
                    // recording-command sends above already treat as fatal.
                    display_source_tx.send(source)?;
                }
            }
        }
    }
}
