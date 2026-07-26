//! The two servers flo connects to: the JSON-lines control link, which is
//! everything below, and the binary video link in [`crate::video_link`]. They
//! are separate connections on separate ports and fail independently — a live
//! control link with a dead video link means the display falls back to the
//! webcam, and vice versa.

use camshow_protocol::{CamshowFramedCodec, FloToCamshow, PROTOCOL_VERSION};
use eyre::{Result, WrapErr};
use flo_core::DisplaySource;
use futures::StreamExt;
use tokio::{net::TcpListener, sync::watch};
use tokio_util::codec::FramedRead;
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
    pub(crate) rtp_bitrate_tx: tokio::sync::mpsc::UnboundedSender<Option<u32>>,
    /// What the capture loop should display. A watch rather than a queue: only
    /// the operator's latest choice matters.
    pub(crate) display_source_tx: watch::Sender<DisplaySource>,
}

impl Server {
    /// Serves both links until either one's listener fails to bind.
    ///
    /// A bind failure is fatal because it cannot be recovered from in place:
    /// the caller shuts the process down so a supervisor can restart it or the
    /// operator can fix the address. Accept and per-connection errors are
    /// retried inside each server.
    pub(crate) async fn run(self) -> Result<()> {
        let video = crate::video_link::serve(self.video_listen_addr.clone(), self.relayed.clone());
        tokio::select! {
            result = self.serve_control() => result,
            result = video => result,
        }
    }

    /// Binds `listen_addr` and serves flo connections, forwarding OSD updates
    /// and recording commands to the channels, until a bind or accept error
    /// ends the loop.
    async fn serve_control(&self) -> Result<()> {
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

            if let Err(e) = self.handle_client(stream).await {
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

    async fn handle_client(&self, stream: tokio::net::TcpStream) -> Result<()> {
        stream.set_nodelay(true).ok();
        let mut frames = FramedRead::new(stream, CamshowFramedCodec::default());

        let mut handshake_done = false;
        while let Some(msg) = frames.next().await {
            let msg = msg?;
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
                    }
                    other => eyre::bail!("client sent {other:?} before Hello"),
                }
                continue;
            }
            match msg {
                FloToCamshow::Hello { .. } => debug!("repeated Hello; ignoring"),
                FloToCamshow::Osd(canvas) => {
                    let _ = self.osd_tx.send(Some(OsdSnapshot {
                        canvas,
                        received_at: std::time::Instant::now(),
                    }));
                }
                FloToCamshow::StartRecording(start) => {
                    self.recording_tx.send(RecordingCommand::Start(start))?;
                }
                FloToCamshow::StopRecording => {
                    self.recording_tx.send(RecordingCommand::Stop)?;
                }
                FloToCamshow::SetRtpBitrateKbps(bitrate_kbps) => {
                    self.rtp_bitrate_tx.send(bitrate_kbps)?;
                }
                FloToCamshow::SetDisplaySource { source } => {
                    // An error here means the capture loop is gone, which the
                    // recording-command sends above already treat as fatal.
                    self.display_source_tx.send(source)?;
                }
            }
        }
        Ok(())
    }
}
