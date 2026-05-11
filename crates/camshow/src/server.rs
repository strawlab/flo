//! TCP server that accepts a single flo connection at a time and forwards
//! its messages to the camera task.

use camshow_protocol::{CamshowFramedCodec, FloToCamshow, PROTOCOL_VERSION};
use eyre::Result;
use futures::StreamExt;
use tokio::{net::TcpListener, sync::watch};
use tokio_util::codec::FramedRead;
use tracing::{debug, info, warn};

use crate::state::{OsdSnapshot, RecordingCommand};

pub(crate) async fn run(
    listener: TcpListener,
    osd_tx: watch::Sender<Option<OsdSnapshot>>,
    recording_tx: tokio::sync::mpsc::UnboundedSender<RecordingCommand>,
) -> Result<()> {
    info!("camshow listening on {}", listener.local_addr()?);
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

        if let Err(e) = handle_client(stream, &osd_tx, &recording_tx).await {
            warn!("flo connection {peer} ended: {e}");
        } else {
            info!("flo {peer} disconnected cleanly");
        }

        // Always stop any in-flight recording when the connection drops so
        // we never accumulate unbounded webcam files behind a dead flo.
        let _ = recording_tx.send(RecordingCommand::Stop);
        // Clear any stale OSD overlay so the next connection starts clean.
        let _ = osd_tx.send(None);
    }
}

async fn handle_client(
    stream: tokio::net::TcpStream,
    osd_tx: &watch::Sender<Option<OsdSnapshot>>,
    recording_tx: &tokio::sync::mpsc::UnboundedSender<RecordingCommand>,
) -> Result<()> {
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
        }
    }
    Ok(())
}
