//! Pushes OSD canvas snapshots and recording commands from `flo` to a
//! `camshow` instance over a localhost TCP connection.
//!
//! The link is best-effort: camshow may be down, restart, or never run at
//! all. This task connects, retries on failure, and forwards whatever data
//! is currently buffered. flo's main loop is unaffected by camshow being
//! unavailable.

use camshow_protocol::{CamshowFramedCodec, FloToCamshow, PROTOCOL_VERSION, RecordingStart};
use color_eyre::eyre::Result;
use futures::SinkExt;
use osd_utils::OsdCache;
use tokio::{net::TcpStream, sync::mpsc, sync::watch};
use tokio_util::codec::FramedWrite;
use tracing::{debug, info, warn};

/// Recording commands flo's coordinator sends to the camshow client.
#[derive(Debug, Clone)]
pub(crate) enum Command {
    Start(Box<RecordingStart>),
    Stop,
}

const RECONNECT_DELAY: std::time::Duration = std::time::Duration::from_secs(1);

pub(crate) async fn run(
    addr: String,
    mut canvas_rx: watch::Receiver<OsdCache>,
    mut command_rx: mpsc::UnboundedReceiver<Command>,
) -> Result<()> {
    info!("camshow link target: {addr}");
    loop {
        match TcpStream::connect(&addr).await {
            Ok(stream) => {
                info!("connected to camshow at {addr}");
                if let Err(e) = serve_connection(stream, &mut canvas_rx, &mut command_rx).await {
                    warn!("camshow link to {addr} failed: {e}");
                } else {
                    info!("camshow link to {addr} closed");
                }
            }
            Err(e) => {
                debug!("camshow connect to {addr} failed: {e}");
            }
        }
        tokio::time::sleep(RECONNECT_DELAY).await;
    }
}

async fn serve_connection(
    stream: TcpStream,
    canvas_rx: &mut watch::Receiver<OsdCache>,
    command_rx: &mut mpsc::UnboundedReceiver<Command>,
) -> Result<()> {
    stream.set_nodelay(true).ok();
    let mut sink = FramedWrite::new(stream, CamshowFramedCodec::default());
    sink.send(FloToCamshow::Hello {
        protocol_version: PROTOCOL_VERSION,
    })
    .await?;

    // Mark current canvas value as seen so we don't immediately push a
    // potentially-stale snapshot from before this connection started.
    canvas_rx.mark_unchanged();

    loop {
        tokio::select! {
            // Recording commands take priority over OSD updates: the
            // semantics matter and they're rare.
            biased;
            cmd = command_rx.recv() => {
                let Some(cmd) = cmd else {
                    debug!("recording command channel closed; ending camshow link");
                    return Ok(());
                };
                let msg = match cmd {
                    Command::Start(start) => FloToCamshow::StartRecording(start),
                    Command::Stop => FloToCamshow::StopRecording,
                };
                sink.send(msg).await?;
            }
            res = canvas_rx.changed() => {
                res?;
                let canvas = canvas_rx.borrow_and_update().clone();
                sink.send(FloToCamshow::Osd(canvas)).await?;
            }
        }
    }
}
