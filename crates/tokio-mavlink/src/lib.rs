use std::{
    fmt::Debug,
    io::{Read, Write},
};

use mavlink::{MavHeader, MavlinkVersion, Message, error::MessageReadError};

pub struct MavlinkConnection<M: Message> {
    pub rx: MavlinkReceiver<M>,
    pub tx: MavlinkSender<M>,
}

/// An asynchronous implementation of a MAVLink receiver.
pub struct MavlinkReceiver<M: Message> {
    read_err: tokio::sync::oneshot::Receiver<RecvError>,
    reader_rx: tokio::sync::mpsc::Receiver<Result<(MavHeader, M), RecvError>>,
}

impl<M: Message + Debug> MavlinkReceiver<M> {
    pub async fn recv(&mut self) -> Result<(MavHeader, M), RecvError> {
        tokio::select! {
            err = &mut self.read_err => {
                tracing::error!("MavlinkReceiver: read error {err:?}");
                Err(flatten(err))
            }
            opt_res = self.reader_rx.recv() => {
                tracing::trace!("MavlinkReceiver::recv() got {opt_res:?}");
                match opt_res {
                    None => {
                        tracing::error!("MavlinkReceiver: sender closed");
                        Err(RecvError::SenderClosed)
                    },
                    Some(res) => res
                }
            }
        }
    }
}

/// An asynchronous implementation of a MAVLink sender.
pub struct MavlinkSender<M: Message> {
    write_err: tokio::sync::oneshot::Receiver<SendError<M>>,
    writer_tx: tokio::sync::mpsc::Sender<(MavHeader, M)>,
}

impl<M: Message> MavlinkSender<M> {
    /// Clone the sender.
    ///
    /// This is at least somewhat suboptimal, as a sender cloned this way will
    /// not perform error checking as done in [`Self::send`].
    pub fn hacky_clone_tx(&self) -> tokio::sync::mpsc::Sender<(MavHeader, M)> {
        self.writer_tx.clone()
    }
}

impl<M: Message + Debug> MavlinkSender<M> {
    pub async fn send(&mut self, msg: (MavHeader, M)) -> Result<(), SendError<M>> {
        tracing::trace!("MavlinkSender::send({msg:?})");
        tokio::select! {
            err = &mut self.write_err => {
                tracing::error!("MavlinkSender: write error: {err:?}");
                Err(flatten2(err))
            }
            res = self.writer_tx.send(msg) => res.map_err(|se| {
                tracing::error!("MavlinkSender: send error: {se:?}");
                SendError::MpscSendError(se)
            }),
        }
    }
}

#[derive(thiserror::Error, Debug)]
pub enum RecvError {
    #[error("IO error {0}")]
    Io(#[from] std::io::Error),
    #[error("sending channel closed")]
    SenderClosed,
    #[error("receiving thread panicked {0}")]
    OneshotRecv(#[from] tokio::sync::oneshot::error::RecvError),
}

#[derive(thiserror::Error)]
pub enum SendError<M: Message> {
    #[error("IO error {0}")]
    Io(#[from] std::io::Error),
    #[error("receiving side closed")]
    ReceiverClosed,
    #[error("sending thread panicked {0}")]
    OneshotRecv(#[from] tokio::sync::oneshot::error::RecvError),
    #[error("mpsc send error {0}")]
    MpscSendError(tokio::sync::mpsc::error::SendError<(MavHeader, M)>),
    #[error("MAVLink2 only message")]
    MAVLink2Only,
}

impl<M: Message> std::fmt::Debug for SendError<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(f, "SendError<M>::")?;
        match self {
            SendError::Io(e) => write!(f, "Io({e:?})"),
            SendError::ReceiverClosed => write!(f, "ReceiverClosed"),
            SendError::OneshotRecv(e) => write!(f, "OneshotRecv({e:?})"),
            SendError::MpscSendError(_e) => write!(f, "MpscSendError(_)"),
            SendError::MAVLink2Only => write!(f, "MAVLink2Only"),
        }
    }
}

/// Read loop, launched on its own thread.
///
/// Closing the asynchronous receiver requests a clean exit.
fn reader<M: Message + std::fmt::Debug>(
    mut rdr: impl Read,
    tx: tokio::sync::mpsc::Sender<Result<(MavHeader, M), RecvError>>,
    protocol_version: MavlinkVersion,
) -> Result<(), RecvError> {
    loop {
        match mavlink::read_versioned_msg(&mut rdr, protocol_version) {
            Ok(val) => {
                if tx.blocking_send(Ok(val)).is_err() {
                    return Ok(());
                }
            }
            Err(MessageReadError::Io(e)) => {
                return Err(e.into());
            }
            Err(MessageReadError::Parse(_)) => continue,
        }
    }
}

/// Write loop, launched on its own thread.
///
/// Closing all asynchronous senders requests a clean exit.
fn writer<M: Message + std::fmt::Debug>(
    mut wtr: impl Write,
    mut rx: tokio::sync::mpsc::Receiver<(MavHeader, M)>,
    protocol_version: MavlinkVersion,
) -> Result<(), SendError<M>> {
    loop {
        let (header, data) = match rx.blocking_recv() {
            Some(msg) => msg,
            None => return Ok(()),
        };
        match mavlink::write_versioned_msg(&mut wtr, protocol_version, header, &data) {
            Ok(_sz) => {}
            Err(mavlink::error::MessageWriteError::Io(e)) => {
                return Err(e.into());
            }
        }
    }
}

/// Takes `Read` and `Write` implementations to MAVLink device and returns a
/// [MavlinkConnection].
///
/// Reading and writing is handled by two newly spawned threads.
pub fn spawn_mavlink_threads<M: Message + Send + std::fmt::Debug + 'static>(
    rdr: impl Read + Send + 'static,
    wtr: impl Write + Send + 'static,
    read_channel_size: usize,
    write_channel_size: usize,
    protocol_version: MavlinkVersion,
) -> std::io::Result<MavlinkConnection<M>> {
    let (reader_tx, reader_rx) = tokio::sync::mpsc::channel(read_channel_size);
    let (read_thread_result_tx, read_thread_result_rx) = tokio::sync::oneshot::channel();
    // As below, we do not bother keeping the std::thread::JoinHandle<_> because
    // we use oneshot channels to signal that the thread has ended.
    std::thread::spawn(move || {
        if let Err(error) = reader(rdr, reader_tx, protocol_version) {
            // The asynchronous receiver may already have been dropped during
            // ordinary application shutdown.
            let _ = read_thread_result_tx.send(error);
        }
    });

    let (writer_tx, writer_rx) = tokio::sync::mpsc::channel(write_channel_size);
    let (write_thread_result_tx, write_thread_result_rx) = tokio::sync::oneshot::channel();
    // As above, we do not bother keeping the std::thread::JoinHandle<_> because
    // we use oneshot channels to signal that the thread has ended.
    std::thread::spawn(move || {
        if let Err(error) = writer(wtr, writer_rx, protocol_version) {
            // The asynchronous receiver may already have been dropped during
            // ordinary application shutdown.
            let _ = write_thread_result_tx.send(error);
        }
    });

    Ok(MavlinkConnection {
        rx: MavlinkReceiver {
            read_err: read_thread_result_rx,
            reader_rx,
        },
        tx: MavlinkSender {
            write_err: write_thread_result_rx,
            writer_tx,
        },
    })
}

fn flatten(full: Result<RecvError, tokio::sync::oneshot::error::RecvError>) -> RecvError {
    match full {
        Ok(res) => res,
        Err(e) => e.into(),
    }
}

fn flatten2<M: Message>(
    full: Result<SendError<M>, tokio::sync::oneshot::error::RecvError>,
) -> SendError<M> {
    match full {
        Ok(res) => res,
        Err(e) => e.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reader_exits_cleanly_when_receiver_is_dropped() {
        let message =
            mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA::default());
        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };
        let mut bytes = Vec::new();
        mavlink::write_versioned_msg(&mut bytes, MavlinkVersion::V1, header, &message).unwrap();
        let (tx, rx) = tokio::sync::mpsc::channel(1);
        drop(rx);

        let result = reader::<mavlink::common::MavMessage>(
            std::io::Cursor::new(bytes),
            tx,
            MavlinkVersion::V1,
        );

        assert!(result.is_ok());
    }

    #[test]
    fn writer_exits_cleanly_when_all_senders_are_dropped() {
        let (tx, rx) = tokio::sync::mpsc::channel(1);
        drop(tx);

        let result = writer::<mavlink::common::MavMessage>(Vec::new(), rx, MavlinkVersion::V1);

        assert!(result.is_ok());
    }
}
