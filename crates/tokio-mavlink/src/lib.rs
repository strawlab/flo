use std::{
    future::Future,
    io::{Read, Write},
    pin::Pin,
};

use futures::future::FutureExt;

use mavlink::{error::MessageReadError, MavHeader, MavlinkVersion, Message};

/// ensure that we never instantiate a NeverOk type
macro_rules! assert_never {
    ($never: expr) => {{
        let _: NeverOk = $never;
        unreachable!("NeverOk was instantiated");
    }};
}

pub struct MavlinkConnection<M: Message> {
    pub rx: MavlinkReceiver<M>,
    pub tx: MavlinkSender<M>,
}

impl<M: Message> MavlinkConnection<M> {
    pub fn split(self) -> (MavlinkSender<M>, MavlinkReceiver<M>) {
        (self.tx, self.rx)
    }
}

/// An asynchronous implementation of a MAVLink receiver.
pub struct MavlinkReceiver<M: Message> {
    read_err: Pin<Box<dyn Future<Output = RecvError> + Send>>,
    reader_rx: tokio::sync::mpsc::Receiver<Result<(MavHeader, M), RecvError>>,
}

impl<M: Message> MavlinkReceiver<M> {
    pub async fn recv(&mut self) -> Result<(MavHeader, M), RecvError> {
        tokio::select! {
            err = &mut self.read_err => {
                Err(err)
            }
            opt_res = self.reader_rx.recv() => {
                match opt_res {
                    None => {Err(RecvError::SenderClosed)},
                    Some(res) => res
                }
            }
        }
    }
}

/// An asynchronous implementation of a MAVLink sender.
pub struct MavlinkSender<M: Message> {
    write_err: Pin<Box<dyn Future<Output = SendError<M>> + Send>>,
    writer_tx: tokio::sync::mpsc::Sender<(MavHeader, M)>,
}

impl<M: Message> MavlinkSender<M> {
    pub async fn send(&mut self, msg: (MavHeader, M)) -> Result<(), SendError<M>> {
        tokio::select! {
            err = &mut self.write_err => {
                Err(err)
            }
            res = self.writer_tx.send(msg) => res.map_err(|se| SendError::MpscSendError(se)),
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
}

impl<M: Message> std::fmt::Debug for SendError<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(f, "SendError<M>::")?;
        match self {
            SendError::Io(e) => write!(f, "Io({e:?})"),
            SendError::ReceiverClosed => write!(f, "ReceiverClosed"),
            SendError::OneshotRecv(e) => write!(f, "OneshotRecv({e:?})"),
            SendError::MpscSendError(_e) => write!(f, "MpscSendError(_)"),
        }
    }
}

/// A zero-sized type which is never created to indicate that Ok(_) never
/// happens.
#[derive(Debug)]
enum NeverOk {}

/// Read loop, launched on own thread. Returns only on error.
fn reader<M: Message>(
    mut rdr: impl Read,
    tx: tokio::sync::mpsc::Sender<Result<(MavHeader, M), RecvError>>,
    protocol_version: MavlinkVersion,
) -> Result<NeverOk, RecvError> {
    loop {
        match mavlink::read_versioned_msg(&mut rdr, protocol_version) {
            Ok(val) => tx.blocking_send(Ok(val)).unwrap(),
            Err(MessageReadError::Io(e)) => {
                return Err(e.into());
            }
            Err(MessageReadError::Parse(_)) => continue,
        }
    }
}

/// Write loop, launched on own thread. Returns only on error.
fn writer<M: Message>(
    mut wtr: impl Write,
    mut rx: tokio::sync::mpsc::Receiver<(MavHeader, M)>,
    protocol_version: MavlinkVersion,
) -> Result<NeverOk, SendError<M>> {
    loop {
        let (header, data) = match rx.blocking_recv() {
            Some(msg) => msg,
            None => {
                return Err(SendError::ReceiverClosed);
            }
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
pub fn open<M: Message + Send + 'static>(
    rdr: impl Read + Send + 'static,
    wtr: impl Write + Send + 'static,
    read_channel_size: usize,
    write_channel_size: usize,
    protocol_version: MavlinkVersion,
) -> std::io::Result<MavlinkConnection<M>> {
    let (reader_tx, reader_rx) = tokio::sync::mpsc::channel(read_channel_size);
    let (read_thread_result_tx, read_thread_result_rx) = tokio::sync::oneshot::channel();
    std::thread::spawn(move || match reader(rdr, reader_tx, protocol_version) {
        Ok(never) => assert_never!(never),
        Err(e) => read_thread_result_tx.send(e).unwrap(),
    });

    let (writer_tx, writer_rx) = tokio::sync::mpsc::channel(write_channel_size);
    let (write_thread_result_tx, write_thread_result_rx) = tokio::sync::oneshot::channel();
    std::thread::spawn(move || match writer(wtr, writer_rx, protocol_version) {
        Ok(never) => assert_never!(never),
        Err(e) => write_thread_result_tx.send(e).unwrap(),
    });

    Ok(MavlinkConnection {
        rx: MavlinkReceiver {
            read_err: Box::pin(read_thread_result_rx.map(flatten)),
            reader_rx,
        },
        tx: MavlinkSender {
            write_err: Box::pin(write_thread_result_rx.map(flatten2)),
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
