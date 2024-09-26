use color_eyre::eyre::{self as anyhow, Result, WrapErr};
use flo_core::MotorValueCache;
use futures::{SinkExt, StreamExt};
use tokio::sync::watch;
use tokio_serial::SerialPortBuilderExt;
use tracing as log;

use tilta_dongle_comms::TitlaFocusDongleMessage as MyMessage;

type MyCodec = json_lines::codec::JsonLinesCodec<MyMessage, MyMessage>;

pub async fn run_tilta_loop(port: &str, mut rx: watch::Receiver<MotorValueCache>) -> Result<()> {
    let baud_rate = 115_200;

    let serial_device = tokio_serial::new(port, baud_rate)
        .open_native_async()
        .with_context(|| format!("Failed to open flo-tilta-dongle serial device {}", port))?;

    let framed = tokio_util::codec::Framed::new(serial_device, MyCodec::default());

    let (mut device_tx, mut device_rx) = framed.split();

    let (start_tx, mut start_rx) = tokio::sync::oneshot::channel();

    // Send request for the firmware version.
    let jh = tokio::spawn(async move {
        use tokio::sync::oneshot::error::TryRecvError;
        loop {
            match start_rx.try_recv() {
                Ok(()) => break,
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Closed) => {
                    anyhow::bail!("did not get version")
                }
            };
            device_tx.send(MyMessage::VersionRequest).await?;
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
        }
        Ok(device_tx)
    });

    // Check the firmware version is what we expect
    while let Some(msg) = device_rx.next().await {
        match msg.unwrap() {
            MyMessage::VersionResponse(version) => {
                assert_eq!(version, MyMessage::VERSION);
                start_tx.send(()).unwrap(); // todo remove unwrap
                break;
            }
            other => {
                log::error!("unexpected message: {other:?}");
            }
        }
        // Ok(())
    }

    // Wait for the firmware version check to complete.
    let mut device_tx = jh.await??;

    loop {
        // The watch channel will hold only the most recent value but it gets
        // updated constantly with the most recent data. By constantly querying
        // and sending it, we should send only the freshest data over serial as
        // fast as possible while avoiding to send stale data.
        //
        // If the serial is faster than the incoming watch data, we will wait on
        // rx.changed(). If incoming watch data is faster than serial, we will
        // send most recent data.
        let current_motors: MotorValueCache = {
            rx.changed().await?;
            // hold lock on watch channel only briefly and not across `await`.
            rx.borrow_and_update().clone()
        };

        if current_motors.focus.is_nan() {
            continue;
        }

        let fpos = current_motors.focus.clamp(0.0, 4095.0).round() as i32;

        log::trace!("serial loop got message {current_motors:?}");
        device_tx.send(MyMessage::SetPos(fpos)).await?;
    }
}
