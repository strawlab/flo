use color_eyre::eyre::{self as anyhow, Result};
use futures::{SinkExt, StreamExt};
use tokio::sync::watch;
use tokio_serial::SerialStream;
use tracing as log;

use flo_core::{MotorConfig, MotorDriveMode, MotorValueCache, PwmConfig, PwmSerial, PwmState};

use super::codec::JsonLinesCodec;

pub(crate) async fn run_rpi_pico_pwm_serial_loop(
    handle: &tokio::runtime::Handle,
    mut rx: watch::Receiver<MotorValueCache>,
    serial_device: SerialStream,
    pwm_pan_config: PwmConfig,
    pan_motor_config: MotorConfig,
    pwm_tilt_config: PwmConfig,
    tilt_motor_config: MotorConfig,
) -> Result<()> {
    let framed = tokio_util::codec::Framed::new(serial_device, JsonLinesCodec::new());

    let (mut device_tx, mut device_rx) = framed.split();

    let (start_tx, mut start_rx) = tokio::sync::oneshot::channel();

    // Send request for the firmware version.
    let jh = handle.spawn(async move {
        use tokio::sync::oneshot::error::TryRecvError;
        loop {
            match start_rx.try_recv() {
                Ok(()) => break,
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Closed) => {
                    anyhow::bail!("did not get version")
                }
            };
            device_tx.send(PwmSerial::VersionRequest).await?;
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
        }
        Ok(device_tx)
    });

    // Check the firmware version is what we expect
    while let Some(msg) = device_rx.next().await {
        match msg.unwrap() {
            PwmSerial::VersionResponse(version) => {
                assert_eq!(version, flo_core::DATATYPES_VERSION);
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
        let current_motors = {
            rx.changed().await?;
            // hold lock on watch channel only briefly and not across `await`.
            rx.borrow_and_update().clone()
        };

        if current_motors.drivemode != MotorDriveMode::Position {
            panic!(
                "only supporting position mode for servos, got {:?}",
                current_motors.drivemode
            );
        }

        let pan = pwm_pan_config.pwm(current_motors.pan, &pan_motor_config);
        let tilt = pwm_tilt_config.pwm(current_motors.tilt, &tilt_motor_config);

        log::trace!("serial loop got message {current_motors:?}");
        // Send message, but timeout with error if it is not sent within one second.
        tokio::time::timeout(
            std::time::Duration::from_secs(1),
            device_tx.send(PwmSerial::Set(PwmState {
                pan,
                tilt,
                enabled: true,
            })),
        )
        .await??;
    }
}
