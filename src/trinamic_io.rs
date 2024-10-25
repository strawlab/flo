use color_eyre::eyre::Result;
use tokio::sync::watch;
use tracing as log;

use flo_core::{MotorDriveMode, MotorPositionResult, MotorValueCache, TrinamicAxisConfig};

pub(crate) async fn run_trinamic_loop(
    mut rx: watch::Receiver<MotorValueCache>,
    motor_position_tx: tokio::sync::mpsc::Sender<MotorPositionResult>,
    mut pan_device: trinamic::Motor,
    mut tilt_device: trinamic::Motor,
    mut focus_device: Option<trinamic::Motor>,
    pan_trinamic_config: TrinamicAxisConfig,
    tilt_trinamic_config: TrinamicAxisConfig,
) -> Result<()> {
    loop {
        // The watch channel will hold only the most recent value but it gets
        // updated constantly with the most recent data. By constantly querying
        // and sending it, we should send only the freshest data over USB as
        // fast as possible while avoiding to send stale data.
        //
        // If the USB is faster than the incoming watch data, we will wait on
        // rx.changed(). If incoming watch data is faster than USB, we will
        // send most recent data.
        let motor_values = {
            rx.changed().await?;
            // hold lock on watch channel only briefly and not across `await`.
            rx.borrow_and_update().clone()
        };
        if motor_values.drivemode != MotorDriveMode::Position {
            panic!(
                "only supporting position mode for trinamic, got {:?}",
                motor_values.drivemode
            );
        }
        log::trace!("trinamic loop got message {motor_values:?}");
        let raw_pan_pos = pan_trinamic_config.convert(motor_values.pan);
        let raw_tilt_pos = tilt_trinamic_config.convert(motor_values.tilt);
        let raw_focus_pos = motor_values.focus.round() as i32;

        // send pan and tilt and focus commands concurrently, returning when both are done.

        if let Some(focus_device) = &mut focus_device {
            //if we have a focus device, include focus command
            let (pan_result, tilt_result, focus_result) = tokio::join!(
                pan_device.move_to_absolute_position(raw_pan_pos),
                tilt_device.move_to_absolute_position(raw_tilt_pos),
                focus_device.move_to_absolute_position(raw_focus_pos),
            );

            // check errors in sending the move command
            pan_result?;
            tilt_result?;
            focus_result?;
        } else {
            //if we have no focus device, send pan and tilt command only
            let (pan_result, tilt_result) = tokio::join!(
                pan_device.move_to_absolute_position(raw_pan_pos),
                tilt_device.move_to_absolute_position(raw_tilt_pos),
            );

            // check errors in sending the move command
            pan_result?;
            tilt_result?;
        }

        // send pan and tilt queries concurrently, returning when both are done.
        let (pan_result, tilt_result) =
            tokio::join!(pan_device.query_position(), tilt_device.query_position(),);
        let local = chrono::Local::now();
        // check errors in sending the move command
        let pan_pos = pan_result?;
        let tilt_pos = tilt_result?;

        // broadcast queried motor position
        motor_position_tx
            .send(MotorPositionResult {
                local,
                pan_enc: pan_trinamic_config.convert_back(pan_pos),
                tilt_enc: tilt_trinamic_config.convert_back(tilt_pos),
                pan_imu: pan_trinamic_config.convert_back(pan_pos),
                tilt_imu: tilt_trinamic_config.convert_back(tilt_pos),
                vpan_imu: None,
                vtilt_imu: None,
            })
            .await?;
    }
}
