use eyre::{Result, WrapErr};
use futures::{
    stream::{SplitSink, SplitStream},
    SinkExt, StreamExt,
};
use simplebgc::{IncomingCommand, OutgoingCommand, ParamsQuery, Payload, RollPitchYaw, V2Codec};
use std::time::Duration;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tokio_util::codec::Framed;

use flo_core::{
    Angle, FloatType, GimbalConfig, MotorDriveMode, MotorPositionResult, MotorValueCache,
    SaveToDiskMsg,
};

pub(crate) mod custom_messages;

const BAUD_RATE: u32 = 115_200;

struct InitializationResult {
    messages_tx: SplitSink<Framed<SerialStream, V2Codec>, OutgoingCommand>,
    messages_rx: SplitStream<Framed<SerialStream, V2Codec>>,
    offset_yaw: f64,
    offset_pitch: f64,
}

async fn initialize_gimbals(
    mut messages_tx: SplitSink<Framed<SerialStream, V2Codec>, OutgoingCommand>,
    mut messages_rx: SplitStream<Framed<SerialStream, V2Codec>>,
) -> Result<InitializationResult> {
    {
        //send control config (to disable confirmation responses)
        let ax_cfg = simplebgc::AxisControlConfigParams {
            angle_lpf: 0,
            speed_lpf: 0,
            rc_lpf: 0,
            acc_limit: 0,
            jerk_slope: 0,
            reserved: 0,
        };
        messages_tx
            .send(OutgoingCommand::ControlConfig(
                simplebgc::ControlConfigData {
                    timeout_ms: 7000,
                    priority_ch1: 0,
                    priority_ch2: 0,
                    priority_ch3: 0,
                    priority_ch4: 0,
                    priority_thischannel: 0,
                    axis_config: RollPitchYaw::<simplebgc::AxisControlConfigParams> {
                        yaw: ax_cfg,
                        pitch: ax_cfg,
                        roll: ax_cfg,
                    },
                    rc_expo_rate: 0,
                    flags: simplebgc::ControlConfigFlags::NoConfirm.into(),
                    reserved: [0; 10],
                },
            ))
            .await?;
    }

    messages_tx
        .send(OutgoingCommand::ReadParamsExt(ParamsQuery {
            profile_id: 0,
        }))
        .await?;

    let mut results;
    loop {
        let msg = messages_rx
            .next()
            .await
            .ok_or_else(|| eyre::eyre!("no response from gimbal"))??;
        match msg {
            IncomingCommand::ReadParamsExt(cmd) => {
                let (y, p, r) = (
                    cmd.encoder_offset.yaw,
                    cmd.encoder_offset.pitch,
                    cmd.encoder_offset.roll,
                );
                tracing::info!("gimbal encoder offsets (raw) y-p-r: {y}, {p}, {r}");
                let offset_yaw = (cmd.encoder_offset.yaw as f64) / ((1 << 14) as f64);
                let offset_pitch = (cmd.encoder_offset.pitch as f64) / ((1 << 14) as f64);

                results = InitializationResult {
                    messages_tx,
                    messages_rx,
                    offset_yaw,
                    offset_pitch,
                };
                break;
            }
            _ => {
                tracing::info!("got some other message while waiting for encoder offsets");
            }
        }
    }

    {
        // Request realtime encoder data stream.
        let msg_data = custom_messages::RequestStreamIntervalCustom {
            interval: 1,
            realtime_data_custom_flags: 1 << 11 /*ENCODER_RAW24*/ | 1 << 0 /*IMU_ANGLES*/ | 1 << 4, /*GYRO_DATA*/
            sync_to_data: true,
            ..Default::default()
        };

        results
            .messages_tx
            .send(OutgoingCommand::RawMessage(simplebgc::RawMessage {
                typ: simplebgc::constants::CMD_DATA_STREAM_INTERVAL,
                payload: Payload::to_bytes(&msg_data),
            }))
            .await
            .unwrap();
    }
    Ok(results)
}

enum InternalGimbalError {
    RxTimeout,
    Wrapped { report: eyre::Report },
}

fn wrap<T: Into<eyre::Report>>(orig: T) -> InternalGimbalError {
    InternalGimbalError::Wrapped {
        report: orig.into(),
    }
}

/// Run the gimbal motors.
///
/// The returned future only resolves in the case of error.
pub async fn run_gimbal_loop(
    rx: tokio::sync::watch::Receiver<MotorValueCache>,
    motor_position_tx: tokio::sync::mpsc::Sender<MotorPositionResult>,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    cfg: GimbalConfig,
) -> Result<()> {
    // Loop forever in case of timeout on gimbals.
    loop {
        // Initialize serial port.
        let serial_device = tokio_serial::new(&cfg.port_path, BAUD_RATE)
            .open_native_async()
            .with_context(|| format!("Failed to open Gimbal serial device {}", cfg.port_path))?;

        // Establish initial connection to gimbals.
        let framed = tokio_util::codec::Framed::new(serial_device, V2Codec::default());
        let (messages_tx, messages_rx) = framed.split();
        let gimbal_config_fut = initialize_gimbals(messages_tx, messages_rx);
        let ir = tokio::time::timeout(Duration::from_millis(5000), gimbal_config_fut).await??;

        // Run the main gimbal loops forever. This future only completes when
        // the gimbal is done, which only happens on an error. If the error is
        // RxTimeout, restart the whole thing. Otherwise, return the error to
        // the caller.
        match run_gimbal_loop_internal(
            ir,
            rx.clone(),
            motor_position_tx.clone(),
            floz_logger.clone(),
            cfg.clone(),
        )
        .await
        {
            Ok(()) => {
                unreachable!();
            }
            Err(InternalGimbalError::RxTimeout) => {
                tracing::error!("Timeout elapsed reading from gimbals. Resetting.");
            }
            Err(InternalGimbalError::Wrapped { report: source }) => {
                return Err(source);
            }
        }
    }
}

/// Run an already-initialized gimbal. This returns only on error.
async fn run_gimbal_loop_internal(
    ir: InitializationResult,
    mut rx: tokio::sync::watch::Receiver<MotorValueCache>,
    motor_position_tx: tokio::sync::mpsc::Sender<MotorPositionResult>,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    cfg: GimbalConfig,
) -> Result<(), InternalGimbalError> {
    let InitializationResult {
        mut messages_tx,
        mut messages_rx,
        offset_yaw,
        offset_pitch,
    } = ir;
    let pan_rev: FloatType = if cfg.reverse_pan { -1.0 } else { 1.0 };
    let tilt_rev: FloatType = if cfg.reverse_tilt { -1.0 } else { 1.0 };

    // Save gimbal offsets
    floz_logger
        .send(SaveToDiskMsg::GimbalEncoderOffsets(
            flo_core::GimbalEncoderOffsets {
                pitch: offset_pitch,
                yaw: offset_yaw,
            },
        ))
        .map_err(wrap)?;

    //loop for encoder readout
    let rx_loop = async {
        let mut last_imu_angles = RollPitchYaw::<FloatType> {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        loop {
            let msg = tokio::time::timeout(Duration::from_millis(500), messages_rx.next())
                .await
                .map_err(|_| InternalGimbalError::RxTimeout)?
                .unwrap()
                .map_err(wrap)?;
            let local = chrono::Local::now();
            match msg {
                IncomingCommand::RawMessage(msg) => {
                    match msg.typ {
                        simplebgc::constants::CMD_REALTIME_DATA_CUSTOM => {
                            let msg_data: custom_messages::RealTimeDataCustomFlo =
                                Payload::from_bytes(msg.payload).unwrap();

                            floz_logger
                                .send(SaveToDiskMsg::GimbalEncoderData(to_msg(&local, &msg_data)))
                                .map_err(wrap)?;

                            let (pan_enc, tilt_enc) = {
                                //convert encoder data
                                let (roll, pitch, yaw): (i32, i32, i32) = (
                                    msg_data.encoder_raw24.roll.into(),
                                    msg_data.encoder_raw24.pitch.into(),
                                    msg_data.encoder_raw24.yaw.into(),
                                );
                                //convert to fractions of turn
                                let (roll, pitch, yaw) = (
                                    roll as f64 / (1 << 24) as f64,
                                    pitch as f64 / (1 << 24) as f64,
                                    yaw as f64 / (1 << 24) as f64,
                                );
                                //subtract offsets
                                let (roll, pitch, yaw) =
                                    (roll, pitch - offset_pitch, yaw - offset_yaw);
                                //reverse
                                let (pitch, yaw) = (pitch * tilt_rev, yaw * pan_rev);
                                // to degrees
                                let (_roll, pitch, yaw) =
                                    (roll * 360.0, pitch * 360.0, yaw * 360.0);
                                (Angle::from_degrees(yaw), Angle::from_degrees(pitch))
                            };

                            let (pan_imu, tilt_imu) = {
                                //convert imu angles
                                let (roll, pitch, yaw) = (
                                    msg_data.imu_angles.roll,
                                    msg_data.imu_angles.pitch,
                                    msg_data.imu_angles.yaw,
                                );
                                //to degrees
                                const UNIT: FloatType = 0.02197265625;
                                let (roll, pitch, yaw) = (
                                    roll as FloatType * UNIT,
                                    pitch as FloatType * UNIT,
                                    yaw as FloatType * UNIT,
                                );
                                //unwarp (prevent angle discontinuity jump from 179 to -179)
                                let (roll, pitch, yaw) = (
                                    roll + FloatType::round((last_imu_angles.roll - roll) / 360.0)
                                        * 360.0,
                                    pitch
                                        + FloatType::round((last_imu_angles.pitch - pitch) / 360.0)
                                            * 360.0,
                                    yaw + FloatType::round((last_imu_angles.yaw - yaw) / 360.0)
                                        * 360.0,
                                );
                                (
                                    last_imu_angles.roll,
                                    last_imu_angles.pitch,
                                    last_imu_angles.yaw,
                                ) = (roll, pitch, yaw);
                                (
                                    Angle::from_degrees(yaw * pan_rev),
                                    Angle::from_degrees(pitch * tilt_rev),
                                )
                            };

                            let (vpan_imu, vtilt_imu) = {
                                //convert imu rotation rates
                                let (roll, pitch, yaw) = (
                                    msg_data.gyro_data.roll,
                                    msg_data.gyro_data.pitch,
                                    msg_data.gyro_data.yaw,
                                );
                                //to radians
                                const UNIT: FloatType =
                                    0.06103701895 / 360.0 * std::f64::consts::TAU;
                                let (roll, _pitch, yaw) = (
                                    roll as FloatType * UNIT,
                                    pitch as FloatType * UNIT,
                                    yaw as FloatType * UNIT,
                                );
                                (
                                    Some(yaw * pan_rev),
                                    Some(-roll * tilt_rev), //why not pitch? probably it is because of how imu is mounted
                                )
                            };

                            let ret = MotorPositionResult {
                                local,
                                pan_enc,
                                tilt_enc,
                                pan_imu,
                                tilt_imu,
                                vpan_imu,
                                vtilt_imu,
                            };

                            motor_position_tx.send(ret).await.unwrap();
                        }
                        _ => {
                            tracing::info!("unknown message #{}", msg.typ);
                        }
                    }
                }
                IncomingCommand::CommandConfirm(_) => {}
                msg => {
                    tracing::info!("got some other message from the gimbal: {msg:?}");
                }
            }
        }
        #[allow(unreachable_code)]
        Ok::<_, InternalGimbalError>(())
    };

    //loop for sending target position/speed
    let control_loop = async {
        use enumflags2::BitFlags;
        use simplebgc::{
            AxisControlFlags, AxisControlMode, AxisControlParams, AxisControlState, ControlData,
            ControlFormat,
        };

        let mut last_mode = (MotorDriveMode::Position, true);

        loop {
            let current_motors = {
                rx.changed().await.unwrap();
                rx.borrow_and_update().clone()
            };

            let new_mode = (current_motors.drivemode, current_motors.rel_frame);

            if last_mode != new_mode {
                // if we just keep sending Control messages to the gimbal but change modes,
                // the gimbal starts silently ignoring the commands. This is probably a gimbal
                // bug, but may as well be an undocumented feature. So, to make it work, we
                // have to first switch to NoControl mode, and then to the new mode.
                messages_tx
                    .send(OutgoingCommand::Control(ControlData {
                        mode: ControlFormat::Legacy(AxisControlState {
                            mode: AxisControlMode::NoControl,
                            flags: BitFlags::<AxisControlFlags>::default(),
                        }),
                        axes: RollPitchYaw {
                            roll: AxisControlParams { speed: 0, angle: 0 },
                            pitch: AxisControlParams { speed: 0, angle: 0 },
                            yaw: AxisControlParams { speed: 0, angle: 0 },
                        },
                    }))
                    .await?;
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            last_mode = new_mode;

            match current_motors.drivemode {
                MotorDriveMode::Position => {
                    const ANGLE_UNIT: FloatType = 0.02197265625;
                    fn convert(x: FloatType) -> i16 {
                        let x_cyc = x / std::f64::consts::TAU;
                        let x_cyc = x_cyc - x_cyc.round();
                        (x_cyc * 360.0 / ANGLE_UNIT).round() as i16
                    }

                    let (mode, flags) = if current_motors.rel_frame {
                        //(AxisControlMode::Angle, BitFlags::<AxisControlFlags>::default())
                        (
                            AxisControlMode::RelFrame,
                            BitFlags::<AxisControlFlags>::from(AxisControlFlags::MixFollow),
                        )
                    } else {
                        //(AxisControlMode::RelFrame, BitFlags::<AxisControlFlags>::from(AxisControlFlags::MixFollow))
                        (
                            AxisControlMode::AngleShortest,
                            BitFlags::<AxisControlFlags>::from(AxisControlFlags::TargetPrecise),
                        )
                    };

                    messages_tx
                        .send(OutgoingCommand::Control(ControlData {
                            mode: ControlFormat::Legacy(AxisControlState { mode, flags }),
                            axes: RollPitchYaw {
                                roll: AxisControlParams { speed: 0, angle: 0 },
                                pitch: AxisControlParams {
                                    speed: 0,
                                    angle: convert(current_motors.tilt.0 * tilt_rev),
                                },
                                yaw: AxisControlParams {
                                    speed: 0,
                                    angle: convert(current_motors.pan.0 * pan_rev),
                                },
                            },
                        }))
                        .await?;
                }
                MotorDriveMode::Speed => {
                    const SPEED_UNIT: FloatType = 0.1220740379; //1 sent to sbgc corresponds to this many degrees per second
                    fn convert(x: FloatType) -> i16 {
                        (x.to_degrees() / SPEED_UNIT)
                            .round()
                            .clamp(-32768.0, 32767.0) as i16
                    }

                    messages_tx
                        .send(OutgoingCommand::Control(ControlData {
                            mode: ControlFormat::Legacy(AxisControlState {
                                mode: AxisControlMode::Speed,
                                flags: 0.try_into().unwrap(), //is there a way to avoid this nonsense and just say "0"?   ~~~victor
                            }),
                            axes: RollPitchYaw {
                                roll: AxisControlParams { speed: 0, angle: 0 },
                                pitch: AxisControlParams {
                                    speed: convert(current_motors.vtilt * tilt_rev),
                                    angle: 0,
                                },
                                yaw: AxisControlParams {
                                    speed: convert(current_motors.vpan * pan_rev),
                                    angle: 0,
                                },
                            },
                        }))
                        .await?;
                }
            }

            //limit the rate of sending the commands.
            // This is a workaround for some mysterious bug, where
            // incoming messages start to arrive in large packs every
            // 0.5 seconds or so, instead of almost immediately.
            tokio::time::sleep(Duration::from_secs_f64(0.015)).await;
        }
        #[allow(unreachable_code)]
        Ok::<_, eyre::Error>(())
    };

    tokio::pin!(rx_loop);
    tokio::pin!(control_loop);

    // Futures should run forever, but if one ends it is an error and we should raise it.
    tokio::select! {
        rx_result = &mut rx_loop => {
            rx_result?;
        }
        control_result = &mut control_loop => {
            control_result.map_err(|report| InternalGimbalError::Wrapped{report})?;
        }
    }

    #[allow(unreachable_code)]
    Ok(())
}

fn to_msg(
    local: &chrono::DateTime<chrono::Local>,
    orig: &custom_messages::RealTimeDataCustomFlo,
) -> flo_core::GimbalEncoderData {
    flo_core::GimbalEncoderData {
        local: *local,
        timestamp_ms: orig.timestamp_ms,
        imu_angle_roll: orig.imu_angles.roll.into(),
        imu_angle_pitch: orig.imu_angles.pitch.into(),
        imu_angle_yaw: orig.imu_angles.yaw.into(),
        gyro_data_roll: orig.gyro_data.roll.into(),
        gyro_data_pitch: orig.gyro_data.pitch.into(),
        gyro_data_yaw: orig.gyro_data.yaw.into(),
        encoder_raw24_roll: orig.encoder_raw24.roll.into(),
        encoder_raw24_pitch: orig.encoder_raw24.pitch.into(),
        encoder_raw24_yaw: orig.encoder_raw24.yaw.into(),
    }
}
