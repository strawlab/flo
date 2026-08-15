use eyre::{Result, WrapErr};
use futures::{
    SinkExt, StreamExt,
    stream::{SplitSink, SplitStream},
};
use simplebgc::{IncomingCommand, OutgoingCommand, ParamsQuery, Payload, RollPitchYaw, V2Codec};
use std::time::{Duration, Instant};
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tokio_util::codec::Framed;

use flo_core::{
    Angle, FloatType, GimbalConfig, MotorDriveMode, MotorPositionResult, MotorValueCache,
    SaveToDiskMsg,
};

pub(crate) mod custom_messages;
mod provenance;

const BAUD_RATE: u32 = 115_200;
/// A short stream warm-up used before sending the first control command.
const STREAM_WARMUP_PACKETS: u64 = 100;
const STREAM_WARMUP_SETTLE_DURATION: Duration = Duration::from_millis(250);
/// Let bytes already queued by a previous run's realtime stream drain after we
/// unregister it. A full 255-byte controller buffer takes about 22 ms to send
/// at 115200 baud; this leaves comfortable margin.
const STARTUP_STREAM_DRAIN_DURATION: Duration = Duration::from_millis(50);
/// A parameter response is large enough that the controller may drop it when
/// its transmit buffer is busy. Repeat the required query until it arrives.
const OFFSETS_QUERY_RETRY_INTERVAL: Duration = Duration::from_millis(500);
const OFFSETS_RESPONSE_TIMEOUT: Duration = Duration::from_secs(5);
/// How long to keep collecting startup query responses once the required
/// encoder offsets have arrived.
///
/// Only the offsets are needed to fly. The identity and configuration queries
/// are provenance, and provenance must never be able to ground the aircraft --
/// if a board does not answer them, that is recorded and startup continues.
const PROVENANCE_COLLECT_TIMEOUT: Duration = Duration::from_millis(1500);

fn realtime_stream_command(interval: u16) -> OutgoingCommand {
    let msg_data = custom_messages::RequestStreamIntervalCustom {
        interval,
        realtime_data_custom_flags: 1 << 11 /*ENCODER_RAW24*/ | 1 << 0 /*IMU_ANGLES*/ | 1 << 4, /*GYRO_DATA*/
        sync_to_data: true,
        ..Default::default()
    };
    OutgoingCommand::RawMessage(simplebgc::RawMessage {
        typ: simplebgc::constants::CMD_DATA_STREAM_INTERVAL,
        payload: Payload::to_bytes(&msg_data),
    })
}

struct InitializationResult {
    messages_tx: SplitSink<Framed<SerialStream, V2Codec>, OutgoingCommand>,
    messages_rx: SplitStream<Framed<SerialStream, V2Codec>>,
    offset_yaw: f64,
    offset_pitch: f64,
    provenance: flo_core::GimbalProvenance,
    stream_requested_at: Instant,
}

async fn initialize_gimbals(
    mut messages_tx: SplitSink<Framed<SerialStream, V2Codec>, OutgoingCommand>,
    mut messages_rx: SplitStream<Framed<SerialStream, V2Codec>>,
    port_path: &str,
) -> Result<InitializationResult> {
    let initialization_started = Instant::now();
    tracing::debug!("starting SimpleBGC initialization");

    // Realtime streams survive the client that registered them. A previous
    // FLO run may therefore have left the board filling this serial link at
    // roughly 125 Hz. Unregister our exact stream before requesting the large
    // parameter response: the SimpleBGC protocol explicitly permits that
    // response to be skipped while the controller's transmit buffer is full.
    messages_tx
        .send(realtime_stream_command(0))
        .await
        .with_context(|| {
            format!("writing the first SimpleBGC command to the gimbal on {port_path}")
        })?;
    tokio::time::sleep(STARTUP_STREAM_DRAIN_DURATION).await;

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

    // Ask the board to describe itself, then for its stored parameters.
    //
    // Only ReadParamsExt is needed to fly -- it carries the encoder offsets.
    // The other three are provenance: BOARD_INFO_3 carries the microcontroller
    // id, which is the only hardware serial this controller has, and the
    // parameter sets carry the encoder calibration that turns a count into an
    // angle. All are read-only and cost a few milliseconds at 115200 baud.
    for query in [
        OutgoingCommand::BoardInfo,
        OutgoingCommand::BoardInfo3,
        OutgoingCommand::ReadParams3(ParamsQuery { profile_id: 0 }),
        OutgoingCommand::ReadParamsExt(ParamsQuery { profile_id: 0 }),
    ] {
        messages_tx.send(query).await?;
    }
    tracing::debug!("requested SimpleBGC encoder offsets and board provenance");

    // Collect until every query is answered, or until the provenance deadline
    // expires with the required offsets in hand. A board that does not
    // implement BOARD_INFO_3 must still fly.
    let mut queries = provenance::StartupQueries::default();
    // Messages that arrived but answered none of the startup queries -- most
    // often realtime stream frames a previous run left the board sending. They
    // prove the link itself is alive, which is the first thing to know when the
    // offsets never show up.
    let mut unrelated_messages = 0usize;
    let offsets_deadline = Instant::now() + OFFSETS_RESPONSE_TIMEOUT;
    let mut collect_deadline = offsets_deadline;
    let mut next_offsets_retry = Instant::now() + OFFSETS_QUERY_RETRY_INTERVAL;
    let mut offsets_query_attempts = 1usize;
    while !queries.is_complete() {
        let deadline_remaining = collect_deadline.saturating_duration_since(Instant::now());
        let retry_remaining = next_offsets_retry.saturating_duration_since(Instant::now());
        let msg = tokio::select! {
            () = tokio::time::sleep(deadline_remaining) => break,
            () = tokio::time::sleep(retry_remaining), if queries.params_ext.is_none() => {
                // The initial unregister may itself have arrived while the
                // board was booting. Repeat it before each retry so a stale
                // stream cannot keep starving the required response.
                messages_tx.send(realtime_stream_command(0)).await?;
                tokio::time::sleep(STARTUP_STREAM_DRAIN_DURATION).await;
                messages_tx
                    .send(OutgoingCommand::ReadParamsExt(ParamsQuery { profile_id: 0 }))
                    .await?;
                offsets_query_attempts += 1;
                next_offsets_retry = Instant::now() + OFFSETS_QUERY_RETRY_INTERVAL;
                tracing::debug!(offsets_query_attempts, "retried SimpleBGC encoder offsets query");
                continue;
            }
            msg = messages_rx.next() => match msg {
                None => return Err(eyre::eyre!(
                    "gimbal serial port {port_path} reached end-of-stream during startup, \
                     after answering {:?}. The device disappeared -- check for a USB \
                     disconnect or an unpowered controller.",
                    queries.answered_keys()
                )),
                Some(msg) => msg.with_context(|| {
                    format!("decoding SimpleBGC startup response from gimbal on {port_path}")
                })?,
            },
        };
        match msg {
            IncomingCommand::BoardInfo(cmd) => queries.board_info = Some(cmd),
            IncomingCommand::BoardInfo3(cmd) => queries.board_info_3 = Some(cmd),
            IncomingCommand::ReadParams3(cmd) | IncomingCommand::ReadParams(cmd) => {
                queries.params_3 = Some(cmd)
            }
            IncomingCommand::ReadParamsExt(cmd) => {
                let (y, p, r) = (
                    cmd.encoder_offset.yaw,
                    cmd.encoder_offset.pitch,
                    cmd.encoder_offset.roll,
                );
                tracing::info!("gimbal encoder offsets (raw) y-p-r: {y}, {p}, {r}");
                tracing::debug!(
                    elapsed = ?initialization_started.elapsed(),
                    offsets_query_attempts,
                    "received SimpleBGC encoder offsets"
                );
                queries.params_ext = Some(cmd);
                // Required state is now in hand. Give optional provenance its
                // own collection window rather than making it consume the
                // board-boot tolerance above.
                collect_deadline = Instant::now() + PROVENANCE_COLLECT_TIMEOUT;
            }
            msg => {
                unrelated_messages += 1;
                tracing::debug!(?msg, "received message while waiting for startup queries");
            }
        }
    }

    let params_ext = queries.params_ext.as_ref().ok_or_else(|| {
        let waited = initialization_started.elapsed();
        let answered = queries.answered_keys();
        if answered.is_empty() && unrelated_messages == 0 {
            // Nothing came back at all, so the fault is the link or the board,
            // not the query. This is the case a restart tends to fix.
            eyre::eyre!(
                "gimbal on {port_path} sent no reply of any kind within {waited:?} \
                 (queried board info and encoder offsets at {BAUD_RATE} baud). \
                 Check that the controller is powered, that {port_path} is the \
                 SimpleBGC board, and that no other process holds the port. \
                 A controller left wedged by a previous run needs a power cycle."
            )
        } else {
            // The board is talking, so the link is fine -- it just never
            // produced CMD_READ_PARAMS_EXT, which is the one response FLO
            // cannot fly without.
            eyre::eyre!(
                "gimbal on {port_path} did not return encoder offsets \
                 (CMD_READ_PARAMS_EXT, profile 0) within {waited:?}, though the link \
                 is alive: it answered {answered:?} and sent {unrelated_messages} \
                 unrelated message(s) across {offsets_query_attempts} query attempts. \
                 The board may have no stored parameters for profile 0."
            )
        }
    })?;
    let offset_yaw = (params_ext.encoder_offset.yaw as f64) / ((1 << 14) as f64);
    let offset_pitch = (params_ext.encoder_offset.pitch as f64) / ((1 << 14) as f64);

    let provenance = provenance::build(&queries);
    if provenance.unanswered.is_empty() {
        tracing::debug!("gimbal answered every startup provenance query");
    } else {
        tracing::warn!(
            unanswered = ?provenance.unanswered,
            "gimbal left some provenance queries unanswered; recording what it did return"
        );
    }
    if let Some(board) = &provenance.board {
        tracing::info!(
            mcu_id = %board.mcu_id,
            firmware = %board.firmware_version,
            profile = board.profile_current,
            fingerprint = provenance.config_fingerprint_sha256.as_deref().unwrap_or("-"),
            "gimbal controller identified"
        );
    }
    if let Some(encoders) = &provenance.encoders {
        // A direct-drive axis reads 1000 here. Anything else means the encoder
        // count is geared relative to the joint, which an analysis that assumes
        // a unit scale would silently get wrong.
        tracing::info!(
            gear_ratio_milli_ypr = ?(
                encoders.gear_ratio_milli.yaw,
                encoders.gear_ratio_milli.pitch,
                encoders.gear_ratio_milli.roll,
            ),
            "gimbal encoder gear ratios"
        );
    }

    let mut results = InitializationResult {
        messages_tx,
        messages_rx,
        offset_yaw,
        offset_pitch,
        provenance,
        // Replaced immediately after the stream request below.
        stream_requested_at: Instant::now(),
    };

    {
        // Request realtime encoder data stream.
        let stream_requested_at = Instant::now();
        results.messages_tx.send(realtime_stream_command(1)).await?;
        results.stream_requested_at = stream_requested_at;
        tracing::debug!("requested SimpleBGC realtime encoder stream");
    }
    Ok(results)
}

enum InternalGimbalError {
    RxTimeout,
    Shutdown,
    Wrapped { report: eyre::Report },
}

fn wrap<T: Into<eyre::Report>>(orig: T) -> InternalGimbalError {
    InternalGimbalError::Wrapped {
        report: orig.into(),
    }
}

/// Run the gimbal motors.
///
/// The returned future resolves when FLO shuts down or the gimbal encounters an
/// unrecoverable error.
pub async fn run_gimbal_loop(
    rx: tokio::sync::watch::Receiver<MotorValueCache>,
    motor_position_tx: tokio::sync::mpsc::Sender<MotorPositionResult>,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    cfg: GimbalConfig,
) -> Result<()> {
    // Loop forever in case of timeout on gimbals.
    loop {
        // Initialize serial port.
        tracing::debug!("Initializing port \"{}\", {BAUD_RATE} baud", cfg.port_path);
        let serial_device = tokio_serial::new(&cfg.port_path, BAUD_RATE)
            .open_native_async()
            .with_context(|| format!("Failed to open Gimbal serial device {}", cfg.port_path))?;

        // Establish initial connection to gimbals.
        let framed = tokio_util::codec::Framed::new(serial_device, V2Codec::default());
        let (messages_tx, messages_rx) = framed.split();
        let gimbal_config_fut = initialize_gimbals(messages_tx, messages_rx, &cfg.port_path);
        const INIT_TIMEOUT: Duration = Duration::from_secs(8);
        let ir = match tokio::time::timeout(INIT_TIMEOUT, gimbal_config_fut).await {
            Err(_elapsed) => {
                return Err(eyre::eyre!(
                    "gimbal startup on {} did not finish within {INIT_TIMEOUT:?}",
                    cfg.port_path
                ));
            }
            Ok(res) => {
                res.with_context(|| format!("initializing gimbal controller on {}", cfg.port_path))?
            }
        };
        let (stream_warmed_up_tx, stream_warmed_up_rx) = tokio::sync::watch::channel(false);

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
            stream_warmed_up_tx,
            stream_warmed_up_rx,
        )
        .await
        {
            Ok(()) => {
                unreachable!();
            }
            Err(InternalGimbalError::Shutdown) => {
                tracing::debug!("gimbal channels closed; ending gimbal task");
                return Ok(());
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
    stream_warmed_up_tx: tokio::sync::watch::Sender<bool>,
    mut stream_warmed_up_rx: tokio::sync::watch::Receiver<bool>,
) -> Result<(), InternalGimbalError> {
    let InitializationResult {
        mut messages_tx,
        mut messages_rx,
        offset_yaw,
        offset_pitch,
        provenance,
        stream_requested_at,
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

    // Save the controller's identity and stored configuration. Held by the
    // writer and re-emitted into every subsequent recording, so a recording
    // started hours after connection still says which board and which
    // calibration produced it.
    floz_logger
        .send(SaveToDiskMsg::GimbalProvenance(Box::new(provenance)))
        .map_err(wrap)?;

    //loop for encoder readout
    let rx_loop = async {
        let mut telemetry_count = 0_u64;
        let mut telemetry_since_stream_start = 0_u64;
        let mut first_custom_telemetry = true;
        let mut last_telemetry_report = Instant::now();
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
                            if first_custom_telemetry {
                                tracing::debug!(
                                    elapsed = ?stream_requested_at.elapsed(),
                                    "received first SimpleBGC custom realtime packet"
                                );
                                first_custom_telemetry = false;
                            }
                            telemetry_count += 1;
                            telemetry_since_stream_start += 1;
                            if telemetry_since_stream_start == STREAM_WARMUP_PACKETS {
                                stream_warmed_up_tx.send_replace(true);
                                tracing::info!(
                                    telemetry_since_stream_start,
                                    "SimpleBGC realtime stream warm-up complete"
                                );
                            }
                            let telemetry_elapsed = last_telemetry_report.elapsed();
                            if telemetry_elapsed >= Duration::from_secs(1) {
                                tracing::debug!(
                                    telemetry_count,
                                    telemetry_hz =
                                        telemetry_count as f64 / telemetry_elapsed.as_secs_f64(),
                                    "SimpleBGC realtime telemetry receive rate"
                                );
                                telemetry_count = 0;
                                last_telemetry_report = Instant::now();
                            }
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
                                (
                                    Angle::from_degrees(yaw).constrained_signed(),
                                    Angle::from_degrees(pitch).constrained_signed(),
                                )
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

                                (Some(yaw * pan_rev), Some(-roll * tilt_rev)) //why not pitch? probably it is because of how imu is mounted
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

                            motor_position_tx
                                .send(ret)
                                .await
                                .map_err(|_| InternalGimbalError::Shutdown)?;
                        }
                        _ => {
                            tracing::debug!(
                                command_id = msg.typ,
                                payload_len = msg.payload.len(),
                                elapsed = ?stream_requested_at.elapsed(),
                                "received non-custom SimpleBGC raw message after stream request"
                            );
                        }
                    }
                }
                IncomingCommand::CommandConfirm(confirm) => {
                    tracing::debug!(
                        ?confirm,
                        elapsed = ?stream_requested_at.elapsed(),
                        "received SimpleBGC command confirmation after stream request"
                    );
                }
                msg => {
                    tracing::debug!(
                        ?msg,
                        elapsed = ?stream_requested_at.elapsed(),
                        "received non-custom SimpleBGC message after stream request"
                    );
                }
            }
        }
        #[expect(unreachable_code)]
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
        let mut control_count = 0_u64;
        let mut last_control_report = Instant::now();

        if !*stream_warmed_up_rx.borrow() {
            tracing::debug!(
                stream_warmup_packets = STREAM_WARMUP_PACKETS,
                "waiting to send SimpleBGC controls until realtime stream warm-up completes"
            );
            stream_warmed_up_rx
                .changed()
                .await
                .map_err(|_| InternalGimbalError::Shutdown)?;
        }
        tracing::debug!(
            ?STREAM_WARMUP_SETTLE_DURATION,
            "waiting briefly before sending first SimpleBGC control"
        );
        tokio::time::sleep(STREAM_WARMUP_SETTLE_DURATION).await;

        loop {
            let current_motors = next_motor_value(&mut rx).await?;

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
                    .await
                    .map_err(wrap)?;
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
                        .await
                        .map_err(wrap)?;
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
                        .await
                        .map_err(wrap)?;
                }
            }

            control_count += 1;
            let control_elapsed = last_control_report.elapsed();
            if control_elapsed >= Duration::from_secs(1) {
                tracing::debug!(
                    control_count,
                    control_hz = control_count as f64 / control_elapsed.as_secs_f64(),
                    drive_mode = ?current_motors.drivemode,
                    relative_frame = current_motors.rel_frame,
                    pan_target_radians = current_motors.pan.0,
                    tilt_target_radians = current_motors.tilt.0,
                    pan_speed_radians_per_second = current_motors.vpan,
                    tilt_speed_radians_per_second = current_motors.vtilt,
                    "SimpleBGC control command transmit rate and latest target"
                );
                control_count = 0;
                last_control_report = Instant::now();
            }

            //limit the rate of sending the commands.
            // This is a workaround for some mysterious bug, where
            // incoming messages start to arrive in large packs every
            // 0.5 seconds or so, instead of almost immediately.
            tokio::time::sleep(Duration::from_secs_f64(0.015)).await;
        }
        #[expect(unreachable_code)]
        Ok::<_, InternalGimbalError>(())
    };

    tokio::pin!(rx_loop);
    tokio::pin!(control_loop);

    // Futures should run forever, but if one ends it is an error and we should raise it.
    let loop_result = tokio::select! {
        rx_result = &mut rx_loop => {
            rx_result
        }
        control_result = &mut control_loop => {
            control_result
        }
    };

    loop_result?;

    Ok(())
}

async fn next_motor_value(
    rx: &mut tokio::sync::watch::Receiver<MotorValueCache>,
) -> Result<MotorValueCache, InternalGimbalError> {
    rx.changed()
        .await
        .map_err(|_| InternalGimbalError::Shutdown)?;
    Ok(rx.borrow_and_update().clone())
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stopping_and_starting_target_the_same_realtime_stream() {
        let OutgoingCommand::RawMessage(stop) = realtime_stream_command(0) else {
            panic!("stream command should be raw")
        };
        let OutgoingCommand::RawMessage(start) = realtime_stream_command(1) else {
            panic!("stream command should be raw")
        };

        assert_eq!(stop.typ, simplebgc::constants::CMD_DATA_STREAM_INTERVAL);
        assert_eq!(start.typ, stop.typ);
        let mut expected_start = stop.payload.to_vec();
        // Payload layout begins with command id followed by the little-endian
        // interval. Registration identity is the command id plus config bytes,
        // so interval must be the only difference between start and stop.
        expected_start[1..3].copy_from_slice(&1u16.to_le_bytes());
        assert_eq!(start.payload.as_ref(), expected_start);
    }

    #[tokio::test]
    async fn closed_motor_command_channel_requests_shutdown() {
        let (tx, mut rx) = tokio::sync::watch::channel(MotorValueCache::default());
        drop(tx);

        assert!(matches!(
            next_motor_value(&mut rx).await,
            Err(InternalGimbalError::Shutdown)
        ));
    }
}
