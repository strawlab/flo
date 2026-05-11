use eyre::{Ok, Result, WrapErr};
use flo_core::{
    Angle, Broadway, CommandSource, DeviceMode, DroneChannelData, FloCommand, FloEvent, FloatType,
    LocalFloState, ModeChangeReason, MyTimestamp, SaveToDiskMsg, StampedJson,
    drone_structs::{self, BatteryState, ChannelCondition, DroneEvent, FlightMode, GnssRtkMode},
    elapsed, now,
};
use mavlink::{
    MavHeader,
    ardupilotmega::{MavComponent, MavMessage, MavModeFlag},
};

mod ntrip;

pub struct MavlinkPort {
    port: Box<dyn serialport::SerialPort>,
    cfg: flo_core::drone_structs::MavlinkConfig,
}

impl MavlinkPort {
    pub fn open(mavlink_cfg: &flo_core::drone_structs::MavlinkConfig) -> Result<Self> {
        let mavlink_cfg = mavlink_cfg.clone();

        let settings = &mavlink_cfg.port_path;

        let settings_toks: Vec<&str> = settings.split(':').collect();
        if settings_toks.len() < 2 {
            eyre::bail!(
                "Incomplete port settings. Expected format: serial://<device_path>:<baud_rate>"
            );
        }

        let settings_toks = &settings_toks[1..];

        let baud_rate = match settings_toks[1].parse() {
            core::result::Result::Ok(baud_rate) => baud_rate,
            Err(e) => {
                eyre::bail!("Error parsing baud rate: {e}");
            }
        };

        let path = settings_toks[0];
        tracing::info!("mavlink at {}", settings);
        let mut port = serialport::new(path, baud_rate)
            .open()
            .with_context(|| format!("Opening mavlink connection {}", settings))?;
        port.set_timeout(std::time::Duration::from_secs(60 * 60 * 24 * 365 * 100))?;
        let cfg = mavlink_cfg.clone();
        Ok(Self { port, cfg })
    }
}

fn convert_gnss_rtk_mode(fix_type: mavlink::ardupilotmega::GpsFixType) -> GnssRtkMode {
    use mavlink::ardupilotmega::GpsFixType::*;
    match fix_type {
        GPS_FIX_TYPE_NO_GPS => GnssRtkMode::NoGps,
        GPS_FIX_TYPE_NO_FIX => GnssRtkMode::NoFix,
        GPS_FIX_TYPE_2D_FIX => GnssRtkMode::TwoDFix,
        GPS_FIX_TYPE_3D_FIX => GnssRtkMode::ThreeDFix,
        GPS_FIX_TYPE_DGPS => GnssRtkMode::DGps,
        GPS_FIX_TYPE_RTK_FLOAT => GnssRtkMode::RtkFloat,
        GPS_FIX_TYPE_RTK_FIXED => GnssRtkMode::RtkFixed,
        GPS_FIX_TYPE_STATIC => GnssRtkMode::Static,
        GPS_FIX_TYPE_PPP => GnssRtkMode::Ppp,
    }
}

struct DroneCoordinator {
    mavlink_cfg: flo_core::drone_structs::MavlinkConfig,
    mavconn: tokio_mavlink::MavlinkConnection<MavMessage>,
    my_header: mavlink::MavHeader,
    rc_cfg: Option<flo_core::RcConfig>,
    broadway: flo_core::Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    rc_program_state: flo_core::drone_structs::RcProgramState,
    armed_cd: flo_core::utils::ChangeDetector<bool>,
    flight_mode_cd: flo_core::utils::ChangeDetector<u32>,
    last_message_timestamp: Option<MyTimestamp>,
    sys_start: MyTimestamp,
    prev_time_boot_ms: u32,
    local_flo_state: LocalFloState,
}

impl DroneCoordinator {
    async fn new(
        mavlink_cfg: &flo_core::drone_structs::MavlinkConfig,
        mavconn: tokio_mavlink::MavlinkConnection<MavMessage>,
        my_header: mavlink::MavHeader,
        rc_cfg: Option<flo_core::RcConfig>,
        broadway: flo_core::Broadway,
        floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
        local_flo_state: LocalFloState,
    ) -> Result<Self> {
        // fill initial struct
        let mut self_ = Self {
            mavlink_cfg: mavlink_cfg.clone(),
            mavconn,
            my_header,
            rc_cfg,
            broadway,
            floz_logger,
            rc_program_state: Default::default(),
            // Set a non-existing initial value so that true first value is detected as change.
            armed_cd: flo_core::utils::ChangeDetector::new_with_initial_state(&false),
            flight_mode_cd: flo_core::utils::ChangeDetector::new_with_initial_state(&0),
            last_message_timestamp: Default::default(),
            sys_start: now(),
            prev_time_boot_ms: 0,
            local_flo_state,
        };

        // Below is the old Self::init() method, now moved into the constructor.

        if let Some(cfg) = &self_.rc_cfg {
            if let Some(knob_cfg) = &cfg.pan_knob {
                self_.rc_program_state.pan_ng.params = knob_cfg.noise_gate.clone();
            }
            if let Some(knob_cfg) = &cfg.tilt_knob {
                self_.rc_program_state.tilt_ng.params = knob_cfg.noise_gate.clone();
            }
        }

        // Below is the old Self::request_streams() method, now moved into the constructor.

        use mavlink::MessageData as _;

        // Request the GPS_GLOBAL_ORIGIN message to be streamed at 1 second interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::GPS_GLOBAL_ORIGIN_DATA::ID as f32, // Message ID to be streamed
                param2: 1_000_000.0, // Interval in microseconds
                target_system: 1,
                target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.mavconn.tx.send((self_.my_header, data)).await?;

        // Send GPS global origin to the drone.
        if let Some(set_gps_global_origin) = &self_.mavlink_cfg.set_gps_global_origin {
            let (lat, lon, alt) = (
                set_gps_global_origin[0],
                set_gps_global_origin[1],
                set_gps_global_origin[2],
            );
            #[expect(
                deprecated,
                reason = "MAV_CMD_SET_GLOBAL_ORIGIN not yet in ardupilotmega dialect"
            )]
            let data = mavlink::ardupilotmega::MavMessage::SET_GPS_GLOBAL_ORIGIN(
                mavlink::ardupilotmega::SET_GPS_GLOBAL_ORIGIN_DATA {
                    latitude: (lat * 1e7) as i32,
                    longitude: (lon * 1e7) as i32,
                    altitude: (alt * 1e3) as i32,
                    target_system: 1,
                },
            );

            self_.mavconn.tx.send((self_.my_header, data)).await?;
            tracing::info!("Sent SET_GPS_GLOBAL_ORIGIN: lat: {lat}, lon: {lon}, alt: {alt}",);
        }

        // Request the RC_CHANNELS message to be streamed at 4 msec interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::RC_CHANNELS_DATA::ID as f32, // Message ID to be streamed
                param2: 4000.0, // Interval in microseconds
                target_system: 1,
                target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.mavconn.tx.send((self_.my_header, data)).await?;

        // Request the SYSTEM_TIME message to be streamed at 1 second interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::SYSTEM_TIME_DATA::ID as f32, // Message ID to be streamed
                param2: 1_000_000.0, // Interval in microseconds
                target_system: 1,
                target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.mavconn.tx.send((self_.my_header, data)).await?;

        // Request the LOCAL_POSITION_NED message to be streamed at 10 millisecond interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::LOCAL_POSITION_NED_DATA::ID as f32, // Message ID to be streamed
                param2: 10_000.0, // Interval in microseconds
                target_system: 1,
                target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.mavconn.tx.send((self_.my_header, data)).await?;

        // Request the ATTITUDE_QUATERNION message to be streamed at 10 millisecond interval.
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                param1: mavlink::ardupilotmega::ATTITUDE_QUATERNION_DATA::ID as f32, // Message ID to be streamed
                param2: 10_000.0, // Interval in microseconds
                target_system: 1,
                target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
                confirmation: 0,
                ..Default::default()
            },
        );
        self_.mavconn.tx.send((self_.my_header, data)).await?;
        Ok(self_)
    }

    async fn send_heartbeat(&mut self) -> Result<()> {
        let data =
            mavlink::ardupilotmega::MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
                system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
                mavlink_version: 0x3,
            });

        self.mavconn.tx.send((self.my_header, data)).await?;
        Ok(())
    }

    /// Issues a MAVLink command to switch the drone's flight mode.
    async fn switch_flight_mode(&mut self, fm: FlightMode) -> Result<()> {
        tracing::trace!("switching drone flight mode to {fm:?}");
        if fm == FlightMode::Other {
            return Err(eyre::eyre!("can't switch to Other flight mode"));
        }
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_DO_SET_MODE,
                param1: 1.0, // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, from ardupilot docs https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html
                param2: fm.mode_numbers().unwrap().1,
                param3: fm.mode_numbers().unwrap().2,
                target_system: 1,
                target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
                confirmation: 0,
                ..Default::default()
            },
        );
        self.mavconn.tx.send((self.my_header, data)).await?;
        Ok(())
    }

    async fn handle_message_from_drone(
        &mut self,
        header: MavHeader,
        msg: MavMessage,
    ) -> Result<()> {
        if !(header.system_id == 1 && header.component_id == 1) {
            tracing::trace!(
                "ignoring message not from flight controller: {header:?}, msg: {msg:?}"
            );
            // not from flight controller - ignore this message
            return Ok(());
        }

        tracing::trace!("header: {header:?}, msg: {msg:?}");

        if self.last_message_timestamp.is_none() {
            tracing::debug!("mavlink established");
        }
        self.last_message_timestamp = Some(now());
        let logger = &mut self.floz_logger;

        match msg {
            MavMessage::HEARTBEAT(msg) => {
                let armed = msg
                    .base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                if self.armed_cd.update(&armed) {
                    self.broadway.drone_events.send(if armed {
                        DroneEvent::Armed
                    } else {
                        DroneEvent::Disarmed
                    })?;
                }
                let fm = msg.custom_mode;
                if self.flight_mode_cd.update(&fm) {
                    self.broadway
                        .drone_events
                        .send(DroneEvent::FlightModeChanged(
                            self.flight_mode_cd
                                .latest_change
                                .unwrap()
                                .0
                                .map(|fm| fm.into()),
                            fm.into(),
                        ))?;
                }
            }
            MavMessage::BATTERY_STATUS(bs) => {
                self.broadway
                    .drone_events
                    .send(DroneEvent::BatteryState(BatteryState {
                        batt_voltage: bs.voltages[0] as FloatType
                            / self.mavlink_cfg.batt_s as FloatType
                            / 1000.0,
                        batt_percent: bs.battery_remaining as FloatType,
                        timestamp: now(),
                    }))?;
            }
            MavMessage::ALTITUDE(_)
            | MavMessage::ATTITUDE(_)
            | MavMessage::ATTITUDE_TARGET(_)
            | MavMessage::CURRENT_EVENT_SEQUENCE(_)
            | MavMessage::ESTIMATOR_STATUS(_)
            | MavMessage::EXTENDED_SYS_STATE(_)
            | MavMessage::LINK_NODE_STATUS(_)
            | MavMessage::TIMESYNC(_)
            | MavMessage::RADIO_STATUS(_)
            | MavMessage::SYS_STATUS(_)
            | MavMessage::VIBRATION(_)
            | MavMessage::VFR_HUD(_) => {
                if header.system_id != 1 {
                    // ignore this message

                    // println!("header: {header:?}");
                    // println!("  received: {msg:?}");
                } else {
                    // ignore this message
                }
            }
            MavMessage::PARAM_VALUE(data) => {
                let valid = if let Some(idx) = data.param_id.iter().position(|x| x == &0x00) {
                    &data.param_id[..idx]
                } else {
                    &data.param_id[..]
                };
                match std::str::from_utf8(valid) {
                    core::result::Result::Ok(param_id_str) => {
                        tracing::debug!("param_id: {param_id_str}");
                    }
                    Err(_) => {
                        tracing::debug!("ignoring invalid param_id: {:?}", data.param_id);
                    }
                }
            }
            MavMessage::RC_CHANNELS(data) => {
                self.handle_rc(data).await?;
            }
            MavMessage::SYSTEM_TIME(v) => {
                save("SYSTEM_TIME", logger, &v)?;
                if v.time_boot_ms < self.prev_time_boot_ms {
                    // This will roll over after 49.7 days, but we ignore that.
                    eyre::bail!(
                        "Received SYSTEM_TIME with time_boot_ms {} < previous {}. Did the flight controller reset?",
                        v.time_boot_ms,
                        self.prev_time_boot_ms
                    );
                } else {
                    self.prev_time_boot_ms = v.time_boot_ms;
                }
            }
            MavMessage::UTM_GLOBAL_POSITION(v) => {
                save("UTM_GLOBAL_POSITION", logger, &v)?;
            }
            MavMessage::GPS_RAW_INT(v) => {
                self.local_flo_state.write().unwrap().gnss_rtk_mode =
                    convert_gnss_rtk_mode(v.fix_type);
                save("GPS_RAW_INT", logger, &v)?;
            }
            MavMessage::LOCAL_POSITION_NED(v) => {
                if (v.x * v.x + v.y * v.y + v.z * v.z) >= (10000.0 * 10000.0) {
                    tracing::error!("local position {v:?} is more than 10km from global origin");
                }
                save("LOCAL_POSITION_NED", logger, &v)?;
            }
            MavMessage::ATTITUDE_QUATERNION(v) => {
                save("ATTITUDE_QUATERNION", logger, &v)?;
            }
            MavMessage::GPS_GLOBAL_ORIGIN(v) => {
                tracing::info!("received GPS_GLOBAL_ORIGIN: {v:?}");
                save("GPS_GLOBAL_ORIGIN", logger, &v)?;
            }
            msg => {
                tracing::trace!("unknown mavlink message: {msg:?}");
            }
        }

        Ok(())
    }

    async fn handle_rc(&mut self, data: mavlink::ardupilotmega::RC_CHANNELS_DATA) -> Result<()> {
        let vals = [
            data.chan1_raw,
            data.chan2_raw,
            data.chan3_raw,
            data.chan4_raw,
            data.chan5_raw,
            data.chan6_raw,
            data.chan7_raw,
            data.chan8_raw,
            data.chan9_raw,
            data.chan10_raw,
            data.chan11_raw,
            data.chan12_raw,
            data.chan13_raw,
            data.chan14_raw,
            data.chan15_raw,
            data.chan16_raw,
            data.chan17_raw,
            data.chan18_raw,
        ];

        let rc = if let Some(rc_cfg) = &self.rc_cfg {
            DroneChannelData {
                timestamp: now(),
                values: std::array::from_fn(|i| rc_cfg.us_mapping.convert(vals[i] as FloatType)),
            }
        } else {
            tracing::warn!("Received RC message, but ignoring because no rc_cfg set.");
            return Ok(());
        };

        self.broadway
            .drone_realtime
            .send(drone_structs::DroneRealtimeEvent::RcChannels(rc.clone()))?;

        self.handle_rc_flo_control(&rc)?;

        Ok(())
    }

    fn handle_rc_flo_control(&mut self, rc: &DroneChannelData) -> Result<()> {
        if self.rc_cfg.is_none() {
            return Ok(());
        };
        let cfg = self.rc_cfg.as_ref().unwrap();

        let track_start = ChannelCondition::test(&cfg.track_start, rc);
        let track_stop = ChannelCondition::test(&cfg.track_stop, rc);
        let set_home = ChannelCondition::test(&cfg.set_home, rc);

        // start tracking
        if self
            .rc_program_state
            .track_start_cd
            .update_and_has_changed_to(&track_start, &true)
        {
            self.broadway.flo_events.send(FloEvent::Command(
                FloCommand::SwitchMode(DeviceMode::AcquiringLock, ModeChangeReason::Operator),
                CommandSource::DroneRC,
            ))?;
        }

        // set home
        if self
            .rc_program_state
            .set_home_cd
            .update_and_has_changed_to(&set_home, &true)
        {
            self.broadway.flo_events.send(FloEvent::Command(
                FloCommand::SetHomePositionFromCurrent,
                CommandSource::DroneRC,
            ))?;
        }

        // stop tracking
        if self
            .rc_program_state
            .track_stop_cd
            .update_and_has_changed_to(&track_stop, &true)
        {
            self.broadway.flo_events.send(FloEvent::Command(
                FloCommand::SwitchToOpenLoop,
                CommandSource::DroneRC,
            ))?;
        }
        {
            use flo_core::utils::NoiseGateStatus as S;
            let mut new_home: (Option<Angle>, Option<Angle>) = (None, None);
            if let Some(knob_cfg) = &cfg.pan_knob {
                let (s, val) = self.rc_program_state.pan_ng.update(rc.get(knob_cfg.ch_no));
                if s == S::Varying {
                    new_home.0 = Some(Angle::from_degrees(val * knob_cfg.max_angle))
                }
            };
            if let Some(knob_cfg) = &cfg.tilt_knob {
                let (s, val) = self.rc_program_state.tilt_ng.update(rc.get(knob_cfg.ch_no));
                if s == S::Varying {
                    new_home.1 = Some(Angle::from_degrees(val * knob_cfg.max_angle))
                }
            }

            if new_home != (None, None) {
                self.broadway.flo_events.send(FloEvent::Command(
                    FloCommand::SetHomePosition((new_home.0, new_home.1, None)),
                    CommandSource::DroneRC,
                ))?;
            }
        }

        Ok(())
    }

    async fn on_flight_setpoint(&mut self, sp: drone_structs::TrajectorySetpoint) -> Result<()> {
        self.send_trajectory_setpoint(sp).await
    }

    async fn on_flight_mode_request(&mut self, fm: FlightMode) -> Result<()> {
        if self.flight_mode_cd.old_value != Some(fm as u32) {
            self.switch_flight_mode(fm).await?;
        }
        Ok(())
    }

    async fn send_trajectory_setpoint(
        &mut self,
        sp: drone_structs::TrajectorySetpoint,
    ) -> Result<()> {
        use mavlink::ardupilotmega::MavMessage::SET_POSITION_TARGET_LOCAL_NED;
        use mavlink::ardupilotmega::*;
        fn zero_if_none(ov: Option<FloatType>) -> f32 {
            ov.map(|v| v as f32).unwrap_or(0.0)
        }
        fn one_if_none(ov: Option<FloatType>) -> u16 {
            if ov.is_none() { 1 } else { 0 }
        }

        #[expect(clippy::identity_op)]
        let ignoremask = (1 << 0) * one_if_none(sp.pos[0])
            + (1 << 1) * one_if_none(sp.pos[1])
            + (1 << 2) * one_if_none(sp.pos[2])
            + (1 << 3) * one_if_none(sp.vel[0])
            + (1 << 4) * one_if_none(sp.vel[1])
            + (1 << 5) * one_if_none(sp.vel[2])
            + (1 << 10) * one_if_none(sp.yaw)
            + (1 << 11) * one_if_none(sp.vyaw);

        let msg = SET_POSITION_TARGET_LOCAL_NED(SET_POSITION_TARGET_LOCAL_NED_DATA {
            x: zero_if_none(sp.pos[0]),
            y: zero_if_none(sp.pos[1]),
            z: zero_if_none(sp.pos[2]),
            vx: zero_if_none(sp.vel[0]),
            vy: zero_if_none(sp.vel[1]),
            vz: zero_if_none(sp.vel[2]),
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: zero_if_none(sp.yaw),
            yaw_rate: zero_if_none(sp.vyaw),
            type_mask: PositionTargetTypemask::from_bits(ignoremask).unwrap(),
            target_system: 1,
            target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
            coordinate_frame: MavFrame::MAV_FRAME_BODY_FRD,
            time_boot_ms: (elapsed(self.sys_start) * 1000.0) as u32,
        });
        self.mavconn.tx.send((self.my_header, msg)).await?;
        Ok(())
    }
}

fn save<T: serde::Serialize>(
    name: &str,
    logger: &mut tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    v: &T,
) -> Result<()> {
    logger.send(SaveToDiskMsg::MavlinkData(StampedJson::new(
        v,
        name.into(),
    )?))?;
    Ok(())
}

async fn main_loop(
    handle: &tokio::runtime::Handle,
    mavlink_cfg: &flo_core::drone_structs::MavlinkConfig,
    mavconn: tokio_mavlink::MavlinkConnection<MavMessage>,
    rc_cfg: Option<flo_core::RcConfig>,
    broadway: flo_core::Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    local_flo_state: LocalFloState,
) -> eyre::Result<()> {
    let mut flight_setpoint_rx = broadway.flight_setpoint.subscribe();
    let mut flight_mode_request_rx = broadway.flight_mode_request.subscribe();

    // This is hacky, but we need to clone the mavconn sender for NTRIP.
    let mavconn_tx = mavconn.tx.hacky_clone_tx();

    let header = mavlink::MavHeader {
        system_id: mavlink_cfg.system_id,
        component_id: mavlink_cfg.component_id,
        sequence: 0,
    };

    let mut coordinator = DroneCoordinator::new(
        mavlink_cfg,
        mavconn,
        header,
        rc_cfg,
        broadway,
        floz_logger,
        local_flo_state,
    )
    .await?;

    // For GNSS RTK, connect to NTRIP server and send RTCM data to the autopilot.
    let mut ntrip_task: std::pin::Pin<Box<dyn std::future::Future<Output = _> + Send>> = {
        if let Some(ntrip_url) = &mavlink_cfg.ntrip_url {
            let ntrip_url = ntrip_url.clone();
            let ntrip_join_handle =
                handle.spawn(async move { ntrip::ntrip_loop(ntrip_url, mavconn_tx, header).await });
            Box::pin(ntrip_join_handle)
        } else {
            Box::pin(std::future::pending())
        }
    };

    let mut send_heartbeat_interval = tokio::time::interval(tokio::time::Duration::from_secs(1));
    send_heartbeat_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            _ = send_heartbeat_interval.tick() => {
                coordinator.send_heartbeat().await?;
            },
            r = coordinator.mavconn.rx.recv() => {
                let (header, msg) = r?;
                coordinator.handle_message_from_drone(header, msg).await?;
            }
            r = flight_setpoint_rx.recv() => {
                coordinator.on_flight_setpoint(r.unwrap()).await?;
            }
            r = flight_mode_request_rx.recv() => {
                coordinator.on_flight_mode_request(r.unwrap()).await?;
            }
            ntrip_result = &mut ntrip_task => {
                let _: ntrip::NeverOk = ntrip_result??;
                unreachable!("NTRIP task completed.");
            }
        }
    }
}

/// spawns tokio task to handle mavlink.
///
/// This returns immediately with the join handle to the spawned task.
pub fn spawn_mavlink(
    handle: &tokio::runtime::Handle,
    broadway: Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    rc_cfg: Option<&flo_core::RcConfig>,
    mavlink_port: MavlinkPort,
    local_flo_state: LocalFloState,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<()>>> {
    let rc_cfg: Option<flo_core::RcConfig> = rc_cfg.cloned();

    let MavlinkPort {
        port: read_port,
        cfg: mavlink_cfg,
    } = mavlink_port;
    let write_port = read_port.try_clone()?;

    let mavconn = tokio_mavlink::spawn_mavlink_threads(
        read_port,
        write_port,
        10,
        10,
        mavlink::MavlinkVersion::V1,
    )?;

    let handle2 = handle.clone();
    let main_jh = handle.spawn(async move {
        main_loop(
            &handle2,
            &mavlink_cfg,
            mavconn,
            rc_cfg,
            broadway,
            floz_logger,
            local_flo_state,
        )
        .await
    });

    Ok(main_jh)
}
