use eyre::{Ok, Result, WrapErr};
use flo_core::{
    drone_structs::{self, BatteryState, ChannelCondition, DroneEvent},
    elapsed_by, now, Angle, Broadway, CommandSource, DeviceMode, DroneChannelData, FloCommand,
    FloEvent, FloatType, ModeChangeReason, MyTimestamp, SaveToDiskMsg, StampedJson,
};
use mavlink::{
    ardupilotmega::{MavComponent, MavMessage, MavModeFlag},
    MavHeader,
};

pub(crate) struct DroneCoordinator {
    pub mavlink_cfg: flo_core::drone_structs::MavlinkConfig,
    pub mavconn_rx: tokio_mavlink::MavlinkReceiver<MavMessage>,
    pub mavconn_tx: tokio_mavlink::MavlinkSender<MavMessage>,
    pub my_header: mavlink::MavHeader,
    pub rc_cfg: Option<flo_core::RcConfig>,
    pub broadway: flo_core::Broadway,
    pub floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    pub rc_program_state: flo_core::drone_structs::RcProgramState,
    pub armed_cd: flo_core::utils::ChangeDetector<bool>,
    pub flight_mode_cd: flo_core::utils::ChangeDetector<u32>,
    pub last_non_chase_flight_mode: Option<u32>,
    pub last_message_timestamp: Option<MyTimestamp>,
}

impl DroneCoordinator {
    pub async fn do_heartbeat(&mut self) -> Result<()> {
        let data = heartbeat_message();
        self.mavconn_tx.send((self.my_header, data)).await?;
        Ok(())
    }

    pub async fn request_streams(&mut self) -> Result<()> {
        use mavlink::MessageData as _;
        let data = mavlink::ardupilotmega::MavMessage::COMMAND_LONG(
            mavlink::ardupilotmega::COMMAND_LONG_DATA {
                target_system: 1,
                target_component: MavComponent::MAV_COMP_ID_AUTOPILOT1 as u8,
                command: mavlink::ardupilotmega::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                confirmation: 0,
                param1: mavlink::ardupilotmega::RC_CHANNELS_DATA::ID as f32,
                param2: 4000.0, //microseconds. 100 seems to be about as fast as px4 will accept, 10 and1 stop the stream altogether
                param7: 0.0,
                ..Default::default()
            },
        );
        self.mavconn_tx.send((self.my_header, data)).await?;
        Ok(())
    }

    pub async fn handle_message_from_drone(
        &mut self,
        header: MavHeader,
        msg: MavMessage,
    ) -> Result<()> {
        tracing::trace!("header: {header:?}, msg: {msg:?}");
        if self.last_message_timestamp.is_none() {
            tracing::trace!("mavlink established");
            self.broadway
                .drone_events
                .send(DroneEvent::CommEstablished)?;
        }
        self.last_message_timestamp = Some(now());

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
                            / self.mavlink_cfg.batt_s
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
            | MavMessage::PING(_)
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
            MavMessage::UTM_GLOBAL_POSITION(v) => {
                self.floz_logger
                    .send(SaveToDiskMsg::MavlinkData(StampedJson::new(
                        &v,
                        "UTM_GLOBAL_POSITION".into(),
                    )?))?;
            }
            MavMessage::GPS_RAW_INT(v) => {
                self.floz_logger
                    .send(SaveToDiskMsg::MavlinkData(StampedJson::new(
                        &v,
                        "GPS_RAW_INT".into(),
                    )?))?;
            }
            msg => {
                tracing::debug!("unknown mavlink message: {msg:?}");
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

        let rc = DroneChannelData {
            timestamp: now(),
            values: std::array::from_fn(|i| (vals[i] as FloatType - 1500.0) / 500.0),
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

    pub fn on_comm_lost(&mut self) -> Result<()> {
        tracing::trace!("mavlink silence");
        self.init();
        self.broadway.drone_events.send(DroneEvent::CommLost)?;
        Ok(())
    }

    pub fn init(&mut self) {
        self.rc_program_state = Default::default();
        self.armed_cd = flo_core::utils::ChangeDetector::new_with_initial_state(&false);
        // Set a non-existing initial value so that true first value is detected as change.
        self.flight_mode_cd = flo_core::utils::ChangeDetector::new_with_initial_state(&0);
        self.last_non_chase_flight_mode = Default::default();
        self.last_message_timestamp = Default::default();

        if let Some(cfg) = &self.rc_cfg {
            if let Some(knob_cfg) = &cfg.pan_knob {
                self.rc_program_state.pan_ng.params = knob_cfg.noise_gate.clone();
            }
            if let Some(knob_cfg) = &cfg.tilt_knob {
                self.rc_program_state.tilt_ng.params = knob_cfg.noise_gate.clone();
            }
        }
    }
}

/// Create a heartbeat message using 'ardupilotmega' dialect
fn heartbeat_message() -> mavlink::ardupilotmega::MavMessage {
    mavlink::ardupilotmega::MavMessage::HEARTBEAT(mavlink::ardupilotmega::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
        system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

pub struct MavLinkTasks {
    pub main_jh: tokio::task::JoinHandle<eyre::Result<()>>,
}

async fn main_loop(
    mavlink_cfg: flo_core::drone_structs::MavlinkConfig,
    mavconn_rx: tokio_mavlink::MavlinkReceiver<MavMessage>,
    mavconn_tx: tokio_mavlink::MavlinkSender<MavMessage>,
    rc_cfg: Option<flo_core::RcConfig>,
    broadway: flo_core::Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
) -> eyre::Result<()> {
    let mut coordinator = DroneCoordinator {
        mavlink_cfg: mavlink_cfg.clone(),
        mavconn_rx,
        mavconn_tx,
        my_header: mavlink::MavHeader {
            system_id: mavlink_cfg.system_id,
            component_id: mavlink_cfg.component_id,
            sequence: 0,
        },
        rc_cfg,
        broadway,
        floz_logger,
        rc_program_state: Default::default(),
        armed_cd: Default::default(),
        flight_mode_cd: Default::default(),
        last_non_chase_flight_mode: Default::default(),
        last_message_timestamp: Default::default(),
    };
    coordinator.init();

    let mut heartbeat_interval = tokio::time::interval(tokio::time::Duration::from_secs(1));
    let mut request_stream_interval = tokio::time::interval(tokio::time::Duration::from_secs(3));
    heartbeat_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    request_stream_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    loop {
        let now = now();
        let loss_timeout = if let Some(lsmt) = &coordinator.last_message_timestamp {
            (coordinator.mavlink_cfg.loss_timeout - elapsed_by(*lsmt, now)).clamp(0.0, 1000.0)
        } else {
            31_536_000.0 //1 year, i.e. forever
        };
        let loss_timeout = tokio::time::sleep(tokio::time::Duration::from_secs_f64(loss_timeout));

        tokio::select! {
            _ = heartbeat_interval.tick() => {
                coordinator.do_heartbeat().await?;
            },
            _ = request_stream_interval.tick() => {
                coordinator.request_streams().await?;
            }
            r = coordinator.mavconn_rx.recv() => {
                let (header, msg) = r?;
                coordinator.handle_message_from_drone(header, msg).await?;
            }
            _ = loss_timeout => {
                coordinator.on_comm_lost()?;
            }
        }
    }
}

pub fn spawn_mavlink(
    mavlink_cfg: &flo_core::drone_structs::MavlinkConfig,
    broadway: Broadway,
    floz_logger: tokio::sync::mpsc::UnboundedSender<SaveToDiskMsg>,
    rc_cfg: Option<&flo_core::RcConfig>,
) -> eyre::Result<MavLinkTasks> {
    let rc_cfg: Option<flo_core::RcConfig> = rc_cfg.cloned();
    let mavlink_cfg = mavlink_cfg.clone();

    let settings = &mavlink_cfg.port_path;

    let settings_toks: Vec<&str> = settings.split(':').collect();
    if settings_toks.len() < 2 {
        eyre::bail!("Incomplete port settings");
    }

    let settings_toks = &settings_toks[1..];

    let baud_rate = match settings_toks[1].parse() {
        core::result::Result::Ok(baud_rate) => baud_rate,
        Err(e) => {
            eyre::bail!("Error parsing baud rate: {e}");
        }
    };

    let port_name = settings_toks[0];
    let mut read_port = serialport::new(port_name, baud_rate)
        .open()
        .with_context(|| format!("Opening mavlink connection {}", mavlink_cfg.port_path))?;
    read_port.set_timeout(std::time::Duration::from_secs(60 * 60 * 24 * 365 * 100))?;
    let write_port = read_port.try_clone()?;

    let mavconn = tokio_mavlink::open(read_port, write_port, 10, 10, mavlink::MavlinkVersion::V1)?;
    let (mavconn_tx, mavconn_rx) = mavconn.split();

    let main_jh = tokio::spawn(async move {
        main_loop(
            mavlink_cfg,
            mavconn_rx,
            mavconn_tx,
            rc_cfg,
            broadway,
            floz_logger,
        )
        .await
    });

    Ok(MavLinkTasks { main_jh })
}
