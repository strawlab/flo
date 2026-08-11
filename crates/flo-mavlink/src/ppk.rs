//! Switching on the flight controller's raw-GNSS logging at startup.
//!
//! PX4 can record what its GNSS receiver actually measured — carrier-phase
//! observations, as RTCM3 MSM7 — in its own flight log, which is what a
//! post-processed kinematic (PPK) solution is computed from after landing. It
//! does not do so by default, and the parameter that turns it on,
//! `GPS_DUMP_COMM`, is read exactly once: when PX4's GPS driver starts. Writing
//! it therefore says nothing about the flight in progress, which is why this
//! module can end in a reboot.
//!
//! What is logged is the rover's own measurements, so it does not matter whether
//! the real-time corrections came from NTRIP through FLO or from a local base
//! station relayed by the ground station. What is *not* logged either way is the
//! base station's observations; PPK needs those too, archived at whatever
//! produced them.
//!
//! The whole exchange runs before [`DroneCoordinator`] requests any message
//! streams, because a reboot would discard those requests along with the global
//! origin. Nothing here is fatal: a flight controller that will not answer, will
//! not store the value, or will not reboot costs post-processing accuracy, and
//! grounding FLO over that would be the worse trade. Every such outcome is
//! logged at `error` or `warn` and startup continues.
//!
//! That extends to the link itself failing mid-exchange, which is not the same
//! as this step failing. FLO's startup ends the whole process if the MAVLink task
//! finishes inside its first 100 ms, so reporting an I/O error from here would
//! both kill FLO earlier than it would otherwise die and pin the blame on the
//! PPK step for a link that was already broken. Being the first code to *read*
//! from the link does not make it the culprit. A link failure is therefore
//! logged, the step gives up, and the main loop reports the failure as it always
//! has. See [`Waited`].

use std::time::Duration;

use eyre::Result;
use mavlink::ardupilotmega::{MavCmd, MavMessage, MavModeFlag, MavParamType, MavResult};
use tokio_mavlink::RecvError;

use crate::{AUTOPILOT_TARGET_COMPONENT, AUTOPILOT_TARGET_SYSTEM, DroneCoordinator, save};

/// The PX4 parameter deciding what GNSS traffic reaches the flight log.
const GPS_DUMP_COMM: &str = "GPS_DUMP_COMM";

/// How long the flight controller has to answer one parameter read or write.
///
/// PX4 answers from its MAVLink work queue within a stream interval or two; this
/// is long enough to also ride out a lossy telemetry link losing a packet.
const PARAM_REPLY_TIMEOUT: Duration = Duration::from_secs(5);

/// How many times an unanswered parameter read or write is re-sent before FLO
/// gives up on it.
const PARAM_ATTEMPTS: u8 = 3;

/// How long to wait between a confirmed write and asking for a reboot.
///
/// PX4 acknowledges a `PARAM_SET` immediately but writes it to storage from a
/// work queue: 300 ms after the change, and no more often than once every 2 s. A
/// reboot arriving inside that window would take the new value with it, so this
/// waits out the slowest case with room to spare.
const PARAM_SAVE_SETTLE: Duration = Duration::from_secs(3);

/// How long to keep discarding traffic after asking for a reboot.
///
/// Two jobs: it drains whatever was already queued, so a heartbeat from *before*
/// the reboot cannot be mistaken for the flight controller coming back, and it
/// covers the 400 ms PX4 waits between acknowledging the command and resetting.
const REBOOT_QUIET: Duration = Duration::from_secs(3);

/// How long to wait for the flight controller's first heartbeat after a reboot.
const REBOOT_TIMEOUT: Duration = Duration::from_secs(30);

/// How long to let the flight controller finish coming up after its first
/// heartbeat, before FLO starts asking it for things again.
const POST_REBOOT_SETTLE: Duration = Duration::from_secs(2);

/// A parameter name in the fixed-width form MAVLink's `param_id` wants.
///
/// A name of exactly 16 characters fills the field with no terminator, so this
/// pads rather than terminates.
fn param_id_bytes(name: &str) -> Result<[u8; 16]> {
    let bytes = name.as_bytes();
    if bytes.len() > 16 {
        eyre::bail!("MAVLink parameter name {name:?} does not fit in 16 bytes");
    }
    let mut out = [0u8; 16];
    out[..bytes.len()].copy_from_slice(bytes);
    Ok(out)
}

/// Whether a `param_id` field names `name`.
fn param_id_matches(field: &[u8; 16], name: &str) -> bool {
    let end = field.iter().position(|b| *b == 0).unwrap_or(field.len());
    field[..end] == *name.as_bytes()
}

/// Pack an `int32` parameter value into MAVLink's float-shaped `param_value`.
///
/// MAVLink carries every parameter value in one `float` field and PX4 does not
/// convert: for an integer parameter it copies the four bytes straight in and out
/// (`mavlink_parameters.cpp`, both `PARAM_SET` and `PARAM_VALUE`). An integer
/// parameter therefore travels as the *bit pattern* of the integer, and sending
/// `2.0` for a value of 2 would arrive as 1073741824.
fn param_value_from_i32(value: i32) -> f32 {
    f32::from_bits(value as u32)
}

/// Unpack an `int32` parameter value. The inverse of [`param_value_from_i32`].
fn param_value_as_i32(value: f32) -> i32 {
    value.to_bits() as i32
}

/// How a wait for a message from the flight controller ended.
///
/// The two ways of not getting one are worth keeping apart. A timeout means the
/// flight controller is there but did not answer, which another attempt may fix.
/// A failed link means nothing will ever arrive, so retrying only spins — and,
/// more importantly, it is not this module's business: the main loop treats a
/// dead link as fatal and reports it, and it did so before any of this existed.
/// Turning a link failure into an error here would only move the blame for it
/// into the PPK step and, because FLO's startup fails the whole process if the
/// MAVLink task ends within its first 100 ms, would make this step the apparent
/// cause of something it merely noticed first.
enum Waited<T> {
    /// The message arrived.
    Got(T),
    /// The wait ran out of time with the link still up.
    TimedOut,
    /// The link failed; nothing further will arrive on it.
    LinkLost(RecvError),
}

/// What to say when the link dies mid-exchange.
///
/// Names the link as the thing that failed, and says explicitly that the main
/// loop is where it will be dealt with, so a reader is not left thinking the PPK
/// step broke something.
fn link_lost_message(doing: &str, e: &RecvError) -> String {
    format!(
        "the MAVLink link failed while {doing} {GPS_DUMP_COMM}: {e}. Raw-GNSS logging is \
         unchanged; the link failure itself is reported by the main MAVLink loop."
    )
}

/// Whether a stored-but-not-yet-active parameter can be made to take effect now.
#[derive(Debug, PartialEq, Eq)]
enum RebootDecision {
    /// Ask the flight controller to reboot.
    Reboot,
    /// The config does not permit FLO to reboot the flight controller.
    NotPermitted,
    /// Rebooting would not be safe: the vehicle is armed, or no heartbeat has
    /// arrived and so FLO cannot tell that it is not.
    Unsafe,
}

/// PX4 refuses to reboot while armed on its own account, but FLO does not lean on
/// that: `armed` being unknown is treated the same as armed, so a link that has
/// gone quiet cannot be mistaken for a vehicle sitting safely on the ground.
fn reboot_decision(reboot_to_apply: bool, armed: Option<bool>) -> RebootDecision {
    if !reboot_to_apply {
        RebootDecision::NotPermitted
    } else if armed == Some(false) {
        RebootDecision::Reboot
    } else {
        RebootDecision::Unsafe
    }
}

impl DroneCoordinator {
    /// Bring the flight controller's `GPS_DUMP_COMM` in line with the config,
    /// rebooting it if that is what the new value needs and the config allows.
    ///
    /// A no-op when the config says nothing about PPK logging, and a single read
    /// on every startup after the value has been set once.
    pub(crate) async fn apply_ppk_logging_config(&mut self) -> Result<()> {
        let Some(cfg) = self.mavlink_cfg.ppk_logging.clone() else {
            return Ok(());
        };
        let wanted = cfg.gps_dump_comm.px4_value();
        let mode = cfg.gps_dump_comm;

        // Each step below logs its own reason for giving up -- a silent flight
        // controller and a failed link call for different words -- so a `None`
        // here needs no further comment.
        let Some(current) = self.read_gps_dump_comm().await? else {
            return Ok(());
        };

        if current == wanted {
            tracing::info!(
                "flight controller already has {GPS_DUMP_COMM}={current}, the configured \
                 {mode:?}: raw-GNSS logging needs no change"
            );
            return Ok(());
        }

        tracing::info!(
            "flight controller has {GPS_DUMP_COMM}={current}; config asks for {wanted} \
             ({mode:?}), writing it"
        );

        let Some(stored) = self.write_gps_dump_comm(wanted).await? else {
            return Ok(());
        };
        if stored != wanted {
            tracing::error!(
                "flight controller reports {GPS_DUMP_COMM}={stored} after being asked for \
                 {wanted}; it refused the write. Raw-GNSS logging is not configured."
            );
            return Ok(());
        }
        tracing::info!("flight controller stored {GPS_DUMP_COMM}={wanted}");

        match reboot_decision(cfg.reboot_to_apply, self.last_reported_armed) {
            RebootDecision::Reboot => {
                tokio::time::sleep(PARAM_SAVE_SETTLE).await;
                self.reboot_for_ppk_logging().await?;
            }
            RebootDecision::NotPermitted => tracing::warn!(
                "{GPS_DUMP_COMM}={wanted} is stored, but PX4 reads it only when its GPS driver \
                 starts and `reboot_to_apply` is off: raw-GNSS logging begins on the flight \
                 controller's next boot, not on this flight."
            ),
            RebootDecision::Unsafe => tracing::warn!(
                "{GPS_DUMP_COMM}={wanted} is stored, but FLO will not reboot a flight controller \
                 it has not seen disarmed (armed: {:?}): raw-GNSS logging begins on its next \
                 boot, not on this flight.",
                self.last_reported_armed
            ),
        }
        Ok(())
    }

    /// Ask for `GPS_DUMP_COMM` and return what the flight controller reports, or
    /// `None` if it never answers. Logs its own reason for giving up.
    async fn read_gps_dump_comm(&mut self) -> Result<Option<i32>> {
        let param_id = param_id_bytes(GPS_DUMP_COMM)?;
        for attempt in 1..=PARAM_ATTEMPTS {
            let data =
                MavMessage::PARAM_REQUEST_READ(mavlink::ardupilotmega::PARAM_REQUEST_READ_DATA {
                    // -1 asks by name. Any other value would have PX4 ignore the
                    // name and answer with whatever parameter sits at that index.
                    param_index: -1,
                    target_system: AUTOPILOT_TARGET_SYSTEM,
                    target_component: AUTOPILOT_TARGET_COMPONENT,
                    param_id,
                });
            self.send_to_autopilot(data).await?;
            match self.await_param_value(GPS_DUMP_COMM).await? {
                Waited::Got(value) => return Ok(Some(value)),
                Waited::LinkLost(e) => {
                    tracing::error!("{}", link_lost_message("reading", &e));
                    return Ok(None);
                }
                Waited::TimedOut => tracing::warn!(
                    "no {GPS_DUMP_COMM} from the flight controller within \
                     {PARAM_REPLY_TIMEOUT:?} (attempt {attempt} of {PARAM_ATTEMPTS})"
                ),
            }
        }
        tracing::error!(
            "flight controller did not answer {PARAM_ATTEMPTS} requests for {GPS_DUMP_COMM}; \
             leaving its raw-GNSS logging as it is. Whether this flight can be post-processed is \
             unknown."
        );
        Ok(None)
    }

    /// Write `GPS_DUMP_COMM` and return the value the flight controller then
    /// reports, or `None` if it never answers.
    ///
    /// PX4 broadcasts a `PARAM_VALUE` for every write it accepts, so the reply is
    /// both acknowledgement and read-back: a refused write comes back as the old
    /// value rather than as silence, and no separate re-read is needed.
    async fn write_gps_dump_comm(&mut self, value: i32) -> Result<Option<i32>> {
        let param_id = param_id_bytes(GPS_DUMP_COMM)?;
        for attempt in 1..=PARAM_ATTEMPTS {
            let data = MavMessage::PARAM_SET(mavlink::ardupilotmega::PARAM_SET_DATA {
                param_value: param_value_from_i32(value),
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                param_id,
                // PX4 rejects a write whose type disagrees with the parameter's
                // own, and `GPS_DUMP_COMM` is an int32.
                param_type: MavParamType::MAV_PARAM_TYPE_INT32,
            });
            self.send_to_autopilot(data).await?;
            match self.await_param_value(GPS_DUMP_COMM).await? {
                Waited::Got(stored) => return Ok(Some(stored)),
                Waited::LinkLost(e) => {
                    tracing::error!("{}", link_lost_message("writing", &e));
                    return Ok(None);
                }
                Waited::TimedOut => tracing::warn!(
                    "flight controller did not acknowledge {GPS_DUMP_COMM}={value} within \
                     {PARAM_REPLY_TIMEOUT:?} (attempt {attempt} of {PARAM_ATTEMPTS})"
                ),
            }
        }
        tracing::error!(
            "flight controller never acknowledged {GPS_DUMP_COMM}={value} in {PARAM_ATTEMPTS} \
             attempts. Raw-GNSS logging is probably still off."
        );
        Ok(None)
    }

    /// Reboot the flight controller so a stored `GPS_DUMP_COMM` takes effect, and
    /// wait for it to come back.
    async fn reboot_for_ppk_logging(&mut self) -> Result<()> {
        tracing::warn!(
            "rebooting the flight controller so it starts logging raw GNSS data. If FLO reaches \
             it over USB rather than a serial link, the device node will not come back and FLO \
             will need restarting."
        );
        let data = MavMessage::COMMAND_LONG(mavlink::ardupilotmega::COMMAND_LONG_DATA {
            command: MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            param1: 1.0, // 1: reboot the autopilot
            target_system: AUTOPILOT_TARGET_SYSTEM,
            target_component: AUTOPILOT_TARGET_COMPONENT,
            confirmation: 0,
            ..Default::default()
        });
        self.send_to_autopilot(data).await?;

        // Wait out the reboot, watching for the acknowledgement on the way past:
        // it is the one place a refusal — a board built without reset support,
        // say — announces itself.
        let mut ack = None;
        let quiet = self
            .recv_until(REBOOT_QUIET, |msg| -> Option<()> {
                if let MavMessage::COMMAND_ACK(data) = msg
                    && data.command == MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
                {
                    ack = Some(data.result);
                }
                None
            })
            .await;

        // A link that drops here is the documented hazard of a USB connection to
        // the flight controller: the reboot takes the device node with it. FLO
        // asked for this reboot, so the failure is expected rather than
        // mysterious, and saying so beats leaving the main loop's bare I/O error
        // to be puzzled over.
        if let Waited::LinkLost(e) = quiet {
            tracing::error!(
                "the MAVLink link failed while the flight controller was rebooting: {e}. \
                 {GPS_DUMP_COMM}={} is stored and will take effect, but FLO cannot reach the \
                 flight controller any more -- expected if it is connected over USB, since the \
                 reboot removes the device node. Restart FLO once it is back.",
                self.mavlink_cfg
                    .ppk_logging
                    .as_ref()
                    .map_or(0, |cfg| cfg.gps_dump_comm.px4_value())
            );
            return Ok(());
        }

        if let Some(result) = ack {
            if result != MavResult::MAV_RESULT_ACCEPTED {
                tracing::error!(
                    "flight controller refused to reboot ({result:?}). {GPS_DUMP_COMM} is stored, \
                     so raw-GNSS logging begins on its next boot, not on this flight."
                );
                return Ok(());
            }
            tracing::debug!("flight controller accepted the reboot request");
        }

        match self.await_heartbeat(REBOOT_TIMEOUT).await {
            Waited::Got(()) => {
                // The flight controller's clock restarted with it, so the
                // monotonicity check on `SYSTEM_TIME` must not read the step back
                // as a fault: this is the one reset FLO asked for.
                self.prev_time_boot_ms = 0;
                tokio::time::sleep(POST_REBOOT_SETTLE).await;
                tracing::info!("flight controller is back up, logging raw GNSS data");
            }
            Waited::TimedOut => tracing::error!(
                "no heartbeat from the flight controller {REBOOT_TIMEOUT:?} after it was asked to \
                 reboot. FLO will carry on, but the link may be down."
            ),
            Waited::LinkLost(e) => tracing::error!(
                "the MAVLink link failed while waiting for the flight controller to come back \
                 from the reboot FLO asked for: {e}. Expected if it is connected over USB. \
                 Restart FLO once it is back."
            ),
        }
        Ok(())
    }

    /// Wait for the flight controller's `PARAM_VALUE` for `name`, and record it.
    async fn await_param_value(&mut self, name: &str) -> Result<Waited<i32>> {
        let waited = self
            .recv_until(PARAM_REPLY_TIMEOUT, |msg| match msg {
                MavMessage::PARAM_VALUE(data) if param_id_matches(&data.param_id, name) => {
                    Some(data.clone())
                }
                _ => None,
            })
            .await;
        Ok(match waited {
            // Worth recording rather than only logging: what the flight
            // controller said about `GPS_DUMP_COMM` is the provenance of every
            // raw observation in the flight log this run is about to produce.
            Waited::Got(data) => {
                save("PARAM_VALUE", &self.floz_logger, &data)?;
                Waited::Got(param_value_as_i32(data.param_value))
            }
            Waited::TimedOut => Waited::TimedOut,
            Waited::LinkLost(e) => Waited::LinkLost(e),
        })
    }

    /// Wait for one heartbeat from the flight controller.
    async fn await_heartbeat(&mut self, timeout: Duration) -> Waited<()> {
        self.recv_until(timeout, |msg| match msg {
            MavMessage::HEARTBEAT(_) => Some(()),
            _ => None,
        })
        .await
    }

    /// Receive from the flight controller until `want` accepts a message or
    /// `timeout` runs out, keeping every heartbeat's armed flag on the way.
    ///
    /// Everything `want` declines is discarded, which is only acceptable where
    /// this is called: before any streams have been requested, so the traffic is
    /// the flight controller's own default heartbeat and status, and before the
    /// main loop, so nothing else is reading. The armed flag is the one thing
    /// kept, because whether a reboot may be asked for turns on it.
    async fn recv_until<T>(
        &mut self,
        timeout: Duration,
        mut want: impl FnMut(&MavMessage) -> Option<T>,
    ) -> Waited<T> {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            let received = match tokio::time::timeout_at(deadline, self.mavconn.rx.recv()).await {
                Err(_elapsed) => return Waited::TimedOut,
                core::result::Result::Ok(core::result::Result::Ok(received)) => received,
                core::result::Result::Ok(Err(e)) => return Waited::LinkLost(e),
            };
            let (header, msg) = received;
            if header.system_id != AUTOPILOT_TARGET_SYSTEM
                || header.component_id != AUTOPILOT_TARGET_COMPONENT
            {
                continue;
            }
            if let MavMessage::HEARTBEAT(data) = &msg {
                self.last_reported_armed = Some(
                    data.base_mode
                        .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
                );
            }
            if let Some(found) = want(&msg) {
                return Waited::Got(found);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_short_name_is_padded_and_still_matches() {
        let field = param_id_bytes(GPS_DUMP_COMM).unwrap();
        assert_eq!(&field[..13], GPS_DUMP_COMM.as_bytes());
        assert_eq!(&field[13..], &[0u8; 3]);
        assert!(param_id_matches(&field, GPS_DUMP_COMM));
        assert!(!param_id_matches(&field, "GPS_DUMP_COM"));
        assert!(!param_id_matches(&field, "GPS_UBX_PPK"));
    }

    /// MAVLink allows a 16-character name to fill the field with no terminator,
    /// so matching cannot assume there is one.
    #[test]
    fn a_full_width_name_needs_no_terminator() {
        let name = "SIXTEEN_CHARS_16";
        let field = param_id_bytes(name).unwrap();
        assert_eq!(&field[..], name.as_bytes());
        assert!(param_id_matches(&field, name));
    }

    #[test]
    fn a_name_too_long_for_the_field_is_refused() {
        assert!(param_id_bytes("SEVENTEEN_CHARS_X").is_err());
    }

    /// The whole point of the bit-cast: PX4 reinterprets the bytes, so the float
    /// that carries an int32 2 is not 2.0.
    #[test]
    fn an_int_parameter_travels_as_its_bit_pattern() {
        for value in [0, 1, 2] {
            assert_eq!(param_value_as_i32(param_value_from_i32(value)), value);
        }
        assert_eq!(param_value_from_i32(2).to_bits(), 2);
        assert_ne!(param_value_from_i32(2), 2.0);
        // What sending the plain float would have arrived as.
        assert_eq!(param_value_as_i32(2.0), 1_073_741_824);
    }

    #[test]
    fn a_disarmed_flight_controller_may_be_rebooted() {
        assert_eq!(reboot_decision(true, Some(false)), RebootDecision::Reboot);
    }

    #[test]
    fn an_armed_flight_controller_is_never_rebooted() {
        assert_eq!(reboot_decision(true, Some(true)), RebootDecision::Unsafe);
    }

    /// Not hearing from the vehicle is not evidence that it is on the ground.
    #[test]
    fn an_unknown_armed_state_is_treated_as_armed() {
        assert_eq!(reboot_decision(true, None), RebootDecision::Unsafe);
    }

    /// A timeout and a dead link are both "no answer", but only one is worth
    /// retrying -- and neither may be turned into an error that ends the process.
    #[test]
    fn a_link_failure_is_distinguishable_from_a_timeout() {
        let timed_out: Waited<i32> = Waited::TimedOut;
        let link_lost: Waited<i32> = Waited::LinkLost(RecvError::Io(std::io::Error::new(
            std::io::ErrorKind::BrokenPipe,
            "broken pipe",
        )));
        assert!(matches!(timed_out, Waited::TimedOut));
        let Waited::LinkLost(e) = link_lost else {
            panic!("expected a lost link");
        };
        // The operator has to be able to tell which of the two happened from the
        // log alone, and that the PPK step is not the thing that broke.
        let msg = link_lost_message("reading", &e);
        assert!(msg.contains("MAVLink link failed"), "{msg}");
        assert!(msg.contains("broken pipe"), "{msg}");
        assert!(msg.contains("reported by the main MAVLink loop"), "{msg}");
    }

    #[test]
    fn the_config_can_forbid_rebooting_outright() {
        for armed in [None, Some(false), Some(true)] {
            assert_eq!(
                reboot_decision(false, armed),
                RebootDecision::NotPermitted,
                "armed: {armed:?}"
            );
        }
    }
}
