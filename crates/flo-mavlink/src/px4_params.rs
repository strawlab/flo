//! Bringing the flight controller's parameters in line with the config at
//! startup.
//!
//! Two things FLO configures live in PX4's own parameter storage rather than in
//! FLO's config, because they change what the *flight controller* records:
//!
//! - `GPS_DUMP_COMM`, which decides whether the flight log holds what the GNSS
//!   receiver actually measured. Without it a log has only the real-time fix,
//!   and no post-processed kinematic (PPK) solution can be computed after
//!   landing. See [`flo_core::drone_structs::PpkLoggingConfig`].
//! - `SDLOG_PROFILE`, which decides which sets of topics the flight log holds
//!   and at what rate. See [`flo_core::drone_structs::SdlogProfileConfig`].
//!
//! Both are read exactly once, when the module that uses them starts: the GPS
//! driver for the first, `logger` for the second. Writing either therefore says
//! nothing about the flight in progress, which is why this module can end in a
//! reboot — one reboot, after every parameter has been reconciled, however many
//! of them changed.
//!
//! The whole exchange runs before [`DroneCoordinator`] requests any message
//! streams, because a reboot would discard those requests along with the global
//! origin. Nothing here is fatal: a flight controller that will not answer, will
//! not store a value, or will not reboot costs recording detail after the fact,
//! and grounding FLO over that would be the worse trade. Every such outcome is
//! logged at `error` or `warn` and startup continues.
//!
//! That extends to the link itself failing mid-exchange, which is not the same
//! as this step failing. FLO's startup ends the whole process if the MAVLink task
//! finishes inside its first 100 ms, so reporting an I/O error from here would
//! both kill FLO earlier than it would otherwise die and pin the blame on this
//! step for a link that was already broken. Being the first code to *read* from
//! the link does not make it the culprit. A link failure is therefore logged, the
//! step gives up, and the main loop reports the failure as it always has. See
//! [`Waited`].

use std::time::Duration;

use eyre::Result;
use mavlink::ardupilotmega::{MavCmd, MavMessage, MavModeFlag, MavParamType, MavResult};
use tokio_mavlink::RecvError;

use crate::{AUTOPILOT_TARGET_COMPONENT, AUTOPILOT_TARGET_SYSTEM, DroneCoordinator, save};

/// The PX4 parameter deciding what GNSS traffic reaches the flight log.
const GPS_DUMP_COMM: &str = "GPS_DUMP_COMM";

/// The PX4 parameter deciding which topics reach the flight log, and how often.
const SDLOG_PROFILE: &str = "SDLOG_PROFILE";

/// How long the flight controller has to answer one parameter read or write.
///
/// PX4 answers from its MAVLink work queue within a stream interval or two; this
/// is long enough to also ride out a lossy telemetry link losing a packet.
const PARAM_REPLY_TIMEOUT: Duration = Duration::from_secs(5);

/// How many times an unanswered parameter read or write is re-sent before FLO
/// gives up on it.
const PARAM_ATTEMPTS: u8 = 3;

/// How long to wait between the last confirmed write and asking for a reboot.
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

/// One PX4 parameter the config asks the flight controller to hold.
struct WantedParam {
    /// The parameter's name, as PX4 knows it.
    name: &'static str,
    /// The value to write, already in PX4's own units.
    value: i32,
    /// What the config asked for, in the config's own words, so a log line can
    /// be matched back to the YAML that caused it.
    asked_for: String,
    /// What holding this value buys, so a log line is intelligible without the
    /// config to hand.
    purpose: &'static str,
    /// Whether the config permits a reboot to make a changed value take effect.
    reboot_to_apply: bool,
}

/// What became of one parameter.
#[derive(Debug, PartialEq, Eq)]
enum Reconciled {
    /// The flight controller already held the wanted value; nothing was written
    /// and nothing needs applying.
    Unchanged,
    /// The value was written and read back, and takes effect on the next boot.
    Stored,
    /// FLO could not set it. The reason is already in the log.
    Failed,
}

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
/// into this step and, because FLO's startup fails the whole process if the
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
/// loop is where it will be dealt with, so a reader is not left thinking this
/// step broke something.
fn link_lost_message(doing: &str, name: &str, e: &RecvError) -> String {
    format!(
        "the MAVLink link failed while {doing} {name}: {e}. The flight controller's parameters \
         are unchanged; the link failure itself is reported by the main MAVLink loop."
    )
}

/// Whether stored-but-not-yet-active parameters can be made to take effect now.
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
///
/// `reboot_to_apply` is unanimous rather than a majority, because a config turns
/// it off to say something about the *link* — that the flight controller is on
/// USB, and a reboot takes the device node with it. That is true of the whole
/// link, not of one parameter, so a single objection settles it. The cost of
/// reading it the other way round would be an unreachable flight controller; the
/// cost of reading it this way is a parameter that waits for the next boot, and
/// says so in the log.
fn reboot_decision(
    reboot_to_apply: impl IntoIterator<Item = bool>,
    armed: Option<bool>,
) -> RebootDecision {
    if !reboot_to_apply.into_iter().all(|permitted| permitted) {
        RebootDecision::NotPermitted
    } else if armed == Some(false) {
        RebootDecision::Reboot
    } else {
        RebootDecision::Unsafe
    }
}

impl DroneCoordinator {
    /// Bring the flight controller's parameters in line with the config,
    /// rebooting it once if that is what the new values need and the config
    /// allows.
    ///
    /// A no-op when the config asks for no parameters, and a read apiece on every
    /// startup after the values have been set once.
    pub(crate) async fn apply_px4_param_config(&mut self) -> Result<()> {
        let wanted = self.wanted_px4_params();
        if wanted.is_empty() {
            return Ok(());
        }

        let mut pending = Vec::new();
        for param in &wanted {
            if self.reconcile_param(param).await? == Reconciled::Stored {
                pending.push(param);
            }
        }
        if pending.is_empty() {
            return Ok(());
        }

        let names = pending
            .iter()
            .map(|param| format!("{}={}", param.name, param.value))
            .collect::<Vec<_>>()
            .join(", ");
        let decision = reboot_decision(
            pending.iter().map(|param| param.reboot_to_apply),
            self.last_reported_armed,
        );
        match decision {
            RebootDecision::Reboot => {
                tokio::time::sleep(PARAM_SAVE_SETTLE).await;
                self.reboot_to_apply_params(&names).await?;
            }
            RebootDecision::NotPermitted => tracing::warn!(
                "{names} is stored, but PX4 reads these only at boot and `reboot_to_apply` is off: \
                 they take effect on the flight controller's next boot, not on this flight."
            ),
            RebootDecision::Unsafe => tracing::warn!(
                "{names} is stored, but FLO will not reboot a flight controller it has not seen \
                 disarmed (armed: {:?}): they take effect on its next boot, not on this flight.",
                self.last_reported_armed
            ),
        }
        Ok(())
    }

    /// The parameters the config asks the flight controller to hold, in the order
    /// they are reconciled.
    fn wanted_px4_params(&self) -> Vec<WantedParam> {
        let mut wanted = Vec::new();
        if let Some(cfg) = &self.mavlink_cfg.ppk_logging {
            wanted.push(WantedParam {
                name: GPS_DUMP_COMM,
                value: cfg.gps_dump_comm.px4_value(),
                asked_for: format!("{:?}", cfg.gps_dump_comm),
                purpose: "raw-GNSS logging for PPK",
                reboot_to_apply: cfg.reboot_to_apply,
            });
        }
        if let Some(cfg) = &self.mavlink_cfg.sdlog_profile {
            wanted.push(WantedParam {
                name: SDLOG_PROFILE,
                value: cfg.px4_value(),
                asked_for: format!("{:?}", cfg.topics),
                purpose: "the flight log's topic profile",
                reboot_to_apply: cfg.reboot_to_apply,
            });
        }
        wanted
    }

    /// Read one parameter, and write it if it is not already what the config
    /// asks for.
    async fn reconcile_param(&mut self, param: &WantedParam) -> Result<Reconciled> {
        let WantedParam {
            name,
            value,
            asked_for,
            purpose,
            ..
        } = param;

        // Each step below logs its own reason for giving up -- a silent flight
        // controller and a failed link call for different words -- so a `None`
        // here needs no further comment.
        let Some(current) = self.read_param(name).await? else {
            return Ok(Reconciled::Failed);
        };

        if current == *value {
            tracing::info!(
                "flight controller already has {name}={current}, the configured {asked_for}: \
                 {purpose} needs no change"
            );
            return Ok(Reconciled::Unchanged);
        }

        tracing::info!(
            "flight controller has {name}={current}; config asks for {value} ({asked_for}), \
             writing it"
        );

        let Some(stored) = self.write_param(name, *value).await? else {
            return Ok(Reconciled::Failed);
        };
        if stored != *value {
            tracing::error!(
                "flight controller reports {name}={stored} after being asked for {value}; it \
                 refused the write. {purpose} is not configured."
            );
            return Ok(Reconciled::Failed);
        }
        tracing::info!("flight controller stored {name}={value}");
        Ok(Reconciled::Stored)
    }

    /// Ask for a parameter and return what the flight controller reports, or
    /// `None` if it never answers. Logs its own reason for giving up.
    async fn read_param(&mut self, name: &str) -> Result<Option<i32>> {
        let param_id = param_id_bytes(name)?;
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
            match self.await_param_value(name).await? {
                Waited::Got(value) => return Ok(Some(value)),
                Waited::LinkLost(e) => {
                    tracing::error!("{}", link_lost_message("reading", name, &e));
                    return Ok(None);
                }
                Waited::TimedOut => tracing::warn!(
                    "no {name} from the flight controller within {PARAM_REPLY_TIMEOUT:?} \
                     (attempt {attempt} of {PARAM_ATTEMPTS})"
                ),
            }
        }
        tracing::error!(
            "flight controller did not answer {PARAM_ATTEMPTS} requests for {name}; leaving it as \
             it is."
        );
        Ok(None)
    }

    /// Write a parameter and return the value the flight controller then
    /// reports, or `None` if it never answers.
    ///
    /// PX4 broadcasts a `PARAM_VALUE` for every write it accepts, so the reply is
    /// both acknowledgement and read-back: a refused write comes back as the old
    /// value rather than as silence, and no separate re-read is needed.
    async fn write_param(&mut self, name: &str, value: i32) -> Result<Option<i32>> {
        let param_id = param_id_bytes(name)?;
        for attempt in 1..=PARAM_ATTEMPTS {
            let data = MavMessage::PARAM_SET(mavlink::ardupilotmega::PARAM_SET_DATA {
                param_value: param_value_from_i32(value),
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                param_id,
                // PX4 rejects a write whose type disagrees with the parameter's
                // own, and both parameters here are int32.
                param_type: MavParamType::MAV_PARAM_TYPE_INT32,
            });
            self.send_to_autopilot(data).await?;
            match self.await_param_value(name).await? {
                Waited::Got(stored) => return Ok(Some(stored)),
                Waited::LinkLost(e) => {
                    tracing::error!("{}", link_lost_message("writing", name, &e));
                    return Ok(None);
                }
                Waited::TimedOut => tracing::warn!(
                    "flight controller did not acknowledge {name}={value} within \
                     {PARAM_REPLY_TIMEOUT:?} (attempt {attempt} of {PARAM_ATTEMPTS})"
                ),
            }
        }
        tracing::error!(
            "flight controller never acknowledged {name}={value} in {PARAM_ATTEMPTS} attempts."
        );
        Ok(None)
    }

    /// Reboot the flight controller so stored parameters take effect, and wait
    /// for it to come back.
    async fn reboot_to_apply_params(&mut self, names: &str) -> Result<()> {
        tracing::warn!(
            "rebooting the flight controller so it picks up {names}. If FLO reaches it over USB \
             rather than a serial link, the device node will not come back and FLO will need \
             restarting."
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
                "the MAVLink link failed while the flight controller was rebooting: {e}. {names} \
                 is stored and will take effect, but FLO cannot reach the flight controller any \
                 more -- expected if it is connected over USB, since the reboot removes the \
                 device node. Restart FLO once it is back."
            );
            return Ok(());
        }

        if let Some(result) = ack {
            if result != MavResult::MAV_RESULT_ACCEPTED {
                tracing::error!(
                    "flight controller refused to reboot ({result:?}). {names} is stored, so it \
                     takes effect on its next boot, not on this flight."
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
                tracing::info!("flight controller is back up with {names}");
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
            // controller said about these parameters is the provenance of the
            // flight log this run is about to produce.
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

    /// Both names have to survive the round trip, and a reply to one must not be
    /// mistaken for a reply to the other now that they are asked for in sequence.
    #[test]
    fn the_two_parameters_do_not_answer_for_each_other() {
        let gps = param_id_bytes(GPS_DUMP_COMM).unwrap();
        let sdlog = param_id_bytes(SDLOG_PROFILE).unwrap();
        assert!(param_id_matches(&sdlog, SDLOG_PROFILE));
        assert!(!param_id_matches(&sdlog, GPS_DUMP_COMM));
        assert!(!param_id_matches(&gps, SDLOG_PROFILE));
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
        for value in [0, 1, 2, 17, 2047] {
            assert_eq!(param_value_as_i32(param_value_from_i32(value)), value);
        }
        assert_eq!(param_value_from_i32(2).to_bits(), 2);
        assert_ne!(param_value_from_i32(2), 2.0);
        // What sending the plain float would have arrived as.
        assert_eq!(param_value_as_i32(2.0), 1_073_741_824);
    }

    #[test]
    fn a_disarmed_flight_controller_may_be_rebooted() {
        assert_eq!(reboot_decision([true], Some(false)), RebootDecision::Reboot);
    }

    #[test]
    fn an_armed_flight_controller_is_never_rebooted() {
        assert_eq!(reboot_decision([true], Some(true)), RebootDecision::Unsafe);
    }

    /// Not hearing from the vehicle is not evidence that it is on the ground.
    #[test]
    fn an_unknown_armed_state_is_treated_as_armed() {
        assert_eq!(reboot_decision([true], None), RebootDecision::Unsafe);
    }

    #[test]
    fn the_config_can_forbid_rebooting_outright() {
        for armed in [None, Some(false), Some(true)] {
            assert_eq!(
                reboot_decision([false], armed),
                RebootDecision::NotPermitted,
                "armed: {armed:?}"
            );
        }
    }

    /// `reboot_to_apply: false` says the link cannot survive a reboot, which is
    /// true of every parameter on it or none. One objection therefore settles it,
    /// however many other parameters would have permitted the reboot.
    #[test]
    fn one_parameter_forbidding_a_reboot_forbids_it_for_all() {
        assert_eq!(
            reboot_decision([true, false], Some(false)),
            RebootDecision::NotPermitted
        );
        assert_eq!(
            reboot_decision([false, true], Some(false)),
            RebootDecision::NotPermitted
        );
        assert_eq!(
            reboot_decision([true, true], Some(false)),
            RebootDecision::Reboot
        );
    }

    /// Nothing pending means nothing to object to, but the caller never asks in
    /// that case -- it returns before deciding. Pinned so a future caller that
    /// does ask gets an answer that cannot lead to a gratuitous reboot request
    /// unless the vehicle is also known to be disarmed.
    #[test]
    fn an_empty_set_of_parameters_leans_on_the_armed_check_alone() {
        assert_eq!(reboot_decision([], Some(false)), RebootDecision::Reboot);
        assert_eq!(reboot_decision([], None), RebootDecision::Unsafe);
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
        // log alone, and that this step is not the thing that broke.
        let msg = link_lost_message("reading", GPS_DUMP_COMM, &e);
        assert!(msg.contains("MAVLink link failed"), "{msg}");
        assert!(msg.contains("broken pipe"), "{msg}");
        assert!(msg.contains("reported by the main MAVLink loop"), "{msg}");
    }
}
