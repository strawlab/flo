//! Reading every parameter the flight controller holds, once per connection.
//!
//! What the flight controller was tuned to, how its sensors were calibrated and
//! what it was logging decide what a recording *is*, and none of it is otherwise
//! recoverable from a `.floz`. So FLO asks for the lot once, at startup, and
//! hands the result to the recording writer, which keeps it and writes it into
//! every recording of the session. See [`flo_core::Px4Params`], which also
//! explains why this is every parameter rather than the shorter list of
//! parameters differing from their firmware default that `param show -c` prints.
//!
//! Unlike [`crate::px4_params`], this runs *after* the message streams have been
//! requested, and cooperatively: a `PARAM_REQUEST_LIST` answers with a parameter
//! per message, on the order of 1400 of them, which on a 57600-baud telemetry
//! link takes long enough that blocking the main loop for it would leave FLO
//! blind to RC and position for the duration. Instead [`DroneCoordinator`]'s
//! ordinary message handling feeds [`ParamSnapshot::insert`] as values arrive and
//! [`DroneCoordinator::poll_param_snapshot`] is ticked once a second alongside
//! the heartbeat.
//!
//! Nothing here is fatal, and it takes no second attempt once it has given up: a
//! flight controller that will not enumerate its parameters costs provenance,
//! and neither grounding FLO nor filling the link with retries for the rest of
//! the flight would be an improvement.
//!
//! One reading is enough for a session because FLO does not survive the flight
//! controller restarting under it: an unrequested `SYSTEM_TIME` step backwards
//! ends the MAVLink task, and with it the process. The reboot FLO asks for
//! itself happens in [`crate::px4_params`], before any of this starts.

use std::collections::{BTreeMap, BTreeSet};

use eyre::Result;
use mavlink::ardupilotmega::{MavMessage, MavParamType, PARAM_VALUE_DATA};

use flo_core::{FloatType, MyTimestamp, Px4ParamValue, Px4Params, SaveToDiskMsg, elapsed, now};

use crate::{AUTOPILOT_TARGET_COMPONENT, AUTOPILOT_TARGET_SYSTEM, DroneCoordinator};

/// How long the parameter stream may go quiet before FLO decides it has stalled
/// and starts asking for what is missing.
///
/// PX4 streams the list from its MAVLink work queue with gaps of well under a
/// second even on a busy link, so this is silence rather than slowness.
const STREAM_STALL_SECS: FloatType = 5.0;

/// How many rounds of asking for missing parameters FLO does before storing
/// whatever it has.
const CHASE_ROUNDS: u8 = 3;

/// How many missing parameters are asked for in one round.
///
/// Bounded because the request goes out on the same link FLO flies on, and a
/// flight controller that has dropped hundreds would otherwise have hundreds of
/// requests queued ahead of the next `RC_CHANNELS`.
const CHASE_BATCH: usize = 40;

/// The parameter set being assembled.
pub(crate) struct ParamSnapshot {
    /// What has arrived, by name. This is the payload.
    params: BTreeMap<String, Px4ParamValue>,
    /// Which indices have arrived. Progress is counted in indices rather than
    /// names because that is what a re-request can name, and because two
    /// `PARAM_VALUE`s for the same parameter -- an answer to a chase arriving
    /// after the original -- must not count twice.
    seen: BTreeSet<u16>,
    /// How many parameters the flight controller says it has, from the
    /// `param_count` every `PARAM_VALUE` carries. `None` until the first one
    /// arrives, which is also how "it never answered at all" is recognized.
    reported_count: Option<u16>,
    /// When the last `PARAM_VALUE` arrived, so silence can be distinguished from
    /// a stream still running.
    last_arrival: MyTimestamp,
    /// How many more rounds of chasing missing indices are left.
    chases_left: u8,
}

/// What a tick of [`DroneCoordinator::poll_param_snapshot`] concluded.
#[derive(Debug, PartialEq, Eq)]
enum Progress {
    /// Values are still arriving; leave it alone.
    Running,
    /// Every parameter arrived.
    Complete,
    /// The stream has gone quiet with parameters outstanding, and there are
    /// rounds left to chase them with.
    Stalled,
    /// Out of rounds. Store what there is.
    GivenUp,
}

impl ParamSnapshot {
    fn new() -> Self {
        Self {
            params: BTreeMap::new(),
            seen: BTreeSet::new(),
            reported_count: None,
            last_arrival: now(),
            chases_left: CHASE_ROUNDS,
        }
    }

    /// Record one `PARAM_VALUE`.
    fn insert(&mut self, data: &PARAM_VALUE_DATA) {
        self.last_arrival = now();
        self.reported_count = Some(data.param_count);

        let Some(name) = param_id_str(&data.param_id) else {
            tracing::debug!("ignoring PARAM_VALUE with a non-UTF-8 param_id");
            return;
        };
        self.params
            .insert(name, decode(data.param_type, data.param_value));

        // A `param_index` at or past the count is not an index at all: some
        // flight controllers use `u16::MAX` for "not applicable" when answering
        // a request by name. The value is still worth keeping -- it is the same
        // value either way -- but counting it as progress would let the snapshot
        // finish with a real index still missing.
        if data.param_index < data.param_count {
            self.seen.insert(data.param_index);
        }
    }

    /// Indices the flight controller has not sent, cheapest first.
    fn missing(&self) -> Vec<u16> {
        let Some(count) = self.reported_count else {
            return Vec::new();
        };
        (0..count).filter(|idx| !self.seen.contains(idx)).collect()
    }

    /// How the collection is going, as of now.
    fn progress(&self) -> Progress {
        match self.reported_count {
            Some(count) if self.seen.len() == usize::from(count) => Progress::Complete,
            _ if elapsed(self.last_arrival) < STREAM_STALL_SECS => Progress::Running,
            _ if self.chases_left > 0 => Progress::Stalled,
            _ => Progress::GivenUp,
        }
    }

    /// The record to store.
    fn finish(self) -> Px4Params {
        let missing_indices = self.missing();
        Px4Params {
            retrieved_at: chrono::Local::now(),
            reported_count: self.reported_count.unwrap_or(0),
            complete: missing_indices.is_empty() && self.reported_count.is_some(),
            params: self.params,
            missing_indices,
        }
    }
}

/// A `param_id` field as a string, or `None` if it is not UTF-8.
///
/// A name of exactly 16 characters fills the field with no terminator, so the
/// terminator cannot be assumed.
fn param_id_str(field: &[u8; 16]) -> Option<String> {
    let end = field.iter().position(|b| *b == 0).unwrap_or(field.len());
    std::str::from_utf8(&field[..end]).ok().map(str::to_owned)
}

/// Read `param_value` the way `param_type` says to.
///
/// MAVLink carries every value in one `float` field and PX4 does not convert, so
/// an integer parameter arrives as the *bit pattern* of the integer. Reading a
/// `SDLOG_PROFILE` of 17 as a float would give 2.4e-44.
fn decode(param_type: MavParamType, value: f32) -> Px4ParamValue {
    match param_type {
        MavParamType::MAV_PARAM_TYPE_REAL32 | MavParamType::MAV_PARAM_TYPE_REAL64 => {
            Px4ParamValue::Real(value)
        }
        _ => Px4ParamValue::Int(value.to_bits() as i32),
    }
}

impl DroneCoordinator {
    /// Ask the flight controller to enumerate its parameters.
    ///
    /// Returns having sent one message; the answers arrive over the following
    /// seconds through [`Self::note_param_value`].
    pub(crate) async fn start_param_snapshot(&mut self) -> Result<()> {
        tracing::info!(
            "asking the flight controller for its full parameter set; it will be stored in every \
             recording of this session"
        );
        let data =
            MavMessage::PARAM_REQUEST_LIST(mavlink::ardupilotmega::PARAM_REQUEST_LIST_DATA {
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
            });
        self.send_to_autopilot(data).await?;
        self.param_snapshot = Some(ParamSnapshot::new());
        Ok(())
    }

    /// Feed one `PARAM_VALUE` to the snapshot, if one is being collected.
    pub(crate) fn note_param_value(&mut self, data: &PARAM_VALUE_DATA) {
        if let Some(snapshot) = self.param_snapshot.as_mut() {
            snapshot.insert(data);
        }
    }

    /// Move the snapshot along: chase what has not arrived, or store it.
    ///
    /// Ticked once a second from the main loop. A no-op once the snapshot has
    /// been stored or given up on, which is what stops this from ever asking
    /// twice.
    pub(crate) async fn poll_param_snapshot(&mut self) -> Result<()> {
        let Some(snapshot) = self.param_snapshot.as_ref() else {
            return Ok(());
        };
        match snapshot.progress() {
            Progress::Running => Ok(()),
            Progress::Complete => {
                let snapshot = self.param_snapshot.take().expect("checked just above");
                tracing::info!(
                    "read all {} parameters from the flight controller",
                    snapshot.params.len()
                );
                self.store_param_snapshot(snapshot)
            }
            Progress::Stalled => {
                let missing = snapshot.missing();
                let asking: Vec<u16> = missing.iter().copied().take(CHASE_BATCH).collect();
                tracing::warn!(
                    "flight controller has sent {} of {} parameters and gone quiet for \
                     {STREAM_STALL_SECS} s; asking for {} of the {} missing",
                    snapshot.seen.len(),
                    snapshot.reported_count.unwrap_or(0),
                    asking.len(),
                    missing.len(),
                );
                let snapshot = self.param_snapshot.as_mut().expect("checked just above");
                snapshot.chases_left -= 1;
                // Without this the same silence would be re-read as a stall on
                // the very next tick, burning every round in three seconds
                // instead of giving the answers time to come back.
                snapshot.last_arrival = now();
                for index in asking {
                    self.request_param_by_index(index).await?;
                }
                Ok(())
            }
            Progress::GivenUp => {
                let snapshot = self.param_snapshot.take().expect("checked just above");
                if snapshot.reported_count.is_none() {
                    tracing::error!(
                        "flight controller never answered PARAM_REQUEST_LIST. This session's \
                         recordings will not say what it was configured to do."
                    );
                    return Ok(());
                }
                tracing::error!(
                    "flight controller sent {} of the {} parameters it says it has, and stopped. \
                     Storing the partial set; it is marked incomplete.",
                    snapshot.seen.len(),
                    snapshot.reported_count.unwrap_or(0),
                );
                self.store_param_snapshot(snapshot)
            }
        }
    }

    /// Ask for one parameter by its index.
    async fn request_param_by_index(&mut self, index: u16) -> Result<()> {
        // `param_index` is signed on the wire, and -1 is reserved for "look the
        // name up instead". PX4 has some 1400 parameters, so this is a guard
        // against an implausible flight controller rather than a real case --
        // but asking for index 40000 as -25536 would silently fetch a name FLO
        // did not send.
        let Ok(param_index) = i16::try_from(index) else {
            tracing::warn!("cannot ask for parameter index {index}: it does not fit in an i16");
            return Ok(());
        };
        let data =
            MavMessage::PARAM_REQUEST_READ(mavlink::ardupilotmega::PARAM_REQUEST_READ_DATA {
                param_index,
                target_system: AUTOPILOT_TARGET_SYSTEM,
                target_component: AUTOPILOT_TARGET_COMPONENT,
                // Ignored when `param_index` is not -1.
                param_id: [0u8; 16],
            });
        self.send_to_autopilot(data).await
    }

    /// Hand a finished snapshot to the recording writer.
    fn store_param_snapshot(&mut self, snapshot: ParamSnapshot) -> Result<()> {
        let params = snapshot.finish();
        self.floz_logger
            .send(SaveToDiskMsg::Px4Params(Box::new(params)))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn param_value(
        name: &str,
        index: u16,
        count: u16,
        ty: MavParamType,
        v: f32,
    ) -> PARAM_VALUE_DATA {
        let mut param_id = [0u8; 16];
        param_id[..name.len()].copy_from_slice(name.as_bytes());
        PARAM_VALUE_DATA {
            param_value: v,
            param_count: count,
            param_index: index,
            param_id,
            param_type: ty,
        }
    }

    fn an_int(name: &str, index: u16, count: u16, v: i32) -> PARAM_VALUE_DATA {
        param_value(
            name,
            index,
            count,
            MavParamType::MAV_PARAM_TYPE_INT32,
            f32::from_bits(v as u32),
        )
    }

    /// The bit-cast is the whole difficulty of PX4's parameter protocol: a
    /// `SDLOG_PROFILE` of 17 is not 17.0, and reading it as a float gives a
    /// denormal rather than an obviously wrong number.
    #[test]
    fn an_int_parameter_is_read_as_its_bit_pattern() {
        assert_eq!(
            decode(MavParamType::MAV_PARAM_TYPE_INT32, f32::from_bits(17)),
            Px4ParamValue::Int(17)
        );
        assert_eq!(
            decode(MavParamType::MAV_PARAM_TYPE_REAL32, 0.5),
            Px4ParamValue::Real(0.5)
        );
    }

    #[test]
    fn a_full_width_name_needs_no_terminator() {
        let mut field = [0u8; 16];
        field.copy_from_slice(b"SIXTEEN_CHARS_16");
        assert_eq!(param_id_str(&field).as_deref(), Some("SIXTEEN_CHARS_16"));
    }

    #[test]
    fn a_complete_set_is_recognized_and_stored_by_name() {
        let mut snapshot = ParamSnapshot::new();
        snapshot.insert(&an_int("A", 0, 2, 1));
        assert_eq!(snapshot.progress(), Progress::Running);
        snapshot.insert(&an_int("B", 1, 2, 2));
        assert_eq!(snapshot.progress(), Progress::Complete);

        let stored = snapshot.finish();
        assert!(stored.complete);
        assert_eq!(stored.reported_count, 2);
        assert!(stored.missing_indices.is_empty());
        assert_eq!(stored.params["A"], Px4ParamValue::Int(1));
        assert_eq!(stored.params["B"], Px4ParamValue::Int(2));
    }

    /// A gap is what a chase is for, and it has to be reported by index: the
    /// name of a parameter that never arrived was never learned.
    #[test]
    fn a_gap_is_reported_by_index() {
        let mut snapshot = ParamSnapshot::new();
        snapshot.insert(&an_int("A", 0, 4, 1));
        snapshot.insert(&an_int("D", 3, 4, 4));
        assert_eq!(snapshot.missing(), vec![1, 2]);

        let stored = snapshot.finish();
        assert!(!stored.complete);
        assert_eq!(stored.missing_indices, vec![1, 2]);
        assert_eq!(stored.params.len(), 2);
    }

    /// A duplicate -- a chased parameter whose original turns up late -- must
    /// not count as progress, or the snapshot finishes with a real gap in it.
    #[test]
    fn a_repeated_index_does_not_count_twice() {
        let mut snapshot = ParamSnapshot::new();
        snapshot.insert(&an_int("A", 0, 2, 1));
        snapshot.insert(&an_int("A", 0, 2, 1));
        assert_eq!(snapshot.progress(), Progress::Running);
        assert_eq!(snapshot.missing(), vec![1]);
    }

    /// Some flight controllers answer a by-name read with `u16::MAX` for the
    /// index. The value is still the value, but it says nothing about progress.
    #[test]
    fn an_out_of_range_index_is_kept_as_a_value_but_not_as_progress() {
        let mut snapshot = ParamSnapshot::new();
        snapshot.insert(&an_int("LATE", u16::MAX, 2, 7));
        assert_eq!(snapshot.params["LATE"], Px4ParamValue::Int(7));
        assert_eq!(snapshot.missing(), vec![0, 1]);
        assert_eq!(snapshot.progress(), Progress::Running);
    }

    /// A flight controller that says nothing at all is a different failure from
    /// one that stops halfway, and only the second leaves anything worth
    /// storing.
    #[test]
    fn silence_is_distinguishable_from_a_truncated_stream() {
        let silent = ParamSnapshot::new();
        assert_eq!(silent.reported_count, None);
        assert!(silent.missing().is_empty());
        // Even so, nothing it could produce claims to be complete.
        assert!(!silent.finish().complete);

        let mut truncated = ParamSnapshot::new();
        truncated.insert(&an_int("A", 0, 2, 1));
        assert_eq!(truncated.reported_count, Some(2));
        assert!(!truncated.finish().complete);
    }

    /// Chasing is bounded: once the rounds are spent, silence means store what
    /// there is rather than ask forever on a link FLO also flies on.
    #[test]
    fn chasing_runs_out() {
        let mut snapshot = ParamSnapshot::new();
        snapshot.insert(&an_int("A", 0, 2, 1));
        snapshot.last_arrival = now() - std::time::Duration::from_secs_f64(STREAM_STALL_SECS + 1.0);
        assert_eq!(snapshot.progress(), Progress::Stalled);
        snapshot.chases_left = 0;
        assert_eq!(snapshot.progress(), Progress::GivenUp);
    }

    /// Completeness outranks the clock: a set that arrived in full is finished
    /// even if the last value came in long enough ago to look like a stall.
    #[test]
    fn a_complete_set_is_never_mistaken_for_a_stall() {
        let mut snapshot = ParamSnapshot::new();
        snapshot.insert(&an_int("A", 0, 1, 1));
        snapshot.last_arrival = now() - std::time::Duration::from_secs_f64(STREAM_STALL_SECS + 1.0);
        snapshot.chases_left = 0;
        assert_eq!(snapshot.progress(), Progress::Complete);
    }
}
