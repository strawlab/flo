//! Turn the gimbal's startup query responses into a durable provenance record.
//!
//! The controller can describe itself completely -- board serial, firmware
//! version, active profile, every stored encoder calibration constant -- and
//! FLO already receives most of it while fetching the two encoder offsets it
//! needs to fly. This module keeps the rest instead of discarding it.
//!
//! The reason to bother: a gimbal controller's stored configuration is
//! *geometry*. Its encoder offset participates in the pose chain exactly as a
//! machined lever does, and unlike a lever it can change with no physical trace
//! whatsoever -- someone re-runs a calibration and every subsequent recording
//! is referenced differently, with nothing in the archive to say so.
//!
//! Because the configuration is machine-readable at every recording, that
//! change does not need to be remembered by a person. Comparing the fingerprint
//! of consecutive recordings detects it.

use flo_core::{AxisTriple, GimbalBoardIdentity, GimbalEncoderConfig, GimbalProvenance};
use sha2::{Digest, Sha256};
use simplebgc::{BoardInfo, BoardInfo3, Params3Data, ParamsExtData, Payload, RollPitchYaw};

/// Units per turn of the encoder offset fields: 2^14.
const ENCODER_UNITS_PER_TURN: f64 = (1 << 14) as f64;

pub(crate) const KEY_BOARD_INFO: &str = "cmd_board_info";
pub(crate) const KEY_BOARD_INFO_3: &str = "cmd_board_info_3";
pub(crate) const KEY_PARAMS_EXT: &str = "cmd_read_params_ext";
pub(crate) const KEY_PARAMS_3: &str = "cmd_read_params_3";

/// Responses gathered during startup. Every field is optional: a board that
/// declines a query is a fact worth recording, not an error.
#[derive(Default)]
pub(crate) struct StartupQueries {
    pub board_info: Option<BoardInfo>,
    pub board_info_3: Option<BoardInfo3>,
    pub params_ext: Option<ParamsExtData>,
    pub params_3: Option<Params3Data>,
}

impl StartupQueries {
    /// Whether every query has been answered, so collection can stop early.
    pub fn is_complete(&self) -> bool {
        self.board_info.is_some()
            && self.board_info_3.is_some()
            && self.params_ext.is_some()
            && self.params_3.is_some()
    }

    /// Which queries the board answered. Used to describe a failed startup: a
    /// board that answered nothing has a different problem than one that
    /// answered three of four.
    pub fn answered_keys(&self) -> Vec<&'static str> {
        let mut answered = Vec::new();
        if self.board_info.is_some() {
            answered.push(KEY_BOARD_INFO);
        }
        if self.board_info_3.is_some() {
            answered.push(KEY_BOARD_INFO_3);
        }
        if self.params_ext.is_some() {
            answered.push(KEY_PARAMS_EXT);
        }
        if self.params_3.is_some() {
            answered.push(KEY_PARAMS_3);
        }
        answered
    }
}

fn triple<T: Copy>(rpy: &RollPitchYaw<T>) -> AxisTriple<T> {
    AxisTriple {
        roll: rpy.roll,
        pitch: rpy.pitch,
        yaw: rpy.yaw,
    }
}

/// Decode `2687` as `"2.68b7"`, per the SimpleBGC encoding.
fn firmware_version_string(raw: u16) -> String {
    let major = raw / 1000;
    let minor = (raw % 1000) / 10;
    let beta = raw % 10;
    if beta == 0 {
        format!("{major}.{minor:02}")
    } else {
        format!("{major}.{minor:02}b{beta}")
    }
}

/// Decode `30` as `"3.0"`.
fn board_version_string(raw: u8) -> String {
    format!("{}.{}", raw / 10, raw % 10)
}

fn board_identity(
    info: Option<&BoardInfo>,
    info3: Option<&BoardInfo3>,
) -> Option<GimbalBoardIdentity> {
    // Either response alone is worth keeping: BOARD_INFO carries the firmware
    // version, BOARD_INFO_3 carries the hardware serial.
    if info.is_none() && info3.is_none() {
        return None;
    }
    let features: Vec<String> = info
        .map(|i| i.board_features.iter().map(|f| format!("{f:?}")).collect())
        .unwrap_or_default();
    Some(GimbalBoardIdentity {
        mcu_id: info3.map(|i| hex::encode(i.mcu_id)).unwrap_or_default(),
        device_id: info3.map(|i| hex::encode(i.device_id)).unwrap_or_default(),
        board_version_raw: info.map(|i| i.board_version).unwrap_or(0),
        board_version: info
            .map(|i| board_version_string(i.board_version))
            .unwrap_or_default(),
        firmware_version_raw: info.map(|i| i.firmware_version).unwrap_or(0),
        firmware_version: info
            .map(|i| firmware_version_string(i.firmware_version))
            .unwrap_or_default(),
        firmware_extra_id: info.map(|i| i.frw_extra_id).unwrap_or(0),
        board_features_raw: info.map(|i| i.board_features.bits()).unwrap_or(0),
        board_features: features,
        state_flags_raw: info.map(|i| i.state.bits()).unwrap_or(0),
        eeprom_size: info3.map(|i| i.eeprom_size).unwrap_or(0),
        profile_slots: info3.map(|i| i.profile_set_slots).unwrap_or(0),
        profile_current: info3.map(|i| i.profile_set_cur).unwrap_or(0),
    })
}

fn encoder_config(params: &ParamsExtData) -> GimbalEncoderConfig {
    let offset_raw = triple(&params.encoder_offset);
    GimbalEncoderConfig {
        offset_raw,
        offset_turns: AxisTriple {
            roll: offset_raw.roll as f64 / ENCODER_UNITS_PER_TURN,
            pitch: offset_raw.pitch as f64 / ENCODER_UNITS_PER_TURN,
            yaw: offset_raw.yaw as f64 / ENCODER_UNITS_PER_TURN,
        },
        field_offset_raw: triple(&params.encoder_field_offset),
        gear_ratio_milli: triple(&params.encoder_gear_ratio),
        encoder_type: triple(&params.encoder_type),
        encoder_cfg: triple(&params.encoder_cfg),
        manual_set_time_10ms: triple(&params.encoder_manual_set_time),
    }
}

/// Digest the stored configuration, returning the hash and what it covers.
///
/// Deliberately covers stored *configuration* only, not board identity:
/// swapping controllers is a part change while rewriting a calibration on the
/// same board is an adjustment, and downstream those want to stay
/// distinguishable. Board state flags are excluded because they vary between
/// boots (init progress, debug mode) and would make every recording look
/// different for no reason.
///
/// `profile_current` *is* included: switching profile slot replaces the live
/// parameter set even though the stored bytes of profile 0 are unchanged.
fn fingerprint(
    raw_payloads: &std::collections::BTreeMap<String, String>,
    profile_current: Option<u8>,
) -> (Option<String>, Vec<String>) {
    let mut hasher = Sha256::new();
    let mut covers = Vec::new();
    for key in [KEY_PARAMS_EXT, KEY_PARAMS_3] {
        if let Some(hexed) = raw_payloads.get(key) {
            hasher.update(key.as_bytes());
            hasher.update(hexed.as_bytes());
            covers.push(key.to_string());
        }
    }
    if let Some(profile) = profile_current {
        hasher.update(b"profile_current");
        hasher.update([profile]);
        covers.push("profile_current".to_string());
    }
    if covers.is_empty() {
        (None, covers)
    } else {
        (Some(hex::encode(hasher.finalize())), covers)
    }
}

/// Assemble the record written into each recording.
pub(crate) fn build(queries: &StartupQueries) -> GimbalProvenance {
    let mut raw_payloads = std::collections::BTreeMap::new();
    let mut unanswered = Vec::new();

    // Keep the verbatim bytes of every response. Selecting fields by name today
    // decides what a future analysis is allowed to ask; the raw payload does
    // not. `to_bytes` round-trips the parsed struct through the same encoding
    // the board used.
    macro_rules! record {
        ($key:expr, $value:expr) => {
            match &$value {
                Some(v) => {
                    raw_payloads.insert($key.to_string(), hex::encode(Payload::to_bytes(v)));
                }
                None => unanswered.push($key.to_string()),
            }
        };
    }
    record!(KEY_BOARD_INFO, queries.board_info);
    record!(KEY_BOARD_INFO_3, queries.board_info_3);
    record!(KEY_PARAMS_EXT, queries.params_ext);
    record!(KEY_PARAMS_3, queries.params_3);

    let (config_fingerprint_sha256, covers) = fingerprint(
        &raw_payloads,
        queries.board_info_3.as_ref().map(|i| i.profile_set_cur),
    );

    GimbalProvenance {
        queried_at: chrono::Local::now(),
        board: board_identity(queries.board_info.as_ref(), queries.board_info_3.as_ref()),
        encoders: queries.params_ext.as_ref().map(encoder_config),
        config_fingerprint_sha256,
        fingerprint_covers: covers,
        raw_payloads,
        unanswered,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[test]
    fn firmware_versions_decode() {
        assert_eq!(firmware_version_string(2687), "2.68b7");
        assert_eq!(firmware_version_string(2680), "2.68");
        assert_eq!(firmware_version_string(2552), "2.55b2");
    }

    #[test]
    fn board_versions_decode() {
        assert_eq!(board_version_string(30), "3.0");
        assert_eq!(board_version_string(31), "3.1");
    }

    fn payloads(params_ext: &str, params_3: &str, board_info: &str) -> BTreeMap<String, String> {
        BTreeMap::from([
            (KEY_PARAMS_EXT.to_string(), params_ext.to_string()),
            (KEY_PARAMS_3.to_string(), params_3.to_string()),
            (KEY_BOARD_INFO.to_string(), board_info.to_string()),
        ])
    }

    #[test]
    fn an_unchanged_configuration_keeps_its_fingerprint() {
        let a = fingerprint(&payloads("aabb", "ccdd", "0011"), Some(0));
        let b = fingerprint(&payloads("aabb", "ccdd", "0011"), Some(0));
        assert_eq!(a.0, b.0);
        assert!(a.0.is_some());
    }

    #[test]
    fn rewriting_an_encoder_calibration_moves_the_fingerprint() {
        // The whole point: this is the change that leaves no physical trace.
        let before = fingerprint(&payloads("aabb", "ccdd", "0011"), Some(0));
        let after = fingerprint(&payloads("aaff", "ccdd", "0011"), Some(0));
        assert_ne!(before.0, after.0);
    }

    #[test]
    fn switching_profile_slot_moves_the_fingerprint() {
        // Stored profile-0 bytes are identical, but a different parameter set
        // is live, so the recordings are not comparable.
        let before = fingerprint(&payloads("aabb", "ccdd", "0011"), Some(0));
        let after = fingerprint(&payloads("aabb", "ccdd", "0011"), Some(2));
        assert_ne!(before.0, after.0);
    }

    #[test]
    fn board_identity_is_not_part_of_the_configuration_fingerprint() {
        // Swapping controllers is a part change, not an adjustment. Keeping
        // identity out of this digest is what lets a downstream effectivity
        // model tell those two apart.
        let before = fingerprint(&payloads("aabb", "ccdd", "0011"), Some(0));
        let after = fingerprint(&payloads("aabb", "ccdd", "ffff"), Some(0));
        assert_eq!(before.0, after.0);
    }

    #[test]
    fn fingerprint_reports_what_it_covered() {
        let (digest, covers) = fingerprint(&payloads("aabb", "ccdd", "0011"), Some(1));
        assert!(digest.is_some());
        assert_eq!(
            covers,
            vec![KEY_PARAMS_EXT, KEY_PARAMS_3, "profile_current"]
        );
    }

    #[test]
    fn a_silent_board_has_no_fingerprint_rather_than_a_misleading_one() {
        let (digest, covers) = fingerprint(&BTreeMap::new(), None);
        assert!(digest.is_none());
        assert!(covers.is_empty());
    }

    #[test]
    fn an_unanswered_board_still_yields_a_record() {
        // A board that answers nothing must not produce an error, and must not
        // silently produce an empty-looking record either: the queries it
        // declined are listed.
        let provenance = build(&StartupQueries::default());
        assert!(provenance.board.is_none());
        assert!(provenance.encoders.is_none());
        assert!(provenance.config_fingerprint_sha256.is_none());
        assert_eq!(provenance.unanswered.len(), 4);
        assert!(provenance.raw_payloads.is_empty());
    }
}
