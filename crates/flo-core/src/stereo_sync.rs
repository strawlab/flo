// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Pairing the two tracking cameras' detections into stereo pairs.
//!
//! The tracking cameras are hardware triggered from one pulse, so a subject is
//! seen by both cameras at the same instant and each camera's hardware counter
//! advances on every trigger. What the two cameras do not share is where those
//! counters started: the difference between them is an arbitrary integer, of
//! either sign, fixed for as long as both cameras run.
//!
//! [`StereoSynchronizer`] learns that one integer at startup from the
//! acquisition timestamps, and pairs on framenumbers from then on. Learning
//! from timestamps and pairing on framenumbers plays to the strengths of each:
//! timestamps say which observations belong to the same trigger when nothing
//! is known about the counters yet, while framenumbers then identify a trigger
//! exactly, with no tolerance to tune and nothing for delivery jitter to
//! disturb.
//!
//! Learning waits for [`StereoSyncParams::confirm_frames`] consecutive
//! observations to agree, because the offset is learned once and never revised:
//! a single coincidence must not be able to settle it. If it were nevertheless
//! wrong, or if a camera restarted its counter mid-session, framenumber matches
//! would start disagreeing with the timestamps — [`StereoSyncStats::rejected`]
//! counts exactly that, and no pair is emitted from a match the timestamps do
//! not vouch for.
//!
//! Pairing is deliberately lossy, following `OrderedLossyFrameBundler` in
//! braid's `flydra2`: a pair is emitted the moment its second half arrives, and
//! an observation older than one already paired is dropped rather than held.
//! FLO is a controller, so the freshest data wins; the dropped observations are
//! still written to the `.floz`, and `floz-retrack` can re-pair them offline
//! with no deadline to meet.

use std::collections::VecDeque;

use chrono::TimeDelta;

use crate::{FloatType, MomentCentroid};

/// Which of the two tracking cameras an observation came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamRole {
    /// The camera FLO steers by. It also defines "now": pairing never waits on
    /// the secondary, and never rewinds behind this camera.
    Primary,
    /// The second camera of the stereo pair, used only for distance.
    Secondary,
}

/// Difference between two hardware framenumbers, as a signed count of triggers.
///
/// Framenumbers are `u32` on the wire and wrap, so this is a wrapping
/// subtraction rather than a widening one: a counter that reads below the other
/// gives a small negative difference, not a difference of nearly 2^32.
pub fn framenumber_offset(a: u32, b: u32) -> i64 {
    i64::from(a.wrapping_sub(b) as i32)
}

/// What identifies an observation for pairing: which trigger its camera says
/// it is, and when the host says it was acquired.
#[derive(Debug, Clone, Copy)]
struct Stamp {
    framenumber: u32,
    timestamp: chrono::DateTime<chrono::Utc>,
}

/// Two observations of one trigger, ready for stereopsis.
#[derive(Debug, Clone, PartialEq)]
pub struct StereoPair {
    pub primary: MomentCentroid,
    pub secondary: MomentCentroid,
}

impl StereoPair {
    /// Difference between the two cameras' acquisition timestamps. Both
    /// cameras being triggered together, this is host stamping jitter.
    pub fn skew(&self) -> TimeDelta {
        self.secondary.timestamp - self.primary.timestamp
    }
}

/// Tuning for [`StereoSynchronizer`].
#[derive(Debug, Clone, PartialEq)]
pub struct StereoSyncParams {
    /// How far apart two acquisition timestamps may be while still counting as
    /// the same trigger.
    ///
    /// Only used to learn and to check the framenumber offset, never to pair.
    /// It has to stay below half a frame interval, or an observation one
    /// trigger away could pass as a partner and teach the wrong offset.
    pub window: TimeDelta,
    /// How many observations have to agree on the framenumber offset before it
    /// is adopted, any disagreement starting the count over. The offset is
    /// learned once and never revised, so this is what stands between a
    /// coincidence and a session spent pairing the wrong triggers.
    pub confirm_frames: usize,
    /// How many observations to keep per camera while waiting for a partner.
    /// This is what lets an observation that was stuck on the way, rather than
    /// lost, still be paired.
    pub buffer_frames: usize,
}

/// Frame rate assumed for the timestamp window when the configuration does not
/// say what the cameras run at.
const ASSUMED_FPS: FloatType = 100.0;

/// Widest timestamp window used, whatever the frame rate. Cameras triggered
/// together are stamped within a millisecond or so of each other; a window
/// wider than this only invites a neighbouring trigger to pass as a partner.
const MAX_WINDOW: TimeDelta = TimeDelta::milliseconds(5);

impl StereoSyncParams {
    /// Parameters for cameras running at `fps` frames per second, falling back
    /// to an assumed rate when it is not configured.
    pub fn for_fps(fps: Option<FloatType>) -> Self {
        let fps = match fps {
            Some(fps) if fps > 0.0 => fps,
            _ => ASSUMED_FPS,
        };
        let interval_secs = 1.0 / fps;
        // Anything up to half a frame interval is unambiguous: no other trigger
        // is that close. `MAX_WINDOW` keeps the window sane for slow cameras.
        let window = TimeDelta::nanoseconds((interval_secs * 0.5 * 1e9) as i64).min(MAX_WINDOW);
        Self {
            window,
            // At any tracking frame rate this is a fraction of a second spent
            // making sure of a number FLO then relies on for the whole session.
            confirm_frames: 5,
            // Half a second or so of history at typical rates, bounded so a
            // camera that stops reporting cannot grow this without limit.
            buffer_frames: 8,
        }
    }
}

impl Default for StereoSyncParams {
    fn default() -> Self {
        Self::for_fps(None)
    }
}

/// Counts describing how pairing is going, for logging and for the BUI.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StereoSyncStats {
    /// Pairs emitted.
    pub paired: u64,
    /// Observations that expired without a partner ever arriving.
    pub unpaired: u64,
    /// Observations dropped for arriving after a newer trigger had been paired.
    pub late: u64,
    /// Framenumber matches whose acquisition timestamps disagreed, and which
    /// were therefore not emitted.
    ///
    /// Cameras on one trigger with fixed counters cannot produce this. A count
    /// that climbs means the learned offset does not (or no longer does)
    /// describe the cameras — a camera restarting its counter mid-session, say.
    pub rejected: u64,
}

/// Pairs the two tracking cameras' detections into stereo pairs.
///
/// Feed every detection to [`Self::push`], which returns a pair as soon as one
/// is complete. See the [module documentation](self) for why pairing works the
/// way it does.
#[derive(Debug)]
pub struct StereoSynchronizer {
    params: StereoSyncParams,
    /// Framenumber offset between the cameras, `secondary - primary`, once
    /// enough observations have agreed on it. Learned once.
    offset: Option<i64>,
    /// The offset being considered, and how many consecutive observations have
    /// suggested it.
    candidate: Option<(i64, usize)>,
    /// Observations still waiting for a partner, oldest first.
    primary: VecDeque<MomentCentroid>,
    secondary: VecDeque<MomentCentroid>,
    /// Acquisition time of the most recently paired trigger. Nothing older is
    /// accepted after it.
    paired_through: Option<chrono::DateTime<chrono::Utc>>,
    stats: StereoSyncStats,
}

impl StereoSynchronizer {
    pub fn new(params: StereoSyncParams) -> Self {
        Self {
            params,
            offset: None,
            candidate: None,
            primary: VecDeque::new(),
            secondary: VecDeque::new(),
            paired_through: None,
            stats: Default::default(),
        }
    }

    /// The framenumber offset in use, `secondary - primary`, once it has been
    /// learned.
    pub fn offset(&self) -> Option<i64> {
        self.offset
    }

    pub fn stats(&self) -> &StereoSyncStats {
        &self.stats
    }

    /// Take one detection from one camera, returning the stereo pair it
    /// completes, if any.
    ///
    /// Returns at most one pair: an observation belongs to exactly one trigger,
    /// so it can complete at most one.
    pub fn push(&mut self, role: CamRole, centroid: MomentCentroid) -> Option<StereoPair> {
        // Data from a trigger already left behind cannot be used: emitting it
        // would hand the controller an older measurement than the one it has.
        if let Some(paired_through) = self.paired_through
            && centroid.timestamp + self.params.window <= paired_through
        {
            self.stats.late += 1;
            tracing::debug!(
                "stereo pairing: dropping {} camera framenumber {}, which arrived after a newer \
                 trigger was already paired",
                cam_label(role),
                centroid.framenumber,
            );
            return None;
        }

        let arrived = Stamp {
            framenumber: centroid.framenumber,
            timestamp: centroid.timestamp,
        };
        self.buffer(role, centroid);

        // With the offset in hand, the partner is identified exactly.
        if self.offset.is_some() {
            return self.take_matching_pair();
        }
        // Otherwise see whether this observation, together with what the other
        // camera has sent, settles what the offset is.
        if self.learn_offset(role, arrived) {
            return self.take_matching_pair();
        }
        None
    }

    /// Store an observation, expiring the oldest if the camera's buffer is
    /// full.
    fn buffer(&mut self, role: CamRole, centroid: MomentCentroid) {
        let queue = match role {
            CamRole::Primary => &mut self.primary,
            CamRole::Secondary => &mut self.secondary,
        };
        queue.push_back(centroid);
        while queue.len() > self.params.buffer_frames {
            queue.pop_front();
            self.stats.unpaired += 1;
        }
    }

    /// Emit the newest pair whose two halves are both buffered, dropping
    /// anything older than it.
    ///
    /// Newest rather than oldest because this feeds a controller: where a
    /// backlog has built up — at startup, while the offset was still being
    /// learned — the pair worth having is the one describing where the subject
    /// is now, not a queue of where it has been.
    fn take_matching_pair(&mut self) -> Option<StereoPair> {
        let offset = self.offset?;
        let (pi, si) = self.primary.iter().enumerate().rev().find_map(|(pi, p)| {
            let si = self
                .secondary
                .iter()
                .position(|s| framenumber_offset(s.framenumber, p.framenumber) == offset)?;
            Some((pi, si))
        })?;

        // A framenumber match is only as good as the offset behind it, and that
        // is the one thing here that was inferred rather than given. Should it
        // be wrong, the same arithmetic pairs observations taken a frame or
        // more apart, putting a subject where it never was into the distance
        // estimate. The timestamps say so; leave the two where they are.
        let skew = self.secondary[si].timestamp - self.primary[pi].timestamp;
        if skew.abs() > self.params.window {
            self.stats.rejected += 1;
            return None;
        }

        // Everything before the pair belongs to older triggers. Holding it
        // would only let an out-of-order pair through later.
        let primary = self.discard_through(CamRole::Primary, pi);
        let secondary = self.discard_through(CamRole::Secondary, si);

        self.stats.paired += 1;
        self.paired_through = Some(primary.timestamp);
        Some(StereoPair { primary, secondary })
    }

    /// Remove and return the observation at `index`, counting everything
    /// dropped ahead of it as unpaired.
    fn discard_through(&mut self, role: CamRole, index: usize) -> MomentCentroid {
        let queue = match role {
            CamRole::Primary => &mut self.primary,
            CamRole::Secondary => &mut self.secondary,
        };
        let dropped = queue.drain(..index).count();
        self.stats.unpaired += dropped as u64;
        queue.pop_front().expect("index was found in this queue")
    }

    /// Take the observation that just arrived, look for one from the other
    /// camera close enough in time to be the same trigger, and adopt the
    /// framenumber offset the two imply once enough of them have agreed.
    ///
    /// The comparison is anchored on the arriving observation rather than on
    /// the newest of each camera: a camera whose frames are consistently stuck
    /// a few triggers behind is never the newest, and anchoring on the newest
    /// would leave such a pair unable to synchronize at all.
    ///
    /// Returns whether the offset was adopted.
    fn learn_offset(&mut self, role: CamRole, arrived: Stamp) -> bool {
        let other = match role {
            CamRole::Primary => &self.secondary,
            CamRole::Secondary => &self.primary,
        };
        let Some(partner) = closest(other, arrived.timestamp) else {
            return false;
        };
        if (partner.timestamp - arrived.timestamp).abs() > self.params.window {
            // Nothing from the other camera is close enough in time to say
            // anything about the offset either way. Most observations are like
            // this while the buffers fill, so this is silence, not
            // disagreement, and leaves any run in progress alone.
            return false;
        }

        let candidate = match role {
            CamRole::Primary => framenumber_offset(partner.framenumber, arrived.framenumber),
            CamRole::Secondary => framenumber_offset(arrived.framenumber, partner.framenumber),
        };
        let support = match self.candidate {
            Some((offset, support)) if offset == candidate => support + 1,
            _ => 1,
        };
        if support < self.params.confirm_frames {
            self.candidate = Some((candidate, support));
            return false;
        }

        tracing::info!("stereo pairing: cameras synchronized, framenumber offset {candidate}");
        self.offset = Some(candidate);
        self.candidate = None;
        true
    }
}

/// The observation in `queue` whose timestamp is closest to `target`.
fn closest(
    queue: &VecDeque<MomentCentroid>,
    target: chrono::DateTime<chrono::Utc>,
) -> Option<&MomentCentroid> {
    queue.iter().min_by_key(|c| (c.timestamp - target).abs())
}

fn cam_label(role: CamRole) -> &'static str {
    match role {
        CamRole::Primary => "primary",
        CamRole::Secondary => "secondary",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One detection, `ms` milliseconds after an arbitrary epoch.
    fn centroid(framenumber: u32, ms: i64) -> MomentCentroid {
        MomentCentroid {
            schema_version: 2,
            framenumber,
            timestamp: chrono::DateTime::<chrono::Utc>::UNIX_EPOCH + TimeDelta::milliseconds(ms),
            mu00: 1.0,
            mu10: f64::from(framenumber),
            ..Default::default()
        }
    }

    /// Parameters for the 100 fps cameras the tests describe.
    fn params() -> StereoSyncParams {
        StereoSyncParams::for_fps(Some(100.0))
    }

    /// Feed one trigger to `sync`, primary first, and return any pair.
    fn trigger(
        sync: &mut StereoSynchronizer,
        primary_framenumber: u32,
        secondary_framenumber: u32,
        ms: i64,
    ) -> Option<StereoPair> {
        let first = sync.push(CamRole::Primary, centroid(primary_framenumber, ms));
        let second = sync.push(CamRole::Secondary, centroid(secondary_framenumber, ms));
        first.or(second)
    }

    #[test]
    fn window_is_below_half_a_frame_interval() {
        // 100 fps: half an interval is 5 ms, the widest window allowed.
        assert_eq!(params().window, TimeDelta::milliseconds(5));
        // 30 fps would give 16.7 ms, which the cap brings back down.
        assert_eq!(
            StereoSyncParams::for_fps(Some(30.0)).window,
            TimeDelta::milliseconds(5)
        );
        // 500 fps: 1 ms, well under the cap.
        assert_eq!(
            StereoSyncParams::for_fps(Some(500.0)).window,
            TimeDelta::milliseconds(1)
        );
        // No configured rate still gives a usable window.
        assert!(StereoSyncParams::for_fps(None).window > TimeDelta::zero());
    }

    #[test]
    fn learns_an_arbitrary_offset_then_pairs_on_framenumbers() {
        let mut sync = StereoSynchronizer::new(params());
        // The secondary counter reads 1000 above the primary's.
        let pairs: Vec<_> = (0..10)
            .filter_map(|i| trigger(&mut sync, i, i + 1000, i64::from(i) * 10))
            .collect();
        assert_eq!(sync.offset(), Some(1000));
        // The first triggers go into learning the offset; every trigger from
        // then on is paired.
        assert_eq!(pairs.len(), 6);
        for pair in &pairs {
            assert_eq!(
                framenumber_offset(pair.secondary.framenumber, pair.primary.framenumber),
                1000
            );
            assert_eq!(pair.skew(), TimeDelta::zero());
        }
    }

    #[test]
    fn learns_a_negative_offset_across_the_u32_wrap() {
        let mut sync = StereoSynchronizer::new(params());
        // The secondary counter reads 7 below the primary's, and the primary
        // started near the end of the u32 range, so both wrap during the run.
        let base = u32::MAX - 2;
        for i in 0..8 {
            trigger(
                &mut sync,
                base.wrapping_add(i),
                base.wrapping_add(i).wrapping_sub(7),
                i64::from(i) * 10,
            );
        }
        assert_eq!(sync.offset(), Some(-7));
    }

    #[test]
    fn pairs_a_secondary_that_arrives_a_few_triggers_late() {
        let mut sync = StereoSynchronizer::new(params());
        for i in 0..8 {
            trigger(&mut sync, i, i + 1000, i64::from(i) * 10);
        }

        // The secondary now runs three triggers behind in delivery, while its
        // framenumbers and timestamps stay correct: frames stuck on the way,
        // not lost.
        let lag = 3;
        let mut pairs = Vec::new();
        for i in 8..18 {
            let ms = i64::from(i) * 10;
            pairs.extend(sync.push(CamRole::Primary, centroid(i, ms)));
            let late = i - lag;
            pairs.extend(sync.push(
                CamRole::Secondary,
                centroid(late + 1000, i64::from(late) * 10),
            ));
        }
        // Pairing resumes with the first trigger whose primary is still
        // buffered, and then covers every trigger: buffering a few frames is
        // what makes stuck data usable rather than lost.
        let framenumbers: Vec<u32> = pairs.iter().map(|p| p.primary.framenumber).collect();
        assert_eq!(framenumbers, (8..=14).collect::<Vec<u32>>());
        for pair in &pairs {
            assert_eq!(pair.primary.framenumber + 1000, pair.secondary.framenumber);
        }
        // The secondaries for triggers already paired before the lag started
        // are dropped rather than re-paired out of order.
        assert!(sync.stats().late > 0);
    }

    #[test]
    fn synchronizes_a_secondary_that_was_late_from_the_start() {
        let mut sync = StereoSynchronizer::new(params());
        // The secondary has been running three triggers behind delivery since
        // before FLO started, so it is never the newest observation held and
        // the two cameras are never each other's most recent data.
        let lag = 3;
        let mut pairs = Vec::new();
        for i in lag..16 {
            let ms = i64::from(i) * 10;
            pairs.extend(sync.push(CamRole::Primary, centroid(i, ms)));
            let late = i - lag;
            pairs.extend(sync.push(
                CamRole::Secondary,
                centroid(late + 1000, i64::from(late) * 10),
            ));
        }
        assert_eq!(sync.offset(), Some(1000));
        assert!(
            !pairs.is_empty(),
            "a permanently late camera still has to synchronize"
        );
    }

    #[test]
    fn a_lagging_camera_does_not_teach_the_lag_as_the_offset() {
        let mut sync = StereoSynchronizer::new(params());
        // Delivery order alone would suggest the secondary's counter runs
        // three behind. Only the timestamps say which observations share a
        // trigger, and they are what the offset is learned from.
        let lag = 3;
        for i in lag..16 {
            let ms = i64::from(i) * 10;
            sync.push(CamRole::Primary, centroid(i, ms));
            let late = i - lag;
            sync.push(
                CamRole::Secondary,
                centroid(late + 1000, i64::from(late) * 10),
            );
        }
        assert_eq!(sync.offset(), Some(1000), "1000 - lag would be wrong");
    }

    #[test]
    fn drops_an_observation_older_than_one_already_paired() {
        let mut sync = StereoSynchronizer::new(params());
        for i in 0..10 {
            trigger(&mut sync, i, i + 1000, i64::from(i) * 10);
        }
        let late_before = sync.stats().late;
        // A straggler from a trigger long since paired.
        assert!(
            sync.push(CamRole::Secondary, centroid(1000, 0)).is_none(),
            "data from an already-paired trigger is not usable"
        );
        assert_eq!(sync.stats().late, late_before + 1);
    }

    #[test]
    fn does_not_pair_a_neighbouring_trigger() {
        let mut sync = StereoSynchronizer::new(params());
        // Only the primary sees the subject on even triggers, only the
        // secondary on odd ones: never the same trigger, so never a pair.
        for i in 0..16 {
            let ms = i64::from(i) * 10;
            let pair = if i % 2 == 0 {
                sync.push(CamRole::Primary, centroid(i, ms))
            } else {
                sync.push(CamRole::Secondary, centroid(i + 1000, ms))
            };
            assert!(pair.is_none(), "observations 10 ms apart are not a pair");
        }
        assert_eq!(sync.offset(), None);
    }

    #[test]
    fn one_coincidence_does_not_settle_the_offset() {
        let mut sync = StereoSynchronizer::new(params());
        // The secondary detects the subject on one trigger only. That single
        // agreeing observation must not become the offset for the session.
        sync.push(CamRole::Primary, centroid(0, 0));
        sync.push(CamRole::Secondary, centroid(500, 0));
        for i in 1..6 {
            sync.push(CamRole::Primary, centroid(i, i64::from(i) * 10));
        }
        assert_eq!(sync.offset(), None);
    }

    #[test]
    fn does_not_emit_a_pair_the_timestamps_disagree_with() {
        let mut sync = StereoSynchronizer::new(params());
        for i in 0..10 {
            trigger(&mut sync, i, i + 1000, i64::from(i) * 10);
        }
        assert_eq!(sync.offset(), Some(1000));

        // Something the cameras cannot do: the secondary's counter jumps, so
        // the learned offset now names observations from other triggers.
        let mut pairs = Vec::new();
        for i in 10..20 {
            pairs.extend(trigger(&mut sync, i, i + 1004, i64::from(i) * 10));
        }
        assert!(
            pairs.is_empty(),
            "a match the timestamps disagree with is not a stereo pair"
        );
        assert!(sync.stats().rejected > 0);
    }

    #[test]
    fn a_camera_that_stops_reporting_does_not_grow_the_buffers() {
        let mut sync = StereoSynchronizer::new(params());
        for i in 0..1000 {
            sync.push(CamRole::Primary, centroid(i, i64::from(i) * 10));
        }
        assert!(sync.primary.len() <= params().buffer_frames);
        assert_eq!(sync.offset(), None);
    }
}
