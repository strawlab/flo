use crate::FloatType;
use chrono;
use serde::{Deserialize, Serialize};

/// A tool to observe value change.
///
/// [Self::update] updates the value and indicates if the value has changed.
#[derive(Clone, Debug)]
pub struct ChangeDetector<T: Eq + Clone> {
    pub old_value: Option<T>,
    pub latest_change: Option<(Option<T>, T)>,
}

impl<T: Eq + Clone> Default for ChangeDetector<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Eq + Clone> ChangeDetector<T> {
    pub fn new() -> Self {
        Self {
            old_value: None,
            latest_change: None,
        }
    }
    pub fn new_with_initial_state(val: &T) -> Self {
        Self {
            old_value: Some(val.clone()),
            latest_change: None,
        }
    }
    /// Update the stored value.
    ///
    /// Returns true if the value has changed vs the previous
    /// call. The first call always returns false.
    pub fn update(&mut self, val: &T) -> bool {
        match self.old_value {
            None => {
                self.latest_change = Some((self.old_value.clone(), val.clone()));
                self.old_value = Some(val.clone());
                false
            }
            Some(ref old_val) => {
                if *old_val != *val {
                    self.latest_change = Some((self.old_value.clone(), val.clone()));
                    self.old_value = Some(val.clone());
                    true
                } else {
                    false
                }
            }
        }
    }
    /// Update the stored value and, if the value has changed, test if the new
    /// value is equal to `to`.
    ///
    /// See [Self::update] for information about the return value.
    pub fn update_and_has_changed_to(&mut self, val: &T, to: &T) -> bool {
        self.update(val) && *val == *to
    }
}

//------------------------------noise gate-------------------------------------

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
pub struct NoiseGateParameters {
    ///how large a noise are we rejecting (peak-to-peak)
    pub noise_gate: FloatType,
    ///for how long to pass through all the noise after the gate is broken
    pub hold_time: FloatType,
}

///Noise gate: detect changes in float value, with some noise threshold. Once the threshold is broken, the noise gate will pass-through all noise for params.hold_time seconds.
#[derive(Debug, Clone)]
pub struct NoiseGate {
    gated_val: FloatType,
    state: NoiseGateStatus,
    ///the last time the threshold has been surpassed
    pub last_break_time: Option<std::time::Instant>,
    pub params: NoiseGateParameters,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum NoiseGateStatus {
    FirstValue,
    Stable,
    Varying,
}

impl NoiseGate {
    pub fn new(params: NoiseGateParameters) -> Self {
        Self {
            gated_val: 0.0,
            last_break_time: None,
            params,
            state: NoiseGateStatus::FirstValue,
        }
    }

    pub fn update(&mut self, value: FloatType) -> (NoiseGateStatus, FloatType) {
        use NoiseGateStatus::*;
        if self.state == FirstValue {
            self.gated_val = value;
            self.state = NoiseGateStatus::Stable;
            (FirstValue, value)
        } else {
            let now = std::time::Instant::now();
            if (value - self.gated_val).abs() >= self.params.noise_gate * 0.5 {
                self.gated_val = value;
                self.last_break_time = Some(now);
                self.state = Varying;
                (Varying, value)
            } else if self.state == Varying {
                let dt = (now - self.last_break_time.unwrap()).as_secs_f64();
                if dt < self.params.hold_time {
                    (Varying, value)
                } else {
                    self.gated_val = value; //transitioning into stable. Update the value one last time
                    self.state = Stable;
                    (Varying, self.gated_val) //still report "Varying" to force external code to use this latest value
                }
            } else {
                (Stable, self.gated_val)
            }
        }
    }
}

pub type MyTimestamp = chrono::DateTime<chrono::Local>;
#[allow(non_upper_case_globals)]
pub fn now() -> MyTimestamp {
    chrono::Local::now()
}

pub fn elapsed(ts: MyTimestamp) -> FloatType {
    let now_ts = now();
    elapsed_by(ts, now_ts)
}

pub fn elapsed_by(ts: MyTimestamp, now_ts: MyTimestamp) -> FloatType {
    (now_ts - ts).num_milliseconds() as FloatType / 1000.0
}

#[derive(Debug, Clone)]
pub struct Timestamped<T: Clone> {
    pub value: T,
    pub timestamp: MyTimestamp,
}

impl<T: Clone> From<T> for Timestamped<T> {
    fn from(value: T) -> Self {
        Self {
            timestamp: now(),
            value,
        }
    }
}

impl<T: Clone> Timestamped<T> {
    pub fn elapsed(&self) -> FloatType {
        elapsed(self.timestamp)
    }
    pub fn elapsed_by(&self, now_ts: MyTimestamp) -> FloatType {
        elapsed_by(self.timestamp, now_ts)
    }
}
