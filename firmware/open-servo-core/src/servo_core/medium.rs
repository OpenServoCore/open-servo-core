//! Medium tick pipeline (1kHz policy FSM).
//!
//! Decimated from fast tick. Handles mode transitions and policy decisions.
//! Uses window_dt_us from FastAccumulator for all timing (dt/sample_count independent).

use crate::servo_core::ServoMode;
#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;
use open_servo_math::{CentiDeg, DegPerSec10};

/// Snapshot of windowed statistics from fast tick accumulator.
///
/// Contains all data needed for medium tick policy decisions.
#[derive(Debug, Clone)]
pub struct MediumSnapshot {
    /// Last position in window
    pub position: CentiDeg,
    /// Velocity computed from position delta over window
    pub velocity: DegPerSec10,
    /// Average current over window (if available)
    #[cfg(feature = "current-sense-bus")]
    pub avg_current: Option<MilliAmp>,
    /// Total elapsed time in this window (microseconds)
    /// This is the key timing value - all FSM logic uses this.
    pub window_dt_us: u32,
}

/// Policy decision from medium tick.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PolicyDecision {
    /// No mode change
    NoChange,
    /// Transition to a new mode
    TransitionTo(ServoMode),
}

/// Setpoint tracking state.
#[derive(Debug, Clone, Default)]
pub struct SetpointTracker {
    /// Previous setpoint for change detection
    pub prev_setpoint: CentiDeg,
    /// Time since setpoint last changed (microseconds)
    pub unchanged_us: u32,
    /// Whether we've received the first setpoint
    initialized: bool,
}

impl SetpointTracker {
    /// Create a new setpoint tracker.
    pub fn new() -> Self {
        Self::default()
    }

    /// Update tracking with current setpoint and elapsed time.
    ///
    /// Returns true if setpoint has been stable for at least `settle_us`.
    pub fn update(&mut self, setpoint: CentiDeg, dt_us: u32, settle_us: u32) -> bool {
        if !self.initialized {
            // First call: just record the setpoint and start accumulating
            self.prev_setpoint = setpoint;
            self.initialized = true;
            self.unchanged_us = self.unchanged_us.saturating_add(dt_us);
            self.unchanged_us >= settle_us
        } else if setpoint != self.prev_setpoint {
            // Setpoint changed: reset timer
            self.prev_setpoint = setpoint;
            self.unchanged_us = 0;
            false
        } else {
            // Same setpoint: accumulate time
            self.unchanged_us = self.unchanged_us.saturating_add(dt_us);
            self.unchanged_us >= settle_us
        }
    }

    /// Check if setpoint changed recently.
    pub fn changed_recently(&self, recent_us: u32) -> bool {
        self.unchanged_us < recent_us
    }

    /// Reset tracker state.
    pub fn reset(&mut self) {
        self.prev_setpoint = CentiDeg::from_cdeg(0);
        self.unchanged_us = 0;
        self.initialized = false;
    }
}

/// Hold entry conditions tracker.
#[derive(Debug, Clone, Default)]
pub struct HoldConditionsTracker {
    /// Time that hold entry conditions have been continuously met (microseconds)
    pub conditions_met_us: u32,
}

impl HoldConditionsTracker {
    /// Create a new hold conditions tracker.
    pub fn new() -> Self {
        Self::default()
    }

    /// Update tracking based on whether conditions are currently met.
    ///
    /// Returns true if conditions have been met for at least `entry_us`.
    pub fn update(&mut self, conditions_met: bool, dt_us: u32, entry_us: u32) -> bool {
        if conditions_met {
            self.conditions_met_us = self.conditions_met_us.saturating_add(dt_us);
            self.conditions_met_us >= entry_us
        } else {
            self.conditions_met_us = 0;
            false
        }
    }

    /// Reset tracker state.
    pub fn reset(&mut self) {
        self.conditions_met_us = 0;
    }
}

/// Backdrive detection state.
#[derive(Debug, Clone, Default)]
pub struct BackdriveDetector {
    /// Previous PWM command for sign comparison
    pub prev_pwm: i16,
    /// Previous error for sign comparison
    pub prev_error: i16,
    /// Time backdrive condition has persisted (microseconds)
    pub elapsed_us: u32,
}

impl BackdriveDetector {
    /// Create a new backdrive detector.
    pub fn new() -> Self {
        Self::default()
    }

    /// Check for backdrive condition.
    ///
    /// Backdrive is detected when:
    /// - PWM and error have opposite signs (motor fighting external force)
    /// - Velocity exceeds threshold (servo is being moved)
    /// - Condition persists for `persist_us`
    ///
    /// Returns true if backdrive should trigger Yield mode.
    pub fn check(
        &mut self,
        pwm: i16,
        error: i16,
        velocity: DegPerSec10,
        dt_us: u32,
        vel_threshold: i16,
        deadband: i16,
        persist_us: u32,
    ) -> bool {
        // Check for opposite signs (backdrive condition)
        let pwm_sign = if pwm > deadband {
            1
        } else if pwm < -deadband {
            -1
        } else {
            0
        };
        let error_sign = if error > 0 {
            1
        } else if error < 0 {
            -1
        } else {
            0
        };

        let vel_abs = velocity.as_dps10().saturating_abs();
        let signs_opposite = pwm_sign != 0 && error_sign != 0 && pwm_sign != error_sign;
        let velocity_high = vel_abs > vel_threshold;

        if signs_opposite && velocity_high {
            self.elapsed_us = self.elapsed_us.saturating_add(dt_us);
            if self.elapsed_us >= persist_us {
                return true;
            }
        } else {
            self.elapsed_us = 0;
        }

        self.prev_pwm = pwm;
        self.prev_error = error;
        false
    }

    /// Reset detector state.
    pub fn reset(&mut self) {
        self.prev_pwm = 0;
        self.prev_error = 0;
        self.elapsed_us = 0;
    }
}

// Note: aggregate_medium, policy_medium, apply_policy, and run_medium_tick
// will be implemented in Commit 4 when we wire up to CoreConfig/CoreInternal.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_setpoint_tracker_stable() {
        let mut tracker = SetpointTracker::new();
        let sp = CentiDeg::from_cdeg(9000);

        // First update - not stable yet
        assert!(!tracker.update(sp, 100_000, 500_000));
        assert_eq!(tracker.unchanged_us, 100_000);

        // Continue accumulating
        assert!(!tracker.update(sp, 200_000, 500_000));
        assert_eq!(tracker.unchanged_us, 300_000);

        // Cross threshold
        assert!(tracker.update(sp, 200_000, 500_000));
        assert_eq!(tracker.unchanged_us, 500_000);
    }

    #[test]
    fn test_setpoint_tracker_change_resets() {
        let mut tracker = SetpointTracker::new();
        let sp1 = CentiDeg::from_cdeg(9000);
        let sp2 = CentiDeg::from_cdeg(9100);

        tracker.update(sp1, 400_000, 500_000);
        assert_eq!(tracker.unchanged_us, 400_000);

        // Change resets
        tracker.update(sp2, 100_000, 500_000);
        assert_eq!(tracker.unchanged_us, 0);
    }

    #[test]
    fn test_hold_conditions_tracker() {
        let mut tracker = HoldConditionsTracker::new();

        // Conditions not met
        assert!(!tracker.update(false, 100_000, 200_000));
        assert_eq!(tracker.conditions_met_us, 0);

        // Conditions met, accumulating
        assert!(!tracker.update(true, 100_000, 200_000));
        assert_eq!(tracker.conditions_met_us, 100_000);

        // Cross threshold
        assert!(tracker.update(true, 100_000, 200_000));
    }

    #[test]
    fn test_backdrive_detector_opposite_signs() {
        let mut detector = BackdriveDetector::new();

        // PWM positive, error negative, high velocity
        let triggered = detector.check(
            5000,                         // pwm
            -1000,                        // error (opposite sign)
            DegPerSec10::from_dps10(200), // velocity
            600,                          // dt_us
            100,                          // vel_threshold
            1000,                         // deadband
            500,                          // persist_us
        );
        assert!(triggered);
    }

    #[test]
    fn test_backdrive_detector_same_signs_no_trigger() {
        let mut detector = BackdriveDetector::new();

        // PWM and error same sign - no backdrive
        let triggered = detector.check(
            5000,
            1000, // same sign
            DegPerSec10::from_dps10(200),
            600,
            100,
            1000,
            500,
        );
        assert!(!triggered);
    }
}
