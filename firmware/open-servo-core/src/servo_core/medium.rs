//! Medium tick pipeline (1kHz policy FSM).
//!
//! Decimated from fast tick. Handles mode transitions and policy decisions.
//! Uses window_dt_us from FastAccumulator for all timing (dt/sample_count independent).

use crate::accumulator::MediumSnapshot;
use crate::inputs::build_medium_control_input;
use crate::servo_core::config::CoreConfig;
use crate::servo_core::features::{backdrive, compliance};
use crate::servo_core::internal::CoreInternal;
use crate::servo_core::ServoMode;
use open_servo_control::ControlLoop;
use open_servo_math::{CentiDeg, CentiDeg32, DegPerSec10, TickCtx};

// ============================================================================
// Policy timing constants (microseconds)
// ============================================================================

/// Time setpoint must be stable before considering Hold entry
pub(crate) const SETPOINT_SETTLE_US: u32 = 400_000; // 400ms
/// Time hold conditions must persist for mode transition
pub(crate) const HOLD_ENTRY_US: u32 = 300_000; // 300ms
/// Recent setpoint change threshold (for exit hysteresis)
pub(crate) const SETPOINT_RECENT_CHANGE_US: u32 = 10_000; // 10ms
/// Time backdrive condition must persist before triggering Yield
pub(crate) const BACKDRIVE_PERSIST_US: u32 = 500; // 0.5ms

// ============================================================================
// Mode transition thresholds (with hysteresis)
// ============================================================================

/// Error threshold for entering Hold mode (centidegrees)
const HOLD_ENTER_ERROR_CDEG: i16 = 500; // 5°
/// Error threshold for exiting Hold mode (centidegrees)
const HOLD_EXIT_ERROR_CDEG: i16 = 700; // 7°
/// Velocity threshold for entering Hold mode (dps*10)
const HOLD_ENTER_VEL_DPS10: i16 = 100; // 10°/s
/// Velocity threshold for exiting Hold mode (dps*10)
const HOLD_EXIT_VEL_DPS10: i16 = 150; // 15°/s
/// Velocity threshold for backdrive detection (dps*10)
const BACKDRIVE_VEL_THRESHOLD: i16 = 300; // 30°/s
/// PWM deadband for sign comparison
const U_DEADBAND: i16 = 1638; // 5%

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

// ============================================================================
// Medium tick pipeline functions
// ============================================================================

/// Aggregate fast-tick samples into medium-tick snapshot.
///
/// Takes ownership of accumulated samples and resets the accumulator.
/// Returns None if no samples were collected.
pub fn aggregate_medium(internal: &mut CoreInternal, fast_dt_us: u32) -> Option<MediumSnapshot> {
    internal.fast_accum.take_snapshot(fast_dt_us)
}

/// Run policy FSM based on current state and snapshot.
///
/// Returns PolicyDecision indicating whether to change modes.
pub fn policy_medium(internal: &CoreInternal, snap: &MediumSnapshot) -> PolicyDecision {
    let window_dt_us = snap.window_dt_us;
    let velocity = snap.velocity_dps10;
    let vel_abs = velocity.as_dps10().saturating_abs();

    // Get error from setpoint vs position
    let Some(setpoint32) = internal.setpoint else {
        return PolicyDecision::NoChange;
    };
    let position32 = CentiDeg32::from(snap.pos_last);
    let error32 = setpoint32 - position32;
    let error = error32.to_cdeg_i16_sat();
    let error_abs = error.saturating_abs();

    match internal.mode {
        ServoMode::Move => {
            // Check if setpoint has been stable long enough
            if internal.setpoint_tracker.unchanged_us < SETPOINT_SETTLE_US {
                return PolicyDecision::NoChange;
            }

            // Check hold entry conditions
            let hold_conditions =
                error_abs < HOLD_ENTER_ERROR_CDEG && vel_abs < HOLD_ENTER_VEL_DPS10;

            if hold_conditions {
                // Check if conditions have been met long enough
                let conditions_time = internal
                    .hold_tracker
                    .conditions_met_us
                    .saturating_add(window_dt_us);
                if conditions_time >= HOLD_ENTRY_US {
                    return PolicyDecision::TransitionTo(ServoMode::Hold);
                }
            }
            PolicyDecision::NoChange
        }

        ServoMode::Hold => {
            // Check exit conditions (hysteresis thresholds)
            let setpoint_changed =
                internal.setpoint_tracker.unchanged_us < SETPOINT_RECENT_CHANGE_US;
            let error_too_large = error_abs > HOLD_EXIT_ERROR_CDEG;
            let velocity_too_high = vel_abs > HOLD_EXIT_VEL_DPS10;

            if setpoint_changed || error_too_large || velocity_too_high {
                return PolicyDecision::TransitionTo(ServoMode::Move);
            }

            // Check for backdrive (opposing velocity/PWM condition)
            if check_backdrive_condition(internal, velocity, error, window_dt_us) {
                return PolicyDecision::TransitionTo(ServoMode::Yield);
            }

            PolicyDecision::NoChange
        }

        ServoMode::Yield => {
            // Yield timing is handled by backdrive window
            // Check if window has completed
            if backdrive::should_exit_backdrive(&internal.backdrive) {
                return PolicyDecision::TransitionTo(backdrive::next_mode_after_backdrive());
            }
            PolicyDecision::NoChange
        }
    }
}

/// Check for backdrive condition in Hold mode.
fn check_backdrive_condition(
    internal: &CoreInternal,
    velocity: DegPerSec10,
    _error: i16,
    window_dt_us: u32,
) -> bool {
    let vel = velocity.as_dps10();
    let vel_abs = vel.saturating_abs();

    // Must exceed velocity threshold
    if vel_abs <= BACKDRIVE_VEL_THRESHOLD {
        return false;
    }

    // Sign mismatch with deadband (using cached previous PWM)
    let u = internal.backdrive_detector.prev_pwm;
    let u_active = u.saturating_abs() > U_DEADBAND;
    let opposing = u_active && ((vel > 0) != (u > 0));

    // Error growing (with small deadband)
    let prev_error = internal.backdrive_detector.prev_error;
    let error_abs = prev_error.saturating_abs();
    let prev_error_abs = internal.backdrive_detector.prev_error.saturating_abs();
    let error_growing = error_abs > prev_error_abs.saturating_add(10);

    // Check if elapsed time exceeds persist threshold
    if opposing || error_growing {
        let elapsed = internal
            .backdrive_detector
            .elapsed_us
            .saturating_add(window_dt_us);
        if elapsed >= BACKDRIVE_PERSIST_US {
            return true;
        }
    }

    false
}

/// Apply policy decision and update state.
///
/// Handles mode transitions, compliance config switching, and backdrive window timing.
pub fn apply_policy(
    internal: &mut CoreInternal,
    config: &CoreConfig,
    snap: &MediumSnapshot,
    decision: PolicyDecision,
) {
    let window_dt_us = snap.window_dt_us;

    // Update velocity from snapshot
    internal.measured_velocity = snap.velocity_dps10;

    // Get setpoint for tracking
    let setpoint = internal
        .setpoint
        .map(|sp32| sp32.to_centi_deg_sat())
        .unwrap_or(CentiDeg::from_cdeg(0));

    // Update setpoint tracking (time-based)
    internal
        .setpoint_tracker
        .update(setpoint, window_dt_us, SETPOINT_SETTLE_US);

    // Compute error for hold tracker
    let position32 = CentiDeg32::from(snap.pos_last);
    let error = internal
        .setpoint
        .map(|sp32| (sp32 - position32).to_cdeg_i16_sat())
        .unwrap_or(0);
    let error_abs = error.saturating_abs();
    let vel_abs = internal.measured_velocity.as_dps10().saturating_abs();

    // Update hold conditions tracker (only in Move mode)
    if internal.mode == ServoMode::Move {
        let hold_conditions = internal.setpoint_tracker.unchanged_us >= SETPOINT_SETTLE_US
            && error_abs < HOLD_ENTER_ERROR_CDEG
            && vel_abs < HOLD_ENTER_VEL_DPS10;
        internal
            .hold_tracker
            .update(hold_conditions, window_dt_us, HOLD_ENTRY_US);
    }

    // Update backdrive detector timing (only in Hold mode)
    if internal.mode == ServoMode::Hold {
        let velocity = snap.velocity_dps10;
        let vel = velocity.as_dps10();
        let vel_abs_check = vel.saturating_abs();

        let u = internal.backdrive_detector.prev_pwm;
        let u_active = u.saturating_abs() > U_DEADBAND;
        let opposing = u_active && ((vel > 0) != (u > 0));

        let prev_error_abs = internal.backdrive_detector.prev_error.saturating_abs();
        let error_growing = error_abs > prev_error_abs.saturating_add(10);

        if vel_abs_check > BACKDRIVE_VEL_THRESHOLD && (opposing || error_growing) {
            internal.backdrive_detector.elapsed_us = internal
                .backdrive_detector
                .elapsed_us
                .saturating_add(window_dt_us);
        } else {
            internal.backdrive_detector.elapsed_us = 0;
        }
    }

    // Update backdrive window timing (only in Yield mode)
    if internal.mode == ServoMode::Yield {
        backdrive::update_backdrive_window(&mut internal.backdrive, window_dt_us);
    }

    // Apply mode transition if requested
    match decision {
        PolicyDecision::NoChange => {}
        PolicyDecision::TransitionTo(new_mode) => {
            let prev_mode = internal.mode;
            internal.mode = new_mode;

            // Handle mode-specific transitions
            match (prev_mode, new_mode) {
                (ServoMode::Move, ServoMode::Hold) => {
                    internal.hold_tracker.reset();
                    compliance::switch_config(
                        &mut internal.compliance,
                        new_mode,
                        &config.compliance,
                    );
                }
                (ServoMode::Hold, ServoMode::Move) => {
                    internal.backdrive_detector.reset();
                    compliance::switch_config(
                        &mut internal.compliance,
                        new_mode,
                        &config.compliance,
                    );
                }
                (ServoMode::Hold, ServoMode::Yield) => {
                    backdrive::on_backdrive_entry(&mut internal.backdrive);
                    internal.backdrive_detector.reset();
                }
                (ServoMode::Yield, ServoMode::Hold) => {
                    backdrive::on_backdrive_exit(&mut internal.backdrive);
                }
                _ => {}
            }
        }
    }
}

/// Execute complete medium tick pipeline.
///
/// Pipeline: aggregate → policy → apply
///
/// All timing uses window_dt_us from the snapshot, which is the actual elapsed
/// time since the last medium tick (sample_count × fast_dt_us).
///
/// Note: `ctx.dt_us` here is the fast tick period (e.g., 100us at 10kHz),
/// which is used to compute window_dt_us = fast_dt_us × sample_count.
pub fn run_medium_tick<C: ControlLoop>(
    internal: &mut CoreInternal,
    config: &CoreConfig,
    ctx: &TickCtx,
    controller: &mut C,
) {
    // Aggregate fast-tick samples (uses ctx.dt_us as fast_dt_us)
    let Some(snap) = aggregate_medium(internal, ctx.dt_us) else {
        return;
    };

    // Get last_input for building medium control input
    let Some(base_input) = internal.last_input.clone() else {
        return;
    };

    // Run policy FSM
    let decision = policy_medium(internal, &snap);

    // Apply policy and update state
    apply_policy(internal, config, &snap, decision);

    // Build medium input and call controller
    let medium_input = build_medium_control_input(&base_input, &snap);
    controller.medium_tick(ctx, &medium_input);
}

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
