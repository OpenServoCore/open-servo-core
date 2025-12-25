//! Backdrive window timing and phase management.
//!
//! When backdrive is detected, the servo enters a timed window that allows
//! external forces to move the mechanism. The window has two phases:
//! - Coast (0-Nms): PWM duty capped at 0, letting the servo coast
//! - Alive (N-Mms): PWM duty capped at low value, maintaining some position
//!
//! Timing parameters come from PolicyConfig (board-supplied).

use crate::servo_core::features::PolicyConfig;
use crate::servo_core::ServoMode;

/// Phase within the backdrive window
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackdrivePhase {
    /// Coast phase (0-100ms): zero duty
    Coast,
    /// Alive phase (100-200ms): low duty allowed
    Alive,
}

/// Mutable backdrive window state.
///
/// Lives in CoreInternal.backdrive
#[derive(Debug, Clone, Copy, Default)]
pub struct BackdriveState {
    /// Accumulated time in backdrive window (microseconds)
    pub elapsed_us: u32,
    /// Maximum duty magnitude during window (set by medium tick, used by fast tick)
    pub max_duty: i16,
    /// Flag to reset controller on next fast tick after entering window
    pub needs_reset: bool,
}

impl BackdriveState {
    /// Create a new BackdriveState
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset backdrive state (called on window exit or clear_fault)
    pub fn reset(&mut self) {
        self.elapsed_us = 0;
        self.max_duty = 0;
        self.needs_reset = false;
    }
}

/// Update backdrive window timing based on elapsed time.
///
/// Called during medium tick when in Yield mode.
/// Updates elapsed_us and max_duty based on which phase we're in.
pub fn update_backdrive_window(state: &mut BackdriveState, policy: &PolicyConfig, dt_us: u32) {
    // Accumulate time, clamping to yield_duration_us
    state.elapsed_us = state
        .elapsed_us
        .saturating_add(dt_us)
        .min(policy.yield_duration_us);

    // Set max_duty based on phase
    state.max_duty = if state.elapsed_us < policy.yield_coast_us {
        0 // Coast phase
    } else {
        policy.yield_alive_duty_max.as_raw() // Alive phase
    };
}

/// Get current backdrive phase from state.
pub fn get_backdrive_phase(state: &BackdriveState, policy: &PolicyConfig) -> BackdrivePhase {
    if state.elapsed_us < policy.yield_coast_us {
        BackdrivePhase::Coast
    } else {
        BackdrivePhase::Alive
    }
}

/// Check if backdrive window has completed and mode should exit.
///
/// Returns true when elapsed_us >= yield_duration_us.
pub fn should_exit_backdrive(state: &BackdriveState, policy: &PolicyConfig) -> bool {
    state.elapsed_us >= policy.yield_duration_us
}

/// Called when entering backdrive window (Yield mode).
///
/// Resets elapsed time and sets up for coast phase.
pub fn on_backdrive_entry(state: &mut BackdriveState) {
    state.elapsed_us = 0;
    state.max_duty = 0;
    state.needs_reset = true;
}

/// Called when exiting backdrive window.
///
/// Resets all backdrive state.
pub fn on_backdrive_exit(state: &mut BackdriveState) {
    state.reset();
}

/// Get the next mode after backdrive window (when should_exit_backdrive returns true).
pub fn next_mode_after_backdrive() -> ServoMode {
    ServoMode::Hold
}

#[cfg(test)]
mod tests {
    use super::*;
    use open_servo_math::{CentiDeg, DegPerSec10, Duty};

    /// Create test policy config with standard timing.
    fn test_policy() -> PolicyConfig {
        PolicyConfig {
            hold_enter_error: CentiDeg::from_cdeg(500),
            hold_exit_error: CentiDeg::from_cdeg(700),
            hold_enter_vel: DegPerSec10::from_dps10(100),
            hold_exit_vel: DegPerSec10::from_dps10(150),
            backdrive_vel_threshold: DegPerSec10::from_dps10(300),
            backdrive_deadband: Duty::from_raw(1638),
            backdrive_persist_us: 500,
            yield_alive_duty_max: Duty::from_raw(1638),
            yield_coast_us: 100_000,
            yield_duration_us: 200_000,
            hold_duty_error_start: CentiDeg::from_cdeg(500),
            hold_duty_error_end: CentiDeg::from_cdeg(1500),
            hold_duty_min: Duty::from_raw(6553),
            hold_duty_max: Duty::from_raw(14746),
        }
    }

    #[test]
    fn test_backdrive_phase_coast() {
        let policy = test_policy();
        let state = BackdriveState {
            elapsed_us: 0,
            max_duty: 0,
            needs_reset: false,
        };
        assert_eq!(get_backdrive_phase(&state, &policy), BackdrivePhase::Coast);
    }

    #[test]
    fn test_backdrive_phase_alive() {
        let policy = test_policy();
        let state = BackdriveState {
            elapsed_us: policy.yield_coast_us,
            max_duty: policy.yield_alive_duty_max.as_raw(),
            needs_reset: false,
        };
        assert_eq!(get_backdrive_phase(&state, &policy), BackdrivePhase::Alive);
    }

    #[test]
    fn test_backdrive_timing_coast_to_alive() {
        let policy = test_policy();
        let mut state = BackdriveState::new();
        on_backdrive_entry(&mut state);
        assert_eq!(state.max_duty, 0);
        assert!(state.needs_reset);

        // Accumulate to just before coast end
        update_backdrive_window(&mut state, &policy, policy.yield_coast_us - 1);
        assert_eq!(state.max_duty, 0);
        assert!(!should_exit_backdrive(&state, &policy));

        // Cross into alive phase
        update_backdrive_window(&mut state, &policy, 2);
        assert_eq!(state.max_duty, policy.yield_alive_duty_max.as_raw());
        assert!(!should_exit_backdrive(&state, &policy));
    }

    #[test]
    fn test_should_exit_backdrive() {
        let policy = test_policy();
        let mut state = BackdriveState::new();
        on_backdrive_entry(&mut state);

        // Accumulate to just before exit
        update_backdrive_window(&mut state, &policy, policy.yield_duration_us - 1);
        assert!(!should_exit_backdrive(&state, &policy));

        // Cross exit threshold
        update_backdrive_window(&mut state, &policy, 2);
        assert!(should_exit_backdrive(&state, &policy));
    }

    #[test]
    fn test_elapsed_clamps_to_duration() {
        let policy = test_policy();
        let mut state = BackdriveState {
            elapsed_us: policy.yield_duration_us - 10,
            max_duty: 0,
            needs_reset: false,
        };

        // Try to accumulate way past duration
        update_backdrive_window(&mut state, &policy, 1_000_000);
        assert_eq!(state.elapsed_us, policy.yield_duration_us);
    }
}
