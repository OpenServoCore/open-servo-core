//! Backdrive window timing and phase management.
//!
//! When backdrive is detected, the servo enters a timed window that allows
//! external forces to move the mechanism. The window has two phases:
//! - Coast (0-100ms): PWM duty capped at 0, letting the servo coast
//! - Alive (100-200ms): PWM duty capped at low value, maintaining some position

use crate::servo_core::ServoMode;

/// Backdrive window timing constants
pub const BACKDRIVE_DURATION_US: u32 = 200_000; // 200ms total window
pub const BACKDRIVE_COAST_US: u32 = 100_000; // 100ms coast phase
pub const BACKDRIVE_ALIVE_DUTY_MAX: i16 = 1638; // ~5% duty during alive phase

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
pub fn update_backdrive_window(state: &mut BackdriveState, dt_us: u32) {
    // Accumulate time, clamping to BACKDRIVE_DURATION_US
    state.elapsed_us = state
        .elapsed_us
        .saturating_add(dt_us)
        .min(BACKDRIVE_DURATION_US);

    // Set max_duty based on phase
    state.max_duty = if state.elapsed_us < BACKDRIVE_COAST_US {
        0 // Coast phase
    } else {
        BACKDRIVE_ALIVE_DUTY_MAX // Alive phase
    };
}

/// Get current backdrive phase from state.
pub fn get_backdrive_phase(state: &BackdriveState) -> BackdrivePhase {
    if state.elapsed_us < BACKDRIVE_COAST_US {
        BackdrivePhase::Coast
    } else {
        BackdrivePhase::Alive
    }
}

/// Check if backdrive window has completed and mode should exit.
///
/// Returns true when elapsed_us >= BACKDRIVE_DURATION_US.
pub fn should_exit_backdrive(state: &BackdriveState) -> bool {
    state.elapsed_us >= BACKDRIVE_DURATION_US
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

    #[test]
    fn test_backdrive_phase_coast() {
        let state = BackdriveState {
            elapsed_us: 0,
            max_duty: 0,
            needs_reset: false,
        };
        assert_eq!(get_backdrive_phase(&state), BackdrivePhase::Coast);
    }

    #[test]
    fn test_backdrive_phase_alive() {
        let state = BackdriveState {
            elapsed_us: BACKDRIVE_COAST_US,
            max_duty: BACKDRIVE_ALIVE_DUTY_MAX,
            needs_reset: false,
        };
        assert_eq!(get_backdrive_phase(&state), BackdrivePhase::Alive);
    }

    #[test]
    fn test_backdrive_timing_coast_to_alive() {
        let mut state = BackdriveState::new();
        on_backdrive_entry(&mut state);
        assert_eq!(state.max_duty, 0);
        assert!(state.needs_reset);

        // Accumulate to just before coast end
        update_backdrive_window(&mut state, BACKDRIVE_COAST_US - 1);
        assert_eq!(state.max_duty, 0);
        assert!(!should_exit_backdrive(&state));

        // Cross into alive phase
        update_backdrive_window(&mut state, 2);
        assert_eq!(state.max_duty, BACKDRIVE_ALIVE_DUTY_MAX);
        assert!(!should_exit_backdrive(&state));
    }

    #[test]
    fn test_should_exit_backdrive() {
        let mut state = BackdriveState::new();
        on_backdrive_entry(&mut state);

        // Accumulate to just before exit
        update_backdrive_window(&mut state, BACKDRIVE_DURATION_US - 1);
        assert!(!should_exit_backdrive(&state));

        // Cross exit threshold
        update_backdrive_window(&mut state, 2);
        assert!(should_exit_backdrive(&state));
    }

    #[test]
    fn test_elapsed_clamps_to_duration() {
        let mut state = BackdriveState {
            elapsed_us: BACKDRIVE_DURATION_US - 10,
            max_duty: 0,
            needs_reset: false,
        };

        // Try to accumulate way past duration
        update_backdrive_window(&mut state, 1_000_000);
        assert_eq!(state.elapsed_us, BACKDRIVE_DURATION_US);
    }
}
