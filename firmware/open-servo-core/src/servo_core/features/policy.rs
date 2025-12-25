//! Policy configuration for Move/Hold/Yield FSM behavior.
//!
//! All policy tuning comes from board config.

use open_servo_hw::BoardPolicyConfig;
use open_servo_math::{CentiDeg, DegPerSec10, Duty};

/// Policy configuration.
///
/// Wraps board-supplied FSM thresholds and tuning parameters.
#[derive(Debug, Clone, Copy)]
pub struct PolicyConfig {
    /// Hold mode entry/exit thresholds
    pub hold_enter_error: CentiDeg,
    pub hold_exit_error: CentiDeg,
    pub hold_enter_vel: DegPerSec10,
    pub hold_exit_vel: DegPerSec10,

    /// Backdrive/yield detection
    pub backdrive_vel_threshold: DegPerSec10,
    pub backdrive_deadband: Duty,
    pub backdrive_persist_us: u32,
    pub yield_alive_duty_max: Duty,
    pub yield_coast_us: u32,
    pub yield_duration_us: u32,

    /// Hold duty curve
    pub hold_duty_error_start: CentiDeg,
    pub hold_duty_error_end: CentiDeg,
    pub hold_duty_min: Duty,
    pub hold_duty_max: Duty,
}

impl PolicyConfig {
    /// Create from board-supplied config.
    pub fn from_board(board: &BoardPolicyConfig) -> Self {
        Self {
            hold_enter_error: board.hold_enter_error,
            hold_exit_error: board.hold_exit_error,
            hold_enter_vel: board.hold_enter_vel,
            hold_exit_vel: board.hold_exit_vel,
            backdrive_vel_threshold: board.backdrive_vel_threshold,
            backdrive_deadband: board.backdrive_deadband,
            backdrive_persist_us: board.backdrive_persist_us,
            yield_alive_duty_max: board.yield_alive_duty_max,
            yield_coast_us: board.yield_coast_us,
            yield_duration_us: board.yield_duration_us,
            hold_duty_error_start: board.hold_duty_error_start,
            hold_duty_error_end: board.hold_duty_error_end,
            hold_duty_min: board.hold_duty_min,
            hold_duty_max: board.hold_duty_max,
        }
    }
}
