//! Control loop traits for servo control algorithms.
//!
//! Stage 1 refactor: Minimal trait with ControlInput/ControlOutput.
//! - Controllers own only algorithm state (integrators, filters, etc.)
//! - ServoCore owns setpoint, engagement, mode, safety state
//! - Limits passed via ControlInput, saturation returned via ControlOutput

#[cfg(feature = "pid")]
use crate::PidConfig;
use open_servo_math::{CentiDeg, DegPerSec10, Duty, MilliAmp, MilliVolt};

// =============================================================================
// Control I/O Types
// =============================================================================

/// Dynamic duty limits for a single tick.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DutyLimits {
    pub min: Duty,
    pub max: Duty,
}

impl DutyLimits {
    /// Create new duty limits.
    pub fn new(min: Duty, max: Duty) -> Self {
        Self { min, max }
    }

    /// Full range limits (no restriction).
    pub const fn full() -> Self {
        Self {
            min: Duty::MIN,
            max: Duty::MAX,
        }
    }
}

/// Inputs to the control loop for a single tick.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ControlInput {
    /// Target position (required - core only calls controller when engaged)
    pub setpoint: CentiDeg,
    /// Current measured position
    pub position: CentiDeg,
    /// Measured velocity (if available)
    pub velocity: Option<DegPerSec10>,
    /// Measured motor current (if available)
    pub current: Option<MilliAmp>,
    /// Bus voltage (if available)
    pub bus_voltage: Option<MilliVolt>,
    /// Dynamic duty limits for this tick
    pub limits: DutyLimits,
}

/// Output from the control loop.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ControlOutput {
    /// Computed duty cycle
    pub duty: Duty,
    /// True if duty was clamped to limits (for anti-windup / stall detection)
    pub saturated: bool,
}

impl ControlOutput {
    /// Create a zero output (stopped, not saturated).
    pub const fn zero() -> Self {
        Self {
            duty: Duty::ZERO,
            saturated: false,
        }
    }
}

// =============================================================================
// ControlLoop Trait
// =============================================================================

/// Minimal control loop trait for position servo algorithms.
///
/// Controllers implement pure control math:
/// - Own only algorithm state (integrators, filters, etc.)
/// - Receive setpoint/limits via ControlInput (no internal storage)
/// - Return saturated flag for stall detection
///
/// ServoCore owns:
/// - Setpoint (Option<CentiDeg>)
/// - Engagement state
/// - Compliance mode
/// - Safety/fault state
pub trait ControlLoop {
    /// Reset controller state (integrators, filters, derivative history).
    fn reset(&mut self);

    /// Fast tick (ControlFast rate) - compute control output.
    ///
    /// Implementations must:
    /// - Compute duty from input.setpoint and input.position
    /// - Clamp output to input.limits
    /// - Set saturated=true if clamped at min or max limit
    fn fast_tick(&mut self, input: &ControlInput) -> ControlOutput;

    /// Medium tick (~1kHz) - optional intermediate processing.
    fn medium_tick(&mut self, input: &ControlInput);

    /// Slow tick (~100Hz) - optional slow processing.
    fn slow_tick(&mut self, input: &ControlInput);
}

// =============================================================================
// PidTunable Trait (separate from ControlLoop)
// =============================================================================

/// Trait for controllers that expose PID tuning.
///
/// Separated from ControlLoop so that:
/// - Non-PID controllers don't need stub implementations
/// - Debug shell can require PidTunable only for PID commands
#[cfg(feature = "pid")]
pub trait PidTunable {
    /// Get read-only access to PID config.
    fn pid_config(&self) -> &PidConfig;

    /// Mutate PID config and apply atomically.
    fn with_pid_config_mut<F: FnOnce(&mut PidConfig)>(&mut self, f: F);
}
