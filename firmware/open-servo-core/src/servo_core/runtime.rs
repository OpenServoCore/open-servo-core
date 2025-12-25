//! CoreRuntime - Read-only telemetry snapshot (replaces SystemState).

#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;
use open_servo_math::{CentiC, CentiDeg, DegPerSec10, Duty, MilliVolt};

use super::ServoMode;

/// Read-only telemetry snapshot for external consumers.
///
/// This is the canonical runtime state exposed to telemetry, REPL, and external APIs.
/// All fields are public for read access; mutation happens internally via tick methods.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CoreRuntime {
    /// Current setpoint
    pub setpoint: CentiDeg,
    /// Current position feedback
    pub position: CentiDeg,
    /// Current PWM duty cycle
    pub pwm_duty: Duty,
    /// Current reading (requires `current-sense-bus` feature)
    #[cfg(feature = "current-sense-bus")]
    pub current: Option<MilliAmp>,
    /// Bus voltage reading (if available)
    pub bus_voltage: Option<MilliVolt>,
    /// Temperature reading (if available)
    pub temperature: Option<CentiC>,
    /// Current compliance mode
    pub mode: ServoMode,
    /// Measured velocity
    pub velocity: DegPerSec10,
    /// Compliance is currently being limited
    pub compliance_limited: bool,
    /// Fast tick sequence counter (telemetry/debug only)
    pub fast_seq: u32,
    /// Motor engaged state
    pub engaged: bool,
}
