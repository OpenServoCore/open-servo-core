//! Control loop traits for servo control algorithms.
//!
//! Sensor traits have been moved to `open-servo-hw::sensor`.

#[cfg(feature = "pid")]
use crate::PidConfig;
use open_servo_math::{CentiDeg, Duty, MilliAmp};

/// Control loop trait for position servo algorithms.
///
/// Implementations include PID, cascade control, velocity loops, etc.
pub trait ControlLoop {
    /// Compute control output based on sensor inputs.
    ///
    /// Returns PWM duty cycle command.
    fn compute(&mut self, setpoint: CentiDeg, position: CentiDeg, current: Option<MilliAmp>)
        -> Duty;

    /// Reset controller state (e.g., clear integral term).
    fn reset(&mut self);

    /// Update setpoint (in centidegrees for position control).
    fn set_setpoint(&mut self, setpoint: CentiDeg);

    /// Get current setpoint (in centidegrees for position control).
    fn get_setpoint(&self) -> CentiDeg;
    
    /// Set output limits dynamically (for torque limiting).
    /// 
    /// Used by the torque limiter to dynamically adjust PWM limits
    /// based on current measurements. Implementations should update
    /// their internal limits and adjust anti-windup accordingly.
    fn set_output_limits(&mut self, min: i32, max: i32);

    // =========================================================================
    // Optional PID config interface (for controllers that support it)
    // =========================================================================

    /// Get read-only access to PID config.
    /// Returns `None` if the controller doesn't have a PID config.
    #[cfg(feature = "pid")]
    fn pid_config(&self) -> Option<&PidConfig> {
        None
    }

    /// Mutate PID config and apply atomically.
    /// The closure runs, then `apply_config()` is called automatically.
    /// Default: no-op for non-PID controllers.
    #[cfg(feature = "pid")]
    fn with_pid_config_mut<F>(&mut self, _f: F)
    where
        F: FnOnce(&mut PidConfig),
    {
        // Default: no-op for non-PID controllers
    }
}
