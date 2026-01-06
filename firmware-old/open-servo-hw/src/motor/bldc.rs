//! Brushless DC motor driver traits.

/// BLDC motor driver control (3-phase).
///
/// Supports both trapezoidal (6-step) and sinusoidal/FOC control.
pub trait BldcMotorDriver {
    /// Set PWM duty cycles for 3 phases (0-100% scaled).
    fn set_phase_pwm(&mut self, phase_a: u16, phase_b: u16, phase_c: u16);

    /// Set commutation state for trapezoidal control (6-step).
    ///
    /// State: 0-5 for the 6 commutation steps, None for high-Z.
    fn set_commutation_step(&mut self, step: Option<u8>);

    /// Enable/disable driver (all phases).
    fn set_enable(&mut self, enabled: bool);

    /// Coast mode - all phases high-impedance.
    fn coast(&mut self);

    /// Active braking (short low-side FETs).
    fn brake(&mut self);

    /// Get hall sensor state (if available).
    ///
    /// Returns 3-bit hall state or None if no hall sensors.
    fn read_hall_sensors(&self) -> Option<u8>;
}
