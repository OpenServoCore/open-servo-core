//! Brushed DC motor driver traits.

/// Brushed DC motor driver control (H-bridge).
///
/// Implementations control motor direction and speed via PWM.
pub trait BdcMotorDriver {
    /// Set PWM duty cycle (-max to +max).
    ///
    /// Positive = forward, Negative = reverse.
    fn set_pwm(&mut self, duty: i32);

    /// Enable/disable motor driver.
    fn set_enable(&mut self, enabled: bool);

    /// Coast mode - high impedance, motor freewheels.
    fn coast(&mut self);

    /// Apply brake (short motor terminals).
    fn brake(&mut self);
}
