//! Brushed DC motor driver traits.

use open_servo_math::Duty;

/// Brushed DC motor driver control (H-bridge).
///
/// Implementations control motor direction and speed via PWM.
pub trait BdcMotorDriver {
    /// Set PWM duty cycle (normalized).
    ///
    /// Positive = forward, Negative = reverse, Zero = stopped.
    fn set_pwm(&mut self, duty: Duty);

    /// Enable/disable motor driver.
    fn set_enable(&mut self, enabled: bool);

    /// Coast mode - high impedance, motor freewheels.
    fn coast(&mut self);

    /// Apply brake (short motor terminals).
    fn brake(&mut self);
}
