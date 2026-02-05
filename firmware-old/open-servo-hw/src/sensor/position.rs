//! Position sensor traits.

use open_servo_math::CentiDeg;

/// Primary position feedback sensor (required for all servos).
///
/// Implementations include potentiometers, magnetic encoders, etc.
pub trait PositionSensor {
    /// Read position in centidegrees.
    fn read_position(&self) -> CentiDeg;

    /// Read raw position value (for diagnostics).
    fn read_position_raw(&self) -> u16;
}

///Motor position sensor for sensor fusion (optional).
///
/// Example: encoder + potentiometer for absolute + incremental.
pub trait MotorPositionSensor {
    /// Read motor position in centidegrees.
    fn read_motor_position(&self) -> CentiDeg;

    /// Read raw motor position value.
    fn read_motor_position_raw(&self) -> u16;
}
