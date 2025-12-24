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

/// Secondary position sensor for sensor fusion (optional).
///
/// Example: encoder + potentiometer for absolute + incremental.
pub trait SecondaryPositionSensor {
    /// Read secondary position in centidegrees.
    fn read_secondary_position(&self) -> CentiDeg;

    /// Read raw secondary position value.
    fn read_secondary_position_raw(&self) -> u16;
}
