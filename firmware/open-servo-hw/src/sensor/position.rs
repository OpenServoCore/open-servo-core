//! Position sensor traits.

use open_servo_math::{Adc12, CentiDeg};

/// Primary position feedback sensor (required for all servos).
///
/// Implementations include potentiometers, magnetic encoders, etc.
pub trait PositionSensor {
    /// Read position in centidegrees.
    fn read_position(&self) -> CentiDeg;

    /// Read raw position value (for diagnostics).
    fn read_position_raw(&self) -> u16;
}

/// Default implementation helper for potentiometer-based position sensors.
///
/// Maps 0-4095 ADC to -500 to 18500 centidegrees (-5° to 185°).
#[inline]
pub fn pot_adc_to_position(raw: u16) -> CentiDeg {
    CentiDeg::from_pot_adc(Adc12::from_raw(raw))
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
