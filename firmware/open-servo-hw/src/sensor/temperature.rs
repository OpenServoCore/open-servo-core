//! Temperature sensor traits.

use open_servo_math::DeciC;

/// Temperature sensing (MCU internal or external).
///
/// Used for over-temperature protection.
pub trait TemperatureSensor {
    /// Read temperature in 0.1°C (DeciC).
    ///
    /// Returns `None` if reading is not available (e.g., ADC not ready).
    fn read_temperature(&self) -> Option<DeciC>;

    /// Read raw temperature ADC value.
    ///
    /// Returns `None` if reading is not available.
    fn read_temperature_raw(&self) -> Option<u16>;
}

/// Safety capability trait for temperature sensing.
///
/// ALL boards implement this trait. Boards with temperature sensing get
/// automatic implementation via blanket impl. Boards without temperature
/// sensing implement this directly, returning `None`.
pub trait SafetyTemperatureSource {
    /// Read temperature for safety checks.
    ///
    /// Returns `None` if the board has no temperature sensor,
    /// which causes SafetyManager to skip over-temperature checks.
    fn read_safety_temperature(&self) -> Option<DeciC>;
}

// Blanket impl: TemperatureSensor automatically provides SafetyTemperatureSource
impl<T: TemperatureSensor> SafetyTemperatureSource for T {
    #[inline]
    fn read_safety_temperature(&self) -> Option<DeciC> {
        self.read_temperature()
    }
}

/// Motor winding temperature (separate from MCU temp).
///
/// For servos with dedicated motor thermistor.
pub trait MotorTemperatureSensor {
    /// Read motor winding temperature in 0.1°C.
    fn read_motor_temperature(&self) -> Option<DeciC>;
}
