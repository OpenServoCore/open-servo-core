//! Voltage sensor traits.

use open_servo_math::MilliVolt;

/// Bus voltage monitoring.
///
/// Used for under-voltage protection and back-EMF compensation.
pub trait BusVoltageSensor {
    /// Read bus voltage in millivolts.
    fn read_bus_voltage(&self) -> MilliVolt;

    /// Read raw voltage ADC value.
    fn read_bus_voltage_raw(&self) -> u16;
}

/// Safety capability trait for voltage sensing.
///
/// ALL boards implement this trait. Boards with voltage sensing get
/// automatic implementation via blanket impl. Boards without voltage
/// sensing implement this directly, returning `None`.
pub trait SafetyVoltageSource {
    /// Read voltage for safety checks.
    ///
    /// Returns `None` if the board has no voltage sensor,
    /// which causes SafetyManager to skip under-voltage checks.
    fn read_safety_voltage(&self) -> Option<MilliVolt>;
}

// Blanket impl: BusVoltageSensor automatically provides SafetyVoltageSource
impl<T: BusVoltageSensor> SafetyVoltageSource for T {
    #[inline]
    fn read_safety_voltage(&self) -> Option<MilliVolt> {
        Some(self.read_bus_voltage())
    }
}
