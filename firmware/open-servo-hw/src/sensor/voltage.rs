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

// ============================================================================
// Motor Terminal Voltage
// ============================================================================

/// Motor terminal voltage sensing (BDC motors).
///
/// Measures voltage on both motor terminals via voltage dividers.
/// Returns both V+ and V- terminal voltages for flexibility in
/// computing differential voltage, back-EMF, etc.
///
/// Useful for:
/// - Back-EMF estimation
/// - Motor voltage monitoring
/// - Detecting motor stall vs free-running
pub trait MotorVoltageSensor {
    /// Read both motor terminal voltages in millivolts.
    ///
    /// Returns (V+ millivolts, V- millivolts).
    fn read_motor_voltage(&self) -> (MilliVolt, MilliVolt);

    /// Read raw ADC values for both motor terminals.
    ///
    /// Returns (V+ raw, V- raw) for custom processing.
    fn read_motor_voltage_raw(&self) -> (u16, u16);
}

/// Safety capability trait for motor voltage sensing.
///
/// Boards with motor voltage sensing get automatic implementation via
/// blanket impl. Boards without implement this directly, returning `None`.
pub trait SafetyMotorVoltageSource {
    /// Read motor terminal voltages for safety/diagnostic checks.
    ///
    /// Returns `None` if the board has no motor voltage sensor.
    /// Returns `Some((V+, V-))` with both terminal voltages.
    fn read_safety_motor_voltage(&self) -> Option<(MilliVolt, MilliVolt)>;
}

// Blanket impl: MotorVoltageSensor automatically provides SafetyMotorVoltageSource
impl<T: MotorVoltageSensor> SafetyMotorVoltageSource for T {
    #[inline]
    fn read_safety_motor_voltage(&self) -> Option<(MilliVolt, MilliVolt)> {
        Some(self.read_motor_voltage())
    }
}
