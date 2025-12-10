//! Temperature sensor traits.
//!
//! Temperature sensing is split into distinct sources with different fault actions:
//! - MCU temp: full shutdown on over-temp (protect the controller)
//! - Motor temp: derate torque/PWM (protect the actuator)
//! - Driver temp: stop driving, MCU can still log/report

use open_servo_math::DeciC;

// ============================================================================
// MCU / PCB Temperature
// ============================================================================

/// MCU/PCB temperature sensing via internal sensor.
///
/// Used for over-temperature protection of the microcontroller.
/// Fault action: full shutdown (McuOverTemp).
pub trait McuTemperatureSensor {
    /// Read MCU temperature in 0.1°C (DeciC).
    ///
    /// Returns `None` if reading is not available (e.g., ADC not ready).
    fn read_mcu_temperature(&self) -> Option<DeciC>;

    /// Read raw MCU temperature ADC value.
    ///
    /// Returns `None` if reading is not available.
    fn read_mcu_temperature_raw(&self) -> Option<u16>;
}

/// Safety capability trait for MCU temperature sensing.
///
/// ALL boards implement this trait. Boards with MCU temp sensing get
/// automatic implementation via blanket impl. Boards without MCU temp
/// sensing implement this directly, returning `None`.
pub trait SafetyMcuTempSource {
    /// Read MCU temperature for safety checks.
    ///
    /// Returns `None` if the board has no MCU temp sensor,
    /// which causes SafetyManager to skip MCU over-temperature checks.
    fn read_safety_mcu_temp(&self) -> Option<DeciC>;
}

// Blanket impl: McuTemperatureSensor automatically provides SafetyMcuTempSource
impl<T: McuTemperatureSensor> SafetyMcuTempSource for T {
    #[inline]
    fn read_safety_mcu_temp(&self) -> Option<DeciC> {
        self.read_mcu_temperature()
    }
}

// ============================================================================
// Motor Winding Temperature
// ============================================================================

/// Motor winding temperature sensing via external NTC thermistor.
///
/// Used for over-temperature protection of the motor/actuator.
/// Fault action: derate torque/PWM (MotorOverTemp).
pub trait MotorTemperatureSensor {
    /// Read motor winding temperature in 0.1°C (DeciC).
    ///
    /// Returns `None` if reading is not available.
    fn read_motor_temperature(&self) -> Option<DeciC>;

    /// Read raw motor temperature ADC value.
    ///
    /// Returns `None` if reading is not available.
    fn read_motor_temperature_raw(&self) -> Option<u16>;
}

/// Safety capability trait for motor temperature sensing.
///
/// Boards with motor temp sensing get automatic implementation via blanket impl.
/// Boards without motor temp sensing implement this directly, returning `None`.
pub trait SafetyMotorTempSource {
    /// Read motor temperature for safety checks.
    ///
    /// Returns `None` if the board has no motor temp sensor.
    fn read_safety_motor_temp(&self) -> Option<DeciC>;
}

// Blanket impl: MotorTemperatureSensor automatically provides SafetyMotorTempSource
impl<T: MotorTemperatureSensor> SafetyMotorTempSource for T {
    #[inline]
    fn read_safety_motor_temp(&self) -> Option<DeciC> {
        self.read_motor_temperature()
    }
}

// ============================================================================
// Driver IC Temperature
// ============================================================================

/// Driver IC temperature sensing (if available from motor driver).
///
/// Used for over-temperature protection of the driver IC.
/// Fault action: stop driving, MCU can still log/report (DriverOverTemp).
pub trait DriverTemperatureSensor {
    /// Read driver IC temperature in 0.1°C (DeciC).
    ///
    /// Returns `None` if reading is not available.
    fn read_driver_temperature(&self) -> Option<DeciC>;
}

/// Safety capability trait for driver temperature sensing.
///
/// Boards with driver temp sensing get automatic implementation via blanket impl.
/// Boards without driver temp sensing implement this directly, returning `None`.
pub trait SafetyDriverTempSource {
    /// Read driver temperature for safety checks.
    ///
    /// Returns `None` if the board has no driver temp sensor.
    fn read_safety_driver_temp(&self) -> Option<DeciC>;
}

// Blanket impl: DriverTemperatureSensor automatically provides SafetyDriverTempSource
impl<T: DriverTemperatureSensor> SafetyDriverTempSource for T {
    #[inline]
    fn read_safety_driver_temp(&self) -> Option<DeciC> {
        self.read_driver_temperature()
    }
}
