//! Board wrapper that implements all required hardware traits.

use stm32f3::stm32f301 as pac;

use open_servo_hw::config::{
    BoardConfig, BoardKinematicsConfig, BoardPolicyConfig, BoardSafetyConfig, BoardThermalConfig,
    DxlIdentity, EepromDefaults,
};
use open_servo_hw::motor::BdcMotorDriver;
use open_servo_hw::peripheral::{SystemTime, UartDriver};
#[cfg(feature = "current-sense-bus")]
use open_servo_hw::sensor::BusCurrentSensor;
// Always import SafetyCurrentSource since we always implement it
use open_servo_hw::sensor::SafetyCurrentSource;
// Always import SafetyMcuTempSource since we always implement it
#[cfg(feature = "temp-sense-mcu")]
use open_servo_hw::sensor::McuTemperatureSensor;
#[cfg(feature = "temp-sense-motor")]
use open_servo_hw::sensor::MotorTemperatureSensor;
use open_servo_hw::sensor::PositionSensor;
use open_servo_hw::sensor::SafetyMcuTempSource;
use open_servo_hw::sensor::SafetyVoltageSource;
// Always import SafetyMotorTempSource since we always implement it
#[cfg(feature = "voltage-sense-motor")]
use open_servo_hw::sensor::MotorVoltageSensor;
use open_servo_hw::sensor::SafetyMotorTempSource;
// Always import SafetyMotorVoltageSource since we always implement it
use open_servo_hw::sensor::SafetyMotorVoltageSource;
use open_servo_hw::UartPort;
use open_servo_math::{CentiC, CentiDeg, ComplianceConfig, Effort, MilliAmp, MilliVolt};

use crate::config::BoardConfigProvider;
use crate::pwm::PwmController;
use crate::sensors::SensorReader;

/// Board wrapper that implements all required traits.
pub struct Board {
    sensors: SensorReader,
}

impl Board {
    pub fn new() -> Self {
        Board {
            sensors: SensorReader::new(),
        }
    }

    /// Initialize the board with all peripherals.
    pub fn init() -> Self {
        let p = pac::Peripherals::take().unwrap();

        // Initialize all peripherals
        crate::init::init_rcc(&p);
        crate::init::init_tim1_pwm(&p);
        crate::init::init_tim2_counter(&p);
        crate::init::init_dma(&p);
        crate::init::init_adc(&p);

        // Start ADC continuous conversion
        p.ADC1.cr.modify(|_, w| w.adstart().start_conversion());

        Board::new()
    }
}

// ============================================================================
// Motor driver trait
// ============================================================================

impl BdcMotorDriver for Board {
    fn set_effort(&mut self, effort: Effort) {
        PwmController::set_effort(effort);
    }

    fn set_enable(&mut self, _enabled: bool) {
        // No hardware enable pin on this board - motor driver is always enabled
    }

    fn coast(&mut self) {
        // High impedance - use PWM controller coast mode
        PwmController::coast();
    }

    fn brake(&mut self) {
        // Short motor terminals - use PWM controller brake mode
        PwmController::brake();
    }
}

// ============================================================================
// Peripheral traits
// ============================================================================

impl SystemTime for Board {
    fn now_us(&self) -> u32 {
        // Read TIM2 counter (configured to count microseconds)
        let tim2 = unsafe { &(*pac::TIM2::ptr()) };
        tim2.cnt.read().cnt().bits()
    }
}

impl UartDriver for Board {
    fn uart_write(&mut self, _port: UartPort, _data: &[u8]) {
        // Not implemented for V0
    }

    fn uart_read_byte(&mut self, _port: UartPort) -> Option<u8> {
        // Not implemented for V0
        None
    }
}

// ============================================================================
// Sensor traits
// ============================================================================

// Position sensor (required - always available)
impl PositionSensor for Board {
    fn read_position(&self) -> CentiDeg {
        self.sensors.position()
    }

    fn read_position_raw(&self) -> u16 {
        self.sensors.position_raw()
    }
}

// Current sensor (optional based on feature)
#[cfg(feature = "current-sense-bus")]
impl BusCurrentSensor for Board {
    fn read_bus_current(&self) -> MilliAmp {
        self.sensors.current()
    }

    fn read_bus_current_raw(&self) -> u16 {
        self.sensors.current_raw()
    }
}

// When current-sense feature is enabled, explicitly implement SafetyCurrentSource
#[cfg(feature = "current-sense-bus")]
impl SafetyCurrentSource for Board {
    fn read_safety_current(&self) -> Option<MilliAmp> {
        Some(self.read_bus_current())
    }
}

// When current-sense feature is disabled, implement SafetyCurrentSource to return None
#[cfg(not(feature = "current-sense-bus"))]
impl SafetyCurrentSource for Board {
    fn read_safety_current(&self) -> Option<MilliAmp> {
        None // No current sensor on this board variant
    }
}

// This board does not have bus voltage sensing (only VDDA measurement)
impl SafetyVoltageSource for Board {
    fn read_safety_voltage(&self) -> Option<MilliVolt> {
        None
    }
}

// MCU Temperature sensor (optional based on feature)
#[cfg(feature = "temp-sense-mcu")]
impl McuTemperatureSensor for Board {
    fn read_mcu_temperature(&self) -> Option<CentiC> {
        self.sensors.mcu_temperature()
    }

    fn read_mcu_temperature_raw(&self) -> Option<u16> {
        self.sensors.mcu_temperature_raw()
    }
}

// When temp-sense-mcu feature is enabled, explicitly implement SafetyMcuTempSource
#[cfg(feature = "temp-sense-mcu")]
impl SafetyMcuTempSource for Board {
    fn read_safety_mcu_temp(&self) -> Option<CentiC> {
        self.read_mcu_temperature()
    }
}

// When temp-sense-mcu feature is disabled, implement SafetyMcuTempSource to return None
#[cfg(not(feature = "temp-sense-mcu"))]
impl SafetyMcuTempSource for Board {
    fn read_safety_mcu_temp(&self) -> Option<CentiC> {
        None // No MCU temperature sensor on this board variant
    }
}

// Motor voltage sensing (optional based on feature)
#[cfg(feature = "voltage-sense-motor")]
impl MotorVoltageSensor for Board {
    fn read_motor_voltage(&self) -> (MilliVolt, MilliVolt) {
        self.sensors.motor_voltage()
    }

    fn read_motor_voltage_raw(&self) -> (u16, u16) {
        self.sensors.motor_voltage_raw()
    }
}

// When voltage-sense-motor feature is enabled, explicitly implement SafetyMotorVoltageSource
#[cfg(feature = "voltage-sense-motor")]
impl SafetyMotorVoltageSource for Board {
    fn read_safety_motor_voltage(&self) -> Option<(MilliVolt, MilliVolt)> {
        Some(self.read_motor_voltage())
    }
}

// When voltage-sense-motor feature is disabled, implement SafetyMotorVoltageSource directly
#[cfg(not(feature = "voltage-sense-motor"))]
impl SafetyMotorVoltageSource for Board {
    fn read_safety_motor_voltage(&self) -> Option<(MilliVolt, MilliVolt)> {
        None // No motor voltage sensor on this board variant
    }
}

// Motor temperature sensing (optional based on feature)
#[cfg(feature = "temp-sense-motor")]
impl MotorTemperatureSensor for Board {
    fn read_motor_temperature(&self) -> Option<CentiC> {
        Some(self.sensors.motor_temperature())
    }

    fn read_motor_temperature_raw(&self) -> Option<u16> {
        Some(self.sensors.motor_temperature_raw())
    }
}

// When temp-sense-motor feature is enabled, explicitly implement SafetyMotorTempSource
#[cfg(feature = "temp-sense-motor")]
impl SafetyMotorTempSource for Board {
    fn read_safety_motor_temp(&self) -> Option<CentiC> {
        self.read_motor_temperature()
    }
}

// When temp-sense-motor feature is disabled, implement SafetyMotorTempSource directly
#[cfg(not(feature = "temp-sense-motor"))]
impl SafetyMotorTempSource for Board {
    fn read_safety_motor_temp(&self) -> Option<CentiC> {
        None // No motor temperature sensor on this board variant
    }
}

// ============================================================================
// Board configuration provider
// ============================================================================

impl BoardConfig for Board {
    fn dxl_identity(&self) -> DxlIdentity {
        BoardConfigProvider::dxl_identity()
    }

    fn eeprom_defaults(&self) -> EepromDefaults {
        BoardConfigProvider::eeprom_defaults()
    }

    fn safety_config(&self) -> BoardSafetyConfig {
        BoardConfigProvider::safety_config()
    }

    fn move_compliance_config(&self) -> ComplianceConfig {
        BoardConfigProvider::move_compliance_config()
    }

    fn hold_compliance_config(&self) -> ComplianceConfig {
        BoardConfigProvider::hold_compliance_config()
    }

    fn thermal_config(&self) -> BoardThermalConfig {
        BoardConfigProvider::thermal_config()
    }

    fn kinematics_config(&self) -> BoardKinematicsConfig {
        BoardConfigProvider::kinematics_config()
    }

    fn policy_config(&self) -> BoardPolicyConfig {
        BoardConfigProvider::policy_config()
    }

    fn pid_gains(&self) -> (i32, i32, i32) {
        BoardConfigProvider::pid_gains()
    }
}
