//! Board wrapper that implements all required hardware traits.

use stm32f3::stm32f301 as pac;

use open_servo_hw::motor::BdcMotorDriver;
use open_servo_hw::peripheral::{SystemTime, UartDriver};
#[cfg(feature = "current-sense-bus")]
use open_servo_hw::sensor::BusCurrentSensor;
#[cfg(not(feature = "current-sense-bus"))]
use open_servo_hw::sensor::SafetyCurrentSource;
#[cfg(not(feature = "temp-sense-mcu"))]
use open_servo_hw::sensor::SafetyMcuTempSource;
#[cfg(feature = "temp-sense-mcu")]
use open_servo_hw::sensor::McuTemperatureSensor;
use open_servo_hw::sensor::PositionSensor;
use open_servo_hw::sensor::SafetyVoltageSource;
#[cfg(feature = "temp-sense-motor")]
use open_servo_hw::sensor::MotorTemperatureSensor;
#[cfg(not(feature = "temp-sense-motor"))]
use open_servo_hw::sensor::SafetyMotorTempSource;
#[cfg(feature = "voltage-sense-motor")]
use open_servo_hw::sensor::MotorVoltageSensor;
#[cfg(not(feature = "voltage-sense-motor"))]
use open_servo_hw::sensor::SafetyMotorVoltageSource;
use open_servo_hw::UartPort;
use open_servo_math::{CentiDeg, DeciC, MilliAmp, MilliVolt};

use crate::hw_impl::Stm32f301Hw;

/// Board wrapper that implements all required traits.
///
/// This delegates to the underlying Stm32f301Hw for actual hardware access.
pub struct Board {
    hw: Stm32f301Hw,
}

impl Board {
    pub fn new() -> Self {
        Board {
            hw: Stm32f301Hw::new(),
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
    fn set_pwm(&mut self, duty: i32) {
        self.hw.set_pwm(duty);
    }

    fn set_enable(&mut self, enabled: bool) {
        self.hw.set_enable(enabled);
    }

    fn coast(&mut self) {
        // High impedance - disable driver
        self.hw.set_enable(false);
    }

    fn brake(&mut self) {
        // Short motor terminals - set PWM to 0 with driver enabled
        self.hw.set_pwm(0);
        self.hw.set_enable(true);
    }
}

// ============================================================================
// Peripheral traits
// ============================================================================

impl SystemTime for Board {
    fn now_us(&self) -> u32 {
        self.hw.now_us()
    }
}

impl UartDriver for Board {
    fn uart_write(&mut self, port: UartPort, data: &[u8]) {
        self.hw.uart_write(port, data);
    }

    fn uart_read_byte(&mut self, port: UartPort) -> Option<u8> {
        self.hw.uart_read_byte(port)
    }
}

// ============================================================================
// Sensor traits
// ============================================================================

// Position sensor (required - always available)
impl PositionSensor for Board {
    fn read_position(&self) -> CentiDeg {
        self.hw.position()
    }

    fn read_position_raw(&self) -> u16 {
        self.hw.position_raw()
    }
}

// Current sensor (optional based on feature)
#[cfg(feature = "current-sense-bus")]
impl BusCurrentSensor for Board {
    fn read_bus_current(&self) -> MilliAmp {
        self.hw.current()
    }

    fn read_bus_current_raw(&self) -> u16 {
        self.hw.current_raw()
    }
}

// When current-sense feature is disabled, we need to explicitly implement
// SafetyCurrentSource to return None
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
    fn read_mcu_temperature(&self) -> Option<DeciC> {
        self.hw.mcu_temperature()
    }

    fn read_mcu_temperature_raw(&self) -> Option<u16> {
        self.hw.mcu_temperature_raw()
    }
}

// When temp-sense-mcu feature is disabled, we need to explicitly implement
// SafetyMcuTempSource to return None
#[cfg(not(feature = "temp-sense-mcu"))]
impl SafetyMcuTempSource for Board {
    fn read_safety_mcu_temp(&self) -> Option<DeciC> {
        None // No MCU temperature sensor on this board variant
    }
}

// Note: When temp-sense-mcu is enabled, SafetyMcuTempSource is auto-implemented via blanket impl

// Motor voltage sensing (optional based on feature)
#[cfg(feature = "voltage-sense-motor")]
impl MotorVoltageSensor for Board {
    fn read_motor_voltage(&self) -> (MilliVolt, MilliVolt) {
        self.hw.motor_voltage()
    }

    fn read_motor_voltage_raw(&self) -> (u16, u16) {
        self.hw.motor_voltage_raw()
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
    fn read_motor_temperature(&self) -> Option<DeciC> {
        Some(self.hw.motor_temperature())
    }

    fn read_motor_temperature_raw(&self) -> Option<u16> {
        Some(self.hw.motor_temperature_raw())
    }
}

// When temp-sense-motor feature is disabled, implement SafetyMotorTempSource directly
#[cfg(not(feature = "temp-sense-motor"))]
impl SafetyMotorTempSource for Board {
    fn read_safety_motor_temp(&self) -> Option<DeciC> {
        None // No motor temperature sensor on this board variant
    }
}
