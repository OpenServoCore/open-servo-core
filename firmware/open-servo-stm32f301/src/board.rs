//! Board wrapper that implements all required hardware traits.

use stm32f3::stm32f301 as pac;

use open_servo_hw::motor::BdcMotorDriver;
use open_servo_hw::peripheral::{SystemTime, UartDriver};
#[cfg(not(feature = "current-sense"))]
use open_servo_hw::sensor::SafetyCurrentSource;
use open_servo_hw::sensor::{
    BusCurrentSensor, BusVoltageSensor, PositionSensor, TemperatureSensor,
};
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
        crate::init::init_gpio(&p);
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

// Position sensor (always available via potentiometer or encoder)
#[cfg(feature = "potentiometer")]
impl PositionSensor for Board {
    fn read_position(&self) -> CentiDeg {
        self.hw.position()
    }

    fn read_position_raw(&self) -> u16 {
        self.hw.position_raw()
    }
}

// Current sensor (optional based on feature)
#[cfg(feature = "current-sense")]
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
#[cfg(not(feature = "current-sense"))]
impl SafetyCurrentSource for Board {
    fn read_safety_current(&self) -> Option<MilliAmp> {
        None // No current sensor on this board variant
    }
}

// Voltage sensor (always available via bus voltage divider)
impl BusVoltageSensor for Board {
    fn read_bus_voltage(&self) -> MilliVolt {
        self.hw.voltage()
    }

    fn read_bus_voltage_raw(&self) -> u16 {
        self.hw.voltage_raw()
    }
}

// Note: SafetyVoltageSource is auto-implemented via blanket impl since we implement BusVoltageSensor

// Temperature sensor (always available via MCU internal sensor)
impl TemperatureSensor for Board {
    fn read_temperature(&self) -> Option<DeciC> {
        self.hw.temperature()
    }

    fn read_temperature_raw(&self) -> Option<u16> {
        self.hw.temperature_raw()
    }
}

// Note: SafetyTemperatureSource is auto-implemented via blanket impl since we implement TemperatureSensor
