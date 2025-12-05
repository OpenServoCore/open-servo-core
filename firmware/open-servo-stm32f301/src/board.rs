use stm32f3::stm32f301 as pac;

use open_servo_hw::{MotorDriver, UartDriver, SystemTime, UartPort};
use open_servo_control::{PositionSensor, CurrentSensor, VoltageSensor, TemperatureSensor};

use crate::hw_impl::Stm32f301Hw;

/// Board wrapper that implements all required traits
pub struct Board {
    hw: Stm32f301Hw,
}

impl Board {
    pub fn new() -> Self {
        Board {
            hw: Stm32f301Hw::new(),
        }
    }
    
    /// Initialize the board with all peripherals
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

// Implement all the required traits by delegating to the hardware implementation

impl MotorDriver for Board {
    fn set_pwm(&mut self, duty: i32) {
        self.hw.set_pwm(duty);
    }
    
    fn set_enable(&mut self, enabled: bool) {
        self.hw.set_enable(enabled);
    }
    
    fn brake(&mut self) {
        // Not implemented for basic H-bridge
        self.hw.set_pwm(0);
    }
}

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

#[cfg(feature = "potentiometer")]
impl PositionSensor for Board {
    fn read_position(&self) -> u16 {
        self.hw.position()
    }
}

#[cfg(feature = "current-sense")]
impl CurrentSensor for Board {
    fn read_current(&self) -> u16 {
        self.hw.phase_current()
    }
}

impl VoltageSensor for Board {
    fn read_voltage(&self) -> u16 {
        self.hw.bus_voltage()
    }
}

impl TemperatureSensor for Board {
    fn read_temperature(&self) -> Option<u16> {
        self.hw.temperature()
    }
}