#![allow(dead_code)]

use crate::types::UartPort;

/// Motor driver control
pub trait MotorDriver {
    /// Set PWM duty cycle (-max to +max)
    /// Positive = forward, Negative = reverse
    fn set_pwm(&mut self, duty: i32);
    
    /// Enable/disable motor driver
    fn set_enable(&mut self, enabled: bool);
    
    /// Apply brake (if supported)
    fn brake(&mut self);
}

/// UART communication
pub trait UartDriver {
    /// Write bytes to UART
    fn uart_write(&mut self, port: UartPort, data: &[u8]);
    
    /// Read a byte from UART (non-blocking)
    fn uart_read_byte(&mut self, port: UartPort) -> Option<u8>;
}

/// System time
pub trait SystemTime {
    /// Get current time in microseconds
    fn now_us(&self) -> u32;
}