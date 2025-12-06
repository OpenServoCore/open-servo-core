#![allow(dead_code)]

use crate::types::UartPort;

/// Brushed DC motor driver control (H-bridge)
pub trait BdcMotorDriver {
    /// Set PWM duty cycle (-max to +max)
    /// Positive = forward, Negative = reverse
    fn set_pwm(&mut self, duty: i32);
    
    /// Enable/disable motor driver
    fn set_enable(&mut self, enabled: bool);
    
    /// Coast mode - high impedance, motor freewheels
    fn coast(&mut self);
    
    /// Apply brake (short motor terminals)
    fn brake(&mut self);
}

/// BLDC motor driver control (3-phase)
pub trait BldcMotorDriver {
    /// Set PWM duty cycles for 3 phases (0-100%)
    fn set_phase_pwm(&mut self, phase_a: u16, phase_b: u16, phase_c: u16);
    
    /// Set commutation state for trapezoidal control (6-step)
    /// State: 0-5 for the 6 commutation steps, None for high-Z
    fn set_commutation_step(&mut self, step: Option<u8>);
    
    /// Enable/disable driver (all phases)
    fn set_enable(&mut self, enabled: bool);
    
    /// Coast mode - all phases high-impedance
    fn coast(&mut self);
    
    /// Active braking (short low-side FETs)
    fn brake(&mut self);
    
    /// Get hall sensor state (if available)
    fn read_hall_sensors(&self) -> Option<u8>;
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