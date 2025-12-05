#![allow(dead_code)]

/// Sensor traits that boards must implement

pub trait PositionSensor {
    /// Read position sensor (0-4095 for 12-bit ADC)
    fn read_position(&self) -> u16;
}

pub trait CurrentSensor {
    /// Read current sensor in milliamps
    fn read_current(&self) -> u16;
}

pub trait VelocitySensor {
    /// Read velocity in units/sec
    fn read_velocity(&self) -> i16;
}

pub trait TemperatureSensor {
    /// Read temperature in decikelvin
    fn read_temperature(&self) -> Option<u16>;
}

pub trait VoltageSensor {
    /// Read bus voltage in millivolts
    fn read_voltage(&self) -> u16;
}

/// Control loop trait
pub trait ControlLoop {
    /// Compute control output based on sensor inputs
    /// Returns PWM duty cycle command
    fn compute(&mut self, setpoint: u16, position: u16, current: Option<u16>) -> i32;
    
    /// Reset controller state (e.g., clear integral term)
    fn reset(&mut self);
    
    /// Update setpoint
    fn set_setpoint(&mut self, setpoint: u16);
    
    /// Get current setpoint
    fn get_setpoint(&self) -> u16;
}