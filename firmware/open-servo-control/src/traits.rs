#![allow(dead_code)]

use crate::units::*;

/// Sensor traits that boards must implement

pub trait PositionSensor {
    /// Read raw position ADC value (0-4095 for 12-bit ADC)
    fn read_position_raw(&self) -> u16;
    
    /// Read position in centidegrees
    /// Default implementation: maps 0-4095 to -500 to 18500 (-5° to 185°)
    fn read_position(&self) -> CentiDeg {
        let raw = self.read_position_raw();
        CentiDeg::from_pot_adc(Adc12::from_raw(raw))
    }
}

pub trait CurrentSensor {
    /// Read raw current ADC value
    fn read_current_raw(&self) -> u16;
    
    /// Read current in milliamps
    fn read_current(&self) -> MilliAmp;
}

pub trait VelocitySensor {
    /// Read angular velocity in 0.1 deg/sec
    fn read_velocity(&self) -> DegPerSec10;
}

pub trait TemperatureSensor {
    /// Read raw temperature ADC value
    fn read_temperature_raw(&self) -> Option<u16>;
    
    /// Read temperature in 0.1°C
    fn read_temperature(&self) -> Option<DeciC>;
}

pub trait VoltageSensor {
    /// Read raw voltage ADC value  
    fn read_voltage_raw(&self) -> u16;
    
    /// Read bus voltage in millivolts
    fn read_voltage(&self) -> MilliVolt;
}

/// Control loop trait
pub trait ControlLoop {
    /// Compute control output based on sensor inputs
    /// Returns PWM duty cycle command
    fn compute(&mut self, setpoint: CentiDeg, position: CentiDeg, current: Option<MilliAmp>) -> i32;
    
    /// Reset controller state (e.g., clear integral term)
    fn reset(&mut self);
    
    /// Update setpoint (in centidegrees for position control)
    fn set_setpoint(&mut self, setpoint: CentiDeg);
    
    /// Get current setpoint (in centidegrees for position control)
    fn get_setpoint(&self) -> CentiDeg;
}