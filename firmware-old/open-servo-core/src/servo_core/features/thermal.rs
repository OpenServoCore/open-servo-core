//! Thermal model and motor overtemperature protection.
//!
//! The thermal model estimates motor winding temperature from current measurements
//! using a first-order thermal model. This runs during slow tick (100Hz).

use crate::fault::FaultKind;
use crate::safety::ThermalFaultDetector;
use open_servo_math::{CentiC, MilliAmp, ThermalModel};

/// Thermal configuration.
///
/// Lives in CoreConfig.thermal
#[derive(Debug, Clone)]
pub struct ThermalConfig {
    /// Maximum motor temperature in centidegrees before fault
    pub max_temp_cdeg: i16,
    /// Hysteresis for temperature fault reset in centidegrees
    pub hysteresis_cdeg: i16,
    /// Default ambient temperature if no sensor (centidegrees)
    pub default_ambient_cdeg: i16,
}

impl ThermalConfig {
    /// Create a new ThermalConfig with all parameters.
    pub fn new(max_temp_cdeg: i16, hysteresis_cdeg: i16, default_ambient_cdeg: i16) -> Self {
        Self {
            max_temp_cdeg,
            hysteresis_cdeg,
            default_ambient_cdeg,
        }
    }
}

/// Mutable thermal state.
///
/// Lives in CoreInternal.thermal
#[derive(Debug, Clone)]
pub struct ThermalState {
    /// The thermal physics model
    pub model: ThermalModel,
    /// Fault detector with hysteresis
    pub fault_detector: ThermalFaultDetector,
    /// Cached temperature from last reading
    pub last_temperature: Option<CentiC>,
}

impl ThermalState {
    /// Create a new ThermalState with the given thermal model and config.
    pub fn new(model: ThermalModel, config: &ThermalConfig) -> Self {
        Self {
            model,
            fault_detector: ThermalFaultDetector::new(config.max_temp_cdeg, config.hysteresis_cdeg),
            last_temperature: None,
        }
    }

    /// Update cached temperature (called during fast tick).
    #[inline]
    pub fn update_temperature(&mut self, temp: Option<CentiC>) {
        self.last_temperature = temp;
    }
}

/// Accumulate I² for thermal model (fast tick - 10kHz).
///
/// Called every fast tick with current measurement.
/// The thermal model internally accumulates I² for averaging.
#[inline]
pub fn accumulate_i_squared(state: &mut ThermalState, current: Option<MilliAmp>) {
    state.model.update_fast(current.map(|c| c.as_ma()));
}

/// Update thermal model physics (slow tick).
///
/// Uses MCU temperature as ambient estimate, or default if no sensor.
/// dt_us is the time since last call (from TickCtx).
pub fn update_slow(state: &mut ThermalState, config: &ThermalConfig, dt_us: u32) {
    let ambient_cdeg = state
        .last_temperature
        .map(|t| t.as_centi_c())
        .unwrap_or(config.default_ambient_cdeg);

    state.model.update_slow(ambient_cdeg, dt_us);
}

/// Check if motor has exceeded safe temperature.
///
/// Returns `Some(FaultKind::MotorOverTemp)` if motor temperature exceeds limit.
#[inline]
pub fn check_motor_temp(state: &mut ThermalState) -> Option<FaultKind> {
    let temp = state.model.temperature_cdeg();
    state.fault_detector.check_fault(temp)
}

/// Try to reset thermal fault.
///
/// Only succeeds if temperature is below (max - hysteresis).
pub fn try_reset_fault(state: &mut ThermalState) {
    let temp = state.model.temperature_cdeg();
    state.fault_detector.try_reset(temp);
}

/// Get motor temperature in degrees (for debug display).
pub fn motor_temp_deg(state: &ThermalState) -> i16 {
    state.model.temperature_deg()
}

/// Get motor temperature rise in degrees (for debug display).
pub fn motor_temp_rise_deg(state: &ThermalState) -> i16 {
    state.model.temp_rise_deg()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test config: 100°C max temp, 10°C hysteresis, 25°C default ambient
    fn test_thermal_config() -> ThermalConfig {
        ThermalConfig::new(10000, 1000, 2500)
    }

    fn make_thermal_state() -> ThermalState {
        // Reasonable defaults for testing:
        // - 100mΩ winding resistance
        // - 10 °C/W thermal resistance
        // - 5 J/°C thermal capacity
        let model = ThermalModel::new(100, 1000, 500);
        let config = test_thermal_config();
        ThermalState::new(model, &config)
    }

    #[test]
    fn test_accumulate_i_squared_none() {
        let mut state = make_thermal_state();
        accumulate_i_squared(&mut state, None);
        // Should not panic
    }

    #[test]
    fn test_accumulate_i_squared_some() {
        let mut state = make_thermal_state();
        accumulate_i_squared(&mut state, Some(MilliAmp::from_ma(1000)));
        // Should not panic
    }

    #[test]
    fn test_update_slow_uses_cached_temp() {
        let mut state = make_thermal_state();
        let config = test_thermal_config();

        state.update_temperature(Some(CentiC::from_centi_c(3000))); // 30°C
        update_slow(&mut state, &config, 10_000); // 10ms
                                                  // Should not panic
    }

    #[test]
    fn test_update_slow_uses_default_when_no_temp() {
        let mut state = make_thermal_state();
        let config = test_thermal_config();

        state.update_temperature(None);
        update_slow(&mut state, &config, 10_000); // 10ms
                                                  // Should not panic, uses default ambient
    }

    #[test]
    fn test_check_motor_temp_no_fault_when_cool() {
        let mut state = make_thermal_state();
        let result = check_motor_temp(&mut state);
        assert!(result.is_none());
    }
}
