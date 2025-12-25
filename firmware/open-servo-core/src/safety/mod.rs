//! Safety management for servo control.
//!
//! This module provides:
//! - `SafetyManager`: Consolidated safety logic for fault detection
//! - `SafetyThresholds`: Configurable limits for current, temperature, position
//! - `SensorHealth`: Tracks sensor validity and detects faulty readings
//!
//! Thresholds are checked at different frequencies based on criticality:
//! - 10kHz (control tick): Position validation, current check, setpoint clamping
//! - 100Hz (slow tick): Temperature check

mod compliance_limiter;
mod sensor_health;
mod thermal_fault;
mod thresholds;

pub use compliance_limiter::ComplianceLimiter;
pub use sensor_health::*;
pub use thermal_fault::*;
pub use thresholds::*;

use crate::fault::FaultKind;
use open_servo_math::{CentiC, CentiDeg, ComplianceConfig, MilliAmp, ThermalModel};

/// Consolidated safety monitoring for servo control.
///
/// SafetyManager owns all safety-related state and provides methods
/// for the various safety checks performed at different frequencies.
///
/// ## Capability-Based Safety
///
/// Safety checks gracefully handle missing sensors:
/// - `check_current(None)` returns `None` (no fault possible)
/// - `check_mcu_temperature(None)` returns `None` (no fault possible)
///
/// This allows boards without certain sensors to skip those safety checks
/// while still benefiting from other protections.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SafetyManager {
    thresholds: SafetyThresholds,
    sensor_health: SensorHealth,
    thermal_model: ThermalModel,
    thermal_fault: ThermalFaultDetector,
    compliance_limiter: ComplianceLimiter,
    last_temperature: Option<CentiC>,
    /// Last position for stall detection
    stall_last_position: CentiDeg,
    /// Consecutive ticks with PWM saturated and no movement
    stall_count: u16,
    /// Consecutive ticks with large position error
    position_error_count: u16,
}


impl SafetyManager {
    /// Create a new SafetyManager with explicit configuration.
    pub fn new(
        thresholds: SafetyThresholds,
        thermal_model: ThermalModel,
        move_compliance_config: ComplianceConfig,
    ) -> Self {
        Self {
            thresholds,
            sensor_health: SensorHealth::new(),
            thermal_model,
            thermal_fault: ThermalFaultDetector::new(10000, 1000), // 100°C max, 10°C hysteresis
            compliance_limiter: ComplianceLimiter::new(move_compliance_config),
            last_temperature: None,
            stall_last_position: CentiDeg::from_cdeg(0),
            stall_count: 0,
            position_error_count: 0,
        }
    }

    /// Validate a position reading against the previous good reading.
    ///
    /// Returns:
    /// - `Ok(position)` if the reading is valid
    /// - `Err(count)` if the reading is invalid (count = consecutive bad reads)
    pub fn validate_position(&mut self, reading: CentiDeg) -> Result<CentiDeg, u8> {
        self.sensor_health
            .validate_position(reading, self.thresholds.position_max_delta)
    }

    /// Check if current exceeds the threshold.
    ///
    /// Returns `None` if:
    /// - `current` is `None` (no sensor available, check skipped)
    /// - Current is within limits
    /// - The `current-sense` feature is disabled (always returns `None`)
    ///
    /// Returns `Some(FaultKind::OverCurrent)` if current exceeds limit.
    ///
    /// This method always exists but becomes a no-op when `current-sense` is disabled.
    pub fn check_current(&self, current: Option<MilliAmp>) -> Option<FaultKind> {
        #[cfg(feature = "current-sense-bus")]
        {
            current.and_then(|c| {
                if c.abs() > self.thresholds.current_limit {
                    Some(FaultKind::OverCurrent)
                } else {
                    None
                }
            })
        }
        #[cfg(not(feature = "current-sense-bus"))]
        {
            let _ = current; // silence unused warning
            None
        }
    }

    /// Check if MCU temperature exceeds the threshold.
    ///
    /// Returns `None` if:
    /// - `temp` is `None` (no sensor available, check skipped)
    /// - Temperature is within limits
    ///
    /// Returns `Some(FaultKind::McuOverTemp)` if temperature exceeds limit.
    pub fn check_mcu_temperature(&self, temp: Option<CentiC>) -> Option<FaultKind> {
        temp.and_then(|t| {
            if t > self.thresholds.mcu_temp_limit {
                Some(FaultKind::McuOverTemp)
            } else {
                None
            }
        })
    }

    /// Update cached temperature (called during fast tick).
    #[inline]
    pub fn update_temperature(&mut self, temp: Option<CentiC>) {
        self.last_temperature = temp;
    }

    /// Get cached temperature for slow tick check.
    #[inline]
    pub fn last_temperature(&self) -> Option<CentiC> {
        self.last_temperature
    }

    /// Check if sensor fault threshold has been exceeded.
    #[inline]
    pub fn is_sensor_fault(&self) -> bool {
        self.sensor_health.bad_count() >= self.thresholds.sensor_fault_count
    }

    /// Check for motor stall (PWM saturated but no movement).
    ///
    /// Call this every control tick with current position and PWM saturation state.
    /// Returns `Some(FaultKind::Stall)` if stall persists for configured timeout.
    pub fn check_stall(&mut self, position: CentiDeg, pwm_saturated: bool) -> Option<FaultKind> {
        let pos_i32 = position.as_cdeg() as i32;
        let last_i32 = self.stall_last_position.as_cdeg() as i32;
        let pos_delta = (pos_i32 - last_i32).abs();
        let tol_i32 = self.thresholds.stall_position_tolerance.as_cdeg() as i32;
        let position_unchanged = pos_delta <= tol_i32;

        if pwm_saturated && position_unchanged {
            self.stall_count = self.stall_count.saturating_add(1);
            if self.stall_count >= self.thresholds.stall_timeout_ticks {
                return Some(FaultKind::Stall);
            }
        } else {
            self.stall_count = 0;
        }

        // Update last position for next tick
        self.stall_last_position = position;
        None
    }

    /// Check for persistent large position error.
    ///
    /// Call this every control tick with setpoint and position.
    /// Returns `Some(FaultKind::PositionError)` if large error persists for configured timeout.
    pub fn check_position_error(
        &mut self,
        setpoint: CentiDeg,
        position: CentiDeg,
    ) -> Option<FaultKind> {
        let sp_i32 = setpoint.as_cdeg() as i32;
        let pos_i32 = position.as_cdeg() as i32;
        let err = (sp_i32 - pos_i32).abs();
        let limit_i32 = self.thresholds.position_error_limit.as_cdeg() as i32;

        if err > limit_i32 {
            self.position_error_count = self.position_error_count.saturating_add(1);
            if self.position_error_count >= self.thresholds.position_error_timeout_ticks {
                return Some(FaultKind::PositionError);
            }
        } else {
            self.position_error_count = 0;
        }

        None
    }

    /// Accumulate I² for thermal model (fast tick - 10kHz).
    ///
    /// Call this every fast tick with current measurement.
    /// The thermal model internally accumulates I² for averaging.
    #[inline]
    pub fn accumulate_thermal_i_squared(&mut self, current: Option<MilliAmp>) {
        self.thermal_model.update_fast(current.map(|c| c.as_ma()));
    }
    
    /// Update thermal model and check for faults (slow tick - 100Hz).
    ///
    /// Call this from slow tick to update the thermal physics model
    /// and check for temperature faults.
    pub fn update_thermal_slow(&mut self) {
        // Use MCU temp as ambient estimate
        let ambient_cdeg = self.last_temperature
            .map(|t| t.as_centi_c())
            .unwrap_or(2500);  // Default to 25°C if no temp sensor
        
        // Update thermal model - it handles I² averaging internally
        self.thermal_model.update_slow(ambient_cdeg);
    }
    
    /// Check if motor has exceeded safe temperature.
    #[inline]
    pub fn check_motor_temperature(&mut self) -> Option<FaultKind> {
        let temp = self.thermal_model.temperature_cdeg();
        self.thermal_fault.check_fault(temp)
    }
    
    /// Get motor temperature in degrees (for debug display).
    pub fn motor_temp_deg(&self) -> i16 {
        self.thermal_model.temperature_deg()
    }
    
    /// Get motor temperature rise in degrees (for debug display).
    pub fn motor_temp_rise_deg(&self) -> i16 {
        self.thermal_model.temp_rise_deg()
    }

    /// Clamp setpoint to position bounds.
    #[inline]
    pub fn clamp_setpoint(&self, setpoint: CentiDeg) -> CentiDeg {
        self.thresholds.clamp_setpoint(setpoint)
    }
    
    /// Get mutable reference to compliance limiter for updates.
    #[inline]
    pub fn compliance_limiter_mut(&mut self) -> &mut ComplianceLimiter {
        &mut self.compliance_limiter
    }
    
    /// Get reference to compliance limiter for reading state.
    #[inline]
    pub fn compliance_limiter(&self) -> &ComplianceLimiter {
        &self.compliance_limiter
    }

    /// Reset safety state after fault clear.
    /// 
    /// NOTE: This only resets fault detection state, NOT physical state.
    /// Temperature and I² accumulation continue to track physical reality.
    pub fn reset(&mut self) {
        self.sensor_health.reset();
        
        // Try to reset thermal fault (only succeeds if cool enough)
        let temp = self.thermal_model.temperature_cdeg();
        self.thermal_fault.try_reset(temp);
        
        // Reset fault detection counters
        self.stall_count = 0;
        self.position_error_count = 0;
        
        // Do NOT reset:
        // - thermal_model (physical temperature state)
        // - last_temperature (just cached sensor data)
    }

    // ============= Threshold accessors =============

    /// Get current safety thresholds.
    pub fn thresholds(&self) -> &SafetyThresholds {
        &self.thresholds
    }

    /// Get mutable reference to thresholds.
    pub fn thresholds_mut(&mut self) -> &mut SafetyThresholds {
        &mut self.thresholds
    }

    /// Set all safety thresholds.
    pub fn set_thresholds(&mut self, thresholds: SafetyThresholds) {
        self.thresholds = thresholds;
    }

    /// Get sensor health state (for debugging).
    pub fn sensor_health(&self) -> &SensorHealth {
        &self.sensor_health
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use open_servo_math::{ComplianceConfig, ThermalModel};

    /// Create a SafetyManager with default test values.
    fn make_safety() -> SafetyManager {
        let thresholds = SafetyThresholds::new(
            800,    // current_limit_ma
            8000,   // mcu_temp_limit_cc (80°C)
            500,    // position_max_delta_cdeg
            10,     // sensor_fault_count
            0,      // position_min_cdeg
            18000,  // position_max_cdeg
            1000,   // stall_timeout_ticks
            10,     // stall_position_tolerance_cdeg
            3000,   // position_error_limit_cdeg
            50,     // position_error_timeout_ticks
        );
        let thermal_model = ThermalModel::new(5000, 1000, 1500);
        let compliance_config = ComplianceConfig::new(600, 50, 3, 230, 3277);
        SafetyManager::new(thresholds, thermal_model, compliance_config)
    }

    // ========== check_current tests ==========

    // This test always passes - check_current always exists and returns None for None input
    #[test]
    fn test_check_current_none_returns_none() {
        let safety = make_safety();
        assert_eq!(safety.check_current(None), None);
    }

    // These tests only make sense when current-sense is enabled
    #[cfg(feature = "current-sense-bus")]
    #[test]
    fn test_check_current_under_limit() {
        let safety = make_safety();
        // Default limit is 800mA
        assert_eq!(safety.check_current(Some(MilliAmp::from_ma(500))), None);
        assert_eq!(safety.check_current(Some(MilliAmp::from_ma(799))), None);
    }

    #[cfg(feature = "current-sense-bus")]
    #[test]
    fn test_check_current_over_limit() {
        let safety = make_safety();
        // Default limit is 800mA
        assert_eq!(
            safety.check_current(Some(MilliAmp::from_ma(801))),
            Some(FaultKind::OverCurrent)
        );
        assert_eq!(
            safety.check_current(Some(MilliAmp::from_ma(1000))),
            Some(FaultKind::OverCurrent)
        );
    }

    // When current-sense is disabled, check_current always returns None (no-op)
    #[cfg(not(feature = "current-sense-bus"))]
    #[test]
    fn test_check_current_disabled_always_none() {
        let safety = make_safety();
        // Even with a value that would trigger overcurrent, returns None
        assert_eq!(safety.check_current(Some(MilliAmp::from_ma(10000))), None);
    }

    // ========== check_mcu_temperature tests ==========

    #[test]
    fn test_check_mcu_temperature_none_returns_none() {
        let safety = make_safety();
        assert_eq!(safety.check_mcu_temperature(None), None);
    }

    #[test]
    fn test_check_mcu_temperature_under_limit() {
        let safety = make_safety();
        // Default limit is 8000 centiC (80°C)
        assert_eq!(safety.check_mcu_temperature(Some(CentiC::from_centi_c(5000))), None);
        assert_eq!(safety.check_mcu_temperature(Some(CentiC::from_centi_c(7999))), None);
    }

    #[test]
    fn test_check_mcu_temperature_over_limit() {
        let safety = make_safety();
        // Default limit is 8000 centiC (80°C)
        assert_eq!(
            safety.check_mcu_temperature(Some(CentiC::from_centi_c(8001))),
            Some(FaultKind::McuOverTemp)
        );
    }

    // ========== check_stall tests ==========

    #[test]
    fn test_check_stall_no_saturation() {
        let mut safety = make_safety();
        // Not saturated - should never fault
        for _ in 0..2000 {
            assert_eq!(
                safety.check_stall(CentiDeg::from_cdeg(9000), false),
                None
            );
        }
    }

    #[test]
    fn test_check_stall_moving() {
        let mut safety = make_safety();
        // Saturated but moving - counter should reset each tick
        // Use smaller increments to avoid i16 overflow
        for i in 0..200i16 {
            let pos = CentiDeg::from_cdeg(i * 100); // Moving 1° per tick
            assert_eq!(safety.check_stall(pos, true), None);
        }
    }

    #[test]
    fn test_check_stall_triggers_after_timeout() {
        let mut safety = make_safety();
        // Default timeout is 1000 ticks
        // First call: delta from 0 is large, resets counter, sets last_position=9000
        let pos = CentiDeg::from_cdeg(9000);
        safety.check_stall(pos, true); // Initialize last_position

        // Next 999 ticks - counter goes 1..999 (no fault yet)
        for _ in 0..999 {
            assert_eq!(safety.check_stall(pos, true), None);
        }

        // Counter reaches 1000 on this tick - fault!
        assert_eq!(safety.check_stall(pos, true), Some(FaultKind::Stall));
    }

    #[test]
    fn test_check_stall_resets_on_movement() {
        let mut safety = make_safety();
        let pos = CentiDeg::from_cdeg(9000);

        // Initialize last_position
        safety.check_stall(pos, true);
        // Build up stall count
        for _ in 0..500 {
            safety.check_stall(pos, true);
        }

        // Move - should reset counter (delta > tolerance)
        let new_pos = CentiDeg::from_cdeg(9500); // Move 5°
        safety.check_stall(new_pos, true);

        // Need 1000 ticks to fault: 999 with no fault, then 1000th triggers
        for _ in 0..999 {
            assert_eq!(safety.check_stall(new_pos, true), None);
        }
        assert_eq!(safety.check_stall(new_pos, true), Some(FaultKind::Stall));
    }

    // ========== check_position_error tests ==========

    #[test]
    fn test_check_position_error_within_limit() {
        let mut safety = make_safety();
        // Default limit is 3000 cdeg (30°)
        let sp = CentiDeg::from_cdeg(9000);
        let pos = CentiDeg::from_cdeg(9000 + 2000); // 20° error

        for _ in 0..100 {
            assert_eq!(safety.check_position_error(sp, pos), None);
        }
    }

    #[test]
    fn test_check_position_error_triggers_after_timeout() {
        let mut safety = make_safety();
        // Default: 3000 cdeg limit, 50 tick timeout (at 100Hz)
        let sp = CentiDeg::from_cdeg(9000);
        let pos = CentiDeg::from_cdeg(0); // -90° error (way over 30°)

        // First 49 ticks - no fault
        for _ in 0..49 {
            assert_eq!(safety.check_position_error(sp, pos), None);
        }

        // 50th tick - fault!
        assert_eq!(
            safety.check_position_error(sp, pos),
            Some(FaultKind::PositionError)
        );
    }

    #[test]
    fn test_check_position_error_resets_when_corrected() {
        let mut safety = make_safety();
        let sp = CentiDeg::from_cdeg(9000);
        let bad_pos = CentiDeg::from_cdeg(0);

        // Build up error count
        for _ in 0..25 {
            safety.check_position_error(sp, bad_pos);
        }

        // Position corrects - should reset counter
        let good_pos = CentiDeg::from_cdeg(8900); // Within 30° of setpoint
        safety.check_position_error(sp, good_pos);

        // Need another 50 ticks to fault
        for _ in 0..49 {
            assert_eq!(safety.check_position_error(sp, bad_pos), None);
        }
        assert_eq!(
            safety.check_position_error(sp, bad_pos),
            Some(FaultKind::PositionError)
        );
    }

    // ========== clamp_setpoint tests ==========

    #[test]
    fn test_clamp_setpoint_within_bounds() {
        let safety = make_safety();
        // Default bounds: 0 to 18000 cdeg
        let sp = CentiDeg::from_cdeg(9000);
        assert_eq!(safety.clamp_setpoint(sp).as_cdeg(), 9000);
    }

    #[test]
    fn test_clamp_setpoint_below_min() {
        let safety = make_safety();
        let sp = CentiDeg::from_cdeg(-1000);
        assert_eq!(safety.clamp_setpoint(sp).as_cdeg(), 0); // Clamped to min
    }

    #[test]
    fn test_clamp_setpoint_above_max() {
        let safety = make_safety();
        let sp = CentiDeg::from_cdeg(20000);
        assert_eq!(safety.clamp_setpoint(sp).as_cdeg(), 18000); // Clamped to max
    }

    // ========== reset tests ==========

    #[test]
    fn test_reset_clears_counters() {
        let mut safety = make_safety();

        // Build up stall and error counts
        let pos = CentiDeg::from_cdeg(9000);
        for _ in 0..500 {
            safety.check_stall(pos, true);
        }
        for _ in 0..25 {
            safety.check_position_error(CentiDeg::from_cdeg(9000), CentiDeg::from_cdeg(0));
        }

        safety.reset();

        // After reset, need full timeout to fault again
        for _ in 0..999 {
            assert_eq!(safety.check_stall(pos, true), None);
        }
        for _ in 0..49 {
            assert_eq!(
                safety.check_position_error(CentiDeg::from_cdeg(9000), CentiDeg::from_cdeg(0)),
                None
            );
        }
    }
}
