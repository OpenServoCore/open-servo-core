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

mod sensor_health;
mod thresholds;

pub use sensor_health::*;
pub use thresholds::*;

use crate::fault::FaultKind;
use open_servo_math::{CentiDeg, DeciC, MilliAmp};

/// Consolidated safety monitoring for servo control.
///
/// SafetyManager owns all safety-related state and provides methods
/// for the various safety checks performed at different frequencies.
///
/// ## Capability-Based Safety
///
/// Safety checks gracefully handle missing sensors:
/// - `check_current(None)` returns `None` (no fault possible)
/// - `check_temperature(None)` returns `None` (no fault possible)
///
/// This allows boards without certain sensors to skip those safety checks
/// while still benefiting from other protections.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SafetyManager {
    thresholds: SafetyThresholds,
    sensor_health: SensorHealth,
    last_temperature: Option<DeciC>,
    /// Last position for stall detection
    stall_last_position: CentiDeg,
    /// Consecutive ticks with PWM saturated and no movement
    stall_count: u16,
    /// Consecutive ticks with large position error
    position_error_count: u16,
}

impl Default for SafetyManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SafetyManager {
    /// Create a new SafetyManager with default thresholds.
    pub fn new() -> Self {
        Self {
            thresholds: SafetyThresholds::default(),
            sensor_health: SensorHealth::new(),
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
    #[inline]
    pub fn validate_position(&mut self, reading: CentiDeg) -> Result<CentiDeg, u8> {
        self.sensor_health
            .validate_position(reading, self.thresholds.position_max_delta)
    }

    /// Check if current exceeds the threshold.
    ///
    /// Returns `None` if:
    /// - `current` is `None` (no sensor available, check skipped)
    /// - Current is within limits
    ///
    /// Returns `Some(FaultKind::OverCurrent)` if current exceeds limit.
    #[inline]
    pub fn check_current(&self, current: Option<MilliAmp>) -> Option<FaultKind> {
        current.and_then(|c| {
            if c.abs() > self.thresholds.current_limit {
                Some(FaultKind::OverCurrent)
            } else {
                None
            }
        })
    }

    /// Check if temperature exceeds the threshold.
    ///
    /// Returns `None` if:
    /// - `temp` is `None` (no sensor available, check skipped)
    /// - Temperature is within limits
    ///
    /// Returns `Some(FaultKind::OverTemp)` if temperature exceeds limit.
    #[inline]
    pub fn check_temperature(&self, temp: Option<DeciC>) -> Option<FaultKind> {
        temp.and_then(|t| {
            if t > self.thresholds.temp_limit {
                Some(FaultKind::OverTemp)
            } else {
                None
            }
        })
    }

    /// Update cached temperature (called during fast tick).
    #[inline]
    pub fn update_temperature(&mut self, temp: Option<DeciC>) {
        self.last_temperature = temp;
    }

    /// Get cached temperature for slow tick check.
    #[inline]
    pub fn last_temperature(&self) -> Option<DeciC> {
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
    #[inline]
    pub fn check_stall(&mut self, position: CentiDeg, pwm_saturated: bool) -> Option<FaultKind> {
        let pos_delta = (position.as_cdeg() - self.stall_last_position.as_cdeg()).abs();
        let position_unchanged = pos_delta <= self.thresholds.stall_position_tolerance.as_cdeg();

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
    #[inline]
    pub fn check_position_error(
        &mut self,
        setpoint: CentiDeg,
        position: CentiDeg,
    ) -> Option<FaultKind> {
        let error = (setpoint.as_cdeg() - position.as_cdeg()).abs();

        if error > self.thresholds.position_error_limit.as_cdeg() {
            self.position_error_count = self.position_error_count.saturating_add(1);
            if self.position_error_count >= self.thresholds.position_error_timeout_ticks {
                return Some(FaultKind::PositionError);
            }
        } else {
            self.position_error_count = 0;
        }

        None
    }

    /// Clamp setpoint to position bounds.
    #[inline]
    pub fn clamp_setpoint(&self, setpoint: CentiDeg) -> CentiDeg {
        self.thresholds.clamp_setpoint(setpoint)
    }

    /// Reset safety state after fault clear.
    pub fn reset(&mut self) {
        self.sensor_health.reset();
        self.last_temperature = None;
        self.stall_count = 0;
        self.position_error_count = 0;
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

    // ========== check_current tests ==========

    #[test]
    fn test_check_current_none_returns_none() {
        let safety = SafetyManager::new();
        assert_eq!(safety.check_current(None), None);
    }

    #[test]
    fn test_check_current_under_limit() {
        let safety = SafetyManager::new();
        // Default limit is 800mA
        assert_eq!(safety.check_current(Some(MilliAmp::from_ma(500))), None);
        assert_eq!(safety.check_current(Some(MilliAmp::from_ma(799))), None);
    }

    #[test]
    fn test_check_current_over_limit() {
        let safety = SafetyManager::new();
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

    // ========== check_temperature tests ==========

    #[test]
    fn test_check_temperature_none_returns_none() {
        let safety = SafetyManager::new();
        assert_eq!(safety.check_temperature(None), None);
    }

    #[test]
    fn test_check_temperature_under_limit() {
        let safety = SafetyManager::new();
        // Default limit is 800 deciC (80°C)
        assert_eq!(safety.check_temperature(Some(DeciC::from_dc(500))), None);
        assert_eq!(safety.check_temperature(Some(DeciC::from_dc(799))), None);
    }

    #[test]
    fn test_check_temperature_over_limit() {
        let safety = SafetyManager::new();
        // Default limit is 800 deciC (80°C)
        assert_eq!(
            safety.check_temperature(Some(DeciC::from_dc(801))),
            Some(FaultKind::OverTemp)
        );
    }

    // ========== check_stall tests ==========

    #[test]
    fn test_check_stall_no_saturation() {
        let mut safety = SafetyManager::new();
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
        let mut safety = SafetyManager::new();
        // Saturated but moving - counter should reset each tick
        // Use smaller increments to avoid i16 overflow
        for i in 0..200i16 {
            let pos = CentiDeg::from_cdeg(i * 100); // Moving 1° per tick
            assert_eq!(safety.check_stall(pos, true), None);
        }
    }

    #[test]
    fn test_check_stall_triggers_after_timeout() {
        let mut safety = SafetyManager::new();
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
        let mut safety = SafetyManager::new();
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
        let mut safety = SafetyManager::new();
        // Default limit is 3000 cdeg (30°)
        let sp = CentiDeg::from_cdeg(9000);
        let pos = CentiDeg::from_cdeg(9000 + 2000); // 20° error

        for _ in 0..100 {
            assert_eq!(safety.check_position_error(sp, pos), None);
        }
    }

    #[test]
    fn test_check_position_error_triggers_after_timeout() {
        let mut safety = SafetyManager::new();
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
        let mut safety = SafetyManager::new();
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
        let safety = SafetyManager::new();
        // Default bounds: 0 to 18000 cdeg
        let sp = CentiDeg::from_cdeg(9000);
        assert_eq!(safety.clamp_setpoint(sp).as_cdeg(), 9000);
    }

    #[test]
    fn test_clamp_setpoint_below_min() {
        let safety = SafetyManager::new();
        let sp = CentiDeg::from_cdeg(-1000);
        assert_eq!(safety.clamp_setpoint(sp).as_cdeg(), 0); // Clamped to min
    }

    #[test]
    fn test_clamp_setpoint_above_max() {
        let safety = SafetyManager::new();
        let sp = CentiDeg::from_cdeg(20000);
        assert_eq!(safety.clamp_setpoint(sp).as_cdeg(), 18000); // Clamped to max
    }

    // ========== reset tests ==========

    #[test]
    fn test_reset_clears_counters() {
        let mut safety = SafetyManager::new();

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
