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
