//! Safety checks for sensor health, stall detection, and position error.
//!
//! This module provides the safety checking functions that run during the fast tick
//! protect stage. State lives in CoreInternal.safety, config in CoreConfig.safety.

use crate::fault::FaultKind;
use crate::safety::{SafetyThresholds, SensorHealth};
use open_servo_math::{CentiC, CentiDeg, MilliAmp};

/// Safety configuration.
///
/// Lives in CoreConfig.safety
#[derive(Debug, Clone)]
pub struct SafetyConfig {
    /// Thresholds for all safety checks
    pub thresholds: SafetyThresholds,
}

impl SafetyConfig {
    /// Create SafetyConfig with sensible defaults.
    ///
    /// Defaults:
    /// - Current limit: 2000mA
    /// - MCU temp limit: 80°C
    /// - Position max delta: 50° per tick
    /// - Sensor fault count: 5
    /// - Position range: 0-327° (i16 max)
    /// - Stall timeout: 500 ticks
    /// - Stall tolerance: 1°
    /// - Position error limit: 30°
    /// - Position error timeout: 500ms
    pub fn with_defaults() -> Self {
        Self {
            thresholds: SafetyThresholds::new(
                2000,    // current_limit_ma
                8000,    // mcu_temp_limit_cc (80°C)
                5000,    // position_max_delta_cdeg (50°)
                5,       // sensor_fault_count
                0,       // position_min_cdeg
                32700,   // position_max_cdeg (327°, ~i16::MAX)
                500,     // stall_timeout_ticks
                100,     // stall_position_tolerance_cdeg (1°)
                3000,    // position_error_limit_cdeg (30°)
                500_000, // position_error_timeout_us (500ms)
            ),
        }
    }
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self::with_defaults()
    }
}

impl SafetyConfig {
    /// Create a new SafetyConfig with the given thresholds.
    pub fn new(thresholds: SafetyThresholds) -> Self {
        Self { thresholds }
    }
}

/// Mutable safety state.
///
/// Lives in CoreInternal.safety
#[derive(Debug, Clone)]
pub struct SafetyState {
    /// Position sensor health tracking
    pub sensor_health: SensorHealth,
    /// Consecutive ticks with PWM saturated and no movement
    pub stall_count: u16,
    /// Last position for stall detection
    pub stall_last_pos: CentiDeg,
    /// Consecutive ticks with large position error
    pub position_error_count: u16,
}

impl Default for SafetyState {
    fn default() -> Self {
        Self {
            sensor_health: SensorHealth::new(),
            stall_count: 0,
            stall_last_pos: CentiDeg::from_cdeg(0),
            position_error_count: 0,
        }
    }
}

impl SafetyState {
    /// Create a new SafetyState.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset safety state after fault clear.
    pub fn reset(&mut self) {
        self.sensor_health.reset();
        self.stall_count = 0;
        self.position_error_count = 0;
    }
}

/// Validate a position reading and check for sensor faults.
///
/// Returns:
/// - `Ok(position)` if the reading is valid
/// - `Err(FaultKind::EncoderFault)` if too many consecutive bad readings
pub fn check_position(
    state: &mut SafetyState,
    config: &SafetyConfig,
    reading: CentiDeg,
) -> Result<CentiDeg, FaultKind> {
    match state
        .sensor_health
        .validate_position(reading, config.thresholds.position_max_delta)
    {
        Ok(pos) => {
            // Check if we've exceeded sensor fault threshold
            if state.sensor_health.bad_count() >= config.thresholds.sensor_fault_count {
                Err(FaultKind::EncoderFault)
            } else {
                Ok(pos)
            }
        }
        Err(_bad_count) => {
            // Still might be under threshold
            if state.sensor_health.bad_count() >= config.thresholds.sensor_fault_count {
                Err(FaultKind::EncoderFault)
            } else {
                // Use last good position
                Ok(state.sensor_health.last_good_position())
            }
        }
    }
}

/// Check if current exceeds the threshold.
///
/// Returns `None` if:
/// - `current` is `None` (no sensor available, check skipped)
/// - Current is within limits
/// - The `current-sense-bus` feature is disabled
///
/// Returns `Some(FaultKind::OverCurrent)` if current exceeds limit.
pub fn check_current(config: &SafetyConfig, current: Option<MilliAmp>) -> Option<FaultKind> {
    #[cfg(feature = "current-sense-bus")]
    {
        current.and_then(|c| {
            if c.abs() > config.thresholds.current_limit {
                Some(FaultKind::OverCurrent)
            } else {
                None
            }
        })
    }
    #[cfg(not(feature = "current-sense-bus"))]
    {
        let _ = (config, current); // silence unused warning
        None
    }
}

/// Check for motor stall (PWM saturated but no movement).
///
/// Returns `Some(FaultKind::Stall)` if stall persists for configured timeout.
pub fn check_stall(
    state: &mut SafetyState,
    config: &SafetyConfig,
    position: CentiDeg,
    pwm_saturated: bool,
) -> Option<FaultKind> {
    let pos_i32 = position.as_cdeg() as i32;
    let last_i32 = state.stall_last_pos.as_cdeg() as i32;
    let pos_delta = (pos_i32 - last_i32).abs();
    let tol_i32 = config.thresholds.stall_position_tolerance.as_cdeg() as i32;
    let position_unchanged = pos_delta <= tol_i32;

    if pwm_saturated && position_unchanged {
        state.stall_count = state.stall_count.saturating_add(1);
        if state.stall_count >= config.thresholds.stall_timeout_ticks {
            return Some(FaultKind::Stall);
        }
    } else {
        state.stall_count = 0;
    }

    // Update last position for next tick
    state.stall_last_pos = position;
    None
}

/// Check for persistent large position error.
///
/// Returns `Some(FaultKind::PositionError)` if large error persists for configured timeout.
///
/// Note: This uses tick-based counting. The timeout_ticks should be derived from
/// the time-based config using `ticks_from_us(fast_dt_us, position_error_timeout_us)`.
pub fn check_position_error(
    state: &mut SafetyState,
    config: &SafetyConfig,
    setpoint: CentiDeg,
    position: CentiDeg,
    timeout_ticks: u32,
) -> Option<FaultKind> {
    let sp_i32 = setpoint.as_cdeg() as i32;
    let pos_i32 = position.as_cdeg() as i32;
    let err = (sp_i32 - pos_i32).abs();
    let limit_i32 = config.thresholds.position_error_limit.as_cdeg() as i32;

    if err > limit_i32 {
        state.position_error_count = state.position_error_count.saturating_add(1);
        if (state.position_error_count as u32) >= timeout_ticks {
            return Some(FaultKind::PositionError);
        }
    } else {
        state.position_error_count = 0;
    }

    None
}

/// Check if MCU temperature exceeds the threshold.
///
/// Returns `None` if:
/// - `temp` is `None` (no sensor available, check skipped)
/// - Temperature is within limits
///
/// Returns `Some(FaultKind::McuOverTemp)` if temperature exceeds limit.
pub fn check_mcu_temperature(config: &SafetyConfig, temp: Option<CentiC>) -> Option<FaultKind> {
    temp.and_then(|t| {
        if t > config.thresholds.mcu_temp_limit {
            Some(FaultKind::McuOverTemp)
        } else {
            None
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> SafetyConfig {
        SafetyConfig::default()
    }

    #[test]
    fn test_check_position_valid() {
        let mut state = SafetyState::new();
        let config = make_config();

        // First reading
        let result = check_position(&mut state, &config, CentiDeg::from_cdeg(9000));
        assert!(result.is_ok());

        // Second reading close to first
        let result = check_position(&mut state, &config, CentiDeg::from_cdeg(9010));
        assert!(result.is_ok());
    }

    #[test]
    fn test_check_stall_no_fault_when_moving() {
        let mut state = SafetyState::new();
        let config = make_config();

        // PWM saturated but position changes (within i16 range)
        for i in 0..100 {
            let pos = CentiDeg::from_cdeg(9000 + i * 10);
            let result = check_stall(&mut state, &config, pos, true);
            assert!(result.is_none());
        }
    }

    #[test]
    fn test_check_stall_no_fault_when_not_saturated() {
        let mut state = SafetyState::new();
        let config = make_config();

        // Position unchanged but PWM not saturated
        let pos = CentiDeg::from_cdeg(9000);
        for _ in 0..1000 {
            let result = check_stall(&mut state, &config, pos, false);
            assert!(result.is_none());
        }
    }

    #[test]
    fn test_check_position_error_within_limit() {
        let mut state = SafetyState::new();
        let config = make_config();

        let setpoint = CentiDeg::from_cdeg(9000);
        let position = CentiDeg::from_cdeg(9010); // Small error

        for _ in 0..1000 {
            let result = check_position_error(&mut state, &config, setpoint, position, 100);
            assert!(result.is_none());
        }
        assert_eq!(state.position_error_count, 0);
    }

    #[test]
    fn test_check_position_error_triggers() {
        let mut state = SafetyState::new();
        let config = make_config();
        let timeout_ticks = 100;

        let setpoint = CentiDeg::from_cdeg(9000);
        let position = CentiDeg::from_cdeg(5000); // Large error (40°)

        // Should trigger after timeout_ticks
        for i in 0..timeout_ticks {
            let result =
                check_position_error(&mut state, &config, setpoint, position, timeout_ticks);
            if i < timeout_ticks - 1 {
                assert!(result.is_none(), "Should not trigger at tick {}", i);
            } else {
                assert_eq!(result, Some(FaultKind::PositionError));
            }
        }
    }

    #[test]
    fn test_reset_clears_counters() {
        let mut state = SafetyState::new();
        state.stall_count = 50;
        state.position_error_count = 30;

        state.reset();

        assert_eq!(state.stall_count, 0);
        assert_eq!(state.position_error_count, 0);
    }
}
