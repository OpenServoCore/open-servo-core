//! Compliance limiting and mode switching.
//!
//! Manages the ComplianceLimiter and handles switching between Move/Hold modes
//! with their different compliance configurations.

use crate::safety::ComplianceLimiter;
use crate::servo_core::ServoMode;
use open_servo_math::{ComplianceConfig as LimiterConfig, MilliAmp};

/// Hold duty cap curve parameters.
///
/// The duty cap ramps linearly from HOLD_DUTY_MIN at small errors
/// to HOLD_DUTY_MAX at large errors.
const HOLD_ERROR_START: i16 = 500; // 5° - start ramping
const HOLD_ERROR_END: i16 = 1500; // 15° - max cap
const HOLD_DUTY_MIN: i16 = 6553; // 20% at small error
const HOLD_DUTY_MAX: i16 = 14746; // 45% at large error

/// Compliance configuration for different modes.
///
/// Lives in CoreConfig.compliance
#[derive(Debug, Clone)]
pub struct ComplianceConfig {
    /// Configuration for Move mode
    pub move_config: LimiterConfig,
    /// Configuration for Hold mode
    pub hold_config: LimiterConfig,
}

impl ComplianceConfig {
    /// Create ComplianceConfig with sensible defaults.
    ///
    /// Defaults:
    /// - Limit: 1500mA
    /// - Hysteresis: 200mA
    /// - Deglitch: 3 samples
    /// - Backoff: 0.88 (225/256)
    /// - Recovery: 10%/sec (3277)
    pub fn with_defaults() -> Self {
        let default_config = LimiterConfig::new(
            1500, // limit_ma
            200,  // hysteresis_ma
            3,    // deglitch_samples
            225,  // backoff_factor_q8 (0.88)
            3277, // recovery_rate (10%/sec)
        );
        Self {
            move_config: default_config,
            hold_config: default_config,
        }
    }
}

impl Default for ComplianceConfig {
    fn default() -> Self {
        Self::with_defaults()
    }
}

impl ComplianceConfig {
    /// Create a new ComplianceConfig with the given configurations.
    pub fn new(move_config: LimiterConfig, hold_config: LimiterConfig) -> Self {
        Self {
            move_config,
            hold_config,
        }
    }
}

/// Mutable compliance state.
///
/// Lives in CoreInternal.compliance
#[derive(Debug, Clone)]
pub struct ComplianceState {
    /// The compliance limiter
    pub limiter: ComplianceLimiter,
    /// Current operating mode
    pub mode: ServoMode,
}

impl ComplianceState {
    /// Create a new ComplianceState with Move mode configuration.
    pub fn new(move_config: LimiterConfig) -> Self {
        Self {
            limiter: ComplianceLimiter::new(move_config),
            mode: ServoMode::Move,
        }
    }

    /// Get whether compliance is currently limited.
    pub fn is_limited(&self) -> bool {
        self.limiter.is_limited()
    }
}

/// Update compliance limiter with new current reading.
///
/// Called during fast tick.
pub fn update_limiter(
    state: &mut ComplianceState,
    current: Option<MilliAmp>,
    pwm_direction: i16,
    dt_us: u32,
) {
    state.limiter.update(current, pwm_direction, dt_us);
}

/// Get current compliance limits from limiter.
///
/// Returns (min, max) duty limits as i32 values.
pub fn get_limits(state: &ComplianceState) -> (i32, i32) {
    state.limiter.get_limits()
}

/// Compute dynamic hold duty cap based on position error.
///
/// Returns the maximum duty magnitude in Hold mode, which ramps
/// from 20% at 5° error to 45% at 15° error.
pub fn compute_hold_duty_cap(error_abs: i16) -> i16 {
    if error_abs <= HOLD_ERROR_START {
        HOLD_DUTY_MIN
    } else if error_abs >= HOLD_ERROR_END {
        HOLD_DUTY_MAX
    } else {
        // Linear ramp between start and end
        let range_error = HOLD_ERROR_END - HOLD_ERROR_START;
        let range_duty = HOLD_DUTY_MAX - HOLD_DUTY_MIN;
        let progress = error_abs - HOLD_ERROR_START;
        HOLD_DUTY_MIN + (progress as i32 * range_duty as i32 / range_error as i32) as i16
    }
}

/// Switch compliance configuration for the new mode.
///
/// Called when mode changes between Move and Hold.
pub fn switch_config(state: &mut ComplianceState, new_mode: ServoMode, config: &ComplianceConfig) {
    state.mode = new_mode;

    // Select configuration based on mode
    let limiter_config = match new_mode {
        ServoMode::Move => &config.move_config,
        ServoMode::Hold => &config.hold_config,
        ServoMode::Yield => &config.hold_config, // Use hold config in yield
    };

    // Create new limiter with appropriate config
    state.limiter = ComplianceLimiter::new(*limiter_config);
}

/// Reset compliance state.
///
/// Called on fault clear or disengage.
pub fn reset(state: &mut ComplianceState, config: &ComplianceConfig) {
    state.mode = ServoMode::Move;
    state.limiter = ComplianceLimiter::new(config.move_config);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hold_duty_cap_at_small_error() {
        let cap = compute_hold_duty_cap(0);
        assert_eq!(cap, HOLD_DUTY_MIN);

        let cap = compute_hold_duty_cap(HOLD_ERROR_START);
        assert_eq!(cap, HOLD_DUTY_MIN);
    }

    #[test]
    fn test_hold_duty_cap_at_large_error() {
        let cap = compute_hold_duty_cap(HOLD_ERROR_END);
        assert_eq!(cap, HOLD_DUTY_MAX);

        let cap = compute_hold_duty_cap(HOLD_ERROR_END + 1000);
        assert_eq!(cap, HOLD_DUTY_MAX);
    }

    #[test]
    fn test_hold_duty_cap_ramp() {
        // Midpoint should be roughly midway
        let midpoint_error = (HOLD_ERROR_START + HOLD_ERROR_END) / 2;
        let cap = compute_hold_duty_cap(midpoint_error);
        let expected_mid = (HOLD_DUTY_MIN + HOLD_DUTY_MAX) / 2;

        // Allow some rounding error
        assert!((cap - expected_mid).abs() <= 1);
    }

    #[test]
    fn test_switch_config() {
        let config = ComplianceConfig::default();
        let mut state = ComplianceState::new(config.move_config);
        assert_eq!(state.mode, ServoMode::Move);

        switch_config(&mut state, ServoMode::Hold, &config);
        assert_eq!(state.mode, ServoMode::Hold);
    }
}
