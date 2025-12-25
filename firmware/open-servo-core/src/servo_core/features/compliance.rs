//! Compliance limiting and mode switching.
//!
//! Manages the ComplianceLimiter and handles switching between Move/Hold modes
//! with their different compliance configurations.
//!
//! Hold duty curve parameters come from PolicyConfig (board-supplied).

use crate::safety::ComplianceLimiter;
use crate::servo_core::features::PolicyConfig;
use crate::servo_core::ServoMode;
use open_servo_math::{ComplianceConfig as LimiterConfig, MilliAmp};

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
/// from hold_duty_min at small errors to hold_duty_max at large errors.
pub fn compute_hold_duty_cap(error_abs: i16, policy: &PolicyConfig) -> i16 {
    let error_start = policy.hold_duty_error_start.as_cdeg();
    let error_end = policy.hold_duty_error_end.as_cdeg();
    let duty_min = policy.hold_duty_min.as_raw();
    let duty_max = policy.hold_duty_max.as_raw();

    if error_abs <= error_start {
        duty_min
    } else if error_abs >= error_end {
        duty_max
    } else {
        // Linear ramp between start and end
        let range_error = error_end - error_start;
        let range_duty = duty_max - duty_min;
        let progress = error_abs - error_start;
        duty_min + (progress as i32 * range_duty as i32 / range_error as i32) as i16
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
    use open_servo_math::{CentiDeg, DegPerSec10, Duty};

    /// Test config: 1500mA limit, 200mA hysteresis, 3 deglitch, 0.88 backoff, 10%/s recovery
    fn test_compliance_config() -> ComplianceConfig {
        let cfg = LimiterConfig::new(1500, 200, 3, 225, 3277);
        ComplianceConfig::new(cfg, cfg)
    }

    /// Test policy config with standard hold duty curve.
    fn test_policy() -> PolicyConfig {
        PolicyConfig {
            hold_enter_error: CentiDeg::from_cdeg(500),
            hold_exit_error: CentiDeg::from_cdeg(700),
            hold_enter_vel: DegPerSec10::from_dps10(100),
            hold_exit_vel: DegPerSec10::from_dps10(150),
            backdrive_vel_threshold: DegPerSec10::from_dps10(300),
            backdrive_deadband: Duty::from_raw(1638),
            backdrive_persist_us: 500,
            yield_alive_duty_max: Duty::from_raw(1638),
            yield_coast_us: 100_000,
            yield_duration_us: 200_000,
            hold_duty_error_start: CentiDeg::from_cdeg(500),
            hold_duty_error_end: CentiDeg::from_cdeg(1500),
            hold_duty_min: Duty::from_raw(6553),
            hold_duty_max: Duty::from_raw(14746),
        }
    }

    #[test]
    fn test_hold_duty_cap_at_small_error() {
        let policy = test_policy();
        let cap = compute_hold_duty_cap(0, &policy);
        assert_eq!(cap, policy.hold_duty_min.as_raw());

        let cap = compute_hold_duty_cap(policy.hold_duty_error_start.as_cdeg(), &policy);
        assert_eq!(cap, policy.hold_duty_min.as_raw());
    }

    #[test]
    fn test_hold_duty_cap_at_large_error() {
        let policy = test_policy();
        let cap = compute_hold_duty_cap(policy.hold_duty_error_end.as_cdeg(), &policy);
        assert_eq!(cap, policy.hold_duty_max.as_raw());

        let cap = compute_hold_duty_cap(policy.hold_duty_error_end.as_cdeg() + 1000, &policy);
        assert_eq!(cap, policy.hold_duty_max.as_raw());
    }

    #[test]
    fn test_hold_duty_cap_ramp() {
        let policy = test_policy();
        // Midpoint should be roughly midway
        let midpoint_error =
            (policy.hold_duty_error_start.as_cdeg() + policy.hold_duty_error_end.as_cdeg()) / 2;
        let cap = compute_hold_duty_cap(midpoint_error, &policy);
        let expected_mid = (policy.hold_duty_min.as_raw() + policy.hold_duty_max.as_raw()) / 2;

        // Allow some rounding error
        assert!((cap - expected_mid).abs() <= 1);
    }

    #[test]
    fn test_switch_config() {
        let config = test_compliance_config();
        let mut state = ComplianceState::new(config.move_config);
        assert_eq!(state.mode, ServoMode::Move);

        switch_config(&mut state, ServoMode::Hold, &config);
        assert_eq!(state.mode, ServoMode::Hold);
    }
}
