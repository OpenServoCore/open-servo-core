//! Hierarchical position limits (sensor → mechanical → user).
//!
//! Wraps the kinematics LimitSystem to provide setpoint clamping
//! and validation for the servo core.

use crate::kinematics::{LimitSystem, MechanicalLimits, SensorLimits, UserLimits};
use open_servo_math::CentiDeg32;

/// Limits configuration.
///
/// Lives in CoreConfig.limits
#[derive(Debug, Clone)]
pub struct LimitsConfig {
    /// The hierarchical limit system from kinematics
    pub limit_system: LimitSystem,
}

impl LimitsConfig {
    /// Create a new LimitsConfig with the given limits.
    ///
    /// Returns None if validation fails (user limits outside mechanical or inverted).
    pub fn new(
        sensor: SensorLimits,
        mechanical: MechanicalLimits,
        user: UserLimits,
    ) -> Option<Self> {
        let system = LimitSystem::new(sensor, mechanical, user);
        if !system.validate() {
            return None; // User limits outside mechanical or inverted
        }
        Some(Self {
            limit_system: system,
        })
    }

    /// Clamp a setpoint to user limits.
    pub fn clamp_setpoint(&self, sp: CentiDeg32) -> CentiDeg32 {
        self.limit_system.clamp_to_user(sp)
    }

    /// Get current user limits as (min, max) in centidegrees.
    pub fn user_limits(&self) -> (i32, i32) {
        (
            self.limit_system.user.min_cdeg.0,
            self.limit_system.user.max_cdeg.0,
        )
    }

    /// Get current mechanical limits as (min, max) in centidegrees.
    pub fn mechanical_limits(&self) -> (i32, i32) {
        (
            self.limit_system.mechanical.min_cdeg.0,
            self.limit_system.mechanical.max_cdeg.0,
        )
    }

    /// Set new user limits.
    ///
    /// Returns false if new limits are invalid (outside mechanical or inverted),
    /// leaving config unchanged.
    pub fn set_user_limits(&mut self, min: i32, max: i32) -> bool {
        let new_user = UserLimits::new(min, max);
        let new_system = LimitSystem::new(
            self.limit_system.sensor,
            self.limit_system.mechanical,
            new_user,
        );
        if !new_system.validate() {
            return false;
        }
        self.limit_system = new_system;
        true
    }
}

/// Validate that a limits configuration is valid.
///
/// Pure function for checking without creating a LimitsConfig.
pub fn validate_config(config: &LimitsConfig) -> bool {
    config.limit_system.validate()
}

/// Clamp a setpoint to user limits.
///
/// Pure function that takes config by reference.
pub fn clamp_setpoint(config: &LimitsConfig, sp: CentiDeg32) -> CentiDeg32 {
    config.clamp_setpoint(sp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_returns_none_for_inverted_limits() {
        // User max < min
        let result = LimitsConfig::new(
            SensorLimits::new(0, 36000),
            MechanicalLimits::new(0, 36000),
            UserLimits::new(20000, 10000), // Inverted
        );
        assert!(result.is_none());
    }

    #[test]
    fn test_new_returns_none_if_user_outside_mechanical() {
        // User min below mechanical min
        let result = LimitsConfig::new(
            SensorLimits::new(0, 36000),
            MechanicalLimits::new(5000, 30000),
            UserLimits::new(0, 25000), // min below mechanical
        );
        assert!(result.is_none());

        // User max above mechanical max
        let result = LimitsConfig::new(
            SensorLimits::new(0, 36000),
            MechanicalLimits::new(5000, 30000),
            UserLimits::new(10000, 35000), // max above mechanical
        );
        assert!(result.is_none());
    }

    #[test]
    fn test_new_valid_limits() {
        let result = LimitsConfig::new(
            SensorLimits::new(0, 36000),
            MechanicalLimits::new(5000, 30000),
            UserLimits::new(10000, 25000),
        );
        assert!(result.is_some());
    }

    #[test]
    fn test_clamp_setpoint_to_user_limits() {
        let config = LimitsConfig::new(
            SensorLimits::new(0, 36000),
            MechanicalLimits::new(5000, 30000),
            UserLimits::new(10000, 25000),
        )
        .unwrap();

        // Below min
        assert_eq!(clamp_setpoint(&config, CentiDeg32(5000)).0, 10000);

        // Above max
        assert_eq!(clamp_setpoint(&config, CentiDeg32(28000)).0, 25000);

        // Within range
        assert_eq!(clamp_setpoint(&config, CentiDeg32(15000)).0, 15000);
    }

    #[test]
    fn test_set_user_limits_rejects_invalid() {
        let mut config = LimitsConfig::new(
            SensorLimits::new(0, 36000),
            MechanicalLimits::new(5000, 30000),
            UserLimits::new(10000, 25000),
        )
        .unwrap();

        // Try to set inverted limits
        assert!(!config.set_user_limits(25000, 10000));
        // Original should be preserved
        assert_eq!(config.user_limits(), (10000, 25000));

        // Try to set outside mechanical
        assert!(!config.set_user_limits(0, 20000));
        assert_eq!(config.user_limits(), (10000, 25000));
    }

    #[test]
    fn test_set_user_limits_accepts_valid() {
        let mut config = LimitsConfig::new(
            SensorLimits::new(0, 36000),
            MechanicalLimits::new(5000, 30000),
            UserLimits::new(10000, 25000),
        )
        .unwrap();

        assert!(config.set_user_limits(15000, 20000));
        assert_eq!(config.user_limits(), (15000, 20000));
    }
}
