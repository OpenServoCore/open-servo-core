//! CoreConfig - Writable configuration owned by features.
//!
//! All servo configuration lives here, organized by feature.

use crate::servo_core::features::{ComplianceConfig, LimitsConfig, SafetyConfig, ThermalConfig};

/// Configuration for the servo core.
///
/// Contains all tunable parameters organized by feature.
#[derive(Debug, Clone)]
pub struct CoreConfig {
    /// Position limits (from kinematics)
    pub limits: LimitsConfig,
    /// Safety thresholds
    pub safety: SafetyConfig,
    /// Compliance mode configs (move/hold)
    pub compliance: ComplianceConfig,
    /// Thermal model parameters
    pub thermal: ThermalConfig,
}

impl CoreConfig {
    /// Create a new CoreConfig with given feature configs.
    pub fn new(
        limits: LimitsConfig,
        safety: SafetyConfig,
        compliance: ComplianceConfig,
        thermal: ThermalConfig,
    ) -> Self {
        Self {
            limits,
            safety,
            compliance,
            thermal,
        }
    }
}
