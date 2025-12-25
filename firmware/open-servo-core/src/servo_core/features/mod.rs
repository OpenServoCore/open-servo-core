//! Feature modules for servo core functionality.
//!
//! Each feature owns its Config and State subtrees:
//! - safety: Sensor health, fault counters
//! - limits: Position limits (wraps kinematics.rs)
//! - compliance: ComplianceLimiter, mode switching
//! - backdrive: Backdrive window timing and phases
//! - thermal: Thermal model and fault detection

pub mod backdrive;
pub mod compliance;
pub mod limits;
pub mod safety;
pub mod thermal;

// Re-export types for convenience
pub use backdrive::{BackdrivePhase, BackdriveState};
pub use compliance::{ComplianceConfig, ComplianceState};
pub use limits::LimitsConfig;
pub use safety::{SafetyConfig, SafetyState};
pub use thermal::{ThermalConfig, ThermalState};
