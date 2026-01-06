//! Safety primitives for servo control.
//!
//! This module provides low-level safety building blocks:
//! - `SafetyThresholds`: Configurable limits for current, temperature, position
//! - `SensorHealth`: Tracks sensor validity and detects faulty readings
//! - `ComplianceLimiter`: Current-aware output limiting with anti-windup
//! - `ThermalFaultDetector`: Hysteresis-based temperature fault detection
//!
//! Safety logic is orchestrated by `servo_core::features::safety` which uses
//! these primitives with pure functions operating on `SafetyState`.

mod compliance_limiter;
mod sensor_health;
mod thermal_fault;
mod thresholds;

pub use compliance_limiter::ComplianceLimiter;
pub use sensor_health::*;
pub use thermal_fault::*;
pub use thresholds::*;
