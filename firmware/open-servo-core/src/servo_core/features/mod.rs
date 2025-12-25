//! Feature modules for servo core functionality.
//!
//! Each feature owns its Config and State subtrees:
//! - safety: Sensor health, fault counters
//! - limits: Position limits (wraps kinematics.rs)
//! - compliance: ComplianceLimiter, mode switching
//! - yield_: Yield timing and phases
//! - thermal: Thermal model and fault detection

// Feature modules will be added in Commit 2:
// pub mod safety;
// pub mod limits;
// pub mod compliance;
// pub mod yield_;
// pub mod thermal;
