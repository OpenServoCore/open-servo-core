//! Core servo control logic.
//!
//! This crate provides the platform-agnostic servo controller core:
//!
//! - `ServoCore`: Pure control logic with no hardware dependencies
//! - `App`: Hardware orchestrator that wires ServoCore to hardware traits
//! - `SafetyManager`: Consolidated safety monitoring
//! - `FaultState`: Fault detection and latching
//! - `DebugShell`: Interactive debug REPL (requires `debug-shell` feature)
//! - `EventQueue`: Lock-free event passing from ISRs to main loop

#![no_std]
#![forbid(unsafe_code)]

pub mod app;
#[cfg(feature = "debug-shell")]
pub mod debug_shell;
pub mod event;
pub mod fault;
pub mod inputs;
pub mod outputs;
pub mod safety;
pub mod servo_core;
#[cfg(test)]
mod test_harness;
#[cfg(test)]
mod test_support;
pub mod tick;

// Re-export commonly used items
pub use app::App;
#[cfg(feature = "debug-shell")]
pub use debug_shell::DebugShell;
pub use event::{Event, EventConsumer, EventProducer, EventQueue};
pub use fault::{FaultKind, FaultState};
pub use inputs::FastInputs;
pub use outputs::FastOutputs;
pub use safety::{SafetyManager, SafetyThresholds, SensorHealth};
pub use servo_core::{ServoCore, SystemState};
pub use tick::{TickCtx, TickDomain};
