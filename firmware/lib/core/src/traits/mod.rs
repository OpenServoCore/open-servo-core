//! One-stop surface for chip implementers.
//!
//! The chip-side board crate implements these traits to plug into the kernel
//! control loop (`ControlIo`/`Sensors`/`Motor`) and the DXL services layer
//! (`DxlBus`/`DxlReply`). Associated boundary types are re-exported here so
//! `use osc_core::traits::*` covers the full contract.

mod control;
mod services;

pub use control::{Capabilities, ControlIo, DecayMode, Motor, MotorCmd, Sensors};
pub use services::{DxlBus, DxlReply};

// Boundary types defined elsewhere that callers of the traits need in scope.
pub use crate::regions::BootMode;
pub use crate::regions::config::ConfigDefaults;
pub use crate::sample::{ConversionVariables, RawSamples, Sample};
