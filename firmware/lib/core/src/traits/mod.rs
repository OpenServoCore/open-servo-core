//! One-stop surface for chip implementers.
//!
//! The chip-side board crate implements these traits to plug into the kernel
//! control loop (`ControlIo`/`Sensors`/`Motor`) and the bus services layer
//! (`Dispatch`/`Reply`). Associated boundary types are re-exported here so
//! `use osc_core::traits::*` covers the full contract.

mod control;
mod services;

pub use control::{Capabilities, ControlIo, DecayMode, Motor, MotorCmd, Sensors};
pub use services::{Dispatch, Reply, Request, RequestCtx, SendError, Speculated, Status};

// Boundary types defined elsewhere that callers of the traits need in scope.
pub use crate::regions::BootMode;
pub use crate::regions::config::ConfigDefaults;
pub use crate::sample::{ConversionVariables, RawSamples, Sample};
