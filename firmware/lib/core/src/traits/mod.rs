//! One-stop surface for chip implementers.
//!
//! The chip-side board crate implements these traits to plug into the kernel
//! control loop (`KernelIo`/`Sensors`/`Motor`) and the DXL services layer
//! (`ServicesIo`/`DxlBus`/`DeviceControl`). Associated boundary types are
//! re-exported here so `use osc_core::traits::*` covers the full contract.

mod kernel;
mod services;

pub use kernel::{Capabilities, DecayMode, KernelIo, Motor, MotorCmd, Sensors};
pub use services::{DeviceControl, DxlBus, ServicesIo};

// Boundary types defined elsewhere that callers of the traits need in scope.
pub use crate::regions::BootMode;
pub use crate::regions::config::ConfigDefaults;
pub use crate::ring_reader::RxSnapshot;
pub use crate::sample_frame::{FrameInputs, RawSamples, SampleFrame};
pub use dxl_protocol::WriteBuf;
