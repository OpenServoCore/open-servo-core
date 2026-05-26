//! One-stop surface for chip implementers.
//!
//! The chip-side board crate implements the traits in this module to plug into
//! the kernel control loop (`KernelIo`/`Sensors`/`Motor`) and the DXL services
//! layer (`ServicesIo`/`DxlBus`/`DeviceControl`). Associated boundary types
//! (`MotorCmd`, `RxSnapshot`, `BootMode`, ...) are re-exported here so
//! `use osc_core::traits::*` covers the full contract.

// Kernel-side: what the control loop talks to.
pub use crate::kernel_io::{
    Capabilities, ConfigDefaults, DecayMode, KernelIo, Motor, MotorCmd, Sensors,
};

// Services-side: what the DXL handler talks to.
pub use crate::services::dxl::{DeviceControl, DxlBus, ServicesIo};

// Boundary types that cross either side.
pub use crate::regions::BootMode;
pub use crate::ring_reader::RxSnapshot;
pub use crate::sample_frame::{FrameInputs, RawSamples, SampleFrame};
pub use dxl_protocol::WriteBuf;
