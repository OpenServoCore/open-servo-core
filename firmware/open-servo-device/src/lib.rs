#![no_std]
//! # open-servo-device
//!
//! Glue layer between:
//! - **Hardware** (`open-servo-hw`): board IO, sensor frames, motor commands
//! - **Kernel** (`open-servo-kernel-api`): control + register/mode host plane
//! - **Protocol services** (e.g. Dynamixel, CAN): byte parsing and host operations
//!
//! This crate is intentionally thin. It provides:
//! - a byte-oriented [`UartBus`] seam
//! - a protocol service seam [`CommsService`] (trait-only; implementation can live elsewhere)
//! - a [`Device`] runner skeleton showing how to wire board/kernel/service together
//!
//! ## What does NOT belong here
//! - Actual control logic (kernel)
//! - Board peripheral drivers (hw)
//! - Full Dynamixel implementation (can be in firmware crate or a separate `open-servo-dxl` crate)
//!
//! ## Feature flags
//! - `defmt`: enables `defmt::Format` where relevant by forwarding to deps
//! - `heapless`: enables optional helpers (future ring buffers, packet buffers, etc.)

pub mod comms_service;
pub mod device;
pub mod executor;
pub mod main_loop;
pub mod uart_bus;

// Re-export the main seams for convenience.
pub use comms_service::{
    CommsService, DxlService, EchoPolicy, HostError, HostOp, HostResp, HostResult,
};
pub use device::Device;
pub use executor::Executor;
pub use uart_bus::{UartBus, UartError};

// Common re-exports so downstream crates don't need to import everything manually.
pub use open_servo_hw as hw;
pub use open_servo_kernel_api as kernel_api;
pub use open_servo_units as units;

/// Prelude for typical device glue usage.
///
/// Prefer:
/// ```rust,ignore
/// use open_servo_device::prelude::*;
/// ```
pub mod prelude {
    pub use crate::main_loop::{drain_and_respond, parse_and_enqueue};
    pub use crate::{
        CommsService, Device, DxlService, EchoPolicy, Executor, HostError, HostOp, HostResp,
        HostResult, UartBus, UartError,
    };

    pub use open_servo_hw::*;
    pub use open_servo_kernel_api::prelude::*;
}
