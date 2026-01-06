#![no_std]
//! # open-servo-runtime
//!
//! Application framework layer between:
//! - **Hardware** (`open-servo-hw`): board IO, sensor frames, motor commands
//! - **Kernel** (`open-servo-kernel-api`): control + register/mode host plane
//! - **Protocol services** (e.g. Dynamixel, CAN): byte parsing and host operations
//!
//! This crate owns:
//! - Resource management (buffers, queues, pools)
//! - Protocol timing (half-duplex turnaround, etc.)
//! - Runtime orchestration
//!
//! ## Crate Layering
//!
//! | Crate | Responsibility |
//! |-------|----------------|
//! | **hw** | Trait definitions only (Board, Timebase) |
//! | **runtime** | Resource ownership, protocol timing, orchestration |
//! | **firmware-*** | Hardware init, provides `impl Board` |
//!
//! ## Feature flags
//! - `defmt`: enables `defmt::Format` where relevant by forwarding to deps

pub mod comms_service;
pub mod device;
pub mod executor;
pub mod main_loop;
pub mod runtime;
pub mod service_primitives;
pub mod services;
pub mod shadow_storage;
pub mod uart_bus;

// Re-export the main seams for convenience.
pub use comms_service::{CommsService, DxlService, EchoPolicy, KernelOp, KernelResult};
pub use device::Device;
pub use executor::ControlExecutor;
pub use runtime::{Runtime, SHADOW_TABLE_SIZE};
pub use service_primitives::ServicePrimitives;
pub use shadow_storage::{HeaplessStagingBuffer, ShadowStorage};
// Re-export HostShadow trait from shadow crate for services
pub use open_servo_shadow::HostShadow;
pub use uart_bus::{UartBus, UartError};

// Common re-exports so downstream crates don't need to import everything manually.
pub use open_servo_hw as hw;
pub use open_servo_kernel_api as kernel_api;
pub use open_servo_registry as registry;
pub use open_servo_units as units;

/// Initialize the runtime subsystems.
///
/// **Note:** If using [`Runtime`], call `Runtime::init()` instead, which
/// handles this automatically along with queue setup.
///
/// Safe to call multiple times (subsequent calls are no-ops).
pub fn init() {
    open_servo_registry::init();
}

/// Prelude for typical device glue usage.
///
/// Prefer:
/// ```rust,ignore
/// use open_servo_runtime::prelude::*;
/// ```
pub mod prelude {
    pub use crate::main_loop::{drain_and_respond, parse_and_enqueue};
    pub use crate::{
        CommsService, ControlExecutor, Device, DxlService, EchoPolicy, HeaplessStagingBuffer,
        KernelOp, KernelResult, ShadowStorage, UartBus, UartError,
    };

    pub use open_servo_hw::*;
    pub use open_servo_kernel_api::prelude::*;
}
