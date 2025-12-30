//! Kernel and service operation vocabulary.
//!
//! This module defines the canonical operation types for communication between
//! async services and the RT kernel.
//!
//! ## Architecture
//!
//! Operations are split into three categories:
//!
//! - [`KernelOp`]: RT-owned, deterministic, consumed by `ControlExecutor` at tick boundary
//! - [`ServiceOp`]: async-owned, cross-task work (persist, telemetry config, debug)
//! - [`Op`]: wrapper for DXL request translation (includes local shadow reads)
//!
//! Only `KernelOp` crosses the async→RT boundary via the kernel op queue.
//! `ServiceOp` stays in async context. Shadow reads are local.
//!
//! ## Single-Outstanding Invariant
//!
//! The kernel op queue operates under **single-outstanding semantics**:
//!
//! - `dxl_req_task` is the **sole producer** of `KernelOp`
//! - Producer waits for `KernelResult` before enqueuing next op
//! - This guarantees 1:1 op-to-result correspondence without sequence IDs
//! - If another producer is added later, add correlation IDs

use crate::mode::ModeRequest;
use crate::reset::ResetScope;

/// Stable fault identifier.
///
/// This is an opaque u16 identifier for a specific fault instance,
/// not a [`FaultKind`](crate::faults::FaultKind) category.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct FaultId(pub u16);

/// Operations consumed by RT executor (deterministic, time-bounded).
///
/// These operations cross the async→RT boundary via the kernel op queue.
/// The RT executor (`ControlExecutor`) processes them at tick boundaries.
///
/// ## Invariant
///
/// Single-outstanding semantics: only one `KernelOp` in flight at a time.
/// The `dxl_req_task` is the sole producer and waits for `KernelResult`
/// before enqueuing the next op.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum KernelOp {
    /// Request a mode change.
    ModeRequest(ModeRequest),

    /// Acknowledge/clear a specific fault by its ID.
    FaultAck { id: FaultId },

    /// Acknowledge/clear all faults.
    FaultAckAll,

    /// Commit staged shadow writes to live kernel state.
    ///
    /// Host writes to control registers are staged; this op applies them.
    CommitShadow,

    /// Request a soft reset of kernel state.
    ///
    /// The reset is *deferred*: the actual reset occurs on a safe boundary.
    SoftReset(ResetScope),

    /// Ping / heartbeat (for protocol keep-alive).
    Ping,
}

/// Result from RT executor (1:1 with `KernelOp` under single-outstanding).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum KernelResult {
    /// Operation completed successfully.
    Ok,

    /// Operation refused (queue full, etc.).
    Busy,

    /// Mode change refused (invalid mode or policy).
    InvalidMode,

    /// Operation refused due to fault state.
    Faulted,

    /// Fault ack refused (not latched, or policy forbids).
    FaultAckRefused,
}

/// Operations handled by async services (cross-task work only).
///
/// These do not cross into the RT executor queue. They're handled
/// entirely in async context by signaling other service tasks.
///
/// Note: Register reads are local shadow reads, not `ServiceOp`s.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ServiceOp {
    /// Trigger EEPROM/flash persistence.
    PersistRequest,

    /// Configure telemetry streaming rate and fields.
    TelemetryConfig {
        /// Streaming rate in Hz (0 = disabled).
        rate_hz: u8,
    },
}

/// Wrapper for DXL request translation.
///
/// Used by `dxl_req_task` to classify incoming Dynamixel requests.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Op {
    /// Operation that crosses to RT executor.
    Kernel(KernelOp),

    /// Operation handled by async service tasks.
    Service(ServiceOp),

    /// Local shadow read - no cross-task work needed.
    ///
    /// Register reads are handled locally via `shadow.host_read()`.
    ShadowRead {
        /// Dynamixel control table address.
        dxl_addr: u16,
        /// Number of bytes to read.
        len: u8,
    },
}
