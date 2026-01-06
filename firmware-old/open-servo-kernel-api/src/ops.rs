//! Kernel operation vocabulary.
//!
//! This module defines [`KernelOp`] for communication between async services
//! and the RT kernel.
//!
//! ## Architecture
//!
//! [`KernelOp`] values cross the async→RT boundary via the kernel op queue.
//! They are consumed by `ControlExecutor` at tick boundaries.
//!
//! Shadow reads/writes are handled directly via `ServiceOps` methods.
//! Async tasks (persist, telemetry) are signaled directly, not via ops.
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
