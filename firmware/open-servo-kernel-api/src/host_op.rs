//! Host operation vocabulary (control-plane commands).
//!
//! This module defines the canonical set of operations that can be
//! requested by protocol services (Dynamixel, CAN, USB, etc.) and
//! executed against the kernel via [`KernelHost::apply_op`].
//!
//! ## Design
//!
//! - [`HostOp`] is the sum of all control-plane operations.
//! - [`HostResp`] contains success-only responses.
//! - [`HostError`] contains typed errors.
//! - [`HostResult`] is the idiomatic `Result<HostResp, HostError>`.
//!
//! Protocol adapters produce `HostOp` values and consume `HostResult`.
//! The kernel dispatches via [`KernelHost::apply_op`].
//!
//! [`KernelHost::apply_op`]: crate::kernel::KernelHost::apply_op

use crate::mode::{ModeError, ModeRequest};
use crate::regs::{RegAddr, RegError, RegValue};

// Re-export ResetScope for convenience (canonical definition is in reset.rs).
pub use crate::reset::ResetScope;

/// Stable fault identifier (not category).
///
/// This is an opaque u16 identifier for a specific fault instance,
/// not a [`FaultKind`](crate::faults::FaultKind) category.
///
/// Protocol layers use this to acknowledge specific faults.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct FaultId(pub u16);

/// Host-plane operations (control plane → kernel).
///
/// These are the commands that protocol services can request.
/// The kernel executes them via [`KernelHost::apply_op`].
///
/// [`KernelHost::apply_op`]: crate::kernel::KernelHost::apply_op
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum HostOp {
    /// Read a register address.
    RegRead { addr: RegAddr },

    /// Write a register address.
    RegWrite { addr: RegAddr, value: RegValue },

    /// Request a mode change.
    ModeRequest(ModeRequest),

    /// Acknowledge/clear a specific fault by its ID.
    FaultAck { id: FaultId },

    /// Acknowledge/clear all faults.
    FaultAckAll,

    /// Request commit of dirty config to persistent storage.
    ///
    /// The kernel schedules the write; actual EEPROM/flash IO happens
    /// outside the kernel boundary.
    PersistCommit,

    /// Request a soft reset of kernel state.
    ///
    /// The reset is *deferred*: `apply_op` sets a pending flag and the
    /// actual reset occurs on a safe boundary (System or Slow tick).
    SoftReset(ResetScope),

    /// Ping / heartbeat (for protocol keep-alive).
    Ping,
}

/// Successful host operation responses.
///
/// This enum contains only success cases. Errors are in [`HostError`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum HostResp {
    /// Acknowledgment (writes, mode requests, acks, reset, persist).
    Ack,

    /// Register read result.
    RegValue(RegValue),

    /// Pong response to Ping.
    Pong,
}

/// Host operation errors.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum HostError {
    /// Register access error.
    Reg(RegError),

    /// Mode change refused.
    Mode(ModeError),

    /// Operation not supported by this kernel.
    UnsupportedOp,

    /// Fault ack refused (not latched, or policy forbids).
    FaultAck,

    /// Persist commit refused (e.g., torque enabled).
    PersistBusy,

    /// Operation refused due to current system state.
    Busy,
}

/// Result type for host operations.
pub type HostResult = Result<HostResp, HostError>;

impl From<RegError> for HostError {
    #[inline]
    fn from(e: RegError) -> Self {
        HostError::Reg(e)
    }
}

impl From<ModeError> for HostError {
    #[inline]
    fn from(e: ModeError) -> Self {
        HostError::Mode(e)
    }
}
