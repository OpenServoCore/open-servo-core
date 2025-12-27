//! Communications service seam (trait-only).
//!
//! This module defines the generic [`CommsService`] trait for protocol services
//! (Dynamixel, CAN, etc.) that need to interact with the kernel host plane.
//!
//! The service is a state machine that:
//! - ingests RX bytes
//! - emits host operations (reg read/write, mode request)
//! - receives host responses
//! - exposes TX bytes for a reply packet
//!
//! ## Protocol Independence
//!
//! `Device` uses [`CommsService`] without knowing the underlying protocol.
//! Dynamixel implementations should implement this trait; CAN or other protocols
//! can do the same with appropriate adapters.

use open_servo_kernel_api::{
    mode::ModeRequest,
    regs::{RegAddr, RegValue},
};

/// Operations requested by the protocol layer.
///
/// The device runner executes these against the kernel host-plane (`KernelHost`)
/// and then feeds results back to the service as [`HostResp`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum HostOp {
    /// Read a register address.
    RegRead { addr: RegAddr },

    /// Write a register address.
    RegWrite { addr: RegAddr, value: RegValue },

    /// Request a mode change.
    ModeRequest(ModeRequest),
}

/// Result of applying a [`HostOp`].
///
/// The service translates errors into protocol-specific error codes.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum HostResp {
    /// Register read result.
    RegValue(RegValue),

    /// Successful completion for writes/mode requests.
    Ok,

    /// Generic failure.
    Err,
}

/// Preference for how to handle single-wire TX echo.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum EchoPolicy {
    /// Keep RX enabled; the service filters echoed bytes in software.
    FilterEcho,
    /// Disable RX during TX; simplest on hardware that supports it.
    DisableRxDuringTx,
}

/// Generic communications service contract.
///
/// Pull-driven design:
/// - Device drains UART RX and calls `ingest_rx_byte()`
/// - Service produces `HostOp` via `next_op()`
/// - Device executes op against `KernelHost` and calls `push_resp()`
/// - Service exposes reply bytes via `tx_pop()`
/// - Device sends bytes using `UartBus`
///
/// This design keeps ISR work minimal and keeps policy in one place.
///
/// ## Implementing for Dynamixel
///
/// Dynamixel protocol implementations should implement this trait directly.
/// The `DxlService` type alias is provided for clarity.
pub trait CommsService {
    /// Preferred echo handling strategy for this service implementation.
    fn echo_policy(&self) -> EchoPolicy;

    /// Feed one RX byte.
    fn ingest_rx_byte(&mut self, b: u8);

    /// Get the next requested host operation, if any.
    fn next_op(&mut self) -> Option<HostOp>;

    /// Provide response for the last host op.
    fn push_resp(&mut self, resp: HostResp);

    /// Pop one TX byte to send, if any.
    fn tx_pop(&mut self) -> Option<u8>;

    /// Returns true if the service currently has bytes waiting to transmit.
    fn tx_pending(&self) -> bool;

    /// Notify the service that TX is fully complete and the line is released.
    fn notify_tx_complete(&mut self);
}

/// Type alias for Dynamixel protocol implementations.
///
/// Dynamixel services implement [`CommsService`] directly; this alias
/// provides naming clarity for Dynamixel-specific code.
pub trait DxlService: CommsService {}

/// Blanket impl: any `CommsService` is also a `DxlService`.
impl<T: CommsService> DxlService for T {}
