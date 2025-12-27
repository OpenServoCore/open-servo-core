//! Communications service seam (trait-only).
//!
//! This module defines the generic [`CommsService`] trait for protocol services
//! (Dynamixel, CAN, etc.) that need to interact with the kernel host plane.
//!
//! The service is a state machine that:
//! - ingests RX bytes
//! - emits host operations (reg read/write, mode request, etc.)
//! - receives host responses
//! - exposes TX bytes for a reply packet
//!
//! ## Protocol Independence
//!
//! `Device` uses [`CommsService`] without knowing the underlying protocol.
//! Dynamixel implementations should implement this trait; CAN or other protocols
//! can do the same with appropriate adapters.
//!
//! ## Host Operations
//!
//! Host operations are defined in [`open_servo_kernel_api::host_op`].
//! This module re-exports the types for convenience, but the canonical
//! definition lives in kernel-api.

// Re-export host operation types from kernel-api (canonical source).
pub use open_servo_kernel_api::host_op::{HostError, HostOp, HostResp, HostResult};

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
/// - Device executes op against `KernelHost` and calls `push_result()`
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

    /// Provide result for the last host op.
    ///
    /// The service implementation is responsible for mapping `HostResult`
    /// to protocol-specific response packets (including error codes).
    fn push_result(&mut self, result: HostResult);

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
