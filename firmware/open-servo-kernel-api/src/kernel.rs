//! Kernel traits (API boundary).
//!
//! Split into two planes:
//! - [`Kernel`]: real-time control plane (called from tick schedule)
//! - [`KernelHost`]: control/config plane (called from comms / host services)
//!
//! This split keeps the tick path minimal and stable, and prevents comms concerns
//! (register map, protocol errors, mode requests) from polluting real-time scheduling.

use crate::{
    host_op::{HostOp, HostResult},
    tick_ctx::TickCtx,
    FaultSink, TelemetrySink,
};

/// Real-time kernel contract.
///
/// The kernel consumes the latest sensor frame via [`update_frame`], and advances time via [`tick`].
///
/// - The kernel does **not** own hardware.
/// - The kernel does **not** parse protocols.
/// - The kernel is the "brain": gating, control loops, supervisor state, etc.
///
/// ### One tick, many domains
/// This trait uses a **single** `tick()` entry point. Domain selection is done by
/// inspecting `ctx.tick.domain`.
///
/// This keeps the boundary small while still enabling multi-rate pipelines.
pub trait Kernel {
    /// Board → kernel sensor input "frame".
    ///
    /// This is typically the type returned by `open-servo-hw::Board::read_sensors()`.
    type Frame: Copy + 'static;

    /// Kernel → board actuator command.
    ///
    /// This is typically `open-servo-hw::io::MotorCommand`.
    type Command: Copy + 'static;

    /// Update the kernel with the latest sensor frame.
    ///
    /// **Must not** advance time. Think of this like "load inputs / last-sample".
    /// The kernel may compute derived values, but should not integrate over time here.
    fn update_frame(&mut self, frame: Self::Frame);

    /// Advance the kernel by one scheduled tick.
    ///
    /// - Timing facts are provided by `ctx.tick` (domain/dt/seq).
    /// - Fault/telemetry side effects happen here.
    ///
    /// Returns a motor command. Many kernels only produce "fresh" commands on
    /// `ControlFast` ticks and return the last command for other domains.
    fn tick<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> Self::Command
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized;
}

/// Host/control-plane interface.
///
/// Protocol services (Dynamixel, CAN, USB, etc.) call [`apply_op`] to execute
/// control-plane operations against the kernel.
///
/// This trait provides a single dispatch point for all host operations,
/// replacing the legacy `reg_read`/`reg_write`/`request_mode` methods.
///
/// # Single-Writer Contract
///
/// `apply_op` may only be called from one context at a time.
/// Boards must ensure mutual exclusion (critical sections, priority masking).
/// This contract allows implementations to avoid atomics.
///
/// [`apply_op`]: KernelHost::apply_op
pub trait KernelHost {
    /// Apply a host operation and return the result.
    ///
    /// This is the primary dispatch point for all control-plane operations.
    /// Protocol adapters should use this rather than individual methods.
    ///
    /// # Operations
    ///
    /// See [`HostOp`] for the full set of supported operations:
    /// - Register read/write
    /// - Mode requests
    /// - Fault acknowledgment
    /// - Persistence commit
    /// - Soft reset
    /// - Ping
    fn apply_op(&mut self, op: HostOp) -> HostResult;
}
