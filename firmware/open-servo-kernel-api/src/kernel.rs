//! Kernel traits (API boundary).
//!
//! Split into two planes:
//! - [`Kernel`]: real-time control plane (called from tick schedule)
//! - [`KernelHost`]: control/config plane (called from RT executor at tick boundary)
//!
//! This split keeps the tick path minimal and stable, and prevents comms concerns
//! (register map, protocol errors, mode requests) from polluting real-time scheduling.

use crate::{
    ops::{KernelOp, KernelResult},
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
/// The RT executor (`ControlExecutor`) calls [`apply_op`] at tick boundaries
/// to execute kernel operations queued by async services.
///
/// # Single-Outstanding Contract
///
/// Operations follow single-outstanding semantics:
/// - Only one `KernelOp` in flight at a time
/// - Producer waits for `KernelResult` before enqueuing next op
/// - This guarantees 1:1 op-to-result correspondence
///
/// [`apply_op`]: KernelHost::apply_op
pub trait KernelHost {
    /// Apply a kernel operation and return the result.
    ///
    /// Called by the RT executor at tick boundaries (typically slow tick).
    ///
    /// # Operations
    ///
    /// See [`KernelOp`] for the full set:
    /// - Mode requests
    /// - Fault acknowledgment
    /// - Shadow commit
    /// - Soft reset
    /// - Ping
    fn apply_op(&mut self, op: KernelOp) -> KernelResult;
}
