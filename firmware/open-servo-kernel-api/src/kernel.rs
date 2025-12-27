//! Kernel traits (API boundary).
//!
//! Split into two planes:
//! - [`Kernel`]: real-time control plane (called from tick schedule)
//! - [`KernelHost`]: control/config plane (called from comms / host services)
//!
//! This split keeps the tick path minimal and stable, and prevents comms concerns
//! (register map, protocol errors, mode requests) from polluting real-time scheduling.

use crate::{
    mode::{ModeError, ModeRequest},
    regs::{RegAddr, RegError, RegValue},
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
/// This exists so a comms service (Dynamixel, CAN, USB, etc.) can:
/// - read/write registers
/// - request mode changes
///
/// without needing access to the concrete kernel type.
pub trait KernelHost {
    /// Read a register-like value.
    fn reg_read(&self, addr: RegAddr) -> Result<RegValue, RegError>;

    /// Write a register-like value.
    fn reg_write(&mut self, addr: RegAddr, value: RegValue) -> Result<(), RegError>;

    /// Request a mode change.
    ///
    /// Mode switching policy is kernel-defined. The request may be accepted,
    /// deferred, or rejected (error).
    fn request_mode(&mut self, req: ModeRequest) -> Result<(), ModeError>;
}
