#![no_std]
//! # open-servo-kernel-api
//!
//! Hard API boundary shared by kernel/controller/board crates.
//!
//! ## What belongs here
//! - **Contracts** (traits) that multiple crates must agree on
//! - **Vocabulary types** (units, IO structs, enums) shared across boundaries
//! - Small, stable helper shapes (register op enums, reset reasons)
//!
//! ## What does NOT belong here
//! - Concrete scheduling policy (timers, divisors, ISR layout)
//! - Feature implementations (thermal models, safety logic, compliance FSM)
//! - Heavy dependencies
//! ## Signal flow / wiring philosophy
//!
//! This API is designed around **explicit, kernel-owned signal flow**:
//!
//! - The **kernel owns shared state** (a `State`/`Bus` struct in the kernel crate).
//! - Nodes are **pure scheduled components**: they cache inputs in `update()` and advance in `tick()`.
//! - Signals between nodes are plain **value wires** (usually `Copy` unit newtypes).
//!
//! This crate intentionally does **not** provide a graph engine or runtime wiring.
//! Instead, it provides:
//!
//! - [`Node<M>`] as the foundational scheduled contract
//! - [`Mailbox<T>`] as a tiny cross-domain “wire” primitive for kernel-owned state
//! - [`Wired<M, S>`] as the **blessed wiring pattern** (`pull`/`push`) that kernels may choose to require
//!
//! The goal is to make dataflow reviewable and to avoid hidden coupling such as
//! nodes holding references/handles into shared state.
//!
//! ## Modules overview
//! - [`tick`] / [`ticks`]: tick-domain vocabulary + `Node<M>` contract
//! - [`tick_ctx`]: per-tick context (time + sinks)
//! - [`io`]: board/kernel IO shapes using unit newtypes
//! - [`controller`]: controller algorithm boundary (controllers are not nodes)
//! - [`faults`]: fault vocabulary + sink boundary
//! - [`reset`]: optional lifecycle hook (`Resettable`)
//! - [`role`]: required node intent labeling (`HasRole` / `Role`)
//! - [`mode`]: minimal operating mode vocabulary
//! - [`timebase`]: monotonic time source boundary
//! - [`rates`]: declared domain rates (diagnostics/system-id metadata)
//! - [`telemetry`]: structured numeric telemetry sink for system identification
//! - [`mailbox`]: tiny cross-domain “wire” primitive (kernel-owned state fields)
//! - [`wired`]: optional wiring contract (`pull`/`push`) + helper runner
//!
//! ## Units
//! This crate re-exports `open-servo-units` as the shared “vocabulary” crate:
//! - You can `use open_servo_kernel_api::units::*;`
//! - Or `use open_servo_kernel_api::{MilliAmp, CentiDeg32, Effort, ...};`

pub mod controller;
pub mod debug_guard;
pub mod faults;
pub mod graph;
pub mod host_op;
pub mod io;
pub mod kernel;
pub mod mailbox;
pub mod mode;
pub mod rates;
pub mod reset;
pub mod role;
pub mod shadow;
pub mod telemetry;
pub mod tick;
pub mod tick_ctx;
pub mod ticks;
pub mod timebase;
pub mod wired;

// Re-export units as both a module and glob for convenience.
pub use open_servo_units as units;
pub use open_servo_units::*;

// Commonly used exports (optional but convenient).
pub use controller::Controller;
pub use debug_guard::DebugReentrancyGuard;
pub use faults::{FaultKind, FaultSink, GateReason};
pub use graph::{Chain, Graph, GraphExt, SwitchByMode};
pub use host_op::{FaultId, HostError, HostOp, HostResp, HostResult};
pub use kernel::{Kernel, KernelHost};
pub use mailbox::Mailbox;
pub use mode::{ModeError, ModeRequest, OperatingMode};
pub use rates::DomainRatesHz;
pub use reset::{ResetReason, ResetScope, Resettable};
pub use role::{HasRole, Role};
pub use shadow::{CommitResult, FieldDesc, KernelView, ShadowKernel, ShadowTable};
pub use telemetry::{TelemetryId, TelemetrySink};
pub use tick::{Tick, TickDomain};
pub use tick_ctx::TickCtx;
pub use ticks::{CtlFast, CtlMedium, CtlSlow, KernelCtx, ModeView, Node, Sys};
pub use timebase::{TimeStampUs, Timebase};
pub use wired::{run_wired, Wired};

/// Convenience imports for most users of the API boundary.
///
/// Prefer `use open_servo_kernel_api::prelude::*;` in kernel and board crates.
pub mod prelude {
    pub use crate::controller::Controller;
    pub use crate::debug_guard::DebugReentrancyGuard;
    pub use crate::faults::{FaultKind, FaultSink, GateReason};
    pub use crate::graph::{Chain, Graph, GraphExt, SwitchByMode};
    pub use crate::host_op::{FaultId, HostError, HostOp, HostResp, HostResult};
    pub use crate::kernel::{Kernel, KernelHost};
    pub use crate::mailbox::Mailbox;
    pub use crate::mode::{ModeError, ModeRequest, OperatingMode};
    pub use crate::rates::DomainRatesHz;
    pub use crate::reset::{ResetReason, ResetScope, Resettable};
    pub use crate::role::{HasRole, Role};
    pub use crate::shadow::{CommitResult, FieldDesc, KernelView, ShadowKernel, ShadowTable};
    pub use crate::telemetry::{TelemetryId, TelemetrySink};
    pub use crate::tick::{Tick, TickDomain};
    pub use crate::tick_ctx::TickCtx;
    pub use crate::ticks::{CtlFast, CtlMedium, CtlSlow, KernelCtx, ModeView, Node, Sys};
    pub use crate::timebase::{TimeStampUs, Timebase};
    pub use crate::units::*;
    pub use crate::wired::{run_wired, Wired};
}
