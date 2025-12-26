//! Wiring helper contract: kernel-owned shared state drives inputs/outputs.
//!
//! This module provides an **optional** (but strongly recommended) pattern for
//! connecting nodes through a shared `State` (or `Bus`) struct **owned by the kernel**.
//!
//! ## Why this exists
//! In embedded control codebases, signal flow can become implicit and hard to audit
//! (nodes reach into global state, hold indices/handles, or exchange ad-hoc mailboxes).
//! This API nudges the opposite: **explicit wiring** where the kernel is the only
//! place that “connects the dots”.
//!
//! ## The pattern
//! The kernel remains the orchestrator:
//! - `pull()` builds the node input from shared state
//! - `tick()` advances the node in time (faults/telemetry happen here via `TickCtx`)
//! - `push()` writes the node output back into shared state
//!
//! This keeps nodes self-contained, testable, and makes the dataflow reviewable.
//!
//! ## Optional here, enforceable by kernels
//! This trait is optional in `open-servo-kernel-api` because this crate does **not**
//! define a concrete kernel `State` type. Kernel implementations may choose to make
//! it mandatory by only scheduling `Wired<M, State>` nodes.
//!
//! ## Guidelines
//! - `pull()` should be **cheap** and **side-effect free** (just read state and assemble `In`).
//! - `push()` should only **write outputs** into state (no extra computation if possible).
//! - Cross-domain “wires” (slow→fast setpoints, budgets, etc.) should live in kernel state
//!   and may be represented with [`Mailbox<T>`] if you need staging/sequence tracking.
//!
//! ## Example
//! ```rust,ignore
//! use open_servo_kernel_api::{Wired, run_wired, CtlFast};
//! use open_servo_kernel_api::role::{HasRole, Role};
//! use open_servo_kernel_api::units::{Effort, DegPerSec10};
//!
//! pub struct State {
//!     pub vel_sp: DegPerSec10,
//!     pub vel: DegPerSec10,
//!     pub effort: Effort,
//! }
//!
//! pub struct VelControllerNode;
//!
//! impl HasRole for VelControllerNode {
//!     const ROLE: Role = Role::Control;
//! }
//!
//! impl open_servo_kernel_api::Node<CtlFast> for VelControllerNode {
//!     type In = (DegPerSec10, DegPerSec10);
//!     type Out = Effort;
//!
//!     fn update(&mut self, _input: Self::In) {}
//!
//!     fn tick<F, T>(&mut self, _ctx: &mut open_servo_kernel_api::TickCtx<'_, F, T>) -> Self::Out
//!     where
//!         F: open_servo_kernel_api::FaultSink + ?Sized,
//!         T: open_servo_kernel_api::TelemetrySink + ?Sized,
//!     {
//!         Effort::ZERO
//!     }
//! }
//!
//! impl Wired<CtlFast, State> for VelControllerNode {
//!     fn pull(&mut self, state: &State) -> <Self as open_servo_kernel_api::Node<CtlFast>>::In {
//!         (state.vel_sp, state.vel)
//!     }
//!
//!     fn push(&mut self, state: &mut State, out: &Effort) {
//!         state.effort = *out;
//!     }
//! }
//!
//! // In the kernel tick:
//! // run_wired::<CtlFast, _, _, _, _>(&mut node, &mut state, &mut ctx);
//! ```

use crate::{FaultSink, Node, TelemetrySink, TickCtx};

/// Blessed wiring contract for kernels that want to enforce explicit signal flow.
///
/// `Wired<M, S>` describes how a node connects to kernel-owned shared state `S`.
/// The intent is that nodes do **not** hold references/indices/handles into shared state;
/// instead, the kernel pulls values, ticks nodes, and pushes results back.
///
/// Many kernels will choose to schedule only `Wired` nodes to make this pattern mandatory.
/// Keeping it as a separate trait (rather than baking `S` into `Node<M>`) avoids infecting
/// every node with a shared-state generic before a kernel `State` type exists.
pub trait Wired<M, S>: Node<M> {
    /// Build this node’s input from shared state.
    ///
    /// Contract:
    /// - Should be cheap and side-effect free.
    /// - Should not advance time (no internal integration here).
    /// - Prefer assembling small `Copy` “wire” values (unit newtypes, small structs).
    fn pull(&mut self, state: &S) -> Self::In;

    /// Write this node’s output back into shared state.
    ///
    /// Contract:
    /// - Should only write results into `state`.
    /// - Avoid additional computation when possible (keep compute in `tick()`).
    fn push(&mut self, state: &mut S, out: &Self::Out);
}

/// Run a wired node using the blessed “kernel owns state” pattern.
///
/// This helper encodes the canonical call order:
/// 1) `pull()` input from shared state
/// 2) `step()` (update + tick) to advance the node
/// 3) `push()` output back into shared state
///
/// Kernels can wrap this to create domain-specific schedulers
/// (e.g. `run_fast`, `run_slow`) and can choose to accept only `Wired` nodes.
#[inline]
pub fn run_wired<M, S, N, F, T>(node: &mut N, state: &mut S, ctx: &mut TickCtx<'_, F, T>) -> N::Out
where
    N: Wired<M, S>,
    F: FaultSink + ?Sized,
    T: TelemetrySink + ?Sized,
{
    let input = node.pull(state);
    let out = node.step(ctx, input);
    node.push(state, &out);
    out
}
