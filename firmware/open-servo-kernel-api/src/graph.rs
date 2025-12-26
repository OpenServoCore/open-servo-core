//! Static node-graph composition helpers.
//!
//! This module provides a *compile-time* way to structure “the node graph” without
//! introducing a dynamic graph runtime.
//!
//! ## Design goals
//! - Keep signal flow explicit via [`crate::wired::Wired`] + kernel-owned shared `State`.
//! - Allow kernels to treat a pipeline as a first-class value: a “graph” you can run.
//! - Enable composition (chain, mode switch) without heap allocation or dynamic dispatch.
//!
//! ## What this is NOT
//! - Not a scheduler (timers/ISRs/rates live in board/kernel crates)
//! - Not a dynamic graph (no runtime node registry)
//! - Not an event bus
//!
//! ## Why this exists
//! In real firmware, “the control graph” often becomes a pile of calls scattered across
//! the codebase. This module encourages an explicit artifact:
//!
//! - `fast_graph: impl Graph<CtlFast, State>`
//! - `medium_graph: impl Graph<CtlMedium, State>`
//! - `slow_graph: impl Graph<CtlSlow, State>`
//! - `sys_graph: impl Graph<Sys, State>`
//!
//! Then the kernel just calls `graph.run(state, ctx)`.
//!
//! ## Composition model
//! - A single `Wired` node is a `Graph` (adapter impl).
//! - `Chain<A, B>` runs A then B against the same shared state.
//! - `SwitchByMode<P, O>` selects which graph to run based on [`OperatingMode`].
//!
//! This is intentionally simple, because simplicity is the feature.

use crate::mode::OperatingMode;
use crate::reset::{ResetReason, Resettable};
use crate::tick_ctx::TickCtx;
use crate::wired::Wired;
use crate::{FaultSink, TelemetrySink};

/// A statically composed runnable graph for domain marker `M` over shared state `S`.
///
/// A `Graph<M, S>` is “something the kernel can run” for a tick domain.
/// It is intentionally minimal and does not define scheduling policy.
///
/// Contract:
/// - The graph may read and write `S` (kernel-owned shared state).
/// - Time advances only when `run()` is called with a `TickCtx` (no hidden timers).
/// - Side effects (fault raise, telemetry) happen during `run()` via the `TickCtx`.
///
/// Note: Graphs are usually made of [`Wired`] nodes; see the blanket impl below.
pub trait Graph<M, S>: Resettable {
    fn run<F, T>(&mut self, state: &mut S, ctx: &mut TickCtx<'_, F, T>)
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized;
}

/// Adapter: any `Wired` node is also a `Graph`.
///
/// This is the core bridge that lets you build higher-order graphs.
impl<M, S, N> Graph<M, S> for N
where
    N: Wired<M, S> + Resettable,
{
    #[inline]
    fn run<F, T>(&mut self, state: &mut S, ctx: &mut TickCtx<'_, F, T>)
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        // Canonical wired call order:
        // 1) pull input from state
        // 2) step (update + tick)
        // 3) push output back to state
        let input = self.pull(state);
        let out = self.step(ctx, input);
        self.push(state, &out);
    }
}

/// Sequential composition: run `a` then `b` (same domain, same shared state).
///
/// This is the “pipeline” primitive.
///
/// Example (conceptual):
/// - estimator -> controller -> limiter -> driver command
pub struct Chain<A, B> {
    pub a: A,
    pub b: B,
}

impl<A, B> Chain<A, B> {
    #[inline]
    pub const fn new(a: A, b: B) -> Self {
        Self { a, b }
    }
}

impl<A, B> Resettable for Chain<A, B>
where
    A: Resettable,
    B: Resettable,
{
    #[inline]
    fn reset(&mut self, reason: ResetReason) {
        // Order is explicit and stable. If order matters for your components,
        // this gives you one place to define it.
        self.a.reset(reason);
        self.b.reset(reason);
    }
}

impl<M, S, A, B> Graph<M, S> for Chain<A, B>
where
    A: Graph<M, S>,
    B: Graph<M, S>,
{
    #[inline]
    fn run<F, T>(&mut self, state: &mut S, ctx: &mut TickCtx<'_, F, T>)
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        self.a.run(state, ctx);
        self.b.run(state, ctx);
    }
}

/// Mode-based graph selection.
///
/// This encodes the recommended “different graphs per mode” design:
/// - avoid pushing `OperatingMode` into every node
/// - instead, select which graph runs
///
/// `SwitchByMode` is intentionally small; policy about *when* mode changes are allowed
/// belongs in the kernel crate.
pub struct SwitchByMode<P, O> {
    pub position: P,
    pub open_loop: O,
}

impl<P, O> SwitchByMode<P, O> {
    #[inline]
    pub const fn new(position: P, open_loop: O) -> Self {
        Self {
            position,
            open_loop,
        }
    }

    /// Run the graph for the current `mode`.
    ///
    /// This does not change mode; it only selects which graph executes.
    #[inline]
    pub fn run_mode<M, S, F, T>(
        &mut self,
        mode: OperatingMode,
        state: &mut S,
        ctx: &mut TickCtx<'_, F, T>,
    ) where
        P: Graph<M, S>,
        O: Graph<M, S>,
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        match mode {
            OperatingMode::Position => self.position.run(state, ctx),
            OperatingMode::OpenLoop => self.open_loop.run(state, ctx),
        }
    }
}

impl<P, O> Resettable for SwitchByMode<P, O>
where
    P: Resettable,
    O: Resettable,
{
    #[inline]
    fn reset(&mut self, reason: ResetReason) {
        // By default, reset both sub-graphs.
        // Kernel policy can decide when to call reset and with what reason.
        self.position.reset(reason);
        self.open_loop.reset(reason);
    }
}

/// Ergonomic extension methods for building graphs.
///
/// This is a tiny builder API that stays purely compile-time.
pub trait GraphExt: Sized {
    /// Chain two graphs/pipelines: `a.then(b)` runs `a` then `b`.
    #[inline]
    fn then<G>(self, g: G) -> Chain<Self, G> {
        Chain::new(self, g)
    }
}

impl<T> GraphExt for T {}
