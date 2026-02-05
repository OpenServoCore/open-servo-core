use core::marker::PhantomData;

use open_servo_units::MicroSecond;

use crate::tick::{Tick, TickDomain};
use crate::tick_ctx::TickCtx;
use crate::{FaultSink, HasRole, Resettable, TelemetrySink};

/// Type-level domain markers.
///
/// These markers are used to select the correct `Node<M>` impl at compile time.
/// They are the “type-level counterpart” to [`TickDomain`].
///
/// Why do we need both?
/// - [`TickDomain`] is runtime data carried in [`Tick`] / [`TickCtx`]
/// - These marker types pick the correct trait impl (e.g. a multi-rate node can implement
///   `Node<CtlFast>` and `Node<CtlSlow>` simultaneously)
///
/// We intentionally mirror [`TickDomain`] 1:1 to avoid ambiguity.
pub enum CtlFast {}
pub enum CtlMedium {}
pub enum CtlSlow {}
pub enum Sys {}

/// Marker trait for types that can be used as node inputs/outputs.
///
/// This enforces the project’s “value wiring” discipline:
/// - inputs/outputs are plain values (`Copy`)
/// - inputs/outputs do not borrow from kernel-owned state (`'static`)
///
/// In practice, `WireValue` types are usually unit newtypes (e.g. `CentiDeg32`, `MilliAmp`,
/// `Effort`) or small `Copy` structs/tuples of those.
///
/// If you need to share data across nodes, put it in kernel-owned shared state and pass
/// the *values* through `update()`/`tick()`.
pub trait WireValue: Copy + 'static {}
impl<T: Copy + 'static> WireValue for T {}

/// Unified node contract specialized by a type-level domain marker `M`.
///
/// A `Node<M>` is a scheduled component in the tick pipeline.
///
/// ## Contract (enforced)
/// - **Role:** every node must declare a [`Role`](crate::role::Role) via [`HasRole`](crate::role::HasRole).
///   This is a design-time prompt: it forces implementers to think “what is this node?”
///   (control, filter, model, supervisor, IO, etc.).
///
/// - **Reset:** every node must implement [`Resettable`]. This is intentionally *not* optional.
///   Most nodes accumulate some hidden state (integrators, windows, FSMs, debouncers).
///   Requiring `Resettable` forces authors to think about reset behavior on:
///   disengage, engage, mode changes, fault edges, and config changes.
///   If a node truly has nothing to reset, use the `impl_reset_nop!` helper macro.
///
/// - **Value wiring:** `In`/`Out` must be [`WireValue`] (`Copy + 'static`).
///   Nodes should not hold references into kernel state, nor accept borrowed inputs.
///   This keeps signal flow explicit and reviewable.
///
/// - `update(input)` ingests the latest input values. It **must not advance time**.
/// - `tick(ctx)` advances the node by `ctx.tick.dt` and may:
///   - raise faults via `ctx.faults` (write-only; fault state is kernel-owned)
///   - emit telemetry via `ctx.telem`
/// - Prefer `step(ctx, input)` at call sites for readability.
///
/// ### Why separate `update` and `tick`?
/// Inputs can change 0..N times between ticks (register writes, commands, mailbox updates),
/// while ticks are clock-driven. Separating them:
/// - prevents accidental double-integration of time
/// - allows batching/overwriting inputs cheaply (last-write-wins)
/// - keeps side effects (faults/telemetry) at known times (`tick`)
///
/// ## Wiring model (recommended)
/// Nodes are wired through **kernel-owned shared state**:
/// - Kernel reads shared state and calls `update(input)`
/// - Kernel calls `tick(ctx)` on schedule
/// - Kernel writes returned output back into shared state
///
/// This keeps nodes self-contained and makes signal flow explicit.
/// Cross-domain values (slow→fast setpoints, budgets, etc.) should live in kernel state
/// (optionally using a mailbox pattern) rather than ad-hoc mailboxes hidden inside nodes.
///
/// ## Context and sinks (no allocation)
/// `tick()` receives a `TickCtx` that provides:
/// - `tick`: timing facts (`domain`, `dt`, `seq`)
/// - `now`: monotonic timestamp (for windows/timeouts)
/// - `faults`: a fault sink (nodes may raise faults; clearing/gating is kernel policy)
/// - `telem`: a telemetry sink for structured numeric signals (system-id)
///
/// Note: `TickCtx` is passed by mutable reference and is not storable by nodes
/// (in safe Rust). This prevents “squirreling away” handles to kernel services.
///
/// ## Example: simplest node
/// ```rust,ignore
/// use open_servo_kernel_api::{Node, CtlFast};
/// use open_servo_kernel_api::role::{HasRole, Role};
/// use open_servo_kernel_api::reset::{Resettable, ResetReason};
///
/// struct Constant;
///
/// impl HasRole for Constant {
///     const ROLE: Role = Role::Control;
/// }
///
/// impl Resettable for Constant {
///     fn reset(&mut self, _reason: ResetReason) {}
/// }
///
/// impl Node<CtlFast> for Constant {
///     type In = ();
///     type Out = i32;
///
///     fn update(&mut self, _input: ()) {}
///
///     fn tick<F, T>(&mut self, _ctx: &mut open_servo_kernel_api::TickCtx<'_, F, T>) -> i32
///     where
///         F: open_servo_kernel_api::FaultSink + ?Sized,
///         T: open_servo_kernel_api::TelemetrySink + ?Sized,
///     {
///         42
///     }
/// }
/// ```
///
/// ## Example: a node with internal state + reset
/// ```rust,ignore
/// use open_servo_kernel_api::{Node, CtlFast};
/// use open_servo_kernel_api::reset::{Resettable, ResetReason};
/// use open_servo_kernel_api::role::{HasRole, Role};
///
/// struct Integrator { acc: i32 }
///
/// impl HasRole for Integrator {
///     const ROLE: Role = Role::Filter;
/// }
///
/// impl Resettable for Integrator {
///     fn reset(&mut self, _reason: ResetReason) {
///         self.acc = 0;
///     }
/// }
///
/// impl Node<CtlFast> for Integrator {
///     type In = i32;
///     type Out = i32;
///
///     fn update(&mut self, input: i32) {
///         // Store the latest input; do not advance time here.
///         self.acc = input;
///     }
///
///     fn tick<F, T>(&mut self, ctx: &mut open_servo_kernel_api::TickCtx<'_, F, T>) -> i32
///     where
///         F: open_servo_kernel_api::FaultSink + ?Sized,
///         T: open_servo_kernel_api::TelemetrySink + ?Sized,
///     {
///         // Optional: sanity check domain in debug builds.
///         ctx.debug_assert_domain(open_servo_kernel_api::TickDomain::ControlFast);
///         self.acc
///     }
/// }
/// ```
pub trait Node<M>: HasRole + Resettable {
    type In: WireValue;
    type Out: WireValue;

    fn update(&mut self, input: Self::In);
    fn tick<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>) -> Self::Out
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized;

    /// Common-case convenience: update then tick.
    #[inline]
    fn step<F, T>(&mut self, ctx: &mut TickCtx<'_, F, T>, input: Self::In) -> Self::Out
    where
        F: FaultSink + ?Sized,
        T: TelemetrySink + ?Sized,
    {
        self.update(input);
        self.tick(ctx)
    }
}

/// Kernel-facing tick context manager.
///
/// `KernelCtx` is a **sequencer and binder factory**, not a scheduler mandate.
///
/// Responsibilities:
/// - track per-domain sequence counters (`Tick.seq`)
/// - construct [`Tick`] values for each domain (`next_control_fast`, etc.)
/// - provide ergonomic "binders" (`on_control_fast`, etc.) so multi-rate nodes can be
///   called without UFCS noise
///
/// Non-goals (intentionally not part of this crate):
/// - choosing timer allocation / ISR layout
/// - deciding per-domain rates or divisors
/// - selecting what nodes run in which order
///
/// The **board/kernel** decides when to call these methods (rates/timers/divisors).
///
/// # Single-Writer Contract
///
/// `KernelCtx` must only be accessed by one writer at a time. Typical usage:
/// - Board owns `KernelCtx` in its Device/System struct
/// - ISRs call `next_*` methods through critical section or priority masking
///
/// This crate does NOT use atomics for sequence counters. The single-writer
/// contract must be enforced by the board.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, Debug)]
pub struct KernelCtx {
    seq_fast: u32,
    seq_medium: u32,
    seq_slow: u32,
    seq_system: u32,
}

/// Generate `next_*` method for a domain.
macro_rules! impl_next_tick {
    ($method:ident, $seq_field:ident, $domain:expr, $doc:expr) => {
        #[doc = $doc]
        #[inline]
        pub fn $method(&mut self, dt: MicroSecond) -> Tick {
            self.$seq_field = self.$seq_field.wrapping_add(1);
            Tick {
                domain: $domain,
                dt,
                seq: self.$seq_field,
            }
        }
    };
}

/// Generate `on_*` binder method for a domain.
macro_rules! impl_on_domain {
    ($method:ident, $marker:ty, $doc:expr) => {
        #[doc = $doc]
        #[inline]
        pub fn $method<'a, N>(&'a mut self, node: &'a mut N) -> ModeView<'a, N, $marker>
        where
            N: Node<$marker>,
        {
            ModeView::new(node)
        }
    };
}

impl KernelCtx {
    #[inline]
    pub const fn new() -> Self {
        Self {
            seq_fast: 0,
            seq_medium: 0,
            seq_slow: 0,
            seq_system: 0,
        }
    }

    /// Get current sequence for a domain (without advancing).
    ///
    /// This is useful for diagnostics, checking for missed ticks, or
    /// correlating events across domains.
    #[inline]
    pub fn current_seq(&self, domain: TickDomain) -> u32 {
        match domain {
            TickDomain::ControlFast => self.seq_fast,
            TickDomain::ControlMedium => self.seq_medium,
            TickDomain::ControlSlow => self.seq_slow,
            TickDomain::System => self.seq_system,
        }
    }

    impl_next_tick!(
        next_control_fast,
        seq_fast,
        TickDomain::ControlFast,
        "Next tick context for ControlFast."
    );
    impl_next_tick!(
        next_control_medium,
        seq_medium,
        TickDomain::ControlMedium,
        "Next tick context for ControlMedium."
    );
    impl_next_tick!(
        next_control_slow,
        seq_slow,
        TickDomain::ControlSlow,
        "Next tick context for ControlSlow."
    );
    impl_next_tick!(
        next_system,
        seq_system,
        TickDomain::System,
        "Next tick context for System."
    );

    // ---- Explicit binders (ergonomics + compile-time domain selection) ----

    impl_on_domain!(
        on_control_fast,
        CtlFast,
        "Bind a node to the ControlFast domain."
    );
    impl_on_domain!(
        on_control_medium,
        CtlMedium,
        "Bind a node to the ControlMedium domain."
    );
    impl_on_domain!(
        on_control_slow,
        CtlSlow,
        "Bind a node to the ControlSlow domain."
    );
    impl_on_domain!(on_system, Sys, "Bind a node to the System domain.");
}

/// Typed view that pins a node reference to a specific type-level domain.
///
/// This is purely ergonomics:
/// - avoids UFCS when a node implements multiple `Node<M>` impls
/// - makes call sites read like: `k.on_control_fast(&mut node).step(ctx, input)`
///
/// Safety/debugging:
/// - In debug builds, `ModeView` checks:
///   - node role is compatible with the runtime domain (`debug_assert_role_domain`)
///   - the provided `TickCtx` carries the expected `TickDomain`
///
/// These checks are meant to catch wiring mistakes early:
/// passing a `System` tick context into a `ControlFast` node, etc.
pub struct ModeView<'a, N, M> {
    node: &'a mut N,
    _m: PhantomData<M>,
}

impl<'a, N, M> ModeView<'a, N, M> {
    #[inline]
    fn new(node: &'a mut N) -> Self {
        Self {
            node,
            _m: PhantomData,
        }
    }
}

/// Generate `ModeView` impl for a specific domain.
macro_rules! impl_mode_view {
    ($marker:ty, $domain:expr) => {
        impl<'a, N> ModeView<'a, N, $marker>
        where
            N: Node<$marker>,
        {
            #[inline]
            pub fn update(self, input: <N as Node<$marker>>::In) {
                self.node.update(input);
            }

            #[inline]
            pub fn tick<F, T>(self, ctx: &mut TickCtx<'_, F, T>) -> <N as Node<$marker>>::Out
            where
                F: FaultSink + ?Sized,
                T: TelemetrySink + ?Sized,
            {
                $crate::debug_assert_role_domain!(N, $domain);
                ctx.debug_assert_domain($domain);
                self.node.tick(ctx)
            }

            /// Convenience call: `update(input)` + `tick(ctx)` with domain validation.
            #[inline]
            pub fn step<F, T>(
                self,
                ctx: &mut TickCtx<'_, F, T>,
                input: <N as Node<$marker>>::In,
            ) -> <N as Node<$marker>>::Out
            where
                F: FaultSink + ?Sized,
                T: TelemetrySink + ?Sized,
            {
                $crate::debug_assert_role_domain!(N, $domain);
                ctx.debug_assert_domain($domain);
                self.node.step(ctx, input)
            }
        }
    };
}

impl_mode_view!(CtlFast, TickDomain::ControlFast);
impl_mode_view!(CtlMedium, TickDomain::ControlMedium);
impl_mode_view!(CtlSlow, TickDomain::ControlSlow);
impl_mode_view!(Sys, TickDomain::System);
