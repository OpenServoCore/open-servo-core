//! ControlExecutor: single-writer kernel runner for ADC ISR context.
//!
//! The [`ControlExecutor`] owns the kernel and runs all tick domains from a single
//! ADC DMA completion ISR. This enforces the single-writer contract:
//!
//! - Only the ADC ISR calls `kernel.tick()` and `kernel.apply_op()`
//! - Main loop only produces [`HostOp`]s and consumes [`HostResult`]s via queues
//!
//! ## Decimation
//!
//! The executor handles decimation from fast tick rate to medium and slow:
//!
//! | Domain | Rate | Trigger |
//! |--------|------|---------|
//! | ControlFast | 10kHz | Every ADC completion |
//! | ControlMedium | 1kHz | Every 10th fast tick |
//! | ControlSlow | 200Hz | Every 5th medium tick (50th fast tick) |
//!
//! Host operations are applied at the ControlSlow boundary with a bounded budget.
//!
//! ## Thread Safety
//!
//! Queue operations are wrapped in explicit critical sections for portability.
//! Note: heapless spsc queues are lock-free on MCUs with native atomics (Cortex-M3+),
//! but some targets (CH32V006) lack hardware atomics and use critical-section
//! emulation via `portable-atomic`. The explicit CS wrappers ensure consistent
//! behavior across all targets - optimize code for the lowest common denominator.

use heapless::spsc::{Consumer, Producer};

use open_servo_hw::v2::io::{MotorCommand, SensorFrame};
use open_servo_kernel_api::{
    host_op::{HostOp, HostResult},
    kernel::{Kernel, KernelHost},
    shadow::ShadowKernel,
    tick_ctx::TickCtx,
    ticks::KernelCtx,
    timebase::TimeStampUs,
    FaultSink, TelemetrySink,
};
use open_servo_units::MicroSecond;

use crate::shadow_storage::ShadowStorage;

/// Decimation ratio: ControlFast → ControlMedium (10kHz → 1kHz).
pub const MEDIUM_DECIMATE: u8 = 10;

/// Decimation ratio: ControlMedium → ControlSlow (1kHz → 200Hz).
pub const SLOW_DECIMATE: u8 = 5;

/// Maximum host operations to apply per ControlSlow tick.
///
/// At 200Hz slow tick rate, this gives max 800 ops/sec throughput.
pub const MAX_OPS_PER_SLOW: u8 = 4;

/// Single-writer kernel executor for ADC ISR context.
///
/// The executor owns the kernel and sequence counters. It runs all tick domains
/// from the ADC completion ISR and applies host operations at safe boundaries.
///
/// # Generic Parameters
///
/// - `K`: Kernel type implementing both [`Kernel`] and [`KernelHost`]
/// - `OP_CAP`: Capacity of the op queue (e.g., 16)
/// - `RESULT_CAP`: Capacity of the result queue (e.g., 16)
pub struct ControlExecutor<K> {
    /// The kernel (owns all control state).
    kernel: K,

    /// Tick sequence counters.
    kctx: KernelCtx,

    /// Decimation counter: fast → medium (counts 0..MEDIUM_DECIMATE-1).
    medium_counter: u8,

    /// Decimation counter: medium → slow (counts 0..SLOW_DECIMATE-1).
    slow_counter: u8,

    /// Pending result from last slow tick if result queue was full.
    ///
    /// This ensures we never drop a sideband command response.
    /// If set, we try to enqueue it before processing more ops.
    pending_result: Option<HostResult>,
}

impl<K> ControlExecutor<K>
where
    K: Kernel<Frame = SensorFrame, Command = MotorCommand> + KernelHost + ShadowKernel,
{
    /// Create a new executor with the given kernel.
    #[inline]
    pub fn new(kernel: K) -> Self {
        Self {
            kernel,
            kctx: KernelCtx::new(),
            medium_counter: 0,
            slow_counter: 0,
            pending_result: None,
        }
    }

    /// ADC ISR entrypoint: run one fast tick with decimated medium/slow ticks.
    ///
    /// This is the **only** function that should call `kernel.tick()` or `kernel.apply_op()`.
    /// It runs at the ADC completion rate (typically 10kHz).
    ///
    /// # Parameters
    ///
    /// - `frame`: Latest sensor readings from ADC
    /// - `fast_dt_us`: Delta time for fast tick (e.g., 100µs for 10kHz)
    /// - `now`: Monotonic timestamp for cross-domain correlation
    /// - `op_cons`: Consumer end of op queue (main → ADC ISR)
    /// - `result_prod`: Producer end of result queue (ADC ISR → main)
    /// - `faults`: Fault sink for raising faults
    /// - `telem`: Telemetry sink for metrics
    ///
    /// # Returns
    ///
    /// The motor command produced by the fast tick.
    ///
    /// # Queue Safety
    ///
    /// All queue operations are wrapped in critical sections for safety
    /// across ISR/main priority boundaries.
    pub fn on_adc_complete<F, T, const OP_CAP: usize, const RESULT_CAP: usize, const N: usize>(
        &mut self,
        frame: SensorFrame,
        fast_dt_us: MicroSecond,
        now: TimeStampUs,
        op_cons: &mut Consumer<'_, HostOp, OP_CAP>,
        result_prod: &mut Producer<'_, HostResult, RESULT_CAP>,
        faults: &mut F,
        telem: &mut T,
        shadow: &ShadowStorage<N>,
    ) -> MotorCommand
    where
        F: FaultSink,
        T: TelemetrySink,
    {
        // Update kernel with latest sensor frame (no time advance).
        self.kernel.update_frame(frame);

        // Run ControlFast tick.
        let tick = self.kctx.next_control_fast(fast_dt_us);
        let mut ctx = TickCtx::new(tick, now, faults, telem);
        let cmd = self.kernel.tick(&mut ctx);

        // Decimation: ControlMedium.
        self.medium_counter += 1;
        if self.medium_counter >= MEDIUM_DECIMATE {
            self.medium_counter = 0;

            let medium_dt = MicroSecond(fast_dt_us.0 * MEDIUM_DECIMATE as u32);
            let tick = self.kctx.next_control_medium(medium_dt);
            let mut ctx = TickCtx::new(tick, now, faults, telem);
            let _ = self.kernel.tick(&mut ctx);

            // Medium tick boundary: publish telemetry + commit shadow if dirty.
            // SAFETY: on_adc_complete is called from ISR context only.
            shadow.kernel_with_view(|view| {
                // Publish telemetry (does NOT mark dirty).
                self.kernel.publish_telemetry(view);

                // Commit dirty shadow→live if any host writes pending.
                if view.ctrl_dirty() {
                    let _ = self.kernel.commit_shadow(view);
                }
            });

            // Decimation: ControlSlow.
            self.slow_counter += 1;
            if self.slow_counter >= SLOW_DECIMATE {
                self.slow_counter = 0;

                let slow_dt = MicroSecond(medium_dt.0 * SLOW_DECIMATE as u32);
                let tick = self.kctx.next_control_slow(slow_dt);
                let mut ctx = TickCtx::new(tick, now, faults, telem);
                let _ = self.kernel.tick(&mut ctx);

                // Apply pending host operations at slow tick boundary.
                self.apply_pending_ops(op_cons, result_prod);
            }
        }

        cmd
    }

    /// Apply pending host operations with backpressure handling.
    ///
    /// This is called at the ControlSlow boundary (200Hz) with a bounded budget.
    /// If the result queue is full, we stash the result and stop processing
    /// to avoid dropping any responses.
    fn apply_pending_ops<const OP_CAP: usize, const RESULT_CAP: usize>(
        &mut self,
        op_cons: &mut Consumer<'_, HostOp, OP_CAP>,
        result_prod: &mut Producer<'_, HostResult, RESULT_CAP>,
    ) {
        // 1) First, try to flush any pending result from last tick.
        if let Some(result) = self.pending_result.take() {
            if cs_enqueue(result_prod, result).is_err() {
                // Still full, put it back and stop.
                self.pending_result = Some(result);
                return;
            }
        }

        // 2) Process ops up to budget.
        for _ in 0..MAX_OPS_PER_SLOW {
            if let Some(op) = cs_dequeue(op_cons) {
                let result = self.kernel.apply_op(op);
                if cs_enqueue(result_prod, result).is_err() {
                    // Result queue full: stash and stop.
                    self.pending_result = Some(result);
                    return;
                }
            } else {
                break; // No more ops.
            }
        }
    }

    /// Access the kernel (read-only) for diagnostics.
    #[inline]
    pub fn kernel(&self) -> &K {
        &self.kernel
    }

    /// Access the kernel context for sequence queries.
    #[inline]
    pub fn kctx(&self) -> &KernelCtx {
        &self.kctx
    }
}

/// Dequeue from a Consumer atomically.
///
/// heapless spsc is lock-free on MCUs with native atomics (Cortex-M3+), but
/// CH32V006 lacks hardware atomics and uses CS emulation via `portable-atomic`.
/// This explicit wrapper ensures consistent behavior across all targets.
#[inline]
fn cs_dequeue<T, const N: usize>(cons: &mut Consumer<'_, T, N>) -> Option<T> {
    critical_section::with(|_| cons.dequeue())
}

/// Enqueue to a Producer atomically.
///
/// heapless spsc is lock-free on MCUs with native atomics (Cortex-M3+), but
/// CH32V006 lacks hardware atomics and uses CS emulation via `portable-atomic`.
/// This explicit wrapper ensures consistent behavior across all targets.
#[inline]
fn cs_enqueue<T, const N: usize>(prod: &mut Producer<'_, T, N>, val: T) -> Result<(), T> {
    critical_section::with(|_| prod.enqueue(val))
}
