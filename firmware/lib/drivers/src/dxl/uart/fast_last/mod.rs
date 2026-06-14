//! Fast Last CRC fold scheduler. Owns the periodic-walk grid that drives
//! classifier + parser + fold work during a Fast Sync / Bulk Read
//! predecessor window, and the final-anchor busy-wait that brings
//! `bytes_walked` up to `predecessor_bytes − GUARD` before the TX-start
//! body folds the residue inline.
//!
//! The FSM lives here; the CRC engine and per-byte fold callback live on
//! the [`DxlUart`] composite (next commit, alongside `crc.rs`). Wall-clock
//! ticks come through a [`Monotonic`] provider so the [`FastLastScheduler`]
//! provider stays single-purpose.
//!
//! [`DxlUart`]: super::DxlUart

use crate::traits::Monotonic;
use crate::traits::dxl::FastLastScheduler;

/// Where in the fold pipeline we currently sit.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FastLastPhase {
    /// No reply in flight.
    Idle,
    /// CC-match-driven grid: one body per `BYTES_PER_INTERVAL` step from
    /// `t_prior_start` to `final_anchor_tick`. The final body runs the
    /// busy-wait fold loop, then returns to [`Idle`] (the TX-start body
    /// owns the post-start residue).
    ///
    /// [`Idle`]: FastLastPhase::Idle
    PeriodicWalk,
}

/// Input to [`FastLast::start`]. All ticks are in scheduler-timer units —
/// the composite converts `rdt_us` → `rdt_ticks` and `byte_time` →
/// `byte_ticks` at its own seam, and translates parser-derived TIM2 u16
/// ticks into scheduler u32 using a boot-captured offset.
#[derive(Copy, Clone, Debug)]
pub struct FastLastSchedule {
    /// Parser-derived end-of-packet tick (`BT[last_byte] + 10·tpb`),
    /// projected into the scheduler's u32 domain.
    pub packet_end_tick: u32,
    /// Return-delay-time, in scheduler ticks.
    pub rdt_ticks: u16,
    /// One wire byte time at the active baud, in scheduler ticks.
    pub byte_ticks: u16,
    /// Count of wire bytes the host's request will pull from servos with
    /// earlier slots than ours, before our reply slot. The busy-wait's
    /// `predecessor_bytes − GUARD` target.
    pub predecessor_bytes: u16,
    /// Wall-clock tick at which our reply's first wire bit must land,
    /// projected into the scheduler's u32 domain.
    pub deadline_tick: u32,
}

/// FSM + grid math for the Fast Last fold pipeline.
///
/// Generic over its scheduler provider `S` and a [`Monotonic`] tick source
/// `M`. On V006 the scheduler binds to a SysTick CMP and the monotonic
/// reads the same SysTick counter (both HCLK-domain, u32).
pub struct FastLast<S: FastLastScheduler, M: Monotonic> {
    scheduler: S,
    monotonic: M,
    phase: FastLastPhase,
    /// Wall-clock anchor of the body currently armed on the scheduler. Body
    /// bumps this by `interval_ticks` before re-arming — drift in any one
    /// body's ISR-entry latency doesn't propagate to the next anchor.
    next_anchor_tick: u32,
    /// Wall-clock anchor of the terminal (busy-wait) body. Set once at
    /// `start`; on equality with `next_anchor_tick` the body switches from
    /// "advance grid" to "busy-wait and return to Idle."
    final_anchor_tick: u32,
    /// `t_prior_end − GUARD · byte_ticks` — the busy-wait's tick-side exit.
    walk_deadline_tick: u32,
    /// `BYTES_PER_INTERVAL · byte_ticks` — grid step in scheduler ticks.
    /// Cached at `start` for fast re-arming.
    interval_ticks: u32,
    /// Cached `predecessor_bytes` for the busy-wait's
    /// `bytes_walked >= predecessor_bytes − GUARD` branch.
    predecessor_bytes: u16,
}

impl<S: FastLastScheduler, M: Monotonic> FastLast<S, M> {
    pub const fn new(scheduler: S, monotonic: M) -> Self {
        Self {
            scheduler,
            monotonic,
            phase: FastLastPhase::Idle,
            next_anchor_tick: 0,
            final_anchor_tick: 0,
            walk_deadline_tick: 0,
            interval_ticks: 0,
            predecessor_bytes: 0,
        }
    }

    /// Compute the periodic-walk grid, arm the first CMP match, and flip the
    /// FSM into [`FastLastPhase::PeriodicWalk`].
    pub fn start(&mut self, p: FastLastSchedule) {
        let byte_ticks = p.byte_ticks as u32;
        let interval = S::BYTES_PER_INTERVAL as u32 * byte_ticks;
        let t_guard = S::GUARD_BYTES as u32 * byte_ticks;
        let rdt_ticks = p.rdt_ticks as u32;
        let t_prior_start = p
            .packet_end_tick
            .wrapping_add(rdt_ticks)
            .wrapping_add(byte_ticks);
        let t_prior_end = p
            .packet_end_tick
            .wrapping_add(rdt_ticks)
            .wrapping_add(p.predecessor_bytes as u32 * byte_ticks);
        let t_prior_duration = p.predecessor_bytes.saturating_sub(1) as u32 * byte_ticks;

        self.walk_deadline_tick = t_prior_end.wrapping_sub(t_guard);
        self.final_anchor_tick = if p.predecessor_bytes > S::GUARD_BYTES {
            p.deadline_tick
                .wrapping_sub(core::cmp::min(interval, t_prior_duration))
                .wrapping_sub(t_guard)
        } else {
            p.deadline_tick
        };

        // Step back by `interval` until ≤ one step from t_prior_start. The
        // i32 cast handles the small-predecessor case where `final_anchor`
        // < `t_prior_start` modular-wraps — without it the loop would step
        // back ~4G ticks.
        let mut anchor = self.final_anchor_tick;
        while (anchor.wrapping_sub(t_prior_start) as i32) >= interval as i32 {
            anchor = anchor.wrapping_sub(interval);
        }
        self.next_anchor_tick = anchor;
        self.interval_ticks = interval;
        self.predecessor_bytes = p.predecessor_bytes;
        self.phase = FastLastPhase::PeriodicWalk;

        // EVERY grid CMP back-dated.
        let cmp = anchor.wrapping_sub(S::FAST_LAST_ENTRY_TICKS as u32);
        self.scheduler.schedule(cmp);
    }

    /// Drive one grid body. Composite calls this from its SysTick demux
    /// when our CMP fires.
    ///
    /// - `walker` runs the classifier + parser-drain + per-byte fold for
    ///   whatever bytes have landed since the last body. Called at least
    ///   once per invocation.
    /// - `bytes_walked` returns the composite-side count of predecessor
    ///   bytes folded so far. The busy-wait exits when this reaches
    ///   `predecessor_bytes − GUARD`.
    pub fn on_step<W, B>(&mut self, mut walker: W, mut bytes_walked: B)
    where
        W: FnMut(),
        B: FnMut() -> u32,
    {
        match self.phase {
            FastLastPhase::PeriodicWalk => {
                walker();
                if self.next_anchor_tick == self.final_anchor_tick {
                    // Final anchor — busy-wait. Uncontested at high baud
                    // because the composite paused DMA1_CH7 HT/TC at start
                    // time, so this body is the sole HIGH consumer in the
                    // window. Exit when `bytes_walked >= predecessor_bytes −
                    // GUARD` OR `tick >= walk_deadline_tick`; the remaining
                    // GUARD bytes are absorbed by the TX-start body's tail
                    // fold.
                    let target =
                        (self.predecessor_bytes as u32).saturating_sub(S::GUARD_BYTES as u32);
                    loop {
                        walker();
                        if bytes_walked() >= target {
                            break;
                        }
                        if (self.monotonic.ticks().wrapping_sub(self.walk_deadline_tick) as i32)
                            >= 0
                        {
                            break;
                        }
                    }
                    self.phase = FastLastPhase::Idle;
                    self.scheduler.cancel();
                    return;
                }
                self.next_anchor_tick = self.next_anchor_tick.wrapping_add(self.interval_ticks);
                self.scheduler.schedule(
                    self.next_anchor_tick
                        .wrapping_sub(S::FAST_LAST_ENTRY_TICKS as u32),
                );
            }
            FastLastPhase::Idle => {
                // Spurious CMP entry — defensive cancel.
                self.scheduler.cancel();
            }
        }
    }

    /// Drop any pending CMP and return to [`FastLastPhase::Idle`]. Idempotent.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        self.phase = FastLastPhase::Idle;
    }

    pub fn is_active(&self) -> bool {
        !matches!(self.phase, FastLastPhase::Idle)
    }

    pub fn phase(&self) -> FastLastPhase {
        self.phase
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{FakeFastLastScheduler, FakeMonotonic, FastLastSchedulerOp};
    use core::cell::Cell;

    /// FAST_LAST_ENTRY_TICKS matching the mock — the back-date applied to
    /// every grid CMP.
    const ENTRY: u32 = 110;
    /// BYTES_PER_INTERVAL · byte_time at the test baud (3M: 15·160 = 2400).
    const INTERVAL_3M: u32 = 2400;
    /// 3M byte_time (10·tpb at HCLK=48M / 3MHz).
    const BYTE_TICKS_3M: u16 = 160;
    /// 3M byte_time as u32 for arithmetic on packet_end / deadline ticks.
    const BYTE_TICKS_3M_U32: u32 = BYTE_TICKS_3M as u32;

    fn fast_last() -> FastLast<FakeFastLastScheduler, FakeMonotonic> {
        FastLast::new(FakeFastLastScheduler::default(), FakeMonotonic::default())
    }

    #[test]
    fn start_schedules_first_cmp_at_anchor_minus_entry_ticks() {
        // Single-anchor case: predecessor_bytes=2 with rdt=0 lands the
        // first anchor at 2000. packet_end=2000 keeps the math in unsigned
        // range so the assertion reads naturally.
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 2000,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 2,
            deadline_tick: 2000 + 2 * BYTE_TICKS_3M_U32,
        });

        assert_eq!(d.next_anchor_tick, 2000);
        assert_eq!(d.final_anchor_tick, 2000);
        assert_eq!(
            d.scheduler.log.as_slice(),
            &[FastLastSchedulerOp::Schedule { tick: 2000 - ENTRY }]
        );
    }

    #[test]
    fn start_step_back_handles_small_predecessor_modular_wrap() {
        // packet_end=0, rdt=0, predecessor_bytes=2: final_anchor = 320 −
        // 160 − 160 = 0, and t_prior_start = 160. The signed-step-back loop
        // must stop at one iteration (0 − 160 as i32 = -160 < INTERVAL).
        // Without the i32 cast the diff is ~4G ≥ 2400 → loop would spin
        // ~4G times.
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 2,
            deadline_tick: 2 * BYTE_TICKS_3M_U32,
        });

        assert_eq!(d.next_anchor_tick, 0);
        // Exactly one Schedule entry — proves the step-back didn't spin.
        assert_eq!(d.scheduler.log.len(), 1);
    }

    #[test]
    fn intermediate_step_advances_grid_by_interval_regardless_of_tick_jitter() {
        // predecessor_bytes=50 spans three anchors (12640 / 15040 / 17440).
        // Mid-grid tick jitter must not affect the next CMP — the body
        // bumps by `interval` from `next_anchor_tick`, never from `ticks()`.
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 12000,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 50,
            deadline_tick: 12000 + 50 * BYTE_TICKS_3M_U32,
        });
        assert_eq!(d.next_anchor_tick, 12640);

        // First intermediate body: jitter the monotonic to a mid-grid
        // value; the body's next CMP must still be 15040 − ENTRY.
        d.monotonic.now = 14_999;
        d.on_step(|| {}, || 0);
        // Second intermediate body: different jitter.
        d.monotonic.now = 15_500;
        d.on_step(|| {}, || 0);

        assert_eq!(
            d.scheduler.log.as_slice(),
            &[
                FastLastSchedulerOp::Schedule {
                    tick: 12640 - ENTRY,
                },
                FastLastSchedulerOp::Schedule {
                    tick: 15040 - ENTRY,
                },
                FastLastSchedulerOp::Schedule {
                    tick: 17440 - ENTRY,
                },
            ]
        );
        // Still in PeriodicWalk — final anchor not yet entered.
        assert_eq!(d.phase(), FastLastPhase::PeriodicWalk);
        assert_eq!(d.next_anchor_tick, 17440);
    }

    #[test]
    fn final_anchor_busy_waits_until_bytes_walked_target() {
        // Single-anchor case sized so the busy-wait visibly iterates:
        // predecessor_bytes=5 with GUARD=1 → target=4. Pre-loop walker →
        // bytes=1; three loop iters bring bytes to 4 → break. Total walker
        // calls = 4.
        let mut d = fast_last();
        let deadline: u32 = 5 * BYTE_TICKS_3M_U32;
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 5,
            deadline_tick: deadline,
        });
        // next_anchor_tick should equal final_anchor_tick (single step).
        assert_eq!(d.next_anchor_tick, d.final_anchor_tick);
        // walk_deadline_tick = deadline − GUARD·byte. Park tick well before
        // it so the deadline branch can't fire.
        let walk_deadline_tick = deadline - BYTE_TICKS_3M_U32;
        d.monotonic.now = walk_deadline_tick - 100;

        let walker_calls = Cell::new(0u32);
        let bytes = Cell::new(0u32);
        d.on_step(
            || {
                walker_calls.set(walker_calls.get() + 1);
                bytes.set(bytes.get() + 1);
            },
            || bytes.get(),
        );

        assert_eq!(walker_calls.get(), 4);
        assert_eq!(d.phase(), FastLastPhase::Idle);
        assert!(matches!(
            d.scheduler.log.last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn final_anchor_busy_waits_until_walk_deadline_when_bytes_lag() {
        let mut d = fast_last();
        let deadline: u32 = 3 * BYTE_TICKS_3M_U32;
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 3,
            deadline_tick: deadline,
        });
        let walk_deadline_tick = deadline - BYTE_TICKS_3M_U32;
        // Park tick AT walk_deadline_tick so the deadline branch catches on
        // the first loop iteration.
        d.monotonic.now = walk_deadline_tick;

        let walker_calls = Cell::new(0u32);
        d.on_step(
            || walker_calls.set(walker_calls.get() + 1),
            || 0_u32, // bytes never arrive
        );

        // Pre-loop walker + one inside the loop before the deadline branch
        // catches: 2 calls.
        assert_eq!(walker_calls.get(), 2);
        assert_eq!(d.phase(), FastLastPhase::Idle);
    }

    #[test]
    fn final_anchor_returns_to_idle_and_cancels_scheduler_after_busy_wait() {
        let mut d = fast_last();
        let deadline: u32 = 2 * BYTE_TICKS_3M_U32;
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 2,
            deadline_tick: deadline,
        });
        let bytes = Cell::new(0u32);
        d.on_step(|| bytes.set(bytes.get() + 1), || bytes.get());

        assert_eq!(d.phase(), FastLastPhase::Idle);
        assert!(!d.is_active());
        // Log: Schedule (from start) then Cancel (from on_step exit).
        assert_eq!(d.scheduler.log.len(), 2);
        assert!(matches!(d.scheduler.log[1], FastLastSchedulerOp::Cancel));
    }

    #[test]
    fn cancel_from_periodic_walk_returns_to_idle() {
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 12000,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 50,
            deadline_tick: 12000 + 50 * BYTE_TICKS_3M_U32,
        });
        assert_eq!(d.phase(), FastLastPhase::PeriodicWalk);

        d.cancel();

        assert_eq!(d.phase(), FastLastPhase::Idle);
        assert!(matches!(
            d.scheduler.log.last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    /// Bench-traced regression at 3M GUARD=1 predecessor_bytes=14 — pins
    /// the grid math against a known-good trace. Single-step grid case
    /// (`t_prior_duration < interval`): `final_anchor_tick` and the first
    /// anchor land at the wall-clock anchor (12000), the scheduled CMP is
    /// `anchor − ENTRY = 11890`, `walk_deadline_tick = req_end + 14080`.
    #[test]
    fn walk_deadline_grid_correctness_3m_predecessor_bytes_14() {
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 12000,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 14,
            deadline_tick: 12000 + 14 * BYTE_TICKS_3M_U32,
        });

        assert_eq!(d.walk_deadline_tick, 14080);
        assert_eq!(d.final_anchor_tick, 12000);
        assert_eq!(d.next_anchor_tick, 12000);
        assert_eq!(d.interval_ticks, INTERVAL_3M);
        assert_eq!(
            d.scheduler.log.as_slice(),
            &[FastLastSchedulerOp::Schedule {
                tick: 12000 - ENTRY,
            }]
        );
    }
}
