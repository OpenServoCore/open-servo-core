//! Periodic-walk grid FSM for the Fast Last fold pipeline. Owns the CMP
//! grid that paces classifier + parser + fold work across a Fast Sync /
//! Bulk Read predecessor window, plus the final-step busy-wait that brings
//! `bytes_walked` up to `predecessor_bytes − GUARD` before the TX-start body
//! folds the residue inline.
//!
//! Half of the [`FastLast`] sub-composite (§4.3): the scheduling half. The
//! fold half is [`FoldEngine`]. This half works in absolute u32 deadlines
//! (WireClock domain) — `start()` composes the packet-end anchor with
//! protocol-derived offsets and hands the provider one u32 per arm. The
//! chip-side provider applies the deadlines directly with no further lift.
//!
//! [`FastLast`]: super::FastLast
//! [`FoldEngine`]: super::fold_engine::FoldEngine

use super::schedule::FastLastSchedule;
use crate::traits::dxl::FastLastScheduler;

/// Where in the fold pipeline the grid currently sits. Private to this half;
/// the composite observes progress through [`FsmScheduler::is_active`].
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum FastLastPhase {
    /// No reply in flight.
    Idle,
    /// CMP-match-driven grid: one body per `BYTES_PER_INTERVAL` step from
    /// `t_prior_start` to `final_anchor_offset`. The final body runs the
    /// busy-wait fold loop, then returns to [`Idle`] (the TX-start body owns
    /// the post-start residue).
    ///
    /// [`Idle`]: FastLastPhase::Idle
    PeriodicWalk,
}

/// Grid geometry for one Last reply, derived by [`FsmScheduler::plan`] from
/// the shared [`FastLastSchedule`]. All offsets are from `packet_end_tick`.
struct GridPlan {
    /// First armed CMP — `final_anchor_offset` stepped back by whole
    /// intervals until ≤ one step from the predecessor's first wire byte.
    first_anchor_offset: u32,
    /// The terminal (busy-wait) body's anchor.
    final_anchor_offset: u32,
    /// `BYTES_PER_INTERVAL · byte_ticks` — grid step in scheduler ticks.
    interval_ticks: u32,
    /// `predecessor_bytes − GUARD_BYTES` — the final body's busy-wait exit.
    busy_wait_target: u32,
    /// Stop-fold deadline: predecessor wire-end minus the GUARD window the
    /// TX-start body absorbs.
    deadline_offset: u32,
}

/// Grid math + FSM for the Fast Last fold pipeline.
///
/// Generic over its scheduler provider `S`. The driver works in offsets from
/// `packet_end_tick`; the provider owns all scheduling-domain tick state
/// internally.
pub struct FsmScheduler<S: FastLastScheduler> {
    scheduler: S,
    phase: FastLastPhase,
    /// Wall-clock offset (from packet_end) of the body currently armed on
    /// the scheduler. Body bumps this by `interval_ticks` before re-arming —
    /// drift in any one body's ISR-entry latency doesn't propagate.
    next_anchor_offset: u32,
    /// Wall-clock offset (from packet_end) of the terminal (busy-wait) body.
    /// Set once at `start`; on equality with `next_anchor_offset` the body
    /// switches from "advance grid" to "busy-wait and return".
    final_anchor_offset: u32,
    /// `BYTES_PER_INTERVAL · byte_ticks` — grid step in scheduler ticks.
    /// Cached at `start` for fast re-arming.
    interval_ticks: u32,
    /// `predecessor_bytes − GUARD_BYTES`, derived once at `start`. The final
    /// body's busy-wait exits when the walked count reaches it; the trailing
    /// GUARD bytes are folded by the TX-start body.
    busy_wait_target: u32,
    /// WireClock u32 anchor cached at `start` — every subsequent grid
    /// `schedule(deadline)` is `packet_end_tick + offset`.
    packet_end_tick: u32,
}

impl<S: FastLastScheduler> FsmScheduler<S> {
    pub const fn new(scheduler: S) -> Self {
        Self {
            scheduler,
            phase: FastLastPhase::Idle,
            next_anchor_offset: 0,
            final_anchor_offset: 0,
            interval_ticks: 0,
            busy_wait_target: 0,
            packet_end_tick: 0,
        }
    }

    // -- events -----------------------------------------------------------------

    /// Compute the periodic-walk grid, stage the deadline, arm the first CMP,
    /// and flip the FSM into [`FastLastPhase::PeriodicWalk`].
    pub fn start(&mut self, p: FastLastSchedule) {
        let plan = Self::plan(&p);
        self.next_anchor_offset = plan.first_anchor_offset;
        self.final_anchor_offset = plan.final_anchor_offset;
        self.interval_ticks = plan.interval_ticks;
        self.busy_wait_target = plan.busy_wait_target;
        self.packet_end_tick = p.packet_end_tick;
        self.phase = FastLastPhase::PeriodicWalk;

        self.scheduler
            .set_deadline(p.packet_end_tick.wrapping_add(plan.deadline_offset));
        self.schedule_walk_cmp(plan.first_anchor_offset);
    }

    /// Pure grid geometry from one [`FastLastSchedule`] — no FSM state.
    fn plan(p: &FastLastSchedule) -> GridPlan {
        let byte_ticks = p.byte_ticks as u32;
        let rdt_ticks = p.rdt_ticks as u32;
        let predecessor_bytes = p.predecessor_bytes;
        let interval_ticks = S::BYTES_PER_INTERVAL as u32 * byte_ticks;
        let t_guard = S::GUARD_BYTES as u32 * byte_ticks;

        let t_prior_start_offset = rdt_ticks.wrapping_add(byte_ticks);
        let t_prior_end_offset = rdt_ticks.wrapping_add(predecessor_bytes * byte_ticks);
        let t_prior_duration = predecessor_bytes.saturating_sub(1) * byte_ticks;
        // Our reply must land at predecessor_end (no inter-slot gap).
        let tx_start_offset = t_prior_end_offset;

        let deadline_offset = t_prior_end_offset.wrapping_sub(t_guard);
        let final_anchor_offset = if predecessor_bytes > S::GUARD_BYTES as u32 {
            tx_start_offset
                .wrapping_sub(core::cmp::min(interval_ticks, t_prior_duration))
                .wrapping_sub(t_guard)
        } else {
            tx_start_offset
        };

        // Step back by `interval` until ≤ one step from t_prior_start_offset.
        // The i32 cast handles the small-predecessor case where
        // `final_anchor_offset` < `t_prior_start_offset` modular-wraps —
        // without it the loop would spin ~4G times.
        let mut first_anchor_offset = final_anchor_offset;
        while (first_anchor_offset.wrapping_sub(t_prior_start_offset) as i32)
            >= interval_ticks as i32
        {
            first_anchor_offset = first_anchor_offset.wrapping_sub(interval_ticks);
        }

        GridPlan {
            first_anchor_offset,
            final_anchor_offset,
            interval_ticks,
            busy_wait_target: predecessor_bytes.saturating_sub(S::GUARD_BYTES as u32),
            deadline_offset,
        }
    }

    /// Arm one periodic-walk CMP at `anchor_offset` past the packet-end
    /// anchor. Single arming point for the grid — EVERY CMP, intermediates
    /// AND the final one, back-dates by `FAST_LAST_ENTRY_TICKS` so
    /// ISR-entry latency lands the body ON the grid point, not after it
    /// (back-dating only the last CMP is exactly how the legacy fold grid
    /// drifted into contention).
    #[inline]
    fn schedule_walk_cmp(&mut self, anchor_offset: u32) {
        self.scheduler.schedule(
            self.packet_end_tick
                .wrapping_add(anchor_offset.wrapping_sub(S::FAST_LAST_ENTRY_TICKS as u32)),
        );
    }

    /// Drive one grid body. Composite calls this from its long-horizon timer
    /// demux when our CMP fires.
    ///
    /// `walker` runs the classifier + parser-drain + per-byte fold for
    /// whatever bytes have landed since the last body, and returns the
    /// composite-side cumulative count of predecessor bytes folded so far.
    /// Called at least once per invocation. The final-anchor busy-wait exits
    /// when the returned count reaches `predecessor_bytes − GUARD`.
    ///
    /// `commit_pending` is invoked exactly once on entry to the final-anchor
    /// body, *before* the busy-wait starts. The composite wires it to
    /// `TxScheduler::commit_pending` so the wire-fire schedule the
    /// TxScheduler stashed at `send_slot(Last)` time is committed while
    /// there's still ~1 byte_time of headroom — the hardware match latches
    /// during the busy-wait spin and the fire ISR runs as soon as the
    /// busy-wait exits. Not called from intermediate-step bodies.
    ///
    /// Single walker closure (not `(walker, bytes_walked)`) so the composite
    /// can capture `&mut FoldEngine` once for both the walk and the read —
    /// splitting them would force interior mutability on the count.
    pub fn on_step<W, C>(&mut self, mut walker: W, commit_pending: C)
    where
        W: FnMut() -> u32,
        C: FnOnce(),
    {
        match self.phase {
            FastLastPhase::PeriodicWalk => {
                let bytes_walked = walker();
                if self.next_anchor_offset == self.final_anchor_offset {
                    // Final anchor — commit the TxScheduler stash so the
                    // wire-fire path arms while there's still a byte_time of
                    // headroom, then busy-wait. Uncontested at high baud
                    // because the composite paused DMA1_CH7 HT/TC at start
                    // time, so this body is the sole HIGH consumer in the
                    // window. Exit when `bytes_walked >= busy_wait_target` OR
                    // scheduler reports deadline passed; the remaining GUARD
                    // bytes are absorbed by the TX-start body's tail fold.
                    commit_pending();
                    //
                    // SAFETY: the plateau check (no progress between walker
                    // calls) is a hard backstop — in production the deadline
                    // is the primary exit, but if scheduler state ever
                    // diverges (set_deadline math wrong, CMP latched but
                    // mask-cleared, etc.) the chip must not spin forever
                    // inside ISR. Bytes can only stall when the wire is
                    // silent — the predecessor either dropped its reply
                    // entirely or finished early — both of which mean there's
                    // no more work to do here regardless.
                    let target = self.busy_wait_target;
                    if bytes_walked < target {
                        let mut prev_walked = bytes_walked;
                        loop {
                            let walked = walker();
                            if walked >= target {
                                break;
                            }
                            if self.scheduler.deadline_passed() {
                                break;
                            }
                            if walked == prev_walked {
                                break;
                            }
                            prev_walked = walked;
                        }
                    }
                    self.phase = FastLastPhase::Idle;
                    self.scheduler.cancel();
                    return;
                }
                self.next_anchor_offset = self.next_anchor_offset.wrapping_add(self.interval_ticks);
                self.schedule_walk_cmp(self.next_anchor_offset);
            }
            FastLastPhase::Idle => {
                // Spurious CMP entry — defensive cancel.
                self.scheduler.cancel();
            }
        }
    }

    // -- commands ---------------------------------------------------------------

    /// Drop any pending CMP and return to [`FastLastPhase::Idle`]. Idempotent.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        self.phase = FastLastPhase::Idle;
    }

    /// Bumped by [`FastLast::on_tx_start`] on either miss route (expired
    /// patch window or predecessor-byte plateau) — see
    /// [`FastLastScheduler::record_patch_deadline_miss`].
    ///
    /// [`FastLast::on_tx_start`]: super::FastLast::on_tx_start
    pub fn record_patch_deadline_miss(&mut self) {
        self.scheduler.record_patch_deadline_miss();
    }

    // -- accessors --------------------------------------------------------------

    pub fn is_active(&self) -> bool {
        !matches!(self.phase, FastLastPhase::Idle)
    }

    /// Polled by the [`FastLast::on_tx_start`] post-fire fold loop — see
    /// [`FastLastScheduler::patch_window_expired`].
    ///
    /// [`FastLast::on_tx_start`]: super::FastLast::on_tx_start
    pub fn patch_window_expired(&self) -> bool {
        self.scheduler.patch_window_expired()
    }
}

#[cfg(test)]
impl<S: FastLastScheduler> FsmScheduler<S> {
    pub(crate) fn next_anchor_offset_for_test(&self) -> u32 {
        self.next_anchor_offset
    }
    pub(crate) fn final_anchor_offset_for_test(&self) -> u32 {
        self.final_anchor_offset
    }
    pub(crate) fn interval_ticks_for_test(&self) -> u32 {
        self.interval_ticks
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::dxl::uart::test_support::{self, FastLastState};
    use crate::mocks::{FastLastSchedulerOp, MockFastLastScheduler};
    use core::cell::Cell;

    /// FAST_LAST_ENTRY_TICKS matching the mock — the back-date applied to
    /// every grid CMP.
    const ENTRY: u32 = 240;
    /// BYTES_PER_INTERVAL · byte_time at the test baud (3M: 15·160 = 2400).
    const INTERVAL_3M: u32 = 2400;
    /// 3M byte_time (10·tpb at HCLK=48M / 3MHz).
    const BYTE_TICKS_3M: u16 = 160;
    /// 3M byte_time as u32 for arithmetic on offsets.
    const BYTE_TICKS_3M_U32: u32 = BYTE_TICKS_3M as u32;

    /// Build a schedule with only the grid fields set; the fold cursor is
    /// irrelevant to the scheduler half.
    fn sched(packet_end_tick: u32, rdt_ticks: u16, predecessor_bytes: u32) -> FastLastSchedule {
        FastLastSchedule {
            packet_end_tick,
            rdt_ticks,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes,
            fold_start_cursor: 0,
        }
    }

    fn mk_fast_last() -> (FsmScheduler<MockFastLastScheduler>, FastLastState) {
        let (m, state) = test_support::mk_fast_last();
        (FsmScheduler::new(m), state)
    }

    #[test]
    fn start_stages_deadline_and_first_cmp_at_anchor_minus_entry() {
        // predecessor_bytes=2 with rdt=12000: tx_start = 12000 + 2·160 =
        // 12320; single-step grid (t_prior_duration = 160 < interval), so
        // the final anchor doubles as the first.
        let p = sched(2000, 12000, 2);
        let plan = FsmScheduler::<MockFastLastScheduler>::plan(&p);
        assert_eq!(plan.final_anchor_offset, 12320 - 160 - 160);
        assert_eq!(plan.first_anchor_offset, plan.final_anchor_offset);
        assert_eq!(plan.deadline_offset, 12320 - 160);

        let (mut d, state) = mk_fast_last();
        d.start(p);

        assert_eq!(d.next_anchor_offset_for_test(), plan.first_anchor_offset);
        assert_eq!(d.final_anchor_offset_for_test(), plan.final_anchor_offset);
        assert_eq!(
            state.operations().as_slice(),
            &[
                FastLastSchedulerOp::SetDeadline {
                    deadline: 2000 + plan.deadline_offset,
                },
                FastLastSchedulerOp::Schedule {
                    deadline: 2000 + (plan.first_anchor_offset - ENTRY),
                },
            ]
        );
    }

    #[test]
    fn start_step_back_handles_small_predecessor_modular_wrap() {
        // rdt=0, predecessor_bytes=2: t_prior_start = 160, t_prior_end =
        // 320, t_prior_duration = 160, final_anchor_offset = 320 − 160 −
        // 160 = 0. Step-back: (0 − 160 as i32) = -160 < INTERVAL → loop
        // doesn't execute. Without i32 cast the diff is ~4G ≥ 2400 → spin.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 0, 2));

        assert_eq!(d.next_anchor_offset_for_test(), 0);
        // SetDeadline + exactly one Schedule — step-back didn't spin.
        assert_eq!(state.operations().len(), 2);
    }

    #[test]
    fn intermediate_step_advances_grid_by_interval() {
        // predecessor_bytes=50 spans three anchors: the final anchor backs
        // off tx_start (20000) by one interval + guard, then the step-back
        // loop retreats two whole intervals toward the predecessor's start.
        let p = sched(0, 12000, 50);
        let plan = FsmScheduler::<MockFastLastScheduler>::plan(&p);
        assert_eq!(plan.final_anchor_offset, 20000 - INTERVAL_3M - 160);
        assert_eq!(
            plan.first_anchor_offset,
            plan.final_anchor_offset - 2 * INTERVAL_3M
        );
        assert_eq!(plan.interval_ticks, INTERVAL_3M);

        let (mut d, state) = mk_fast_last();
        d.start(p);
        assert_eq!(d.next_anchor_offset_for_test(), plan.first_anchor_offset);
        assert_eq!(d.final_anchor_offset_for_test(), plan.final_anchor_offset);

        d.on_step(|| 0, || {});
        d.on_step(|| 0, || {});

        assert_eq!(
            state.operations().as_slice(),
            &[
                FastLastSchedulerOp::SetDeadline {
                    deadline: 12000 + 50 * BYTE_TICKS_3M_U32 - BYTE_TICKS_3M_U32,
                },
                FastLastSchedulerOp::Schedule {
                    deadline: plan.first_anchor_offset - ENTRY,
                },
                FastLastSchedulerOp::Schedule {
                    deadline: plan.first_anchor_offset + INTERVAL_3M - ENTRY,
                },
                FastLastSchedulerOp::Schedule {
                    deadline: plan.final_anchor_offset - ENTRY,
                },
            ]
        );
        assert!(d.is_active());
        assert_eq!(d.next_anchor_offset_for_test(), plan.final_anchor_offset);
    }

    #[test]
    fn final_anchor_busy_waits_until_bytes_walked_target() {
        // predecessor_bytes=5 with GUARD=1 → target=4. Pre-loop walker →
        // bytes=1; three loop iters bring bytes to 4 → break. Total walker
        // calls = 4. deadline_passed stays false the entire time.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 0, 5));
        assert_eq!(
            d.next_anchor_offset_for_test(),
            d.final_anchor_offset_for_test()
        );

        let walker_calls = Cell::new(0u32);
        let bytes = Cell::new(0u32);
        let commit_calls = Cell::new(0u32);
        d.on_step(
            || {
                walker_calls.set(walker_calls.get() + 1);
                bytes.set(bytes.get() + 1);
                bytes.get()
            },
            || {
                commit_calls.set(commit_calls.get() + 1);
            },
        );

        assert_eq!(walker_calls.get(), 4);
        // commit_pending fires exactly once, on entry to the final-anchor body.
        assert_eq!(commit_calls.get(), 1);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn final_anchor_busy_waits_until_deadline_passed_when_bytes_lag() {
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 0, 3));
        // Drive the deadline-passed branch on the first inner-loop check.
        state.stage_deadline_passed(true);

        let walker_calls = Cell::new(0u32);
        d.on_step(
            || {
                walker_calls.set(walker_calls.get() + 1);
                0_u32 // bytes never arrive
            },
            || {},
        );

        // Pre-loop walker + one inside the loop before the deadline branch
        // catches: 2 calls.
        assert_eq!(walker_calls.get(), 2);
        assert!(!d.is_active());
    }

    #[test]
    fn final_anchor_busy_wait_breaks_on_byte_plateau_when_deadline_stuck() {
        // Backstop for the production-safety hazard the on_fold_step_*
        // composite test surfaced: if the scheduler's `deadline_passed`
        // ever lies (CMP misconfigured, IF cleared but match missed, etc.)
        // the busy-wait must still exit when no new bytes arrive between
        // walker calls. predecessor_bytes=10 / target=9; walker returns 4
        // every call (no progress); deadline_passed stays false. Expected:
        // 2 walker calls (1 pre-loop + 1 inside loop that observes the
        // plateau against pre-loop's count) then break.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 0, 10));
        assert_eq!(
            d.next_anchor_offset_for_test(),
            d.final_anchor_offset_for_test()
        );

        let walker_calls = Cell::new(0u32);
        d.on_step(
            || {
                walker_calls.set(walker_calls.get() + 1);
                4 // never advances
            },
            || {},
        );

        assert_eq!(walker_calls.get(), 2);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn final_anchor_returns_to_idle_and_cancels_after_busy_wait() {
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 0, 2));
        let bytes = Cell::new(0u32);
        d.on_step(
            || {
                bytes.set(bytes.get() + 1);
                bytes.get()
            },
            || {},
        );

        assert!(!d.is_active());
        // Log: SetDeadline + Schedule (from start) then Cancel (from on_step exit).
        let ops = state.operations();
        assert_eq!(ops.len(), 3);
        assert!(matches!(ops[2], FastLastSchedulerOp::Cancel));
    }

    #[test]
    fn cancel_from_periodic_walk_returns_to_idle() {
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 12000, 50));
        assert!(d.is_active());

        d.cancel();

        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    /// Bench-traced regression at 3M GUARD=1 predecessor_bytes=14 — pins the
    /// grid math against a known-good trace. Single-step grid case
    /// (`t_prior_duration < interval`): `final_anchor_offset` and the first
    /// anchor land at 12000, scheduled CMP is `anchor − ENTRY`,
    /// deadline_offset = `t_prior_end − GUARD · byte` = 14080.
    #[test]
    fn deadline_grid_correctness_3m_predecessor_bytes_14() {
        let p = sched(0, 12000, 14);
        let plan = FsmScheduler::<MockFastLastScheduler>::plan(&p);
        assert_eq!(plan.final_anchor_offset, 12000);
        assert_eq!(plan.first_anchor_offset, 12000);
        assert_eq!(plan.interval_ticks, INTERVAL_3M);
        assert_eq!(plan.deadline_offset, 14080);
        assert_eq!(plan.busy_wait_target, 13);

        let (mut d, state) = mk_fast_last();
        d.start(p);

        assert_eq!(d.final_anchor_offset_for_test(), 12000);
        assert_eq!(d.next_anchor_offset_for_test(), 12000);
        assert_eq!(d.interval_ticks_for_test(), INTERVAL_3M);
        assert_eq!(
            state.operations().as_slice(),
            &[
                FastLastSchedulerOp::SetDeadline { deadline: 14080 },
                FastLastSchedulerOp::Schedule {
                    deadline: 12000 - ENTRY,
                },
            ]
        );
    }
}
