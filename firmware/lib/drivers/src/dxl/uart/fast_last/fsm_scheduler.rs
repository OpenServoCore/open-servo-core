//! Periodic-walk grid FSM for the Fast Last fold pipeline. Owns the CMP
//! grid that paces classifier + parser + fold work across a Fast Sync /
//! Bulk Read predecessor window, plus the completion body that lands the
//! chain-CRC patch after the last predecessor byte. The wire start itself
//! is hardware-armed (TX kickoff, doc §5) and fires in parallel — no grid
//! body sits on the wire deadline; the only race the grid runs is
//! `patch_crc` against the TX DMA's read of the trailing CRC slot
//! (~`own_reply_bytes · byte_time` wide).
//!
//! Half of the [`FastLast`] sub-composite (§4.3): the scheduling half. The
//! fold half is [`FoldEngine`]. This half works in absolute u32 deadlines
//! (WireClock domain) — `start()` composes the status-start anchor with
//! protocol-derived offsets and hands the provider one u32 per staged CMP. The
//! chip-side provider applies the deadlines directly with no further lift.
//!
//! [`FastLast`]: super::FastLast
//! [`FoldEngine`]: super::fold_engine::FoldEngine

use super::FoldExit;
use super::schedule::FastLastSchedule;
use crate::traits::dxl::FastLastScheduler;

/// Where in the fold pipeline the grid currently sits. Private to this half;
/// the composite observes progress through [`FsmScheduler::is_active`].
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum FastLastPhase {
    /// No reply in flight.
    Idle,
    /// CMP-match-driven grid: one body per `BYTES_PER_INTERVAL` step from
    /// `t_prior_start` to `final_anchor_offset`. The final-anchor body
    /// commits any far-horizon TX stash and hands off to [`Completion`].
    ///
    /// [`Completion`]: FastLastPhase::Completion
    PeriodicWalk,
    /// Post-grid completion: one CMP just past the predecessor's wire end
    /// folds the tail bytes + our own reply and patches the chain CRC. A
    /// starved body (silent predecessor) re-arms at the grid stride until
    /// the fold finalizes or the patch window expires — the hardware
    /// kickoff fires regardless, so a doomed fold converges via the TX
    /// drain closing the window.
    Completion,
}

/// Grid geometry for one Last reply, derived by [`FsmScheduler::plan`] from
/// the shared [`FastLastSchedule`]. All offsets are from `status_start_tick`.
struct GridPlan {
    /// First scheduled CMP — `final_anchor_offset` stepped back by whole
    /// intervals until ≤ one step from the predecessor's first wire byte.
    first_anchor_offset: u32,
    /// The last periodic-walk body's anchor.
    final_anchor_offset: u32,
    /// `BYTES_PER_INTERVAL · byte_ticks` — grid step in scheduler ticks.
    interval_ticks: u32,
    /// The completion body's anchor: one byte-time past the predecessor's
    /// wire end, so the last predecessor byte has landed and published.
    completion_offset: u32,
    /// The final body's spin exit: one byte-time past the predecessor's
    /// wire end — bytes still short of target by then are late (silent
    /// predecessor) and the fold falls back to the completion CMP.
    spin_deadline_offset: u32,
    /// `predecessor_bytes` — the fold finalizes (and patches) on reaching it.
    fold_target: u32,
}

/// Grid math + FSM for the Fast Last fold pipeline.
///
/// Generic over its scheduler provider `S`. The driver works in offsets from
/// `status_start_tick`; the provider owns all scheduling-domain tick state
/// internally.
pub struct FsmScheduler<S: FastLastScheduler> {
    scheduler: S,
    phase: FastLastPhase,
    /// Wall-clock offset (from status_start) of the body currently scheduled on
    /// the scheduler. Body bumps this by `interval_ticks` before re-scheduling —
    /// drift in any one body's ISR-entry latency doesn't propagate.
    next_anchor_offset: u32,
    /// Wall-clock offset (from status_start) of the last periodic-walk body.
    /// Set once at `start`; on equality with `next_anchor_offset` the body
    /// commits any far-horizon TX stash and hands off to the completion body.
    final_anchor_offset: u32,
    /// `BYTES_PER_INTERVAL · byte_ticks` — grid step in scheduler ticks.
    /// Cached at `start` for fast re-scheduling.
    interval_ticks: u32,
    /// Wall-clock offset (from status_start) of the completion body.
    completion_offset: u32,
    /// `predecessor_bytes`, derived once at `start`. The fold finalizes
    /// when the walked count reaches it — the fold engine has patched the
    /// chain CRC by then.
    fold_target: u32,
    /// WireClock u32 anchor cached at `start` — every subsequent grid
    /// `schedule(deadline)` is `status_start_tick + offset`.
    status_start_tick: u32,
}

impl<S: FastLastScheduler> FsmScheduler<S> {
    pub const fn new(scheduler: S) -> Self {
        Self {
            scheduler,
            phase: FastLastPhase::Idle,
            next_anchor_offset: 0,
            final_anchor_offset: 0,
            interval_ticks: 0,
            completion_offset: 0,
            fold_target: 0,
            status_start_tick: 0,
        }
    }

    // -- events -----------------------------------------------------------------

    /// Drive one grid body. Composite calls this from its long-horizon timer
    /// demux when our CMP triggers.
    ///
    /// `walker` runs the parser-raw drain + per-byte fold for whatever
    /// bytes have landed since the last body, and returns the
    /// composite-side cumulative count of predecessor bytes folded so
    /// far. Intermediate and completion bodies call it once and re-arm;
    /// the FINAL walk body spins it across the predecessor's tail so the
    /// chain-CRC patch lands within a couple hundred ticks of the last
    /// predecessor byte — the only way to beat the TX DMA's read of a
    /// short reply's CRC slot (see the spin comment in the body). The
    /// wire start itself is hardware-armed and never waits on any of
    /// this.
    ///
    /// `commit_pending` is invoked exactly once, on the final periodic-walk
    /// body. The composite wires it to `TxScheduler::commit_pending` so a
    /// far-horizon TX stash (low baud — the grid co-owned the long-horizon
    /// timer until here) commits while the remaining wait fits the
    /// provider's direct-arm horizon. A no-op when `schedule` armed the
    /// hardware at observation time.
    ///
    /// Returns the body's [`FoldExit`]; the composite routes
    /// [`FoldExit::WindowExpired`] to the `crc_patch_deadline_miss`
    /// counter.
    ///
    /// Single walker closure (not `(walker, bytes_walked)`) so the composite
    /// can capture `&mut FoldEngine` once for both the walk and the read —
    /// splitting them would force interior mutability on the count.
    pub fn on_step<W, C>(&mut self, mut walker: W, commit_pending: C) -> FoldExit
    where
        W: FnMut() -> u32,
        C: FnOnce(),
    {
        match self.phase {
            FastLastPhase::PeriodicWalk => {
                let bytes_walked = walker();
                if bytes_walked >= self.fold_target {
                    // Fold complete at whatever body this is — a late
                    // observation (deadline already past → kickoff firing
                    // ASAP) lands every predecessor byte in one walk. The
                    // wire start still depends on any far-horizon stash:
                    // commit before finishing.
                    commit_pending();
                    self.finish();
                    return FoldExit::Finalized;
                }
                if self.next_anchor_offset != self.final_anchor_offset {
                    self.next_anchor_offset =
                        self.next_anchor_offset.wrapping_add(self.interval_ticks);
                    self.schedule_walk_cmp(self.next_anchor_offset);
                    return FoldExit::Pending;
                }
                // Final periodic-walk body — commit any far-horizon TX
                // stash so the hardware kickoff arms, then SPIN the
                // predecessor's tail. The spin is what beats the patch
                // race for short own replies: the completion-CMP path
                // costs a PFIC re-entry (~240 ticks) after the last
                // predecessor byte, but a 5-byte emission's CRC slot is
                // read ~2 byte-times after the (hardware, parallel) fire
                // — only a body already running at wire-end can patch in
                // time. Bounded: bytes stream in live, so the spin lasts
                // at most the remaining tail (≤ one grid interval, the
                // pre-#134 site-1 envelope); a silent predecessor exits
                // via `deadline_passed` (one byte past the window) or
                // `patch_window_expired` (the armed fire drains CH4
                // regardless) and falls back to the completion CMP.
                commit_pending();
                loop {
                    let walked = walker();
                    if walked >= self.fold_target {
                        self.finish();
                        return FoldExit::Finalized;
                    }
                    if self.scheduler.patch_window_expired() {
                        self.finish();
                        return FoldExit::WindowExpired;
                    }
                    if self.scheduler.deadline_passed() {
                        break;
                    }
                }
                self.phase = FastLastPhase::Completion;
                self.next_anchor_offset = self.completion_offset;
                self.schedule_completion_cmp(self.completion_offset);
                FoldExit::Pending
            }
            FastLastPhase::Completion => {
                let bytes_walked = walker();
                if bytes_walked >= self.fold_target {
                    self.finish();
                    return FoldExit::Finalized;
                }
                if self.scheduler.patch_window_expired() {
                    // The TX DMA's read cursor reached the trailing CRC
                    // slot — a patch can no longer ship. Predecessor
                    // starvation lands here (the hardware kickoff fired
                    // regardless and the drain closed the window).
                    self.finish();
                    return FoldExit::WindowExpired;
                }
                self.next_anchor_offset = self.next_anchor_offset.wrapping_add(self.interval_ticks);
                self.schedule_completion_cmp(self.next_anchor_offset);
                FoldExit::Pending
            }
            FastLastPhase::Idle => {
                // Spurious CMP entry — defensive cancel.
                self.scheduler.cancel();
                FoldExit::Idle
            }
        }
    }

    // -- commands ---------------------------------------------------------------

    /// Compute the periodic-walk grid, schedule the first CMP, and flip the
    /// FSM into [`FastLastPhase::PeriodicWalk`].
    pub fn start(&mut self, p: FastLastSchedule) {
        let plan = Self::plan(&p);
        self.next_anchor_offset = plan.first_anchor_offset;
        self.final_anchor_offset = plan.final_anchor_offset;
        self.interval_ticks = plan.interval_ticks;
        self.completion_offset = plan.completion_offset;
        self.fold_target = plan.fold_target;
        self.status_start_tick = p.status_start_tick;
        self.phase = FastLastPhase::PeriodicWalk;

        self.scheduler
            .set_busy_wait_deadline(p.status_start_tick.wrapping_add(plan.spin_deadline_offset));
        self.schedule_walk_cmp(plan.first_anchor_offset);
    }

    /// Pure grid geometry from one [`FastLastSchedule`] — no FSM state.
    fn plan(p: &FastLastSchedule) -> GridPlan {
        let byte_ticks = p.byte_ticks as u32;
        let predecessor_bytes = p.predecessor_bytes;
        let interval_ticks = S::BYTES_PER_INTERVAL as u32 * byte_ticks;

        // Offsets from the observed status start: the predecessor window
        // begins AT the anchor, so the first predecessor byte completes one
        // byte-time in and the window ends after `predecessor_bytes` whole
        // bytes — no RDT term ([[rdt-single-target-only]]).
        let t_prior_start_offset = byte_ticks;
        let t_prior_end_offset = predecessor_bytes * byte_ticks;
        let t_prior_duration = predecessor_bytes.saturating_sub(1) * byte_ticks;

        let final_anchor_offset =
            t_prior_end_offset.wrapping_sub(core::cmp::min(interval_ticks, t_prior_duration));

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
            completion_offset: t_prior_end_offset.wrapping_add(byte_ticks),
            spin_deadline_offset: t_prior_end_offset.wrapping_add(byte_ticks),
            fold_target: predecessor_bytes,
        }
    }

    /// Schedule one periodic-walk CMP at `anchor_offset` past the status-start
    /// anchor. Every walk CMP back-dates by `FAST_LAST_ENTRY_TICKS` so
    /// ISR-entry latency lands the body ON the grid point, not after it
    /// (back-dating only the last CMP is exactly how the legacy fold grid
    /// drifted into contention).
    #[inline]
    fn schedule_walk_cmp(&mut self, anchor_offset: u32) {
        self.scheduler.schedule(
            self.status_start_tick
                .wrapping_add(anchor_offset.wrapping_sub(S::FAST_LAST_ENTRY_TICKS as u32)),
        );
    }

    /// Schedule a completion-phase CMP at `anchor_offset` — NOT entry-lag
    /// back-dated. The completion body has no wire punctuality requirement
    /// (the hardware kickoff fires in parallel; the patch window is
    /// `own_reply` bytes wide), so entry latency only delays the patch
    /// INTO its window. Back-dating would instead let a fast ISR entry run
    /// the body BEFORE the last predecessor byte has published — the fold
    /// comes up one byte short, the retry burns a whole grid interval, and
    /// the TX-complete cleanup can cancel the fold before the retry lands.
    #[inline]
    fn schedule_completion_cmp(&mut self, anchor_offset: u32) {
        self.scheduler
            .schedule(self.status_start_tick.wrapping_add(anchor_offset));
    }

    /// Shared terminal transition: drop the pending CMP, return to idle.
    fn finish(&mut self) {
        self.phase = FastLastPhase::Idle;
        self.scheduler.cancel();
    }

    /// Drop any pending CMP and return to [`FastLastPhase::Idle`]. Idempotent.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        self.phase = FastLastPhase::Idle;
    }

    // -- accessors --------------------------------------------------------------

    pub fn is_active(&self) -> bool {
        !matches!(self.phase, FastLastPhase::Idle)
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
    fn sched(status_start_tick: u32, predecessor_bytes: u32) -> FastLastSchedule {
        FastLastSchedule {
            status_start_tick,
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
    fn start_schedules_first_cmp_at_anchor_minus_entry() {
        // predecessor_bytes=2 anchored on the observed status start:
        // t_prior_end = 320, t_prior_duration = 160 < interval → single-
        // step grid: the final anchor doubles as the first and sits at
        // the predecessor's first byte boundary (offset 160).
        let p = sched(2000, 2);
        let plan = FsmScheduler::<MockFastLastScheduler>::plan(&p);
        assert_eq!(plan.final_anchor_offset, 160);
        assert_eq!(plan.first_anchor_offset, plan.final_anchor_offset);
        assert_eq!(plan.completion_offset, 320 + 160);
        assert_eq!(plan.spin_deadline_offset, plan.completion_offset);
        assert_eq!(plan.fold_target, 2);

        let (mut d, state) = mk_fast_last();
        d.start(p);

        assert_eq!(d.next_anchor_offset_for_test(), plan.first_anchor_offset);
        assert_eq!(d.final_anchor_offset_for_test(), plan.final_anchor_offset);
        assert_eq!(
            state.operations().as_slice(),
            &[
                FastLastSchedulerOp::SetBusyWaitDeadline {
                    deadline: 2000 + plan.spin_deadline_offset,
                },
                FastLastSchedulerOp::Schedule {
                    deadline: 2000u32.wrapping_add(plan.first_anchor_offset.wrapping_sub(ENTRY)),
                },
            ]
        );
    }

    #[test]
    fn start_step_back_handles_small_predecessor_modular_wrap() {
        // predecessor_bytes=2: t_prior_start = 160, final_anchor_offset =
        // 160. Step-back: (160 − 160 as i32) = 0 < INTERVAL → loop doesn't
        // execute. Without the i32 cast a wrapped diff is ~4G ≥ 2400 → spin.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 2));

        assert_eq!(d.next_anchor_offset_for_test(), 160);
        // Exactly one Schedule — step-back didn't spin.
        assert_eq!(
            state
                .operations()
                .iter()
                .filter(|op| matches!(op, FastLastSchedulerOp::Schedule { .. }))
                .count(),
            1
        );
    }

    #[test]
    fn intermediate_step_advances_grid_by_interval() {
        // predecessor_bytes=50 spans three anchors: the final anchor backs
        // off t_prior_end (8000) by one interval, then the step-back loop
        // retreats two whole intervals toward the predecessor's start.
        let p = sched(0, 50);
        let plan = FsmScheduler::<MockFastLastScheduler>::plan(&p);
        assert_eq!(plan.final_anchor_offset, 8000 - INTERVAL_3M);
        assert_eq!(
            plan.first_anchor_offset,
            plan.final_anchor_offset - 2 * INTERVAL_3M
        );
        assert_eq!(plan.interval_ticks, INTERVAL_3M);

        let (mut d, state) = mk_fast_last();
        d.start(p);
        assert_eq!(d.next_anchor_offset_for_test(), plan.first_anchor_offset);
        assert_eq!(d.final_anchor_offset_for_test(), plan.final_anchor_offset);

        assert_eq!(d.on_step(|| 0, || {}), FoldExit::Pending);
        assert_eq!(d.on_step(|| 0, || {}), FoldExit::Pending);

        assert_eq!(
            state.operations().as_slice(),
            &[
                FastLastSchedulerOp::SetBusyWaitDeadline {
                    deadline: plan.spin_deadline_offset,
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
    fn final_anchor_with_all_bytes_folded_finalizes_without_completion_body() {
        // predecessor_bytes=5: everything already landed by the final
        // body — one walker call, commit once, straight to Idle.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 5));
        assert_eq!(
            d.next_anchor_offset_for_test(),
            d.final_anchor_offset_for_test()
        );

        let walker_calls = Cell::new(0u32);
        let commit_calls = Cell::new(0u32);
        let exit = d.on_step(
            || {
                walker_calls.set(walker_calls.get() + 1);
                5
            },
            || {
                commit_calls.set(commit_calls.get() + 1);
            },
        );

        assert_eq!(exit, FoldExit::Finalized);
        assert_eq!(walker_calls.get(), 1, "no busy-wait — one walk per body");
        assert_eq!(commit_calls.get(), 1);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn early_fold_completion_at_intermediate_body_commits_and_finishes() {
        // predecessor_bytes=50 (multi-interval grid) observed LATE: every
        // byte is already in the ring, so the first walk body folds all
        // of them. It must still commit the far-horizon stash (the wire
        // start depends on it) and finish without stepping the remaining
        // grid.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 50));
        assert_ne!(
            d.next_anchor_offset_for_test(),
            d.final_anchor_offset_for_test(),
            "multi-interval grid — first body is an intermediate",
        );

        let commit_calls = Cell::new(0u32);
        let exit = d.on_step(|| 50, || commit_calls.set(commit_calls.get() + 1));

        assert_eq!(exit, FoldExit::Finalized);
        assert_eq!(commit_calls.get(), 1);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn final_anchor_short_of_target_hands_off_to_completion_body() {
        // predecessor_bytes=14 (bench-traced chain shape): the final body
        // sees only part of the window — it commits the TX stash and
        // re-arms one completion CMP at t_prior_end + 1 byte, NOT
        // entry-lag back-dated (a fast entry must never land before the
        // last predecessor byte publishes). The follow-up body with all
        // bytes folded finalizes.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));

        let commit_calls = Cell::new(0u32);
        let exit = d.on_step(|| 4, || commit_calls.set(commit_calls.get() + 1));
        assert_eq!(exit, FoldExit::Pending);
        assert_eq!(commit_calls.get(), 1);
        assert!(d.is_active());
        assert_eq!(
            state.operations().last(),
            Some(&FastLastSchedulerOp::Schedule {
                deadline: 14 * BYTE_TICKS_3M_U32 + BYTE_TICKS_3M_U32,
            }),
        );

        let exit = d.on_step(|| 14, || panic!("commit must not re-run"));
        assert_eq!(exit, FoldExit::Finalized);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn final_body_spin_folds_live_tail_and_finalizes() {
        // The case the spin exists for: the final body enters short of
        // target and the remaining bytes stream in while it spins —
        // finalize in-body, no completion CMP. Deadline held open so the
        // spin actually iterates (the mock default degenerates it).
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));
        state.stage_deadline_passed(false);

        let walked = Cell::new(4u32);
        let exit = d.on_step(
            || {
                let w = walked.get();
                walked.set(w + 5);
                w
            },
            || {},
        );

        assert_eq!(exit, FoldExit::Finalized);
        assert!(!d.is_active());
        assert_eq!(walked.get(), 19, "walker re-ran inside the spin");
        assert_eq!(
            state.operations().len(),
            3,
            "start's SetBusyWaitDeadline + walk CMP + finish's Cancel — no completion CMP",
        );
    }

    #[test]
    fn final_body_spin_exits_window_expired_when_drain_closes_window() {
        // Bounded even with a silent predecessor: the hardware kickoff
        // fired in parallel, so the TX drain eventually closes the patch
        // window and the spin exits WindowExpired.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));
        state.stage_patch_window_expired(true);

        assert_eq!(d.on_step(|| 4, || {}), FoldExit::WindowExpired);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn starved_completion_body_reschedules_until_window_expires() {
        // Silent predecessor: bytes never arrive. The completion body
        // re-arms at the grid stride while the patch window is open, then
        // exits WindowExpired once the TX drain closes it — the hardware
        // kickoff fired in parallel, so the window always closes.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));
        assert_eq!(d.on_step(|| 1, || {}), FoldExit::Pending); // → Completion

        let ops_before = state.operations().len();
        assert_eq!(d.on_step(|| 1, || {}), FoldExit::Pending);
        assert!(d.is_active());
        assert_eq!(
            state.operations().len(),
            ops_before + 1,
            "starved body re-arms one CMP"
        );

        state.stage_patch_window_expired(true);
        assert_eq!(d.on_step(|| 1, || {}), FoldExit::WindowExpired);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn spurious_cmp_in_idle_cancels_defensively() {
        let (mut d, state) = mk_fast_last();
        assert_eq!(d.on_step(|| 0, || {}), FoldExit::Idle);
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn cancel_from_periodic_walk_returns_to_idle() {
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 50));
        assert!(d.is_active());

        d.cancel();

        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    /// Grid-math pin at 3M predecessor_bytes=14 (the bench-traced chain
    /// shape, anchored on the observed status start per #142). Single-step
    /// grid case (`t_prior_duration < interval`): `final_anchor_offset`
    /// and the first anchor land at the predecessor's first byte boundary
    /// (offset 160), scheduled CMP is `anchor − ENTRY`, completion body at
    /// `t_prior_end + byte = 2400`.
    #[test]
    fn deadline_grid_correctness_3m_predecessor_bytes_14() {
        let p = sched(20000, 14);
        let plan = FsmScheduler::<MockFastLastScheduler>::plan(&p);
        assert_eq!(plan.final_anchor_offset, 160);
        assert_eq!(plan.first_anchor_offset, 160);
        assert_eq!(plan.interval_ticks, INTERVAL_3M);
        assert_eq!(plan.completion_offset, 2400);
        assert_eq!(plan.fold_target, 14);

        let (mut d, state) = mk_fast_last();
        d.start(p);

        assert_eq!(d.final_anchor_offset_for_test(), 160);
        assert_eq!(d.next_anchor_offset_for_test(), 160);
        assert_eq!(d.interval_ticks_for_test(), INTERVAL_3M);
        assert_eq!(
            state.operations().as_slice(),
            &[
                FastLastSchedulerOp::SetBusyWaitDeadline {
                    deadline: 20000 + 2400,
                },
                FastLastSchedulerOp::Schedule {
                    deadline: 20000 + 160 - ENTRY,
                },
            ]
        );
    }
}
