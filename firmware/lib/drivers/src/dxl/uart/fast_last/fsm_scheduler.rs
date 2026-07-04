//! Checkpoint-wake scheduler for the Fast successor CRC pipeline. Owns the
//! single CMP that wakes the chip just before the predecessor window's
//! trailing CRC bytes (the chain-state checkpoint) land, plus the starve
//! retries that keep re-checking until the patch window closes. The wire
//! start itself is hardware-armed (TX kickoff, doc §5) and fires in
//! parallel — nothing here sits on the wire deadline; the only race is
//! `patch_crc` against the TX DMA's read of the trailing CRC slot
//! (~`own_reply_bytes · byte_time` wide), and the checkpoint pickup wins
//! it by construction (the checkpoint lands one byte-time BEFORE the
//! fire, and the pickup is ~a dozen table steps).
//!
//! Half of the [`FastLast`] sub-composite (§4.3): the scheduling half. The
//! CRC half is [`FoldEngine`]. This half works in absolute u32 deadlines
//! (WireClock domain) — `start()` composes the status-start anchor with
//! protocol-derived offsets and hands the provider one u32 per staged CMP.
//!
//! [`FastLast`]: super::FastLast
//! [`FoldEngine`]: super::fold_engine::FoldEngine

use super::FoldExit;
use super::schedule::FastLastSchedule;
use crate::traits::dxl::FastLastScheduler;

/// Starve-retry stride, in wire bytes. A late checkpoint (silent or
/// stalled predecessor) re-checks on this cadence until the patch window
/// closes.
const STARVE_RETRY_STRIDE_BYTES: u32 = 1;

/// Where in the reply timeline the scheduler currently sits.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum FastLastPhase {
    /// No reply in flight.
    Idle,
    /// Intermediate ring-drain CMPs for windows longer than the drain
    /// stride: each body runs one O(1) pickup attempt, whose publish +
    /// consume-to-cap side effect keeps the RX ring's NDTR-derived
    /// producer head from aliasing (see `FastLastSchedule::
    /// drain_stride_bytes`). No spinning, no folding.
    Drain,
    /// Wake CMP armed (or a starve retry pending): each body attempts the
    /// checkpoint pickup, spinning briefly while the bytes are still on
    /// the wire.
    Wake,
}

/// Wake scheduler + starve-retry FSM for the Fast successor CRC pipeline.
///
/// Generic over its scheduler provider `S`. The driver works in offsets from
/// `status_start_tick`; the provider owns all scheduling-domain tick state
/// internally.
pub struct FsmScheduler<S: FastLastScheduler> {
    scheduler: S,
    phase: FastLastPhase,
    /// WireClock u32 anchor cached at `start` — every subsequent
    /// `schedule(deadline)` is `status_start_tick + offset`.
    status_start_tick: u32,
    /// Offset of the predecessor window's wire end (== our fire deadline):
    /// `predecessor_bytes · byte_ticks`. The checkpoint occupies the
    /// window's last two byte-times.
    window_end_offset: u32,
    /// One wire byte in scheduler ticks — the starve-retry stride.
    byte_ticks: u32,
    /// Offset of the most recently scheduled starve retry; each starved
    /// body advances it one byte-time before re-arming, so retry drift
    /// never compounds.
    retry_offset: u32,
    /// Ring-drain stride in scheduler ticks (`drain_stride_bytes ·
    /// byte_ticks`); the [`FastLastPhase::Drain`] CMP cadence.
    drain_stride_ticks: u32,
    /// Offset of the next pending drain CMP while in
    /// [`FastLastPhase::Drain`].
    next_drain_offset: u32,
}

impl<S: FastLastScheduler> FsmScheduler<S> {
    pub const fn new(scheduler: S) -> Self {
        Self {
            scheduler,
            phase: FastLastPhase::Idle,
            status_start_tick: 0,
            window_end_offset: 0,
            byte_ticks: 0,
            retry_offset: 0,
            drain_stride_ticks: 0,
            next_drain_offset: 0,
        }
    }

    // -- events -----------------------------------------------------------------

    /// Drive one wake body. Composite calls this from its long-horizon
    /// timer demux when our CMP triggers, and inline from the
    /// status-start observation when the window is within the inline
    /// horizon (no CMP dispatch latency to pay).
    ///
    /// `pickup` attempts the checkpoint pickup: it returns `true` once the
    /// predecessor's trailing CRC bytes have been read and the trailing
    /// slot patched (or when nothing is armed — already finalized). The
    /// body spins it against the live wire: the wake is deliberately
    /// early-biased ([`FastLastScheduler::WAKE_LEAD_TICKS`]), so the
    /// typical body waits out the lead margin and finalizes within a few
    /// ticks of the checkpoint's stop bit. A silent predecessor exits via
    /// `deadline_passed` (one byte past the window) to a byte-stride
    /// retry, and converges at `patch_window_expired` (the hardware fire
    /// drains CH4 regardless) — the composite routes that exit to the
    /// `crc_patch_deadline_miss` counter.
    ///
    /// `commit_pending` is invoked before the spin so a far-horizon TX
    /// stash (low baud — this pipeline co-owned the long-horizon timer
    /// until here) commits while the remaining wait fits the provider's
    /// direct-arm horizon. A no-op when `schedule` armed the hardware at
    /// observation time.
    // RAM placement is load-bearing: the spin runs while the predecessor
    // streams at wire rate, and flash i-fetch stalls in the loop eat the
    // early-wake margin. `inline(never)` keeps the body out of
    // flash-resident callers so the section actually applies.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    #[inline(never)]
    pub fn on_step<W, C>(&mut self, mut pickup: W, commit_pending: C) -> FoldExit
    where
        W: FnMut() -> bool,
        C: FnOnce(),
    {
        match self.phase {
            FastLastPhase::Drain => {
                // One O(1) pickup attempt: publishes the ring head and
                // consumes published window bytes; succeeds outright when
                // the whole window is already in (late observation).
                if pickup() {
                    commit_pending();
                    self.finish();
                    return FoldExit::Finalized;
                }
                let next = self.next_drain_offset.wrapping_add(self.drain_stride_ticks);
                let wake_offset = self
                    .window_end_offset
                    .wrapping_sub(S::WAKE_LEAD_TICKS as u32);
                if (wake_offset.wrapping_sub(next) as i32) > self.drain_stride_ticks as i32 {
                    self.next_drain_offset = next;
                    self.scheduler
                        .schedule(self.status_start_tick.wrapping_add(next));
                } else {
                    self.phase = FastLastPhase::Wake;
                    self.scheduler
                        .schedule(self.status_start_tick.wrapping_add(wake_offset));
                }
                FoldExit::Pending
            }
            FastLastPhase::Wake => {
                commit_pending();
                loop {
                    if pickup() {
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
                // Checkpoint is late (silent or stalled predecessor).
                // Re-arm one stride out and keep retrying — the armed
                // hardware fire guarantees `patch_window_expired`
                // eventually closes the loop.
                self.retry_offset = self
                    .retry_offset
                    .wrapping_add(self.byte_ticks.wrapping_mul(STARVE_RETRY_STRIDE_BYTES));
                self.scheduler
                    .schedule(self.status_start_tick.wrapping_add(self.retry_offset));
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

    /// Arm the wake for one successor reply: CMP at the window end minus
    /// the (worst-case-entry-sized) wake lead, so the body arrives BEFORE
    /// the checkpoint bytes and spins them in — waking early costs a
    /// short spin; waking late eats the patch-vs-fetch margin. A past
    /// target (late observation, short window) triggers the provider's
    /// fire-ASAP CMP path and the body runs immediately.
    pub fn start(&mut self, p: FastLastSchedule) {
        let byte_ticks = p.byte_ticks as u32;
        self.status_start_tick = p.status_start_tick;
        self.window_end_offset = p.predecessor_bytes.wrapping_mul(byte_ticks);
        self.byte_ticks = byte_ticks;
        self.retry_offset = self.window_end_offset;
        self.drain_stride_ticks = p.drain_stride_bytes.wrapping_mul(byte_ticks);

        // Starve bound for the body's spin: one byte-time past the window
        // — a checkpoint that hasn't landed by then is late, not in
        // flight.
        self.scheduler.set_busy_wait_deadline(
            p.status_start_tick
                .wrapping_add(self.window_end_offset)
                .wrapping_add(byte_ticks),
        );
        let wake_offset = self
            .window_end_offset
            .wrapping_sub(S::WAKE_LEAD_TICKS as u32);
        if p.predecessor_bytes > p.drain_stride_bytes {
            // Long window: intermediate drains keep the ring head honest
            // (one per stride) until the wake point.
            self.phase = FastLastPhase::Drain;
            self.next_drain_offset = self.drain_stride_ticks;
            self.scheduler
                .schedule(p.status_start_tick.wrapping_add(self.drain_stride_ticks));
        } else {
            self.phase = FastLastPhase::Wake;
            self.scheduler
                .schedule(p.status_start_tick.wrapping_add(wake_offset));
        }
    }

    /// Return to idle, dropping any armed CMP. Idempotent.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        self.phase = FastLastPhase::Idle;
    }

    // -- accessors --------------------------------------------------------------

    /// True while a wake (or starve retry) is pending.
    pub fn is_active(&self) -> bool {
        self.phase != FastLastPhase::Idle
    }

    // -- internals --------------------------------------------------------------

    fn finish(&mut self) {
        self.scheduler.cancel();
        self.phase = FastLastPhase::Idle;
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::dxl::uart::test_support::{self, FastLastState};
    use crate::mocks::{FastLastSchedulerOp, MockFastLastScheduler};
    use core::cell::Cell;

    /// WAKE_LEAD_TICKS matching the mock.
    const LEAD: u32 = 500;
    /// 3M byte_time (10·tpb at HCLK=48M / 3MHz).
    const BYTE_TICKS_3M: u16 = 160;
    const BYTE_TICKS_3M_U32: u32 = BYTE_TICKS_3M as u32;

    /// Half a 64-deep RX ring — the composite's production value.
    const DRAIN_STRIDE_BYTES: u32 = 32;

    fn sched(status_start_tick: u32, predecessor_bytes: u32) -> FastLastSchedule {
        FastLastSchedule {
            status_start_tick,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes,
            fold_start_cursor: 0,
            drain_stride_bytes: DRAIN_STRIDE_BYTES,
        }
    }

    fn mk_fast_last() -> (FsmScheduler<MockFastLastScheduler>, FastLastState) {
        let (m, state) = test_support::mk_fast_last();
        (FsmScheduler::new(m), state)
    }

    #[test]
    fn start_arms_the_wake_lead_before_the_window_end() {
        // predecessor_bytes=14 at 3M: window end (== fire deadline) at
        // offset 2240; wake CMP early-biased by LEAD; starve bound one
        // byte past the window.
        let (mut d, state) = mk_fast_last();
        d.start(sched(20_000, 14));

        assert!(d.is_active());
        assert_eq!(
            state.operations().as_slice(),
            &[
                FastLastSchedulerOp::SetBusyWaitDeadline {
                    deadline: 20_000 + 2240 + BYTE_TICKS_3M_U32,
                },
                FastLastSchedulerOp::Schedule {
                    deadline: 20_000 + 2240 - LEAD,
                },
            ]
        );
    }

    #[test]
    fn long_window_steps_drain_cmps_then_hands_to_the_wake() {
        // predecessor_bytes=140 at 3M: window end 22400, wake at
        // 22400−LEAD. Drains at one stride (32 bytes = 5120 ticks) apart
        // keep the ring head honest; the hand-off to Wake happens once
        // the next drain would land within a stride of the wake.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 140));

        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Schedule { deadline: 5120 })
        ));

        // Drain bodies step by one stride while short of the wake.
        assert_eq!(d.on_step(|| false, || {}), FoldExit::Pending);
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Schedule { deadline: 10240 })
        ));
        assert_eq!(d.on_step(|| false, || {}), FoldExit::Pending);
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Schedule { deadline: 15360 })
        ));
        // 15360 + 5120 = 20480; wake = 22400 − 500 = 21900; gap 1420 <
        // stride → hand off to the wake CMP.
        assert_eq!(d.on_step(|| false, || {}), FoldExit::Pending);
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Schedule { deadline: 21900 })
        ));
        assert!(d.is_active());
    }

    #[test]
    fn drain_body_finalizes_outright_when_the_window_is_already_in() {
        // Late observation of a long window: everything already published
        // — the first drain body's pickup succeeds and commits the stash.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 140));

        let commit_calls = Cell::new(0u32);
        let exit = d.on_step(|| true, || commit_calls.set(commit_calls.get() + 1));
        assert_eq!(exit, FoldExit::Finalized);
        assert_eq!(commit_calls.get(), 1);
        assert!(!d.is_active());
        let _ = state;
    }

    #[test]
    fn pickup_success_finalizes_and_commits_once() {
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));

        let commit_calls = Cell::new(0u32);
        let exit = d.on_step(|| true, || commit_calls.set(commit_calls.get() + 1));

        assert_eq!(exit, FoldExit::Finalized);
        assert_eq!(commit_calls.get(), 1);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn wake_body_spins_the_checkpoint_in() {
        // The early-biased wake arrives before the checkpoint bytes; the
        // body spins `pickup` until they land — no re-arm, no dispatch.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));
        state.stage_deadline_passed(false);

        let calls = Cell::new(0u32);
        let exit = d.on_step(
            || {
                let n = calls.get() + 1;
                calls.set(n);
                n >= 3
            },
            || {},
        );

        assert_eq!(exit, FoldExit::Finalized);
        assert_eq!(calls.get(), 3, "spun until the bytes landed");
        assert!(!d.is_active());
    }

    #[test]
    fn starved_body_rearms_one_stride_out() {
        // Silent predecessor: pickup never succeeds, the starve bound is
        // past → re-arm one retry stride out and stay active.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));

        assert_eq!(d.on_step(|| false, || {}), FoldExit::Pending);
        assert!(d.is_active());
        assert_eq!(
            state.operations().last(),
            Some(&FastLastSchedulerOp::Schedule {
                deadline: 2240 + STARVE_RETRY_STRIDE_BYTES * BYTE_TICKS_3M_U32,
            }),
        );

        // Next retry steps one more stride out.
        assert_eq!(d.on_step(|| false, || {}), FoldExit::Pending);
        assert_eq!(
            state.operations().last(),
            Some(&FastLastSchedulerOp::Schedule {
                deadline: 2240 + 2 * STARVE_RETRY_STRIDE_BYTES * BYTE_TICKS_3M_U32,
            }),
        );
    }

    #[test]
    fn expired_window_exits_with_the_miss_disposition() {
        // The TX drain reached the CRC slot (the hardware fire happened
        // regardless) — a patch can no longer ship.
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));
        state.stage_patch_window_expired(true);

        assert_eq!(d.on_step(|| false, || {}), FoldExit::WindowExpired);
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn spurious_cmp_in_idle_cancels_defensively() {
        let (mut d, state) = mk_fast_last();
        assert_eq!(d.on_step(|| false, || {}), FoldExit::Idle);
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn cancel_from_wake_returns_to_idle() {
        let (mut d, state) = mk_fast_last();
        d.start(sched(0, 14));
        assert!(d.is_active());

        d.cancel();
        assert!(!d.is_active());
        assert!(matches!(
            state.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }
}
