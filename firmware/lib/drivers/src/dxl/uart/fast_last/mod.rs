//! Fast Last CRC fold pipeline — a mini §4.3 sub-composite of two
//! independent halves:
//!
//! - [`FsmScheduler`] — the periodic-walk CMP grid that paces classifier +
//!   parser + fold work across the predecessor window, plus the final-step
//!   busy-wait.
//! - [`FoldEngine`] — the running chain CRC and the bookkeeping that decides
//!   which wire bytes feed it, finalizing into a consumer-owned
//!   [`CrcPatchSink`].
//!
//! The composite owns no timing state of its own — `start` routes the shared
//! [`FastLastSchedule`] to each half, and the driver reaches the halves
//! together via [`Self::split_mut`] when it must drive the grid and fold in
//! one borrow (the walker closure folds bytes while the FSM advances).
//!
//! [`CrcPatchSink`]: crc_patch_sink::CrcPatchSink

mod crc_patch_sink;
mod fold_engine;
mod fsm_scheduler;
mod schedule;

use dxl_protocol::CrcUmts;

use crate::traits::dxl::FastLastScheduler;

pub use crc_patch_sink::CrcPatchSink;
pub use fold_engine::FoldEngine;
pub use fsm_scheduler::FsmScheduler;
pub use schedule::FastLastSchedule;

/// Fast Last pipeline composite. Generic over its scheduler provider `S` and
/// CRC engine `CRC`; holds the two halves and routes between them.
pub struct FastLast<S: FastLastScheduler, CRC: CrcUmts> {
    scheduler: FsmScheduler<S>,
    crc: FoldEngine<CRC>,
}

impl<S: FastLastScheduler, CRC: CrcUmts> FastLast<S, CRC> {
    pub fn new(scheduler: S) -> Self {
        Self {
            scheduler: FsmScheduler::new(scheduler),
            crc: FoldEngine::new(),
        }
    }

    // -- events -----------------------------------------------------------------

    /// The TX-start tick arrived with the fold still active — run the
    /// post-fire residue fold that absorbs any GUARD bytes in flight at
    /// fire time and patches the trailing CRC slot before DMA1_CH4's
    /// prefetch reads it (doc §10.6.2 CC3 body). `drain` is one pass of
    /// the caller's RX drain, folding fresh bytes into the handed
    /// [`FoldEngine`]. No-op when the fold is idle.
    ///
    /// Three exits:
    /// - **finalize** — the engine reaches its predecessor-byte target
    ///   inside `drain`, patches the CRC slot, and clears `active`.
    ///   Success; no telemetry event.
    /// - **patch-window-expired** — [`FsmScheduler::patch_window_expired`]
    ///   reports the TX DMA channel has prefetched into the trailing CRC
    ///   slot; any further patch ships too late. Bumps
    ///   `crc_patch_deadline_miss`.
    /// - **plateau** — a `drain` pass folded nothing new; predecessor
    ///   starvation backstop ([[busy-wait-plateau-backstop]]). Same
    ///   observable failure as expired-window (placeholder CRC ships);
    ///   bumps the same counter.
    pub fn on_tx_start<D>(&mut self, mut drain: D)
    where
        D: FnMut(&mut FoldEngine<CRC>),
    {
        if !self.crc.is_active() {
            return;
        }
        loop {
            if self.scheduler.patch_window_expired() {
                self.scheduler.record_patch_deadline_miss();
                break;
            }
            let before = self.crc.bytes_folded();
            drain(&mut self.crc);
            if !self.crc.is_active() {
                break;
            }
            if self.crc.bytes_folded() == before {
                self.scheduler.record_patch_deadline_miss();
                break;
            }
        }
    }

    // -- commands ---------------------------------------------------------------

    /// Arm both halves for one Last reply: the scheduler builds its grid from
    /// the timing fields; the fold engine begins folding from
    /// `fold_start_cursor` with the same `predecessor_bytes` finalize cap.
    pub fn start(&mut self, p: FastLastSchedule) {
        self.scheduler.start(p);
        self.crc.start(p.fold_start_cursor, p.predecessor_bytes);
    }

    /// Disarm both halves. Idempotent — the fold engine's finalize path and
    /// the scheduler's busy-wait exit already return each half to idle on the
    /// successful path, so this is the belt-and-suspenders reset at
    /// `on_tx_complete`.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        self.crc.cancel();
    }

    // -- accessors --------------------------------------------------------------

    /// The two halves, mutably and disjointly, for the driver's `on_fold_step`
    /// / `on_tx_start` bodies: the FSM advances the grid while the walker
    /// closure folds freshly-drained bytes into the engine in the same borrow.
    pub fn split_mut(&mut self) -> (&mut FsmScheduler<S>, &mut FoldEngine<CRC>) {
        (&mut self.scheduler, &mut self.crc)
    }

    /// True while the CMP grid is running. Distinct from [`Self::fold_active`]:
    /// the grid returns to idle after its final busy-wait, but the fold stays
    /// active through the TX-start residue fold until finalize.
    pub fn grid_active(&self) -> bool {
        self.scheduler.is_active()
    }

    /// True while the chain CRC is folding — the RX-tail ownership gate
    /// (`dxl-streaming-rx.md` §6). Outlives [`Self::grid_active`].
    pub fn fold_active(&self) -> bool {
        self.crc.is_active()
    }

    /// Cumulative predecessor bytes folded since [`Self::start`].
    pub fn bytes_folded(&self) -> u32 {
        self.crc.bytes_folded()
    }
}
