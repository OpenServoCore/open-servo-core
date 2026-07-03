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

/// Exit disposition of [`FastLast::on_tx_start`]'s residue fold. The
/// composite routes [`Self::WindowExpired`] / [`Self::Plateau`] to the
/// `crc_patch_deadline_miss` telemetry counter — both ship a placeholder
/// CRC observable to the host as a bad-CRC packet.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FoldExit {
    /// Fold wasn't active — nothing to do.
    Idle,
    /// Predecessor-byte target reached; trailing CRC patched in time.
    Finalized,
    /// The TX DMA channel prefetched into the trailing CRC slot before
    /// finalize — any patch would ship too late.
    WindowExpired,
    /// A drain pass folded nothing new — predecessor starvation backstop
    /// ([[busy-wait-plateau-backstop]]).
    Plateau,
}

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
    /// post-start residue fold that absorbs any GUARD bytes in flight at
    /// TX-start time and patches the trailing CRC slot before DMA1_CH4's
    /// prefetch reads it (doc §10.6.2 CC3 body). `drain` is one pass of
    /// the caller's RX drain, folding fresh bytes into the handed
    /// [`FoldEngine`]. No-op when the fold is idle.
    ///
    /// Returns the [`FoldExit`] disposition; the composite routes the two
    /// miss variants to telemetry (this half only decides *when* to stop,
    /// not what the miss means chip-side).
    pub fn on_tx_start<D>(&mut self, mut drain: D) -> FoldExit
    where
        D: FnMut(&mut FoldEngine<CRC>),
    {
        if !self.crc.is_active() {
            return FoldExit::Idle;
        }
        loop {
            if self.scheduler.patch_window_expired() {
                return FoldExit::WindowExpired;
            }
            let before = self.crc.bytes_folded();
            drain(&mut self.crc);
            if !self.crc.is_active() {
                return FoldExit::Finalized;
            }
            if self.crc.bytes_folded() == before {
                return FoldExit::Plateau;
            }
        }
    }

    // -- commands ---------------------------------------------------------------

    /// Start both halves for one Last reply: the scheduler builds its grid from
    /// the timing fields; the fold engine begins folding from
    /// `fold_start_cursor` with the same `predecessor_bytes` finalize cap.
    pub fn start(&mut self, p: FastLastSchedule) {
        self.scheduler.start(p);
        self.crc.start(p.fold_start_cursor, p.predecessor_bytes);
    }

    /// Return both halves to idle. Idempotent — the fold engine's finalize path and
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
