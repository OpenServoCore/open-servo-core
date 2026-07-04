//! Fast Last CRC fold pipeline — a mini §4.3 sub-composite of two
//! independent halves:
//!
//! - [`FsmScheduler`] — the periodic-walk CMP grid that paces classifier +
//!   parser + fold work across the predecessor window, plus the completion
//!   body that lands the chain-CRC patch after the last predecessor byte.
//! - [`FoldEngine`] — the running chain CRC and the bookkeeping that decides
//!   which wire bytes feed it, finalizing into a consumer-owned
//!   [`CrcPatchSink`].
//!
//! The wire start is hardware-armed through the TX scheduler and fires in
//! parallel with the grid — no fold work sits on the wire deadline; the
//! patch only races the TX DMA's read of the trailing CRC slot.
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

use crate::traits::dxl::FastLastScheduler;

pub use crc_patch_sink::CrcPatchSink;
pub use fold_engine::FoldEngine;
pub use fsm_scheduler::FsmScheduler;
pub use schedule::FastLastSchedule;

/// Exit disposition of one [`FsmScheduler::on_step`] grid body. The
/// composite routes [`Self::WindowExpired`] to the
/// `crc_patch_deadline_miss` telemetry counter — it ships a placeholder
/// CRC observable to the host as a bad-CRC packet.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FoldExit {
    /// Grid wasn't active — spurious CMP, nothing to do.
    Idle,
    /// Grid continues — the body re-armed the next CMP.
    Pending,
    /// Predecessor-byte target reached; trailing CRC patched in time.
    Finalized,
    /// The TX DMA channel prefetched into the trailing CRC slot before
    /// finalize — any patch would ship too late. Terminal for a starved
    /// fold (silent predecessor): the hardware kickoff fired regardless
    /// and the TX drain closed the window.
    WindowExpired,
}

/// Fast Last pipeline composite. Generic over its scheduler provider `S` and
/// CRC engine `CRC`; holds the two halves and routes between them.
pub struct FastLast<S: FastLastScheduler> {
    scheduler: FsmScheduler<S>,
    crc: FoldEngine,
}

impl<S: FastLastScheduler> FastLast<S> {
    pub fn new(scheduler: S) -> Self {
        Self {
            scheduler: FsmScheduler::new(scheduler),
            crc: FoldEngine::new(),
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
    pub fn split_mut(&mut self) -> (&mut FsmScheduler<S>, &mut FoldEngine) {
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

    /// Wire cursor one past the armed pickup window — the composite's
    /// cancel path disposes of the window's published bytes up to here.
    pub fn window_end_cursor(&self) -> u32 {
        self.crc.window_end_cursor()
    }
}
