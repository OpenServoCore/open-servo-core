//! Reply-side FSM — the staged Status reply's path to the wire.
//! Transitions on reply events only (context staged at Crc-good,
//! dispatcher send, predecessor skip completion, TX completion); one
//! exchange's lifetime. Runs out of phase with the parse-side
//! `InstructionTracker`: a staged context the dispatcher never consumed
//! (e.g. a broadcast write with no Status) survives the next
//! Instruction's header and is displaced at that packet's Crc-good.

use super::reply_context::ReplyContext;

/// Deferred-schedule record for a FAST Sync/Bulk Read slot k > 0 — the
/// reply is encoded, its wire start waits on the observed start of the
/// chain's single Status packet (task #142 model). Payload of
/// [`ReplyPhase::AwaitingStatusStart`]; pure data crossing the gate
/// boundary (driver-pattern §3.3).
#[derive(Copy, Clone, Debug)]
pub(crate) struct StatusStartWait {
    /// Wire cursor where the Status packet's first byte lands — the
    /// instruction Crc's `next_status_pos` (= `fold_start_cursor`).
    pub(crate) status_start_cursor: u32,
    /// Wire bytes from the Status packet's first byte to our slot's
    /// first byte (= [`ReplyContext::slot_offset_bytes`]).
    pub(crate) slot_offset_bytes: u32,
    /// Latest plausible status-start tick (WireClock u32): packet end
    /// plus effective RDT plus turnaround slack. A stamp past this is
    /// stale traffic (the host's retry after a dead chain), not the
    /// awaited reply: drop the slot instead of scheduling into the
    /// host's instruction. Hygiene bound only, never a wire TX deadline.
    pub(crate) latest_start_tick: u32,
    /// Slot position is Last — on observation the composite also starts
    /// the Fast Last fold/grid off the observed anchor.
    pub(crate) is_last: bool,
}

/// Wire-placement phase of the staged reply.
enum ReplyPhase {
    /// Nothing staged. A reply already handed to the scheduler is the
    /// scheduler's business, not this machine's.
    Idle,
    /// Crc-good derived a context; waiting for the dispatcher's `send_*`
    /// call to consume it.
    Staged(ReplyContext),
    /// Plain Sync/Bulk chain k > 0: reply encoded, wire start deferred
    /// until the named predecessor's Status frame finishes skipping —
    /// the sequence-driven start path of `docs/dxl-streaming-rx.md` §5.2.
    AwaitingPredecessor(u8),
    /// FAST Sync/Bulk chain k > 0: reply encoded, wire start deferred
    /// until the Status packet's first byte is observed on the wire.
    AwaitingStatusStart(StatusStartWait),
}

pub(super) struct ReplyGate {
    phase: ReplyPhase,
}

impl ReplyGate {
    pub(super) const fn new() -> Self {
        Self {
            phase: ReplyPhase::Idle,
        }
    }

    /// The awaiting phases are the only ones external packet boundaries
    /// may clear; a staged context stays put (displaced at the next
    /// Crc-good, exactly the pre-FSM overwrite semantics).
    fn clear_awaiting(&mut self) {
        if let ReplyPhase::AwaitingPredecessor(_) | ReplyPhase::AwaitingStatusStart(_) = self.phase
        {
            self.phase = ReplyPhase::Idle;
        }
    }
}

// -- events -------------------------------------------------------------

impl ReplyGate {
    /// Crc-good derived a fresh context — stage it, displacing any
    /// unconsumed one.
    pub(super) fn on_reply_context(&mut self, ctx: ReplyContext) {
        self.phase = ReplyPhase::Staged(ctx);
    }

    /// A foreign packet's byte-skip completed under `pred`'s id. `true`
    /// when it was the awaited predecessor — the caller starts the wire
    /// send now; the wait is over either way on a match.
    pub(super) fn on_skip_complete(&mut self, pred: u8) -> bool {
        match self.phase {
            ReplyPhase::AwaitingPredecessor(p) if p == pred => {
                self.phase = ReplyPhase::Idle;
                true
            }
            _ => false,
        }
    }

    /// The deferred FAST slot's wire start was scheduled off the observed
    /// status-start (or the wait was judged stale and dropped) — either
    /// way the wait is over.
    pub(super) fn on_status_start_scheduled(&mut self) {
        if let ReplyPhase::AwaitingStatusStart(_) = self.phase {
            self.phase = ReplyPhase::Idle;
        }
    }

    /// A new Instruction header parsed — stale chain-pending state from a
    /// silent predecessor must reset per `dxl-streaming-rx.md` §5.3, or a
    /// foreign Status whose id happens to match would trigger a spurious
    /// wire start on the new chain.
    pub(super) fn on_new_instruction(&mut self) {
        self.clear_awaiting();
    }

    /// Crc-bad / parser resync — the wire state is void; drop the wait.
    pub(super) fn on_resync(&mut self) {
        self.clear_awaiting();
    }

    /// Our reply fully drained the wire. Belt-and-suspenders — the
    /// SkipComplete path clears the wait on success, but a silent
    /// predecessor would leave it staged across the next reply; clearing
    /// here bounds the chain-pending state by the in-flight exchange.
    pub(super) fn on_tx_complete(&mut self) {
        self.clear_awaiting();
    }
}

// -- commands -----------------------------------------------------------

impl ReplyGate {
    /// Consume the staged context for a `send_*` call. Take-once: a
    /// double-send without a fresh Crc-good gets `None` and no-ops on the
    /// scheduler.
    pub(super) fn take_reply_context(&mut self) -> Option<ReplyContext> {
        match core::mem::replace(&mut self.phase, ReplyPhase::Idle) {
            ReplyPhase::Staged(ctx) => Some(ctx),
            other => {
                self.phase = other;
                None
            }
        }
    }

    /// The taken context named a chain predecessor — hold the encoded
    /// reply until that id's Status frame finishes skipping.
    pub(super) fn defer_to_predecessor(&mut self, pred: u8) {
        self.phase = ReplyPhase::AwaitingPredecessor(pred);
    }

    /// The taken context is a FAST slot k > 0 — hold the encoded reply
    /// until the Status packet's first byte is observed on the wire.
    pub(super) fn defer_to_status_start(&mut self, wait: StatusStartWait) {
        self.phase = ReplyPhase::AwaitingStatusStart(wait);
    }

    /// Drop the staged context and any deferred wait; the next reply
    /// must come through a fresh Crc-good.
    pub(super) fn cancel(&mut self) {
        self.phase = ReplyPhase::Idle;
    }
}

// -- accessors ----------------------------------------------------------

impl ReplyGate {
    /// The predecessor id a deferred chain reply is waiting on, if any.
    pub(super) fn awaited_predecessor(&self) -> Option<u8> {
        match self.phase {
            ReplyPhase::AwaitingPredecessor(p) => Some(p),
            _ => None,
        }
    }

    /// The status-start wait a deferred FAST slot is parked on, if any.
    pub(super) fn awaited_status_start(&self) -> Option<StatusStartWait> {
        match self.phase {
            ReplyPhase::AwaitingStatusStart(w) => Some(w),
            _ => None,
        }
    }
}

#[cfg(test)]
impl ReplyGate {
    /// Peek the staged context without consuming it.
    pub(super) fn staged(&self) -> Option<ReplyContext> {
        match self.phase {
            ReplyPhase::Staged(ctx) => Some(ctx),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::poll_src::PollSrc;

    fn ctx() -> ReplyContext {
        ReplyContext {
            packet_end_tick: 100,
            slot_offset_bytes: 0,
            fast_slot_position: None,
            fold_start_cursor: 0,
            predecessor_id: None,
            rdt_us: 250,
            src: PollSrc::ByteBatch,
        }
    }

    #[test]
    fn staged_context_takes_once() {
        let mut g = ReplyGate::new();
        g.on_reply_context(ctx());
        assert!(g.take_reply_context().is_some());
        assert!(g.take_reply_context().is_none());
    }

    #[test]
    fn take_while_awaiting_returns_none_and_keeps_the_wait() {
        let mut g = ReplyGate::new();
        g.defer_to_predecessor(0x42);
        assert!(g.take_reply_context().is_none());
        assert_eq!(g.awaited_predecessor(), Some(0x42));
    }

    #[test]
    fn skip_complete_fires_only_on_the_awaited_predecessor() {
        let mut g = ReplyGate::new();
        g.defer_to_predecessor(0x42);
        assert!(!g.on_skip_complete(0x05));
        assert_eq!(g.awaited_predecessor(), Some(0x42));
        assert!(g.on_skip_complete(0x42));
        assert_eq!(g.awaited_predecessor(), None);
        assert!(!g.on_skip_complete(0x42), "wait consumed on match");
    }

    fn wait() -> StatusStartWait {
        StatusStartWait {
            status_start_cursor: 10,
            slot_offset_bytes: 14,
            latest_start_tick: 5000,
            is_last: false,
        }
    }

    #[test]
    fn packet_boundaries_clear_the_waits_but_keep_a_staged_context() {
        for clear in [
            ReplyGate::on_new_instruction,
            ReplyGate::on_resync,
            ReplyGate::on_tx_complete,
        ] {
            let mut g = ReplyGate::new();
            g.defer_to_predecessor(0x42);
            clear(&mut g);
            assert_eq!(g.awaited_predecessor(), None);

            let mut g = ReplyGate::new();
            g.defer_to_status_start(wait());
            clear(&mut g);
            assert!(g.awaited_status_start().is_none());

            let mut g = ReplyGate::new();
            g.on_reply_context(ctx());
            clear(&mut g);
            assert!(g.take_reply_context().is_some(), "staged context survives");
        }
    }

    #[test]
    fn cancel_clears_all_phases() {
        let mut g = ReplyGate::new();
        g.on_reply_context(ctx());
        g.cancel();
        assert!(g.take_reply_context().is_none());

        g.defer_to_predecessor(0x42);
        g.cancel();
        assert_eq!(g.awaited_predecessor(), None);

        g.defer_to_status_start(wait());
        g.cancel();
        assert!(g.awaited_status_start().is_none());
    }

    #[test]
    fn status_start_wait_peeks_until_scheduled() {
        let mut g = ReplyGate::new();
        g.defer_to_status_start(wait());
        // Peek is non-consuming: retries across polls read the same wait.
        assert_eq!(g.awaited_status_start().unwrap().slot_offset_bytes, 14);
        assert_eq!(g.awaited_status_start().unwrap().status_start_cursor, 10);
        g.on_status_start_scheduled();
        assert!(g.awaited_status_start().is_none());
    }

    #[test]
    fn take_while_awaiting_status_start_returns_none_and_keeps_the_wait() {
        let mut g = ReplyGate::new();
        g.defer_to_status_start(wait());
        assert!(g.take_reply_context().is_none());
        assert!(g.awaited_status_start().is_some());
    }

    #[test]
    fn skip_complete_does_not_touch_a_status_start_wait() {
        // The two deferral mechanisms are disjoint: a foreign skip-exhaust
        // (Plain chain machinery) must not release a FAST slot.
        let mut g = ReplyGate::new();
        g.defer_to_status_start(wait());
        assert!(!g.on_skip_complete(0x42));
        assert!(g.awaited_status_start().is_some());
    }

    #[test]
    fn fresh_context_displaces_a_stale_one() {
        let mut g = ReplyGate::new();
        g.on_reply_context(ctx());
        let fresh = ReplyContext {
            packet_end_tick: 999,
            ..ctx()
        };
        g.on_reply_context(fresh);
        assert_eq!(g.take_reply_context().unwrap().packet_end_tick, 999);
    }
}
