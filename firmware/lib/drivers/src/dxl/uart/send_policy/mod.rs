//! Send-side policy sub-composite — the state that decides how this servo
//! participates in a DXL exchange, split out of the `DxlUart` composite per
//! driver-pattern §4.3. Owns [`ConfigMediator`] (bus identity + staged-
//! config mailbox) and [`ReplyGate`] (reply-side FSM over the staged Status
//! reply). [`SendPolicy::on_instruction`] walks a decoded instruction's
//! typed views into the [`ReplyContext`] the gate stages and the
//! [`DxlRequest`] the single-shot dispatcher consumes.

mod config_mediator;
mod dispatch;
mod fast_shape;
mod inflight;
mod reply_context;
mod reply_gate;

pub(super) use reply_context::ReplyContext;
pub(super) use reply_gate::StatusStartWait;

use config_mediator::ConfigMediator;
use dxl_protocol::types::packet::Instruction;
use osc_core::{BootMode, DxlRequest, DxlRequestCtx};
use reply_gate::ReplyGate;

use crate::dxl::uart::poll_src::PollSrc;

pub(super) struct SendPolicy {
    config: ConfigMediator,
    reply: ReplyGate,
}

impl SendPolicy {
    pub(super) const fn new(id: u8, rdt_us: u32) -> Self {
        Self {
            config: ConfigMediator::new(id, rdt_us),
            reply: ReplyGate::new(),
        }
    }
}

// -- events -------------------------------------------------------------

impl SendPolicy {
    /// Walk a decoded own/broadcast instruction: stage the derived
    /// [`ReplyContext`] on the reply gate and return the [`DxlRequest`] +
    /// [`DxlRequestCtx`] this servo dispatches, if it participates. `wr`
    /// backs the destuffed write payload for Write-family requests; the
    /// returned request borrows it. `None` when this servo is not a target
    /// (own id absent from a Sync/Bulk chain) — the reply gate bookkeeping
    /// still ran. Clears any stale predecessor wait first
    /// (`dxl-streaming-rx.md` §5.3): a foreign Status whose id happens to
    /// match must not trigger a spurious wire start on the new chain.
    pub(super) fn on_instruction<'w>(
        &mut self,
        instr: &Instruction<'_>,
        broadcast: bool,
        packet_end_tick: u32,
        fold_start_cursor: u32,
        src: PollSrc,
        wr: &'w mut [u8],
    ) -> Option<(DxlRequest<'w>, DxlRequestCtx)> {
        self.reply.on_new_instruction();
        let id = self.config.id();
        let (ctx, request) = dispatch::walk(instr, broadcast, id, wr);
        // Stage the reply context for participants and non-participants alike
        // (a broadcast chain we're absent from still resets stale waits).
        self.reply.on_reply_context(ctx.into_reply_context(
            id,
            self.config.rdt_us(),
            packet_end_tick,
            fold_start_cursor,
            src,
        ));
        request
    }

    /// A foreign packet's byte-skip completed under `pred`'s id. `true`
    /// when it was the awaited chain predecessor — the caller starts the
    /// deferred wire send now.
    pub(super) fn on_skip_complete(&mut self, pred: u8) -> bool {
        self.reply.on_skip_complete(pred)
    }

    /// The deferred FAST slot's wire start was scheduled off the observed
    /// status-start (or the wait was judged stale and dropped) — the wait
    /// is over.
    pub(super) fn on_status_start_scheduled(&mut self) {
        self.reply.on_status_start_scheduled()
    }

    /// Our reply fully drained the wire — the safe commit window for
    /// staged config; also drops any stale predecessor wait. Returns any
    /// staged reboot for the composite to route chip-side.
    pub(super) fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.reply.on_tx_complete();
        self.config.on_tx_complete()
    }
}

// -- commands -----------------------------------------------------------

impl SendPolicy {
    /// Consume the staged reply context for a `send_*` call — take-once;
    /// see [`ReplyGate::take_reply_context`].
    pub(super) fn take_reply_context(&mut self) -> Option<ReplyContext> {
        self.reply.take_reply_context()
    }

    /// Hold the encoded reply until `pred`'s Status frame finishes
    /// skipping — the Plain chain k > 0 deferral.
    pub(super) fn defer_to_predecessor(&mut self, pred: u8) {
        self.reply.defer_to_predecessor(pred)
    }

    /// Hold the encoded reply until the Status packet's first byte is
    /// observed on the wire — the FAST chain k > 0 deferral.
    pub(super) fn defer_to_status_start(&mut self, wait: StatusStartWait) {
        self.reply.defer_to_status_start(wait)
    }

    /// Drop the staged reply context and any predecessor wait.
    pub(super) fn cancel(&mut self) {
        self.reply.cancel()
    }

    pub(super) fn stage_id(&mut self, id: u8) {
        self.config.stage_id(id)
    }

    pub(super) fn stage_rdt(&mut self, us: u32) {
        self.config.stage_rdt(us)
    }

    pub(super) fn stage_reboot(&mut self, mode: BootMode) {
        self.config.stage_reboot(mode)
    }
}

// -- accessors ----------------------------------------------------------

impl SendPolicy {
    pub(super) fn id(&self) -> u8 {
        self.config.id()
    }

    /// The predecessor id a deferred chain reply is waiting on, if any.
    pub(super) fn awaited_predecessor(&self) -> Option<u8> {
        self.reply.awaited_predecessor()
    }

    /// The status-start wait a deferred FAST slot is parked on, if any.
    /// Non-consuming: the composite re-reads it across polls until the
    /// observation resolves or the wait goes stale.
    pub(super) fn awaited_status_start(&self) -> Option<StatusStartWait> {
        self.reply.awaited_status_start()
    }
}

#[cfg(test)]
impl SendPolicy {
    /// Peek the staged reply context without consuming it — composite
    /// tests assert on the derived wire shape before any send.
    pub(super) fn staged_reply_for_test(&self) -> Option<ReplyContext> {
        self.reply.staged()
    }

    /// Stage a hand-built context directly, bypassing the parse-side
    /// tracker — `ReplyHandle` tests drive the send paths in isolation.
    pub(super) fn stage_reply_context_for_test(&mut self, ctx: ReplyContext) {
        self.reply.on_reply_context(ctx);
    }

    pub(super) fn rdt_us(&self) -> u32 {
        self.config.rdt_us()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::test_support::TEST_ID;
    use dxl_protocol::Bytes;

    /// Walk one addressed Instruction and peek the staged reply context.
    fn staged_after(instr: &Instruction<'_>) -> ReplyContext {
        let mut p = SendPolicy::new(TEST_ID, 250);
        let mut wr = [0u8; 64];
        p.on_instruction(instr, true, 1000, 0, PollSrc::ByteBatch, &mut wr);
        p.staged_reply_for_test().expect("context staged")
    }

    fn sync_read(ids: &[u8]) -> Instruction<'_> {
        Instruction::SyncRead {
            address: 0,
            length: 2,
            ids: Bytes::raw(ids),
        }
    }

    /// A BulkRead body of `[id, addr16=0, len16=2]` entries.
    fn bulk_read_body(ids: &[u8]) -> heapless::Vec<u8, 64> {
        let mut body: heapless::Vec<u8, 64> = heapless::Vec::new();
        for &id in ids {
            body.extend_from_slice(&[id, 0, 0, 2, 0]).unwrap();
        }
        body
    }

    // Chain start for slots k > 0 (`docs/dxl-streaming-rx.md` §5.2): the
    // staged context names the immediate predecessor for Plain chains
    // only — slot 0 and Fast chains schedule without one.

    #[test]
    fn sync_read_slot_zero_has_no_predecessor() {
        let ctx = staged_after(&sync_read(&[TEST_ID]));
        assert_eq!(ctx.predecessor_id, None);
    }

    // FAST chain k > 0 deferral: the wait routes through the gate intact
    // and resolves at the composite's status-start observation.

    #[test]
    fn status_start_wait_round_trips_through_the_gate() {
        let mut p = SendPolicy::new(TEST_ID, 250);
        p.defer_to_status_start(StatusStartWait {
            status_start_cursor: 10,
            slot_offset_bytes: 14,
            latest_start_tick: 5000,
        });
        let w = p.awaited_status_start().expect("wait staged");
        assert_eq!(w.status_start_cursor, 10);
        assert_eq!(w.slot_offset_bytes, 14);
        assert_eq!(w.latest_start_tick, 5000);
        p.on_status_start_scheduled();
        assert!(p.awaited_status_start().is_none());
    }

    #[test]
    fn sync_read_slot_k_gt_zero_records_immediate_predecessor() {
        let ctx = staged_after(&sync_read(&[0x42, 0x09, TEST_ID]));
        assert_eq!(ctx.predecessor_id, Some(0x09));
    }

    #[test]
    fn bulk_read_slot_k_gt_zero_records_immediate_predecessor() {
        let body = bulk_read_body(&[0x42, TEST_ID]);
        let ctx = staged_after(&Instruction::BulkRead {
            body: Bytes::raw(&body),
        });
        assert_eq!(ctx.predecessor_id, Some(0x42));
    }

    #[test]
    fn fast_sync_read_slot_k_gt_zero_has_no_predecessor() {
        // Fast chains carry no per-slot Status headers → chain-start is
        // absolute-deadline, not sequence-driven. `predecessor_id` must
        // stay None per `docs/dxl-streaming-rx.md` §5.2.
        let ctx = staged_after(&Instruction::FastSyncRead {
            address: 0,
            length: 2,
            ids: Bytes::raw(&[0x42, TEST_ID]),
        });
        assert_eq!(ctx.predecessor_id, None);
    }
}
