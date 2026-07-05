//! Per-poll verdict router — disjoint borrows of the `DxlUart` composite's
//! sub-drivers, bundled once per [`DxlUart::poll`] so the poll closure routes
//! each frame verdict through named methods instead of one inline match.
//! Sibling of [`ReplyHandle`] (the same §4.3 borrow-bundle shape, aimed at
//! the dispatcher instead of the poll loop); [`Self::reply`] reborrows the
//! handle's fields per verdict.
//!
//! [`DxlUart::poll`]: super::DxlUart::poll

use dxl_protocol::types::packet::Instruction;
use osc_core::{DxlRequest, DxlRequestCtx};

use super::clock::Clock;
use super::codec::{CodecTx, FrameAction, HELD_FRAME_MAX, PacketEnd};
use super::drift_window::RxWakeGate;
use super::fast_last::FastLast;
use super::reply_handle::ReplyHandle;
use super::send_policy::{SendPolicy, synthesize};
use crate::traits::dxl::{Providers, TxBus};

pub(super) struct PollRouter<'a, P: Providers, const TX_BUF_LEN: usize> {
    pub(super) tx: &'a mut CodecTx<P::Crc, TX_BUF_LEN>,
    pub(super) scheduler: &'a mut P::TxScheduler,
    pub(super) tx_bus: &'a mut P::TxBus,
    pub(super) fast_last: &'a mut FastLast<P::FastLastScheduler>,
    pub(super) clock: &'a mut Clock<P::UsartBaud, P::ClockTrim>,
    pub(super) send_policy: &'a mut SendPolicy,
    /// Threaded through to [`ReplyHandle`] for the FAST k > 0 defer path's
    /// status-start wake window; the router itself never touches it.
    pub(super) rx_dma: &'a mut P::RxDma,
    /// RXNEIE reason set — threaded to [`ReplyHandle`] so the FAST defer
    /// path opens the wake through the shared gate (OR-edge), not by
    /// toggling `rx_dma` directly.
    pub(super) rx_wake: &'a mut RxWakeGate,
    /// Live bus ID snapshot for log lines — stable for the poll's duration
    /// (staged ID commits only at `on_tx_complete`).
    pub(super) id: u8,
}

// -- events -------------------------------------------------------------

impl<P: Providers, const TX_BUF_LEN: usize> PollRouter<'_, P, TX_BUF_LEN> {
    /// Route one decoded own/broadcast instruction frame: replay it onto the
    /// send policy (staging the reply context), derive the request + ctx,
    /// and hand them to the dispatcher closure `f` once. Returns
    /// [`FrameAction::Stop`] when the dispatch engaged the Fast Last fold or
    /// a deferred successor wait — the in-flight poll must bail so those
    /// consumers own the remaining ring bytes (doc §6).
    #[inline(always)]
    pub(super) fn on_instruction<F>(
        &mut self,
        instr: Instruction<'_>,
        broadcast: bool,
        packet_end: PacketEnd,
        fold_start_cursor: u32,
        f: &mut F,
    ) -> FrameAction
    where
        F: FnMut(DxlRequest<'_>, DxlRequestCtx, &mut ReplyHandle<'_, P, TX_BUF_LEN>),
    {
        // Backs the destuffed write payload for a Write-family request; the
        // derived request borrows it for the duration of the dispatch call.
        let mut wr = [0u8; HELD_FRAME_MAX];
        let derived = synthesize(
            &mut *self.send_policy,
            &instr,
            broadcast,
            packet_end.tick,
            fold_start_cursor,
            packet_end.src,
            &mut wr,
        );
        if let Some((req, ctx)) = derived {
            crate::log::trace!(
                "dxl[id={}]: dispatch broadcast={} may_reply={}",
                self.id,
                ctx.broadcast,
                ctx.may_reply
            );
            let mut reply = self.reply();
            f(req, ctx, &mut reply);
        }
        if self.fast_last.fold_active() || self.send_policy.awaited_status_start().is_some() {
            FrameAction::Stop
        } else {
            FrameAction::Continue
        }
    }

    /// A foreign packet's byte-skip completed under `pred`'s id. When it was
    /// the awaited chain predecessor, start the deferred wire send now — the
    /// Plain chain k > 0 sequence-driven path (`docs/dxl-streaming-rx.md`
    /// §5.2).
    #[inline(always)]
    pub(super) fn on_skip_complete(&mut self, pred: u8) {
        crate::log::trace!(
            "dxl[id={}]: skip_complete pred={} predecessor_id={:?}",
            self.id,
            pred,
            self.send_policy.awaited_predecessor()
        );
        if self.send_policy.on_skip_complete(pred) {
            crate::log::debug!(
                "dxl[id={}]: skip_complete match pred={} -> start_now byte_count={}",
                self.id,
                pred,
                self.tx.tx_len()
            );
            self.tx_bus.start_now(self.tx.tx_len());
        }
    }
}

// -- accessors ------------------------------------------------------------

impl<P: Providers, const TX_BUF_LEN: usize> PollRouter<'_, P, TX_BUF_LEN> {
    /// Per-verdict reply handle over the router's borrowed halves — built
    /// fresh for each dispatcher call, reborrowing the send-side fields.
    #[inline(always)]
    pub(super) fn reply(&mut self) -> ReplyHandle<'_, P, TX_BUF_LEN> {
        ReplyHandle {
            tx: &mut *self.tx,
            scheduler: &mut *self.scheduler,
            clock: &mut *self.clock,
            rx_dma: &mut *self.rx_dma,
            rx_wake: &mut *self.rx_wake,
            send_policy: &mut *self.send_policy,
        }
    }
}
