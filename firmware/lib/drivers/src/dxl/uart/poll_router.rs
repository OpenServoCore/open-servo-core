//! Per-poll event router — disjoint borrows of the `DxlUart` composite's
//! sub-drivers, bundled once per [`DxlUart::poll`] so the poll closure
//! routes each codec event through named methods instead of one inline
//! match. Sibling of [`ReplyHandle`] (the same §4.3 borrow-bundle shape,
//! aimed at the dispatcher instead of the poll loop); [`Self::reply`]
//! reborrows the handle's five fields per event.
//!
//! [`DxlUart::poll`]: super::DxlUart::poll

use dxl_protocol::streaming::{CrcResult, Event, HeaderEvent, PayloadEvent};

use super::clock::Clock;
use super::codec::{CodecTx, PacketEnd, PollAction};
use super::fast_last::FastLast;
use super::reply_handle::ReplyHandle;
use super::send_policy::{SendPolicy, header_target};
use crate::traits::dxl::{Providers, RxDma, TxBus};

pub(super) struct PollRouter<'a, P: Providers, const TX_BUF_LEN: usize> {
    pub(super) tx: &'a mut CodecTx<P::Crc, TX_BUF_LEN>,
    pub(super) scheduler: &'a mut P::TxScheduler,
    pub(super) tx_bus: &'a mut P::TxBus,
    pub(super) fast_last: &'a mut FastLast<P::FastLastScheduler, P::Crc>,
    pub(super) clock: &'a mut Clock<P::UsartBaud, P::ClockTrim>,
    pub(super) send_policy: &'a mut SendPolicy,
    pub(super) rx_dma: &'a mut P::RxDma,
    /// Live bus ID snapshot for log lines — stable for the poll's duration
    /// (staged ID commits only at `on_tx_complete`).
    pub(super) id: u8,
}

// -- events -------------------------------------------------------------

impl<P: Providers, const TX_BUF_LEN: usize> PollRouter<'_, P, TX_BUF_LEN> {
    /// Route one parser [`Event`] into the send policy. The returned
    /// action is the skip/continue decision the codec resumes with —
    /// `Skip` engages the universal byte-skip on foreign bodies
    /// (doc §3 driver rule).
    #[inline(always)]
    pub(super) fn on_parser_event(
        &mut self,
        ev: Event,
        next_status_pos: u32,
        packet_end: Option<PacketEnd>,
    ) -> PollAction {
        match ev {
            Event::Header(HeaderEvent::Instruction(h)) => {
                let skip_target = self.send_policy.on_instruction_header(&h);
                crate::log::trace!(
                    "dxl[id={}]: event=header_instruction target={} addressable={}",
                    self.id,
                    header_target(&h).as_byte(),
                    skip_target.is_none()
                );
                match skip_target {
                    None => PollAction::Continue,
                    Some(target) => PollAction::Skip { id: target },
                }
            }
            Event::Header(HeaderEvent::Status(sh)) => {
                crate::log::trace!(
                    "dxl[id={}]: event=header_status status_id={}",
                    self.id,
                    sh.id.as_byte()
                );
                self.send_policy.on_status_header();
                PollAction::Skip {
                    id: sh.id.as_byte(),
                }
            }
            Event::Payload(PayloadEvent::Instruction(p)) => {
                crate::log::trace!("dxl[id={}]: event=payload_instruction", self.id);
                self.send_policy.on_slot(&p);
                PollAction::Continue
            }
            Event::Payload(PayloadEvent::Status(_)) => {
                debug_assert!(false, "Status payload should have been byte-skipped");
                PollAction::Continue
            }
            Event::Crc(CrcResult::Good) => self.on_crc_good(next_status_pos, packet_end),
            Event::Crc(CrcResult::Bad) | Event::Resync(_) => {
                crate::log::trace!("dxl[id={}]: event=crc(bad)/resync", self.id);
                self.send_policy.on_resync();
                PollAction::Continue
            }
            Event::Sync => PollAction::Continue,
        }
    }

    /// Crc-good on the in-flight packet. The event carries codec-resolved
    /// packet-end timing (anchored wire-end tick, or `None` when
    /// interference / edge loss starved the tail-anchor back-search, plus
    /// the per-source ISR-entry estimate). Policy lives here: the anchor
    /// miss counts as RX telemetry, and the fallback estimate is
    /// acceptable only for ops that allow it — FAST chain ops don't, see
    /// `SendPolicy::allows_packet_end_fallback`.
    #[inline(always)]
    fn on_crc_good(&mut self, next_status_pos: u32, packet_end: Option<PacketEnd>) -> PollAction {
        crate::log::trace!("dxl[id={}]: event=crc(good)", self.id);
        if self.send_policy.is_tracking()
            && let Some(pe) = packet_end
        {
            let packet_end_tick = match pe.tick {
                Some(t) => Some(t),
                None => {
                    self.rx_dma.record_edge_anchor_miss();
                    self.send_policy
                        .allows_packet_end_fallback()
                        .then_some(pe.fallback_tick)
                }
            };
            // At Crc-of-host-instruction, the codec's wire position has
            // just walked past the request's last CRC byte — the next
            // byte on the wire is the First predecessor's leading `0xFF`.
            // So `next_status_pos` is exactly the fold-start cursor for
            // the Fast Last CRC engine.
            match self
                .send_policy
                .on_crc_good(packet_end_tick, next_status_pos, pe.src)
            {
                Some(t) => {
                    crate::log::debug!("dxl[id={}]: crc packet_end_tick={}", self.id, t);
                }
                None => {
                    crate::log::debug!(
                        "dxl: crc anchor missing and fallback disallowed — drop reply"
                    );
                }
            }
        }
        PollAction::Continue
    }

    /// A foreign packet's byte-skip completed under `pred`'s id. When it
    /// was the awaited chain predecessor, start the deferred wire send
    /// now — the Plain chain k > 0 sequence-driven path
    /// (`docs/dxl-streaming-rx.md` §5.2).
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
        // Codec clears `packet_is_instruction` internally on
        // SkipComplete; no anchor is set between the previous Crc (where
        // the codec resets it) and here, so no driver-side invalidation
        // is required.
    }
}

// -- accessors ------------------------------------------------------------

impl<P: Providers, const TX_BUF_LEN: usize> PollRouter<'_, P, TX_BUF_LEN> {
    /// Per-event reply handle over the router's borrowed halves — built
    /// fresh for each dispatcher call, reborrowing the five send-side
    /// fields.
    #[inline(always)]
    pub(super) fn reply(&mut self) -> ReplyHandle<'_, P, TX_BUF_LEN> {
        ReplyHandle {
            tx: &mut *self.tx,
            scheduler: &mut *self.scheduler,
            fast_last: &mut *self.fast_last,
            clock: &mut *self.clock,
            send_policy: &mut *self.send_policy,
        }
    }
}
