//! Wire-positioning record for one staged reply. Pure data per
//! driver-pattern §4's carve-out — produced by
//! [`super::inflight::InflightCtx::into_reply_context`] at the parser's
//! Crc-good event, consumed field-by-field by the composite's send paths.

use crate::dxl::uart::poll_src::PollSrc;
use dxl_protocol::SlotPosition;

/// What the driver needs to position a reply on the wire. Built from the
/// `InflightCtx` slot-walk at the parser's Crc-good event; consumed by
/// `send_status` / `send_slot`. Per driver-pattern §7.4 — the dispatcher
/// passes data; the driver derives wire shape from its cached request state.
#[derive(Copy, Clone, Debug)]
pub(crate) struct ReplyContext {
    /// Packet-end tick (WireClock u32 domain) — anchored from the
    /// classifier at the parser's Crc-good event, or the codec's
    /// per-source fallback estimate when the classifier was unanchored
    /// (interference / edge loss).
    pub(crate) packet_end_tick: u32,
    /// Wire-byte offset from request wire-end to this reply's fire moment.
    /// Zero for direct unicast; non-zero for broadcast Ping and
    /// Sync/Bulk/Fast Read slot N. For Fast Last replies this also equals
    /// `predecessor_bytes` — the count of predecessor wire bytes the
    /// chain-CRC fold pipeline must absorb before patching our trailing
    /// CRC slot.
    pub(crate) slot_offset_bytes: u32,
    /// Fast Sync/Bulk Read slot position. `None` for non-Fast paths;
    /// `Some(Last { .. })` arms the Fast Last CRC fold pipeline so our
    /// own trailing CRC slot gets patched with the chain CRC covering
    /// predecessor + own bytes before DMA1_CH4 reads it.
    pub(crate) fast_slot_position: Option<SlotPosition>,
    /// Parser wire-byte cursor at parse-complete. Forwarded into
    /// [`FoldEngine::start`] as its `start_cursor` — the first
    /// predecessor reply byte arrives at exactly this cursor, so the
    /// fold's `cursor < start_cursor` guard skips everything up to (but
    /// not including) the first predecessor byte.
    ///
    /// [`FoldEngine::start`]: crate::dxl::uart::fast_last::FastLast
    pub(crate) fold_start_cursor: u32,
    /// `Some(predecessor_id)` for Plain Sync / Bulk Read chain slots at
    /// k > 0 — the sequence-driven fire path of `docs/dxl-streaming-rx.md`
    /// §5.2. `None` for single-target replies, slot 0 of any chain, and
    /// all Fast chain replies. When Some, `send_status` defers the wire
    /// send to the codec's matching `PollEvent::SkipComplete` event.
    pub(crate) predecessor_id: Option<u8>,
    /// RDT (µs) the deadline math adds to `packet_end_tick`. Resolved at
    /// `into_reply_context` build time so the send path is rdt-source-
    /// agnostic: single-target / Fast chain replies see the per-instance
    /// register value, broadcast Ping sees the uniform driver default
    /// (`crate::dxl::DEFAULT_RDT_US`) — see `DEFAULT_RDT_2US`'s doc for the
    /// collision-avoidance reasoning.
    pub(crate) rdt_us: u32,
    /// Which ISR fired the parser's Crc event (HT/TC vs USART IDLE). The
    /// Fast slot send path floors the effective RDT by the source's
    /// `now − packet_end` offset so slot 0's fire wall doesn't slip behind
    /// the IDLE-poll horizon while slot k > 0's CCR1 stays anchored to the
    /// raw RDT — that mismatch would land slot k > 0 inside slot 0's TX.
    /// See `ReplyHandle::send_slot`.
    pub(crate) src: PollSrc,
}
