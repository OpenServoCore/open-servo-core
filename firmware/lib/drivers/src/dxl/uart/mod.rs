//! DXL-over-UART transport. Composite over the codec (bytes ↔ packets)
//! and the clock (tick math + drift integration). The chip-side ISR layer
//! only ever reaches through `DxlUart`; cross-sub-driver routing (e.g.
//! the BT-pair walk that feeds drift samples from codec into clock) lives
//! in this file per driver-pattern §4 + §10.1.
//!
//! Generic over its leaf providers AND its three storage sizes — see the
//! [`DxlUart`] doc. `firmware/ch32/src/runtime/registry.rs` binds each to
//! its V006 value. Future sub-drivers (`Tx`, `ChainCatchup`) land as
//! additional fields on this composite (TX work also extends `Codec`).

pub mod clock;
pub mod codec;
pub mod fast_last;
pub mod fast_last_crc;

use dxl_protocol::streaming::{
    Event, HeaderEvent, InstructionHeader, InstructionPayload, PayloadEvent,
};
use dxl_protocol::wire::{BROADCAST_ID, CRC_BYTES, RESPONSE_HEADER_BYTES};
use dxl_protocol::{Id, Slot, SlotPosition, Status, WriteError};
use osc_core::{BaudRate, BootMode, DxlReply};

use crate::traits::dxl::{Providers, RxDma, SendKind, TxBus, TxScheduler, WireClock};
use clock::Clock;
use codec::{Codec, CodecTx, PollAction, PollEvent};
use fast_last::{FastLast, FastLastSchedule};
use fast_last_crc::FastLastCrc;

/// Bits on the wire for a single UART character: 1 start + 8 data + 1 stop
/// (8N1). Multiply by `ticks_per_bit` to get one byte's wire duration in
/// scheduler ticks. Also the IDLE detection threshold — CH32V00X RM §UART:
/// "an idle frame is 10-or-11-bit high, including the stop bit"; M=0 → 10.
pub(crate) const BITS_PER_FRAME: u16 = 10;

/// Wire bytes of a single Ping Status reply — `RESPONSE_HEADER_BYTES`
/// (header(4) + id + len(2) + inst + err = 9) + 3 payload bytes
/// (model_lo + model_hi + firmware) + CRC_BYTES (2). Multi-servo broadcast
/// Ping convention positions servo N's reply at `N × PING_STATUS_FRAME_BYTES`
/// wire bytes past wire-end so the servos don't collide.
const PING_STATUS_FRAME_BYTES: u32 = RESPONSE_HEADER_BYTES as u32 + 3 + CRC_BYTES as u32;

/// What the driver needs to position a reply on the wire. Built from the
/// `InflightCtx` slot-walk at the parser's Crc-good event; consumed by
/// `send_status` / `send_slot`. Per driver-pattern §7.4 — the dispatcher
/// passes data; the driver derives wire shape from its cached request state.
#[derive(Copy, Clone, Debug, Default)]
struct ReplyContext {
    /// Packet-end tick — anchored from the classifier at the parser's
    /// Crc-good event, or a fallback estimate when the classifier was
    /// unanchored (interference / edge loss). See
    /// [`crate::dxl::uart::codec::rx::Classifier::packet_end_tick_fallback`].
    packet_end_tick: u16,
    /// Wire-byte offset from request wire-end to this reply's fire moment.
    /// Zero for direct unicast; non-zero for broadcast Ping and
    /// Sync/Bulk/Fast Read slot N. For Fast Last replies this also equals
    /// `predecessor_bytes` — the count of predecessor wire bytes the
    /// chain-CRC fold pipeline must absorb before patching our trailing
    /// CRC slot.
    slot_offset_bytes: u32,
    /// Fast Sync/Bulk Read slot position. `None` for non-Fast paths;
    /// `Some(Last { .. })` arms the Fast Last CRC fold pipeline so our
    /// own trailing CRC slot gets patched with the chain CRC covering
    /// predecessor + own bytes before DMA1_CH4 reads it.
    fast_slot_position: Option<SlotPosition>,
    /// Parser wire-byte cursor at parse-complete. Forwarded into
    /// [`FastLastCrc::start`] as its `start_cursor` — the first
    /// predecessor reply byte arrives at exactly this cursor, so the
    /// fold's `cursor < start_cursor` guard skips everything up to (but
    /// not including) the first predecessor byte.
    fold_start_cursor: u32,
    /// `Some(predecessor_id)` for Plain Sync / Bulk Read chain slots at
    /// k > 0 — the sequence-driven fire path of `docs/dxl-streaming-rx.md`
    /// §5.2. `None` for single-target replies, slot 0 of any chain, and
    /// all Fast chain replies. When Some, `send_status` defers the wire
    /// send to the codec's matching `PollEvent::SkipComplete` event.
    predecessor_id: Option<u8>,
}

/// Slot-header bytes a Fast First/Only emission carries (`FF FF FD 00` +
/// BROADCAST id + length(2) + Status instruction = 8 bytes). Sized to match
/// `emit_slot_header` in `dxl-protocol`.
const FAST_SLOT_HEADER_BYTES: u32 = 8;

/// Per-slot body bytes excluding the trailing CRC: `error(1) + id(1) + data`.
fn fast_slot_body_bytes(length: u32) -> u32 {
    2 + length
}

/// Wire bytes a Fast First emission consumes for a per-slot `length`: header +
/// body, no trailing CRC (successors continue).
fn fast_first_bytes(length: u32) -> u32 {
    FAST_SLOT_HEADER_BYTES + fast_slot_body_bytes(length)
}

/// Wire bytes a Fast Middle emission consumes for a per-slot `length`: body
/// only.
fn fast_middle_bytes(length: u32) -> u32 {
    fast_slot_body_bytes(length)
}

/// Target id carried in any Instruction header. Mirrors
/// `osc_core::services::dxl::dispatcher::header_target` — copied here to keep
/// osc-drivers self-contained per the driver-pattern doc.
fn header_target(h: &InstructionHeader) -> Id {
    use InstructionHeader::*;
    match *h {
        Ping { id }
        | Read { id, .. }
        | Write { id, .. }
        | RegWrite { id, .. }
        | Action { id }
        | Reboot { id }
        | FactoryReset { id, .. }
        | Clear { id, .. }
        | ControlTableBackup { id, .. }
        | SyncRead { id, .. }
        | SyncWrite { id, .. }
        | BulkRead { id }
        | BulkWrite { id }
        | FastSyncRead { id, .. }
        | FastBulkRead { id }
        | Raw { id, .. } => id,
    }
}

/// True when the instruction's target is the chip's own ID or BROADCAST.
fn target_addressable(h: &InstructionHeader, id: u8) -> bool {
    let target = header_target(h);
    target.as_byte() == id || target.as_byte() == BROADCAST_ID
}

/// Per-packet wire-state aggregator. Lives while the parser is inside an
/// instruction the chip will reply to; consumed at the Crc-good event to
/// derive the [`ReplyContext`] the send path needs. State accumulates from
/// header + per-slot demarcation events; finalized math lives in
/// [`InflightCtx::into_reply_context`].
#[derive(Copy, Clone, Debug)]
struct InflightCtx {
    header: InstructionHeader,
    /// Slot-walk cursor. Bumped on each SyncSlot/BulkSlot event; resolves to
    /// the chain's `n_total` at Crc time.
    next_slot_index: u8,
    /// Index of the chip's own slot once identified. Stays `None` for
    /// non-Sync/Bulk variants and for chains the chip is not in.
    slot: Option<u8>,
    /// Wire bytes preceding the chip's slot — accumulated incrementally
    /// during BulkRead / FastBulkRead walks (per-slot length varies);
    /// derived at Crc for SyncRead / FastSyncRead (uniform per-slot length).
    bytes_before: u32,
    /// Per-slot length captured at BulkSlot demarcation just before the chip's
    /// own slot, used to size FastBulkRead's `Middle`/`Last` emission. None for
    /// non-Bulk variants.
    slot_length: Option<u16>,
    /// Sum of per-slot data byte counts across the chain — feeds Fast
    /// chain `packet_length` math for First/Only emissions. Accumulated
    /// during `slot_walk` for FastBulkRead (per-slot length varies); for
    /// FastSyncRead derived in `into_reply_context` from header.length ×
    /// `n_total`.
    chain_data_bytes: u32,
    /// Slot ID seen at the most recent SyncSlot / BulkSlot demarcation
    /// before the chip's own slot lands. Updates per demarcation while
    /// `slot` is still `None`; freezes once `slot` resolves — that's the
    /// chip's chain predecessor for sequence-driven scheduling
    /// (`docs/dxl-streaming-rx.md` §5.2). Stays `None` for slot 0 of any
    /// chain and for non-chain instructions.
    predecessor_id: Option<u8>,
}

impl InflightCtx {
    fn new(header: InstructionHeader) -> Self {
        Self {
            header,
            next_slot_index: 0,
            slot: None,
            bytes_before: 0,
            slot_length: None,
            chain_data_bytes: 0,
            predecessor_id: None,
        }
    }

    /// Whether a `packet_end_tick` fallback estimate is safe to use when
    /// the classifier lost anchor. FAST chain ops drive chain-CRC
    /// fold-grid back-dating; a guessed anchor mispatches the CRC by ≥1
    /// byte and corrupts the entire downstream chain — strictly worse
    /// than silence. Single-target replies (Ping / Read / Write /
    /// RegWrite) absorb the ~µs-scale fallback jitter inside RDT slack;
    /// plain Sync/Bulk Read slot k>0 schedules sequence-driven from the
    /// predecessor and ignores `packet_end_tick` entirely.
    fn allows_packet_end_fallback(&self) -> bool {
        !matches!(
            self.header,
            InstructionHeader::FastSyncRead { .. } | InstructionHeader::FastBulkRead { .. },
        )
    }

    /// Final ReplyContext at Crc-good. `packet_end_tick` is captured from the
    /// classifier at the same event; `fold_start_cursor` is the codec's
    /// wire-byte cursor at the parser's Crc emit point — the cursor where
    /// the First predecessor reply byte will land (the host's chain
    /// instruction is fully consumed by then, so the next wire byte is the
    /// First servo's `0xFF`).
    fn into_reply_context(
        self,
        id: u8,
        packet_end_tick: u16,
        fold_start_cursor: u32,
    ) -> ReplyContext {
        // Plain SyncRead / BulkRead don't appear here: slot 0 of either
        // chain is single-target (slot_offset_bytes = 0 — handled by the
        // catch-all), and slot k > 0 takes the chain-pending path in
        // `ReplyHandle::send_status`, which never reads slot_offset_bytes.
        let (slot_offset_bytes, fast_slot_position) = match (self.header, self.slot) {
            (InstructionHeader::Ping { id: target }, _) if target.as_byte() == BROADCAST_ID => {
                ((id as u32) * PING_STATUS_FRAME_BYTES, None)
            }
            (InstructionHeader::FastSyncRead { length, .. }, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, (n as u32) * (length as u32));
                let position = compute_fast_position(k, n, packet_length);
                let bytes_before = fast_bytes_before(k, length as u32);
                (bytes_before, Some(position))
            }
            (InstructionHeader::FastBulkRead { .. }, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, self.chain_data_bytes);
                let position = compute_fast_position(k, n, packet_length);
                (self.bytes_before, Some(position))
            }
            _ => (0, None),
        };
        let predecessor_id = match (self.header, self.slot) {
            (InstructionHeader::SyncRead { .. } | InstructionHeader::BulkRead { .. }, Some(k))
                if k > 0 =>
            {
                self.predecessor_id
            }
            _ => None,
        };
        ReplyContext {
            packet_end_tick,
            slot_offset_bytes,
            fast_slot_position,
            fold_start_cursor,
            predecessor_id,
        }
    }
}

/// Bump slot-walk cursor on a per-slot demarcation event and update
/// `slot` / `bytes_before` if applicable.
fn slot_walk(ctx: &mut InflightCtx, payload: &InstructionPayload, id: u8) {
    let (slot_id, slot_length) = match *payload {
        InstructionPayload::SyncSlot { id, .. } => (id, None),
        InstructionPayload::BulkSlot { id, length, .. } => (id, Some(length)),
        InstructionPayload::WriteDataChunk { .. } => return,
    };
    let k = ctx.next_slot_index;
    ctx.next_slot_index = ctx.next_slot_index.saturating_add(1);
    // Chain-total data byte accumulator runs for every BulkSlot in a Fast
    // Bulk Read — successor slots included — because First/Only's
    // packet_length is the sum across the whole chain.
    if let Some(length) = slot_length
        && let InstructionHeader::FastBulkRead { .. } = ctx.header
    {
        ctx.chain_data_bytes = ctx.chain_data_bytes.saturating_add(length as u32);
    }
    if slot_id.as_byte() == id && ctx.slot.is_none() {
        ctx.slot = Some(k);
        ctx.slot_length = slot_length;
        return;
    }
    if ctx.slot.is_some() {
        return;
    }
    // Predecessor slot — record the latest candidate (overwriting prior).
    // The chain-fire path for slots k > 0 only reads `predecessor_id` if
    // our own slot lands next; the standing value is always the immediate
    // predecessor when it's read (`docs/dxl-streaming-rx.md` §5.2).
    ctx.predecessor_id = Some(slot_id.as_byte());
    // Fast Bulk Read needs per-slot wire counts to size the chain CRC fold
    // (FastSlotInfo::bytes_before on Last). Plain Bulk Read takes the
    // chain-pending path on k > 0, so its predecessor sizes don't matter.
    if let Some(length) = slot_length
        && let InstructionHeader::FastBulkRead { .. } = ctx.header
    {
        let length = length as u32;
        let bytes = if k == 0 {
            fast_first_bytes(length)
        } else {
            fast_middle_bytes(length)
        };
        ctx.bytes_before += bytes;
    }
}

/// Wire bytes preceding slot `k` in a Fast Sync Read chain with uniform
/// per-slot register length. Slot 0 sits at offset 0 (it carries the chain
/// header). Slot k > 0 follows a First emission and `k-1` Middle emissions.
fn fast_bytes_before(k: u8, length: u32) -> u32 {
    if k == 0 {
        0
    } else {
        fast_first_bytes(length) + ((k as u32) - 1) * fast_middle_bytes(length)
    }
}

/// Wire length carried by the Fast chain Status header — bytes from `INST`
/// through trailing CRC inclusive. Encoder-input for both Only and First
/// `packet_length`. Per-slot wire shape is `err(1) + id(1) + data(L_k)`,
/// so the sum is `INST(1) + n·2 + Σ L_k + CRC(2)`.
fn fast_chain_packet_length(n_total: u8, chain_data_bytes: u32) -> u16 {
    (3 + 2 * (n_total as u32) + chain_data_bytes) as u16
}

/// Map a chain `(slot_index, total_slots, packet_length)` to the
/// [`SlotPosition`] the encoder consumes. `packet_length` on First/Only is
/// the chain's advertised wire length — emitted straight onto the wire by
/// [`dxl_protocol::encoder::SlotEncoder::emit`].
fn compute_fast_position(k: u8, n_total: u8, packet_length: u16) -> SlotPosition {
    if n_total == 1 {
        SlotPosition::Only { packet_length }
    } else if k == 0 {
        SlotPosition::First { packet_length }
    } else if k + 1 == n_total {
        SlotPosition::Last { crc: 0 }
    } else {
        SlotPosition::Middle
    }
}

/// A reply handle borrowed from disjoint pieces of [`DxlUart`] — the codec
/// TX half + scheduler + clock + the small set of pending-state fields. The
/// parent's closure-based [`DxlUart::poll`] hands the dispatcher one of
/// these alongside the parsed packet (which borrows the codec RX half), so
/// the dispatcher can call `send_status` / `send_slot` / `stage_*` /
/// `cancel` without a borrow conflict against the packet.
///
/// Implements [`osc_core::DxlReply`] — the chip-side `Ch32Bus::poll` forwards
/// the handle straight to the user closure as `&mut dyn DxlReply`. The
/// inherent methods stay so driver-crate tests (which don't import the trait)
/// can drive the handle directly.
pub struct ReplyHandle<'a, P: Providers, const TX_BUF_LEN: usize> {
    tx: &'a mut CodecTx<P::Crc, TX_BUF_LEN>,
    scheduler: &'a mut P::TxScheduler,
    fast_last: &'a mut FastLast<P::FastLastScheduler>,
    fast_last_crc: &'a mut FastLastCrc<P::Crc>,
    clock: &'a mut Clock<P::UsartBaud, P::ClockTrim>,
    last_reply_ctx: &'a mut Option<ReplyContext>,
    /// Chain-pending state on `DxlUart`. Set to `Some(pred)` by
    /// [`Self::send_status`] when the cached reply context names a Plain
    /// chain k > 0 predecessor; the matching `PollEvent::SkipComplete`
    /// arm consumes it and fires `TxBus::start_now`. Cleared by
    /// [`Self::cancel`] alongside the schedule cancel.
    predecessor_id: &'a mut Option<u8>,
    pending_id: &'a mut Option<u8>,
    pending_rdt_us: &'a mut Option<u32>,
    pending_reboot: &'a mut Option<BootMode>,
    /// Snapshot of the parent's `id` field at poll surface. Used by
    /// `stage_id` for the no-op-on-unchanged comparison; the actual
    /// `id` field on the parent isn't mutated until `on_tx_complete`,
    /// so this snapshot stays consistent for the lifetime of the
    /// reply handle.
    id: u8,
    /// Snapshot of the parent's `rdt_us` field — same reasoning as `id`.
    rdt_us: u32,
}

impl<P: Providers, const TX_BUF_LEN: usize> ReplyHandle<'_, P, TX_BUF_LEN> {
    /// Encode a Status reply into the codec's TX buffer and either schedule
    /// its wire start (single-target / Plain chain slot 0) or stage the
    /// chain-pending state (Plain chain slot k > 0; see
    /// `docs/dxl-streaming-rx.md` §5.2). For the scheduled path: fold RDT +
    /// slot offset (broadcast Ping / Sync / Bulk Read slot N) from the
    /// [`ReplyContext`] cached at `poll()`, convert the µs delay to
    /// scheduler ticks via `TxScheduler::TICKS_PER_US`, and hand the
    /// pre-computed `deadline_tick` to the provider. For the chain-pending
    /// path: record the predecessor's ID on the parent driver; the codec's
    /// matching `PollEvent::SkipComplete` invokes `TxBus::start_now`. If no
    /// context is staged (foreign Instruction filtered upstream) or the
    /// wire-end tick wasn't ready (and the path needs it), the encode still
    /// succeeds but no schedule is armed — the bytes simply won't ship.
    /// Context is taken on use so a double-send without a fresh `poll()`
    /// no-ops on the scheduler.
    pub fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        crate::log::trace!("dxl: send_status entry");
        self.tx.send_status(status)?;
        let Some(ctx) = self.last_reply_ctx.take() else {
            crate::log::debug!("dxl: send_status drop (no reply ctx)");
            return Ok(());
        };
        // Plain chain slot k > 0 — defer the wire send to the codec's
        // SkipComplete match on `predecessor_id` (`docs/dxl-streaming-rx.md`
        // §5.2). Anchor-independent: no `packet_end_tick` needed.
        if let Some(pred) = ctx.predecessor_id {
            crate::log::debug!("dxl: send_status defer to predecessor={}", pred);
            *self.predecessor_id = Some(pred);
            return Ok(());
        }
        let packet_end_tick = ctx.packet_end_tick;
        let byte_count = self.tx.tx_len();
        let delay_us = self.rdt_us + self.clock.bytes_to_us(ctx.slot_offset_bytes);
        let delay_ticks =
            delay_us.wrapping_mul(<P::TxScheduler as TxScheduler>::TICKS_PER_US as u32);
        let deadline_tick = packet_end_tick.wrapping_add(delay_ticks as u16);
        crate::log::debug!(
            "dxl: send_status schedule packet_end={} deadline={} byte_count={}",
            packet_end_tick,
            deadline_tick,
            byte_count
        );
        self.scheduler
            .schedule(deadline_tick, byte_count, SendKind::Plain);
        Ok(())
    }

    /// Encode one Fast Sync/Bulk Read slot reply and schedule its wire
    /// start. Slot position (Only/First/Middle/Last) comes from the cached
    /// [`ReplyContext`]; Last tags the schedule with [`SendKind::FastLast`]
    /// so the provider knows to coordinate with chain-CRC catchup (M6,
    /// `#6`). Q8.8 µs precision flows through to the tick math so the
    /// inter-slot gap (~3.33 µs at 3 Mbaud) is sub-µs-aligned.
    pub fn send_slot(&mut self, slot: &Slot<'_>) -> Result<(), WriteError> {
        let Some(ctx) = self.last_reply_ctx.take() else {
            return Ok(());
        };
        let Some(position) = ctx.fast_slot_position else {
            // No Fast slot in the cached context — caller routed wrong;
            // drop silently (dispatcher bug, not driver concern).
            return Ok(());
        };
        self.tx.send_slot(slot, position)?;
        let byte_count = self.tx.tx_len();
        let packet_end_tick = ctx.packet_end_tick;
        let (delay_ticks, kind) = match position {
            SlotPosition::Last { .. } => {
                // Q8.8 µs × ticks/µs = Q8.8 ticks; >> 8 lands on integer ticks.
                let delay_q88 =
                    (self.rdt_us << 8) + self.clock.bytes_to_us_q88(ctx.slot_offset_bytes);
                let ticks = (delay_q88
                    .wrapping_mul(<P::TxScheduler as TxScheduler>::TICKS_PER_US as u32))
                    >> 8;
                (ticks, SendKind::FastLast)
            }
            _ => {
                let delay_us = self.rdt_us + self.clock.bytes_to_us(ctx.slot_offset_bytes);
                let ticks =
                    delay_us.wrapping_mul(<P::TxScheduler as TxScheduler>::TICKS_PER_US as u32);
                (ticks, SendKind::Plain)
            }
        };
        let deadline_tick = packet_end_tick.wrapping_add(delay_ticks as u16);
        self.scheduler.schedule(deadline_tick, byte_count, kind);
        // Fast Last: arm the periodic-walk grid + chain-CRC fold engine.
        // `slot_offset_bytes` on Last == `bytes_before` == predecessor wire
        // bytes (excluding our own reply); the fold absorbs that count
        // before patching our trailing placeholder CRC slot.
        if matches!(position, SlotPosition::Last { .. }) {
            let byte_ticks = self.clock.ticks_per_bit().wrapping_mul(BITS_PER_FRAME);
            // RDT in scheduler ticks (FastLast trait surfaces u32 offsets,
            // but FastLastSchedule's fields stay u16-shaped per doc §10.6
            // — typical RDT ≤ 24k ticks, well under u16).
            let rdt_ticks = (self
                .rdt_us
                .wrapping_mul(<P::TxScheduler as TxScheduler>::TICKS_PER_US as u32)
                & 0xFFFF) as u16;
            self.fast_last.start(FastLastSchedule {
                packet_end_tick,
                rdt_ticks,
                byte_ticks,
                predecessor_bytes: ctx.slot_offset_bytes as u16,
            });
            self.fast_last_crc
                .start(ctx.fold_start_cursor, ctx.slot_offset_bytes);
        }
        Ok(())
    }

    /// Drop any scheduled TX, clear chain-pending state, and clear the
    /// staged request context so the next reply must come through a fresh
    /// `poll()`.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        *self.last_reply_ctx = None;
        *self.predecessor_id = None;
    }

    /// Stage a deferred ID change — applies at the next `on_tx_complete`.
    pub fn stage_id(&mut self, id: u8) {
        if id != self.id {
            *self.pending_id = Some(id);
        }
    }

    /// Stage a deferred baud-rate change. Forwarded to `Clock`; applied
    /// at its `on_tx_complete` (driven by the parent's `on_tx_complete`).
    pub fn stage_baud(&mut self, baud: BaudRate) {
        self.clock.stage_baud(baud);
    }

    /// Stage a deferred Return Delay Time change in µs — applies at the
    /// next `on_tx_complete`.
    pub fn stage_rdt(&mut self, us: u32) {
        if us != self.rdt_us {
            *self.pending_rdt_us = Some(us);
        }
    }

    /// Stage a deferred reboot, honored after any in-flight TX drains.
    pub fn stage_reboot(&mut self, mode: BootMode) {
        *self.pending_reboot = Some(mode);
    }
}

impl<P: Providers, const TX_BUF_LEN: usize> DxlReply for ReplyHandle<'_, P, TX_BUF_LEN> {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        ReplyHandle::send_status(self, status)
    }

    fn send_slot(&mut self, slot: &Slot<'_>) -> Result<(), WriteError> {
        ReplyHandle::send_slot(self, slot)
    }

    fn stage_id(&mut self, id: u8) {
        ReplyHandle::stage_id(self, id)
    }

    fn stage_baud(&mut self, baud: BaudRate) {
        ReplyHandle::stage_baud(self, baud)
    }

    fn stage_rdt(&mut self, us: u32) {
        ReplyHandle::stage_rdt(self, us)
    }

    fn stage_reboot(&mut self, mode: BootMode) {
        ReplyHandle::stage_reboot(self, mode)
    }
}

/// The DXL bus composite. `P` bundles the chip-side leaf interfaces this
/// driver pulls — see [`Providers`]. The const generics are storage sizes:
///
/// - `RX_BUF_LEN`: DMA1_CH5 byte-ring depth (typically 64 per doc §8.1).
/// - `EDGE_BUF_LEN`: DMA1_CH7 edge-timestamp ring depth (typically 128 /
///   option A in doc §8.4; 64 / option B trades CPU for memory).
/// - `TX_BUF_LEN`: DMA1_CH4 source-buffer depth sized to
///   `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES` (140 with the
///   default control-RW). Held by `Codec` so encoder methods write into
///   driver-owned storage instead of a chip-side static.
pub struct DxlUart<
    P: Providers,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> {
    codec: Codec<P::EdgeDma, P::Crc, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
    clock: Clock<P::UsartBaud, P::ClockTrim>,
    /// NDTR-only readback for DMA1_CH5 (the RX byte ring). Plumbed
    /// independently of the parser-path `on_rx_dma_advance` calls so the
    /// Fast Last fold body's intra-loop refresh doesn't go through the
    /// chip-side ISR — see [`Self::on_fold_step`] / [`Self::on_tx_start`].
    rx_dma: P::RxDma,
    scheduler: P::TxScheduler,
    /// Chip-side bus-control provider. Used by [`Self::on_tx_start`] /
    /// [`Self::on_tx_complete`] for the scheduled wire-driver lifecycle,
    /// and by [`Self::poll`]'s SkipComplete arm for the Plain chain
    /// k > 0 sequence-driven fire path (`docs/dxl-streaming-rx.md` §5.2).
    tx_bus: P::TxBus,
    /// Periodic-walk grid scheduler for Fast Sync / Bulk Read Last replies.
    /// Driver-pattern §4 sub-driver: armed at `send_slot(Last)` via
    /// `ReplyHandle`; `on_systick_match` drives one body per CMP.
    fast_last: FastLast<P::FastLastScheduler>,
    /// Chain-CRC fold engine. Per-byte hook is wired into the codec's
    /// `drain_raw` callback during the SysTick walker; finalize patches our
    /// own trailing CRC slot before DMA1_CH4 reads it.
    fast_last_crc: FastLastCrc<P::Crc>,
    /// 16-bit wire-clock readout used by `on_rx_edge_advance`, `on_rx_idle`,
    /// and `poll` to source `now` without taking it as a parameter. Same
    /// physical counter the chip-side ISR layer stamps into the edge ring;
    /// the provider exists so the driver stays unaware of which peripheral
    /// the chip picked.
    wire_clock: P::WireClock,

    id: u8,
    rdt_us: u32,

    /// Wire-positioning info derived from the most recently surfaced
    /// packet — wire-end tick, slot offset, Fast slot position, chain-CRC
    /// anchor. Computed at poll surface and consumed by `ReplyHandle`'s
    /// send methods. Per driver-pattern §7.4 — driver-derivable wire shape
    /// stays in the driver, not on the trait surface.
    last_reply_ctx: Option<ReplyContext>,

    /// Slot-walk + addressed-instruction state while the parser is inside
    /// an Instruction the chip will reply to. Cleared at Crc / Resync /
    /// foreign Header.
    inflight: Option<InflightCtx>,

    /// Predecessor ID the chip is awaiting before sending its own Plain
    /// chain k > 0 reply. `Some(id)` set by [`ReplyHandle::send_status`]
    /// when the cached [`ReplyContext`] names a chain k > 0 predecessor;
    /// the codec's matching `PollEvent::SkipComplete { id }` arm consumes
    /// it and fires `TxBus::start_now`. Cleared on
    /// [`ReplyHandle::cancel`], on Resync, and on [`Self::on_tx_complete`]
    /// (belt-and-suspenders — the SkipComplete path clears it on success).
    /// A silent predecessor — SkipComplete never matches — leaves it
    /// `Some` until the next chain or reset; harmless because the next
    /// `send_status` overwrites it. Per `docs/dxl-streaming-rx.md` §5.2.
    predecessor_id: Option<u8>,

    pending_id: Option<u8>,
    pending_rdt_us: Option<u32>,
    pending_reboot: Option<BootMode>,
}

impl<P: Providers, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize, const TX_BUF_LEN: usize>
    DxlUart<P, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        codec: Codec<P::EdgeDma, P::Crc, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
        clock: Clock<P::UsartBaud, P::ClockTrim>,
        rx_dma: P::RxDma,
        scheduler: P::TxScheduler,
        tx_bus: P::TxBus,
        fast_last: FastLast<P::FastLastScheduler>,
        wire_clock: P::WireClock,
        id: u8,
        rdt_us: u32,
    ) -> Self {
        Self {
            codec,
            clock,
            rx_dma,
            scheduler,
            tx_bus,
            fast_last,
            fast_last_crc: FastLastCrc::new(),
            wire_clock,
            id,
            rdt_us,
            last_reply_ctx: None,
            inflight: None,
            predecessor_id: None,
            pending_id: None,
            pending_rdt_us: None,
            pending_reboot: None,
        }
    }

    /// New RX falling-edge timestamps may be available — pull the current
    /// `ticks_per_bit` from the clock and walk the classifier. Drift pairs
    /// emit while the classifier's `hsi_active` flag is set (instructions in
    /// flight); they route into the clock's drift integrator. `now` is
    /// self-sourced from the [`WireClock`] provider and stashed on the codec
    /// RX half so the parser's Crc handler can fall back to a tick estimate
    /// when the classifier was unanchored.
    pub fn on_rx_edge_advance(&mut self) {
        let now = self.wire_clock.now();
        let ticks_per_bit = self.clock.ticks_per_bit();
        let clock = &mut self.clock;
        self.codec
            .on_edge_advance(now, ticks_per_bit, |prev, curr| {
                clock.on_byte_pair(prev, curr)
            });
    }

    /// RX wire went idle — drain tail edges (drift-routed when active).
    /// `now` is self-sourced from the [`WireClock`] provider — see
    /// [`Self::on_rx_edge_advance`].
    pub fn on_rx_idle(&mut self) {
        let now = self.wire_clock.now();
        let ticks_per_bit = self.clock.ticks_per_bit();
        let clock = &mut self.clock;
        self.codec.on_idle(now, ticks_per_bit, |prev, curr| {
            clock.on_byte_pair(prev, curr)
        });
    }

    /// USART1 RX DMA published progress — `remaining` is the channel's
    /// NDTR readback.
    pub fn on_rx_dma_advance(&mut self, remaining: u16) {
        self.codec.on_rx_dma_advance(remaining);
    }

    /// Drive the codec event stream, manage wire-level state (anchor,
    /// drift gating, universal byte-skip, slot-walk, reply context) and
    /// forward every parser [`Event`] to the dispatcher closure alongside
    /// the ring slice and a [`ReplyHandle`]. The closure shape matches
    /// `osc_core::DxlDispatcher::on_event` so the chip-side
    /// `Ch32Bus::poll` forwarder is a one-liner.
    ///
    /// Per [doc §3 / §4.3]:
    /// - Instruction Header → `try_anchor_from_header`, `set_hsi_active(true)`.
    /// - Instruction Header (foreign) → universal byte-skip on the body.
    /// - Status Header → universal byte-skip on the body.
    /// - SyncSlot / BulkSlot → slot-walk against `self.id`.
    /// - Crc → stamp packet-end tick, derive [`ReplyContext`],
    ///   `reset_anchor` (deterministic packet boundary).
    /// - Resync → `reset_anchor`, drop inflight.
    ///
    /// Closure-based shape exists to break the borrow conflict between the
    /// parser-event borrow on the codec RX half and the `&mut` access the
    /// send path needs on the TX half. [`Codec::split_mut`] returns the two
    /// halves disjointly so the closure sees both at once.
    pub fn poll<F>(&mut self, mut f: F)
    where
        F: FnMut(Event, &[u8], &mut ReplyHandle<'_, P, TX_BUF_LEN>),
    {
        self.codec.on_rx_dma_advance(self.rx_dma.remaining());
        let now = self.wire_clock.now();
        let id = self.id;
        let rdt_us = self.rdt_us;
        let ticks_per_bit = self.clock.ticks_per_bit();
        let Self {
            codec,
            clock,
            scheduler,
            tx_bus,
            fast_last,
            fast_last_crc,
            rx_dma,
            last_reply_ctx,
            inflight,
            predecessor_id,
            pending_id,
            pending_rdt_us,
            pending_reboot,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        rx.poll(now, ticks_per_bit, |pe, rx_inner| match pe {
            PollEvent::Event {
                ev,
                ring,
                next_status_pos,
            } => {
                let action = match ev {
                    Event::Header(HeaderEvent::Instruction(h)) => {
                        let target = header_target(&h).as_byte();
                        let addressable = target_addressable(&h, id);
                        crate::log::trace!(
                            "dxl[id={}]: event=header_instruction target={} addressable={}",
                            id,
                            target,
                            addressable
                        );
                        // Implicit chain-state reset per `dxl-streaming-rx.md`
                        // §5.3 — any stale `predecessor_id` from a prior chain
                        // whose predecessor went silent must clear here, or a
                        // foreign Status whose id happens to match would
                        // trigger a spurious `start_now` on the new chain.
                        *predecessor_id = None;
                        rx_inner.try_anchor_from_header(ticks_per_bit);
                        rx_inner.set_hsi_active(true);
                        if addressable {
                            *inflight = Some(InflightCtx::new(h));
                            PollAction::Continue
                        } else {
                            *inflight = None;
                            PollAction::Skip { id: target }
                        }
                    }
                    Event::Header(HeaderEvent::Status(sh)) => {
                        crate::log::trace!(
                            "dxl[id={}]: event=header_status status_id={}",
                            id,
                            sh.id.as_byte()
                        );
                        *inflight = None;
                        PollAction::Skip {
                            id: sh.id.as_byte(),
                        }
                    }
                    Event::Payload(PayloadEvent::Instruction(p)) => {
                        crate::log::trace!("dxl[id={}]: event=payload_instruction", id);
                        if let Some(ctx) = inflight.as_mut() {
                            slot_walk(ctx, &p, id);
                        }
                        PollAction::Continue
                    }
                    Event::Payload(PayloadEvent::Status(_)) => {
                        debug_assert!(false, "Status payload should have been byte-skipped");
                        PollAction::Continue
                    }
                    Event::Crc => {
                        crate::log::trace!("dxl[id={}]: event=crc", id);
                        if let Some(ctx) = inflight.take() {
                            // Bring `last_byte_start` current before stamping
                            // packet_end. The walker only drains at HT/TC or
                            // IDLE boundaries; for a short packet bounded by a
                            // single such event the parser fires Header → Crc
                            // within one poll, leaving the anchor placed by
                            // `try_anchor_from_header` but never forward-walked
                            // across bytes past the signature.
                            rx_inner.drain_walker(ticks_per_bit, |prev, curr| {
                                clock.on_byte_pair(prev, curr)
                            });
                            // Anchored path is the common case; fallback fires
                            // when interference / edge loss starved the
                            // classifier. FAST chain ops skip the fallback
                            // — see `InflightCtx::allows_packet_end_fallback`.
                            let packet_end_tick = match rx_inner.packet_end_tick(ticks_per_bit) {
                                Some(t) => Some(t),
                                None => {
                                    rx_dma.record_edge_anchor_miss();
                                    ctx.allows_packet_end_fallback().then(|| {
                                        let (now, src) = rx_inner.last_isr_capture();
                                        rx_inner.packet_end_tick_fallback(src, now, ticks_per_bit)
                                    })
                                }
                            };
                            if let Some(t) = packet_end_tick {
                                crate::log::debug!("dxl[id={}]: crc packet_end_tick={}", id, t);
                                // At Crc-of-host-instruction, the codec's
                                // wire position has just walked past the
                                // request's last CRC byte — the next byte on
                                // the wire is the First predecessor's leading
                                // `0xFF`. So `next_status_pos` is exactly the
                                // fold-start cursor for the Fast Last CRC
                                // engine.
                                *last_reply_ctx =
                                    Some(ctx.into_reply_context(id, t, next_status_pos));
                            } else {
                                crate::log::debug!(
                                    "dxl: crc anchor missing and fallback disallowed — drop reply"
                                );
                            }
                        }
                        // Read packet_end_tick first, then invalidate —
                        // Crc is the deterministic packet boundary.
                        rx_inner.reset_anchor();
                        PollAction::Continue
                    }
                    Event::Resync(_) => {
                        crate::log::trace!("dxl[id={}]: event=resync", id);
                        rx_inner.reset_anchor();
                        *inflight = None;
                        *predecessor_id = None;
                        PollAction::Continue
                    }
                    Event::Sync => PollAction::Continue,
                };
                let mut reply = ReplyHandle {
                    tx,
                    scheduler,
                    fast_last,
                    fast_last_crc,
                    clock,
                    last_reply_ctx,
                    predecessor_id,
                    pending_id,
                    pending_rdt_us,
                    pending_reboot,
                    id,
                    rdt_us,
                };
                f(ev, ring, &mut reply);
                action
            }
            PollEvent::SkipComplete { id: pred } => {
                crate::log::trace!(
                    "dxl[id={}]: skip_complete pred={} predecessor_id={:?}",
                    id,
                    pred,
                    *predecessor_id
                );
                if *predecessor_id == Some(pred) {
                    crate::log::debug!(
                        "dxl[id={}]: skip_complete match pred={} -> start_now byte_count={}",
                        id,
                        pred,
                        tx.tx_len()
                    );
                    tx_bus.start_now(tx.tx_len());
                    *predecessor_id = None;
                }
                PollAction::Continue
            }
        });
    }

    /// Monotonic count of Instruction packets seen on the wire — own and
    /// foreign IDs included; Status frames excluded. Drift estimator's
    /// tick source.
    pub fn instruction_count(&self) -> u32 {
        self.codec.instruction_count()
    }

    /// The TX-start tick has arrived (chip-side CC3 IRQ). Activates the
    /// wire driver FIRST so the first wire bit lands on `fire_deadline`;
    /// for Fast Last replies the body then tails with a post-fire residue
    /// fold that absorbs any GUARD bytes still in-flight at fire time and
    /// patches the trailing CRC slot before DMA1_CH4's prefetch reads it
    /// (doc §10.6.2 CC3 body).
    ///
    /// The post-fire fold has three exits:
    /// - **finalize** — `fast_last_crc` reaches its predecessor-byte target
    ///   inside `on_byte`, which patches `tx_buf[len-CRC_BYTES..len]` and
    ///   clears `active`. Success; no telemetry event.
    /// - **patch-window-expired** — [`FastLast::patch_window_expired`]
    ///   reports the TX DMA channel has prefetched into the trailing CRC
    ///   slot. Any further patch ships too late; bump
    ///   `crc_patch_deadline_miss`.
    /// - **plateau** — no new RX bytes between iterations; predecessor
    ///   starvation backstop (`[[busy-wait-plateau-backstop]]`). Same
    ///   observable failure as expired-window (placeholder CRC ships); bumps
    ///   the same counter.
    ///
    /// Each [`CodecRx::drain_raw`] pass refreshes the byte-ring producer
    /// head from [`RxDma::remaining`] so newly-arrived GUARD bytes become
    /// visible inside the spin (per `dxl-streaming-rx.md` §6, the
    /// `crc_patch_deadline_miss` counter is a bench-defended floor signal at
    /// the 3 Mbaud floor, not a wire-correctness failure).
    pub fn on_tx_start(&mut self) {
        self.tx_bus.handle_start();
        if !self.fast_last_crc.is_active() {
            return;
        }
        let Self {
            codec,
            rx_dma,
            fast_last,
            fast_last_crc,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        loop {
            if fast_last.patch_window_expired() {
                fast_last.record_patch_deadline_miss();
                break;
            }
            let before = fast_last_crc.bytes_folded();
            rx.drain_raw(rx_dma, |byte, cursor| {
                fast_last_crc.on_byte(byte, cursor, tx);
            });
            if !fast_last_crc.is_active() {
                break;
            }
            if fast_last_crc.bytes_folded() == before {
                fast_last.record_patch_deadline_miss();
                break;
            }
        }
    }

    /// One Fast Last periodic-walk fold body is due (chip-side SysTick
    /// CMP). First entry masks DMA1_CH7 HT/TC IRQ via [`Codec::pause_edges`]
    /// so the busy-wait inside the FSM's final-anchor body is the sole
    /// PFIC HIGH consumer (doc §10.6.3); edge DMA itself keeps capturing
    /// into ET. Body drives [`FastLast::on_step`] forward — the walker
    /// closure drains pending RX bytes raw through [`FastLastCrc::on_byte`]
    /// and returns the cumulative folded count for the FSM's `target =
    /// predecessor_bytes − GUARD` busy-wait exit (`fast_last/mod.rs`).
    pub fn on_fold_step(&mut self) {
        self.codec.pause_edges();
        let Self {
            codec,
            rx_dma,
            fast_last,
            fast_last_crc,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        fast_last.on_step(|| {
            rx.drain_raw(rx_dma, |byte, cursor| {
                fast_last_crc.on_byte(byte, cursor, tx);
            });
            fast_last_crc.bytes_folded()
        });
    }

    /// USART1 TC fired — the reply has fully drained the wire. Release
    /// the wire driver *first* (drop TX_EN, mask TC IRQ, disable DMA) so
    /// stale TX_EN doesn't sit on the bus while pending config mutates;
    /// then drain staged writes (id / baud / trim / rdt) and surface any
    /// pending reboot to the chip-side ISR. Also disarms the Fast Last
    /// state (idempotent — both already disarmed naturally on the
    /// successful path) and re-enables DMA1_CH7 HT/TC (no-op for Plain
    /// replies that never paused). Reboot is returned (rather than
    /// self-applied) because the chip controls how the reset actually
    /// happens; the driver only knows it was asked.
    pub fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.tx_bus.handle_tx_complete();
        self.fast_last.cancel();
        self.fast_last_crc.cancel();
        self.codec.resume_edges();
        // Belt-and-suspenders: the SkipComplete handler clears this on
        // success, but a silent predecessor would leave it `Some` across
        // the next reply. Clearing here keeps the chain-pending state
        // bounded by the in-flight reply.
        self.predecessor_id = None;
        if let Some(id) = self.pending_id.take() {
            self.id = id;
        }
        if let Some(rdt) = self.pending_rdt_us.take() {
            self.rdt_us = rdt;
        }
        self.clock.on_tx_complete();
        self.pending_reboot.take()
    }

    /// Stable peripheral-memory address for DMA1_CH7's destination buffer.
    pub fn edges_addr(&self) -> usize {
        self.codec.edges_addr()
    }

    /// Stable peripheral-memory address for DMA1_CH5's destination buffer.
    pub fn rx_buf_addr(&self) -> usize {
        self.codec.rx_buf_addr()
    }

    /// Stable peripheral-memory address for DMA1_CH4's source buffer.
    pub fn tx_buf_addr(&self) -> usize {
        self.codec.tx_buf_addr()
    }

    /// Length in bytes of the most-recent encoded packet — the DMA1_CH4
    /// transfer count for the next fire.
    pub fn tx_len(&self) -> u16 {
        self.codec.tx_len()
    }
}

// Shelved pending U4 (osc-drivers unit test audit): tests below bind to
// hand-rolled mock fields; will be migrated to the mockall + state-companion
// API as part of the audit.
#[cfg(any())]
#[cfg(test)]
mod tests {
    extern crate alloc;
    use super::*;
    use crate::mocks::{
        FastLastSchedulerOp, MockClockTrim, MockEdgeDma, MockFastLastScheduler, MockRxDma,
        MockTxBus, MockTxScheduler, MockUsartBaud, ScheduleOp, TestProviders, TxBusOp,
    };
    use dxl_protocol::types::StatusError;
    use dxl_protocol::{InstructionEncoder, SoftwareCrcUmts, StatusEncoder};
    use heapless::Vec;
    use osc_core::BaudRate;

    /// Test-side storage sizing — matches V006 defaults per doc §§8.1, 8.3,
    /// 8.4 so any drift between driver tests and chip-side reality stays
    /// visible.
    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;
    const TX_BUF_LEN: usize = 140;

    const TEST_ID: u8 = 0x07;
    const TEST_RDT_US: u32 = 250;

    type TestCodec = Codec<MockEdgeDma, SoftwareCrcUmts, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;
    type TestBus = DxlUart<TestProviders, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

    fn make_clock(baud: BaudRate) -> Clock<MockUsartBaud, MockClockTrim> {
        Clock::new(baud, MockUsartBaud::default(), MockClockTrim::default())
    }

    fn make_bus_with(codec: TestCodec, baud: BaudRate) -> TestBus {
        DxlUart::new(
            codec,
            make_clock(baud),
            MockRxDma::default(),
            MockTxScheduler::default(),
            MockTxBus::default(),
            FastLast::new(MockFastLastScheduler::default()),
            TEST_ID,
            TEST_RDT_US,
        )
    }

    fn make_bus() -> TestBus {
        make_bus_with(Codec::new(MockEdgeDma::default()), BaudRate::B3000000)
    }

    /// Pre-seed classifier `last_byte_start` so `packet_end_tick` reads a
    /// known tick at Crc-good without staging real edges. Composite tests
    /// for the TX scheduling path use this — the classifier-leaf tests cover
    /// the per-edge walker; the composite asserts the wiring.
    const SEED_TICK: u16 = 1000;
    fn force_anchor(bus: &mut TestBus) {
        bus.codec.force_byte_tick_for_test(SEED_TICK);
    }

    /// Stage `bytes` into the codec's RX byte ring at sequence `at` AND
    /// publish the matching DMA1_CH5 NDTR readback through `MockRxDma` so
    /// the next `bus.poll()` entry sees `on_publish(remaining)` as a no-op
    /// delta. Mirrors the chip-side wiring where the byte ring's producer
    /// head only advances via NDTR readback inside `poll()`.
    fn stage_rx(bus: &mut TestBus, at: u16, bytes: &[u8]) {
        bus.codec.stage_rx_bytes_for_test(at, bytes);
        let n = RX_BUF_LEN as u16;
        let head_pos = at.wrapping_add(bytes.len() as u16) % n;
        bus.rx_dma.remaining.set((n - head_pos) % n);
    }

    /// Run one poll over `bytes`. Returns the captured event tags so tests
    /// can match on the parser stream the dispatcher closure receives.
    #[derive(Debug, PartialEq, Eq)]
    enum Tag {
        Sync,
        InstrPing(u8),
        InstrSyncRead,
        InstrFastSyncRead,
        StatusHeader(u8),
        Crc,
        Resync,
        Other,
    }

    fn ev_tag(ev: Event) -> Tag {
        match ev {
            Event::Sync => Tag::Sync,
            Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { id })) => {
                Tag::InstrPing(id.as_byte())
            }
            Event::Header(HeaderEvent::Instruction(InstructionHeader::SyncRead { .. })) => {
                Tag::InstrSyncRead
            }
            Event::Header(HeaderEvent::Instruction(InstructionHeader::FastSyncRead { .. })) => {
                Tag::InstrFastSyncRead
            }
            Event::Header(HeaderEvent::Status(sh)) => Tag::StatusHeader(sh.id.as_byte()),
            Event::Crc => Tag::Crc,
            Event::Resync(_) => Tag::Resync,
            _ => Tag::Other,
        }
    }

    fn poll_capture(bus: &mut TestBus, bytes: &[u8]) -> alloc::vec::Vec<Tag> {
        stage_rx(bus, 0, bytes);
        let mut tags = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| {
            tags.push(ev_tag(ev));
        });
        tags
    }

    /// True if the stream surfaced a Crc — proxy for "an addressed
    /// instruction completed and the dispatcher would commit."
    fn saw_crc(tags: &[Tag]) -> bool {
        tags.iter().any(|t| matches!(t, Tag::Crc))
    }

    #[test]
    fn on_rx_edge_advance_does_not_panic_with_empty_buffer() {
        let mut bus = make_bus();
        bus.on_rx_edge_advance();
    }

    #[test]
    fn on_rx_idle_does_not_panic_with_empty_buffer() {
        let mut bus = make_bus();
        bus.on_rx_idle();
    }

    #[test]
    fn stage_id_defers_until_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_id(0x42));
        assert_eq!(bus.id, TEST_ID);
        let reboot = bus.on_tx_complete();
        assert_eq!(bus.id, 0x42);
        assert!(reboot.is_none());
    }

    #[test]
    fn stage_id_noop_when_unchanged() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_id(TEST_ID));
        assert!(bus.pending_id.is_none());
    }

    #[test]
    fn stage_rdt_defers_until_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_rdt(500));
        assert_eq!(bus.rdt_us, TEST_RDT_US);
        bus.on_tx_complete();
        assert_eq!(bus.rdt_us, 500);
    }

    #[test]
    fn stage_baud_forwards_to_clock() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_baud(BaudRate::B1000000));
        // Still at 3M until the apply.
        assert_eq!(bus.clock.ticks_per_bit(), 16);
        bus.on_tx_complete();
        assert_eq!(bus.clock.ticks_per_bit(), 48);
    }

    #[test]
    fn stage_reboot_surfaces_through_on_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_reboot(BootMode::Bootloader));
        assert!(bus.pending_reboot.is_some());
        let mode = bus.on_tx_complete();
        assert_eq!(mode, Some(BootMode::Bootloader));
        // Consume-on-take: next TC is clean.
        assert!(bus.on_tx_complete().is_none());
    }

    #[test]
    fn rx_buf_addr_is_stable() {
        let bus = make_bus();
        let a = bus.rx_buf_addr();
        let b = bus.rx_buf_addr();
        assert_eq!(a, b);
        assert_ne!(a, 0);
    }

    /// Emit a Ping addressed to `id`.
    fn wire_ping(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .ping(Id::new(id))
            .unwrap();
        out
    }

    /// Emit a Status reply from `id` (the kind we never want to surface).
    fn wire_status(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        StatusEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .empty(Id::new(id), StatusError::OK)
            .unwrap();
        out
    }

    /// Sync Read targets BROADCAST on the wire — exercises the
    /// id-as-BROADCAST branch with a richer (non-Ping) packet shape.
    fn wire_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .sync_read(addr, length, ids)
            .unwrap();
        out
    }

    /// Fast Sync Read — same broadcast addressing as Sync Read but the
    /// reply path goes through `send_slot` per the FastSlotInfo position.
    fn wire_fast_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8, 64> {
        let mut out: Vec<u8, 64> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .fast_sync_read(addr, length, ids)
            .unwrap();
        out
    }

    #[test]
    fn poll_returns_no_events_when_no_new_bytes() {
        let mut bus = make_bus();
        let mut count = 0;
        bus.poll(|_, _, _| count += 1);
        assert_eq!(count, 0);
        // Idempotent — calling again still drains nothing.
        bus.poll(|_, _, _| count += 1);
        assert_eq!(count, 0);
    }

    #[test]
    fn poll_streams_instruction_addressed_to_us_through_crc() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        let tags = poll_capture(&mut bus, &pkt);
        assert!(tags.contains(&Tag::InstrPing(TEST_ID)));
        assert!(saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_byte_skips_foreign_instruction_body() {
        let mut bus = make_bus();
        let pkt = wire_ping(0x42);
        let tags = poll_capture(&mut bus, &pkt);
        // Foreign Header surfaces; its Crc does NOT (universal byte-skip
        // consumed the body before the parser reaches CRC).
        assert!(tags.contains(&Tag::InstrPing(0x42)));
        assert!(!saw_crc(&tags));
        // Foreign Instructions still tick the instruction count.
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_surfaces_broadcast_instruction() {
        let mut bus = make_bus();
        let pkt = wire_sync_read(0x84, 4, &[0x01, 0x02, 0x03]);
        let tags = poll_capture(&mut bus, &pkt);
        assert!(tags.contains(&Tag::InstrSyncRead));
        assert!(saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_byte_skips_status_frames() {
        let mut bus = make_bus();
        let pkt = wire_status(TEST_ID);
        let tags = poll_capture(&mut bus, &pkt);
        assert!(tags.contains(&Tag::StatusHeader(TEST_ID)));
        assert!(!saw_crc(&tags));
        // Status frames never tick the instruction count.
        assert_eq!(bus.instruction_count(), 0);
    }

    #[test]
    fn poll_recovers_from_bad_crc() {
        let mut bus = make_bus();
        let mut bad = wire_ping(TEST_ID);
        let crc_lo_pos = bad.len() - 2;
        bad[crc_lo_pos] ^= 0xFF;
        let good = wire_ping(TEST_ID);

        let mut combined: Vec<u8, 32> = Vec::new();
        combined.extend_from_slice(&bad).unwrap();
        combined.extend_from_slice(&good).unwrap();

        let tags = poll_capture(&mut bus, &combined);
        assert!(tags.iter().any(|t| matches!(t, Tag::Resync)));
        // The good frame's Crc surfaces after recovery.
        assert!(saw_crc(&tags));
        // instruction_count ticks on every emitted Instruction Header
        // regardless of subsequent Crc verdict — both Pings show up.
        assert_eq!(bus.instruction_count(), 2);
    }

    #[test]
    fn poll_handles_ring_wrap() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        let start = (RX_BUF_LEN as u16).wrapping_sub(4);
        bus.codec.set_rx_read_seq_for_test(start);
        stage_rx(&mut bus, start, &pkt);

        let mut tags = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| tags.push(ev_tag(ev)));
        assert!(saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_partial_packet_resumes_on_next_call() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);

        let split = pkt.len() - 1;
        stage_rx(&mut bus, 0, &pkt[..split]);
        let mut tags = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| tags.push(ev_tag(ev)));
        assert!(!saw_crc(&tags));
        // Header ticked instruction_count even on the partial — the parser
        // surfaces Header before it has the trailing CRC.
        assert_eq!(bus.instruction_count(), 1);

        stage_rx(&mut bus, split as u16, &pkt[split..]);
        let mut tags2 = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| tags2.push(ev_tag(ev)));
        assert!(saw_crc(&tags2));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_sets_hsi_active_at_instruction_header_and_clears_at_crc() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        // Snapshot the classifier flag mid-stream — at the Crc event the
        // composite has already cleared `hsi_active`, so we observe it
        // through the Payload event instead. Ping has no payload so we
        // observe via post-state: hsi_active is False after the Crc.
        stage_rx(&mut bus, 0, &pkt);
        bus.poll(|_, _, _| {});
        // Post-Crc state.
        assert!(!bus.codec.rx_classifier_hsi_active_for_test());
    }

    #[test]
    fn poll_resync_resets_hsi_active() {
        let mut bus = make_bus();
        let mut bad = wire_ping(TEST_ID);
        let crc_lo = bad.len() - 2;
        bad[crc_lo] ^= 0xFF;
        stage_rx(&mut bus, 0, &bad);
        bus.poll(|_, _, _| {});
        assert!(!bus.codec.rx_classifier_hsi_active_for_test());
    }

    #[test]
    fn instruction_count_advances_on_every_instruction_regardless_of_id() {
        let mut bus = make_bus();
        let ours = wire_ping(TEST_ID);
        let theirs = wire_ping(0x42);
        let status = wire_status(0x42);

        let mut combined: Vec<u8, 64> = Vec::new();
        combined.extend_from_slice(&ours).unwrap();
        combined.extend_from_slice(&theirs).unwrap();
        combined.extend_from_slice(&status).unwrap();
        stage_rx(&mut bus, 0, &combined);

        bus.poll(|_, _, _| {});
        // Foreign Ping bumps the counter, Status doesn't.
        assert_eq!(bus.instruction_count(), 2);
    }

    // ------------------------------------------------------------------
    // TX scheduling
    // ------------------------------------------------------------------

    /// Construct a bus pre-loaded with `pkt` bytes and a forced classifier
    /// anchor at [`SEED_TICK`] so `packet_end_tick` reads
    /// `SEED_TICK + 10·tpb` at Crc-good. Returns the bus and the expected
    /// packet-end tick.
    fn bus_seeded_with(pkt: &[u8]) -> (TestBus, u16) {
        let mut bus = make_bus();
        stage_rx(&mut bus, 0, pkt);
        force_anchor(&mut bus);
        // tpb=16 @ 3M; packet_end = SEED_TICK + 10·tpb.
        let packet_end_tick = SEED_TICK.wrapping_add(16_u16.wrapping_mul(10));
        (bus, packet_end_tick)
    }

    fn empty_status() -> Status<'static> {
        Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        }
    }

    /// Packet-end tick + delay_us → expected deadline_tick at V006's
    /// TICKS_PER_US=48 (matching the mock's const).
    fn expected_deadline(packet_end_tick: u16, delay_us: u32) -> u16 {
        packet_end_tick.wrapping_add(delay_us.wrapping_mul(48) as u16)
    }

    #[test]
    fn send_status_after_poll_schedules_at_packet_end_plus_rdt() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, packet_end_tick) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::Schedule {
                deadline_tick: expected_deadline(packet_end_tick, TEST_RDT_US),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_status_drops_silently_for_foreign_instruction() {
        // Foreign-ID Instruction → no last_reply_ctx → send_status no-ops on
        // the scheduler even if the dispatcher fires it.
        let ping = wire_ping(0x42);
        let (mut bus, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));
        assert!(bus.scheduler.log.is_empty());
    }

    #[test]
    fn send_slot_only_schedules_plain() {
        // Fast Sync Read with our_id as the sole slot → SlotPosition::Only.
        let req = wire_fast_sync_read(0, 2, &[TEST_ID]);
        let (mut bus, packet_end_tick) = bus_seeded_with(&req);

        let payload = [0x11_u8, 0x22];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });

        // our_slot=0 of n_slots=1 → bytes_before=0 → delay = RDT, kind=Plain.
        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::Schedule {
                deadline_tick: expected_deadline(packet_end_tick, TEST_RDT_US),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_slot_last_schedules_fast_last() {
        // Fast Sync Read with two slaves where we're the last → Last position.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, packet_end_tick) = bus_seeded_with(&req);

        let payload = [0xAA_u8, 0xBB];
        // First slot of 2 emits 8 (header) + 2 (error+id) + 2 (data) = 12
        // wire bytes; bytes_before for slot 1 = 12.
        let expected_delay_q88 = (TEST_RDT_US << 8) + bus.clock.bytes_to_us_q88(12);
        let expected_ticks = (expected_delay_q88.wrapping_mul(48)) >> 8;
        let expected = packet_end_tick.wrapping_add(expected_ticks as u16);

        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::Schedule {
                deadline_tick: expected,
                byte_count,
                kind: SendKind::FastLast,
            }]
        );
        let fl_log = bus.fast_last.scheduler().log.as_slice();
        assert!(matches!(
            fl_log,
            [
                FastLastSchedulerOp::SetDeadline { .. },
                FastLastSchedulerOp::Schedule { .. },
            ]
        ));
        assert!(bus.fast_last_crc.is_active());
        assert!(bus.fast_last.is_active());
    }

    #[test]
    fn on_tx_complete_cancels_fast_last_state() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        assert!(bus.fast_last.is_active());
        assert!(bus.fast_last_crc.is_active());

        let _ = bus.on_tx_complete();

        assert!(!bus.fast_last.is_active());
        assert!(!bus.fast_last_crc.is_active());
        assert!(matches!(
            bus.fast_last.scheduler().log.last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn send_slot_without_fast_context_drops_silently() {
        // Polled a Ping → no Fast slot position cached. send_slot must
        // silently no-op (dispatcher bug, not driver concern).
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);

        let payload = [0x00_u8];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        assert!(bus.scheduler.log.is_empty());
    }

    #[test]
    fn cancel_clears_context_and_forwards() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        // Gate to Crc — the event-stream poll dispatches the closure on
        // every event; one Cancel per packet is what we want, not three.
        bus.poll(|ev, _, reply| {
            if matches!(ev, Event::Crc) {
                reply.cancel();
            }
        });
        assert_eq!(bus.scheduler.log.as_slice(), &[ScheduleOp::Cancel]);
        assert!(bus.last_reply_ctx.is_none());
    }

    #[test]
    fn send_status_consumes_ctx_so_double_send_is_silent() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        // Drive the per-packet send only at the Crc event so we don't fire
        // it twice via the multi-event stream.
        bus.poll(|ev, _, reply| {
            if matches!(ev, Event::Crc) {
                reply.send_status(empty_status()).expect("encode fits");
                // Second send within the same callback consumes nothing —
                // ctx was taken on the first; scheduler log shows one entry.
                reply.send_status(empty_status()).expect("encode fits");
            }
        });
        assert_eq!(bus.scheduler.log.len(), 1);
    }

    #[test]
    fn broadcast_ping_folds_id_indexed_slot_offset() {
        let req = wire_ping(BROADCAST_ID);
        let (mut bus, packet_end_tick) = bus_seeded_with(&req);

        let expected_offset_us = bus
            .clock
            .bytes_to_us(TEST_ID as u32 * PING_STATUS_FRAME_BYTES);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::Schedule {
                deadline_tick: expected_deadline(packet_end_tick, TEST_RDT_US + expected_offset_us),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn on_tx_start_routes_to_handle_start() {
        let mut bus = make_bus();
        bus.on_tx_start();
        assert_eq!(bus.tx_bus.log.as_slice(), &[TxBusOp::HandleStart]);
        assert!(bus.scheduler.log.is_empty());
    }

    #[test]
    fn on_tx_complete_releases_wire_before_draining_pending_config() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| {
            reply.stage_id(0x42);
            reply.stage_rdt(500);
            reply.stage_reboot(BootMode::Bootloader);
            reply.send_status(empty_status()).expect("encode fits");
        });
        bus.tx_bus.log.clear();

        assert_eq!(bus.id, TEST_ID);
        assert_eq!(bus.rdt_us, TEST_RDT_US);

        let pending_reboot = bus.on_tx_complete();

        assert_eq!(bus.tx_bus.log.as_slice(), &[TxBusOp::HandleTxComplete]);
        assert_eq!(bus.id, 0x42);
        assert_eq!(bus.rdt_us, 500);
        assert_eq!(pending_reboot, Some(BootMode::Bootloader));
    }

    // ------------------------------------------------------------------
    // Chain fire for slots k > 0 (DXL streaming RX §5.2)
    // ------------------------------------------------------------------

    /// Emit a BulkRead with the given (id, addr, length) slots — host
    /// targets BROADCAST per the spec, so target-addressable resolves to
    /// our chain participation rather than the chain's target ID.
    fn wire_bulk_read(slots: &[(u8, u16, u16)]) -> Vec<u8, 32> {
        use dxl_protocol::BulkReadEntry;
        let mut entries: Vec<BulkReadEntry, 4> = Vec::new();
        for &(id, address, length) in slots {
            entries
                .push(BulkReadEntry {
                    id: Id::new(id),
                    address,
                    length,
                })
                .unwrap();
        }
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .bulk_read(&entries)
            .unwrap();
        out
    }

    #[test]
    fn sync_read_slot_zero_has_no_predecessor() {
        let req = wire_sync_read(0, 2, &[TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .last_reply_ctx
            .expect("SyncRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, None);
    }

    #[test]
    fn sync_read_slot_k_gt_zero_records_immediate_predecessor() {
        let req = wire_sync_read(0, 2, &[0x42, 0x09, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .last_reply_ctx
            .expect("SyncRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, Some(0x09));
    }

    #[test]
    fn bulk_read_slot_k_gt_zero_records_immediate_predecessor() {
        let req = wire_bulk_read(&[(0x42, 0, 2), (TEST_ID, 0, 2)]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .last_reply_ctx
            .expect("BulkRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, Some(0x42));
    }

    #[test]
    fn fast_sync_read_slot_k_gt_zero_has_no_predecessor() {
        // Fast chains carry no per-slot Status headers and therefore no
        // per-slot SkipComplete events — chain-fire is absolute-deadline,
        // not sequence-driven. `predecessor_id` must stay None per
        // `docs/dxl-streaming-rx.md` §5.2.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .last_reply_ctx
            .expect("FastSyncRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, None);
    }

    #[test]
    fn send_status_defers_wire_send_for_chain_k_gt_zero() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        assert!(
            bus.scheduler.log.is_empty(),
            "chain k > 0 does not arm a deadline",
        );
        assert!(
            bus.tx_bus.log.is_empty(),
            "wire send waits for SkipComplete",
        );
        assert_eq!(bus.predecessor_id, Some(0x42));
    }

    #[test]
    fn skip_complete_matching_predecessor_fires_start_now() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));
        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(bus.predecessor_id, Some(0x42));

        // Predecessor's Status frame — byte-skip consumes its body and
        // surfaces SkipComplete { id: 0x42 } at exhaust.
        let pred_status = wire_status(0x42);
        stage_rx(&mut bus, req.len() as u16, &pred_status);
        bus.poll(|_, _, _| {});

        assert_eq!(
            bus.tx_bus.log.as_slice(),
            &[TxBusOp::StartNow { byte_count }],
        );
        assert!(
            bus.predecessor_id.is_none(),
            "chain-pending cleared on match",
        );
    }

    #[test]
    fn skip_complete_mismatched_predecessor_does_not_fire() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        // Some unrelated servo replies first — SkipComplete fires with id=0x05.
        let other_status = wire_status(0x05);
        stage_rx(&mut bus, req.len() as u16, &other_status);
        bus.poll(|_, _, _| {});

        assert!(bus.tx_bus.log.is_empty());
        assert_eq!(
            bus.predecessor_id,
            Some(0x42),
            "still waiting for the immediate predecessor",
        );
    }

    #[test]
    fn cancel_clears_chain_pending() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|ev, _, reply| {
            if matches!(ev, Event::Crc) {
                reply.send_status(empty_status()).expect("encode fits");
                reply.cancel();
            }
        });
        assert!(bus.predecessor_id.is_none());
    }

    #[test]
    fn on_tx_complete_clears_chain_pending() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));
        assert_eq!(bus.predecessor_id, Some(0x42));

        bus.on_tx_complete();
        assert!(bus.predecessor_id.is_none());
    }

    #[test]
    fn resync_clears_chain_pending() {
        // Stage chain-pending directly — the alternative is a full chain
        // + corrupted-predecessor encode, far more setup for the same
        // observable.
        let mut bus = make_bus();
        bus.predecessor_id = Some(0x42);
        let mut bad = wire_ping(TEST_ID);
        let crc_lo = bad.len() - 2;
        bad[crc_lo] ^= 0xFF;
        stage_rx(&mut bus, 0, &bad);
        bus.poll(|_, _, _| {});
        assert!(bus.predecessor_id.is_none());
    }

    #[test]
    fn instruction_header_clears_stale_chain_pending() {
        // Per `dxl-streaming-rx.md` §5.3: any stale `predecessor_id` from a
        // prior chain whose immediate predecessor went silent must clear at
        // the next instruction-header event, or a foreign Status whose id
        // happens to match would trigger a spurious `start_now`.
        let mut bus = make_bus();
        bus.predecessor_id = Some(0x42);
        let req = wire_ping(TEST_ID);
        stage_rx(&mut bus, 0, &req);
        bus.poll(|_, _, _| {});
        assert!(
            bus.predecessor_id.is_none(),
            "instruction-header event must reset stale chain-pending",
        );
    }

    // ------------------------------------------------------------------
    // Fast Last NDTR fold (Chunk 5)
    // ------------------------------------------------------------------

    /// `fold_start_cursor` lands at the host chain instruction's CRC byte
    /// — the codec's wire-byte cursor past that point is where the First
    /// servo's leading `0xFF` will arrive. Any earlier capture would let
    /// the host's own request bytes feed into the chain CRC.
    #[test]
    fn poll_captures_fold_start_cursor_at_crc() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .last_reply_ctx
            .expect("FastSyncRead surfaces a reply context");
        assert_eq!(ctx.fold_start_cursor, req.len() as u32);
    }

    /// Drive `on_fold_step` with predecessor bytes staged in `rx_buf` and
    /// assert the running fold absorbs them. Predecessor_bytes here is
    /// chosen `> staged.len()` so the FSM stays in `PeriodicWalk` (no
    /// finalize-on-target this pass) — the assertion is purely that the
    /// raw drain fed the bytes through.
    #[test]
    fn on_fold_step_drains_raw_bytes_into_fold() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        // FastLast is armed via `send_slot(Last)` above. The fold's
        // start_cursor = fold_start_cursor = req.len(); stage predecessor
        // bytes starting at that same wire-byte position so the drain's
        // cursor matches `start_cursor` on the first byte folded.
        let start = req.len() as u16;
        // SyncRead has 2 slots → bytes_before for our slot = 12 wire bytes
        // (`FAST_SLOT_HEADER_BYTES + body`). Stage fewer so the FSM doesn't
        // hit its busy-wait target this poll.
        let predecessor = [0x11_u8, 0x22, 0x33, 0x44];
        stage_rx(&mut bus, start, &predecessor);
        // RxDma.remaining = N − (start + len) so on_publish leaves
        // write_seq at start+len (the same head stage_rx_bytes_for_test set).
        let new_head = start.wrapping_add(predecessor.len() as u16);
        bus.rx_dma
            .remaining
            .set((RX_BUF_LEN as u16).wrapping_sub(new_head));

        bus.codec.set_rx_read_seq_for_test(start);
        let before = bus.fast_last_crc.bytes_folded();
        bus.on_fold_step();
        let after = bus.fast_last_crc.bytes_folded();
        assert_eq!(after - before, predecessor.len() as u32);
    }

    /// CC3 fire body folds the GUARD residue and patches the trailing
    /// CRC. Stage `predecessor_bytes` worth of bytes so finalize lands on
    /// the last drained byte; assert the TX buffer's trailing slot is no
    /// longer the placeholder `[0x00, 0x00]`.
    #[test]
    fn on_tx_start_folds_residue_and_patches_crc() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        // bytes_before for the second slot of a Fast SyncRead with length=2
        // is `FAST_SLOT_HEADER_BYTES(8) + body(4) = 12`. Stage exactly that
        // many predecessor bytes so finalize lands inside on_tx_start.
        let start = req.len() as u16;
        let predecessor = [
            0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC,
        ];
        stage_rx(&mut bus, start, &predecessor);
        let new_head = start.wrapping_add(predecessor.len() as u16);
        bus.rx_dma
            .remaining
            .set((RX_BUF_LEN as u16).wrapping_sub(new_head));
        bus.codec.set_rx_read_seq_for_test(start);

        // Snapshot the trailing slot before the fold runs.
        let tx_len_before = bus.codec.tx_len() as usize;
        // SAFETY: tx_buf is initialized up to tx_len_before; reading by raw
        // pointer matches the production DMA1_CH4 view.
        let trailing_before = unsafe {
            core::slice::from_raw_parts(bus.codec.tx_buf_addr() as *const u8, tx_len_before)
        }[tx_len_before - 2..]
            .to_vec();
        assert_eq!(trailing_before, [0x00, 0x00]);

        // send_slot(Last) above pushed a FastLast `Schedule` entry;
        // on_tx_start now routes `handle_start` through `TxBus` rather
        // than the scheduler. Assert only the trailing op so this test
        // stays focused on the post-fire fold (FastLast scheduling math
        // is a fast_last test concern).
        bus.on_tx_start();
        assert_eq!(
            bus.tx_bus.log.last(),
            Some(&TxBusOp::HandleStart),
            "on_tx_start must call handle_start once",
        );

        let tx_len_after = bus.codec.tx_len() as usize;
        let trailing_after = unsafe {
            core::slice::from_raw_parts(bus.codec.tx_buf_addr() as *const u8, tx_len_after)
        }[tx_len_after - 2..]
            .to_vec();
        assert_ne!(
            trailing_after,
            [0x00, 0x00],
            "patch_crc should overwrite the placeholder slot"
        );
        assert!(!bus.fast_last_crc.is_active());
        assert_eq!(
            bus.fast_last.scheduler().patch_miss_count.get(),
            0,
            "finalize path must not bump the deadline-miss counter",
        );
    }

    /// Bytes-starved CC3 body must not hang. With no predecessor bytes
    /// staged the plateau check (no progress between drain passes) exits
    /// the loop; trailing CRC stays at the placeholder. Plateau-exit also
    /// bumps `crc_patch_deadline_miss` — same observable failure as the
    /// expired-window route.
    #[test]
    fn on_tx_start_plateau_records_miss() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        // Pin write_seq == read_seq == start. The NDTR value that keeps
        // on_publish a no-op is `LEN - write_seq` (ring_pos = LEN - remaining
        // = start, prev_pos = start → delta = 0). Setting remaining = LEN
        // would advance the publisher by LEN - start bytes of garbage and
        // mask the plateau backstop under test.
        let start = req.len() as u16;
        stage_rx(&mut bus, start, &[]);
        bus.codec.set_rx_read_seq_for_test(start);
        bus.rx_dma
            .remaining
            .set((RX_BUF_LEN as u16).wrapping_sub(start));

        bus.on_tx_start();
        assert!(bus.fast_last_crc.is_active(), "active stays set on bail");
        assert_eq!(
            bus.fast_last.scheduler().patch_miss_count.get(),
            1,
            "plateau-exit must bump the deadline-miss counter",
        );
    }

    /// CH4 prefetch has reached the trailing CRC slot before finalize
    /// landed. The expired-window check exits the loop and bumps
    /// `crc_patch_deadline_miss`; placeholder CRC ships on the wire.
    #[test]
    fn on_tx_start_window_expiry_records_miss() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        bus.fast_last
            .scheduler()
            .patch_window_expired_value
            .set(true);

        bus.on_tx_start();
        assert!(
            bus.fast_last_crc.is_active(),
            "active stays set on expired-window exit",
        );
        assert_eq!(
            bus.fast_last.scheduler().patch_miss_count.get(),
            1,
            "expired-window exit must bump the deadline-miss counter",
        );
    }

    fn allows_fallback(header: InstructionHeader) -> bool {
        InflightCtx::new(header).allows_packet_end_fallback()
    }

    #[test]
    fn inflight_for_single_target_allows_packet_end_fallback() {
        assert!(allows_fallback(InstructionHeader::Ping {
            id: Id::new(TEST_ID),
        }));
        assert!(allows_fallback(InstructionHeader::Read {
            id: Id::new(TEST_ID),
            address: 0,
            length: 2,
        }));
        assert!(allows_fallback(InstructionHeader::Write {
            id: Id::new(TEST_ID),
            address: 0,
            length: 2,
        }));
        assert!(allows_fallback(InstructionHeader::RegWrite {
            id: Id::new(TEST_ID),
            address: 0,
            length: 2,
        }));
    }

    #[test]
    fn inflight_for_plain_sync_and_bulk_allows_packet_end_fallback() {
        assert!(allows_fallback(InstructionHeader::SyncRead {
            id: Id::new(BROADCAST_ID),
            address: 0,
            length: 2,
        }));
        assert!(allows_fallback(InstructionHeader::BulkRead {
            id: Id::new(BROADCAST_ID),
        }));
    }

    #[test]
    fn inflight_for_fast_sync_read_disallows_packet_end_fallback() {
        assert!(!allows_fallback(InstructionHeader::FastSyncRead {
            id: Id::new(BROADCAST_ID),
            address: 0,
            length: 2,
        }));
    }

    #[test]
    fn inflight_for_fast_bulk_read_disallows_packet_end_fallback() {
        assert!(!allows_fallback(InstructionHeader::FastBulkRead {
            id: Id::new(BROADCAST_ID),
        }));
    }
}
