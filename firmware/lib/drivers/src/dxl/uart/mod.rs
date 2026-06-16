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

use crate::traits::dxl::{Providers, SendKind, TxScheduler};
use clock::Clock;
use codec::{Codec, CodecTx, PollAction, PollEvent};
use fast_last::{FastLast, FastLastSchedule};
use fast_last_crc::FastLastCrc;

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
    /// Packet-end tick = `current_byte_tick + 10·tpb`, captured at the
    /// classifier's instruction Crc-good event. `None` if the classifier
    /// anchor was lost mid-packet — send drops silently.
    packet_end_tick: Option<u16>,
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
    /// Codec's wire-byte cursor at the instruction-header surface. The first
    /// predecessor reply byte arrives at this cursor; Fast Last fold pipeline
    /// uses it as `start_cursor` so the fold skips the host's request bytes.
    fold_start_cursor: u32,
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
}

impl InflightCtx {
    fn new(header: InstructionHeader, fold_start_cursor: u32) -> Self {
        Self {
            header,
            fold_start_cursor,
            next_slot_index: 0,
            slot: None,
            bytes_before: 0,
            slot_length: None,
        }
    }

    /// Final ReplyContext at Crc-good. `packet_end_tick` is captured from the
    /// classifier at the same event.
    fn into_reply_context(self, id: u8, packet_end_tick: Option<u16>) -> ReplyContext {
        let (slot_offset_bytes, fast_slot_position) = match (self.header, self.slot) {
            (InstructionHeader::Ping { id: target }, _) if target.as_byte() == BROADCAST_ID => {
                ((id as u32) * PING_STATUS_FRAME_BYTES, None)
            }
            (InstructionHeader::SyncRead { length, .. }, Some(k)) => {
                let per_slot = RESPONSE_HEADER_BYTES as u32 + length as u32 + CRC_BYTES as u32;
                ((k as u32) * per_slot, None)
            }
            (InstructionHeader::BulkRead { .. }, Some(_)) => (self.bytes_before, None),
            (InstructionHeader::FastSyncRead { length, .. }, Some(k)) => {
                let n = self.next_slot_index;
                let position = compute_fast_position(k, n, length as u32);
                let bytes_before = fast_bytes_before(k, length as u32);
                (bytes_before, Some(position))
            }
            (InstructionHeader::FastBulkRead { .. }, Some(k)) => {
                let n = self.next_slot_index;
                let slot_length = self.slot_length.unwrap_or(0) as u32;
                let position = compute_fast_position(k, n, slot_length);
                (self.bytes_before, Some(position))
            }
            _ => (0, None),
        };
        ReplyContext {
            packet_end_tick,
            slot_offset_bytes,
            fast_slot_position,
            fold_start_cursor: self.fold_start_cursor,
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
    if slot_id.as_byte() == id && ctx.slot.is_none() {
        ctx.slot = Some(k);
        ctx.slot_length = slot_length;
        return;
    }
    if ctx.slot.is_some() {
        return;
    }
    // Predecessor slot — accumulate wire bytes for Bulk variants. Per-slot
    // shape depends on whether the chain is Fast (compact) or Plain
    // (Status-per-slot).
    if let Some(length) = slot_length {
        let length = length as u32;
        match ctx.header {
            InstructionHeader::BulkRead { .. } => {
                ctx.bytes_before += RESPONSE_HEADER_BYTES as u32 + length + CRC_BYTES as u32;
            }
            InstructionHeader::FastBulkRead { .. } => {
                let bytes = if k == 0 {
                    fast_first_bytes(length)
                } else {
                    fast_middle_bytes(length)
                };
                ctx.bytes_before += bytes;
            }
            _ => {}
        }
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

/// Map a chain `(slot_index, total_slots, slot_length)` to the
/// [`SlotPosition`] the encoder consumes. `packet_length` on First/Only is
/// the chain's advertised wire length — emitted straight onto the wire by
/// [`dxl_protocol::encoder::SlotEncoder::emit`]. Surfacing `0` here is
/// interim: the chain-total length math (sum of all slot bodies + chain CRC)
/// needs the full slot-list walk Chunks 5/6 plumb. `slot_length` is the
/// chip's own slot length, captured here for that future arithmetic.
fn compute_fast_position(k: u8, n_total: u8, _slot_length: u32) -> SlotPosition {
    if n_total == 1 {
        SlotPosition::Only { packet_length: 0 }
    } else if k == 0 {
        SlotPosition::First { packet_length: 0 }
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
    /// Encode a Status reply into the codec's TX buffer and schedule its
    /// wire start. Fold RDT + slot offset (broadcast Ping / Sync / Bulk
    /// Read slot N) from the [`ReplyContext`] cached at `poll()`, convert
    /// the µs delay to scheduler ticks via `TxScheduler::TICKS_PER_US`, and
    /// hand the pre-computed `deadline_tick` to the provider. If no context is
    /// staged (foreign Instruction filtered upstream) or the wire-end tick
    /// wasn't ready, the encode still succeeds but no schedule is armed —
    /// the bytes simply won't ship. Context is taken on use so a
    /// double-send without a fresh `poll()` no-ops on the scheduler.
    pub fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        self.tx.send_status(status)?;
        let byte_count = self.tx.tx_len();
        let Some(ctx) = self.last_reply_ctx.take() else {
            return Ok(());
        };
        let Some(packet_end_tick) = ctx.packet_end_tick else {
            return Ok(());
        };
        let delay_us = self.rdt_us + self.clock.bytes_to_us(ctx.slot_offset_bytes);
        let delay_ticks =
            delay_us.wrapping_mul(<P::TxScheduler as TxScheduler>::TICKS_PER_US as u32);
        let deadline_tick = packet_end_tick.wrapping_add(delay_ticks as u16);
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
        let Some(packet_end_tick) = ctx.packet_end_tick else {
            return Ok(());
        };
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
            let byte_ticks = self.clock.ticks_per_bit().wrapping_mul(10);
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

    /// Drop any scheduled TX and clear the staged request state so the next
    /// reply must come through a fresh `poll()`.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        *self.last_reply_ctx = None;
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
/// - `DECODER_CAP`: streaming-decoder accumulator size. Sized to hold the
///   longest unstuffed frame the dispatcher will encounter (typically 256
///   covers max-RW + header + margin); decoupled from the on-wire RX byte
///   ring because the parser drains continuously per doc §8.1.
/// - `RX_BUF_LEN`: DMA1_CH5 byte-ring depth (typically 64 per doc §8.1).
///   Also drives the BT ring depth inside `Codec` — doc §8.3 requires
///   they match so byte index `i` in RX maps to `BT[i mod RX_BUF_LEN]`,
///   and the composite enforces that coupling by construction.
/// - `EDGE_BUF_LEN`: DMA1_CH7 edge-timestamp ring depth (typically 128 /
///   option A in doc §8.4; 64 / option B trades CPU for memory).
/// - `TX_BUF_LEN`: DMA1_CH4 source-buffer depth sized to
///   `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES` (140 with the
///   default control-RW). Held by `Codec` so encoder methods write into
///   driver-owned storage instead of a chip-side static.
pub struct DxlUart<
    P: Providers,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> {
    codec: Codec<P::EdgeDma, P::Crc, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
    clock: Clock<P::UsartBaud, P::ClockTrim>,
    scheduler: P::TxScheduler,
    /// Periodic-walk grid scheduler for Fast Sync / Bulk Read Last replies.
    /// Driver-pattern §4 sub-driver: armed at `send_slot(Last)` via
    /// `ReplyHandle`; `on_systick_match` drives one body per CMP.
    fast_last: FastLast<P::FastLastScheduler>,
    /// Chain-CRC fold engine. Per-byte hook is wired into the codec's
    /// `poll_one` callback during the SysTick walker; finalize patches our
    /// own trailing CRC slot before DMA1_CH4 reads it.
    fast_last_crc: FastLastCrc<P::Crc>,

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

    pending_id: Option<u8>,
    pending_rdt_us: Option<u32>,
    pending_reboot: Option<BootMode>,
}

impl<
    P: Providers,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> DxlUart<P, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    pub fn new(
        codec: Codec<P::EdgeDma, P::Crc, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
        clock: Clock<P::UsartBaud, P::ClockTrim>,
        scheduler: P::TxScheduler,
        fast_last: FastLast<P::FastLastScheduler>,
        id: u8,
        rdt_us: u32,
    ) -> Self {
        Self {
            codec,
            clock,
            scheduler,
            fast_last,
            fast_last_crc: FastLastCrc::new(),
            id,
            rdt_us,
            last_reply_ctx: None,
            inflight: None,
            pending_id: None,
            pending_rdt_us: None,
            pending_reboot: None,
        }
    }

    /// New RX falling-edge timestamps may be available — pull the current
    /// `ticks_per_bit` from the clock and walk the classifier. Drift pairs
    /// emit while the classifier's `hsi_active` flag is set (instructions in
    /// flight); they route into the clock's drift integrator.
    pub fn on_rx_edge_advance(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        let clock = &mut self.clock;
        self.codec
            .on_edge_advance(ticks_per_bit, |prev, curr| clock.on_byte_pair(prev, curr));
    }

    /// RX wire went idle — drain tail edges (drift-routed when active) and
    /// reset the classifier anchor for the next burst.
    pub fn on_rx_idle(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        let clock = &mut self.clock;
        self.codec
            .on_idle(ticks_per_bit, |prev, curr| clock.on_byte_pair(prev, curr));
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
    /// `Ch32Bus::poll` forwarder is a one-liner (Chunk 6 wires it).
    ///
    /// Per [doc §3 / §4.3]:
    /// - Instruction Header → `try_anchor_from_header`, `set_hsi_active(true)`.
    /// - Instruction Header (foreign) → universal byte-skip on the body.
    /// - Status Header → universal byte-skip on the body.
    /// - SyncSlot / BulkSlot → slot-walk against `self.id`.
    /// - Crc → stamp packet-end tick, derive [`ReplyContext`],
    ///   `set_hsi_active(false)`.
    /// - Resync → `reset_anchor`, drop inflight, `set_hsi_active(false)`.
    ///
    /// Closure-based shape exists to break the borrow conflict between the
    /// parser-event borrow on the codec RX half and the `&mut` access the
    /// send path needs on the TX half. [`Codec::split_mut`] returns the two
    /// halves disjointly so the closure sees both at once.
    pub fn poll<F>(&mut self, mut f: F)
    where
        F: FnMut(Event, &[u8], &mut ReplyHandle<'_, P, TX_BUF_LEN>),
    {
        let id = self.id;
        let rdt_us = self.rdt_us;
        let ticks_per_bit = self.clock.ticks_per_bit();
        let Self {
            codec,
            clock,
            scheduler,
            fast_last,
            fast_last_crc,
            last_reply_ctx,
            inflight,
            pending_id,
            pending_rdt_us,
            pending_reboot,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        rx.poll(|pe, rx_inner| match pe {
            PollEvent::Event { ev, ring } => {
                let action = match ev {
                    Event::Header(HeaderEvent::Instruction(h)) => {
                        rx_inner.try_anchor_from_header(ticks_per_bit);
                        rx_inner.set_hsi_active(true);
                        if target_addressable(&h, id) {
                            // TODO(Chunk 5 / #5): fold_start_cursor is the
                            // codec's wire-byte cursor at packet start.
                            // CodecRx::wire_byte_cursor is not reachable
                            // from inside the poll callback (parser borrow);
                            // pinned at 0 until Chunk 5 rewires Fast Last to
                            // read raw NDTR bytes directly per doc §6.
                            *inflight = Some(InflightCtx::new(h, 0));
                            PollAction::Continue
                        } else {
                            *inflight = None;
                            PollAction::Skip {
                                id: header_target(&h).as_byte(),
                            }
                        }
                    }
                    Event::Header(HeaderEvent::Status(sh)) => {
                        *inflight = None;
                        PollAction::Skip {
                            id: sh.id.as_byte(),
                        }
                    }
                    Event::Payload(PayloadEvent::Instruction(p)) => {
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
                        rx_inner.set_hsi_active(false);
                        if let Some(ctx) = inflight.take() {
                            let packet_end_tick = rx_inner.packet_end_tick(ticks_per_bit);
                            *last_reply_ctx = Some(ctx.into_reply_context(id, packet_end_tick));
                        }
                        PollAction::Continue
                    }
                    Event::Resync(_) => {
                        rx_inner.reset_anchor();
                        rx_inner.set_hsi_active(false);
                        *inflight = None;
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
                    pending_id,
                    pending_rdt_us,
                    pending_reboot,
                    id,
                    rdt_us,
                };
                f(ev, ring, &mut reply);
                action
            }
            PollEvent::SkipComplete { .. } => PollAction::Continue,
        });
    }

    /// Monotonic count of Instruction packets seen on the wire — own and
    /// foreign IDs included; Status frames excluded. Drift estimator's
    /// tick source.
    pub fn instruction_count(&self) -> u32 {
        self.codec.instruction_count()
    }

    /// The TX-start tick has arrived (chip-side CC3 IRQ) — route to the
    /// scheduler so it activates the wire driver. The Fast Last post-fire
    /// residue fold body lives here once Chunk 5 wires it; today the path
    /// is a no-op stub because the classifier-and-parser walk that lived
    /// inline got retired alongside the BT ring (Chunks 2/3). Chunk 5
    /// replaces with an NDTR-driven raw-byte fold per doc §6.
    pub fn on_tx_start(&mut self) {
        self.scheduler.handle_start();
        // TODO(Chunk 5 / #5): NDTR-fold body that absorbs predecessor + own
        // bytes into the chain CRC and patches before DMA1_CH4 reads the
        // trailing slot. The classifier-walk shape that lived here is
        // retired per dxl-streaming-rx.md §6.
    }

    /// One Fast Last periodic-walk fold body is due. The composite's
    /// SysTick demux calls this. First entry pauses DMA1_CH7 HT/TC so the
    /// classifier ISR can't preempt the body (doc §10.6.3); subsequent
    /// entries no-op the pause (idempotent). The fold body itself lives
    /// here once Chunk 5 wires it; today it's a stub for the same reason
    /// `on_tx_start` is — the classifier walk got retired.
    pub fn on_fold_step(&mut self) {
        self.codec.pause_edges();
        // TODO(Chunk 5 / #5): SysTick periodic-walk body that drives the
        // NDTR fold engine forward. Was classifier+parser walk; retired.
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
        self.scheduler.handle_tx_complete();
        self.fast_last.cancel();
        self.fast_last_crc.cancel();
        self.codec.resume_edges();
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
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use super::*;
    use crate::mocks::{
        FakeClockTrim, FakeEdgeDma, FakeFastLastScheduler, FakeTxScheduler, FakeUsartBaud,
        FastLastSchedulerOp, ScheduleOp, TestProviders,
    };
    use dxl_protocol::types::StatusError;
    use dxl_protocol::{InstructionEncoder, SoftwareCrcUmts, StatusEncoder};
    use heapless::Vec;
    use osc_core::BaudRate;

    /// Test-side storage sizing — matches V006 defaults per doc §§8.1, 8.3,
    /// 8.4 so any drift between driver tests and chip-side reality stays
    /// visible.
    const DECODER_CAP: usize = 256;
    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;
    const TX_BUF_LEN: usize = 140;

    const TEST_ID: u8 = 0x07;
    const TEST_RDT_US: u32 = 250;

    type TestCodec =
        Codec<FakeEdgeDma, SoftwareCrcUmts, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;
    type TestBus = DxlUart<TestProviders, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

    fn make_clock(baud: BaudRate) -> Clock<FakeUsartBaud, FakeClockTrim> {
        Clock::new(baud, FakeUsartBaud::default(), FakeClockTrim::default())
    }

    fn make_bus_with(codec: TestCodec, baud: BaudRate) -> TestBus {
        DxlUart::new(
            codec,
            make_clock(baud),
            FakeTxScheduler::default(),
            FastLast::new(FakeFastLastScheduler::default()),
            TEST_ID,
            TEST_RDT_US,
        )
    }

    fn make_bus() -> TestBus {
        make_bus_with(Codec::new(FakeEdgeDma::default()), BaudRate::B3000000)
    }

    /// Pre-seed classifier `last_byte_start` so `packet_end_tick` reads a
    /// known tick at Crc-good without staging real edges. Composite tests
    /// for the TX scheduling path use this — the classifier-leaf tests cover
    /// the per-edge walker; the composite asserts the wiring.
    const SEED_TICK: u16 = 1000;
    fn force_anchor(bus: &mut TestBus) {
        bus.codec.force_byte_tick_for_test(SEED_TICK);
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
        bus.codec.stage_rx_bytes_for_test(0, bytes);
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
        bus.codec.stage_rx_bytes_for_test(start, &pkt);

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
        bus.codec.stage_rx_bytes_for_test(0, &pkt[..split]);
        let mut tags = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| tags.push(ev_tag(ev)));
        assert!(!saw_crc(&tags));
        // Header ticked instruction_count even on the partial — the parser
        // surfaces Header before it has the trailing CRC.
        assert_eq!(bus.instruction_count(), 1);

        bus.codec
            .stage_rx_bytes_for_test(split as u16, &pkt[split..]);
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
        bus.codec.stage_rx_bytes_for_test(0, &pkt);
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
        bus.codec.stage_rx_bytes_for_test(0, &bad);
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
        bus.codec.stage_rx_bytes_for_test(0, &combined);

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
        bus.codec.stage_rx_bytes_for_test(0, pkt);
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
    fn sync_read_folds_bytes_before_for_later_slot() {
        let req = wire_sync_read(0, 2, &[0x09, 0x05, TEST_ID]);
        let (mut bus, packet_end_tick) = bus_seeded_with(&req);

        // per_slot = RESPONSE_HEADER_BYTES(9) + length(2) + CRC(2) = 13;
        // bytes_before = 2 × 13 = 26.
        let expected_offset_us = bus.clock.bytes_to_us(26);
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
        assert_eq!(bus.scheduler.log.as_slice(), &[ScheduleOp::HandleStart]);
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
        bus.scheduler.log.clear();

        assert_eq!(bus.id, TEST_ID);
        assert_eq!(bus.rdt_us, TEST_RDT_US);

        let pending_reboot = bus.on_tx_complete();

        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::HandleTxComplete]
        );
        assert_eq!(bus.id, 0x42);
        assert_eq!(bus.rdt_us, 500);
        assert_eq!(pending_reboot, Some(BootMode::Bootloader));
    }
}
