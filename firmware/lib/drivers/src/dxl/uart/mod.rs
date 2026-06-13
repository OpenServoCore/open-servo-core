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

use dxl_protocol::packet::{Id, Slot, Status};
use dxl_protocol::{
    BROADCAST_ID, CRC_BYTES, CrcUmts, InstructionPacket, RESPONSE_HEADER_BYTES, SlotPosition,
    WriteError,
};
use osc_core::{BaudRate, BootMode, DxlReply};

use crate::traits::{ClockTrim, DmaRing, DxlTxScheduler, SendKind, UsartBaud};
use crate::util::Seq;
use clock::Clock;
use codec::{Codec, CodecTx};

/// Wire bytes of a single Ping Status reply — `RESPONSE_HEADER_BYTES`
/// (header(4) + id + len(2) + inst + err = 9) + 3 payload bytes
/// (model_lo + model_hi + firmware) + CRC_BYTES (2). Multi-slave broadcast
/// Ping convention positions slave N's reply at `N × PING_STATUS_FRAME_BYTES`
/// wire bytes past wire-end so the slaves don't collide.
const PING_STATUS_FRAME_BYTES: u32 = RESPONSE_HEADER_BYTES as u32 + 3 + CRC_BYTES as u32;

/// Spec cap on Sync/Bulk Read slot count. DXL 2.0 carries IDs in a single
/// byte; 0xFD/0xFE/0xFF are reserved, leaving 252 addressable. Used as the
/// `max_slots` bound on `find_slot` for Fast variants — protects against
/// malformed requests that would otherwise unbounded the slot walk.
const MAX_FAST_SLOTS: usize = 252;

/// What the driver needs to position a reply on the wire. Computed once at
/// `poll()` from the surfaced packet + `self.id`; consumed by `send_status`
/// / `send_slot`. Per driver-pattern §7.4 — the dispatcher passes data; the
/// driver derives wire shape from its cached request state.
#[derive(Copy, Clone, Debug, Default)]
struct ReplyContext {
    /// Wire-end tick = `BT[last_byte_seq] + 10·tpb`, captured eagerly at
    /// `poll()` so the send path doesn't need RX access (the in-flight
    /// packet borrow on `CodecRx` would otherwise block the BT lookup).
    /// `None` if BT hadn't caught up by poll surface — send drops silently.
    wire_end_tick: Option<u16>,
    /// Wire-byte offset from request wire-end to this reply's fire moment.
    /// Zero for direct unicast; non-zero for broadcast Ping and
    /// Sync/Bulk/Fast Read slot N.
    slot_offset_bytes: u32,
    /// Fast Sync/Bulk Read slot position. `None` for non-Fast paths;
    /// `Some(Last { .. })` flags the schedule with [`SendKind::FastLast`]
    /// so the chain-CRC catchup sub-driver (M6, `#6`) can pick it up.
    fast_slot_position: Option<SlotPosition>,
}

fn slot_offset_for(packet: &InstructionPacket<'_>, id: u8) -> u32 {
    match packet {
        InstructionPacket::Ping(p) if p.header.id.as_byte() == BROADCAST_ID => {
            (id as u32) * PING_STATUS_FRAME_BYTES
        }
        InstructionPacket::SyncRead(p) => {
            p.find_slot(Id::new(id)).map_or(0, |info| info.bytes_before)
        }
        InstructionPacket::BulkRead(p) => {
            p.find_slot(Id::new(id)).map_or(0, |info| info.bytes_before)
        }
        InstructionPacket::FastSyncRead(p) => p
            .find_slot(Id::new(id), MAX_FAST_SLOTS)
            .map_or(0, |info| info.bytes_before),
        InstructionPacket::FastBulkRead(p) => p
            .find_slot(Id::new(id), MAX_FAST_SLOTS)
            .map_or(0, |info| info.bytes_before),
        _ => 0,
    }
}

fn fast_slot_position(packet: &InstructionPacket<'_>, id: u8) -> Option<SlotPosition> {
    match packet {
        InstructionPacket::FastSyncRead(p) => p
            .find_slot(Id::new(id), MAX_FAST_SLOTS)
            .map(|i| i.position()),
        InstructionPacket::FastBulkRead(p) => p
            .find_slot(Id::new(id), MAX_FAST_SLOTS)
            .map(|i| i.position()),
        _ => None,
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
pub struct ReplyHandle<
    'a,
    U: UsartBaud,
    T: ClockTrim,
    S: DxlTxScheduler,
    CRC: CrcUmts,
    const TX_BUF_LEN: usize,
> {
    tx: &'a mut CodecTx<CRC, TX_BUF_LEN>,
    scheduler: &'a mut S,
    clock: &'a mut Clock<U, T>,
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

impl<U: UsartBaud, T: ClockTrim, S: DxlTxScheduler, CRC: CrcUmts, const TX_BUF_LEN: usize>
    ReplyHandle<'_, U, T, S, CRC, TX_BUF_LEN>
{
    /// Encode a Status reply into the codec's TX buffer and schedule its
    /// wire start. Fold RDT + slot offset (broadcast Ping / Sync / Bulk
    /// Read slot N) from the [`ReplyContext`] cached at `poll()`, convert
    /// the µs delay to scheduler ticks via `S::TICKS_PER_US`, and hand the
    /// pre-computed `deadline_tick` to the provider. If no context is
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
        let Some(wire_end_tick) = ctx.wire_end_tick else {
            return Ok(());
        };
        let delay_us = self.rdt_us + self.clock.bytes_to_us(ctx.slot_offset_bytes);
        let delay_ticks = delay_us.wrapping_mul(S::TICKS_PER_US as u32);
        let deadline_tick = wire_end_tick.wrapping_add(delay_ticks as u16);
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
        let Some(wire_end_tick) = ctx.wire_end_tick else {
            return Ok(());
        };
        let (delay_ticks, kind) = match position {
            SlotPosition::Last { .. } => {
                // Q8.8 µs × ticks/µs = Q8.8 ticks; >> 8 lands on integer ticks.
                let delay_q88 =
                    (self.rdt_us << 8) + self.clock.bytes_to_us_q88(ctx.slot_offset_bytes);
                let ticks = (delay_q88.wrapping_mul(S::TICKS_PER_US as u32)) >> 8;
                (ticks, SendKind::FastLast)
            }
            _ => {
                let delay_us = self.rdt_us + self.clock.bytes_to_us(ctx.slot_offset_bytes);
                let ticks = delay_us.wrapping_mul(S::TICKS_PER_US as u32);
                (ticks, SendKind::Plain)
            }
        };
        let deadline_tick = wire_end_tick.wrapping_add(delay_ticks as u16);
        self.scheduler.schedule(deadline_tick, byte_count, kind);
        // Chain-CRC predecessor fold lives on DxlChainCatchup (M6, #6),
        // which reads its own anchor from the codec at TX-start time —
        // nothing crosses the scheduler boundary here.
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

impl<U: UsartBaud, T: ClockTrim, S: DxlTxScheduler, CRC: CrcUmts, const TX_BUF_LEN: usize> DxlReply
    for ReplyHandle<'_, U, T, S, CRC, TX_BUF_LEN>
{
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

/// The DXL bus composite. The type parameters together describe everything
/// this driver wants the chip side to supply — what peripherals to drive
/// it with, what CRC to use, and how much memory each of its three rings
/// gets:
///
/// - `U`: USART baud-rate setter (sub-driver `Clock`).
/// - `T`: HSI / HSE trim setter (sub-driver `Clock`).
/// - `R`: DMA-ring ISR surface for DMA1_CH7 (carried by `Codec` through
///   its inner `Rx`).
/// - `S`: TX-start scheduler — given a pre-computed deadline tick (driver
///   does the µs→tick math via `S::TICKS_PER_US`), arms the chip's
///   TX-start hardware (M3 #5: TIM2_CH3 OC fire + TIM2_CH2 OC TX_EN).
/// - `CRC`: CRC-16/UMTS engine — software impl on V006, peripheral impl
///   on a chip that has one (see `providers::dxl_crc`).
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
    U: UsartBaud,
    T: ClockTrim,
    R: DmaRing,
    S: DxlTxScheduler,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> {
    codec: Codec<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
    clock: Clock<U, T>,
    scheduler: S,

    id: u8,
    rdt_us: u32,

    /// Wire-positioning info derived from the most recently surfaced
    /// packet — wire-end tick, slot offset, Fast slot position, chain-CRC
    /// anchor. Computed at poll surface and consumed by `ReplyHandle`'s
    /// send methods. Per driver-pattern §7.4 — driver-derivable wire shape
    /// stays in the driver, not on the trait surface.
    last_reply_ctx: Option<ReplyContext>,

    pending_id: Option<u8>,
    pending_rdt_us: Option<u32>,
    pending_reboot: Option<BootMode>,
}

impl<
    U: UsartBaud,
    T: ClockTrim,
    R: DmaRing,
    S: DxlTxScheduler,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> DxlUart<U, T, R, S, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    pub fn new(
        codec: Codec<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
        clock: Clock<U, T>,
        scheduler: S,
        id: u8,
        rdt_us: u32,
    ) -> Self {
        Self {
            codec,
            clock,
            scheduler,
            id,
            rdt_us,
            last_reply_ctx: None,
            pending_id: None,
            pending_rdt_us: None,
            pending_reboot: None,
        }
    }

    /// New RX falling-edge timestamps may be available — pull the current
    /// `ticks_per_bit` from the clock and forward to the codec's RX
    /// classifier.
    pub fn on_rx_edge_advance(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        self.codec.on_edge_advance(ticks_per_bit);
    }

    /// RX wire went idle — drain tail edges and reset the classifier
    /// anchor for the next burst.
    pub fn on_rx_idle(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        self.codec.on_idle(ticks_per_bit);
    }

    /// USART1 RX DMA published progress — `remaining` is the channel's
    /// NDTR readback.
    pub fn on_rx_dma_advance(&mut self, remaining: u16) {
        self.codec.on_rx_dma_advance(remaining);
    }

    /// Drain the codec until an Instruction addressed to us (or BROADCAST)
    /// surfaces; route each parsed Instruction's BT pair walk into the
    /// clock's drift integrator, derive the reply context from the parsed
    /// packet, and hand `(packet, &mut ReplyHandle)` to the dispatcher
    /// closure. Foreign Instructions feed drift but don't reach the
    /// closure; Status frames feed neither.
    ///
    /// The closure-based shape exists to break the otherwise-fatal borrow
    /// conflict between `InstructionPacket<'_>` (borrowed from the codec's
    /// RX half) and the `&mut` access the send path needs on the codec's
    /// TX half. The codec's [`Codec::split_mut`] returns the two halves as
    /// disjoint mutable references, so the closure sees both at once.
    pub fn poll<F>(&mut self, f: F)
    where
        F: for<'a> FnOnce(InstructionPacket<'a>, &mut ReplyHandle<'_, U, T, S, CRC, TX_BUF_LEN>),
    {
        let id = self.id;
        let rdt_us = self.rdt_us;
        let Self {
            codec,
            clock,
            scheduler,
            last_reply_ctx,
            pending_id,
            pending_rdt_us,
            pending_reboot,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        loop {
            let Some(token) = rx.poll_one() else { return };
            for (prev, curr) in rx.byte_pairs(token.start, token.end) {
                clock.on_byte_pair(prev, curr);
            }
            if token.id != id && token.id != BROADCAST_ID {
                // Foreign Instruction — drift fed above, drop and continue.
                continue;
            }
            // Eager wire-end tick: BT[last_byte_seq] + 10·tpb. Captured now
            // so the send path doesn't need to re-acquire `&rx` (the parsed
            // packet's shared borrow on `rx` would block that). `None` is
            // fine — send drops silently.
            let wire_end_tick = rx
                .byte_ts_at(token.end.predecessor())
                .map(|ts| ts.wrapping_add(clock.ticks_per_bit().wrapping_mul(10)));
            // dispatch() takes `&self` so the packet's borrow on `rx` is
            // shared — compatible with the (already-captured) BT lookup
            // and the disjoint `&mut tx` the reply handle holds.
            let Some(packet) = rx.dispatch() else { return };
            let ctx = ReplyContext {
                wire_end_tick,
                slot_offset_bytes: slot_offset_for(&packet, id),
                fast_slot_position: fast_slot_position(&packet, id),
            };
            *last_reply_ctx = Some(ctx);
            let mut reply = ReplyHandle {
                tx,
                scheduler,
                clock,
                last_reply_ctx,
                pending_id,
                pending_rdt_us,
                pending_reboot,
                id,
                rdt_us,
            };
            f(packet, &mut reply);
            return;
        }
    }

    /// Monotonic count of Instruction packets seen on the wire — own and
    /// foreign IDs included; Status frames excluded. Drift estimator's
    /// tick source.
    pub fn instruction_count(&self) -> u32 {
        self.codec.instruction_count()
    }

    /// The TX-start tick has arrived (chip-side CC3 IRQ) — route to the
    /// scheduler so it activates the wire driver (USART TE on, TX DMA on).
    /// Logical event name per [[driver_events_logical_naming]]; the chip
    /// ISR's peripheral-shaped name (`on_tim2_cc3`) lives in `runtime/isr.rs`.
    pub fn on_tx_start(&mut self) {
        self.scheduler.handle_start();
    }

    /// USART1 TC fired — the reply has fully drained the wire. Release
    /// the wire driver *first* (drop TX_EN, mask TC IRQ, disable DMA) so
    /// stale TX_EN doesn't sit on the bus while pending config mutates;
    /// then drain staged writes (id / baud / trim / rdt) and surface any
    /// pending reboot to the chip-side ISR. Reboot is returned (rather
    /// than self-applied) because the chip controls how the reset
    /// actually happens; the driver only knows it was asked.
    pub fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.scheduler.handle_tx_complete();
        if let Some(id) = self.pending_id.take() {
            self.id = id;
        }
        if let Some(rdt) = self.pending_rdt_us.take() {
            self.rdt_us = rdt;
        }
        self.clock.on_tx_complete();
        self.pending_reboot.take()
    }

    /// Sequence number of the next BT slot to write — one past the last
    /// published BT entry.
    #[allow(dead_code)]
    pub fn byte_ts_head(&self) -> Seq<u16, RX_BUF_LEN> {
        self.codec.byte_ts_head()
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
    use super::*;
    use crate::mocks::{FakeClockTrim, FakeDmaRing, FakeDxlTxScheduler, FakeUsartBaud, ScheduleOp};
    use crate::traits::DmaFlags;
    use dxl_protocol::packet::{Id, StatusError};
    use dxl_protocol::{InstructionEmitter, SoftwareCrcUmts, StatusEmitter};
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
    /// One byte-time at 3 Mbaud (10·tpb).
    const BYTE_TICKS_3M: u16 = 160;

    type TestCodec =
        Codec<FakeDmaRing, SoftwareCrcUmts, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;
    type TestBus = DxlUart<
        FakeUsartBaud,
        FakeClockTrim,
        FakeDmaRing,
        FakeDxlTxScheduler,
        SoftwareCrcUmts,
        DECODER_CAP,
        RX_BUF_LEN,
        EDGE_BUF_LEN,
        TX_BUF_LEN,
    >;

    fn make_codec_with_edges(vals: &[u16], flags: DmaFlags) -> TestCodec {
        let mut c: TestCodec = Codec::new(FakeDmaRing::default());
        c.stage_edges_for_test(vals);
        c.arm_next_flags_for_test(flags);
        c
    }

    fn make_clock(baud: BaudRate) -> Clock<FakeUsartBaud, FakeClockTrim> {
        Clock::new(baud, FakeUsartBaud::default(), FakeClockTrim::default())
    }

    fn make_bus_with(codec: TestCodec, baud: BaudRate) -> TestBus {
        DxlUart::new(
            codec,
            make_clock(baud),
            FakeDxlTxScheduler::default(),
            TEST_ID,
            TEST_RDT_US,
        )
    }

    fn make_bus() -> TestBus {
        make_bus_with(Codec::new(FakeDmaRing::default()), BaudRate::B3000000)
    }

    /// Variant of [`TestBus::poll`] that captures the surfaced packet's
    /// outer enum tag (Ping / SyncRead / Status / …) for tests that don't
    /// also exercise a send path. Tests that DO send use a closure that
    /// records via assertions inline.
    #[derive(Default, Debug, PartialEq, Eq)]
    enum Surfaced {
        #[default]
        None,
        Ping,
        SyncRead,
        FastSyncRead,
    }

    fn poll_into(bus: &mut TestBus) -> Surfaced {
        let mut out = Surfaced::None;
        bus.poll(|pkt, _reply| {
            out = match pkt {
                InstructionPacket::Ping(_) => Surfaced::Ping,
                InstructionPacket::SyncRead(_) => Surfaced::SyncRead,
                InstructionPacket::FastSyncRead(_) => Surfaced::FastSyncRead,
                _ => Surfaced::None,
            };
        });
        out
    }

    /// Stage `bytes` at offset 0 (auto-publishing write head) and poll.
    /// Returns whether anything surfaced.
    fn poll_after(bus: &mut TestBus, bytes: &[u8]) -> bool {
        bus.codec.stage_rx_bytes_for_test(0, bytes);
        let mut surfaced = false;
        bus.poll(|_pkt, _reply| {
            surfaced = true;
        });
        surfaced
    }

    #[test]
    fn on_rx_edge_advance_pulls_tpb_from_clock_at_3m() {
        let codec = make_codec_with_edges(
            &[1000, 1160],
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = make_bus_with(codec, BaudRate::B3000000);

        bus.on_rx_edge_advance();

        assert_eq!(bus.byte_ts_head().test_raw(), 2);
    }

    #[test]
    fn on_rx_edge_advance_pulls_tpb_from_clock_at_1m() {
        let codec = make_codec_with_edges(
            &[1000, 1160],
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = make_bus_with(codec, BaudRate::B1000000);

        bus.on_rx_edge_advance();

        // SEED at 1000, then SKIP (intra-byte) — head stays at 1.
        assert_eq!(bus.byte_ts_head().test_raw(), 1);
    }

    #[test]
    fn on_rx_idle_walks_tail_edges() {
        let codec = make_codec_with_edges(&[500], DmaFlags::default());
        let mut bus = make_bus_with(codec, BaudRate::B3000000);

        bus.on_rx_idle();

        assert_eq!(bus.byte_ts_head().test_raw(), 1);
    }

    #[test]
    fn stage_id_defers_until_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.stage_id(0x42));
        assert_eq!(bus.id, TEST_ID);
        let reboot = bus.on_tx_complete();
        assert_eq!(bus.id, 0x42);
        assert!(reboot.is_none());
    }

    #[test]
    fn stage_id_noop_when_unchanged() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.stage_id(TEST_ID));
        assert!(bus.pending_id.is_none());
    }

    #[test]
    fn stage_rdt_defers_until_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.stage_rdt(500));
        assert_eq!(bus.rdt_us, TEST_RDT_US);
        bus.on_tx_complete();
        assert_eq!(bus.rdt_us, 500);
    }

    #[test]
    fn stage_baud_forwards_to_clock() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.stage_baud(BaudRate::B1000000));
        // Still at 3M until the apply.
        assert_eq!(bus.clock.ticks_per_bit(), 16);
        bus.on_tx_complete();
        assert_eq!(bus.clock.ticks_per_bit(), 48);
    }

    #[test]
    fn stage_reboot_surfaces_through_on_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.stage_reboot(BootMode::Bootloader));
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
        InstructionEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .ping(Id::new(id))
            .unwrap();
        out
    }

    /// Emit a Status reply from `id` (the kind we never want to surface).
    fn wire_status(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        StatusEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .empty(Id::new(id), dxl_protocol::packet::StatusError::OK)
            .unwrap();
        out
    }

    /// Sync Read targets BROADCAST on the wire — exercises the
    /// id-as-BROADCAST branch with a richer (non-Ping) packet shape.
    fn wire_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .sync_read(addr, length, ids)
            .unwrap();
        out
    }

    /// Fast Sync Read — same broadcast addressing as Sync Read but the
    /// reply path goes through `send_slot` per the FastSlotInfo position.
    fn wire_fast_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8, 64> {
        let mut out: Vec<u8, 64> = Vec::new();
        InstructionEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .fast_sync_read(addr, length, ids)
            .unwrap();
        out
    }

    #[test]
    fn poll_returns_none_when_no_new_bytes() {
        let mut bus = make_bus();
        assert_eq!(poll_into(&mut bus), Surfaced::None);
        // Idempotent — calling again still drains nothing.
        assert_eq!(poll_into(&mut bus), Surfaced::None);
    }

    #[test]
    fn poll_surfaces_instruction_addressed_to_us() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        assert!(poll_after(&mut bus, &pkt));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_drops_instruction_for_other_id() {
        let mut bus = make_bus();
        let pkt = wire_ping(0x42);
        assert!(!poll_after(&mut bus, &pkt));
        // Still counted — foreign Instructions feed the drift signal.
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_surfaces_broadcast_instruction() {
        let mut bus = make_bus();
        let pkt = wire_sync_read(0x84, 4, &[0x01, 0x02, 0x03]);
        bus.codec.stage_rx_bytes_for_test(0, &pkt);
        assert_eq!(poll_into(&mut bus), Surfaced::SyncRead);
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_drops_status_silently() {
        let mut bus = make_bus();
        let pkt = wire_status(TEST_ID);
        assert!(!poll_after(&mut bus, &pkt));
        // Status frames never tick the drift signal — peer HSI is its own
        // clock domain (see [[drift_sampling_instruction_only]]).
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
        bus.codec.stage_rx_bytes_for_test(0, &combined);

        let surfaced = poll_into(&mut bus);
        assert_eq!(surfaced, Surfaced::Ping);
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_handles_ring_wrap() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        // Position the packet so it straddles the ring boundary.
        let start = (RX_BUF_LEN as u16).wrapping_sub(4);
        bus.codec.set_rx_read_seq_for_test(start);
        bus.codec.stage_rx_bytes_for_test(start, &pkt);

        assert_eq!(poll_into(&mut bus), Surfaced::Ping);
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_partial_packet_resumes_on_next_call() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);

        // Stage everything except the last byte — decoder is mid-CRC.
        let split = pkt.len() - 1;
        bus.codec.stage_rx_bytes_for_test(0, &pkt[..split]);
        assert_eq!(poll_into(&mut bus), Surfaced::None);
        assert_eq!(bus.instruction_count(), 0);

        // Stage the final byte (extending the producer head) — packet now
        // completes and surfaces.
        bus.codec
            .stage_rx_bytes_for_test(split as u16, &pkt[split..]);
        assert_eq!(poll_into(&mut bus), Surfaced::Ping);
        assert_eq!(bus.instruction_count(), 1);
    }

    /// Stage a stream of `n_bytes` edges spaced `step` ticks apart and run
    /// the classifier so BT[0..n_bytes] is populated before the test polls.
    fn bus_with_staged_bt(n_bytes: usize, step: u16) -> TestBus {
        let mut edges: Vec<u16, 32> = Vec::new();
        for i in 0..n_bytes as u16 {
            edges
                .push(1000_u16.wrapping_add(i.wrapping_mul(step)))
                .unwrap();
        }
        let codec = make_codec_with_edges(
            &edges,
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = make_bus_with(codec, BaudRate::B3000000);
        bus.on_rx_edge_advance();
        assert_eq!(bus.byte_ts_head().test_raw(), n_bytes as u16);
        bus
    }

    #[test]
    fn poll_feeds_one_drift_sample_per_bt_pair_on_instruction() {
        let ping = wire_ping(TEST_ID);
        let mut bus = bus_with_staged_bt(ping.len(), BYTE_TICKS_3M);
        bus.codec.stage_rx_bytes_for_test(0, &ping);

        assert_eq!(poll_into(&mut bus), Surfaced::Ping);

        // 10-byte packet → 9 in-window BT pairs → 9 samples to the integrator.
        assert_eq!(bus.clock.drift_samples(), (ping.len() - 1) as u16);
    }

    #[test]
    fn poll_feeds_drift_for_foreign_instruction() {
        let ping = wire_ping(0x42);
        let mut bus = bus_with_staged_bt(ping.len(), BYTE_TICKS_3M);
        bus.codec.stage_rx_bytes_for_test(0, &ping);

        assert_eq!(poll_into(&mut bus), Surfaced::None);
        assert_eq!(bus.clock.drift_samples(), (ping.len() - 1) as u16);
    }

    #[test]
    fn poll_does_not_feed_drift_for_status() {
        let status = wire_status(TEST_ID);
        let mut bus = bus_with_staged_bt(status.len(), BYTE_TICKS_3M);
        bus.codec.stage_rx_bytes_for_test(0, &status);

        assert_eq!(poll_into(&mut bus), Surfaced::None);
        assert_eq!(bus.clock.drift_samples(), 0);
    }

    #[test]
    fn poll_filters_out_of_window_bt_pairs() {
        let ping = wire_ping(TEST_ID);
        let mut bus = bus_with_staged_bt(ping.len(), 200);
        bus.codec.stage_rx_bytes_for_test(0, &ping);

        assert_eq!(poll_into(&mut bus), Surfaced::Ping);
        assert_eq!(bus.clock.drift_samples(), 0);
    }

    #[test]
    fn poll_drift_walk_anchors_at_first_consumed_byte_not_zero() {
        let ping = wire_ping(TEST_ID);
        let mut bus = bus_with_staged_bt(5 + ping.len(), BYTE_TICKS_3M);

        bus.codec.set_rx_read_seq_for_test(5);
        bus.codec.stage_rx_bytes_for_test(5, &ping);

        assert_eq!(poll_into(&mut bus), Surfaced::Ping);
        // Exactly (ping.len() - 1) samples, not 5 + (ping.len() - 1).
        assert_eq!(bus.clock.drift_samples(), (ping.len() - 1) as u16);
    }

    #[test]
    fn poll_drift_walk_resumes_across_resync() {
        let mut bad = wire_ping(TEST_ID);
        let crc_lo = bad.len() - 2;
        bad[crc_lo] ^= 0xFF;
        let good = wire_ping(TEST_ID);

        let total_bytes = bad.len() + good.len();
        let mut bus = bus_with_staged_bt(total_bytes, BYTE_TICKS_3M);

        let mut combined: Vec<u8, 32> = Vec::new();
        combined.extend_from_slice(&bad).unwrap();
        combined.extend_from_slice(&good).unwrap();
        bus.codec.stage_rx_bytes_for_test(0, &combined);

        assert_eq!(poll_into(&mut bus), Surfaced::Ping);
        // Only the good frame's pairs feed; the bad frame's bytes were
        // dropped through Resync, so its BT range never enters the walk.
        assert_eq!(bus.clock.drift_samples(), (good.len() - 1) as u16);
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

        // First call surfaces ours; decoder pauses at packet boundary.
        assert_eq!(poll_into(&mut bus), Surfaced::Ping);

        // Drain the rest — foreign Ping bumps the counter, Status doesn't.
        assert_eq!(poll_into(&mut bus), Surfaced::None);
        assert_eq!(bus.instruction_count(), 2);
    }

    // ------------------------------------------------------------------
    // TX fire scheduling
    // ------------------------------------------------------------------

    /// Construct a bus pre-loaded with `pkt`'s BT entries (so `wire_end_tick`
    /// has a real ts to look up) and the bytes themselves staged into the
    /// codec, ready for one `poll()`. Returns the bus and the expected
    /// wire-end tick the scheduler should see — `BT[pkt.len()-1] + 10·tpb`.
    fn bus_armed_with(pkt: &[u8]) -> (TestBus, u16) {
        let mut bus = bus_with_staged_bt(pkt.len(), BYTE_TICKS_3M);
        bus.codec.stage_rx_bytes_for_test(0, pkt);
        // tpb=16 @ 3M; first edge in `bus_with_staged_bt` is 1000 and they
        // step by BYTE_TICKS_3M, so BT[pkt.len()-1] == 1000 + (pkt.len()-1)·160.
        let last_bt = 1000_u16.wrapping_add(((pkt.len() as u16) - 1).wrapping_mul(BYTE_TICKS_3M));
        let wire_end_tick = last_bt.wrapping_add(16_u16.wrapping_mul(10));
        (bus, wire_end_tick)
    }

    fn empty_status() -> dxl_protocol::packet::Status<'static> {
        Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        }
    }

    /// Wire-end tick + delay_us → expected deadline_tick at V006's
    /// TICKS_PER_US=48 (matching the mock's const).
    fn expected_deadline(wire_end_tick: u16, delay_us: u32) -> u16 {
        wire_end_tick.wrapping_add(delay_us.wrapping_mul(48) as u16)
    }

    #[test]
    fn send_status_after_poll_schedules_at_wire_end_plus_rdt() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, wire_end_tick) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.send_status(empty_status()).expect("encode fits"));

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::Schedule {
                deadline_tick: expected_deadline(wire_end_tick, TEST_RDT_US),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_status_drops_silently_when_no_token() {
        // Foreign-ID Instruction → poll closure doesn't fire → scheduler
        // stays untouched.
        let ping = wire_ping(0x42);
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.send_status(empty_status()).expect("encode fits"));
        assert!(bus.scheduler.log.is_empty());
    }

    #[test]
    fn send_slot_only_schedules_plain() {
        // Fast Sync Read with our_id as the sole slot → SlotPosition::Only.
        let req = wire_fast_sync_read(0, 2, &[TEST_ID]);
        let (mut bus, wire_end_tick) = bus_armed_with(&req);

        let payload = [0x11_u8, 0x22];
        bus.poll(|_, reply| {
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
                deadline_tick: expected_deadline(wire_end_tick, TEST_RDT_US),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_slot_last_schedules_fast_last() {
        // Fast Sync Read with two slaves where we're the last → Last position.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, wire_end_tick) = bus_armed_with(&req);

        let payload = [0xAA_u8, 0xBB];
        // delay_q88 = (RDT << 8) + bytes_to_us_q88(12 bytes_before); ticks =
        // (delay_q88 × TICKS_PER_US) >> 8.
        let expected_delay_q88 = (TEST_RDT_US << 8) + bus.clock.bytes_to_us_q88(12);
        let expected_ticks = (expected_delay_q88.wrapping_mul(48)) >> 8;
        let expected = wire_end_tick.wrapping_add(expected_ticks as u16);

        bus.poll(|_, reply| {
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
    }

    #[test]
    fn send_slot_without_fast_context_drops_silently() {
        // Polled a Ping → no Fast slot position cached. send_slot must
        // silently no-op (dispatcher bug, not driver concern).
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_armed_with(&ping);

        let payload = [0x00_u8];
        bus.poll(|_, reply| {
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
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| reply.cancel());
        assert_eq!(bus.scheduler.log.as_slice(), &[ScheduleOp::Cancel]);
        assert!(bus.last_reply_ctx.is_none());
    }

    #[test]
    fn send_status_consumes_ctx_so_double_send_is_silent() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _) = bus_armed_with(&ping);
        bus.poll(|_, reply| {
            reply.send_status(empty_status()).expect("encode fits");
            // Second send within the same poll consumes nothing (ctx was
            // taken on the first); scheduler log shows just one entry.
            reply.send_status(empty_status()).expect("encode fits");
        });
        assert_eq!(bus.scheduler.log.len(), 1);
    }

    #[test]
    fn foreign_instruction_does_not_run_closure() {
        // poll() filters foreign-ID instructions; closure never fires →
        // scheduler stays untouched.
        let ping = wire_ping(0x42);
        let (mut bus, _) = bus_armed_with(&ping);
        let mut ran = false;
        bus.poll(|_, _| ran = true);
        assert!(!ran);
        assert!(bus.scheduler.log.is_empty());
    }

    #[test]
    fn broadcast_ping_folds_id_indexed_slot_offset() {
        // DXL 2.0 convention: each slave answers a broadcast Ping at offset
        // `id × PING_STATUS_FRAME_BYTES` so replies don't collide. Driver
        // computes this internally from the cached packet at poll time.
        let req = wire_ping(BROADCAST_ID);
        let (mut bus, wire_end_tick) = bus_armed_with(&req);

        let expected_offset_us = bus
            .clock
            .bytes_to_us(TEST_ID as u32 * PING_STATUS_FRAME_BYTES);
        bus.poll(|_, reply| reply.send_status(empty_status()).expect("encode fits"));

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::Schedule {
                deadline_tick: expected_deadline(wire_end_tick, TEST_RDT_US + expected_offset_us),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn sync_read_folds_bytes_before_for_later_slot() {
        // our_id (TEST_ID=0x07) at slot index 2 — `bytes_before` = 2 × per_slot.
        let req = wire_sync_read(0, 2, &[0x09, 0x05, TEST_ID]);
        let (mut bus, wire_end_tick) = bus_armed_with(&req);

        // per_slot = RESPONSE_HEADER_BYTES(9) + length(2) + CRC(2) = 13;
        // bytes_before = 2 × 13 = 26.
        let expected_offset_us = bus.clock.bytes_to_us(26);
        bus.poll(|_, reply| reply.send_status(empty_status()).expect("encode fits"));

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::Schedule {
                deadline_tick: expected_deadline(wire_end_tick, TEST_RDT_US + expected_offset_us),
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
        let (mut bus, _) = bus_armed_with(&ping);
        // Stage a pending id + rdt + reboot via a successful poll.
        bus.poll(|_, reply| {
            reply.stage_id(0x42);
            reply.stage_rdt(500);
            reply.stage_reboot(BootMode::Bootloader);
            reply.send_status(empty_status()).expect("encode fits");
        });
        // Clear the Schedule entry from the poll so the assertion below is
        // tight on the on_tx_complete-side log.
        bus.scheduler.log.clear();

        assert_eq!(bus.id, TEST_ID);
        assert_eq!(bus.rdt_us, TEST_RDT_US);

        let pending_reboot = bus.on_tx_complete();

        // Release runs first; pending config is then applied.
        assert_eq!(
            bus.scheduler.log.as_slice(),
            &[ScheduleOp::HandleTxComplete]
        );
        assert_eq!(bus.id, 0x42);
        assert_eq!(bus.rdt_us, 500);
        assert_eq!(pending_reboot, Some(BootMode::Bootloader));
    }
}
