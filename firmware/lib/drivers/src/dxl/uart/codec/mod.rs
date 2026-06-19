//! Bytes ↔ packets. Composite over two field-projection halves — `CodecRx`
//! (streaming parser + RX byte ring + classifier + drift bookkeeping) and
//! `CodecTx` (encoder + TX byte ring) — joined under `Codec`. The split lets
//! the parent driver hand a `&mut CodecTx` reply handle to the dispatcher
//! while a parser-event borrow lives in `CodecRx`.
//!
//! `CodecRx::poll` drives the streaming parser over the RX DMA ring and
//! fans each [`streaming::Event`] (along with a resolved contiguous ring
//! slice for `WriteDataChunk`) through a sink-style callback. The sink may
//! return [`PollAction::Skip`] in response to a Header event to engage the
//! universal byte-skip (doc §3 / §5.2) — codec records the body+CRC
//! distance from [`Parser::packet_remaining`], resets the parser, and
//! advances the RX ring tail past those bytes as they arrive. On
//! completion the sink sees [`PollEvent::SkipComplete`], at which point
//! the chain-fire predecessor-match check rides.

pub mod rx;

use core::cell::SyncUnsafeCell;
use core::marker::PhantomData;

use dxl_protocol::streaming::{Event, HeaderEvent, InstructionPayload, Parser, PayloadEvent};
use dxl_protocol::types::{Slot, Status};
use dxl_protocol::{CrcUmts, SlotEncoder, SlotPosition, StatusEncoder, WriteError};

use super::BITS_PER_FRAME;
use crate::traits::dxl::{EdgeDma, RxDma};
use crate::util::HwRing;
use rx::Rx;

/// Slack added past the byte-count-derived skip end, in wire bytes. Absorbs
/// inter-byte gaps and HSI wobble within healthy streams so a deadline-
/// bounded skip doesn't false-trigger on a slow-but-fine predecessor. 2
/// bytes ≈ 7 µs at 3 Mbaud — negligible vs. the host's own ~1 ms timeout,
/// tight enough that a truncated chain aborts well before the host's retry.
const SKIP_DEADLINE_SLACK_BYTES: u16 = 2;

/// Event surfaced from [`CodecRx::poll`] to its sink callback.
///
/// `Event { .. }` is a 1:1 forward of [`streaming::Event`]; the codec
/// translates `WriteDataChunk` `(offset, length)` into a contiguous ring
/// slice via `ring` (empty for other events). `next_status_pos` is the
/// codec's wire-byte position at this event's emit point — the value
/// `wire_bytes_consumed` will hold after the parser has consumed the
/// bytes that produced this event. Named for its load-bearing use: the
/// Fast Last fold path captures it at the chain instruction's Crc event
/// as `fold_start_cursor` — at that point it equals the wire position
/// where the First predecessor's status packet will start (the next byte
/// on the wire). At Header / Payload events the value is still the
/// codec's running cursor, but no consumer reads it there today.
/// `SkipComplete` fires when the universal byte-skip's remaining-byte
/// counter hits zero — `id` round-trips the value the sink passed in
/// [`PollAction::Skip`], so the chain-fire check (doc §5.2) can compare
/// against `predecessor_id`.
pub enum PollEvent<'a> {
    Event {
        ev: Event,
        ring: &'a [u8],
        next_status_pos: u32,
    },
    SkipComplete {
        id: u8,
    },
}

/// Return value from the sink callback.
///
/// `Skip` is meaningful only after [`PollEvent::Event`] carrying a
/// [`Event::Header`]; on other events the codec ignores it and continues
/// normally. On `Skip`, the codec reads [`Parser::packet_remaining`],
/// resets the parser, and consumes the indicated count from the RX ring
/// tail before surfacing [`PollEvent::SkipComplete`].
pub enum PollAction {
    Continue,
    Skip { id: u8 },
}

struct SkipState {
    bytes_remaining: u16,
    id: u8,
    /// WireClock u32 tick at which the skip gives up and clears itself, so
    /// a truncated upstream packet can't bleed its uncounted bytes into
    /// the next packet on the wire. Set at skip entry to
    /// `now + (bytes_remaining + SKIP_DEADLINE_SLACK_BYTES) * frame_ticks`.
    /// Compared with `(now.wrapping_sub(deadline_tick) as i32) >= 0` —
    /// u32 modular signed-comparison works for any elapsed budget under
    /// 2³¹ ticks (~44 s at HCLK), comfortably above the slowest baud's
    /// longest packet.
    deadline_tick: u32,
}

/// RX half — streaming parser, RX byte ring, classifier, drift bookkeeping.
/// Splits off from [`CodecTx`] under [`Codec`] so the parent driver can hand
/// the dispatcher a `&mut CodecTx` reply handle alongside the parser event
/// stream the dispatcher is consuming.
pub struct CodecRx<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize> {
    rx: Rx<R, EDGE_BUF_LEN>,
    parser: Parser<CRC>,
    /// DMA1_CH5 destination for received bytes. `SyncUnsafeCell` because
    /// USART1's DMA writes it concurrently with the parser's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware. [`HwRing`] enforces pow-2 sizing so
    /// `% RX_BUF_LEN` collapses to AND, and the chip-side ISR publishes
    /// the producer head via [`HwRing::on_publish`] from NDTR.
    rx_buf: SyncUnsafeCell<HwRing<u8, RX_BUF_LEN>>,
    /// Monotonic count of Instruction headers the parser emitted — own
    /// or foreign IDs — across this codec's lifetime. Drift sampling
    /// ([[drift_sampling_instruction_only]]) ticks on every Instruction
    /// so foreign-target instructions still calibrate, while Status
    /// frames never contribute (peer HSI is its own clock domain).
    instruction_count: u32,
    /// Monotonic count of RX-ring bytes the codec has advanced past
    /// across this codec's lifetime — covers both parser-consumed bytes
    /// and byte-skip-consumed bytes. 32 bits ≈ 23.8 days at 3M sustained;
    /// wrap is non-physical.
    wire_bytes_consumed: u32,
    /// Universal byte-skip state. `Some` between a sink-requested
    /// [`PollAction::Skip`] and the matching [`PollEvent::SkipComplete`].
    skip: Option<SkipState>,
}

impl<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize>
    CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>
{
    fn new(ring: R) -> Self {
        Self {
            rx: Rx::new(ring),
            parser: Parser::new(),
            rx_buf: SyncUnsafeCell::new(HwRing::new(0)),
            instruction_count: 0,
            wire_bytes_consumed: 0,
            skip: None,
        }
    }

    /// New RX falling-edge timestamps may be available — forward
    /// `ticks_per_bit` (pulled from the clock by the composite) into the
    /// RX classifier so its HIT window matches the current baud. Drift
    /// pairs route through `on_pair` when the classifier's `hsi_active`
    /// flag is set; otherwise it's never called.
    pub fn on_edge_advance<F: FnMut(u16, u16)>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        on_pair: F,
    ) {
        self.rx.on_edge_advance(now, ticks_per_bit, on_pair);
    }

    /// USART1 IDLE backstop — drain tail edges through `on_pair`, then
    /// reset the classifier anchor and drift flag for the next burst.
    pub fn on_idle<F: FnMut(u16, u16)>(&mut self, now: u32, ticks_per_bit: u16, on_pair: F) {
        self.rx.on_idle(now, ticks_per_bit, on_pair);
    }

    /// USART1 RX DMA published progress — `remaining` is the channel's
    /// NDTR readback. Advances the `rx_buf` producer head so `poll` sees
    /// newly-DMA'd bytes.
    pub fn on_rx_dma_advance(&mut self, remaining: u16) {
        // SAFETY: rx_buf is written only by DMA1_CH5 (hardware writer)
        // and read here from the same PFIC priority level as the DMA
        // HT/TC ISR, so no other consumer can `&mut` it concurrently.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        rx_buf.on_publish(remaining);
    }

    /// Drain the RX byte ring through the streaming parser, fanning each
    /// event (or skip-complete pseudo-event) to `on_event`. Reads the
    /// front (pre-wrap) slice first, then the back (post-wrap) slice if
    /// any; advances the ring tail by what the parser consumed. The sink
    /// decides per-Header whether to forward the body or universal-byte-
    /// skip past it (doc §3 driver rule). The skip is observable as
    /// [`PollEvent::SkipComplete`] when its counter hits zero — the
    /// chain-fire predecessor-match check (doc §5.2) rides there.
    ///
    /// `instruction_count` ticks on every emitted Instruction Header
    /// regardless of subsequent Skip; Status frames never tick. The
    /// `wire_bytes_consumed` cursor advances over both parser- and
    /// skip-consumed bytes.
    ///
    /// `now` and `ticks_per_bit` are read on entry to bound the universal
    /// byte-skip: skip entry stamps a `deadline_tick` from the expected
    /// byte count plus `SKIP_DEADLINE_SLACK_BYTES`, and each `poll()`
    /// drops a stale skip whose deadline has passed. The deadline path
    /// suppresses `SkipComplete` — it's a truncation recovery, not a
    /// successful drain, so chain-fire's predecessor-match must not ride.
    pub fn poll<F>(&mut self, now: u32, ticks_per_bit: u16, mut on_event: F)
    where
        F: FnMut(PollEvent<'_>, &mut Rx<R, EDGE_BUF_LEN>) -> PollAction,
    {
        // SAFETY: rx_buf lives in a SyncUnsafeCell; this is the codec's
        // single consumer path. The reference is independent of any
        // borrow on `self` for the parser / counters because it's
        // constructed via a raw pointer (`SyncUnsafeCell::get` takes
        // `&self`, returns `*mut T`; the &mut deref is fresh).
        let rx_buf_ptr = self.rx_buf.get();
        loop {
            // Skip phase: drain ring up to bytes_remaining; emit
            // SkipComplete on exhaust. Deadline check rides first so a
            // truncated upstream packet can't leak its uncounted bytes
            // into the next packet — once the expected duration has
            // elapsed the ring's contents belong to whatever came next.
            if let Some(skip) = self.skip.as_mut() {
                if (now.wrapping_sub(skip.deadline_tick) as i32) >= 0 {
                    self.skip = None;
                    continue;
                }
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                let avail = rx_buf.reader().avail();
                let take = skip.bytes_remaining.min(avail);
                if take > 0 {
                    rx_buf.reader().advance(take);
                    skip.bytes_remaining -= take;
                    self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(take as u32);
                }
                if skip.bytes_remaining > 0 {
                    // Ring exhausted mid-skip; resume on a later poll.
                    return;
                }
                let id = skip.id;
                self.skip = None;
                on_event(PollEvent::SkipComplete { id }, &mut self.rx);
                continue;
            }

            // Feed phase: extract a contiguous front slice from the
            // ring; feed to the parser; advance by what was consumed.
            let (input_ptr, input_len) = {
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                let mut reader = rx_buf.reader();
                let (front, _back) = reader.peek_slices();
                (front.as_ptr(), front.len())
            };
            if input_len == 0 {
                return;
            }
            // SAFETY: `input_ptr` points into `rx_buf.data`, owned by
            // self for the lifetime of this poll. The data is not
            // mutated until we call `reader.advance(consumed)` after
            // the parser loop. The parser only reads; on_event has no
            // access path back to rx_buf.
            let input: &[u8] = unsafe { core::slice::from_raw_parts(input_ptr, input_len) };

            let mut break_for_skip: Option<u8> = None;
            let consumed = {
                let mut stream = self.parser.feed(input);
                // `while let` rather than `for ... by_ref()` so each iteration
                // releases the `&mut stream` borrow before we read
                // `stream.consumed()` to compute the per-event wire position.
                while let Some(ev) = stream.next() {
                    let ring: &[u8] = if let Event::Payload(PayloadEvent::Instruction(
                        InstructionPayload::WriteDataChunk { offset, length },
                    )) = ev
                    {
                        &input[offset as usize..(offset as usize + length as usize)]
                    } else {
                        &[]
                    };
                    if matches!(ev, Event::Header(HeaderEvent::Instruction(_))) {
                        self.instruction_count = self.instruction_count.wrapping_add(1);
                    }
                    let next_status_pos = self
                        .wire_bytes_consumed
                        .wrapping_add(stream.consumed() as u32);
                    match on_event(
                        PollEvent::Event {
                            ev,
                            ring,
                            next_status_pos,
                        },
                        &mut self.rx,
                    ) {
                        PollAction::Continue => {}
                        PollAction::Skip { id } => {
                            break_for_skip = Some(id);
                            break;
                        }
                    }
                }
                stream.consumed()
            };

            if consumed > 0 {
                self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(consumed as u32);
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                rx_buf.reader().advance(consumed as u16);
            }

            if let Some(id) = break_for_skip {
                let bytes_remaining = self.parser.packet_remaining();
                self.parser.reset();
                let frame_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
                let budget_bytes = bytes_remaining.saturating_add(SKIP_DEADLINE_SLACK_BYTES);
                let elapsed = (budget_bytes as u32).wrapping_mul(frame_ticks);
                let deadline_tick = now.wrapping_add(elapsed);
                self.skip = Some(SkipState {
                    bytes_remaining,
                    id,
                    deadline_tick,
                });
                continue;
            }

            if consumed == 0 {
                // Parser made no progress on the front slice — either
                // it's idling at end-of-input or the slice was empty.
                // Try the back slice on next iteration via peek_slices.
                // If the front slice was non-empty and consumed is 0,
                // we'd loop forever — short-circuit.
                return;
            }
        }
    }

    /// Drain newly-published RX bytes through `fold_byte` without invoking
    /// the parser. Used by the Fast Last fold path during the predecessor
    /// window: the SysTick CMP body and the CC3 post-fire body each spin
    /// inside this drain, refreshing the producer head from
    /// `rx_dma.remaining()` on every pass so newly-DMA'd bytes become
    /// reader-visible mid-loop without re-entering the chip-side ISR.
    /// Advances `wire_bytes_consumed` over each drained byte so a
    /// subsequent `poll()` resumes at the right cursor.
    ///
    /// The chip-side caller masks DMA1_CH7 HT/TC for the duration of the
    /// Fast Last window (doc §10.6.3); during that window `poll()` does
    /// not run, so the parser and this drain never race on `rx_buf`.
    pub fn drain_raw<D: RxDma, F: FnMut(u8, u32)>(&mut self, rx_dma: &D, mut fold_byte: F) {
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
        // The chip-side caller masks DMA1_CH7 HT/TC for the Fast Last window
        // so the parser and this drain never race on the ring.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        loop {
            rx_buf.on_publish(rx_dma.remaining());
            let n = {
                let mut reader = rx_buf.reader();
                let (front, _back) = reader.peek_slices();
                if front.is_empty() {
                    return;
                }
                for &b in front {
                    fold_byte(b, self.wire_bytes_consumed);
                    self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(1);
                }
                front.len() as u16
            };
            rx_buf.reader().advance(n);
        }
    }

    /// Clear any in-flight universal byte-skip. Called by the composite at
    /// the chip's own TX-complete event (doc §5.3): once our chain
    /// participation is over, a skip that's still tracking the chain
    /// envelope is stale, and at slow baud the deadline-bounded skip
    /// (`SKIP_DEADLINE_SLACK_BYTES`) can outlive the inter-packet gap and
    /// eat the next preamble. On non-chain TX (Plain reply) the skip is
    /// already `None`, so the clear is a no-op.
    pub fn cancel_skip(&mut self) {
        self.skip = None;
    }

    /// Monotonic count of Instruction headers the parser has emitted.
    /// Foreign IDs count too (sink filters at its layer); Status frames
    /// don't.
    pub fn instruction_count(&self) -> u32 {
        self.instruction_count
    }

    /// Stable peripheral-memory address for DMA1_CH7's destination buffer.
    /// Bringup hands this to `dma::configure(CH7, ...)`.
    pub fn edges_addr(&self) -> usize {
        self.rx.edges_addr()
    }

    /// Stable peripheral-memory address for DMA1_CH5's destination buffer.
    /// Bringup hands this to `dma::configure(CH5, ...)` so the USART byte
    /// stream lands directly in driver-owned storage. [`HwRing::as_ptr`]
    /// returns the address of the first storage slot — the struct's outer
    /// address is offset by the bookkeeping fields.
    pub fn rx_buf_addr(&self) -> usize {
        // SAFETY: address-of read; no value materialized. Sound even while
        // DMA is writing the storage concurrently.
        unsafe { (*self.rx_buf.get()).as_ptr() as usize }
    }

    /// Forward to [`rx::Rx::pause_edges`].
    pub fn pause_edges(&mut self) {
        self.rx.pause_edges();
    }

    /// Forward to [`rx::Rx::resume_edges`].
    pub fn resume_edges(&mut self) {
        self.rx.resume_edges();
    }

    /// Forward to [`rx::Rx::try_anchor_from_header`].
    pub fn try_anchor_from_header(&mut self, ticks_per_bit: u16) -> bool {
        self.rx.try_anchor_from_header(ticks_per_bit)
    }

    /// Forward to [`rx::Rx::current_byte_tick`].
    pub fn current_byte_tick(&self, now: u32) -> Option<u32> {
        self.rx.current_byte_tick(now)
    }

    /// Forward to [`rx::Rx::packet_end_tick`].
    pub fn packet_end_tick(&self, ticks_per_bit: u16, now: u32) -> Option<u32> {
        self.rx.packet_end_tick(ticks_per_bit, now)
    }

    /// Forward to [`rx::Rx::drain_walker`].
    pub fn drain_walker<F: FnMut(u16, u16)>(&mut self, ticks_per_bit: u16, on_pair: F) {
        self.rx.drain_walker(ticks_per_bit, on_pair);
    }

    /// Forward to [`rx::Rx::set_hsi_active`].
    pub fn set_hsi_active(&mut self, on: bool) {
        self.rx.set_hsi_active(on);
    }

    /// Forward to [`rx::Rx::reset_anchor`].
    pub fn reset_anchor(&mut self) {
        self.rx.reset_anchor();
    }
}

/// TX half — encoder + TX byte ring. Splits off from [`CodecRx`] under
/// [`Codec`] so the parent driver's split-borrow `poll` can hand the
/// dispatcher's reply handle a `&mut CodecTx` alongside a parser event
/// stream that borrows the disjoint RX half.
pub struct CodecTx<CRC: CrcUmts, const TX_BUF_LEN: usize> {
    /// DMA1_CH4 source for transmitted bytes. Single-shot DMA per fire:
    /// the encoder methods ([`Self::send_status`], [`Self::send_slot`])
    /// fill it from offset 0; bringup hands [`Self::tx_buf_addr`] to
    /// `dma::configure(CH4, ...)` once. Producer (encoder) and consumer
    /// (DMA shift-out) phases are exclusive — the composite holds the only
    /// `&mut CodecTx` and stops writing once it routes to the scheduler —
    /// so no `SyncUnsafeCell`.
    tx_buf: heapless::Vec<u8, TX_BUF_LEN>,
    _crc: PhantomData<CRC>,
}

impl<CRC: CrcUmts, const TX_BUF_LEN: usize> CodecTx<CRC, TX_BUF_LEN> {
    fn new() -> Self {
        Self {
            tx_buf: heapless::Vec::new(),
            _crc: PhantomData,
        }
    }

    /// Encode a Status reply into the TX buffer. Clears any previous
    /// contents first, then drives [`StatusEncoder`] over `tx_buf` —
    /// `Vec::push` propagates `WriteError::Overflow` if the encoded form
    /// exceeds `TX_BUF_LEN`. The composite reads [`Self::tx_len`] after
    /// for the DMA transfer count.
    pub fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        self.tx_buf.clear();
        StatusEncoder::<_, CRC>::new(&mut self.tx_buf).emit(status)
    }

    /// Encode one Fast slot reply into the TX buffer. Same buffer-clear
    /// and emitter shape as [`Self::send_status`]; [`SlotEncoder::emit`]
    /// dispatches on `position` (Only/First/Middle/Last) and writes the
    /// header, payload, and (locally-computed or caller-supplied) CRC.
    pub fn send_slot(&mut self, slot: &Slot<'_>, position: SlotPosition) -> Result<(), WriteError> {
        self.tx_buf.clear();
        SlotEncoder::<_, CRC>::new(&mut self.tx_buf).emit(slot, position)
    }

    /// Stable peripheral-memory address for DMA1_CH4's source buffer.
    /// Bringup hands this to `dma::configure(CH4, ...)` once; per-fire
    /// arm reads [`Self::tx_len`] for the transfer count.
    pub fn tx_buf_addr(&self) -> usize {
        self.tx_buf.as_ptr() as usize
    }

    /// Length in bytes of the most-recent encoded packet — the DMA1_CH4
    /// transfer count for the next fire. Zero until the first send.
    pub fn tx_len(&self) -> u16 {
        self.tx_buf.len() as u16
    }

    /// Overwrite the trailing 2 bytes of the encoded TX buffer with `crc`
    /// in little-endian. The Fast Last chain-CRC fold path calls this once
    /// the predecessor wire bytes have been folded and our own reply bytes
    /// are mixed in: the encoder emitted a placeholder CRC at
    /// `send_slot(Last)` time and this patches it before DMA1_CH4's read
    /// cursor reaches the trailing slot (doc §10.6). No-op when `tx_buf`
    /// hasn't been encoded yet (length < 2).
    pub fn patch_crc(&mut self, crc: u16) {
        let n = self.tx_buf.len();
        if n < 2 {
            return;
        }
        self.tx_buf[n - 2] = (crc & 0xFF) as u8;
        self.tx_buf[n - 1] = (crc >> 8) as u8;
    }

    /// Slice of our own reply bytes excluding the trailing 2-byte CRC
    /// slot — the bytes the Fast Last fold mixes into the chain CRC
    /// before patching. Empty when no reply has been encoded yet.
    pub fn own_reply_bytes(&self) -> &[u8] {
        let n = self.tx_buf.len();
        if n < 2 {
            return &[];
        }
        &self.tx_buf[..n - 2]
    }
}

pub struct Codec<
    R: EdgeDma,
    CRC: CrcUmts,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> {
    pub(super) rx: CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>,
    pub(super) tx: CodecTx<CRC, TX_BUF_LEN>,
}

impl<
    R: EdgeDma,
    CRC: CrcUmts,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> Codec<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    pub fn new(ring: R) -> Self {
        Self {
            rx: CodecRx::new(ring),
            tx: CodecTx::new(),
        }
    }

    /// Disjoint mutable borrow of the RX and TX halves. The parent driver's
    /// `poll` uses this to hand the dispatcher a `&mut CodecTx` reply handle
    /// while a parser event borrow lives in `&mut CodecRx`.
    pub fn split_mut(
        &mut self,
    ) -> (
        &mut CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>,
        &mut CodecTx<CRC, TX_BUF_LEN>,
    ) {
        (&mut self.rx, &mut self.tx)
    }

    // ----- Forwarders. Keep sequential single-half call sites compact
    // without forcing every caller to disambiguate `.rx` / `.tx`. -----

    pub fn on_edge_advance<F: FnMut(u16, u16)>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        on_pair: F,
    ) {
        self.rx.on_edge_advance(now, ticks_per_bit, on_pair);
    }

    pub fn on_idle<F: FnMut(u16, u16)>(&mut self, now: u32, ticks_per_bit: u16, on_pair: F) {
        self.rx.on_idle(now, ticks_per_bit, on_pair);
    }

    pub fn on_rx_dma_advance(&mut self, remaining: u16) {
        self.rx.on_rx_dma_advance(remaining);
    }

    pub fn poll<F>(&mut self, now: u32, ticks_per_bit: u16, on_event: F)
    where
        F: FnMut(PollEvent<'_>, &mut Rx<R, EDGE_BUF_LEN>) -> PollAction,
    {
        self.rx.poll(now, ticks_per_bit, on_event);
    }

    pub fn cancel_skip(&mut self) {
        self.rx.cancel_skip();
    }

    pub fn instruction_count(&self) -> u32 {
        self.rx.instruction_count()
    }

    pub fn edges_addr(&self) -> usize {
        self.rx.edges_addr()
    }

    pub fn rx_buf_addr(&self) -> usize {
        self.rx.rx_buf_addr()
    }

    pub fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        self.tx.send_status(status)
    }

    pub fn send_slot(&mut self, slot: &Slot<'_>, position: SlotPosition) -> Result<(), WriteError> {
        self.tx.send_slot(slot, position)
    }

    pub fn tx_buf_addr(&self) -> usize {
        self.tx.tx_buf_addr()
    }

    pub fn tx_len(&self) -> u16 {
        self.tx.tx_len()
    }

    pub fn patch_crc(&mut self, crc: u16) {
        self.tx.patch_crc(crc);
    }

    pub fn own_reply_bytes(&self) -> &[u8] {
        self.tx.own_reply_bytes()
    }

    pub fn pause_edges(&mut self) {
        self.rx.pause_edges();
    }

    pub fn resume_edges(&mut self) {
        self.rx.resume_edges();
    }

    /// Forward to [`CodecRx::try_anchor_from_header`].
    pub fn try_anchor_from_header(&mut self, ticks_per_bit: u16) -> bool {
        self.rx.try_anchor_from_header(ticks_per_bit)
    }

    /// Forward to [`CodecRx::current_byte_tick`].
    pub fn current_byte_tick(&self, now: u32) -> Option<u32> {
        self.rx.current_byte_tick(now)
    }

    /// Forward to [`CodecRx::packet_end_tick`].
    pub fn packet_end_tick(&self, ticks_per_bit: u16, now: u32) -> Option<u32> {
        self.rx.packet_end_tick(ticks_per_bit, now)
    }

    /// Forward to [`CodecRx::set_hsi_active`].
    pub fn set_hsi_active(&mut self, on: bool) {
        self.rx.set_hsi_active(on);
    }

    /// Forward to [`CodecRx::reset_anchor`].
    pub fn reset_anchor(&mut self) {
        self.rx.reset_anchor();
    }
}

// Shelved pending U4 (osc-drivers unit test audit): helpers below are only
// reached from the shelved tests; revive together with them.
#[cfg(any())]
#[cfg(test)]
impl<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize>
    CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>
{
    /// Stage `bytes` into `rx_buf` starting at sequence `at` and publish
    /// the producer head to `at + bytes.len()`. Mirrors the chip-side
    /// DMA1_CH5 writer for host tests.
    pub(crate) fn stage_rx_bytes_for_test(&mut self, at: u16, bytes: &[u8]) {
        // SAFETY: test-only access; no DMA in tests.
        let buf = unsafe { &mut *self.rx_buf.get() };
        buf.stage(at, bytes);
        buf.set_write_seq_for_test(at.wrapping_add(bytes.len() as u16));
    }

    /// Pre-position the parser cursor (wrap-test seed). Production has no
    /// reason to set this directly — it's monotonic from `new()` onwards.
    pub(crate) fn set_rx_read_seq_for_test(&mut self, seq: u16) {
        // SAFETY: test-only access; no DMA in tests.
        let buf = unsafe { &mut *self.rx_buf.get() };
        buf.set_read_seq_for_test(seq);
    }

    /// Force the classifier's `last_byte_start` to a known tick. Composite
    /// tests use this to seed `packet_end_tick` without staging real edges.
    pub(crate) fn force_byte_tick_for_test(&mut self, tick: u16) {
        self.rx.force_byte_tick_for_test(tick);
    }

    /// Read the classifier's `hsi_active` flag for composite tests.
    pub(crate) fn rx_classifier_hsi_active_for_test(&self) -> bool {
        self.rx.hsi_active()
    }

    /// Cumulative parser- + skip-consumed wire-byte cursor. Production
    /// callers read this per-event via `PollEvent::Event::next_status_pos`;
    /// tests reach the underlying counter directly to verify the cursor
    /// walks both parser and skip paths.
    pub(crate) fn wire_byte_cursor_for_test(&self) -> u32 {
        self.wire_bytes_consumed
    }
}

// Shelved pending U4 (osc-drivers unit test audit): helpers below are only
// reached from the shelved tests; revive together with them.
#[cfg(any())]
#[cfg(test)]
impl<
    R: EdgeDma,
    CRC: CrcUmts,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> Codec<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    pub(crate) fn stage_rx_bytes_for_test(&mut self, at: u16, bytes: &[u8]) {
        self.rx.stage_rx_bytes_for_test(at, bytes);
    }

    pub(crate) fn set_rx_read_seq_for_test(&mut self, seq: u16) {
        self.rx.set_rx_read_seq_for_test(seq);
    }

    pub(crate) fn force_byte_tick_for_test(&mut self, tick: u16) {
        self.rx.force_byte_tick_for_test(tick);
    }

    pub(crate) fn rx_classifier_hsi_active_for_test(&self) -> bool {
        self.rx.rx_classifier_hsi_active_for_test()
    }

    pub(crate) fn wire_byte_cursor_for_test(&self) -> u32 {
        self.rx.wire_byte_cursor_for_test()
    }
}

// Shelved pending U4 (osc-drivers unit test audit): tests below bind to
// hand-rolled mock fields; will be migrated to the mockall + state-companion
// API as part of the audit.
#[cfg(any())]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::MockEdgeDma;
    use dxl_protocol::streaming::{HeaderEvent, InstructionHeader, ResyncKind};
    use dxl_protocol::types::{Id, StatusError};
    use dxl_protocol::{InstructionEncoder, SoftwareCrcUmts, StatusEncoder};
    use heapless::Vec;

    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;
    /// `DXL_TX_MAX_BYTES` per `osc-core::services::dxl::limits` — chip-side
    /// registry uses the same value.
    const TX_BUF_LEN: usize = 140;
    const TEST_ID: u8 = 0x07;
    const FOREIGN_ID: u8 = 0x42;

    type TestCodec = Codec<MockEdgeDma, SoftwareCrcUmts, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

    fn make() -> TestCodec {
        Codec::new(MockEdgeDma::default())
    }

    fn wire_ping(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .ping(Id::new(id))
            .unwrap();
        out
    }

    fn wire_write(id: u8, addr: u16, body: &[u8]) -> Vec<u8, 64> {
        let mut out: Vec<u8, 64> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .write(Id::new(id), addr, body)
            .unwrap();
        out
    }

    fn wire_status(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        StatusEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .empty(Id::new(id), StatusError::OK)
            .unwrap();
        out
    }

    /// Drain a codec into a `Vec` of captured events. `decide` controls
    /// per-Header skip behavior; default `|_| PollAction::Continue` forwards
    /// everything.
    fn collect_events<F>(c: &mut TestCodec, mut decide: F) -> alloc::vec::Vec<Capture>
    where
        F: FnMut(&Event) -> PollAction,
    {
        let mut out = alloc::vec::Vec::new();
        c.poll(|pe, _rx| match pe {
            PollEvent::Event { ev, ring, .. } => {
                let action = if matches!(ev, Event::Header(_)) {
                    decide(&ev)
                } else {
                    PollAction::Continue
                };
                out.push(Capture::Event {
                    ev,
                    ring: ring.into(),
                });
                action
            }
            PollEvent::SkipComplete { id } => {
                out.push(Capture::SkipComplete { id });
                PollAction::Continue
            }
        });
        out
    }

    #[derive(Debug)]
    enum Capture {
        Event {
            ev: Event,
            ring: alloc::vec::Vec<u8>,
        },
        SkipComplete {
            id: u8,
        },
    }

    extern crate alloc;

    #[test]
    fn poll_no_op_when_ring_empty() {
        let mut c = make();
        let captures = collect_events(&mut c, |_| PollAction::Continue);
        assert!(captures.is_empty());
        assert_eq!(c.instruction_count(), 0);
        assert_eq!(c.wire_byte_cursor_for_test(), 0);
    }

    #[test]
    fn poll_emits_sync_header_chunk_crc_for_own_write() {
        let mut c = make();
        let wire = wire_write(TEST_ID, 0x0050, &[0xAA, 0xBB, 0xCC, 0xDD]);
        c.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut c, |_| PollAction::Continue);

        // Walk: Sync, Header(Write), Payload(WriteDataChunk), Crc.
        let mut iter = captures.iter();
        match iter.next() {
            Some(Capture::Event {
                ev: Event::Sync, ..
            }) => {}
            other => panic!("expected Sync, got {other:?}"),
        }
        match iter.next() {
            Some(Capture::Event {
                ev:
                    Event::Header(HeaderEvent::Instruction(InstructionHeader::Write {
                        id,
                        address,
                        length,
                    })),
                ..
            }) => {
                assert_eq!(*id, Id::new(TEST_ID));
                assert_eq!(*address, 0x0050);
                assert_eq!(*length, 4);
            }
            other => panic!("expected Header(Write), got {other:?}"),
        }
        match iter.next() {
            Some(Capture::Event {
                ev:
                    Event::Payload(PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                        length,
                        ..
                    })),
                ring,
            }) => {
                assert_eq!(*length, 4);
                assert_eq!(ring.as_slice(), &[0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected WriteDataChunk, got {other:?}"),
        }
        match iter.next() {
            Some(Capture::Event { ev: Event::Crc, .. }) => {}
            other => panic!("expected Crc, got {other:?}"),
        }
        assert!(iter.next().is_none());
    }

    #[test]
    fn poll_skip_consumes_foreign_ping_body_and_crc() {
        let mut c = make();
        let wire = wire_ping(FOREIGN_ID);
        c.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut c, |ev| match ev {
            Event::Header(HeaderEvent::Instruction(_)) => PollAction::Skip { id: FOREIGN_ID },
            _ => PollAction::Continue,
        });

        // Sync, Header, SkipComplete. The skip distance is body(0) + CRC(2).
        let mut iter = captures.iter();
        assert!(matches!(
            iter.next(),
            Some(Capture::Event {
                ev: Event::Sync,
                ..
            })
        ));
        assert!(matches!(
            iter.next(),
            Some(Capture::Event {
                ev: Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { .. })),
                ..
            })
        ));
        assert!(matches!(
            iter.next(),
            Some(Capture::SkipComplete { id }) if *id == FOREIGN_ID
        ));
        assert!(iter.next().is_none());
        // Wire cursor advanced over the full packet.
        assert_eq!(c.wire_byte_cursor_for_test() as usize, wire.len());
    }

    #[test]
    fn poll_skip_consumes_foreign_write_body_and_crc() {
        let mut c = make();
        let wire = wire_write(FOREIGN_ID, 0x0050, &[1, 2, 3, 4]);
        c.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut c, |ev| match ev {
            Event::Header(HeaderEvent::Instruction(_)) => PollAction::Skip { id: FOREIGN_ID },
            _ => PollAction::Continue,
        });

        // No Payload / Crc events past Header — the parser was reset.
        let payloads = captures
            .iter()
            .filter(|c| {
                matches!(
                    c,
                    Capture::Event {
                        ev: Event::Payload(_),
                        ..
                    }
                )
            })
            .count();
        assert_eq!(payloads, 0);
        let crcs = captures
            .iter()
            .filter(|c| matches!(c, Capture::Event { ev: Event::Crc, .. }))
            .count();
        assert_eq!(crcs, 0);
        assert!(
            captures
                .iter()
                .any(|c| matches!(c, Capture::SkipComplete { id } if *id == FOREIGN_ID))
        );
        assert_eq!(c.wire_byte_cursor_for_test() as usize, wire.len());
    }

    #[test]
    fn poll_skip_complete_carries_id() {
        let mut c = make();
        let wire = wire_ping(FOREIGN_ID);
        c.stage_rx_bytes_for_test(0, &wire);

        let custom = 0x33u8;
        let captures = collect_events(&mut c, |_| PollAction::Skip { id: custom });
        let id = captures.iter().find_map(|c| match c {
            Capture::SkipComplete { id } => Some(*id),
            _ => None,
        });
        assert_eq!(id, Some(custom));
    }

    #[test]
    fn poll_skip_then_own_packet_parses_clean() {
        let mut c = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_write(TEST_ID, 0x0060, &[0x11, 0x22]);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        c.stage_rx_bytes_for_test(0, &combined);

        let captures = collect_events(&mut c, |ev| match ev {
            Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { id }))
                if id.as_byte() == FOREIGN_ID =>
            {
                PollAction::Skip { id: FOREIGN_ID }
            }
            _ => PollAction::Continue,
        });

        let skip_done = captures
            .iter()
            .any(|c| matches!(c, Capture::SkipComplete { id } if *id == FOREIGN_ID));
        assert!(skip_done);
        let saw_write = captures.iter().any(|c| {
            matches!(
                c,
                Capture::Event {
                    ev: Event::Header(HeaderEvent::Instruction(InstructionHeader::Write { .. })),
                    ..
                }
            )
        });
        assert!(saw_write);
        let saw_chunk = captures.iter().any(|c| {
            matches!(
                c,
                Capture::Event {
                    ev: Event::Payload(PayloadEvent::Instruction(
                        InstructionPayload::WriteDataChunk { .. }
                    )),
                    ring,
                } if ring.as_slice() == [0x11, 0x22]
            )
        });
        assert!(saw_chunk);
    }

    #[test]
    fn wire_bytes_consumed_counts_parser_and_skip_bytes() {
        let mut c = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_ping(TEST_ID);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        c.stage_rx_bytes_for_test(0, &combined);

        let _ = collect_events(&mut c, |ev| match ev {
            Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { id }))
                if id.as_byte() == FOREIGN_ID =>
            {
                PollAction::Skip { id: FOREIGN_ID }
            }
            _ => PollAction::Continue,
        });
        assert_eq!(c.wire_byte_cursor_for_test() as usize, combined.len());
    }

    #[test]
    fn instruction_count_ticks_on_every_instruction_header() {
        let mut c = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_ping(TEST_ID);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        c.stage_rx_bytes_for_test(0, &combined);

        let _ = collect_events(&mut c, |_| PollAction::Continue);
        // Two Instruction headers (foreign + own).
        assert_eq!(c.instruction_count(), 2);
    }

    #[test]
    fn instruction_count_does_not_tick_on_status_header() {
        let mut c = make();
        let status = wire_status(TEST_ID);
        c.stage_rx_bytes_for_test(0, &status);

        let _ = collect_events(&mut c, |_| PollAction::Continue);
        assert_eq!(c.instruction_count(), 0);
    }

    #[test]
    fn poll_resync_on_bad_crc_emits_resync_event() {
        let mut c = make();
        let mut wire = wire_ping(TEST_ID);
        let last = wire.len() - 1;
        wire[last] ^= 0xFF;
        c.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut c, |_| PollAction::Continue);
        let saw_resync = captures.iter().any(|c| {
            matches!(
                c,
                Capture::Event {
                    ev: Event::Resync(ResyncKind::BadCrc),
                    ..
                }
            )
        });
        assert!(saw_resync);
    }

    #[test]
    fn poll_handles_ring_wrap_across_two_feed_slices() {
        let mut c = make();
        // Pre-position the read/write seqs so the packet straddles the
        // wrap boundary. RX_BUF_LEN = 64; start at seq 60 so 4 bytes fit
        // before wrap and the remainder lands at seq 0+.
        let wire = wire_write(TEST_ID, 0x0050, &[0xAA, 0xBB, 0xCC, 0xDD]);
        c.set_rx_read_seq_for_test(60);
        c.stage_rx_bytes_for_test(60, &wire);

        let captures = collect_events(&mut c, |_| PollAction::Continue);
        let saw_crc = captures
            .iter()
            .any(|c| matches!(c, Capture::Event { ev: Event::Crc, .. }));
        assert!(
            saw_crc,
            "expected Crc after wrap parse; got captures: {captures:?}"
        );
    }

    #[test]
    fn send_status_writes_wire_bytes_into_tx_buf() {
        let mut c = make();
        c.send_status(Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        })
        .expect("encode fits");

        let mut expected: Vec<u8, TX_BUF_LEN> = Vec::new();
        StatusEncoder::<_, SoftwareCrcUmts>::new(&mut expected)
            .empty(Id::new(TEST_ID), StatusError::OK)
            .unwrap();
        assert!(c.tx_len() > 0);
        assert_eq!(c.tx_len() as usize, expected.len());
        // SAFETY: tx_buf_addr exposes the buffer's storage; len bytes are
        // initialized per `tx_len`. Read-only slice for assertion. `usize`
        // is the pointer width on both production (RV32) and host (x86_64)
        // — no truncation.
        let actual = unsafe {
            core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, c.tx_len() as usize)
        };
        assert_eq!(actual, expected.as_slice());
    }

    #[test]
    fn send_status_overwrites_previous_contents() {
        let mut c = make();
        c.send_status(Status::Ping {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            status: dxl_protocol::types::PingStatus {
                model: 0x0123,
                fw_version: 0x45,
            },
        })
        .unwrap();
        let first_len = c.tx_len();
        assert!(first_len > 0);

        c.send_status(Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        })
        .unwrap();
        let second_len = c.tx_len();
        assert!(second_len < first_len);

        // SAFETY: see `send_status_writes_wire_bytes_into_tx_buf`.
        let actual = unsafe {
            core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, second_len as usize)
        };
        assert_eq!(&actual[0..4], &[0xFF, 0xFF, 0xFD, 0x00]);
    }

    #[test]
    fn tx_buf_addr_is_stable() {
        let c = make();
        let a = c.tx_buf_addr();
        let b = c.tx_buf_addr();
        assert_eq!(a, b);
        assert_ne!(a, 0);
    }

    #[test]
    fn send_slot_only_writes_header_plus_body_plus_crc() {
        let mut c = make();
        let payload = [0x11_u8, 0x22, 0x33];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        c.send_slot(&slot, SlotPosition::Only { packet_length: 8 })
            .expect("encode fits");

        let actual = unsafe {
            core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, c.tx_len() as usize)
        };
        assert_eq!(&actual[0..4], &[0xFF, 0xFF, 0xFD, 0x00]);

        let mut expected: Vec<u8, TX_BUF_LEN> = Vec::new();
        SlotEncoder::<_, SoftwareCrcUmts>::new(&mut expected)
            .emit(&slot, SlotPosition::Only { packet_length: 8 })
            .unwrap();
        assert_eq!(actual, expected.as_slice());
    }

    #[test]
    fn send_slot_last_writes_caller_supplied_crc() {
        let mut c = make();
        let payload = [0xAA_u8, 0xBB];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        c.send_slot(&slot, SlotPosition::Last { crc: 0xDEAD })
            .expect("encode fits");

        let len = c.tx_len() as usize;
        let actual = unsafe { core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, len) };
        assert_eq!(&actual[len - 2..], &[0xAD, 0xDE]);
    }

    #[test]
    fn patch_crc_overwrites_last_two_bytes_le() {
        let mut c = make();
        let payload = [0x11_u8, 0x22, 0x33];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        c.send_slot(&slot, SlotPosition::Last { crc: 0x0000 })
            .expect("encode fits");

        let len = c.tx_len() as usize;
        c.patch_crc(0xBEEF);
        let actual = unsafe { core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, len) };
        assert_eq!(&actual[len - 2..], &[0xEF, 0xBE]);
    }

    #[test]
    fn patch_crc_noop_when_tx_buf_empty() {
        let mut c = make();
        assert_eq!(c.tx_len(), 0);
        c.patch_crc(0xDEAD);
        assert_eq!(c.tx_len(), 0);
    }

    #[test]
    fn own_reply_bytes_excludes_trailing_crc_slot() {
        let mut c = make();
        let payload = [0xAA_u8, 0xBB];
        let slot = Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data: &payload,
        };
        c.send_slot(&slot, SlotPosition::Last { crc: 0xDEAD })
            .expect("encode fits");

        let len = c.tx_len() as usize;
        let bytes = c.own_reply_bytes();
        assert_eq!(bytes.len(), len - 2);
        let actual = unsafe { core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, len) };
        assert_eq!(bytes, &actual[..len - 2]);
    }

    #[test]
    fn own_reply_bytes_empty_when_tx_buf_empty() {
        let c = make();
        assert!(c.own_reply_bytes().is_empty());
    }
}
