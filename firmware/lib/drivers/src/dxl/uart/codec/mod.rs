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

mod anchor;
mod edge_capture;
mod edge_parser;
mod skip;

pub use edge_capture::EdgeCapture;
pub use edge_parser::edge_buf_len;

use skip::{SKIP_DEADLINE_SLACK_BYTES, SkipFsm};

use core::cell::SyncUnsafeCell;
use core::marker::PhantomData;

use dxl_protocol::streaming::{Event, HeaderEvent, InstructionPayload, Parser, PayloadEvent};
use dxl_protocol::types::{Id, Slot, Status, StatusError};
use dxl_protocol::{Chunk, CrcUmts, SlotEncoder, SlotPosition, StatusEncoder, WriteError};

use super::BITS_PER_FRAME;
use crate::dxl::uart::poll_src::PollSrc;
use crate::ring::HwRing;
use crate::traits::dxl::{EdgeDma, RxDma};

/// Number of raw wire bytes at the tail of a just-parsed packet handed to
/// the tail-signature back-search at `Event::Crc`. 4 bytes — including
/// both CRC bytes plus the last 2 data bytes — gives high signature
/// uniqueness (intra-byte deltas across at least one non-`0xFF` byte
/// disambiguate from any same-spacing window elsewhere in the ring) while
/// keeping the worst-case back-search work bounded at ~20 edges checked.
const TAIL_BYTES_FOR_ANCHOR: usize = 4;

/// Read the last 4 wire bytes of a just-consumed packet + the summed
/// edge count of any bytes DMA already latched past that tail. Shared
/// between the parser `Event::Crc` path and the byte-skip `SkipComplete`
/// path — both anchor the drift walker at the CRC byte's start tick and
/// need the same signature bytes and `d_min` shift on the ET back-search.
///
/// `head_gap` = bytes from parser/skip cursor to producer head (bytes
/// DMA'd but not yet parsed/skipped). At `head_gap = 0` (typical IDLE
/// drain), the ring's most-recent 4 bytes ARE the packet's tail. Under
/// byte-ring HT/TC drain `head_gap` can be nonzero (next packet's
/// leading bytes already in the ring), so the tail sits `head_gap`
/// slots back from head. Returns `None` if the ring can't resolve the
/// tail slots (cold-start, not yet enough bytes published).
fn read_tail_and_d_min<const RX_BUF_LEN: usize>(
    rx_buf: &HwRing<u8, RX_BUF_LEN>,
    head_gap: u16,
) -> Option<([u8; TAIL_BYTES_FOR_ANCHOR], u16)> {
    let mut tail = [0u8; TAIL_BYTES_FOR_ANCHOR];
    // Oldest → newest: recent(head_gap + 3) … recent(head_gap).
    for (i, slot) in tail.iter_mut().enumerate() {
        let off = head_gap.wrapping_add((TAIL_BYTES_FOR_ANCHOR - 1 - i) as u16);
        *slot = *rx_buf.recent(off)?;
    }
    let mut d_min: u16 = 0;
    for off in 0..head_gap {
        if let Some(&b) = rx_buf.recent(off) {
            d_min = d_min.wrapping_add(edge_parser::edges_in_byte(b) as u16);
        }
    }
    Some((tail, d_min))
}

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
///
/// `Stop` exits the poll immediately after committing the bytes the
/// parser already consumed for the in-flight event. Returned when the
/// sink arms a per-byte consumer (today: the Fast Last CRC fold engine)
/// that must own all subsequent ring bytes; leaving them in the ring is
/// the only way to hand them off without losing the cursor. Used to
/// honor the RX-tail ownership contract across the fold-arm boundary
/// itself (`dxl-streaming-rx.md` §6) — the edge-IRQ mask at arm only
/// stops *future* polls.
pub enum PollAction {
    Continue,
    Skip { id: u8 },
    Stop,
}

/// RX half — streaming parser, RX byte ring, drift-sampling gate.
/// Splits off from [`CodecTx`] under [`Codec`] so the parent driver can hand
/// the dispatcher a `&mut CodecTx` reply handle alongside the parser event
/// stream the dispatcher is consuming.
pub struct CodecRx<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize> {
    edge_capture: EdgeCapture<R, EDGE_BUF_LEN>,
    parser: Parser<CRC>,
    /// DMA1_CH5 destination for received bytes. `SyncUnsafeCell` because
    /// USART1's DMA writes it concurrently with the parser's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware. [`HwRing`] enforces pow-2 sizing so
    /// `% RX_BUF_LEN` collapses to AND, and the chip-side ISR publishes
    /// the producer head via [`HwRing::on_publish`] from NDTR.
    rx_buf: SyncUnsafeCell<HwRing<u8, RX_BUF_LEN>>,
    /// Monotonic count of Instruction headers the parser emitted — own
    /// or foreign IDs — across this codec's lifetime.
    instruction_count: u32,
    /// Monotonic count of RX-ring bytes the codec has advanced past
    /// across this codec's lifetime — covers both parser-consumed bytes
    /// and byte-skip-consumed bytes. 32 bits ≈ 23.8 days at 3M sustained;
    /// wrap is non-physical.
    wire_bytes_consumed: u32,
    /// Universal byte-skip FSM + the drift-sampling gate — the poll loop's
    /// packet-kind and skip-counter state, owned together off the ring
    /// bookkeeping.
    skip_fsm: SkipFsm,
}

impl<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize>
    CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>
{
    fn new(ring: R) -> Self {
        Self {
            edge_capture: EdgeCapture::new(ring),
            parser: Parser::new(),
            rx_buf: SyncUnsafeCell::new(HwRing::new(0)),
            instruction_count: 0,
            wire_bytes_consumed: 0,
            skip_fsm: SkipFsm::new(),
        }
    }

    /// USART1 IDLE ISR entry — publishes the ET producer head, stashes
    /// the ISR-entry tick + LineIdle source for the Crc-time fallback path.
    pub fn on_idle(&mut self, now: u32) {
        self.edge_capture.on_idle(now);
    }

    /// USART1 RX DMA published progress — `remaining` is the channel's
    /// NDTR readback. Advances the `rx_buf` producer head so `poll` sees
    /// newly-DMA'd bytes. Pure ring bookkeeping; does not touch
    /// `last_isr` — the DMA1_CH5 ISR entry that owns the stash calls
    /// [`Self::on_byte_batch_wake`] alongside.
    pub fn on_rx_progress(&mut self, remaining: u16) {
        // SAFETY: rx_buf is written only by DMA1_CH5 (hardware writer)
        // and read here from the same PFIC priority level as the DMA
        // HT/TC ISR, so no other consumer can `&mut` it concurrently.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        rx_buf.on_publish(remaining);
    }

    /// Stash `(now, ByteBatch)` for the Crc-time fallback path. Called from the
    /// DMA1_CH5 HT/TC ISR entry — that vector drives the parser drain,
    /// so its wake tick is what Crc's fallback formula expects.
    pub fn on_byte_batch_wake(&mut self, now: u32) {
        self.edge_capture.record_wake_tick(now);
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
    pub fn poll<F, const PAIRS_LEN: usize>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        n_pairs_wanted: u8,
        pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
        mut on_event: F,
    ) where
        F: FnMut(PollEvent<'_>, &mut EdgeCapture<R, EDGE_BUF_LEN>) -> PollAction,
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
            if self.skip_fsm.is_skipping() {
                if self.skip_fsm.deadline_passed(now) {
                    self.skip_fsm.clear();
                    self.skip_fsm.on_packet_end();
                    continue;
                }
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                let take = self.skip_fsm.take(rx_buf.reader().avail());
                if take > 0 {
                    rx_buf.reader().advance(take);
                    self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(take as u32);
                }
                if !self.skip_fsm.is_exhausted() {
                    // Ring exhausted mid-skip; resume on a later poll.
                    return;
                }
                if let Some(id) = self.skip_fsm.finish() {
                    on_event(PollEvent::SkipComplete { id }, &mut self.edge_capture);
                }
                // Foreign Instruction packets never emit a parser
                // `Event::Crc` (parser byte-skipped past the tail), so
                // anchor + retroactive walk piggyback here. Skip drain
                // just consumed both CRC bytes, so the ring's last 4
                // bytes past `head_gap` are the packet's tail signature.
                // Fires AFTER `on_event` for deadline-path hygiene —
                // SkipComplete's sink (chain-fire predecessor-match ->
                // `start_now`) is deadline-sensitive but doesn't read
                // `packet_end_tick`, so the walk cost sits off it. Gated
                // on the instruction-only drift rule per
                // [[drift_sampling_instruction_only]] — Status skips
                // never contribute.
                if self.skip_fsm.should_sample_drift() {
                    // SAFETY: see note above.
                    let rx_buf = unsafe { &mut *rx_buf_ptr };
                    let head_gap = rx_buf.reader().avail();
                    if let Some((tail, d_min)) = read_tail_and_d_min(rx_buf, head_gap)
                        && self
                            .edge_capture
                            .anchor_at_tail(ticks_per_bit, &tail, d_min)
                    {
                        self.edge_capture
                            .walk_pairs_back(n_pairs_wanted, ticks_per_bit, pairs);
                    }
                    self.edge_capture.reset_anchor();
                }
                self.skip_fsm.on_packet_end();
                continue;
            }

            // Feed phase: extract a contiguous front slice from the
            // ring; feed to the parser; advance by what was consumed.
            // Also record `back_len` — the parser only sees `front`, but
            // `avail_at_iter_start = input_len + back_len` is what maps
            // parser cursor → producer-head distance at Crc time.
            let (input_ptr, input_len, back_len) = {
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                let mut reader = rx_buf.reader();
                let (front, back) = reader.peek_slices();
                (front.as_ptr(), front.len(), back.len())
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
            let mut stop = false;
            // Ring depth from parser cursor to producer head at the start
            // of this feed iteration. Stays fixed for the parser loop
            // (no `on_publish` runs mid-poll), so `head_gap` at any point
            // inside the loop is `avail_at_iter_start - stream.consumed()`.
            let avail_at_iter_start = input_len + back_len;
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

                    // Track packet kind for the Crc-time retroactive walk
                    // gate: Instruction packets (own or foreign) contribute
                    // drift samples per [[drift_sampling_instruction_only]];
                    // Status frames don't.
                    match ev {
                        Event::Header(HeaderEvent::Instruction(_)) => {
                            self.skip_fsm.on_header(true);
                        }
                        Event::Header(HeaderEvent::Status(_)) => {
                            self.skip_fsm.on_header(false);
                        }
                        _ => {}
                    }

                    // At Crc: back-search the ET ring against the last 4
                    // raw wire bytes the parser just consumed. Sets
                    // `tail_anchor` so `packet_end_tick` reads in wire-
                    // edge time anchored on the CRC byte's start.
                    //
                    // Read the tail bytes directly from the RX ring via
                    // `recent(offset)` back from the producer head.
                    // `head_gap` is the byte distance between the parser
                    // cursor and the producer head — 0 when Crc lands as
                    // the very last byte in the ring; nonzero under batch
                    // drain (RX HT/TC) when bytes past CRC still sit
                    // unread. Those past-CRC bytes' edges contributed to
                    // ET, so their summed `EDGES_PER_BYTE` becomes `d_min`
                    // — the anchor search origin shift.
                    if matches!(ev, Event::Crc(_)) {
                        // SAFETY: `rx_buf_ptr` is valid for the duration of
                        // this poll; `stream` only reads `input` (not the
                        // ring), so a fresh shared borrow here doesn't
                        // alias the parser's slice.
                        let rx_buf = unsafe { &*rx_buf_ptr };
                        let head_gap = (avail_at_iter_start - stream.consumed() as usize) as u16;
                        if let Some((tail, d_min)) = read_tail_and_d_min(rx_buf, head_gap) {
                            self.edge_capture
                                .anchor_at_tail(ticks_per_bit, &tail, d_min);
                        }
                    }

                    match on_event(
                        PollEvent::Event {
                            ev,
                            ring,
                            next_status_pos,
                        },
                        &mut self.edge_capture,
                    ) {
                        PollAction::Continue => {}
                        PollAction::Skip { id } => {
                            break_for_skip = Some(id);
                            break;
                        }
                        PollAction::Stop => {
                            stop = true;
                            break;
                        }
                    }

                    // Retroactive integrator walk. Runs AFTER `on_event`
                    // — i.e. after the wire-schedule call (`arm_tim2` /
                    // `scheduler.schedule`) has fired inside the sink —
                    // so the walk cost sits off the deadline path. Gated
                    // on Instruction packets only; Status frames don't
                    // contribute drift samples ([[drift_sampling_instruction_only]]).
                    if matches!(ev, Event::Crc(_))
                        && self.skip_fsm.should_sample_drift()
                        && self.edge_capture.tail_anchor().is_some()
                    {
                        self.edge_capture
                            .walk_pairs_back(n_pairs_wanted, ticks_per_bit, pairs);
                    }

                    if matches!(ev, Event::Crc(_) | Event::Resync(_)) {
                        self.skip_fsm.on_packet_end();
                        // Packet boundary — anchor state is stale.
                        // Owned here (not in the driver's `on_event`) so
                        // `walk_pairs_back` above still sees the anchor
                        // set by `anchor_at_tail` for THIS packet.
                        self.edge_capture.reset_anchor();
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

            if stop {
                // Sink armed a per-byte consumer (Fast Last fold). Leave
                // any unconsumed bytes in the ring so the fold engine —
                // driven by `drain_raw` on `on_fold_step` / `on_tx_start`
                // — owns them. The same poll continuing here would let
                // the parser eat predecessor reply bytes before the fold
                // ever gets a turn (per `dxl-streaming-rx.md` §6 —
                // edge-IRQ masking at arm shuts the door on future polls
                // only).
                return;
            }

            if let Some(id) = break_for_skip {
                let bytes_remaining = self.parser.packet_remaining();
                self.parser.reset();
                let frame_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
                let budget_bytes = bytes_remaining.saturating_add(SKIP_DEADLINE_SLACK_BYTES);
                let elapsed = (budget_bytes as u32).wrapping_mul(frame_ticks);
                let deadline_tick = now.wrapping_add(elapsed);
                self.skip_fsm.arm(bytes_remaining, id, deadline_tick);
                continue;
            }

            if consumed == 0 {
                // Parser made no progress on the front slice — either
                // it's idling at end-of-input or the slice was empty.
                return;
            }
        }
    }

    /// Drain newly-published RX bytes through `fold_slice` without invoking
    /// the parser. Used by the Fast Last fold path during the predecessor
    /// window: the SysTick CMP body and the CC3 post-fire body each spin
    /// inside this drain, refreshing the producer head from
    /// `rx_dma.remaining()` on every pass so newly-DMA'd bytes become
    /// reader-visible mid-loop without re-entering the chip-side ISR.
    /// Advances `wire_bytes_consumed` by each handed-off slice so a
    /// subsequent `poll()` resumes at the right cursor.
    ///
    /// `fold_slice` receives `(slice, base_cursor)` where `base_cursor` is
    /// the value of `wire_bytes_consumed` BEFORE this slice is folded —
    /// i.e. `slice[i]` sits at wire cursor `base_cursor + i`. The callback
    /// runs once per ring front-slice (typically one call per drain pass),
    /// so a bulk CRC fold and a single skip-before-`start_cursor` check
    /// suffice. Multiple ring laps still surface as multiple calls because
    /// `peek_slices` returns a wrap-aware front slice.
    ///
    /// The Fast Last fold owns the RX ring tail for the duration of the
    /// window: the chip-side caller masks DMA1_CH7 HT/TC at arm
    /// (`dxl-hw-timed-transport.md` §10.6.3), and `poll()` self-gates on
    /// `fast_last.fold_active()` at entry (`dxl-streaming-rx.md` §6),
    /// so the parser and this drain never race on `rx_buf`.
    pub fn drain_raw<D: RxDma, F: FnMut(&[u8], u32)>(&mut self, rx_dma: &D, mut fold_slice: F) {
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
        // The fold-window contract above keeps `poll()` and this drain
        // from racing on the ring.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        loop {
            rx_buf.on_publish(rx_dma.remaining());
            let n = {
                let mut reader = rx_buf.reader();
                let (front, _back) = reader.peek_slices();
                if front.is_empty() {
                    return;
                }
                fold_slice(front, self.wire_bytes_consumed);
                self.wire_bytes_consumed =
                    self.wire_bytes_consumed.wrapping_add(front.len() as u32);
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
        self.skip_fsm.clear();
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
        self.edge_capture.edges_addr()
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

    /// Forward to [`edge_capture::EdgeCapture::anchor_at_tail`].
    pub fn anchor_at_tail(&mut self, ticks_per_bit: u16, tail_bytes: &[u8], d_min: u16) -> bool {
        self.edge_capture
            .anchor_at_tail(ticks_per_bit, tail_bytes, d_min)
    }

    /// Forward to [`edge_capture::EdgeCapture::on_baud_change`].
    pub fn on_baud_change(&mut self, rx_edge_comp_ticks: u16) {
        self.edge_capture.on_baud_change(rx_edge_comp_ticks);
    }

    /// Forward to [`edge_capture::EdgeCapture::packet_end_tick`].
    pub fn packet_end_tick(&self, ticks_per_bit: u16, now: u32, src: PollSrc) -> Option<u32> {
        self.edge_capture.packet_end_tick(ticks_per_bit, now, src)
    }

    /// Forward to [`edge_capture::EdgeCapture::reset_anchor`].
    pub fn reset_anchor(&mut self) {
        self.edge_capture.reset_anchor();
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

    /// Streamed counterpart of [`Self::send_status`] for `Status::Read`
    /// replies: the dispatcher hands an iterator of [`Chunk`]s sourced
    /// directly from a control-table read; the stuffer consumes them in
    /// place of a scratch buffer.
    pub fn send_status_read_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        self.tx_buf.clear();
        StatusEncoder::<_, CRC>::new(&mut self.tx_buf).read_chunked(id, error, chunks)
    }

    /// Streamed counterpart of [`Self::send_slot`]: slot body bytes come
    /// from a chunk iterator. Slot bodies are unstuffed, so each
    /// `Chunk::Slice` is a single `push_slice` and `Chunk::Zero` a
    /// single `push_zero` on the TX buffer.
    pub fn send_slot_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        position: SlotPosition,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        self.tx_buf.clear();
        SlotEncoder::<_, CRC>::new(&mut self.tx_buf).emit_chunked(id, error, position, chunks)
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

/// The Fast Last fold engine finalizes into the TX half without importing it
/// (driver-pattern §5.1). Both methods delegate to the inherent forms above.
impl<CRC: CrcUmts, const TX_BUF_LEN: usize> super::fast_last::CrcPatchSink
    for CodecTx<CRC, TX_BUF_LEN>
{
    fn own_reply_bytes(&self) -> &[u8] {
        CodecTx::own_reply_bytes(self)
    }

    fn patch_crc(&mut self, crc: u16) {
        CodecTx::patch_crc(self, crc)
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

    pub fn on_idle(&mut self, now: u32) {
        self.rx.on_idle(now);
    }

    pub fn on_rx_progress(&mut self, remaining: u16) {
        self.rx.on_rx_progress(remaining);
    }

    pub fn on_byte_batch_wake(&mut self, now: u32) {
        self.rx.on_byte_batch_wake(now);
    }

    pub fn poll<F, const PAIRS_LEN: usize>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        n_pairs_wanted: u8,
        pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
        on_event: F,
    ) where
        F: FnMut(PollEvent<'_>, &mut EdgeCapture<R, EDGE_BUF_LEN>) -> PollAction,
    {
        self.rx
            .poll(now, ticks_per_bit, n_pairs_wanted, pairs, on_event);
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

    /// Forward to [`CodecRx::anchor_at_tail`].
    pub fn anchor_at_tail(&mut self, ticks_per_bit: u16, tail_bytes: &[u8], d_min: u16) -> bool {
        self.rx.anchor_at_tail(ticks_per_bit, tail_bytes, d_min)
    }

    /// Forward to [`CodecRx::on_baud_change`].
    pub fn on_baud_change(&mut self, rx_edge_comp_ticks: u16) {
        self.rx.on_baud_change(rx_edge_comp_ticks);
    }

    /// Forward to [`CodecRx::packet_end_tick`].
    pub fn packet_end_tick(&self, ticks_per_bit: u16, now: u32, src: PollSrc) -> Option<u32> {
        self.rx.packet_end_tick(ticks_per_bit, now, src)
    }

    /// Forward to [`CodecRx::reset_anchor`].
    pub fn reset_anchor(&mut self) {
        self.rx.reset_anchor();
    }
}

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

    /// Cumulative parser- + skip-consumed wire-byte cursor. Production
    /// callers read this per-event via `PollEvent::Event::next_status_pos`;
    /// tests reach the underlying counter directly to verify the cursor
    /// walks both parser and skip paths.
    pub(crate) fn wire_byte_cursor_for_test(&self) -> u32 {
        self.wire_bytes_consumed
    }

    /// Stage ET-ring stamps matching `tail_bytes`' falling-edge signature so
    /// [`EdgeCapture::anchor_at_tail`] resolves to `tail_anchor = anchor_tick` when
    /// the codec's poll reaches the packet's Crc event. Returns the number
    /// of edge stamps written — the caller staggers `MockEdgeDma::remaining`
    /// so `EdgeCapture`'s edge publish advances `write_seq` by that count.
    pub(crate) fn stage_tail_signature_for_test(
        &mut self,
        tail_bytes: &[u8],
        ticks_per_bit: u16,
        anchor_tick: u16,
    ) -> u16 {
        self.edge_capture
            .stage_tail_signature_for_test(tail_bytes, ticks_per_bit, anchor_tick)
    }
}

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

    pub(crate) fn wire_byte_cursor_for_test(&self) -> u32 {
        self.rx.wire_byte_cursor_for_test()
    }

    pub(crate) fn stage_tail_signature_for_test(
        &mut self,
        tail_bytes: &[u8],
        ticks_per_bit: u16,
        anchor_tick: u16,
    ) -> u16 {
        self.rx
            .stage_tail_signature_for_test(tail_bytes, ticks_per_bit, anchor_tick)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::MockEdgeDma;
    use dxl_protocol::streaming::{HeaderEvent, InstructionHeader};
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
        // Tests stage bytes manually via `stage_rx_bytes_for_test`; the
        // edge-ring DMA counter is never advanced by anything real, so
        // `remaining()` can return a fixed value for every call.
        let mut edge_dma = MockEdgeDma::default();
        edge_dma.expect_remaining().returning(|| 0);
        Codec::new(edge_dma)
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
    /// everything. The walker args are unused here (`n_pairs_wanted = 0`) —
    /// these tests validate parser/skip behavior, not drift sampling.
    fn collect_events<F>(c: &mut TestCodec, mut decide: F) -> alloc::vec::Vec<Capture>
    where
        F: FnMut(&Event) -> PollAction,
    {
        let mut out = alloc::vec::Vec::new();
        let mut pairs: heapless::Vec<(u16, u16), 4> = heapless::Vec::new();
        c.poll(0, 160, 0, &mut pairs, |pe, _rx| match pe {
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
            Some(Capture::Event {
                ev: Event::Crc(_), ..
            }) => {}
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
            .filter(|c| {
                matches!(
                    c,
                    Capture::Event {
                        ev: Event::Crc(_),
                        ..
                    }
                )
            })
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
    fn poll_emits_crc_bad_verdict_on_bad_crc() {
        let mut c = make();
        let mut wire = wire_ping(TEST_ID);
        let last = wire.len() - 1;
        wire[last] ^= 0xFF;
        c.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut c, |_| PollAction::Continue);
        let saw_bad = captures.iter().any(|c| {
            matches!(
                c,
                Capture::Event {
                    ev: Event::Crc(dxl_protocol::streaming::CrcResult::Bad),
                    ..
                }
            )
        });
        assert!(saw_bad);
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
        let saw_crc = captures.iter().any(|c| {
            matches!(
                c,
                Capture::Event {
                    ev: Event::Crc(_),
                    ..
                }
            )
        });
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
