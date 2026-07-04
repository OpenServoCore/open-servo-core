//! RX half — streaming parser, RX byte ring, drift-sampling gate. Splits
//! off from [`CodecTx`](super::codec_tx::CodecTx) under
//! [`Codec`](super::Codec) so the parent driver can hand the dispatcher a
//! `&mut CodecTx` reply handle alongside the parser event stream the
//! dispatcher is consuming.

use core::cell::SyncUnsafeCell;
use core::ops::ControlFlow;

use dxl_protocol::CrcUmts;
use dxl_protocol::streaming::{Event, HeaderEvent, InstructionPayload, Parser, PayloadEvent};

use super::edge_capture::EdgeCapture;
use super::edge_parser;
use super::poll_event::{PollAction, PollEvent};
use super::skip::SkipFsm;
use crate::dxl::uart::BITS_PER_FRAME;
use crate::ring::HwRing;
use crate::traits::dxl::{EdgeDma, RxDma};

/// Number of raw wire bytes at the tail of a just-parsed packet handed to
/// the tail-signature back-search at `Event::Crc`. 4 bytes — including
/// both CRC bytes plus the last 2 data bytes — gives high signature
/// uniqueness (intra-byte deltas across at least one non-`0xFF` byte
/// disambiguate from any same-spacing window elsewhere in the ring) while
/// keeping the worst-case back-search work bounded at ~20 edges checked.
/// Defined as [`super::anchor::TAIL_STARTS`] because the two MUST agree:
/// the walker's signature match only completes when the last tail byte
/// lands inside the starts cache — a longer tail slice would silently
/// never anchor (every packet degrading to the fallback path with no
/// error).
pub(crate) const TAIL_BYTES_FOR_ANCHOR: usize = super::anchor::TAIL_STARTS;

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

/// Exit disposition of one [`CodecRx::feed_parser`] phase — why the feed
/// stopped, decided by ring supply or by the sink's [`PollAction`]. The
/// poll loop dispatches on this instead of re-inspecting a
/// bool + `Option<u8>` flag pair.
enum FeedExit {
    /// Ring supplied no (more) parseable bytes — the poll is done.
    Exhausted,
    /// Sink rejected the in-flight Header; the poll arms the universal
    /// byte-skip for `id` and re-enters the skip phase.
    Skip { id: u8 },
    /// Sink engaged a per-byte consumer (Fast Last fold) that owns all
    /// remaining ring bytes — the poll exits without touching them.
    Stop,
}

/// RX half — streaming parser, RX byte ring, drift-sampling gate.
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
    pub(super) fn new(ring: R) -> Self {
        Self {
            edge_capture: EdgeCapture::new(ring),
            parser: Parser::new(),
            rx_buf: SyncUnsafeCell::new(HwRing::new(0)),
            instruction_count: 0,
            wire_bytes_consumed: 0,
            skip_fsm: SkipFsm::new(),
        }
    }
}

// -- events -------------------------------------------------------------

impl<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize>
    CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>
{
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

    /// Stash `(now, ByteBatch)` for the Crc-time fallback path. Called from
    /// the DMA1_CH5 HT/TC ISR entry — that vector drives the parser drain,
    /// so its wake tick is what Crc's fallback formula expects.
    pub fn on_byte_batch_wake(&mut self, now: u32) {
        self.edge_capture.on_byte_batch_wake(now);
    }

    /// Refresh the edge parser's per-baud RX edge-stamp compensation.
    /// Routed from the composite after every applied baud change.
    pub fn on_baud_change(&mut self, rx_edge_comp_ticks: u16) {
        self.edge_capture.on_baud_change(rx_edge_comp_ticks);
    }
}

// -- commands -----------------------------------------------------------

impl<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize>
    CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>
{
    /// Drain the RX byte ring through the streaming parser, fanning each
    /// event (or skip-complete pseudo-event) to `on_event`. Reads the
    /// front (pre-wrap) slice first, then the back (post-wrap) slice if
    /// any; advances the ring tail by what the parser consumed. The sink
    /// decides per-Header whether to forward the body or universal-byte-
    /// skip past it (doc §3 driver rule). The skip is observable as
    /// [`PollEvent::SkipComplete`] when its counter hits zero — the
    /// chain predecessor-match check (doc §5.2) rides there.
    ///
    /// `instruction_count` ticks on every emitted Instruction Header
    /// regardless of subsequent Skip; Status frames never tick. The
    /// `wire_bytes_consumed` cursor advances over both parser- and
    /// skip-consumed bytes.
    ///
    /// `now` and `ticks_per_bit` are read on entry to bound the universal
    /// byte-skip: skip entry stamps a give-up deadline from the expected
    /// byte count (see [`SkipFsm::arm`]), and each `poll()` drops a stale
    /// skip whose deadline has passed. The deadline path suppresses
    /// `SkipComplete` — it's a truncation recovery, not a successful
    /// drain, so the chain predecessor-match must not ride.
    pub fn poll<F, const PAIRS_LEN: usize>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        n_pairs_wanted: u8,
        pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
        mut on_event: F,
    ) where
        F: FnMut(PollEvent<'_>) -> PollAction,
    {
        loop {
            // Skip phase first: an armed byte-skip owns the ring tail
            // until it exhausts. Break = ring starved mid-skip; resume
            // on a later poll.
            if self.skip_fsm.is_skipping()
                && self
                    .drain_skip(now, ticks_per_bit, n_pairs_wanted, pairs, &mut on_event)
                    .is_break()
            {
                return;
            }
            // Feed phase: parser drains the ring until it runs dry or the
            // sink redirects the poll.
            match self.feed_parser(ticks_per_bit, n_pairs_wanted, pairs, &mut on_event) {
                FeedExit::Exhausted | FeedExit::Stop => return,
                FeedExit::Skip { id } => {
                    let bytes_remaining = self.parser.packet_remaining();
                    self.parser.reset();
                    let frame_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
                    self.skip_fsm.arm(bytes_remaining, id, now, frame_ticks);
                }
            }
        }
    }

    /// Skip phase of [`Self::poll`]: drain the ring up to the armed skip's
    /// remaining count; emit [`PollEvent::SkipComplete`] on exhaust.
    /// `Break` = the ring starved mid-skip (resume on a later poll);
    /// `Continue` = the skip finished (or its deadline dropped it) and the
    /// feed phase may run. The deadline check rides first so a truncated
    /// upstream packet can't leak its uncounted bytes into the next packet
    /// — once the expected duration has elapsed the ring's contents belong
    /// to whatever came next.
    #[inline]
    fn drain_skip<F, const PAIRS_LEN: usize>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        n_pairs_wanted: u8,
        pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
        on_event: &mut F,
    ) -> ControlFlow<()>
    where
        F: FnMut(PollEvent<'_>) -> PollAction,
    {
        if self.skip_fsm.deadline_passed(now) {
            self.skip_fsm.clear();
            self.skip_fsm.on_packet_end();
            return ControlFlow::Continue(());
        }
        // SAFETY: rx_buf lives in a SyncUnsafeCell; this is the codec's
        // single consumer path. The reference is independent of any
        // borrow on `self` for the counters because it's constructed via
        // a raw pointer (`SyncUnsafeCell::get` takes `&self`, returns
        // `*mut T`; the &mut deref is fresh).
        let rx_buf_ptr = self.rx_buf.get();
        // SAFETY: see note above.
        let rx_buf = unsafe { &mut *rx_buf_ptr };
        let take = self.skip_fsm.take(rx_buf.reader().avail());
        if take > 0 {
            rx_buf.reader().advance(take);
            self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(take as u32);
        }
        if !self.skip_fsm.is_exhausted() {
            // Ring exhausted mid-skip; resume on a later poll.
            return ControlFlow::Break(());
        }
        if let Some(id) = self.skip_fsm.finish() {
            on_event(PollEvent::SkipComplete { id });
        }
        // Foreign Instruction packets never emit a parser `Event::Crc`
        // (parser byte-skipped past the tail), so anchor + retroactive
        // walk piggyback here. Skip drain just consumed both CRC bytes,
        // so the ring's last 4 bytes past `head_gap` are the packet's
        // tail signature. Runs AFTER `on_event` for deadline-path hygiene
        // — SkipComplete's sink (chain predecessor-match -> `start_now`)
        // is deadline-sensitive but doesn't read `packet_end_tick`, so
        // the walk cost sits off it. Gated on the instruction-only drift
        // rule per [[drift_sampling_instruction_only]] — Status skips
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
        ControlFlow::Continue(())
    }

    /// Feed phase of [`Self::poll`]: extract contiguous front slices from
    /// the ring, feed each to the parser, and advance by what was consumed
    /// — until the ring runs dry ([`FeedExit::Exhausted`]) or the sink
    /// redirects the poll ([`FeedExit::Skip`] / [`FeedExit::Stop`]).
    ///
    /// The parser only sees `front`, but `avail_at_iter_start =
    /// input_len + back_len` is what maps parser cursor → producer-head
    /// distance at Crc time.
    #[inline]
    fn feed_parser<F, const PAIRS_LEN: usize>(
        &mut self,
        ticks_per_bit: u16,
        n_pairs_wanted: u8,
        pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
        on_event: &mut F,
    ) -> FeedExit
    where
        F: FnMut(PollEvent<'_>) -> PollAction,
    {
        // SAFETY: see `drain_skip` — same single-consumer contract.
        let rx_buf_ptr = self.rx_buf.get();
        loop {
            let (input_ptr, input_len, back_len) = {
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                let mut reader = rx_buf.reader();
                let (front, back) = reader.peek_slices();
                (front.as_ptr(), front.len(), back.len())
            };
            if input_len == 0 {
                return FeedExit::Exhausted;
            }
            // SAFETY: `input_ptr` points into `rx_buf.data`, owned by
            // self for the lifetime of this poll. The data is not
            // mutated until we call `reader.advance(consumed)` after
            // the parser loop. The parser only reads; on_event has no
            // access path back to rx_buf.
            let input: &[u8] = unsafe { core::slice::from_raw_parts(input_ptr, input_len) };

            // Set when the sink's `PollAction` breaks the parser loop;
            // returned only after the consumed bytes are committed below.
            let mut sink_exit: Option<FeedExit> = None;
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
                        // SAFETY: the parser emits chunk coordinates inside
                        // the slice it was fed, so the range is always in
                        // bounds; defensive empty slice (never panic) on a
                        // contract violation.
                        input
                            .get(offset as usize..(offset as usize + length as usize))
                            .unwrap_or(&[])
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

                    // Resolve packet-end timing while the anchor set above
                    // is still fresh, so the Crc event carries primitives
                    // and the sink never reaches into the capture half.
                    let packet_end = matches!(ev, Event::Crc(_))
                        .then(|| self.edge_capture.packet_end(ticks_per_bit));

                    match on_event(PollEvent::Parser {
                        ev,
                        ring,
                        next_status_pos,
                        packet_end,
                    }) {
                        PollAction::Continue => {}
                        PollAction::Skip { id } => {
                            sink_exit = Some(FeedExit::Skip { id });
                            break;
                        }
                        PollAction::Stop => {
                            sink_exit = Some(FeedExit::Stop);
                            break;
                        }
                    }

                    // Retroactive integrator walk. Runs AFTER `on_event`
                    // — i.e. after the wire-schedule call
                    // (`scheduler.schedule`) has completed inside the sink
                    // — so the walk cost sits off the deadline path. Gated
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

            match sink_exit {
                // On Stop: leave any unconsumed bytes in the ring so the
                // fold engine — driven by `drain_raw` on `on_fold_step` /
                // `on_tx_start` — owns them. Continuing to feed here would
                // let the parser eat predecessor reply bytes before the
                // fold ever gets a turn (per `dxl-streaming-rx.md` §6 —
                // edge-IRQ masking at fold start shuts the door on future
                // polls only).
                Some(exit) => return exit,
                // Parser made no progress on the front slice — it's
                // idling at end-of-input.
                None if consumed == 0 => return FeedExit::Exhausted,
                // Progress without a sink redirect: feed the next slice.
                None => {}
            }
        }
    }

    /// Drain newly-published RX bytes through `fold_slice` without invoking
    /// the parser. Used by the Fast Last fold path during the predecessor
    /// window. Advances `wire_bytes_consumed` by each handed-off slice so a
    /// subsequent `poll()` resumes at the right cursor.
    ///
    /// SINGLE PASS: one producer-head refresh from `rx_dma.remaining()`,
    /// one wrap-aware peek, at most two `fold_slice` calls (front + back
    /// slice), one cursor advance. Bytes that land while the fold runs are
    /// picked up by the caller's next call — the final grid body spins this
    /// drain continuously, so the pass structure bounds the per-iteration
    /// cost the fold pays per live wire byte (the pre-single-pass loop paid
    /// the full publish/reader/advance ceremony per contiguous run, which
    /// at 3M is per byte — the measured ~150 ticks/byte that lost the CRC
    /// patch race).
    ///
    /// `fold_slice` receives `(slice, base_cursor)` where `base_cursor` is
    /// the value of `wire_bytes_consumed` BEFORE this slice is folded —
    /// i.e. `slice[i]` sits at wire cursor `base_cursor + i`.
    ///
    /// The Fast Last fold owns the RX ring tail for the duration of the
    /// window: the chip-side caller masks DMA1_CH7 HT/TC at fold start
    /// (`dxl-hw-timed-transport.md` §10.6.3), and `poll()` self-gates on
    /// `fast_last.fold_active()` at entry (`dxl-streaming-rx.md` §6),
    /// so the parser and this drain never race on `rx_buf`.
    // RAM placement is load-bearing — see the note on `FsmScheduler::on_step`.
    // No `inline(never)`: the only caller is the (RAM-placed) spin body, and
    // fusing into it drops a call frame per live wire byte; the section is
    // the fallback when the inliner declines.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    pub fn drain_raw<D: RxDma, F: FnMut(&[u8], u32)>(&mut self, rx_dma: &D, mut fold_slice: F) {
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
        // The fold-window contract above keeps `poll()` and this drain
        // from racing on the ring.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        rx_buf.on_publish(rx_dma.remaining());
        let mut reader = rx_buf.reader();
        let consumed = {
            let (front, back) = reader.peek_slices();
            if front.is_empty() {
                return;
            }
            fold_slice(front, self.wire_bytes_consumed);
            if !back.is_empty() {
                fold_slice(
                    back,
                    self.wire_bytes_consumed.wrapping_add(front.len() as u32),
                );
            }
            front.len() + back.len()
        };
        self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(consumed as u32);
        reader.advance(consumed as u16);
    }

    /// FAST status-start query — start tick (WireClock u32) of the
    /// awaited Status packet's FIRST wire byte, read straight off the
    /// ET-ring mark planted at the chain instruction's `Crc(Good)`
    /// anchor (the first edge past the instruction's last edge — the
    /// wire between is idle, so it can only be the Status packet's
    /// start bit). Called from the RXNE trap during a FAST slot k > 0
    /// wait; `now` must be a fresh wire-clock reading for the u16 lift.
    ///
    /// O(1) and immune to byte-ring/edge-ring sampling skew: no byte
    /// counting, no age window — the mark's slot IS the answer once any
    /// edge lands there. `None` while nothing has landed on the mark
    /// (spurious trap — the V006 residual-drain quirk) or when no mark
    /// exists (defensive: FAST k > 0 slots are dropped on an anchor
    /// miss, so a parked wait always planted one).
    pub fn status_start_tick(&mut self, now: u32) -> Option<u32> {
        self.edge_capture.status_start_from_mark(now)
    }

    /// Clear any in-flight universal byte-skip. Called by the composite at
    /// the chip's own TX-complete event (doc §5.3): once our chain
    /// participation is over, a skip that's still tracking the chain
    /// envelope is stale, and at slow baud the deadline-bounded skip can
    /// outlive the inter-packet gap and eat the next preamble. On
    /// non-chain TX (Plain reply) the skip is already `None`, so the
    /// clear is a no-op.
    pub fn cancel_skip(&mut self) {
        self.skip_fsm.clear();
    }
}

// -- accessors ------------------------------------------------------------

impl<R: EdgeDma, CRC: CrcUmts, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize>
    CodecRx<R, CRC, RX_BUF_LEN, EDGE_BUF_LEN>
{
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
    /// callers read this per-event via `PollEvent::Parser::next_status_pos`;
    /// tests reach the underlying counter directly to verify the cursor
    /// walks both parser and skip paths.
    pub(crate) fn wire_byte_cursor_for_test(&self) -> u32 {
        self.wire_bytes_consumed
    }

    /// Stage ET-ring stamps matching `tail_bytes`' falling-edge signature so
    /// [`EdgeCapture::anchor_at_tail`] resolves to `tail_anchor = anchor_tick`
    /// when the codec's poll reaches the packet's Crc event. Returns the
    /// number of edge stamps written — the caller staggers
    /// `MockEdgeDma::remaining` so `EdgeCapture`'s edge publish advances
    /// `write_seq` by that count.
    pub(crate) fn stage_tail_signature_for_test(
        &mut self,
        tail_bytes: &[u8],
        ticks_per_bit: u16,
        anchor_tick: u16,
    ) -> u16 {
        self.edge_capture
            .stage_tail_signature_for_test(tail_bytes, ticks_per_bit, anchor_tick)
    }

    /// See [`EdgeCapture::stage_edge_at_head_for_test`].
    pub(crate) fn stage_edge_at_head_for_test(&mut self, stamp: u16) -> u16 {
        self.edge_capture.stage_edge_at_head_for_test(stamp)
    }

    /// See [`EdgeCapture::plant_reply_mark_for_test`].
    pub(crate) fn plant_reply_mark_for_test(&mut self, seq: u16) {
        self.edge_capture.plant_reply_mark_for_test(seq)
    }

    /// See [`EdgeCapture::edge_head_seq_for_test`].
    pub(crate) fn edge_head_seq_for_test(&mut self) -> u16 {
        self.edge_capture.edge_head_seq_for_test()
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::dxl::uart::test_support::{TEST_ID, wire_ping, wire_status};
    use crate::mocks::MockEdgeDma;
    use dxl_protocol::streaming::InstructionHeader;
    use dxl_protocol::types::Id;
    use dxl_protocol::{InstructionEncoder, SoftwareCrcUmts};
    use heapless::Vec;

    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;
    const FOREIGN_ID: u8 = 0x42;

    type TestRx = CodecRx<MockEdgeDma, SoftwareCrcUmts, RX_BUF_LEN, EDGE_BUF_LEN>;

    fn make() -> TestRx {
        // Tests stage bytes manually via `stage_rx_bytes_for_test`; the
        // edge-ring DMA counter is never advanced by anything real, so
        // `remaining()` can return a fixed value for every call.
        let mut edge_dma = MockEdgeDma::default();
        edge_dma.expect_remaining().returning(|| 0);
        CodecRx::new(edge_dma)
    }

    fn wire_write(id: u8, addr: u16, body: &[u8]) -> Vec<u8, 64> {
        let mut out: Vec<u8, 64> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .write(Id::new(id), addr, body)
            .unwrap();
        out
    }

    /// Drain the RX half into a `Vec` of captured events. `decide` controls
    /// per-Header skip behavior; default `|_| PollAction::Continue` forwards
    /// everything. The walker args are unused here (`n_pairs_wanted = 0`) —
    /// these tests validate parser/skip behavior, not drift sampling.
    fn collect_events<F>(rx: &mut TestRx, mut decide: F) -> alloc::vec::Vec<Capture>
    where
        F: FnMut(&Event) -> PollAction,
    {
        let mut out = alloc::vec::Vec::new();
        let mut pairs: heapless::Vec<(u16, u16), 4> = heapless::Vec::new();
        rx.poll(0, 160, 0, &mut pairs, |pe| match pe {
            PollEvent::Parser { ev, ring, .. } => {
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

    fn saw_crc(captures: &[Capture]) -> bool {
        captures.iter().any(|c| {
            matches!(
                c,
                Capture::Event {
                    ev: Event::Crc(_),
                    ..
                }
            )
        })
    }

    #[test]
    fn poll_no_op_when_ring_empty() {
        let mut rx = make();
        let captures = collect_events(&mut rx, |_| PollAction::Continue);
        assert!(captures.is_empty());
        assert_eq!(rx.instruction_count(), 0);
        assert_eq!(rx.wire_byte_cursor_for_test(), 0);
    }

    #[test]
    fn rx_buf_addr_is_stable() {
        let rx = make();
        let a = rx.rx_buf_addr();
        assert_eq!(a, rx.rx_buf_addr());
        assert_ne!(a, 0);
    }

    #[test]
    fn poll_partial_packet_resumes_on_next_call() {
        let mut rx = make();
        let pkt = wire_ping(TEST_ID);
        let split = pkt.len() - 1;

        rx.stage_rx_bytes_for_test(0, &pkt[..split]);
        let captures = collect_events(&mut rx, |_| PollAction::Continue);
        assert!(!saw_crc(&captures));
        assert_eq!(rx.instruction_count(), 1);

        rx.stage_rx_bytes_for_test(split as u16, &pkt[split..]);
        let captures = collect_events(&mut rx, |_| PollAction::Continue);
        assert!(saw_crc(&captures));
        assert_eq!(rx.instruction_count(), 1, "same packet, no re-parse");
    }

    #[test]
    fn poll_emits_sync_header_chunk_crc_for_own_write() {
        let mut rx = make();
        let wire = wire_write(TEST_ID, 0x0050, &[0xAA, 0xBB, 0xCC, 0xDD]);
        rx.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut rx, |_| PollAction::Continue);

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
        let mut rx = make();
        let wire = wire_ping(FOREIGN_ID);
        rx.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut rx, |ev| match ev {
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
        assert_eq!(rx.wire_byte_cursor_for_test() as usize, wire.len());
    }

    #[test]
    fn poll_skip_consumes_foreign_write_body_and_crc() {
        let mut rx = make();
        let wire = wire_write(FOREIGN_ID, 0x0050, &[1, 2, 3, 4]);
        rx.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut rx, |ev| match ev {
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
        assert_eq!(rx.wire_byte_cursor_for_test() as usize, wire.len());
    }

    #[test]
    fn poll_skip_complete_carries_id() {
        let mut rx = make();
        let wire = wire_ping(FOREIGN_ID);
        rx.stage_rx_bytes_for_test(0, &wire);

        let custom = 0x33u8;
        let captures = collect_events(&mut rx, |_| PollAction::Skip { id: custom });
        let id = captures.iter().find_map(|c| match c {
            Capture::SkipComplete { id } => Some(*id),
            _ => None,
        });
        assert_eq!(id, Some(custom));
    }

    #[test]
    fn poll_skip_then_own_packet_parses_clean() {
        let mut rx = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_write(TEST_ID, 0x0060, &[0x11, 0x22]);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        rx.stage_rx_bytes_for_test(0, &combined);

        let captures = collect_events(&mut rx, |ev| match ev {
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
        let mut rx = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_ping(TEST_ID);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        rx.stage_rx_bytes_for_test(0, &combined);

        let _ = collect_events(&mut rx, |ev| match ev {
            Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { id }))
                if id.as_byte() == FOREIGN_ID =>
            {
                PollAction::Skip { id: FOREIGN_ID }
            }
            _ => PollAction::Continue,
        });
        assert_eq!(rx.wire_byte_cursor_for_test() as usize, combined.len());
    }

    #[test]
    fn instruction_count_ticks_on_every_instruction_header() {
        let mut rx = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_ping(TEST_ID);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        rx.stage_rx_bytes_for_test(0, &combined);

        let _ = collect_events(&mut rx, |_| PollAction::Continue);
        // Two Instruction headers (foreign + own).
        assert_eq!(rx.instruction_count(), 2);
    }

    #[test]
    fn instruction_count_does_not_tick_on_status_header() {
        let mut rx = make();
        let status = wire_status(TEST_ID);
        rx.stage_rx_bytes_for_test(0, &status);

        let _ = collect_events(&mut rx, |_| PollAction::Continue);
        assert_eq!(rx.instruction_count(), 0);
    }

    #[test]
    fn poll_emits_crc_bad_verdict_on_bad_crc() {
        let mut rx = make();
        let mut wire = wire_ping(TEST_ID);
        let last = wire.len() - 1;
        wire[last] ^= 0xFF;
        rx.stage_rx_bytes_for_test(0, &wire);

        let captures = collect_events(&mut rx, |_| PollAction::Continue);
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
        let mut rx = make();
        // Pre-position the read/write seqs so the packet straddles the
        // wrap boundary. RX_BUF_LEN = 64; start at seq 60 so 4 bytes fit
        // before wrap and the remainder lands at seq 0+.
        let wire = wire_write(TEST_ID, 0x0050, &[0xAA, 0xBB, 0xCC, 0xDD]);
        rx.set_rx_read_seq_for_test(60);
        rx.stage_rx_bytes_for_test(60, &wire);

        let captures = collect_events(&mut rx, |_| PollAction::Continue);
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

    // ---------- FAST status-start query ----------

    const TPB_3M: u16 = 16;
    const BYTE_TICKS_3M: u32 = 10 * TPB_3M as u32;

    fn make_with_edge_remaining(remaining: u16) -> TestRx {
        let mut edge_dma = MockEdgeDma::default();
        edge_dma.expect_remaining().returning(move || remaining);
        CodecRx::new(edge_dma)
    }

    #[test]
    fn status_start_tick_none_without_mark() {
        // No instruction anchored → no mark planted → refuse (defensive;
        // FAST k > 0 slots drop on an anchor miss, so a parked wait
        // always planted one in production).
        let mut rx = make();
        assert_eq!(rx.status_start_tick(5000), None);
    }

    #[test]
    fn status_start_tick_none_before_edge_lands_on_mark() {
        // Spurious RXNE trap (V006 residual-drain quirk): the mark is
        // planted but no edge has landed on it yet.
        let mut rx = make_with_edge_remaining(EDGE_BUF_LEN as u16);
        let mark = rx.edge_head_seq_for_test();
        rx.plant_reply_mark_for_test(mark);
        assert_eq!(rx.status_start_tick(5000), None);
    }

    #[test]
    fn status_start_tick_resolves_the_marked_edge() {
        // The first edge past the mark IS the Status packet's first
        // byte's start — resolved directly, no window, no byte count.
        let mut rx = make_with_edge_remaining(EDGE_BUF_LEN as u16 - 1);
        let mark = rx.edge_head_seq_for_test();
        rx.plant_reply_mark_for_test(mark);
        let n = rx.stage_tail_signature_for_test(&[0xFF], TPB_3M, 1000);
        assert_eq!(n, 1, "0xFF must contribute exactly one falling edge");
        let now = 1000 + BYTE_TICKS_3M + 50;
        assert_eq!(rx.status_start_tick(now), Some(1000));
    }

    #[test]
    fn status_start_tick_ignores_later_edges_past_the_mark() {
        // Coalesced wake several bytes in: later stamps (whole or partial
        // bytes) are irrelevant — the marked slot alone answers, so
        // byte-ring/edge-ring sampling skew can't mis-index the start.
        let mut rx = make_with_edge_remaining(EDGE_BUF_LEN as u16 - 5);
        let mark = rx.edge_head_seq_for_test();
        rx.plant_reply_mark_for_test(mark);
        let hdr = [0xFF, 0xFF, 0xFD, 0x00];
        rx.stage_tail_signature_for_test(&hdr, TPB_3M, 2000 + 3 * BYTE_TICKS_3M as u16);
        // The staged signature's OLDEST edge (byte 0's start) sits at the
        // mark; the query must return it, not the newer stamps.
        let now = 2000 + 4 * BYTE_TICKS_3M + 200;
        assert_eq!(rx.status_start_tick(now), Some(2000));
    }

    #[test]
    fn drain_raw_single_pass_hands_wrapped_region_as_two_slices() {
        let mut rx = make();
        // Cursor parked just before the ring's wrap; 8 staged bytes span it.
        rx.set_rx_read_seq_for_test(60);
        rx.stage_rx_bytes_for_test(60, &[1, 2, 3, 4, 5, 6, 7, 8]);
        let mut rx_dma = crate::mocks::MockRxDma::new();
        // Producer head at ring pos 4 (seq 68) → NDTR remaining = 64 − 4.
        rx_dma.expect_remaining().returning(|| 60);

        let mut calls: alloc::vec::Vec<(alloc::vec::Vec<u8>, u32)> = alloc::vec::Vec::new();
        rx.drain_raw(&rx_dma, |slice, base| calls.push((slice.to_vec(), base)));

        assert_eq!(
            calls,
            alloc::vec![(alloc::vec![1, 2, 3, 4], 0), (alloc::vec![5, 6, 7, 8], 4)],
            "one pass: front slice to the wrap boundary, back slice after it",
        );

        // Fully consumed — a second pass hands nothing.
        calls.clear();
        rx.drain_raw(&rx_dma, |slice, base| calls.push((slice.to_vec(), base)));
        assert!(calls.is_empty());
    }
}
