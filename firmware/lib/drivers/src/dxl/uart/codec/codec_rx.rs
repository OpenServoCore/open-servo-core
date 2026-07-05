//! RX half — streaming parser, RX byte ring, drift-sampling gate. Splits
//! off from [`CodecTx`](super::codec_tx::CodecTx) under
//! [`Codec`](super::Codec) so the parent driver can hand the dispatcher a
//! `&mut CodecTx` reply handle alongside the parser event stream the
//! dispatcher is consuming.

use core::cell::SyncUnsafeCell;
use core::ops::ControlFlow;

use dxl_protocol::CrcUmts;
use dxl_protocol::streaming::{Event, HeaderEvent, InstructionPayload, Parser, PayloadEvent};
use dxl_protocol::wire;

use super::packet_end;
use super::poll_event::{PollAction, PollEvent};
use super::skip::SkipFsm;
use super::span::{DriftWindow, SpanTracker};
use crate::dxl::uart::BITS_PER_FRAME;
use crate::dxl::uart::poll_src::PollSrc;
use crate::ring::HwRing;
use crate::traits::dxl::RxDma;

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
pub struct CodecRx<CRC: CrcUmts, const RX_BUF_LEN: usize> {
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
    /// Drift-span tracker — pairs consecutive drain-ISR stamps into
    /// `(d_ticks, d_bytes)` spans for the HSI integrator.
    span: SpanTracker,
    /// RXNE cold-start window — accumulates per-byte wake stamps into one
    /// long span per burst, the isolated-short-packet path the drain-ISR
    /// [`SpanTracker`] can't cover (a short packet trips a single drain).
    window: DriftWindow,
    /// Most recent drain-ISR's wake capture — WireClock u32 tick at ISR
    /// entry plus the source flavor. Set on every [`Self::on_idle`] /
    /// [`Self::on_byte_batch_wake`] entry; consumed at the parser's Crc
    /// event by [`packet_end::resolve`] to derive the packet-end tick.
    /// Default `(0, PollSrc::ByteBatch)` is unreachable in production — Crc
    /// follows bytes which follow an ISR — but keeps the field non-Optional
    /// so the hot path carries no `unwrap`.
    last_isr: (u32, PollSrc),
}

impl<CRC: CrcUmts, const RX_BUF_LEN: usize> CodecRx<CRC, RX_BUF_LEN> {
    pub(super) fn new() -> Self {
        Self {
            parser: Parser::new(),
            rx_buf: SyncUnsafeCell::new(HwRing::new(0)),
            instruction_count: 0,
            wire_bytes_consumed: 0,
            skip_fsm: SkipFsm::new(),
            span: SpanTracker::new(),
            window: DriftWindow::new(),
            last_isr: (0, PollSrc::ByteBatch),
        }
    }
}

// -- events -------------------------------------------------------------

impl<CRC: CrcUmts, const RX_BUF_LEN: usize> CodecRx<CRC, RX_BUF_LEN> {
    /// USART1 IDLE ISR entry — stashes the ISR-entry tick + LineIdle source
    /// for the packet-end estimate and records a drift-span stamp. IDLE
    /// latches one frame after the
    /// last stop bit, so `now` is back-dated by `byte_ticks` before storing
    /// — both flavors then sit on the same last-byte reference, which the
    /// same-flavor rule needs for the IDLE→IDLE pair spanning one packet.
    /// The caller must publish the byte ring (via [`Self::on_rx_progress`])
    /// before this so the span's cursor is current.
    pub fn on_idle(&mut self, now: u32, byte_ticks: u32) -> Option<(u32, u32)> {
        self.last_isr = (now, PollSrc::LineIdle);
        let cursor = self.published_cursor();
        self.span.on_stamp(
            now.wrapping_sub(byte_ticks),
            cursor,
            PollSrc::LineIdle,
            byte_ticks,
            self.skip_fsm.should_sample_drift(),
        )
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

    /// Stash `(now, ByteBatch)` for the packet-end fallback path and record
    /// a drift-span stamp. Called from the DMA1_CH5 HT/TC ISR entry, whose
    /// caller publishes the byte ring (via [`Self::on_rx_progress`]) first,
    /// so the span's cursor is current.
    pub fn on_byte_batch_wake(&mut self, now: u32, byte_ticks: u32) -> Option<(u32, u32)> {
        self.last_isr = (now, PollSrc::ByteBatch);
        let cursor = self.published_cursor();
        self.span.on_stamp(
            now,
            cursor,
            PollSrc::ByteBatch,
            byte_ticks,
            self.skip_fsm.should_sample_drift(),
        )
    }

    /// One per-byte RX wake landed while the drift window is open — publish
    /// the byte ring from `remaining` (NDTR) so the cursor is current, then
    /// record `(now, published_cursor)`. The burst's accumulated span is
    /// emitted later at the packet boundary during [`Self::poll`]; see
    /// [`DriftWindow`](super::span::DriftWindow).
    pub fn record_drift_wake(&mut self, now: u32, remaining: u16) {
        self.on_rx_progress(remaining);
        let cursor = self.published_cursor();
        self.window.record(now, cursor);
    }

    /// Take the drift window's accumulated span from the most recent
    /// packet boundary, if one qualified. Take-once; the caller feeds it
    /// to `Clock::on_span`.
    pub fn take_window_span(&mut self) -> Option<(u32, u32)> {
        self.window.take_span()
    }

    /// The producer head as a monotonic wire-byte cursor —
    /// `wire_bytes_consumed + reader.avail()`. The span tracker diffs it
    /// across stamps to get `d_bytes`; the caller must have published the
    /// ring from NDTR first.
    fn published_cursor(&self) -> u32 {
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        self.wire_bytes_consumed
            .wrapping_add(rx_buf.reader().avail() as u32)
    }
}

// -- commands -----------------------------------------------------------

impl<CRC: CrcUmts, const RX_BUF_LEN: usize> CodecRx<CRC, RX_BUF_LEN> {
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
    pub fn poll<F>(&mut self, now: u32, ticks_per_bit: u16, packet_end_comp: u32, mut on_event: F)
    where
        F: FnMut(PollEvent<'_>) -> PollAction,
    {
        loop {
            // Skip phase first: an armed byte-skip owns the ring tail
            // until it exhausts. Break = ring starved mid-skip; resume
            // on a later poll.
            if self.skip_fsm.is_skipping()
                && self
                    .drain_skip(now, ticks_per_bit, &mut on_event)
                    .is_break()
            {
                return;
            }
            // Feed phase: parser drains the ring until it runs dry or the
            // sink redirects the poll.
            match self.feed_parser(ticks_per_bit, packet_end_comp, &mut on_event) {
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
    fn drain_skip<F>(&mut self, now: u32, ticks_per_bit: u16, on_event: &mut F) -> ControlFlow<()>
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
        let byte_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
        self.window
            .settle(byte_ticks, self.skip_fsm.should_sample_drift());
        self.skip_fsm.on_packet_end();
        ControlFlow::Continue(())
    }

    /// Feed phase of [`Self::poll`]: extract contiguous front slices from
    /// the ring, feed each to the parser, and advance by what was consumed
    /// — until the ring runs dry ([`FeedExit::Exhausted`]) or the sink
    /// redirects the poll ([`FeedExit::Skip`] / [`FeedExit::Stop`]).
    #[inline]
    fn feed_parser<F>(
        &mut self,
        ticks_per_bit: u16,
        packet_end_comp: u32,
        on_event: &mut F,
    ) -> FeedExit
    where
        F: FnMut(PollEvent<'_>) -> PollAction,
    {
        // SAFETY: see `drain_skip` — same single-consumer contract.
        let rx_buf_ptr = self.rx_buf.get();
        loop {
            let (input_ptr, input_len) = {
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                let mut reader = rx_buf.reader();
                let (front, _back) = reader.peek_slices();
                (front.as_ptr(), front.len())
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

                    // Track packet kind for the span tracker's drift gate:
                    // Instruction packets (own or foreign) contribute drift
                    // samples per [[drift_sampling_instruction_only]]; Status
                    // frames don't.
                    match ev {
                        Event::Header(HeaderEvent::Instruction(_)) => {
                            self.skip_fsm.on_header(true);
                        }
                        Event::Header(HeaderEvent::Status(_)) => {
                            self.skip_fsm.on_header(false);
                        }
                        _ => {}
                    }

                    // Resolve packet-end timing so the Crc event carries a
                    // primitive and the sink never reaches into the codec's
                    // drain-ISR stash.
                    let (isr_now, isr_src) = self.last_isr;
                    let packet_end = matches!(ev, Event::Crc(_)).then(|| {
                        packet_end::resolve(isr_src, isr_now, ticks_per_bit, packet_end_comp)
                    });

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

                    if matches!(ev, Event::Crc(_) | Event::Resync(_)) {
                        let byte_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
                        self.window
                            .settle(byte_ticks, self.skip_fsm.should_sample_drift());
                        self.skip_fsm.on_packet_end();
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
                // fold engine — driven by `poll_checkpoint` on `on_fold_step` /
                // `on_tx_start` — owns them. Continuing to feed here would
                // let the parser eat predecessor reply bytes before the
                // fold ever gets a turn (per `dxl-streaming-rx.md` §6 —
                // the poll's `fold_active()` self-gate shuts the door on
                // future polls only).
                Some(exit) => return exit,
                // Parser made no progress on the front slice — it's
                // idling at end-of-input.
                None if consumed == 0 => return FeedExit::Exhausted,
                // Progress without a sink redirect: feed the next slice.
                None => {}
            }
        }
    }

    /// Checkpoint pickup for the Fast successor CRC pipeline: once the RX
    /// ring has published through `end_cursor` (the predecessor window's
    /// wire end), return the window's trailing [`CRC_BYTES`] — the
    /// predecessor's cumulative chain CRC, i.e. the running chain state —
    /// and advance consumption to `end_cursor` so the parser resumes at
    /// our own block's cursor. Returns `None` while the checkpoint bytes
    /// are still on the wire; the wake body spins this against the live
    /// ring (one NDTR publish per call).
    ///
    /// The pipeline owns the RX ring tail for the duration of the window:
    /// `poll()` self-gates on `fast_last.fold_active()` at entry
    /// (`dxl-streaming-rx.md` §6), so the parser and this pickup never
    /// race on `rx_buf` — there is no edge channel to mask.
    // RAM placement is load-bearing — see `FsmScheduler::on_step`.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    /// One spin iteration of the checkpoint pickup: publish the ring head,
    /// consume the published window prefix (pure cursor arithmetic — no
    /// reads, no slices; keeping the reader caught up is also what keeps
    /// `avail()` lap-free on windows deeper than the ring), and — once the
    /// window (through `end_cursor`) has fully arrived — return its
    /// trailing [`CRC_BYTES`]: the predecessor's cumulative chain CRC,
    /// i.e. the running chain state. `None` while the checkpoint is still
    /// on the wire. Single-pass so the succeeding iteration pays no
    /// re-publish ceremony — at 3M the patch races the TX DMA's read of
    /// the trailing CRC slot with ~a quarter byte-time to spare.
    ///
    /// The pipeline owns the RX ring tail for the duration of the window:
    /// `poll()` self-gates on `fast_last.fold_active()` at entry
    /// (`dxl-streaming-rx.md` §6), so the parser and this pickup never
    /// race on `rx_buf` — there is no edge channel to mask.
    // RAM placement is load-bearing — see `FsmScheduler::on_step`.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    pub fn poll_checkpoint<D: RxDma>(
        &mut self,
        rx_dma: &D,
        end_cursor: u32,
    ) -> Option<[u8; wire::CRC_BYTES]> {
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
        // The ownership contract above keeps `poll()` and this pickup
        // from racing on the ring.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        rx_buf.on_publish(rx_dma.remaining());
        let mut reader = rx_buf.reader();
        let published = self.wire_bytes_consumed.wrapping_add(reader.avail() as u32);
        let ready = (published.wrapping_sub(end_cursor) as i32) >= 0;
        // Consume up to (never past) the window end. The checkpoint bytes
        // themselves stay readable — the read below resolves by producer
        // offset, which consumption doesn't disturb.
        if (end_cursor.wrapping_sub(self.wire_bytes_consumed) as i32) >= 0 {
            let target = if ready { end_cursor } else { published };
            let advance = target.wrapping_sub(self.wire_bytes_consumed);
            reader.advance(advance as u16);
            self.wire_bytes_consumed = target;
        }
        if !ready {
            return None;
        }
        let mut checkpoint = [0u8; wire::CRC_BYTES];
        for (i, slot) in checkpoint.iter_mut().enumerate() {
            let cursor = end_cursor
                .wrapping_sub(wire::CRC_BYTES as u32)
                .wrapping_add(i as u32);
            let back = published.wrapping_sub(1).wrapping_sub(cursor);
            *slot = *rx_buf.recent(back as u16)?;
        }
        Some(checkpoint)
    }

    /// Release a canceled pickup window: consume (raw, unparsed) whatever
    /// of it has published, capped at `end_cursor`. Called when the
    /// pipeline gives the ring tail back before its pickup ran (own TX
    /// complete after a starved wait) — the window's bytes must never
    /// reach the parser (a First header inside advertises the whole
    /// chain's length; parsing it after a collapsed chain swallows the
    /// next instruction), and the cursor must land at the wire frontier
    /// so live traffic parses cleanly.
    pub fn release_window<D: RxDma>(&mut self, rx_dma: &D, end_cursor: u32) {
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        rx_buf.on_publish(rx_dma.remaining());
        if (end_cursor.wrapping_sub(self.wire_bytes_consumed) as i32) < 0 {
            return;
        }
        let published = self
            .wire_bytes_consumed
            .wrapping_add(rx_buf.reader().avail() as u32);
        let target = if (published.wrapping_sub(end_cursor) as i32) >= 0 {
            end_cursor
        } else {
            published
        };
        let advance = target.wrapping_sub(self.wire_bytes_consumed);
        rx_buf.reader().advance(advance as u16);
        self.wire_bytes_consumed = target;
    }

    /// FAST status-start query — start tick (WireClock u32) of the awaited
    /// Status packet's FIRST wire byte, estimated by back-dating the wake
    /// tick `now` by the whole bytes already published past the packet's
    /// `start_cursor`. Publishes the RX byte ring from `rx_dma.remaining()`,
    /// counts `n = published − start_cursor` whole frames, and returns
    /// `now − n · byte_ticks`. Called from the RXNE trap during a FAST slot
    /// k > 0 wait; `now` must be a fresh wire-clock reading.
    ///
    /// The estimate carries the wake's ISR-entry latency, which only
    /// *delays* the observed start — never advances it into the
    /// predecessor's window — so the derived slot start can slip late but
    /// can never fire early. `None` while no whole byte has landed past the
    /// cursor (spurious wake — the V006 residual-drain quirk; same contract
    /// as before).
    pub fn status_start_tick<D: RxDma>(
        &mut self,
        rx_dma: &D,
        start_cursor: u32,
        now: u32,
        byte_ticks: u32,
    ) -> Option<u32> {
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        rx_buf.on_publish(rx_dma.remaining());
        let published = self
            .wire_bytes_consumed
            .wrapping_add(rx_buf.reader().avail() as u32);
        let n = published.wrapping_sub(start_cursor) as i32;
        if n <= 0 {
            return None;
        }
        Some(now.wrapping_sub(byte_ticks.wrapping_mul(n as u32)))
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

impl<CRC: CrcUmts, const RX_BUF_LEN: usize> CodecRx<CRC, RX_BUF_LEN> {
    /// Monotonic count of Instruction headers the parser has emitted.
    /// Foreign IDs count too (sink filters at its layer); Status frames
    /// don't.
    pub fn instruction_count(&self) -> u32 {
        self.instruction_count
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
impl<CRC: CrcUmts, const RX_BUF_LEN: usize> CodecRx<CRC, RX_BUF_LEN> {
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
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::dxl::uart::test_support::{TEST_ID, wire_ping, wire_status};
    use dxl_protocol::streaming::InstructionHeader;
    use dxl_protocol::types::Id;
    use dxl_protocol::{InstructionEncoder, SoftwareCrcUmts};
    use heapless::Vec;

    const RX_BUF_LEN: usize = 64;
    const FOREIGN_ID: u8 = 0x42;

    type TestRx = CodecRx<SoftwareCrcUmts, RX_BUF_LEN>;

    fn make() -> TestRx {
        CodecRx::new()
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
    /// everything.
    fn collect_events<F>(rx: &mut TestRx, mut decide: F) -> alloc::vec::Vec<Capture>
    where
        F: FnMut(&Event) -> PollAction,
    {
        let mut out = alloc::vec::Vec::new();
        rx.poll(0, 160, 0, |pe| match pe {
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

    /// `MockRxDma` whose `remaining()` returns a fixed NDTR readback so the
    /// estimate's `on_publish` sees the ring head the test staged.
    fn rx_dma_with_remaining(remaining: u16) -> crate::mocks::MockRxDma {
        let mut m = crate::mocks::MockRxDma::new();
        m.expect_remaining().returning(move || remaining);
        m
    }

    #[test]
    fn status_start_tick_none_before_any_reply_byte() {
        // Spurious wake (V006 residual-drain quirk): nothing published past
        // the cursor yet, so no whole byte to back-date from.
        let mut rx = make();
        let rx_dma = rx_dma_with_remaining(RX_BUF_LEN as u16);
        assert_eq!(rx.status_start_tick(&rx_dma, 0, 5000, BYTE_TICKS_3M), None);
    }

    #[test]
    fn status_start_tick_backdates_one_byte() {
        // One whole byte past the cursor → back-date the wake by one frame.
        let mut rx = make();
        rx.stage_rx_bytes_for_test(0, &[0xFF]);
        let rx_dma = rx_dma_with_remaining(RX_BUF_LEN as u16 - 1);
        let now = 5000;
        assert_eq!(
            rx.status_start_tick(&rx_dma, 0, now, BYTE_TICKS_3M),
            Some(now - BYTE_TICKS_3M)
        );
    }

    #[test]
    fn status_start_tick_counts_whole_bytes_past_the_cursor() {
        // Coalesced wake several bytes in: the estimate back-dates by every
        // whole byte published past the cursor, not just the first.
        let mut rx = make();
        rx.stage_rx_bytes_for_test(0, &[0xFF, 0xFF, 0xFD, 0x00]);
        let rx_dma = rx_dma_with_remaining(RX_BUF_LEN as u16 - 4);
        let now = 8000;
        // Cursor at 1 → 3 whole bytes (seqs 1..4) sit past it.
        assert_eq!(
            rx.status_start_tick(&rx_dma, 1, now, BYTE_TICKS_3M),
            Some(now - 3 * BYTE_TICKS_3M)
        );
    }

    #[test]
    fn status_start_tick_none_when_cursor_is_past_published() {
        // Defensive: a stale cursor ahead of the published head yields a
        // non-positive count — refuse rather than back-date into the future.
        let mut rx = make();
        rx.stage_rx_bytes_for_test(0, &[0xFF]);
        let rx_dma = rx_dma_with_remaining(RX_BUF_LEN as u16 - 1);
        assert_eq!(rx.status_start_tick(&rx_dma, 4, 5000, BYTE_TICKS_3M), None);
    }

    #[test]
    fn poll_checkpoint_returns_the_window_tail_and_advances_consumption() {
        let mut rx = make();
        // Cursor parked just before the ring's wrap; the 8-byte window
        // spans it, its checkpoint (last 2 bytes) lands after the wrap.
        rx.set_rx_read_seq_for_test(60);
        rx.stage_rx_bytes_for_test(60, &[1, 2, 3, 4, 5, 6, 0xCD, 0xAB]);
        let mut rx_dma = crate::mocks::MockRxDma::new();
        // Producer head at ring pos 4 (seq 68) → NDTR remaining = 64 − 4.
        rx_dma.expect_remaining().returning(|| 60);

        // wire_bytes_consumed starts at 0 in a fresh codec, so the window
        // occupies cursors 0..8.
        assert_eq!(rx.poll_checkpoint(&rx_dma, 8), Some([0xCD, 0xAB]));

        // Consumption advanced to the window end: nothing left unread.
        let again = rx.poll_checkpoint(&rx_dma, 8);
        assert_eq!(again, Some([0xCD, 0xAB]), "idempotent re-read is lap-safe");
    }

    #[test]
    fn poll_checkpoint_waits_while_the_tail_is_still_on_the_wire() {
        let mut rx = make();
        // Only 6 of the 8 window bytes published — checkpoint incomplete.
        rx.stage_rx_bytes_for_test(0, &[1, 2, 3, 4, 5, 6]);
        let mut rx_dma = crate::mocks::MockRxDma::new();
        rx_dma.expect_remaining().returning(|| 64 - 6);

        assert_eq!(rx.poll_checkpoint(&rx_dma, 8), None);
    }
}
