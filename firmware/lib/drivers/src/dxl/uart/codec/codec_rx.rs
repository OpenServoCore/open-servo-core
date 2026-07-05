//! RX half — streaming parser, RX byte ring, drift-sampling gate. Splits
//! off from [`CodecTx`](super::codec_tx::CodecTx) under
//! [`Codec`](super::Codec) so the parent driver can hand the dispatcher a
//! `&mut CodecTx` reply handle alongside the parser event stream the
//! dispatcher is consuming.

use core::cell::SyncUnsafeCell;
use core::ops::ControlFlow;

use dxl_protocol::CrcUmts;
use dxl_protocol::wire;

use super::framer::{FrameClass, FrameOutcome, Framer};
use super::packet_end;
use super::poll_event::{FrameAction, FrameVerdict};
use super::skip::SkipFsm;
use super::span::{DriftWindow, SpanTracker};
use crate::dxl::uart::BITS_PER_FRAME;
use crate::dxl::uart::poll_src::PollSrc;
use crate::ring::HwRing;
use crate::traits::dxl::RxDma;

/// RX half — frame classifier, RX byte ring, drift-sampling gate.
pub struct CodecRx<CRC: CrcUmts, const RX_BUF_LEN: usize> {
    framer: Framer<CRC>,
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
            framer: Framer::new(),
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
    /// Drive the RX byte ring through the frame classifier, fanning one
    /// verdict per whole frame to `on_verdict`. Own/broadcast instruction
    /// frames are copied into the framer's scratch (fused with the CRC fold)
    /// and surfaced as [`FrameVerdict::Instruction`] once complete and
    /// CRC-verified; foreign / Status / ChainStatus / oversize frames are
    /// byte-skipped whole and surfaced as [`FrameVerdict::SkipComplete`] on
    /// exhaust (the chain predecessor-match check, doc §5.2, rides there).
    ///
    /// `instruction_count` ticks on every classified Instruction frame (own
    /// or foreign) regardless of subsequent skip; Status / ChainStatus
    /// frames never tick. The `wire_bytes_consumed` cursor advances over
    /// copied, skipped, and dropped bytes alike.
    ///
    /// `now` / `ticks_per_bit` bound the universal byte-skip (skip entry
    /// stamps a give-up deadline; a stale skip is dropped without a
    /// `SkipComplete`). `our_id` decides own vs foreign for plain
    /// instructions. `on_verdict` returns [`FrameAction::Stop`] to bail the
    /// in-flight poll after engaging the Fast Last fold / a successor wait
    /// (doc §6).
    pub fn poll<F>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        packet_end_comp: u32,
        our_id: u8,
        mut on_verdict: F,
    ) where
        F: FnMut(FrameVerdict<'_>) -> FrameAction,
    {
        let byte_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
        // SAFETY: rx_buf is single-consumer at PFIC HIGH; the raw pointer
        // keeps the ring reference independent of the `&mut self` counter
        // borrows below (see the original streaming path's contract).
        let rx_buf_ptr = self.rx_buf.get();
        loop {
            // Skip phase: an armed byte-skip owns the ring tail until it
            // exhausts. Break = ring starved mid-skip; resume on a later poll.
            if self.skip_fsm.is_skipping()
                && self.drain_skip(now, byte_ticks, &mut on_verdict).is_break()
            {
                return;
            }

            // Copy phase: absorb newly-published bytes of an own frame.
            if self.framer.is_copying() {
                let consumed = {
                    // SAFETY: see note above.
                    let rx_buf = unsafe { &mut *rx_buf_ptr };
                    let mut reader = rx_buf.reader();
                    let (front, back) = reader.peek_slices();
                    let n = self.framer.absorb(front, back);
                    reader.advance(n as u16);
                    n
                };
                self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(consumed as u32);
                if !self.framer.copy_complete() {
                    return; // ring starved mid-copy; resume on a later poll
                }
                // The whole frame is a packet boundary regardless of verdict.
                let sample = self.skip_fsm.should_sample_drift();
                self.window.settle(byte_ticks, sample);
                self.skip_fsm.on_packet_end();
                match self.framer.finish() {
                    FrameOutcome::Good { instr, broadcast } => {
                        let (isr_now, isr_src) = self.last_isr;
                        let packet_end =
                            packet_end::resolve(isr_src, isr_now, ticks_per_bit, packet_end_comp);
                        // The whole frame is consumed, so the cursor now sits
                        // where the First predecessor reply's leading `0xFF`
                        // will land — the Fast Last fold's start cursor.
                        let fold_start_cursor = self.wire_bytes_consumed;
                        let action = on_verdict(FrameVerdict::Instruction {
                            instr,
                            broadcast,
                            packet_end,
                            fold_start_cursor,
                        });
                        self.framer.reset();
                        if matches!(action, FrameAction::Stop) {
                            return;
                        }
                    }
                    FrameOutcome::Bad => {
                        // Silent drop: the copy already consumed the whole
                        // frame off the ring, so re-probe resumes past it —
                        // matching the streaming parser's "move on after a
                        // Crc(Bad)" behavior. No extra advance.
                        self.framer.reset();
                    }
                }
                continue;
            }

            // Hunt phase: classify the frame at the ring head.
            let class = {
                // SAFETY: see note above.
                let rx_buf = unsafe { &mut *rx_buf_ptr };
                let mut reader = rx_buf.reader();
                let (front, back) = reader.peek_slices();
                self.framer.classify(front, back, our_id)
            };
            match class {
                FrameClass::NeedMore => return,
                FrameClass::Junk { skip } => {
                    // SAFETY: see note above.
                    let rx_buf = unsafe { &mut *rx_buf_ptr };
                    rx_buf.reader().advance(skip as u16);
                    self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(skip as u32);
                }
                FrameClass::Copy {
                    total,
                    id,
                    instruction,
                } => {
                    self.skip_fsm.on_header(true);
                    self.instruction_count = self.instruction_count.wrapping_add(1);
                    self.framer.begin(total, id, instruction);
                }
                FrameClass::Skip {
                    total,
                    id,
                    is_instruction,
                } => {
                    self.skip_fsm.on_header(is_instruction);
                    if is_instruction {
                        self.instruction_count = self.instruction_count.wrapping_add(1);
                    }
                    // Consume the classified header off the ring, then skip only
                    // the body + CRC — mirrors the streaming parser's
                    // header-then-skip split so the skip's give-up deadline
                    // bounds the remaining body, not the whole frame. A
                    // truncated over-advertised frame (collapsed Fast chain
                    // reply) must drop soon enough that the next packet's bytes
                    // don't get counted into the stale skip.
                    let header = wire::REQUEST_HEADER_BYTES.min(total);
                    {
                        // SAFETY: see note above.
                        let rx_buf = unsafe { &mut *rx_buf_ptr };
                        rx_buf.reader().advance(header as u16);
                    }
                    self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(header as u32);
                    self.skip_fsm
                        .arm((total - header) as u16, id, now, byte_ticks);
                }
            }
        }
    }

    /// Skip phase of [`Self::poll`]: drain the ring up to the armed skip's
    /// remaining count; emit [`FrameVerdict::SkipComplete`] on exhaust.
    /// `Break` = the ring starved mid-skip (resume on a later poll);
    /// `Continue` = the skip finished (or its deadline dropped it). The
    /// deadline check rides first so a truncated upstream packet can't leak
    /// its uncounted bytes into the next packet on the wire.
    #[inline]
    fn drain_skip<F>(&mut self, now: u32, byte_ticks: u32, on_verdict: &mut F) -> ControlFlow<()>
    where
        F: FnMut(FrameVerdict<'_>) -> FrameAction,
    {
        if self.skip_fsm.deadline_passed(now) {
            self.skip_fsm.clear();
            self.skip_fsm.on_packet_end();
            return ControlFlow::Continue(());
        }
        // SAFETY: rx_buf is single-consumer at PFIC HIGH (same as `poll`).
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
            on_verdict(FrameVerdict::SkipComplete { id });
        }
        self.window
            .settle(byte_ticks, self.skip_fsm.should_sample_drift());
        self.skip_fsm.on_packet_end();
        ControlFlow::Continue(())
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
    use dxl_protocol::SoftwareCrcUmts;
    use dxl_protocol::encode::encode_instruction;
    use dxl_protocol::types::Id;
    use dxl_protocol::types::packet::Instruction;
    use heapless::Vec;

    const RX_BUF_LEN: usize = 64;
    const FOREIGN_ID: u8 = 0x42;

    type TestRx = CodecRx<SoftwareCrcUmts, RX_BUF_LEN>;

    fn make() -> TestRx {
        CodecRx::new()
    }

    fn wire_write(id: u8, addr: u16, body: &[u8]) -> Vec<u8, 64> {
        let mut buf = [0u8; 64];
        let n = encode_instruction::<SoftwareCrcUmts>(
            &mut buf,
            Id::new(id),
            dxl_protocol::Instruction::Write.as_u8(),
            &[&addr.to_le_bytes(), body],
        )
        .unwrap();
        Vec::from_slice(&buf[..n]).unwrap()
    }

    /// One captured frame verdict, flattened to the facts the tests assert.
    #[derive(Debug)]
    enum Cap {
        Ping,
        Write {
            address: u16,
            data: alloc::vec::Vec<u8>,
        },
        OtherInstr,
        Skip {
            id: u8,
        },
    }

    /// Drive the RX half addressed as `TEST_ID`, capturing one [`Cap`] per
    /// frame verdict. Own/foreign classification is intrinsic to the wire
    /// id now (no per-header sink decision), so there is no `decide` hook.
    fn collect(rx: &mut TestRx) -> alloc::vec::Vec<Cap> {
        let mut out = alloc::vec::Vec::new();
        rx.poll(0, 160, 0, TEST_ID, |v| {
            match v {
                FrameVerdict::Instruction { instr, .. } => out.push(match instr {
                    Instruction::Ping { .. } => Cap::Ping,
                    Instruction::Write { address, data, .. } => Cap::Write {
                        address,
                        data: data.iter().collect(),
                    },
                    _ => Cap::OtherInstr,
                }),
                FrameVerdict::SkipComplete { id } => out.push(Cap::Skip { id }),
            }
            FrameAction::Continue
        });
        out
    }

    fn saw_instruction(caps: &[Cap]) -> bool {
        caps.iter().any(|c| !matches!(c, Cap::Skip { .. }))
    }

    #[test]
    fn poll_no_op_when_ring_empty() {
        let mut rx = make();
        let caps = collect(&mut rx);
        assert!(caps.is_empty());
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
        let caps = collect(&mut rx);
        assert!(!saw_instruction(&caps), "frame not complete yet");
        // The framer ticks instruction_count once, at classification.
        assert_eq!(rx.instruction_count(), 1);

        rx.stage_rx_bytes_for_test(split as u16, &pkt[split..]);
        let caps = collect(&mut rx);
        assert!(saw_instruction(&caps), "frame completes on the second poll");
        assert_eq!(rx.instruction_count(), 1, "same packet, no re-classify");
    }

    #[test]
    fn poll_decodes_own_write_in_one_verdict() {
        let mut rx = make();
        let wire = wire_write(TEST_ID, 0x0050, &[0xAA, 0xBB, 0xCC, 0xDD]);
        rx.stage_rx_bytes_for_test(0, &wire);

        let caps = collect(&mut rx);
        assert!(matches!(
            caps.as_slice(),
            [Cap::Write { address: 0x0050, data }] if data.as_slice() == [0xAA, 0xBB, 0xCC, 0xDD]
        ));
    }

    #[test]
    fn poll_skip_consumes_foreign_ping_body_and_crc() {
        let mut rx = make();
        let wire = wire_ping(FOREIGN_ID);
        rx.stage_rx_bytes_for_test(0, &wire);

        let caps = collect(&mut rx);
        // Only a SkipComplete carrying the foreign id — no instruction verdict.
        assert!(matches!(caps.as_slice(), [Cap::Skip { id }] if *id == FOREIGN_ID));
        // Wire cursor advanced over the full packet.
        assert_eq!(rx.wire_byte_cursor_for_test() as usize, wire.len());
    }

    #[test]
    fn poll_skip_consumes_foreign_write_body_and_crc() {
        let mut rx = make();
        let wire = wire_write(FOREIGN_ID, 0x0050, &[1, 2, 3, 4]);
        rx.stage_rx_bytes_for_test(0, &wire);

        let caps = collect(&mut rx);
        assert!(!saw_instruction(&caps), "foreign frame is never decoded");
        assert!(
            caps.iter()
                .any(|c| matches!(c, Cap::Skip { id } if *id == FOREIGN_ID))
        );
        assert_eq!(rx.wire_byte_cursor_for_test() as usize, wire.len());
    }

    #[test]
    fn poll_skip_complete_carries_frame_id() {
        let mut rx = make();
        let wire = wire_ping(FOREIGN_ID);
        rx.stage_rx_bytes_for_test(0, &wire);

        let caps = collect(&mut rx);
        let id = caps.iter().find_map(|c| match c {
            Cap::Skip { id } => Some(*id),
            _ => None,
        });
        assert_eq!(id, Some(FOREIGN_ID));
    }

    #[test]
    fn poll_skip_then_own_packet_decodes_clean() {
        let mut rx = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_write(TEST_ID, 0x0060, &[0x11, 0x22]);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        rx.stage_rx_bytes_for_test(0, &combined);

        let caps = collect(&mut rx);
        assert!(
            caps.iter()
                .any(|c| matches!(c, Cap::Skip { id } if *id == FOREIGN_ID))
        );
        assert!(caps.iter().any(|c| matches!(
            c,
            Cap::Write { address: 0x0060, data } if data.as_slice() == [0x11, 0x22]
        )));
    }

    #[test]
    fn wire_bytes_consumed_counts_copied_and_skipped_bytes() {
        let mut rx = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_ping(TEST_ID);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        rx.stage_rx_bytes_for_test(0, &combined);

        let _ = collect(&mut rx);
        assert_eq!(rx.wire_byte_cursor_for_test() as usize, combined.len());
    }

    #[test]
    fn instruction_count_ticks_on_every_instruction_frame() {
        let mut rx = make();
        let foreign = wire_ping(FOREIGN_ID);
        let own = wire_ping(TEST_ID);
        let mut combined: Vec<u8, 96> = Vec::new();
        combined.extend_from_slice(&foreign).unwrap();
        combined.extend_from_slice(&own).unwrap();
        rx.stage_rx_bytes_for_test(0, &combined);

        let _ = collect(&mut rx);
        // Two Instruction frames (foreign skip + own decode).
        assert_eq!(rx.instruction_count(), 2);
    }

    #[test]
    fn instruction_count_does_not_tick_on_status_frame() {
        let mut rx = make();
        let status = wire_status(TEST_ID);
        rx.stage_rx_bytes_for_test(0, &status);

        let _ = collect(&mut rx);
        assert_eq!(rx.instruction_count(), 0);
    }

    #[test]
    fn poll_drops_own_frame_with_bad_crc() {
        let mut rx = make();
        let mut wire = wire_ping(TEST_ID);
        let last = wire.len() - 1;
        wire[last] ^= 0xFF;
        rx.stage_rx_bytes_for_test(0, &wire);

        let caps = collect(&mut rx);
        // Silent drop: no instruction verdict surfaces for a bad-CRC frame.
        assert!(!saw_instruction(&caps));
    }

    #[test]
    fn poll_handles_ring_wrap_across_two_slices() {
        let mut rx = make();
        // Pre-position the read/write seqs so the packet straddles the
        // wrap boundary. RX_BUF_LEN = 64; start at seq 60 so 4 bytes fit
        // before wrap and the remainder lands at seq 0+.
        let wire = wire_write(TEST_ID, 0x0050, &[0xAA, 0xBB, 0xCC, 0xDD]);
        rx.set_rx_read_seq_for_test(60);
        rx.stage_rx_bytes_for_test(60, &wire);

        let caps = collect(&mut rx);
        assert!(
            caps.iter().any(|c| matches!(
                c,
                Cap::Write { address: 0x0050, data } if data.as_slice() == [0xAA, 0xBB, 0xCC, 0xDD]
            )),
            "expected decoded Write after wrap; got {caps:?}"
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
