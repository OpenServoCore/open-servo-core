//! Bytes ↔ packets. Composite over two field-projection halves — `CodecRx`
//! (decoder + RX byte ring + classifier + drift bookkeeping) and `CodecTx`
//! (encoder + TX byte ring) — joined under `Codec`. The split lets the
//! parent driver's closure-based `poll<F>` hand a parsed packet (borrowed
//! from `CodecRx`) and a reply handle (borrowed from `CodecTx`) to the
//! dispatcher closure simultaneously without a borrow conflict; outside
//! that hot path, `Codec`'s forwarder methods keep the simpler call sites
//! shape-unchanged.
//!
//! `Codec::poll_one` (delegating to `CodecRx`) surfaces an
//! [`InstructionToken`] (a `Copy` description of the wire location) per
//! decoded Instruction so the composite can walk BT pairs via
//! [`Codec::byte_pairs`] and apply its id-filter without holding a `&mut`
//! borrow that a returned `InstructionPacket<'_>` would imply. The packet
//! itself materializes on demand through [`Codec::dispatch`] (shared
//! borrow) so it can coexist with a disjoint `&mut CodecTx`.

pub mod rx;

use core::cell::SyncUnsafeCell;
use core::marker::PhantomData;

use dxl_protocol::decoder::{Decoder, Step};
use dxl_protocol::packet::{Slot, Status};
use dxl_protocol::{
    CrcUmts, InstructionPacket, SlotEmitter, SlotPosition, StatusEmitter, WriteError,
};

use crate::traits::dxl::DmaRing;
use crate::util::{HwRing, Seq};
use rx::Rx;

/// Copy-token describing a just-decoded Instruction's wire location.
/// `start` / `end` are RX seqs (one per UART byte); the composite hands
/// them to [`Codec::byte_pairs`] to walk the matching BT range. The token
/// is `Copy` and carries no borrow, so the composite can call
/// `byte_pairs(...)` + (conditionally) `dispatch()` after `poll_one`
/// returns without a borrow conflict.
#[derive(Copy, Clone, Debug)]
pub struct InstructionToken<const RX_BUF_LEN: usize> {
    pub id: u8,
    pub start: Seq<u8, RX_BUF_LEN>,
    pub end: Seq<u8, RX_BUF_LEN>,
}

/// RX half — decoder, RX byte ring, classifier, drift bookkeeping. Splits
/// off from [`CodecTx`] under [`Codec`] so the parent driver's closure-based
/// `poll<F>` can hand a packet borrowed from this half and a reply handle
/// borrowed from the TX half to the dispatcher at the same time.
pub struct CodecRx<
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
> {
    rx: Rx<R, EDGE_BUF_LEN, RX_BUF_LEN>,
    decoder: Decoder<DECODER_CAP, CRC>,
    /// DMA1_CH5 destination for received bytes. `SyncUnsafeCell` because
    /// USART1's DMA writes it concurrently with the parser's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware. [`HwRing`] enforces pow-2 sizing so
    /// `% RX_BUF_LEN` collapses to AND, and the chip-side ISR publishes
    /// the producer head via [`HwRing::on_publish`] from NDTR.
    rx_buf: SyncUnsafeCell<HwRing<u8, RX_BUF_LEN>>,
    /// Monotonic count of Instruction packets the decoder emitted —
    /// regardless of target ID — across this codec's lifetime. The drift
    /// signal ([[drift_sampling_instruction_only]]) ticks on every
    /// Instruction so foreign-target instructions still calibrate, while
    /// Status frames never contribute (peer HSI is its own clock domain).
    instruction_count: u32,
    /// Monotonic count of wire bytes consumed by `poll_one` across this
    /// codec's lifetime. Bumped once per `reader.advance(1)` so it tracks
    /// every byte the parser saw — including resync'd prefixes. Composite
    /// reads it at poll-time as the chain-CRC fold cursor for Fast Last
    /// replies (doc §10.6 — "snoop_head = rx_write_pos at parse-complete").
    /// 32 bits ≈ 23.8 days at 3M sustained — wrap is non-physical.
    wire_bytes_consumed: u32,
    /// RX seq of the first byte of the in-progress decoder packet. `None`
    /// whenever the decoder is between packets (fresh boot, post-Packet,
    /// post-Resync); lazy-set to the pre-advance read seq on the next byte
    /// `poll_one` consumes. Paired with the post-Packet read seq it bounds
    /// the BT range the composite feeds into [`super::Clock::on_byte_pair`].
    ///
    /// Carries `Option` rather than a sentinel because [`Seq`]'s raw u16
    /// wraps over the full value range — every potential sentinel is a
    /// live cursor for one byte every 65536 RX bytes (~175 ms at 3 M).
    packet_start_rx_seq: Option<Seq<u8, RX_BUF_LEN>>,
}

impl<
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
> CodecRx<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>
{
    fn new(ring: R) -> Self {
        Self {
            rx: Rx::new(ring),
            decoder: Decoder::new(),
            rx_buf: SyncUnsafeCell::new(HwRing::new(0)),
            instruction_count: 0,
            wire_bytes_consumed: 0,
            packet_start_rx_seq: None,
        }
    }

    /// New RX falling-edge timestamps may be available — forward
    /// `ticks_per_bit` (pulled from the clock by the composite) into the
    /// RX classifier so its HIT window matches the current baud.
    pub fn on_edge_advance(&mut self, ticks_per_bit: u16) {
        self.rx.on_edge_advance(ticks_per_bit);
    }

    /// USART1 IDLE backstop — drain tail edges and reset the classifier
    /// anchor for the next burst.
    pub fn on_idle(&mut self, ticks_per_bit: u16) {
        self.rx.on_idle(ticks_per_bit);
    }

    /// USART1 RX DMA published progress — `remaining` is the channel's
    /// NDTR readback. Advances the `rx_buf` producer head so `poll_one`
    /// sees newly-DMA'd bytes.
    pub fn on_rx_dma_advance(&mut self, remaining: u16) {
        // SAFETY: rx_buf is written only by DMA1_CH5 (hardware writer)
        // and read here from the same PFIC priority level as the DMA
        // HT/TC ISR, so no other consumer can `&mut` it concurrently.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        rx_buf.on_publish(remaining);
    }

    /// Drain bytes from `rx_buf` through the streaming decoder; return on
    /// the next Instruction (own *or* foreign — composite filters by ID).
    /// Status frames continue past silently. `None` when the byte ring is
    /// exhausted before another Instruction completes — the decoder yields
    /// with its in-progress state intact and resumes on the next call.
    ///
    /// Bumps `instruction_count` per emitted Instruction. Lazy-seeds
    /// `packet_start_rx_seq` on the first new byte; clears on `Step::Resync`
    /// and on every `Step::Packet`. The returned [`InstructionToken`] is
    /// `Copy` — its borrow on `&mut self` ends at the call's statement
    /// boundary, so the composite is free to call [`Self::byte_pairs`] and
    /// (conditionally) [`Self::dispatch`] after.
    pub fn poll_one(&mut self) -> Option<InstructionToken<RX_BUF_LEN>> {
        // SAFETY: see `on_rx_dma_advance`.
        let rx_buf = unsafe { &mut *self.rx_buf.get() };
        let mut reader = rx_buf.reader();
        while let Some(&byte) = reader.peek() {
            // Lazy-seed packet_start on the first byte of each new packet
            // hypothesis. After Resync / Packet the field is cleared back
            // to None; the next byte we consume is, by definition, the
            // first byte of whatever the decoder will parse next.
            let pre_advance = reader.read_seq();
            self.packet_start_rx_seq.get_or_insert(pre_advance);
            reader.advance(1);
            self.wire_bytes_consumed = self.wire_bytes_consumed.wrapping_add(1);
            let (step, _) = self.decoder.feed(&[byte]);
            match step {
                Step::NeedMore => continue,
                Step::Resync(_) => {
                    // Decoder dropped the in-progress prefix; clear so the
                    // next byte re-seeds.
                    self.packet_start_rx_seq = None;
                    continue;
                }
                Step::Packet(pkt) => {
                    let end_seq = reader.read_seq();
                    // Always Some here — get_or_insert at the loop top
                    // seeded it this iteration (or kept an earlier seed).
                    // Take to clear so the next iteration starts a fresh
                    // hypothesis.
                    let start_seq = self.packet_start_rx_seq.take().unwrap_or(end_seq);
                    if let Some(ip) = pkt.into_instruction_packet() {
                        let id = ip.id().as_byte();
                        self.instruction_count = self.instruction_count.wrapping_add(1);
                        return Some(InstructionToken {
                            id,
                            start: start_seq,
                            end: end_seq,
                        });
                    }
                    // Status frames: don't surface, no drift (peer HSI is
                    // its own clock domain — [[drift_sampling_instruction_only]]).
                    // Decoder is back in NeedMore on the next feed; loop continues.
                }
            }
        }
        None
    }

    /// Re-derive the most recent decoded packet through the decoder's
    /// immutable `dispatch_packet` path. The decoder stays in `Done`
    /// between `feed()` calls, so the overlay is still valid for as long
    /// as no new `poll_one` runs. Shared `&self` so the parent's closure
    /// can hold the packet (which borrows the decoder's buffer through
    /// `&self`) alongside a `&mut CodecTx` reply handle.
    pub fn dispatch(&self) -> Option<InstructionPacket<'_>> {
        self.decoder.dispatch_packet().into_instruction_packet()
    }

    /// Iterate consecutive `(prev, curr)` BT pairs across the RX seq
    /// range. Converts the RX-typed seqs to BT-typed (shared seq space
    /// per doc §8.3) at the boundary and forwards to
    /// [`rx::Rx::byte_pairs`]. Composite calls this on each
    /// [`InstructionToken`]'s span and routes each pair to
    /// `Clock::on_byte_pair`.
    pub fn byte_pairs(
        &self,
        start: Seq<u8, RX_BUF_LEN>,
        end: Seq<u8, RX_BUF_LEN>,
    ) -> impl Iterator<Item = (u16, u16)> + '_ {
        self.rx.byte_pairs(start.into(), end.into())
    }

    /// Monotonic count of Instruction packets the decoder has emitted.
    /// Foreign IDs count too (composite filters at the surface);
    /// Status frames don't.
    pub fn instruction_count(&self) -> u32 {
        self.instruction_count
    }

    /// Cumulative wire-byte cursor (parser-consumed) for chain-CRC fold
    /// anchoring. Composite captures this at poll surface as the Fast Last
    /// scheduler's `anchor_bytes` input — see doc §10.6.
    pub fn wire_byte_cursor(&self) -> u32 {
        self.wire_bytes_consumed
    }

    /// Sequence number of the next BT slot to write — one past the last
    /// published BT entry.
    pub fn byte_ts_head(&self) -> Seq<u16, RX_BUF_LEN> {
        self.rx.byte_ts_head()
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

    /// Look up the start-bit tick of the byte at `seq`. The DXL composite
    /// pairs this with `Clock::ticks_per_bit()` to derive
    /// `wire_end_tick = BT[token.end.predecessor()] + 10·tpb` for fire
    /// scheduling. Returns `None` if `seq` is past the BT head or has
    /// lapped out of the ring window.
    pub fn byte_ts_at(&self, seq: Seq<u8, RX_BUF_LEN>) -> Option<u16> {
        // RX and BT share a seq space per doc §8.3 — `.into()` retags
        // the type, raw is preserved.
        self.rx.byte_ts_at(seq.into())
    }
}

/// TX half — encoder + TX byte ring. Splits off from [`CodecRx`] under
/// [`Codec`] so the parent driver's closure-based `poll<F>` can hand the
/// dispatcher's reply handle a `&mut CodecTx` alongside a packet that
/// borrows the disjoint RX half.
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
    /// contents first, then drives [`StatusEmitter`] over `tx_buf` —
    /// `Vec::push` propagates `WriteError::Overflow` if the encoded form
    /// exceeds `TX_BUF_LEN`. The composite reads [`Self::tx_len`] after
    /// for the DMA transfer count.
    pub fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        self.tx_buf.clear();
        StatusEmitter::<_, CRC>::new(&mut self.tx_buf).emit(status)
    }

    /// Encode one Fast slot reply into the TX buffer. Same buffer-clear
    /// and emitter shape as [`Self::send_status`]; [`SlotEmitter::emit`]
    /// dispatches on `position` (Only/First/Middle/Last) and writes the
    /// header, payload, and (locally-computed or caller-supplied) CRC.
    pub fn send_slot(&mut self, slot: &Slot<'_>, position: SlotPosition) -> Result<(), WriteError> {
        self.tx_buf.clear();
        SlotEmitter::<_, CRC>::new(&mut self.tx_buf).emit(slot, position)
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
}

pub struct Codec<
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> {
    pub(super) rx: CodecRx<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>,
    pub(super) tx: CodecTx<CRC, TX_BUF_LEN>,
}

impl<
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> Codec<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    pub fn new(ring: R) -> Self {
        Self {
            rx: CodecRx::new(ring),
            tx: CodecTx::new(),
        }
    }

    /// Disjoint mutable borrow of the RX and TX halves. The parent driver's
    /// closure-based `poll<F>` uses this to hand a packet (borrowed via
    /// `CodecRx::dispatch`, which takes `&self`) and a `&mut CodecTx` reply
    /// handle to the dispatcher at the same time.
    pub fn split_mut(
        &mut self,
    ) -> (
        &mut CodecRx<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>,
        &mut CodecTx<CRC, TX_BUF_LEN>,
    ) {
        (&mut self.rx, &mut self.tx)
    }

    // ----- Forwarders. Keep sequential single-half call sites compact
    // without forcing every caller to disambiguate `.rx` / `.tx`. -----

    pub fn on_edge_advance(&mut self, ticks_per_bit: u16) {
        self.rx.on_edge_advance(ticks_per_bit);
    }

    pub fn on_idle(&mut self, ticks_per_bit: u16) {
        self.rx.on_idle(ticks_per_bit);
    }

    pub fn on_rx_dma_advance(&mut self, remaining: u16) {
        self.rx.on_rx_dma_advance(remaining);
    }

    pub fn poll_one(&mut self) -> Option<InstructionToken<RX_BUF_LEN>> {
        self.rx.poll_one()
    }

    pub fn dispatch(&self) -> Option<InstructionPacket<'_>> {
        self.rx.dispatch()
    }

    pub fn byte_pairs(
        &self,
        start: Seq<u8, RX_BUF_LEN>,
        end: Seq<u8, RX_BUF_LEN>,
    ) -> impl Iterator<Item = (u16, u16)> + '_ {
        self.rx.byte_pairs(start, end)
    }

    pub fn instruction_count(&self) -> u32 {
        self.rx.instruction_count()
    }

    pub fn wire_byte_cursor(&self) -> u32 {
        self.rx.wire_byte_cursor()
    }

    pub fn byte_ts_head(&self) -> Seq<u16, RX_BUF_LEN> {
        self.rx.byte_ts_head()
    }

    pub fn edges_addr(&self) -> usize {
        self.rx.edges_addr()
    }

    pub fn rx_buf_addr(&self) -> usize {
        self.rx.rx_buf_addr()
    }

    pub fn byte_ts_at(&self, seq: Seq<u8, RX_BUF_LEN>) -> Option<u16> {
        self.rx.byte_ts_at(seq)
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
}

#[cfg(test)]
impl<
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
> CodecRx<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>
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
}

#[cfg(test)]
impl<
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> Codec<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    pub(crate) fn stage_rx_bytes_for_test(&mut self, at: u16, bytes: &[u8]) {
        self.rx.stage_rx_bytes_for_test(at, bytes);
    }

    pub(crate) fn set_rx_read_seq_for_test(&mut self, seq: u16) {
        self.rx.set_rx_read_seq_for_test(seq);
    }
}

#[cfg(test)]
impl<const EDGE_BUF_LEN: usize, const RX_BUF_LEN: usize, CRC: CrcUmts, const DECODER_CAP: usize>
    CodecRx<crate::mocks::FakeDmaRing, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>
{
    pub(crate) fn stage_edges_for_test(&mut self, vals: &[u16]) {
        self.rx.stage_edges_for_test(vals);
    }

    pub(crate) fn arm_next_flags_for_test(&mut self, flags: crate::traits::dxl::DmaFlags) {
        self.rx.arm_next_flags_for_test(flags);
    }
}

#[cfg(test)]
impl<
    const EDGE_BUF_LEN: usize,
    const RX_BUF_LEN: usize,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const TX_BUF_LEN: usize,
> Codec<crate::mocks::FakeDmaRing, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    /// Drive the inner `Rx`'s edge buffer staging + flag-arming through
    /// the codec. Lets composite tests prepare the BT ring without
    /// reaching past the codec boundary.
    pub(crate) fn stage_edges_for_test(&mut self, vals: &[u16]) {
        self.rx.stage_edges_for_test(vals);
    }

    pub(crate) fn arm_next_flags_for_test(&mut self, flags: crate::traits::dxl::DmaFlags) {
        self.rx.arm_next_flags_for_test(flags);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::FakeDmaRing;
    use dxl_protocol::packet::{Id, StatusError};
    use dxl_protocol::{InstructionEmitter, SoftwareCrcUmts, StatusEmitter};
    use heapless::Vec;

    const DECODER_CAP: usize = 256;
    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;
    /// `DXL_TX_MAX_BYTES` per `osc-core::services::dxl::limits` — chip-side
    /// registry uses the same value.
    const TX_BUF_LEN: usize = 140;
    const TEST_ID: u8 = 0x07;

    type TestCodec =
        Codec<FakeDmaRing, SoftwareCrcUmts, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

    fn make() -> TestCodec {
        Codec::new(FakeDmaRing::default())
    }

    fn wire_ping(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .ping(Id::new(id))
            .unwrap();
        out
    }

    fn wire_status(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        StatusEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .empty(Id::new(id), dxl_protocol::packet::StatusError::OK)
            .unwrap();
        out
    }

    #[test]
    fn poll_one_returns_none_when_no_new_bytes() {
        let mut c = make();
        assert!(c.poll_one().is_none());
    }

    #[test]
    fn poll_one_yields_token_per_instruction_without_id_filter() {
        let mut c = make();
        let ours = wire_ping(TEST_ID);
        let theirs = wire_ping(0x42);
        let mut combined: Vec<u8, 64> = Vec::new();
        combined.extend_from_slice(&ours).unwrap();
        combined.extend_from_slice(&theirs).unwrap();
        c.stage_rx_bytes_for_test(0, &combined);

        // First Instruction — own id.
        let t1 = c.poll_one().expect("first instruction");
        assert_eq!(t1.id, TEST_ID);
        // Second Instruction — foreign id; codec doesn't filter.
        let t2 = c.poll_one().expect("second instruction");
        assert_eq!(t2.id, 0x42);
        // Nothing else queued.
        assert!(c.poll_one().is_none());
        assert_eq!(c.instruction_count(), 2);
    }

    #[test]
    fn poll_one_drains_status_without_emitting_token() {
        let mut c = make();
        let status = wire_status(TEST_ID);
        c.stage_rx_bytes_for_test(0, &status);

        assert!(c.poll_one().is_none());
        // Status doesn't count — peer HSI is its own clock domain.
        assert_eq!(c.instruction_count(), 0);
    }

    #[test]
    fn poll_one_clears_packet_start_on_resync() {
        let mut c = make();
        let mut bad = wire_ping(TEST_ID);
        let crc_lo = bad.len() - 2;
        bad[crc_lo] ^= 0xFF;
        let good = wire_ping(TEST_ID);

        let mut combined: Vec<u8, 64> = Vec::new();
        combined.extend_from_slice(&bad).unwrap();
        combined.extend_from_slice(&good).unwrap();
        c.stage_rx_bytes_for_test(0, &combined);

        let token = c.poll_one().expect("good instruction after resync");
        assert_eq!(token.id, TEST_ID);
        // The token's start must be inside the GOOD frame's RX range, not
        // anchored at the bad frame's start — Resync cleared the cursor.
        let good_start = bad.len() as u16;
        let good_end = (bad.len() + good.len()) as u16;
        // start is RX-typed Seq; we compare via test_raw. The decoder may
        // re-anchor mid-bad-frame, so the precise raw value isn't fixed —
        // assert it's strictly inside the good frame's wire range.
        let s = token.start.test_raw();
        assert!(
            s >= good_start && s < good_end,
            "start {s} not in good frame [{good_start}, {good_end})"
        );
    }

    #[test]
    fn dispatch_surfaces_token_packet() {
        let mut c = make();
        let ping = wire_ping(TEST_ID);
        c.stage_rx_bytes_for_test(0, &ping);

        let token = c.poll_one().expect("instruction");
        assert_eq!(token.id, TEST_ID);
        match c.dispatch() {
            Some(InstructionPacket::Ping(p)) => assert_eq!(p.header.id.as_byte(), TEST_ID),
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn byte_pairs_iterates_consecutive_bt_entries() {
        let mut c = make();
        // Stage 4 edges spaced by one byte-time @ 3M (tpb=16 → 160 ticks
        // per byte), publish them so the classifier emits 4 BT entries.
        let edges: [u16; 4] = [1000, 1160, 1320, 1480];
        c.stage_edges_for_test(&edges);
        c.arm_next_flags_for_test(crate::traits::dxl::DmaFlags {
            ht: true,
            tc: false,
        });
        c.on_edge_advance(16);
        assert_eq!(c.byte_ts_head().test_raw(), 4);

        let start: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(0);
        let end: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(4);
        let pairs: heapless::Vec<(u16, u16), 8> = c.byte_pairs(start, end).collect();
        assert_eq!(
            pairs.as_slice(),
            &[(1000, 1160), (1160, 1320), (1320, 1480)]
        );
    }

    #[test]
    fn byte_pairs_stops_at_first_lapped_seq() {
        let mut c = make();
        // Only 2 BTs published; ask for range covering 4. Iterator must
        // stop at the first lapped lookup rather than skipping past it
        // and producing a "consecutive" pair that isn't.
        let edges: [u16; 2] = [1000, 1160];
        c.stage_edges_for_test(&edges);
        c.arm_next_flags_for_test(crate::traits::dxl::DmaFlags {
            ht: true,
            tc: false,
        });
        c.on_edge_advance(16);

        let start: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(0);
        let end: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(4);
        let pairs: heapless::Vec<(u16, u16), 8> = c.byte_pairs(start, end).collect();
        // Only one valid pair from BT[0..2].
        assert_eq!(pairs.as_slice(), &[(1000, 1160)]);
    }

    #[test]
    fn send_status_writes_wire_bytes_into_tx_buf() {
        let mut c = make();
        c.send_status(Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        })
        .expect("encode fits");

        // Round-trip the encoded bytes through the same emitter via a
        // reference Vec — the codec's tx_buf must match byte-for-byte.
        let mut expected: Vec<u8, TX_BUF_LEN> = Vec::new();
        StatusEmitter::<_, SoftwareCrcUmts>::new(&mut expected)
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
        // First emit a wider payload (Ping reply) to push the head past
        // what a subsequent Empty would write.
        c.send_status(Status::Ping {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            status: dxl_protocol::packet::PingStatus {
                model: dxl_protocol::packet::U16Le::from_u16(0x0123),
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

        // Empty Status is smaller than Ping Status — second write must
        // shorten tx_len, proving the buffer was cleared (not appended).
        assert!(second_len < first_len);

        // Bytes at offset 0 are the new packet's header, not leftover
        // tail of the previous one.
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
        // packet_length = 3 (cmd+err+reserved) + 3 (payload) + 2 (CRC) = 8.
        c.send_slot(&slot, SlotPosition::Only { packet_length: 8 })
            .expect("encode fits");

        // SAFETY: see `send_status_writes_wire_bytes_into_tx_buf`.
        let actual = unsafe {
            core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, c.tx_len() as usize)
        };
        // Wire-layout sanity: DXL 2.0 header begins `FF FF FD 00`.
        assert_eq!(&actual[0..4], &[0xFF, 0xFF, 0xFD, 0x00]);

        // Round-trip via a reference SlotEmitter so byte-for-byte equality
        // covers length / cmd / err / reserved / payload / CRC.
        let mut expected: Vec<u8, TX_BUF_LEN> = Vec::new();
        SlotEmitter::<_, SoftwareCrcUmts>::new(&mut expected)
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
        // SlotPosition::Last writes ID + error + payload + caller CRC.
        c.send_slot(&slot, SlotPosition::Last { crc: 0xDEAD })
            .expect("encode fits");

        let len = c.tx_len() as usize;
        // SAFETY: see `send_status_writes_wire_bytes_into_tx_buf`.
        let actual = unsafe { core::slice::from_raw_parts(c.tx_buf_addr() as *const u8, len) };
        // Trailing two bytes are the caller-supplied CRC, little-endian.
        assert_eq!(&actual[len - 2..], &[0xAD, 0xDE]);
    }

    #[test]
    fn byte_ts_at_returns_published_entry() {
        let mut c = make();
        let edges: [u16; 3] = [1000, 1160, 1320];
        c.stage_edges_for_test(&edges);
        c.arm_next_flags_for_test(crate::traits::dxl::DmaFlags {
            ht: true,
            tc: false,
        });
        c.on_edge_advance(16);
        assert_eq!(c.byte_ts_head().test_raw(), 3);

        let s0: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(0);
        let s1: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(1);
        let s2: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(2);
        assert_eq!(c.byte_ts_at(s0), Some(1000));
        assert_eq!(c.byte_ts_at(s1), Some(1160));
        assert_eq!(c.byte_ts_at(s2), Some(1320));
    }

    #[test]
    fn byte_ts_at_returns_none_past_head() {
        let mut c = make();
        c.stage_edges_for_test(&[1000, 1160]);
        c.arm_next_flags_for_test(crate::traits::dxl::DmaFlags {
            ht: true,
            tc: false,
        });
        c.on_edge_advance(16);
        assert_eq!(c.byte_ts_head().test_raw(), 2);

        // Seq 2 is at the head — not yet written.
        let past: Seq<u8, RX_BUF_LEN> = Seq::test_from_raw(2);
        assert_eq!(c.byte_ts_at(past), None);
    }
}
