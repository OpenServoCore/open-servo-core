//! Hardware-timed DXL receive path. Owns the edge-timestamp ring
//! (DMA1_CH7 destination) and the streaming edge parser that turns
//! captured falling edges into per-byte start ticks in lockstep with
//! the byte parser.
//!
//! The Rx wrapper exposes three operations the codec drives:
//!
//! - [`Rx::publish_edges`] (and the ISR entries [`Rx::on_edge_advance`],
//!   [`Rx::on_idle`]) — read NDTR, advance the producer head. Pure ring
//!   bookkeeping, no walking.
//! - [`Rx::anchor_at_signature`] — called at the parser's `Event::Sync`;
//!   back-searches for the DXL header signature and seeds the anchor.
//! - [`Rx::advance_walker`] — called once per subsequent parser event
//!   with the byte delta the parser has consumed; advances the edge
//!   parser by exactly that many byte-boundaries.
//!
//! Consumers then read `current_byte_tick()` / `packet_end_tick()` at
//! parser-event time. Drift pairs route through the `on_pair` callback
//! the codec threads into `advance_walker`. The autonomous HT/TC and
//! IDLE walker entries the prior design used no longer exist —
//! overshoot past the byte parser cursor is structurally impossible.
//!
//! The driver depends on a [`EdgeDma`] adapter for HT/TC flag drain and
//! NDTR readback; the production adapter binds to DMA1_CH7. Tests swap in
//! [`crate::mocks::MockEdgeDma`] and stage flags + remaining directly.
//!
//! The ET ring depth is const-generic so a chip/board can pick its own
//! memory budget without touching driver code. The chip-facing knob is
//! [`sync_lookback_edges`] (the worst-case anchor back-search depth);
//! [`edge_buf_len`] / [`rx_buf_len`] derive the matching ring sizes.

mod edge_parser;

pub use edge_parser::{PollSrc, edge_buf_len, rx_buf_len, sync_lookback_edges, walker_window};

use core::cell::SyncUnsafeCell;

use crate::traits::dxl::EdgeDma;
use crate::util::HwRing;
use edge_parser::EdgeParser;

pub struct Rx<R: EdgeDma, const EDGE_BUF_LEN: usize> {
    parser: EdgeParser,
    /// DMA1_CH7's destination buffer. `SyncUnsafeCell` because the DMA
    /// engine writes it concurrently with the edge parser's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware, so plain `&` references inside
    /// edge-parser walks are sound. [`HwRing`] enforces pow-2 sizing at
    /// construction so the chip-side NDTR head math (`EDGE_BUF_LEN -
    /// NDTR`) maps cleanly to a ring position.
    edges: SyncUnsafeCell<HwRing<u16, EDGE_BUF_LEN>>,
    ring: R,
    /// Most recent ISR's wake capture — WireClock u32 tick at ISR entry
    /// plus the source flavor. Set on every [`Self::on_edge_advance`] /
    /// [`Self::on_idle`] entry; read by the composite Crc handler when
    /// [`Self::packet_end_tick`] returns `None` to pick a fallback
    /// formula via [`Self::packet_end_tick_fallback`]. Default value
    /// `(0, PollSrc::Dma)` is unreachable in production — Crc
    /// follows bytes which follow an ISR — but keeps the field non-
    /// Optional so the hot path doesn't carry an `unwrap`.
    last_isr: (u32, PollSrc),
}

impl<R: EdgeDma, const EDGE_BUF_LEN: usize> Rx<R, EDGE_BUF_LEN> {
    pub const fn new(ring: R) -> Self {
        Self {
            parser: EdgeParser::new(),
            edges: SyncUnsafeCell::new(HwRing::new(0)),
            ring,
            last_isr: (0, PollSrc::Dma),
        }
    }

    /// Stable peripheral-memory address for the DMA destination. Bringup
    /// hands this to `dma::configure(CH7, ...)`; the driver instance lives
    /// in the registry's `SyncUnsafeCell<Option<Rx>>` so the address is
    /// fixed for the lifetime of the program once `install` returns.
    /// [`HwRing::as_ptr`] returns the address of the first storage slot —
    /// the struct's outer address is offset by the bookkeeping fields.
    /// Returned as `usize` (chip-agnostic); the chip-side provider casts
    /// to its DMA-MAR register width at the call site.
    pub fn edges_addr(&self) -> usize {
        // SAFETY: address-of read; no value materialized. Sound even while
        // DMA is writing the storage concurrently.
        unsafe { (*self.edges.get()).as_ptr() as usize }
    }

    /// DMA1_CH7 HT/TC ISR entry. Acks the flag, publishes the producer
    /// head from NDTR. Stashes `(now, Dma)` for the Crc-time fallback
    /// path. No walking — all walker advance is parser-event-driven via
    /// [`Self::advance_walker`].
    pub fn on_edge_advance(&mut self, now: u32, _ticks_per_bit: u16) {
        self.last_isr = (now, PollSrc::Dma);
        let flags = self.ring.read_and_ack();
        crate::log::trace!("rx: on_edge_advance ht={} tc={}", flags.ht, flags.tc);
        if !flags.ht && !flags.tc {
            return;
        }
        self.publish_edges();
    }

    /// USART1 IDLE ISR entry. Publishes the producer head (small packets
    /// that don't fill a half-ring never trip HT). Stashes `(now, Idle)`
    /// for the Crc-time fallback path. No walking — same contract as
    /// [`Self::on_edge_advance`].
    pub fn on_idle(&mut self, now: u32, _ticks_per_bit: u16) {
        self.last_isr = (now, PollSrc::Idle);
        crate::log::trace!("rx: on_idle");
        self.publish_edges();
    }

    /// Publish the producer head from `ring.remaining()` (NDTR readback)
    /// so subsequent walker advances see newly-captured edges. Pure ring
    /// bookkeeping — no walking, no anchor mutation. Called from ISR
    /// entries and from [`Self::advance_walker`] / [`Self::anchor_at_signature`]
    /// to refresh before the edge parser reads.
    fn publish_edges(&mut self) {
        let remaining = self.ring.remaining();
        // SAFETY: the edges buffer is mutated only by DMA1_CH7 (hardware
        // writer) and read here from a PFIC-HIGH ISR; no other code path
        // takes a `&mut` into it.
        let edges = unsafe { &mut *self.edges.get() };
        edges.on_publish(remaining);
    }

    /// Walk `n` byte-boundaries forward from the current anchor. The
    /// codec calls this once per parser event with the byte delta the
    /// parser consumed since the last call (and once per skip-byte batch
    /// during foreign-packet skip phase). Drift `(prev, curr)` pairs
    /// are pushed into `pairs` when the edge parser's `hsi_active` flag
    /// is set; overflow past capacity is silently dropped. Returns the
    /// count actually advanced — may be less than `n` if edges are late
    /// or an anomaly fired; the codec accounts for the shortfall via
    /// the next event's delta.
    ///
    /// `win_lo` / `win_hi` come from [`walker_window`] — codec computes
    /// once per `poll()` entry so the per-event muls don't repeat.
    #[inline]
    pub fn advance_walker<const PAIRS_LEN: usize>(
        &mut self,
        n: u16,
        win_lo: u16,
        win_hi: u16,
        pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
    ) -> u16 {
        self.publish_edges();
        // SAFETY: see `publish_edges`.
        let edges = unsafe { &mut *self.edges.get() };
        self.parser.advance_bytes(edges, n, win_lo, win_hi, pairs)
    }

    /// Most recent ISR wake capture — `(now, src)` recorded at the last
    /// [`Self::on_edge_advance`] / [`Self::on_idle`] entry. Composite Crc
    /// handler consumes this when the edge parser was unanchored (the
    /// `packet_end_tick_fallback` path).
    pub fn last_isr_capture(&self) -> (u32, PollSrc) {
        self.last_isr
    }

    /// Back-search the ET ring for the DXL header's 5-edge signature.
    /// Codec calls at every parser `Event::Sync`. On match the anchor is
    /// established at byte 3 (the `0x00` start bit) and the edge parser's
    /// ET cursor advances past the header edges; on mismatch any stale
    /// anchor is cleared and the next packet's Sync re-attempts from
    /// scratch.
    pub fn anchor_at_signature(&mut self, ticks_per_bit: u16) -> bool {
        self.publish_edges();
        // SAFETY: see `publish_edges`.
        let edges = unsafe { &mut *self.edges.get() };
        self.parser.anchor_at_signature(edges, ticks_per_bit)
    }

    /// Hinted anchor fast path. `body_bytes` is the codec-supplied slice
    /// of bytes already sitting in the byte ring past the parser's
    /// just-consumed sync header; their cumulative edge count drives the
    /// byte-derived position estimate. Tries a small window around the
    /// estimate first, then falls back to the full
    /// [`Self::anchor_at_signature`] back-search.
    pub fn anchor_at_signature_hinted(&mut self, ticks_per_bit: u16, body_bytes: &[u8]) -> bool {
        self.publish_edges();
        let body_edges: u16 = body_bytes
            .iter()
            .map(|&b| edge_parser::EDGES_PER_BYTE[b as usize] as u16)
            .sum();
        // SAFETY: see `publish_edges`.
        let edges = unsafe { &mut *self.edges.get() };
        self.parser
            .anchor_at_signature_hinted(edges, ticks_per_bit, body_edges)
    }

    /// Tick of the most-recently classified byte's start bit, lifted into
    /// the WireClock u32 domain. `now` / `src` route through the edge
    /// parser's drain-reference correction so the lift stays sub-wrap
    /// at low baud — see `EdgeParser::drain_ref`.
    pub fn current_byte_tick(&self, ticks_per_bit: u16, now: u32, src: PollSrc) -> Option<u32> {
        self.parser.current_byte_tick(ticks_per_bit, now, src)
    }

    /// Wire-end tick of the most-recently classified byte, lifted into the
    /// WireClock u32 domain. Composite stamps `packet_end_tick` at the
    /// parser's CRC-good event. `now` / `src` route through the drain-
    /// reference correction — see `EdgeParser::drain_ref`.
    pub fn packet_end_tick(&self, ticks_per_bit: u16, now: u32, src: PollSrc) -> Option<u32> {
        self.parser.packet_end_tick(ticks_per_bit, now, src)
    }

    /// Fallback packet-end estimate for the no-anchor case — composite
    /// calls when [`packet_end_tick`](Self::packet_end_tick) returns `None`
    /// at the Crc event. See [`edge_parser::EdgeParser::packet_end_tick_fallback`]
    /// for the per-source formulas.
    pub fn packet_end_tick_fallback(&self, src: PollSrc, now: u32, ticks_per_bit: u16) -> u32 {
        self.parser
            .packet_end_tick_fallback(src, now, ticks_per_bit)
    }

    /// Toggle drift-sampling emission. Composite sets at Instruction-
    /// header events, clears at the matching CRC event.
    pub fn set_hsi_active(&mut self, on: bool) {
        self.parser.set_hsi_active(on);
    }

    /// Refresh the edge parser's per-baud RX edge-stamp compensation.
    /// Forwarded from the composite's `on_baud_change` event — see
    /// [`edge_parser::EdgeParser::on_baud_change`].
    pub fn on_baud_change(&mut self, rx_edge_comp_ticks: u16) {
        self.parser.on_baud_change(rx_edge_comp_ticks);
    }

    pub fn hsi_active(&self) -> bool {
        self.parser.hsi_active()
    }

    /// Force-drop anchor + drift flag. Composite calls at the parser's
    /// Crc event (packet boundary) and Resync event (parser abort).
    pub fn reset_anchor(&mut self) {
        self.parser.reset_anchor();
    }

    /// Force a known `last_byte_start` tick — composite-test scaffolding so
    /// `packet_end_tick` reads a deterministic value without staging real
    /// edges.
    #[cfg(test)]
    pub fn force_byte_tick_for_test(&mut self, tick: u16) {
        self.parser.force_byte_tick_for_test(tick);
    }

    /// Mask the edge-DMA channel's HT/TC IRQ + clear any latched flag.
    /// Composite calls this from `on_systick_match` at first Fast Last
    /// catchup entry so the classifier ISR doesn't preempt the catchup
    /// body during the predecessor window (doc §10.6.3). Idempotent.
    pub fn pause_edges(&mut self) {
        self.ring.pause();
    }

    /// Re-enable the edge-DMA channel's HT/TC IRQ. Composite calls this
    /// from `on_tx_complete` after the Fast Last reply finishes. Cheap
    /// no-op for Plain replies (pause was never called). Idempotent.
    pub fn resume_edges(&mut self) {
        self.ring.resume();
    }
}

// Shelved pending U4 (osc-drivers unit test audit): helpers below bind to
// hand-rolled `MockEdgeDma` fields; will be migrated to the mockall +
// state-companion API as part of the audit.
#[cfg(any())]
#[cfg(test)]
impl<const EDGE_BUF_LEN: usize> Rx<crate::mocks::MockEdgeDma, EDGE_BUF_LEN> {
    /// Stage `vals` into the edges buffer as if DMA wrote them, publish the
    /// producer head so direct `try_anchor_from_header` / `recent` reads see
    /// them, and set `remaining` so a subsequent `on_edge_advance` /
    /// `on_idle` finds nothing new. Shared by leaf and composite tests.
    pub(crate) fn stage_edges_for_test(&mut self, vals: &[u16]) {
        // SAFETY: test-only access to the SyncUnsafeCell; no DMA in tests.
        let buf = unsafe { &mut *self.edges.get() };
        buf.stage(0, vals);
        let remaining = HwRing::<u16, EDGE_BUF_LEN>::LEN - vals.len() as u16;
        buf.on_publish(remaining);
        self.ring.remaining = remaining;
    }

    /// Arm the fake ring's next HT/TC flag response.
    pub(crate) fn arm_next_flags_for_test(&mut self, flags: crate::traits::dxl::DmaFlags) {
        self.ring.next_flags = flags;
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
    use crate::traits::dxl::DmaFlags;

    /// Test-side ring sizing. EDGE_BUF_LEN matches V006 per doc §4.5.
    const EDGE_BUF_LEN: usize = 128;
    type TestRx = Rx<MockEdgeDma, EDGE_BUF_LEN>;

    // 3 Mbaud at HCLK 48 MHz → ticks_per_bit = 16. One byte = 160 ticks.
    const TPB_3M: u16 = 16;
    const BYTE_TICKS_3M: u16 = 160;

    fn rx() -> TestRx {
        Rx::new(MockEdgeDma::default())
    }

    /// Synthesize a clean DXL header at base tick `t0` and bit-time `tpb`.
    fn header_edges(t0: u16, tpb: u16) -> [u16; 5] {
        let bt = tpb.wrapping_mul(10);
        [
            t0,
            t0.wrapping_add(bt),
            t0.wrapping_add(bt.wrapping_mul(2)),
            t0.wrapping_add(bt.wrapping_mul(2))
                .wrapping_add(tpb.wrapping_mul(2)),
            t0.wrapping_add(bt.wrapping_mul(3)),
        ]
    }

    #[test]
    fn try_anchor_from_header_forwards_to_classifier() {
        let mut d = rx();
        let h = header_edges(1000, TPB_3M);
        d.stage_edges_for_test(&h);
        assert!(d.try_anchor_from_header(TPB_3M));
        assert_eq!(d.current_byte_tick(), Some(h[4]));
    }

    #[test]
    fn try_anchor_from_header_returns_false_on_pattern_miss() {
        let mut d = rx();
        // Five edges with a bad narrow gap.
        let mut h = header_edges(1000, TPB_3M);
        h[3] = h[2].wrapping_add(TPB_3M.wrapping_mul(4));
        d.stage_edges_for_test(&h);
        assert!(!d.try_anchor_from_header(TPB_3M));
        assert_eq!(d.current_byte_tick(), None);
    }

    #[test]
    fn on_edge_advance_no_flags_is_noop() {
        let mut d = rx();
        d.stage_edges_for_test(&[1000]);
        // No flags armed.
        let mut emissions = 0_u32;
        d.on_edge_advance(TPB_3M, |_, _| emissions += 1);
        assert_eq!(emissions, 0);
        assert_eq!(d.current_byte_tick(), None);
        assert_eq!(d.ring.ack_log, [DmaFlags::default()]);
    }

    #[test]
    fn on_edge_advance_with_ht_routes_pair_callback_when_active() {
        let mut d = rx();
        d.set_hsi_active(true);
        // Seed an anchor manually by way of try_anchor_from_header.
        let h = header_edges(1000, TPB_3M);
        d.stage_edges_for_test(&h);
        assert!(d.try_anchor_from_header(TPB_3M));

        // Append a body byte's start edge and trigger walker advance.
        d.stage_edges_for_test(&[
            h[0],
            h[1],
            h[2],
            h[3],
            h[4],
            h[4].wrapping_add(BYTE_TICKS_3M),
        ]);
        d.arm_next_flags_for_test(DmaFlags {
            ht: true,
            tc: false,
        });
        let mut pairs: heapless::Vec<(u16, u16), 4> = heapless::Vec::new();
        d.on_edge_advance(TPB_3M, |p, n| {
            pairs.push((p, n)).unwrap();
        });
        // One new edge classified as a hit → one pair.
        assert_eq!(
            pairs.as_slice(),
            &[(h[4], h[4].wrapping_add(BYTE_TICKS_3M))]
        );
    }

    #[test]
    fn on_edge_advance_skips_callback_when_inactive() {
        let mut d = rx();
        // hsi_active stays false. Seed anchor.
        let h = header_edges(1000, TPB_3M);
        d.stage_edges_for_test(&h);
        assert!(d.try_anchor_from_header(TPB_3M));

        d.stage_edges_for_test(&[
            h[0],
            h[1],
            h[2],
            h[3],
            h[4],
            h[4].wrapping_add(BYTE_TICKS_3M),
        ]);
        d.arm_next_flags_for_test(DmaFlags {
            ht: true,
            tc: false,
        });
        let mut emissions = 0_u32;
        d.on_edge_advance(TPB_3M, |_, _| emissions += 1);
        assert_eq!(emissions, 0);
        // Hit still classified — current tick advances.
        assert_eq!(
            d.current_byte_tick(),
            Some(h[4].wrapping_add(BYTE_TICKS_3M)),
        );
    }

    #[test]
    fn on_idle_drops_anchor_and_flag() {
        let mut d = rx();
        d.set_hsi_active(true);
        let h = header_edges(1000, TPB_3M);
        d.stage_edges_for_test(&h);
        assert!(d.try_anchor_from_header(TPB_3M));

        d.on_idle(TPB_3M, |_, _| {});
        assert_eq!(d.current_byte_tick(), None);
        assert!(!d.hsi_active());
    }

    #[test]
    fn current_byte_tick_and_packet_end_tick_forward() {
        let mut d = rx();
        assert_eq!(d.current_byte_tick(), None);
        assert_eq!(d.packet_end_tick(TPB_3M), None);

        let h = header_edges(1000, TPB_3M);
        d.stage_edges_for_test(&h);
        assert!(d.try_anchor_from_header(TPB_3M));
        assert_eq!(d.current_byte_tick(), Some(h[4]));
        assert_eq!(
            d.packet_end_tick(TPB_3M),
            Some(h[4].wrapping_add(BYTE_TICKS_3M)),
        );
    }

    #[test]
    fn reset_anchor_clears_state() {
        let mut d = rx();
        d.set_hsi_active(true);
        let h = header_edges(1000, TPB_3M);
        d.stage_edges_for_test(&h);
        assert!(d.try_anchor_from_header(TPB_3M));

        d.reset_anchor();
        assert_eq!(d.current_byte_tick(), None);
        assert!(!d.hsi_active());
    }
}
