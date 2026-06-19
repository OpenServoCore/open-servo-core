//! Hardware-timed DXL receive path. Owns the edge-timestamp ring
//! (DMA1_CH7 destination) and the header-anchored classifier that turns
//! captured falling edges into per-byte start ticks. Consumers query
//! `current_byte_tick()` / `packet_end_tick()` at parser-event time in
//! lieu of IDLE backdates, and route drift pairs through the `on_pair`
//! callback to the HSI clock service.
//!
//! The driver depends on a [`EdgeDma`] adapter for HT/TC flag drain and
//! NDTR readback; the production adapter binds to DMA1_CH7. Tests swap in
//! [`crate::mocks::MockEdgeDma`] and stage flags + remaining directly.
//!
//! The ET ring depth is const-generic so a chip/board can pick its own
//! memory budget without touching driver code. The chip-facing knob is
//! [`sync_lookback_edges`] (the worst-case anchor back-search depth);
//! [`edge_buf_len`] / [`rx_buf_len`] derive the matching ring sizes.

mod classifier;

pub use classifier::{FallbackSrc, edge_buf_len, rx_buf_len, sync_lookback_edges};

use core::cell::SyncUnsafeCell;

use crate::traits::dxl::EdgeDma;
use crate::util::HwRing;
use classifier::Classifier;

pub struct Rx<R: EdgeDma, const EDGE_BUF_LEN: usize> {
    classifier: Classifier,
    /// DMA1_CH7's destination buffer. `SyncUnsafeCell` because the DMA
    /// engine writes it concurrently with the classifier's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware, so plain `&` references inside
    /// classifier walks are sound. [`HwRing`] enforces pow-2 sizing at
    /// construction so the chip-side NDTR head math (`EDGE_BUF_LEN -
    /// NDTR`) maps cleanly to a ring position.
    edges: SyncUnsafeCell<HwRing<u16, EDGE_BUF_LEN>>,
    ring: R,
    /// Most recent ISR's wake capture — WireClock u32 tick at ISR entry
    /// plus the source flavor. Set on every [`Self::on_edge_advance`] /
    /// [`Self::on_idle`] entry; read by the composite Crc handler when
    /// [`Self::packet_end_tick`] returns `None` to pick a fallback
    /// formula via [`Self::packet_end_tick_fallback`]. Default value
    /// `(0, FallbackSrc::Dma)` is unreachable in production — Crc
    /// follows bytes which follow an ISR — but keeps the field non-
    /// Optional so the hot path doesn't carry an `unwrap`.
    last_isr: (u32, FallbackSrc),
}

impl<R: EdgeDma, const EDGE_BUF_LEN: usize> Rx<R, EDGE_BUF_LEN> {
    pub const fn new(ring: R) -> Self {
        Self {
            classifier: Classifier::new(),
            edges: SyncUnsafeCell::new(HwRing::new(0)),
            ring,
            last_isr: (0, FallbackSrc::Dma),
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

    /// Called when new RX falling-edge timestamps may be available. Drains
    /// HT/TC flags through the adapter, publishes the write head from
    /// NDTR, walks newly-captured edges through the classifier. Drift
    /// pairs route through `on_pair` only while the classifier's
    /// `hsi_active` flag is set. No-op if neither flag is set (defends
    /// against spurious vector entry). `now` is the ISR-entry WireClock
    /// u32 reading, stashed for the no-anchor fallback path at the
    /// parser's Crc event — see [`Self::packet_end_tick_fallback`].
    pub fn on_edge_advance<F: FnMut(u16, u16)>(
        &mut self,
        now: u32,
        ticks_per_bit: u16,
        on_pair: F,
    ) {
        self.last_isr = (now, FallbackSrc::Dma);
        let flags = self.ring.read_and_ack();
        crate::log::trace!("rx: on_edge_advance ht={} tc={}", flags.ht, flags.tc);
        if !flags.ht && !flags.tc {
            return;
        }
        let remaining = self.ring.remaining();
        crate::log::trace!("rx: on_edge_advance publish remaining={}", remaining);
        // SAFETY: the edges buffer is mutated only by DMA1_CH7 (hardware
        // writer) and read here from a PFIC-HIGH ISR; no other code path
        // takes a `&mut` into it.
        let edges = unsafe { &mut *self.edges.get() };
        edges.on_publish(remaining);
        self.classifier
            .on_edge_advance(edges, ticks_per_bit, on_pair);
    }

    /// USART1 IDLE backstop. Drains any tail edges the HT/TC ISR hasn't
    /// drained yet (short packets that don't fill a half-ring never trip
    /// HT). Anchor + drift gating stay until the parser's Crc / Resync
    /// event ([`reset_anchor`](Self::reset_anchor)) — IDLE only signals
    /// "no more edges right now." `now` is the ISR-entry WireClock u32
    /// reading, stashed for the no-anchor fallback path at Crc — see
    /// [`Self::packet_end_tick_fallback`].
    pub fn on_idle<F: FnMut(u16, u16)>(&mut self, now: u32, ticks_per_bit: u16, on_pair: F) {
        self.last_isr = (now, FallbackSrc::Idle);
        let remaining = self.ring.remaining();
        crate::log::trace!("rx: on_idle publish remaining={}", remaining);
        // SAFETY: see `on_edge_advance`.
        let edges = unsafe { &mut *self.edges.get() };
        edges.on_publish(remaining);
        self.classifier.on_idle(edges, ticks_per_bit, on_pair);
    }

    /// Catchup-walk pending edges through the classifier without touching
    /// `last_isr` or HT/TC flags. The composite calls this at the parser's
    /// Crc event so `last_byte_start` reflects the packet's final byte
    /// before [`Self::packet_end_tick`] reads it: HT/TC and IDLE are the
    /// only walker drivers, and a packet bounded by a single such event
    /// fires Header → Crc within one parser poll, leaving the anchor
    /// placed by [`Self::try_anchor_from_header`] but never forward-walked
    /// across bytes past the signature. Idempotent — the reader cursor
    /// only advances over edges the walker hadn't yet processed.
    pub fn drain_walker<F: FnMut(u16, u16)>(&mut self, ticks_per_bit: u16, on_pair: F) {
        let remaining = self.ring.remaining();
        crate::log::trace!("rx: drain_walker publish remaining={}", remaining);
        // SAFETY: see `on_edge_advance`.
        let edges = unsafe { &mut *self.edges.get() };
        edges.on_publish(remaining);
        self.classifier
            .on_edge_advance(edges, ticks_per_bit, on_pair);
    }

    /// Most recent ISR wake capture — `(now, src)` recorded at the last
    /// [`Self::on_edge_advance`] / [`Self::on_idle`] entry. Composite Crc
    /// handler consumes this when the classifier was unanchored (the
    /// `packet_end_tick_fallback` path).
    pub fn last_isr_capture(&self) -> (u32, FallbackSrc) {
        self.last_isr
    }

    /// Back-search the ET ring for the DXL header's 5-edge signature.
    /// Composite calls at every parser Header event. On match the anchor
    /// is established and the walker's ET cursor advances past the
    /// header edges; on mismatch any stale anchor is cleared and the
    /// next packet's header re-attempts from scratch.
    pub fn try_anchor_from_header(&mut self, ticks_per_bit: u16) -> bool {
        // SAFETY: see `on_edge_advance`.
        let edges = unsafe { &mut *self.edges.get() };
        self.classifier.try_anchor_from_header(edges, ticks_per_bit)
    }

    /// Tick of the most-recently classified byte's start bit, lifted into
    /// the WireClock u32 domain via `now`.
    pub fn current_byte_tick(&self, now: u32) -> Option<u32> {
        self.classifier.current_byte_tick(now)
    }

    /// Wire-end tick of the most-recently classified byte, lifted into the
    /// WireClock u32 domain. Composite stamps `packet_end_tick` at the
    /// parser's CRC-good event.
    pub fn packet_end_tick(&self, ticks_per_bit: u16, now: u32) -> Option<u32> {
        self.classifier.packet_end_tick(ticks_per_bit, now)
    }

    /// Fallback packet-end estimate for the no-anchor case — composite
    /// calls when [`packet_end_tick`](Self::packet_end_tick) returns `None`
    /// at the Crc event. See [`classifier::Classifier::packet_end_tick_fallback`]
    /// for the per-source formulas.
    pub fn packet_end_tick_fallback(&self, src: FallbackSrc, now: u32, ticks_per_bit: u16) -> u32 {
        self.classifier
            .packet_end_tick_fallback(src, now, ticks_per_bit)
    }

    /// Toggle drift-sampling emission. Composite sets at Instruction-
    /// header events, clears at the matching CRC event.
    pub fn set_hsi_active(&mut self, on: bool) {
        self.classifier.set_hsi_active(on);
    }

    pub fn hsi_active(&self) -> bool {
        self.classifier.hsi_active()
    }

    /// Force-drop anchor + drift flag. Composite calls at the parser's
    /// Crc event (packet boundary) and Resync event (parser abort).
    pub fn reset_anchor(&mut self) {
        self.classifier.reset_anchor();
    }

    /// Force a known `last_byte_start` tick — composite-test scaffolding so
    /// `packet_end_tick` reads a deterministic value without staging real
    /// edges.
    #[cfg(test)]
    pub fn force_byte_tick_for_test(&mut self, tick: u16) {
        self.classifier.force_byte_tick_for_test(tick);
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
