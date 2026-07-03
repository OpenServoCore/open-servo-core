//! Hardware-timed DXL receive path. Owns the edge-timestamp ring
//! (DMA1_CH7 destination) and the [`AnchorCache`] the stateless edge
//! walker ([`super::edge_parser`]) threads to turn captured falling edges
//! into the tail-anchor tick and the retroactive integrator walk.
//!
//! The EdgeCapture wrapper exposes four operations the codec drives:
//!
//! - [`EdgeCapture::on_idle`] and [`EdgeCapture::on_byte_batch_wake`] — ISR
//!   entries that stash the wake tick for the Crc-time fallback path;
//!   `on_idle` also publishes the ET producer head from NDTR.
//! - [`EdgeCapture::anchor_at_tail`] — called at the parser's `Event::Crc`;
//!   back-searches the ET ring for the tail-byte signature at
//!   `[d_min ..= d_min + PADDING_EDGES_AHEAD]` back from head, where the
//!   codec-supplied `d_min` locates CRC's tail edge in ET. Sets the tail
//!   anchor.
//! - [`EdgeCapture::walk_pairs_back`] — called AFTER the wire-schedule call
//!   at the parser's `Event::Crc`. Retroactively feeds the drift integrator
//!   so the walk cost lives off the deadline path.
//!
//! Consumers read `packet_end_tick()` at parser Crc time.
//!
//! The driver depends on a [`EdgeDma`] adapter for HT/TC flag drain and
//! NDTR readback; the production adapter binds to DMA1_CH7. Tests swap in
//! [`crate::mocks::MockEdgeDma`] and stage flags + remaining directly.
//!
//! The ET ring depth is const-generic so a chip/board can pick its own
//! memory budget without touching driver code. The chip-facing knob is
//! the byte-ring size; [`edge_buf_len`] derives the matching ET depth
//! from it.
//!
//! [`edge_buf_len`]: super::edge_parser::edge_buf_len

use core::cell::SyncUnsafeCell;

use super::anchor::AnchorCache;
use super::edge_parser;
use crate::dxl::uart::poll_src::PollSrc;
use crate::ring::HwRing;
use crate::traits::dxl::EdgeDma;

pub struct EdgeCapture<R: EdgeDma, const EDGE_BUF_LEN: usize> {
    /// Tail-anchor state the edge walker ([`super::edge_parser`]) threads.
    anchor: AnchorCache,
    /// DMA1_CH7's destination buffer. `SyncUnsafeCell` because the DMA
    /// engine writes it concurrently with the edge walker's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware, so plain `&` references inside
    /// walker passes are sound. [`HwRing`] enforces pow-2 sizing at
    /// construction so the chip-side NDTR head math (`EDGE_BUF_LEN -
    /// NDTR`) maps cleanly to a ring position.
    edges: SyncUnsafeCell<HwRing<u16, EDGE_BUF_LEN>>,
    ring: R,
    /// Most recent ISR's wake capture — WireClock u32 tick at ISR entry
    /// plus the source flavor. Set on every [`Self::on_idle`] /
    /// [`Self::on_byte_batch_wake`] entry; read by the composite Crc
    /// handler when [`Self::packet_end_tick`] returns `None` to pick a
    /// fallback formula via [`Self::packet_end_tick_fallback`]. Default
    /// value `(0, PollSrc::ByteBatch)` is unreachable in production —
    /// Crc follows bytes which follow an ISR — but keeps the field non-
    /// Optional so the hot path doesn't carry an `unwrap`.
    last_isr: (u32, PollSrc),
}

impl<R: EdgeDma, const EDGE_BUF_LEN: usize> EdgeCapture<R, EDGE_BUF_LEN> {
    pub const fn new(ring: R) -> Self {
        Self {
            anchor: AnchorCache::new(),
            edges: SyncUnsafeCell::new(HwRing::new(0)),
            ring,
            last_isr: (0, PollSrc::ByteBatch),
        }
    }

    /// Publish the producer head from `ring.remaining()` (NDTR readback)
    /// so subsequent back-searches see newly-captured edges. Pure ring
    /// bookkeeping — no walking, no anchor mutation. Called from ISR
    /// entries and from [`Self::anchor_at_tail`] to refresh before the
    /// edge walker reads.
    fn publish_edges(&mut self) {
        let remaining = self.ring.remaining();
        // SAFETY: the edges buffer is mutated only by DMA1_CH7 (hardware
        // writer) and read here from a PFIC-HIGH ISR; no other code path
        // takes a `&mut` into it.
        let edges = unsafe { &mut *self.edges.get() };
        edges.on_publish(remaining);
    }
}

// -- events -------------------------------------------------------------

impl<R: EdgeDma, const EDGE_BUF_LEN: usize> EdgeCapture<R, EDGE_BUF_LEN> {
    /// USART1 IDLE ISR entry. Publishes the producer head (small packets
    /// that don't fill a half-ring never trip HT). Stashes `(now, LineIdle)`
    /// for the Crc-time fallback path.
    pub fn on_idle(&mut self, now: u32) {
        self.last_isr = (now, PollSrc::LineIdle);
        crate::log::trace!("edge_capture: on_idle");
        self.publish_edges();
    }

    /// Stash `(now, ByteBatch)` on the DMA1_CH5 (byte-ring) HT/TC ISR entry.
    /// That vector drives the parser drain, so the Crc-time fallback path
    /// needs a fresh capture from here — not from DMA1_CH7 (edge-ring),
    /// which small packets bypass entirely.
    pub fn on_byte_batch_wake(&mut self, now: u32) {
        self.last_isr = (now, PollSrc::ByteBatch);
    }

    /// Refresh the tail-anchor cache's per-baud RX edge-stamp compensation.
    /// Forwarded from the composite's `on_baud_change` event — see
    /// [`AnchorCache::set_edge_comp`].
    pub fn on_baud_change(&mut self, rx_edge_comp_ticks: u16) {
        self.anchor.set_edge_comp(rx_edge_comp_ticks);
    }
}

// -- commands -----------------------------------------------------------

impl<R: EdgeDma, const EDGE_BUF_LEN: usize> EdgeCapture<R, EDGE_BUF_LEN> {
    /// Back-search the ET ring for the just-received tail bytes' wire-edge
    /// signature. Codec calls at every parser `Event::Crc` with the last
    /// few raw wire bytes the parser consumed and `d_min` (summed
    /// [`super::edge_parser::edges_in_byte`] of bytes drained-but-not-yet-parsed
    /// past CRC in the byte ring at Crc emit — the exact shift from ET
    /// head to CRC's tail edge). On match the tail anchor is set to the
    /// CRC byte's start tick — what [`Self::packet_end_tick`] reads to
    /// derive `packet_end_tick` in wire-edge time.
    pub fn anchor_at_tail(&mut self, ticks_per_bit: u16, tail_bytes: &[u8], d_min: u16) -> bool {
        self.publish_edges();
        // SAFETY: see `publish_edges`.
        let edges = unsafe { &mut *self.edges.get() };
        edge_parser::anchor_at_tail(edges, &mut self.anchor, ticks_per_bit, tail_bytes, d_min)
    }

    /// Retroactive integrator walk. Codec calls after the wire-schedule
    /// call at the parser's `Event::Crc` so the walk cost sits off the
    /// deadline path. See [`super::edge_parser::walk_pairs_back`].
    pub fn walk_pairs_back<const PAIRS_LEN: usize>(
        &self,
        n_pairs: u8,
        ticks_per_bit: u16,
        out_pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
    ) {
        // SAFETY: see `publish_edges`; read-only path.
        let edges = unsafe { &*self.edges.get() };
        edge_parser::walk_pairs_back(edges, &self.anchor, n_pairs, ticks_per_bit, out_pairs)
    }

    /// Force-drop the tail anchor. Composite calls at the parser's Crc
    /// event (packet boundary) and Resync event (parser abort).
    pub fn reset_anchor(&mut self) {
        self.anchor.reset();
    }
}

// -- accessors ------------------------------------------------------------

impl<R: EdgeDma, const EDGE_BUF_LEN: usize> EdgeCapture<R, EDGE_BUF_LEN> {
    /// Stable peripheral-memory address for the DMA destination. Bringup
    /// hands this to `dma::configure(CH7, ...)`; the driver instance lives
    /// in the registry's `SyncUnsafeCell<Option<EdgeCapture>>` so the address is
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

    /// Most recent ISR wake capture — `(now, src)` recorded at the last
    /// [`Self::on_idle`] / [`Self::on_byte_batch_wake`] entry. Composite Crc
    /// handler consumes this when the edge walker was unanchored (the
    /// `packet_end_tick_fallback` path).
    pub fn last_isr_capture(&self) -> (u32, PollSrc) {
        self.last_isr
    }

    /// Read the tail-anchor tick in wire-edge time. Composite reads at
    /// walk-time after the wire-schedule call completes.
    pub fn tail_anchor(&self) -> Option<u16> {
        self.anchor.tail_anchor()
    }

    /// Wire-end tick of the most-recently classified byte, lifted into the
    /// WireClock u32 domain. Composite stamps `packet_end_tick` at the
    /// parser's CRC-good event. `now` / `src` route through the drain-
    /// reference correction — see `edge_parser::drain_ref`.
    pub fn packet_end_tick(&self, ticks_per_bit: u16, now: u32, src: PollSrc) -> Option<u32> {
        edge_parser::packet_end_tick(&self.anchor, ticks_per_bit, now, src)
    }

    /// Fallback packet-end estimate for the no-anchor case — composite
    /// calls when [`packet_end_tick`](Self::packet_end_tick) returns `None`
    /// at the Crc event. See [`super::edge_parser::packet_end_tick_fallback`]
    /// for the per-source formulas.
    pub fn packet_end_tick_fallback(&self, src: PollSrc, now: u32, ticks_per_bit: u16) -> u32 {
        edge_parser::packet_end_tick_fallback(src, now, ticks_per_bit)
    }
}

#[cfg(test)]
impl<R: EdgeDma, const EDGE_BUF_LEN: usize> EdgeCapture<R, EDGE_BUF_LEN> {
    /// Force a known tail-anchor tick — composite-test scaffolding so
    /// `packet_end_tick` reads a deterministic value without staging real
    /// edges.
    pub fn force_byte_tick_for_test(&mut self, tick: u16) {
        self.anchor.tail_anchor = Some(tick);
    }

    /// Stage falling-edge stamps into the ET ring so [`Self::anchor_at_tail`]
    /// resolves to `tail_anchor = anchor_tick` for `tail_bytes`' signature.
    /// Composite-test scaffolding — the caller staggers
    /// `EdgeDma::remaining()` so the follow-up [`Self::publish_edges`]
    /// advances `write_seq` by the returned count. Assumes
    /// `rx_edge_comp_ticks = 0` on the cache (test setup).
    pub fn stage_tail_signature_for_test(
        &mut self,
        tail_bytes: &[u8],
        ticks_per_bit: u16,
        anchor_tick: u16,
    ) -> u16 {
        // SAFETY: test-only mut access; no DMA in tests.
        let edges = unsafe { &mut *self.edges.get() };
        edge_parser::stage_tail_signature_for_test(edges, tail_bytes, ticks_per_bit, anchor_tick)
    }
}
