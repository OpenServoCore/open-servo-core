//! Hardware-timed DXL receive path. Owns the edge-timestamp ring (DMA1_CH7
//! destination) and the window classifier that turns the captured falling
//! edges into per-byte timestamps (BT). Consumers read `byte_ts(idx)` for
//! fire / snoop / drift decisions in lieu of IDLE backdates.
//!
//! The driver depends on a [`DmaRing`] adapter for HT/TC flag drain and
//! NDTR readback; the production adapter binds to DMA1_CH7. Tests swap in
//! [`crate::adapters::mocks::FakeDmaRing`] and stage flags + remaining
//! directly.

mod classifier;

use core::cell::SyncUnsafeCell;

use crate::adapters;
use crate::drivers::traits::DmaRing;
use classifier::Classifier;

/// Edge-timestamp ring depth (doc §8.4 default). Power-of-two for cheap
/// mask indexing; HT fires at the halfway point so the classifier never
/// has to chase a write head that's lapped its read cursor under sustained
/// 3M traffic.
pub const EDGE_BUF_LEN: usize = 128;

const _: () = assert!(EDGE_BUF_LEN.is_power_of_two() && EDGE_BUF_LEN <= u16::MAX as usize + 1);

pub struct DxlRx<R: DmaRing = adapters::dma::Ch7> {
    classifier: Classifier,
    /// DMA1_CH7's destination buffer. `SyncUnsafeCell` because the DMA
    /// engine writes it concurrently with the classifier's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware, so plain `&` references inside
    /// classifier walks are sound.
    edges: SyncUnsafeCell<[u16; EDGE_BUF_LEN]>,
    ring: R,
}

impl<R: DmaRing> DxlRx<R> {
    pub const fn new(ring: R) -> Self {
        Self {
            classifier: Classifier::new(),
            edges: SyncUnsafeCell::new([0; EDGE_BUF_LEN]),
            ring,
        }
    }

    /// Stable peripheral-memory address for the DMA destination. Bringup
    /// hands this to `dma::configure(CH7, ...)`; the driver instance lives
    /// in the registry's `SyncUnsafeCell<Option<DxlRx>>` so the address is
    /// fixed for the lifetime of the program once `install` returns.
    pub fn edges_addr(&self) -> u32 {
        self.edges.get() as u32
    }

    /// DMA1_CH7 HT/TC handler. Drains flags through the adapter, computes
    /// the write head from NDTR, walks newly-captured edges through the
    /// classifier. No-op if neither flag is set (defends against spurious
    /// vector entry).
    pub fn on_dma_event(&mut self, ticks_per_bit: u16) {
        let flags = self.ring.read_and_ack();
        if !flags.ht && !flags.tc {
            return;
        }
        let head = (EDGE_BUF_LEN as u16).wrapping_sub(self.ring.remaining());
        // SAFETY: the edges buffer is mutated only by DMA1_CH7 (hardware
        // writer) and read here from a PFIC-HIGH ISR; no other code path
        // takes a `&mut` into it.
        let edges = unsafe { &*self.edges.get() };
        self.classifier
            .on_edge_advance(edges, head, ticks_per_bit);
    }

    /// USART1 IDLE backstop. Walks any tail edges the HT/TC ISR hasn't
    /// drained yet (short packets that don't fill a half-ring never trip
    /// HT), then invalidates the anchor so the next packet's first edge
    /// re-seeds.
    pub fn on_idle(&mut self, ticks_per_bit: u16) {
        let head = (EDGE_BUF_LEN as u16).wrapping_sub(self.ring.remaining());
        // SAFETY: see `on_dma_event`.
        let edges = unsafe { &*self.edges.get() };
        self.classifier
            .on_edge_advance(edges, head, ticks_per_bit);
        self.classifier.reset_anchor();
    }

    #[allow(dead_code)]
    pub fn byte_ts_at(&self, seq: u16) -> Option<u16> {
        self.classifier.byte_ts_at(seq)
    }

    #[allow(dead_code)]
    pub fn byte_ts_head(&self) -> u16 {
        self.classifier.byte_ts_head()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mocks::FakeDmaRing;
    use crate::drivers::traits::DmaFlags;

    // 3 Mbaud at HCLK 48 MHz → ticks_per_bit = 16.
    const TPB_3M: u16 = 16;
    const BYTE_TICKS_3M: u16 = 160;

    fn rx() -> DxlRx<FakeDmaRing> {
        DxlRx::new(FakeDmaRing::default())
    }

    /// Stage `edges[..n]` into the driver's buffer as if DMA had written
    /// them, and set `remaining` so `head == n`.
    fn stage_edges(d: &mut DxlRx<FakeDmaRing>, vals: &[u16]) {
        // SAFETY: test-only access to the SyncUnsafeCell; no DMA in tests.
        let buf = unsafe { &mut *d.edges.get() };
        for (i, &v) in vals.iter().enumerate() {
            buf[i] = v;
        }
        d.ring.remaining = (EDGE_BUF_LEN - vals.len()) as u16;
    }

    #[test]
    fn on_dma_event_no_flags_is_noop() {
        let mut d = rx();
        stage_edges(&mut d, &[1000]);
        d.on_dma_event(TPB_3M);
        assert_eq!(d.byte_ts_head(), 0);
        assert_eq!(d.ring.ack_log, [DmaFlags::default()]);
    }

    #[test]
    fn on_dma_event_with_ht_drives_classifier() {
        let mut d = rx();
        stage_edges(&mut d, &[1000, 1000 + BYTE_TICKS_3M]);
        d.ring.next_flags = DmaFlags { ht: true, tc: false };
        d.on_dma_event(TPB_3M);
        assert_eq!(d.byte_ts_head(), 2);
        assert_eq!(d.byte_ts_at(0), Some(1000));
        assert_eq!(d.byte_ts_at(1), Some(1000 + BYTE_TICKS_3M));
    }

    #[test]
    fn on_dma_event_with_tc_drives_classifier() {
        let mut d = rx();
        stage_edges(&mut d, &[2000]);
        d.ring.next_flags = DmaFlags { ht: false, tc: true };
        d.on_dma_event(TPB_3M);
        assert_eq!(d.byte_ts_head(), 1);
    }

    #[test]
    fn on_idle_resets_anchor() {
        let mut d = rx();
        // First burst: seed an anchor.
        stage_edges(&mut d, &[5000]);
        d.ring.next_flags = DmaFlags { ht: true, tc: false };
        d.on_dma_event(TPB_3M);
        assert_eq!(d.byte_ts_head(), 1);

        // IDLE drains nothing new (same head), but invalidates the anchor.
        d.on_idle(TPB_3M);

        // After reset, the next edge re-seeds rather than gap-classifying.
        // Stage a single edge far from the prior anchor — would be a GAP
        // if anchor were still 5000.
        stage_edges(&mut d, &[5000, 59_000]);
        d.ring.next_flags = DmaFlags { ht: true, tc: false };
        d.on_dma_event(TPB_3M);
        assert_eq!(d.byte_ts_head(), 2);
        assert_eq!(d.byte_ts_at(1), Some(59_000));
    }
}
