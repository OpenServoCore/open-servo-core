//! Hardware-timed DXL receive path. Owns the window classifier that turns
//! TIM2_CH4 edge timestamps into per-byte timestamps (BT); consumers read
//! `byte_ts(idx)` for fire/snoop/drift decisions in lieu of IDLE backdates.
//!
//! Driver is concrete (no generics) — chip is fixed and the algorithm is
//! pure compute. The DMA-fed edge ring lives in `dxl::statics`; this driver
//! borrows it through `on_edge_advance` rather than owning a static cell.

mod classifier;

use classifier::Classifier;

pub struct DxlRx {
    classifier: Classifier,
}

impl DxlRx {
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self {
            classifier: Classifier::new(),
        }
    }

    /// Walks newly-captured edges through the window classifier. Called
    /// from the DMA1_CH7 HT/TC ISR.
    #[allow(dead_code)]
    pub fn on_edge_advance(&mut self, edges: &[u16], edges_head: u16, ticks_per_bit: u16) {
        self.classifier
            .on_edge_advance(edges, edges_head, ticks_per_bit);
    }

    /// Drains residual edges (HT/TC didn't fire for the tail of a short
    /// packet), then invalidates the anchor so the next packet's first
    /// edge re-seeds. Called from the USART1 IDLE handler.
    #[allow(dead_code)]
    pub fn on_idle(&mut self, edges: &[u16], edges_head: u16, ticks_per_bit: u16) {
        self.classifier
            .on_edge_advance(edges, edges_head, ticks_per_bit);
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
