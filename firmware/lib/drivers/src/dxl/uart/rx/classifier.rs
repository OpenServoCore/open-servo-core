//! Window classifier — turns falling-edge timestamps (TIM2_CH4 IC) into
//! byte-start timestamps. One BT entry per UART byte; intra-byte edges drop
//! out. Algorithm proven in the v006 bringup spike across 11 patterns at
//! 3 Mbaud (`docs/dxl-hw-timed-transport.md` §10).
//!
//! ## Window math
//! `ticks_since_anchor = t - anchor` (u16 wrap) classifies each new edge
//! against the current anchor:
//! - `ticks_since_anchor < 9·bit_time`             → SKIP (intra-byte edge)
//! - `ticks_since_anchor ∈ [9·bit_time, 11·bit_time]` → HIT, re-anchor, push BT
//! - `ticks_since_anchor > 11·bit_time`            → GAP, re-anchor, push BT
//!
//! No anchor yet → seed: push first edge as BT[0] and adopt as anchor.
//!
//! ## Why ±10% width
//! HSI is rated ±1 %, MCU temp drift another ±1 %, host clock another ±0.5 %.
//! ±10 % gives ~6× margin over expected worst-case drift while staying tight
//! enough that a missed byte boundary (delta ≈ 20·bit_time) lands clean in
//! the GAP bin instead of being mis-classified as HIT.

use crate::util::DmaBuffer;

const WINDOW_LO_MUL: u16 = 9;
const WINDOW_HI_MUL: u16 = 11;

pub struct Classifier<const BT_BUF_LEN: usize> {
    /// Last edge timestamp that anchored a byte start, if any. New edges
    /// are classified by `t - anchor` (u16 wrap). `None` after a fresh
    /// `Classifier::new` or `reset_anchor` — the next edge re-seeds.
    anchor: Option<u16>,
    /// Consumer cursor into the caller's `edges` slice — masked by
    /// `edges.len() - 1` on entry. Persisted between `on_edge_advance`
    /// calls so the DMA HT/TC interleave doesn't re-walk already-classified
    /// entries.
    edges_read_idx: u16,

    byte_ts_buf: [u16; BT_BUF_LEN],
    /// Sequence number of the next BT slot to write. Monotonically
    /// increasing across u16; consumers read by sequence number and
    /// [`Classifier::byte_ts_at`] masks down to a ring slot.
    /// (Doc §10.1 names this the `bt_frontier`.)
    byte_ts_head: u16,

    seeds: u16,
    hits: u16,
    skips: u16,
    gaps: u16,
}

impl<const BT_BUF_LEN: usize> Classifier<BT_BUF_LEN> {
    /// Compile-time guard. `byte_ts_at`'s ring-window math assumes
    /// power-of-two so `% BT_BUF_LEN` collapses to AND; bound at u16 so
    /// the slot index fits the timer's native CNT width.
    const _CHECK_BT_BUF_LEN: () = assert!(
        BT_BUF_LEN.is_power_of_two() && BT_BUF_LEN > 0 && BT_BUF_LEN <= u16::MAX as usize + 1,
        "BT_BUF_LEN must be a power of two in (0, 1<<16]",
    );

    #[allow(dead_code)]
    pub const fn new() -> Self {
        // Force the const assertion to fire at instantiation.
        let _: () = Self::_CHECK_BT_BUF_LEN;
        Self {
            anchor: None,
            edges_read_idx: 0,
            byte_ts_buf: [0; BT_BUF_LEN],
            byte_ts_head: 0,
            seeds: 0,
            hits: 0,
            skips: 0,
            gaps: 0,
        }
    }

    /// Process the edge ring from the last-walked index up to `edges_head`
    /// (a ring position in `[0, EDGE_BUF_LEN)`), pushing BT entries for
    /// SEED/HIT/GAP edges and dropping SKIPs. Advances the read cursor to
    /// `edges_head` on exit.
    ///
    /// `ticks_per_bit` is the spec UART bit time in TIM2 ticks; callers
    /// read it from [`crate::dxl::uart::clock::Clock`] so a baud change
    /// widens/narrows the window in lockstep.
    pub fn on_edge_advance<const EDGE_BUF_LEN: usize>(
        &mut self,
        edges: &DmaBuffer<u16, EDGE_BUF_LEN>,
        edges_head: u16,
        ticks_per_bit: u16,
    ) {
        let win_lo_ticks = ticks_per_bit.wrapping_mul(WINDOW_LO_MUL);
        let win_hi_ticks = ticks_per_bit.wrapping_mul(WINDOW_HI_MUL);
        let mut idx = self.edges_read_idx;
        if idx == edges_head {
            return;
        }

        let mut anchor = self.anchor;
        let mut byte_ts_head = self.byte_ts_head;
        let mut seeds = self.seeds;
        let mut hits = self.hits;
        let mut skips = self.skips;
        let mut gaps = self.gaps;
        let byte_ts_buf = &mut self.byte_ts_buf;

        while idx != edges_head {
            let t = *edges.at(idx);
            match anchor {
                None => {
                    anchor = Some(t);
                    byte_ts_buf[(byte_ts_head as usize) % BT_BUF_LEN] = t;
                    byte_ts_head = byte_ts_head.wrapping_add(1);
                    seeds = seeds.wrapping_add(1);
                }
                Some(a) => {
                    let ticks_since_anchor = t.wrapping_sub(a);
                    if ticks_since_anchor < win_lo_ticks {
                        skips = skips.wrapping_add(1);
                    } else if ticks_since_anchor <= win_hi_ticks {
                        anchor = Some(t);
                        byte_ts_buf[(byte_ts_head as usize) % BT_BUF_LEN] = t;
                        byte_ts_head = byte_ts_head.wrapping_add(1);
                        hits = hits.wrapping_add(1);
                    } else {
                        anchor = Some(t);
                        byte_ts_buf[(byte_ts_head as usize) % BT_BUF_LEN] = t;
                        byte_ts_head = byte_ts_head.wrapping_add(1);
                        gaps = gaps.wrapping_add(1);
                    }
                }
            }
            idx = idx.wrapping_add(1) % EDGE_BUF_LEN as u16;
        }

        self.anchor = anchor;
        self.byte_ts_head = byte_ts_head;
        self.seeds = seeds;
        self.hits = hits;
        self.skips = skips;
        self.gaps = gaps;
        self.edges_read_idx = edges_head;
    }

    /// Drops the current anchor so the next `on_edge_advance` re-seeds
    /// from scratch. Called from the USART1 IDLE backstop — a packet
    /// boundary always invalidates whatever mid-burst anchor the
    /// classifier carried.
    #[allow(dead_code)]
    pub fn reset_anchor(&mut self) {
        self.anchor = None;
    }

    /// Looks up a byte's start timestamp by sequence number. `None` if
    /// `seq` is past the head or has already wrapped out of the BT ring
    /// (i.e. the BT entry has been overwritten).
    #[allow(dead_code)]
    pub fn byte_ts_at(&self, seq: u16) -> Option<u16> {
        let ahead_of_seq = self.byte_ts_head.wrapping_sub(seq);
        if ahead_of_seq == 0 || ahead_of_seq > BT_BUF_LEN as u16 {
            None
        } else {
            Some(self.byte_ts_buf[(seq as usize) % BT_BUF_LEN])
        }
    }

    /// Sequence number of the next BT slot to write — i.e. one past the
    /// last published BT entry. (Doc §10.1: `bt_frontier`.)
    #[allow(dead_code)]
    pub fn byte_ts_head(&self) -> u16 {
        self.byte_ts_head
    }
}

#[cfg(test)]
mod tests {
    use super::Classifier;
    use crate::util::DmaBuffer;

    /// Power-of-two BT ring depth for these tests — matches the V006
    /// production sizing per doc §8.3 (BT must equal RX_BUF_LEN = 64).
    const BT_BUF_LEN: usize = 64;
    // 3 Mbaud at HCLK 48 MHz → BRR = 16 → window = [144, 176] ticks.
    const TPB_3M: u16 = 16;
    // 1 Mbaud at HCLK 48 MHz → BRR = 48 → window = [432, 528] ticks.
    const TPB_1M: u16 = 48;
    /// One byte-time at 3 Mbaud.
    const BYTE_TICKS_3M: u16 = 160;

    fn make() -> Classifier<BT_BUF_LEN> {
        Classifier::new()
    }

    /// Stage edge timestamps into a fresh 8-slot DmaBuffer.
    fn edges8(vals: &[u16]) -> DmaBuffer<u16, 8> {
        let mut b = DmaBuffer::new(0);
        b.stage(0, vals);
        b
    }

    #[test]
    fn seeds_on_first_entry() {
        let mut c = make();
        let edges = edges8(&[1000]);
        c.on_edge_advance(&edges, 1, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 0);
        assert_eq!(c.skips, 0);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head(), 1);
        assert_eq!(c.anchor, Some(1000));
        assert_eq!(c.byte_ts_at(0), Some(1000));
    }

    #[test]
    fn skips_intra_byte_edges() {
        let mut c = make();
        // Start at 1000, then four intra-byte edges spaced 32 ticks
        // (0xAA at 3M has falling edges every 2 bit-times = 32 ticks).
        let edges = edges8(&[1000, 1032, 1064, 1096, 1128]);
        c.on_edge_advance(&edges, 5, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 0);
        assert_eq!(c.skips, 4);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head(), 1);
    }

    #[test]
    fn hits_at_byte_boundary() {
        let mut c = make();
        let edges = edges8(&[1000, 1000 + BYTE_TICKS_3M]);
        c.on_edge_advance(&edges, 2, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 1);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head(), 2);
        assert_eq!(c.byte_ts_at(1), Some(1000 + BYTE_TICKS_3M));
    }

    #[test]
    fn re_anchors_on_gap() {
        let mut c = make();
        // ticks_since_anchor = 200 > 11·16 = 176 → GAP.
        let edges = edges8(&[1000, 1200]);
        c.on_edge_advance(&edges, 2, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.gaps, 1);
        assert_eq!(c.anchor, Some(1200));
        assert_eq!(c.byte_ts_at(1), Some(1200));
    }

    #[test]
    fn reset_anchor_invalidates() {
        let mut c = make();
        let edges = edges8(&[1000]);
        c.on_edge_advance(&edges, 1, TPB_3M);
        assert!(c.anchor.is_some());
        c.reset_anchor();
        assert!(c.anchor.is_none());
        let edges2 = edges8(&[1000, 9000]);
        c.on_edge_advance(&edges2, 2, TPB_3M);
        // Second entry re-seeds (no anchor to compare against), not a GAP.
        assert_eq!(c.seeds, 2);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.anchor, Some(9000));
    }

    #[test]
    fn respects_ticks_per_bit_arg() {
        // ticks_since_anchor = 160: HIT at 3M (window 144..176),
        // SKIP at 1M (window 432..528).
        let edges = edges8(&[1000, 1160]);

        let mut c_3m = make();
        c_3m.on_edge_advance(&edges, 2, TPB_3M);
        assert_eq!(c_3m.hits, 1);
        assert_eq!(c_3m.skips, 0);

        let mut c_1m = make();
        c_1m.on_edge_advance(&edges, 2, TPB_1M);
        assert_eq!(c_1m.hits, 0);
        assert_eq!(c_1m.skips, 1);
    }

    #[test]
    fn multi_byte_pattern_0xaa_x16() {
        // 0xAA on the wire has 4 falling edges per byte at offsets
        // {0, 32, 64, 96} from the byte start; bytes are 160 ticks apart.
        // 16 bytes → 64 edges. Expected: SEED=1, HIT=15, SKIP=48.
        const N_BYTES: usize = 16;
        const EDGES_PER_BYTE: usize = 4;
        const TOTAL: usize = N_BYTES * EDGES_PER_BYTE;
        let mut edges: DmaBuffer<u16, 128> = DmaBuffer::new(0);
        let intra = [0_u16, 32, 64, 96];
        let mut staged = [0_u16; TOTAL];
        for byte_i in 0..N_BYTES {
            let t0 = 1000_u16.wrapping_add((byte_i as u16).wrapping_mul(BYTE_TICKS_3M));
            for j in 0..EDGES_PER_BYTE {
                staged[byte_i * EDGES_PER_BYTE + j] = t0.wrapping_add(intra[j]);
            }
        }
        edges.stage(0, &staged);

        let mut c = make();
        c.on_edge_advance(&edges, TOTAL as u16, TPB_3M);

        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 15);
        assert_eq!(c.skips, 48);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head(), 16);

        // BT entries land at byte starts; spacing == one byte-time.
        let bt0 = c.byte_ts_at(0).unwrap();
        let bt1 = c.byte_ts_at(1).unwrap();
        assert_eq!(bt1.wrapping_sub(bt0), BYTE_TICKS_3M);
    }

    #[test]
    fn byte_ts_at_returns_none_past_head() {
        let mut c = make();
        let edges = edges8(&[1000]);
        c.on_edge_advance(&edges, 1, TPB_3M);
        assert_eq!(c.byte_ts_at(0), Some(1000));
        assert_eq!(c.byte_ts_at(1), None);
        assert_eq!(c.byte_ts_at(2), None);
    }

    #[test]
    fn byte_ts_at_returns_none_past_ring_window() {
        let mut c = make();
        c.byte_ts_head = (BT_BUF_LEN as u16).wrapping_add(5);
        // Entries [5, BUF_LEN+5) are in-window; [0, 5) have been overwritten.
        assert_eq!(c.byte_ts_at(0), None);
        assert_eq!(c.byte_ts_at(4), None);
        assert!(c.byte_ts_at(5).is_some());
    }
}
