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

use crate::util::{HwRing, Seq, SwRing};

const WINDOW_LO_MUL: u16 = 9;
const WINDOW_HI_MUL: u16 = 11;

pub struct Classifier<const BT_BUF_LEN: usize> {
    /// Last edge timestamp that anchored a byte start, if any. New edges
    /// are classified by `t - anchor` (u16 wrap). `None` after a fresh
    /// `Classifier::new` or `reset_anchor` — the next edge re-seeds.
    anchor: Option<u16>,
    /// Per-UART-byte start timestamps. Software-fed by this classifier;
    /// consumers (drift, fire) read via `byte_ts.at(seq)`. The seq space
    /// is hardware-coupled to the RX byte ring per doc §8.3.
    byte_ts: SwRing<u16, BT_BUF_LEN>,

    seeds: u16,
    hits: u16,
    skips: u16,
    gaps: u16,
}

impl<const BT_BUF_LEN: usize> Classifier<BT_BUF_LEN> {
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self {
            anchor: None,
            byte_ts: SwRing::new(0),
            seeds: 0,
            hits: 0,
            skips: 0,
            gaps: 0,
        }
    }

    /// Walk newly-published edges through the window classifier, pushing
    /// BT entries for SEED / HIT / GAP and dropping SKIPs. The edge
    /// ring's `write_seq` must be current — the chip-side caller publishes
    /// via [`HwRing::on_publish`] before invoking this.
    ///
    /// `ticks_per_bit` is the spec UART bit time in TIM2 ticks; callers
    /// read it from [`crate::dxl::uart::clock::Clock`] so a baud change
    /// widens / narrows the window in lockstep.
    pub fn on_edge_advance<const EDGE_BUF_LEN: usize>(
        &mut self,
        edges: &mut HwRing<u16, EDGE_BUF_LEN>,
        ticks_per_bit: u16,
    ) {
        let win_lo_ticks = ticks_per_bit.wrapping_mul(WINDOW_LO_MUL);
        let win_hi_ticks = ticks_per_bit.wrapping_mul(WINDOW_HI_MUL);

        let mut anchor = self.anchor;
        let mut seeds = self.seeds;
        let mut hits = self.hits;
        let mut skips = self.skips;
        let mut gaps = self.gaps;
        let byte_ts = &mut self.byte_ts;

        let mut reader = edges.reader();
        while let Some(&t) = reader.peek() {
            match anchor {
                None => {
                    anchor = Some(t);
                    byte_ts.writer().push(t);
                    seeds = seeds.wrapping_add(1);
                }
                Some(a) => {
                    let ticks_since_anchor = t.wrapping_sub(a);
                    if ticks_since_anchor < win_lo_ticks {
                        skips = skips.wrapping_add(1);
                    } else if ticks_since_anchor <= win_hi_ticks {
                        anchor = Some(t);
                        byte_ts.writer().push(t);
                        hits = hits.wrapping_add(1);
                    } else {
                        anchor = Some(t);
                        byte_ts.writer().push(t);
                        gaps = gaps.wrapping_add(1);
                    }
                }
            }
            reader.advance(1);
        }

        self.anchor = anchor;
        self.seeds = seeds;
        self.hits = hits;
        self.skips = skips;
        self.gaps = gaps;
    }

    /// Drops the current anchor so the next `on_edge_advance` re-seeds
    /// from scratch. Called from the USART1 IDLE backstop — a packet
    /// boundary always invalidates whatever mid-burst anchor the
    /// classifier carried.
    #[allow(dead_code)]
    pub fn reset_anchor(&mut self) {
        self.anchor = None;
    }

    /// Look up a byte's start timestamp by sequence number. `None` if
    /// `seq` is past the head or has already wrapped out of the BT ring.
    #[allow(dead_code)]
    pub fn byte_ts_at(&self, seq: Seq<u16, BT_BUF_LEN>) -> Option<u16> {
        self.byte_ts.at(seq).copied()
    }

    /// Sequence of the next BT slot to write — one past the last
    /// published entry (doc §10.1: `bt_frontier`).
    #[allow(dead_code)]
    pub fn byte_ts_head(&self) -> Seq<u16, BT_BUF_LEN> {
        self.byte_ts.write_seq()
    }
}

#[cfg(test)]
mod tests {
    use super::Classifier;
    use crate::util::{HwRing, Seq};

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

    /// Stage edge timestamps into a fresh 8-slot HwRing and publish the
    /// producer head so the classifier sees them.
    fn edges8(vals: &[u16]) -> HwRing<u16, 8> {
        let mut b: HwRing<u16, 8> = HwRing::new(0);
        b.stage(0, vals);
        // Publish with NDTR-style remaining: head at slot vals.len() ↔
        // remaining = LEN - vals.len().
        b.on_publish(HwRing::<u16, 8>::LEN - vals.len() as u16);
        b
    }

    fn bt_at<const N: usize>(c: &Classifier<N>, raw: u16) -> Option<u16> {
        c.byte_ts_at(Seq::from_raw(raw))
    }

    #[test]
    fn seeds_on_first_entry() {
        let mut c = make();
        let mut edges = edges8(&[1000]);
        c.on_edge_advance(&mut edges, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 0);
        assert_eq!(c.skips, 0);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head().raw(), 1);
        assert_eq!(c.anchor, Some(1000));
        assert_eq!(bt_at(&c, 0), Some(1000));
    }

    #[test]
    fn skips_intra_byte_edges() {
        let mut c = make();
        // Start at 1000, then four intra-byte edges spaced 32 ticks
        // (0xAA at 3M has falling edges every 2 bit-times = 32 ticks).
        let mut edges = edges8(&[1000, 1032, 1064, 1096, 1128]);
        c.on_edge_advance(&mut edges, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 0);
        assert_eq!(c.skips, 4);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head().raw(), 1);
    }

    #[test]
    fn hits_at_byte_boundary() {
        let mut c = make();
        let mut edges = edges8(&[1000, 1000 + BYTE_TICKS_3M]);
        c.on_edge_advance(&mut edges, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 1);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head().raw(), 2);
        assert_eq!(bt_at(&c, 1), Some(1000 + BYTE_TICKS_3M));
    }

    #[test]
    fn re_anchors_on_gap() {
        let mut c = make();
        // ticks_since_anchor = 200 > 11·16 = 176 → GAP.
        let mut edges = edges8(&[1000, 1200]);
        c.on_edge_advance(&mut edges, TPB_3M);
        assert_eq!(c.seeds, 1);
        assert_eq!(c.gaps, 1);
        assert_eq!(c.anchor, Some(1200));
        assert_eq!(bt_at(&c, 1), Some(1200));
    }

    #[test]
    fn reset_anchor_invalidates() {
        let mut c = make();
        let mut edges: HwRing<u16, 8> = HwRing::new(0);
        edges.stage(0, &[1000]);
        edges.on_publish(HwRing::<u16, 8>::LEN - 1); // head at slot 1
        c.on_edge_advance(&mut edges, TPB_3M);
        assert!(c.anchor.is_some());

        c.reset_anchor();
        assert!(c.anchor.is_none());

        // Append the next edge and re-publish. Classifier's read cursor
        // is at seq 1 from the first call; the second call sees only the
        // new edge.
        edges.stage(1, &[9000]);
        edges.on_publish(HwRing::<u16, 8>::LEN - 2); // head at slot 2
        c.on_edge_advance(&mut edges, TPB_3M);
        // After reset, the next edge re-seeds (no anchor) rather than
        // gap-classifying against the old 1000-anchor.
        assert_eq!(c.seeds, 2);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.anchor, Some(9000));
    }

    #[test]
    fn respects_ticks_per_bit_arg() {
        // ticks_since_anchor = 160: HIT at 3M (window 144..176),
        // SKIP at 1M (window 432..528).
        let mut c_3m = make();
        let mut edges_3m = edges8(&[1000, 1160]);
        c_3m.on_edge_advance(&mut edges_3m, TPB_3M);
        assert_eq!(c_3m.hits, 1);
        assert_eq!(c_3m.skips, 0);

        let mut c_1m = make();
        let mut edges_1m = edges8(&[1000, 1160]);
        c_1m.on_edge_advance(&mut edges_1m, TPB_1M);
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
        let mut edges: HwRing<u16, 128> = HwRing::new(0);
        let intra = [0_u16, 32, 64, 96];
        let mut staged = [0_u16; TOTAL];
        for byte_i in 0..N_BYTES {
            let t0 = 1000_u16.wrapping_add((byte_i as u16).wrapping_mul(BYTE_TICKS_3M));
            for j in 0..EDGES_PER_BYTE {
                staged[byte_i * EDGES_PER_BYTE + j] = t0.wrapping_add(intra[j]);
            }
        }
        edges.stage(0, &staged);
        edges.on_publish(HwRing::<u16, 128>::LEN - TOTAL as u16);

        let mut c = make();
        c.on_edge_advance(&mut edges, TPB_3M);

        assert_eq!(c.seeds, 1);
        assert_eq!(c.hits, 15);
        assert_eq!(c.skips, 48);
        assert_eq!(c.gaps, 0);
        assert_eq!(c.byte_ts_head().raw(), 16);

        // BT entries land at byte starts; spacing == one byte-time.
        let bt0 = bt_at(&c, 0).unwrap();
        let bt1 = bt_at(&c, 1).unwrap();
        assert_eq!(bt1.wrapping_sub(bt0), BYTE_TICKS_3M);
    }

    #[test]
    fn byte_ts_at_returns_none_past_head() {
        let mut c = make();
        let mut edges = edges8(&[1000]);
        c.on_edge_advance(&mut edges, TPB_3M);
        assert_eq!(bt_at(&c, 0), Some(1000));
        assert_eq!(bt_at(&c, 1), None);
        assert_eq!(bt_at(&c, 2), None);
    }
}
