//! Header-anchored window classifier — turns falling-edge timestamps
//! (TIM2_CH4 IC) into per-byte start ticks. Anchors come from a back-
//! search of the ET ring for the DXL header's edge signature `FF FF FD
//! 00` (5 edges with intervals `[~10·bit, ~10·bit, ~2·bit, ~8·bit]`,
//! followed by `≥7·bit` quiet). Forward walker uses the `[9·bit, 11·bit]`
//! start-bit window from the anchored byte to identify each next byte.
//! Algorithm specified in `docs/dxl-streaming-rx.md` §4.
//!
//! ## Why the back-search
//! The prior design seeded the anchor on the first edge after IDLE — fine
//! for clean idle gaps, ambiguous at cold boot or post-resync. Header
//! pattern match is unambiguous (DXL stuffing guarantees `FF FF FD` never
//! appears in payload) and re-establishes anchor on every packet, so
//! timing drift never accumulates across packet boundaries.
//!
//! ## Tolerance encoding
//! Per doc §4.4: `±1·bit` on wide intervals (`10·bit`, `8·bit`), `±0.5·bit`
//! on the narrow `2·bit` gap. To keep the half-bit precision integer, all
//! deltas are compared as `delta·2 ≷ MUL_X2 · tpb` with both sides widened
//! to `u32` — no integer division anywhere on the hot path.

use crate::dxl::uart::BITS_PER_FRAME;
use crate::util::HwRing;

// Forward-walker window for the `[9·bit, 11·bit]` start-bit match.
const WINDOW_LO_MUL: u16 = 9;
const WINDOW_HI_MUL: u16 = 11;

// Half-bit-scaled band multipliers for the header pattern. See module
// docs — `_X2` = the multiplier doubled, so the narrow `1.5·bit`/`2.5·bit`
// bounds stay integer when compared against `delta·2`.
const WIDE_LO_X2: u32 = 18; //  9·bit · 2  (10·bit − 1·bit)
const WIDE_HI_X2: u32 = 22; // 11·bit · 2
const MID_LO_X2: u32 = 14; //  7·bit · 2  ( 8·bit − 1·bit)
const MID_HI_X2: u32 = 18; //  9·bit · 2
const NARROW_LO_X2: u32 = 3; // 1.5·bit · 2
const NARROW_HI_X2: u32 = 5; // 2.5·bit · 2
const QUIET_MIN_X2: u32 = 14; //  7·bit · 2

// Edges in the `FF FF FD 00` Sync signature. `idle→0xFF start`, `0xFF
// end→0xFF start`, `0xFF end→0xFD start`, `0xFD bit0→bit1 transition`,
// `0xFD end→0x00 start` = 5 falling edges. Load-bearing in the buffer-
// sizing formulas below.
const SYNC_SIGNATURE_EDGES: usize = 5;

/// Source of a [`Classifier::packet_end_tick_fallback`] call — picks the
/// formula. Each variant maps to one parser-event poll context. Used when
/// interference / edge loss left the classifier unanchored at the parser's
/// Crc event; downstream RDT scheduling needs a tick even if degraded.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FallbackSrc {
    /// HT/TC poll context. Bytes arrived steadily; the CRC byte landed in
    /// DMA ~now (sub-µs ISR-entry latency aside) so `now` is the
    /// packet-end estimate.
    Dma,
    /// IDLE poll context. The wire has been quiet for one idle character
    /// (`BITS_PER_FRAME` bit-times) so the last data byte ended that long
    /// ago — back-date `now` by the idle gap.
    Idle,
}

/// Worst-case anchor back-search depth (in edges) given an `EDGE_BUF_LEN`-
/// slot DMA edge ring polled at HT/TC cadence. Equals `EDGE_BUF_LEN/2 − 5`:
/// the half-buffer drain depth (the staleness budget between HT firings)
/// minus the 5 edges spanned by the Sync signature itself.
///
/// Drives the [`Classifier::try_anchor_from_header`] back-search bound. The
/// chip lib picks the desired lookback as a CPU/RAM budget knob; the byte
/// ring and edge ring are then sized via [`edge_buf_len`] / [`rx_buf_len`]
/// so this relationship holds at compile time.
///
/// ## Cost per +1 increment of the lookback target
/// - **CPU**: ~63 cycles (~1.3 µs at 48 MHz RV32IMC) added to the worst-
///   case Sync-not-found back-search. Per failed offset: 5 ring reads +
///   4 band checks + 1 quiet check. Header events fire once per packet so
///   this is a packet-rate cost, not a per-byte cost.
/// - **RAM**: +4 B edge ring (u16) + 1 B byte ring before power-of-two
///   rounding. After rounding, costs step in PoW2-sized jumps:
///     - lookback 11  → E=32, R=16  =  80 B
///     - lookback 27  → E=64, R=32  = 160 B
///     - lookback 59  → E=128, R=64 = 320 B
///
/// CH32V006 (48 MHz, 8 KiB SRAM) targets 59 — see
/// `firmware/ch32/src/runtime/registry.rs`.
pub const fn sync_lookback_edges(edge_buf_len: usize) -> u16 {
    (edge_buf_len / 2 - SYNC_SIGNATURE_EDGES) as u16
}

/// Minimum edge-ring depth (in `u16` slots, rounded up to a power of two)
/// that satisfies the Sync back-search at the given lookback target. Power-
/// of-two so the chip-side `% EDGE_BUF_LEN` math collapses to AND on the
/// hardware-divider-free CH32V006.
pub const fn edge_buf_len(sync_lookback_edges: u16) -> usize {
    (2 * (sync_lookback_edges as usize + SYNC_SIGNATURE_EDGES)).next_power_of_two()
}

/// Minimum byte-ring depth (in `u8` slots, rounded up to a power of two)
/// that holds one HT half-period of bytes. Bytes drain at `edges / k` where
/// `k ∈ [1, 5]`; worst case `k=1` (high-density `0xFF` content) → one byte
/// per edge → exactly `EDGE_BUF_LEN / 2` bytes between HT firings.
pub const fn rx_buf_len(sync_lookback_edges: u16) -> usize {
    (sync_lookback_edges as usize + SYNC_SIGNATURE_EDGES).next_power_of_two()
}

#[inline]
fn in_band(delta: u16, lo_x2: u32, hi_x2: u32, tpb: u16) -> bool {
    let d_x2 = (delta as u32) * 2;
    let tpb = tpb as u32;
    d_x2 >= lo_x2 * tpb && d_x2 <= hi_x2 * tpb
}

pub struct Classifier {
    /// Tick of the most recently classified byte's start bit. `None` when
    /// no anchor is held (cold boot, post-Crc, post-Resync, post-GAP,
    /// post-failed back-search). Window math `last_byte_start ± 10·bit`
    /// drives subsequent edges; updated per byte boundary by the walker.
    last_byte_start: Option<u16>,
    /// Tick of the byte before `last_byte_start`. Lets the walker emit
    /// `(prev, curr)` pairs to the drift consumer without the consumer
    /// having to thread its own previous-tick state.
    prev_byte_start: Option<u16>,
    /// Drift-sampling enable. The composite sets this at the parser's
    /// Instruction-header event, clears at the matching CRC event (or
    /// earlier on Resync). Status frames never set it — drift sampling
    /// is sourced from host HSE only (doc §5.4).
    hsi_active: bool,

    hits: u16,
    skips: u16,
    gaps: u16,
    header_matches: u16,
    header_mismatches: u16,
}

impl Classifier {
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self {
            last_byte_start: None,
            prev_byte_start: None,
            hsi_active: false,
            hits: 0,
            skips: 0,
            gaps: 0,
            header_matches: 0,
            header_mismatches: 0,
        }
    }

    /// Back-search the ET ring for the DXL header's 5-edge signature.
    /// Called by the composite at every parser Header event. On match,
    /// the anchor is set to edge 5's tick (= byte 3 = `0x00` start bit),
    /// the ET reader cursor advances to one past edge 5, and `true` is
    /// returned. On mismatch, anchor + prev are cleared and `false` is
    /// returned — the next packet's header re-attempts anchor from
    /// scratch (doc §4.4).
    ///
    /// Scans up to [`sync_lookback_edges`]`(EDGE_BUF_LEN) + 1` windows
    /// ending at the most recent published edges, most-recent first.
    /// Accepts the first window whose 4 inter-edge deltas land in the
    /// `[~10, ~10, ~2, ~8]·bit` bands and whose successor edge (if
    /// present) sits beyond the `≥7·bit` quiet floor.
    pub fn try_anchor_from_header<const EDGE_BUF_LEN: usize>(
        &mut self,
        edges: &mut HwRing<u16, EDGE_BUF_LEN>,
        ticks_per_bit: u16,
    ) -> bool {
        let avail = edges.reader().avail();
        crate::log::trace!(
            "classifier: anchor entry avail={} ticks_per_bit={}",
            avail,
            ticks_per_bit
        );
        if avail < 5 {
            crate::log::trace!("classifier: anchor insufficient avail<5");
            return false;
        }

        let max_offset = sync_lookback_edges(EDGE_BUF_LEN).min(avail - 5);
        for offset in 0..=max_offset {
            // Window edges (terminal = e5 at `offset` slots back from
            // the head, e1 the oldest of the window):
            //   e5 = recent(offset),     e4 = recent(offset + 1),
            //   e3 = recent(offset + 2), e2 = recent(offset + 3),
            //   e1 = recent(offset + 4).
            // Quiet check (when offset > 0) reads e6 = recent(offset - 1).
            let e5 = match edges.recent(offset) {
                Some(&v) => v,
                None => continue,
            };
            let e4 = match edges.recent(offset + 1) {
                Some(&v) => v,
                None => continue,
            };
            let e3 = match edges.recent(offset + 2) {
                Some(&v) => v,
                None => continue,
            };
            let e2 = match edges.recent(offset + 3) {
                Some(&v) => v,
                None => continue,
            };
            let e1 = match edges.recent(offset + 4) {
                Some(&v) => v,
                None => continue,
            };

            let d_12 = e2.wrapping_sub(e1);
            let d_23 = e3.wrapping_sub(e2);
            let d_34 = e4.wrapping_sub(e3);
            let d_45 = e5.wrapping_sub(e4);

            if !in_band(d_12, WIDE_LO_X2, WIDE_HI_X2, ticks_per_bit) {
                continue;
            }
            if !in_band(d_23, WIDE_LO_X2, WIDE_HI_X2, ticks_per_bit) {
                continue;
            }
            if !in_band(d_34, NARROW_LO_X2, NARROW_HI_X2, ticks_per_bit) {
                continue;
            }
            if !in_band(d_45, MID_LO_X2, MID_HI_X2, ticks_per_bit) {
                continue;
            }

            // Quiet check: if a successor edge has been published, its
            // delta from e5 must clear the `≥7·bit` floor. If no
            // successor edge yet (offset == 0 at the cold-edge case),
            // accept — byte 4's start bit may not have latched at the
            // moment the parser emitted Header.
            if offset > 0
                && let Some(&e6) = edges.recent(offset - 1)
            {
                let d_56 = e6.wrapping_sub(e5);
                let d_x2 = (d_56 as u32) * 2;
                if d_x2 < QUIET_MIN_X2 * ticks_per_bit as u32 {
                    continue;
                }
            }

            // Match. Anchor = e5 (byte 3 start bit). Seed prev = e5 so
            // the first forward HIT (byte 4) emits the (byte 3, byte 4)
            // pair — the first drift-eligible body-byte transition.
            self.last_byte_start = Some(e5);
            self.prev_byte_start = Some(e5);
            // Advance ET reader to one past e5: e5 sits `offset + 1` slots
            // before the head, so the slot just after e5 is `offset` slots
            // before the head, i.e. distance `avail - offset` from the
            // current read cursor.
            edges.reader().advance(avail - offset);
            self.header_matches = self.header_matches.wrapping_add(1);
            crate::log::debug!(
                "classifier: anchor match offset={} e5={} advance={}",
                offset,
                e5,
                avail - offset
            );
            return true;
        }

        // No window matched: drop any stale anchor; the packet boundary
        // is real even if the pattern didn't resolve.
        self.last_byte_start = None;
        self.prev_byte_start = None;
        self.header_mismatches = self.header_mismatches.wrapping_add(1);
        crate::log::debug!(
            "classifier: anchor pattern miss across {} windows",
            max_offset + 1
        );
        false
    }

    /// Walk newly-published edges, identifying byte starts via the
    /// `[9·bit, 11·bit]` window from `last_byte_start`. Emits each new
    /// `(prev, curr)` byte pair through `on_pair` when `hsi_active` is
    /// set. Bails on the first no-anchor edge without advancing the
    /// reader — those edges stay in the ring for the next parser-Header
    /// event's [`try_anchor_from_header`] back-search. Per doc §4.4,
    /// "until the first header arrives there is no timing state to be
    /// wrong about" — and the back-search needs the producer-side edge
    /// history to find the signature.
    pub fn on_edge_advance<F, const EDGE_BUF_LEN: usize>(
        &mut self,
        edges: &mut HwRing<u16, EDGE_BUF_LEN>,
        ticks_per_bit: u16,
        mut on_pair: F,
    ) where
        F: FnMut(u16, u16),
    {
        let win_lo = ticks_per_bit.wrapping_mul(WINDOW_LO_MUL);
        let win_hi = ticks_per_bit.wrapping_mul(WINDOW_HI_MUL);
        crate::log::trace!(
            "classifier: walk entry avail={} anchor={:?}",
            edges.reader().avail(),
            self.last_byte_start
        );

        let mut reader = edges.reader();
        while let Some(&t) = reader.peek() {
            let Some(a) = self.last_byte_start else {
                // No anchor → leave edge in ring for back-search.
                crate::log::trace!("classifier: walk bail edge={} (no anchor)", t);
                return;
            };
            let delta = t.wrapping_sub(a);
            if delta < win_lo {
                self.skips = self.skips.wrapping_add(1);
                crate::log::trace!("classifier: walk skip edge={} delta={}", t, delta);
                reader.advance(1);
            } else if delta <= win_hi {
                self.prev_byte_start = self.last_byte_start;
                self.last_byte_start = Some(t);
                if self.hsi_active {
                    on_pair(a, t);
                }
                self.hits = self.hits.wrapping_add(1);
                crate::log::trace!("classifier: walk hit edge={} delta={}", t, delta);
                reader.advance(1);
            } else {
                // GAP. Per doc §4.4 — byte-alignment loss; drop anchor +
                // prev and leave the edge in the ring. Mirrors the no-
                // anchor bail: cross-packet stale anchors trip here on
                // the next packet's first signature edge, and the back-
                // search needs that edge to find the signature.
                self.last_byte_start = None;
                self.prev_byte_start = None;
                self.gaps = self.gaps.wrapping_add(1);
                crate::log::trace!("classifier: walk gap edge={} delta={}", t, delta);
                return;
            }
        }
    }

    /// USART1 IDLE backstop. Drains pending tail edges through the
    /// walker. Anchor + drift gating live until the parser's Crc /
    /// Resync event (composite calls [`reset_anchor`] there) — IDLE
    /// only means "no more edges right now," which doesn't imply
    /// state invalidation. The deterministic packet boundary is the
    /// parser-side Crc, not the wire-side quiet.
    #[allow(dead_code)]
    pub fn on_idle<F, const EDGE_BUF_LEN: usize>(
        &mut self,
        edges: &mut HwRing<u16, EDGE_BUF_LEN>,
        ticks_per_bit: u16,
        on_pair: F,
    ) where
        F: FnMut(u16, u16),
    {
        crate::log::trace!(
            "classifier: on_idle entry avail={} anchor={:?}",
            edges.reader().avail(),
            self.last_byte_start
        );
        self.on_edge_advance(edges, ticks_per_bit, on_pair);
    }

    /// Tick of the most recently classified byte's start bit. Composite
    /// reads at parser event-handling time for one-shot stamps.
    #[allow(dead_code)]
    pub fn current_byte_tick(&self) -> Option<u16> {
        self.last_byte_start
    }

    /// Wire-end tick of the most recently classified byte. Composite
    /// stamps `packet_end_tick` at the parser's CRC-good event — the
    /// CRC byte's start sits at `last_byte_start`, the wire-end one
    /// byte-time later.
    #[allow(dead_code)]
    pub fn packet_end_tick(&self, ticks_per_bit: u16) -> Option<u16> {
        self.last_byte_start
            .map(|t| t.wrapping_add(ticks_per_bit.wrapping_mul(BITS_PER_FRAME)))
    }

    /// Fallback packet-end estimate for the no-anchor case — the composite
    /// calls this at Crc when [`packet_end_tick`] returns `None` (interference
    /// or edge loss starved the classifier). Formula per `src`:
    ///
    /// - [`FallbackSrc::Dma`]: returns `now`. CRC byte landed in DMA
    ///   ~immediately before the poll, so `now` IS packet-end with negligible
    ///   ISR-entry offset.
    /// - [`FallbackSrc::Idle`]: returns `now − BITS_PER_FRAME · ticks_per_bit`.
    ///   USART1 IDLE asserts one idle character after the last data byte's
    ///   stop bit, so back-date by that interval.
    ///
    /// Bumps no state; the composite increments the telemetry counter via
    /// [`crate::traits::dxl::RxDma::record_edge_anchor_miss`] alongside.
    #[allow(dead_code)]
    pub fn packet_end_tick_fallback(&self, src: FallbackSrc, now: u16, ticks_per_bit: u16) -> u16 {
        match src {
            FallbackSrc::Dma => now,
            FallbackSrc::Idle => {
                let gap = (BITS_PER_FRAME as u32).wrapping_mul(ticks_per_bit as u32);
                debug_assert!(
                    gap < u16::MAX as u32,
                    "idle gap exceeds TIM2 wrap window — unreachable at supported baud rates",
                );
                now.wrapping_sub(gap as u16)
            }
        }
    }

    #[allow(dead_code)]
    pub fn set_hsi_active(&mut self, on: bool) {
        self.hsi_active = on;
    }

    #[allow(dead_code)]
    pub fn hsi_active(&self) -> bool {
        self.hsi_active
    }

    /// Force-drop anchor, prev, and the drift flag. Composite calls at
    /// the parser's Crc event (deterministic packet boundary — state is
    /// stale once the packet ended) and Resync event (mid-packet abort
    /// — discard any in-flight timing state).
    #[allow(dead_code)]
    pub fn reset_anchor(&mut self) {
        self.last_byte_start = None;
        self.prev_byte_start = None;
        self.hsi_active = false;
    }

    /// Force `last_byte_start` to a known tick without staging real edges.
    /// Composite-test scaffolding so [`packet_end_tick`] reads a deterministic
    /// value at Crc-good simulation. Production code mutates this only
    /// through the walker.
    #[cfg(test)]
    pub fn force_byte_tick_for_test(&mut self, tick: u16) {
        self.last_byte_start = Some(tick);
        self.prev_byte_start = Some(tick);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::util::HwRing;

    // 3 Mbaud at HCLK 48 MHz → ticks_per_bit = 16. One byte = 160 ticks.
    const TPB_3M: u16 = 16;
    const BYTE_TICKS_3M: u16 = 160;
    // 1 Mbaud at HCLK 48 MHz → ticks_per_bit = 48. One byte = 480 ticks.
    const TPB_1M: u16 = 48;
    const BYTE_TICKS_1M: u16 = 480;

    fn make() -> Classifier {
        Classifier::new()
    }

    /// Stage edges into a 16-slot HwRing and publish the producer head
    /// at `vals.len()`. Returns the populated ring.
    fn edges16(vals: &[u16]) -> HwRing<u16, 16> {
        let mut b: HwRing<u16, 16> = HwRing::new(0);
        b.stage(0, vals);
        b.on_publish(HwRing::<u16, 16>::LEN - vals.len() as u16);
        b
    }

    /// Synthesize a clean DXL header at base tick `t0` and bit-time `tpb`.
    /// Returns the 5 falling-edge timestamps.
    fn header_edges(t0: u16, tpb: u16) -> [u16; 5] {
        let bt = tpb.wrapping_mul(10);
        [
            t0,                                  // byte 0 (0xFF) start
            t0.wrapping_add(bt),                 // byte 1 (0xFF) start
            t0.wrapping_add(bt.wrapping_mul(2)), // byte 2 (0xFD) start
            t0.wrapping_add(bt.wrapping_mul(2))
                .wrapping_add(tpb.wrapping_mul(2)), // 0xFD bit0→1 transition
            t0.wrapping_add(bt.wrapping_mul(3)), // byte 3 (0x00) start
        ]
    }

    #[test]
    fn back_search_matches_3m_header_pattern() {
        let mut c = make();
        let h = header_edges(1000, TPB_3M);
        let mut edges = edges16(&h);
        assert!(c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.last_byte_start, Some(h[4]));
        assert_eq!(c.prev_byte_start, Some(h[4]));
        assert_eq!(c.header_matches, 1);
        assert_eq!(c.header_mismatches, 0);
    }

    #[test]
    fn back_search_matches_1m_header_pattern() {
        let mut c = make();
        let h = header_edges(2000, TPB_1M);
        let mut edges = edges16(&h);
        assert!(c.try_anchor_from_header(&mut edges, TPB_1M));
        assert_eq!(c.last_byte_start, Some(h[4]));
    }

    #[test]
    fn back_search_rejects_too_few_edges() {
        let mut c = make();
        // Only 4 of the 5 header edges present.
        let mut edges = edges16(&[1000, 1160, 1320, 1352]);
        assert!(!c.try_anchor_from_header(&mut edges, TPB_3M));
        // No anchor seeded; counter doesn't tick a mismatch either —
        // insufficient data is distinct from pattern rejection.
        assert_eq!(c.last_byte_start, None);
        assert_eq!(c.header_matches, 0);
        assert_eq!(c.header_mismatches, 0);
    }

    #[test]
    fn back_search_rejects_wrong_narrow_gap() {
        let mut c = make();
        let mut h = header_edges(1000, TPB_3M);
        // Perturb the 2·bit gap (e3→e4) to 4·bit.
        h[3] = h[2].wrapping_add(TPB_3M.wrapping_mul(4));
        let mut edges = edges16(&h);
        assert!(!c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.header_mismatches, 1);
    }

    #[test]
    fn back_search_rejects_wrong_wide_interval() {
        let mut c = make();
        let mut h = header_edges(1000, TPB_3M);
        // Perturb e1→e2 to 13·bit (way out of [9, 11]·bit).
        h[1] = h[0].wrapping_add(TPB_3M.wrapping_mul(13));
        h[2] = h[1].wrapping_add(TPB_3M.wrapping_mul(10));
        h[3] = h[2].wrapping_add(TPB_3M.wrapping_mul(2));
        h[4] = h[3].wrapping_add(TPB_3M.wrapping_mul(8));
        let mut edges = edges16(&h);
        assert!(!c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.header_mismatches, 1);
    }

    #[test]
    fn back_search_disambiguates_stuffed_payload() {
        // FF FF FD FD near-match: the 6th edge would sit ~2·bit after e5
        // (the second 0xFD's bit-0→bit-1 transition) instead of at the
        // quiet ≥7·bit gap. Verifies the quiet check guards against
        // stuffed payload near-matches.
        let mut c = make();
        let mut h = header_edges(1000, TPB_3M).to_vec();
        // Push a 6th edge at e5 + 2·bit (would be quiet ≥7·bit on a real
        // header).
        h.push(h[4].wrapping_add(TPB_3M.wrapping_mul(2)));
        let mut edges = edges16(&h);
        // With offset==0, the 5 most recent edges are [e2, e3, e4, e5, e6];
        // those deltas don't match the pattern, so window 0 rejects.
        // With offset==1, the 5-edge window is [e1, e2, e3, e4, e5];
        // pattern matches BUT the e6 quiet check fires and rejects.
        assert!(!c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.header_mismatches, 1);
    }

    #[test]
    fn back_search_advances_read_cursor_past_header_edges() {
        let mut c = make();
        let h = header_edges(1000, TPB_3M);
        let mut edges = edges16(&h);
        assert!(c.try_anchor_from_header(&mut edges, TPB_3M));
        // Walker's next call should see zero available edges — the 5
        // header edges were consumed by the back-search advance.
        assert_eq!(edges.reader().avail(), 0);
    }

    #[test]
    fn back_search_with_one_extra_edge_finds_window_at_offset_1() {
        let mut c = make();
        let mut h = header_edges(1000, TPB_3M).to_vec();
        // Add byte 4's start bit at e5 + 10·bit (clears the ≥7·bit quiet
        // floor) — exercises offset>0 path with a clean quiet check.
        h.push(h[4].wrapping_add(BYTE_TICKS_3M));
        let mut edges = edges16(&h);
        assert!(c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.last_byte_start, Some(h[4]));
        // Reader advanced past e5 — byte 4's edge is still pending.
        assert_eq!(edges.reader().avail(), 1);
    }

    #[test]
    fn back_search_mismatch_drops_stale_anchor() {
        let mut c = make();
        c.last_byte_start = Some(7777);
        c.prev_byte_start = Some(7777);
        let mut edges = edges16(&[1000, 1160, 1320, 1352, 9999]);
        // The 4·bit delta from e4 (1352) to e5 (9999) is way out of band.
        assert!(!c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.last_byte_start, None);
        assert_eq!(c.prev_byte_start, None);
    }

    #[test]
    fn walker_without_anchor_leaves_edges_in_ring() {
        let mut c = make();
        let mut edges = edges16(&[1000, 1160, 1320]);
        let avail_before = edges.reader().avail();
        let mut emissions = 0_u32;
        c.on_edge_advance(&mut edges, TPB_3M, |_, _| emissions += 1);
        assert_eq!(c.last_byte_start, None);
        assert_eq!(c.hits, 0);
        assert_eq!(c.skips, 0);
        assert_eq!(c.gaps, 0);
        assert_eq!(emissions, 0);
        // Fix for the Ping-canary bug: edges must NOT be consumed when
        // no anchor is held — `try_anchor_from_header` needs the
        // producer-side edge history to back-search for the signature.
        assert_eq!(edges.reader().avail(), avail_before);
    }

    #[test]
    fn walker_with_no_anchor_preserves_header_signature_for_back_search() {
        // Regression for the Ping-canary failure: walker ran first over a
        // freshly-published header signature with no anchor, silently
        // consumed all 5 edges, then `try_anchor_from_header` saw avail=0
        // and the composite never stamped `packet_end_tick`, so the
        // dispatcher dropped the reply schedule. The fixed walker bails
        // on no-anchor; the signature stays available for the anchor.
        let mut c = make();
        let h = header_edges(1000, TPB_3M);
        let mut edges = edges16(&h);

        c.on_edge_advance(&mut edges, TPB_3M, |_, _| {});
        assert_eq!(edges.reader().avail(), 5, "walker must leave edges");

        assert!(c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.last_byte_start, Some(h[4]));
        assert_eq!(c.header_matches, 1);
    }

    #[test]
    fn walker_hit_emits_pair_when_active() {
        let mut c = make();
        c.last_byte_start = Some(1000);
        c.prev_byte_start = Some(1000);
        c.hsi_active = true;
        let mut edges = edges16(&[1000 + BYTE_TICKS_3M]);
        let mut pairs: heapless::Vec<(u16, u16), 4> = heapless::Vec::new();
        c.on_edge_advance(&mut edges, TPB_3M, |p, n| {
            pairs.push((p, n)).unwrap();
        });
        assert_eq!(pairs.as_slice(), &[(1000, 1000 + BYTE_TICKS_3M)]);
        assert_eq!(c.hits, 1);
        assert_eq!(c.last_byte_start, Some(1000 + BYTE_TICKS_3M));
        assert_eq!(c.prev_byte_start, Some(1000));
    }

    #[test]
    fn walker_hit_skips_pair_when_inactive() {
        let mut c = make();
        c.last_byte_start = Some(1000);
        c.prev_byte_start = Some(1000);
        c.hsi_active = false;
        let mut edges = edges16(&[1000 + BYTE_TICKS_3M]);
        let mut emissions = 0_u32;
        c.on_edge_advance(&mut edges, TPB_3M, |_, _| emissions += 1);
        assert_eq!(emissions, 0);
        // Hit still classified — flag only gates emission, not classification.
        assert_eq!(c.hits, 1);
        assert_eq!(c.last_byte_start, Some(1000 + BYTE_TICKS_3M));
    }

    #[test]
    fn walker_skip_drops_intra_byte_edge() {
        let mut c = make();
        c.last_byte_start = Some(1000);
        c.prev_byte_start = Some(1000);
        c.hsi_active = true;
        // 32 ticks = 2·bit — intra-byte for 0xAA at 3 Mbaud.
        let mut edges = edges16(&[1032]);
        let mut emissions = 0_u32;
        c.on_edge_advance(&mut edges, TPB_3M, |_, _| emissions += 1);
        assert_eq!(c.skips, 1);
        assert_eq!(c.hits, 0);
        assert_eq!(c.last_byte_start, Some(1000));
        assert_eq!(emissions, 0);
    }

    #[test]
    fn walker_gap_drops_anchor_and_preserves_edge() {
        let mut c = make();
        c.last_byte_start = Some(1000);
        c.prev_byte_start = Some(1000);
        c.hsi_active = true;
        // 13·bit delta is way out of [9, 11]·bit → GAP.
        let mut edges = edges16(&[1000 + TPB_3M * 13]);
        let avail_before = edges.reader().avail();
        let mut emissions = 0_u32;
        c.on_edge_advance(&mut edges, TPB_3M, |_, _| emissions += 1);
        assert_eq!(c.gaps, 1);
        assert_eq!(c.hits, 0);
        assert_eq!(c.last_byte_start, None);
        assert_eq!(c.prev_byte_start, None);
        assert_eq!(emissions, 0);
        // Edge stays in the ring so the next parser-Header back-search
        // has producer-side material to find the signature — cross-
        // packet stale-anchor case relies on this.
        assert_eq!(edges.reader().avail(), avail_before);
    }

    #[test]
    fn stale_anchor_then_new_header_re_anchors_via_back_search() {
        // Cross-packet regression: classifier holds a stale anchor from a
        // prior packet; the next packet's first signature edge arrives
        // with a huge delta from the stale anchor → walker GAP arm
        // clears the anchor and leaves the edge available; the remaining
        // signature edges accumulate (no anchor → walker bails) until
        // try_anchor_from_header back-searches the full 5-edge window.
        let mut c = make();
        c.last_byte_start = Some(1000);
        c.prev_byte_start = Some(1000);

        let h = header_edges(50_000, TPB_3M);
        let mut edges = edges16(&h);

        c.on_edge_advance(&mut edges, TPB_3M, |_, _| {});
        assert_eq!(c.gaps, 1, "first edge of new packet trips GAP");
        assert_eq!(c.last_byte_start, None);
        assert_eq!(edges.reader().avail(), 5, "no edges consumed");

        assert!(c.try_anchor_from_header(&mut edges, TPB_3M));
        assert_eq!(c.last_byte_start, Some(h[4]));
    }

    #[test]
    fn walker_consecutive_hits_chain_pairs() {
        let mut c = make();
        c.last_byte_start = Some(1000);
        c.prev_byte_start = Some(1000);
        c.hsi_active = true;
        let t1 = 1000 + BYTE_TICKS_3M;
        let t2 = t1 + BYTE_TICKS_3M;
        let t3 = t2 + BYTE_TICKS_3M;
        let mut edges = edges16(&[t1, t2, t3]);
        let mut pairs: heapless::Vec<(u16, u16), 4> = heapless::Vec::new();
        c.on_edge_advance(&mut edges, TPB_3M, |p, n| {
            pairs.push((p, n)).unwrap();
        });
        assert_eq!(pairs.as_slice(), &[(1000, t1), (t1, t2), (t2, t3)]);
        assert_eq!(c.hits, 3);
    }

    #[test]
    fn current_byte_tick_returns_last_byte_start() {
        let mut c = make();
        assert_eq!(c.current_byte_tick(), None);
        c.last_byte_start = Some(4242);
        assert_eq!(c.current_byte_tick(), Some(4242));
    }

    #[test]
    fn packet_end_tick_adds_10bit() {
        let mut c = make();
        assert_eq!(c.packet_end_tick(TPB_3M), None);
        c.last_byte_start = Some(5000);
        assert_eq!(c.packet_end_tick(TPB_3M), Some(5000 + BYTE_TICKS_3M));
        assert_eq!(c.packet_end_tick(TPB_1M), Some(5000 + BYTE_TICKS_1M));
    }

    #[test]
    fn fallback_dma_returns_now_unchanged() {
        let c = make();
        // Dma path: regardless of anchor state, packet_end ≈ now.
        assert_eq!(
            c.packet_end_tick_fallback(FallbackSrc::Dma, 1234, TPB_3M),
            1234
        );
        assert_eq!(c.packet_end_tick_fallback(FallbackSrc::Dma, 0, TPB_3M), 0);
        assert_eq!(
            c.packet_end_tick_fallback(FallbackSrc::Dma, u16::MAX, TPB_3M),
            u16::MAX,
        );
    }

    #[test]
    fn fallback_idle_back_dates_by_one_idle_char() {
        let c = make();
        // 3 Mbaud: gap = 10·16 = 160 ticks.
        assert_eq!(
            c.packet_end_tick_fallback(FallbackSrc::Idle, 1600, TPB_3M),
            1600 - BYTE_TICKS_3M,
        );
        // 1 Mbaud: gap = 10·48 = 480 ticks.
        assert_eq!(
            c.packet_end_tick_fallback(FallbackSrc::Idle, 2000, TPB_1M),
            2000 - BYTE_TICKS_1M,
        );
    }

    #[test]
    fn fallback_idle_wraps_at_tim2_boundary() {
        // `now = 5`, idle gap = 160 → packet_end wraps below 0.
        let c = make();
        let expected = 5_u16.wrapping_sub(BYTE_TICKS_3M);
        assert_eq!(
            c.packet_end_tick_fallback(FallbackSrc::Idle, 5, TPB_3M),
            expected,
        );
    }

    #[test]
    fn fallback_does_not_read_anchor_state() {
        // Both formulas are pure in (src, now, ticks_per_bit); anchor
        // state must not influence the result.
        let mut c = make();
        let unset = c.packet_end_tick_fallback(FallbackSrc::Dma, 4242, TPB_3M);
        c.last_byte_start = Some(9999);
        c.prev_byte_start = Some(9999);
        let set = c.packet_end_tick_fallback(FallbackSrc::Dma, 4242, TPB_3M);
        assert_eq!(unset, set);
    }

    #[test]
    fn reset_anchor_clears_state() {
        let mut c = make();
        c.last_byte_start = Some(1234);
        c.prev_byte_start = Some(1100);
        c.hsi_active = true;
        c.reset_anchor();
        assert_eq!(c.last_byte_start, None);
        assert_eq!(c.prev_byte_start, None);
        assert!(!c.hsi_active);
    }

    #[test]
    fn on_idle_drains_tail_without_resetting_anchor() {
        let mut c = make();
        c.last_byte_start = Some(1000);
        c.prev_byte_start = Some(1000);
        c.hsi_active = true;
        let mut edges = edges16(&[1000 + BYTE_TICKS_3M]);
        let mut pairs = 0_u32;
        c.on_idle(&mut edges, TPB_3M, |_, _| pairs += 1);
        // Tail edge classified as a hit, pair emitted; anchor + drift
        // flag stay — only the parser-side Crc / Resync event
        // ([`reset_anchor`]) invalidates them.
        assert_eq!(pairs, 1);
        assert_eq!(c.hits, 1);
        assert_eq!(c.last_byte_start, Some(1000 + BYTE_TICKS_3M));
        assert_eq!(c.prev_byte_start, Some(1000));
        assert!(c.hsi_active);
    }

    #[test]
    fn on_idle_preserves_hsi_active() {
        let mut c = make();
        c.hsi_active = true;
        let mut edges = edges16(&[]);
        c.on_idle(&mut edges, TPB_3M, |_, _| {});
        assert!(c.hsi_active);
    }

    #[test]
    fn header_match_then_walker_advances_through_body() {
        // Full happy path: synthesize header at t=1000 plus 4 body bytes
        // of 0xFF (1 edge/byte at byte_ticks_3M spacing). Back-search
        // anchors at byte 3; walker classifies the 4 body bytes as hits
        // and emits 4 (prev, curr) pairs when hsi_active.
        let mut c = make();
        let h = header_edges(1000, TPB_3M);
        let body_start = h[4].wrapping_add(BYTE_TICKS_3M);
        let body: [u16; 4] = [
            body_start,
            body_start.wrapping_add(BYTE_TICKS_3M),
            body_start.wrapping_add(BYTE_TICKS_3M.wrapping_mul(2)),
            body_start.wrapping_add(BYTE_TICKS_3M.wrapping_mul(3)),
        ];

        let mut combined: heapless::Vec<u16, 16> = heapless::Vec::new();
        combined.extend_from_slice(&h).unwrap();
        let mut edges = edges16(&combined);

        assert!(c.try_anchor_from_header(&mut edges, TPB_3M));
        c.hsi_active = true;

        // Stage the body edges as if they arrived next.
        let mut edges2 = edges; // move
        edges2.stage(5, &body);
        edges2.on_publish(HwRing::<u16, 16>::LEN - 5 - body.len() as u16);

        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.on_edge_advance(&mut edges2, TPB_3M, |p, n| {
            pairs.push((p, n)).unwrap();
        });
        assert_eq!(
            pairs.as_slice(),
            &[
                (h[4], body[0]),
                (body[0], body[1]),
                (body[1], body[2]),
                (body[2], body[3]),
            ]
        );
        assert_eq!(c.hits, 4);
    }

    #[test]
    fn respects_ticks_per_bit_arg() {
        // Same delta at different baud → HIT vs SKIP, exercising the
        // ticks_per_bit-scaled window.
        let mut c_3m = make();
        c_3m.last_byte_start = Some(1000);
        c_3m.prev_byte_start = Some(1000);
        c_3m.hsi_active = false;
        let mut e1 = edges16(&[1160]);
        c_3m.on_edge_advance(&mut e1, TPB_3M, |_, _| {});
        assert_eq!(c_3m.hits, 1);
        assert_eq!(c_3m.skips, 0);

        let mut c_1m = make();
        c_1m.last_byte_start = Some(1000);
        c_1m.prev_byte_start = Some(1000);
        c_1m.hsi_active = false;
        let mut e2 = edges16(&[1160]);
        c_1m.on_edge_advance(&mut e2, TPB_1M, |_, _| {});
        assert_eq!(c_1m.hits, 0);
        assert_eq!(c_1m.skips, 1);
    }
}
