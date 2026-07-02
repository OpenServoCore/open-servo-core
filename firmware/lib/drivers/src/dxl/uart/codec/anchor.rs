//! Tail-anchor state for the edge parser. A co-varying data record, not a
//! driver: the four fields move as a unit across the parser's two
//! operations (anchor-at-tail signature match, retroactive walk). Split out
//! of [`super::edge_parser::EdgeParser`] so the parser is a pure wire-walker
//! over a borrowed cache — the algorithm and the state it threads no longer
//! share one struct.

/// Tail-signature byte count — the codec always passes 4 bytes
/// ([data_last_2, data_last_1, CRC_lo, CRC_hi]) to
/// [`super::edge_parser::EdgeParser::anchor_at_tail`]. Load-bearing for the
/// walker's [`AnchorCache::tail_starts`] cache size and the offset math that
/// reseeds the walker past the cached-pair region.
pub(super) const TAIL_STARTS: usize = 4;

/// Co-varying tail-anchor state the edge parser reads and writes. Pure data
/// (per driver-pattern §4 pure-data carve-out) — the parser owns the
/// algorithm, this owns the state it operates on.
pub(super) struct AnchorCache {
    /// Tail-signature back-search anchor — the LAST tail byte's start tick,
    /// set by [`super::edge_parser::EdgeParser::anchor_at_tail`] at the
    /// parser's Crc event. Sole source for
    /// [`super::edge_parser::EdgeParser::packet_end_tick`]; also the walk
    /// seed. Cleared at packet boundary via [`Self::reset`].
    pub(super) tail_anchor: Option<u16>,
    /// Start-edge stamps of the four tail-signature bytes, oldest first
    /// (`tail_starts[3]` == `tail_anchor` on match). Populated during
    /// signature validation — the walk already visits each byte's start
    /// edge for band-checking, so caching them costs a few u16 writes.
    /// Consumed by the walker to emit up to three
    /// `(tail_starts[k], tail_starts[k+1])` pairs directly, skipping three
    /// per-step ring scans in steady mode. Valid iff `tail_anchor.is_some()`.
    pub(super) tail_starts: [u16; TAIL_STARTS],
    /// Ring offset of `tail_starts[0]` — the OLDEST cached start edge — so
    /// the walker can seed the ring walk from there when it needs pairs
    /// beyond the cache. Same lifecycle as [`Self::tail_starts`].
    pub(super) tail_starts_oldest_ring_off: Option<u16>,
    /// Per-baud RX edge-stamp compensation, in IC-stamp ticks. Every stamp
    /// read from the edge ring gets `wrapping_sub(rx_edge_comp_ticks)`
    /// applied before any internal use, so stored state, band checks, and
    /// pushed `(prev, curr)` pairs all live in wire-edge time. Pushed via
    /// [`Self::set_edge_comp`] at driver construction and after every
    /// applied baud change. Default `0` until the first push — safe (no
    /// anchor exists yet at default) and matches the no-compensation case
    /// on chips that ship a zero-delay stamp path.
    pub(super) rx_edge_comp_ticks: u16,
}

impl AnchorCache {
    pub(super) const fn new() -> Self {
        Self {
            tail_anchor: None,
            tail_starts: [0u16; TAIL_STARTS],
            tail_starts_oldest_ring_off: None,
            rx_edge_comp_ticks: 0,
        }
    }

    /// Force-drop the tail anchor. Codec calls at the parser's Crc event
    /// (deterministic packet boundary) and Resync event (mid-packet abort).
    pub(super) fn reset(&mut self) {
        self.tail_anchor = None;
        self.tail_starts_oldest_ring_off = None;
    }

    /// Refresh the per-baud RX edge-stamp compensation. Routed from the
    /// clock's currently-applied baud — once at construction and once after
    /// every applied baud change.
    pub(super) fn set_edge_comp(&mut self, rx_edge_comp_ticks: u16) {
        self.rx_edge_comp_ticks = rx_edge_comp_ticks;
    }

    /// Read the tail-anchor tick in wire-edge time.
    pub(super) fn tail_anchor(&self) -> Option<u16> {
        self.tail_anchor
    }
}
