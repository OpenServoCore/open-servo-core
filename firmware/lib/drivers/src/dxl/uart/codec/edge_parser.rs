//! Streaming edge parser — turns falling-edge timestamps (TIM2_CH4 IC)
//! into the packet's wire-end tick and the trailing byte-pair samples the
//! HSI-drift integrator feeds on.
//!
//! ## Two operations, both event-driven at Crc
//!
//! - **Anchor at tail signature** ([`anchor_at_tail`]). At the parser's
//!   `Event::Crc`, hint-first back-search over the ET ring using the last
//!   few raw wire bytes the parser just consumed as signature. Sets
//!   `tail_anchor` to the CRC byte's start tick on match.
//! - **Retroactive integrator walk** ([`walk_pairs_back`]). Called AFTER
//!   the wire-schedule call at Crc so the tail-anchor jitter sits off the
//!   deadline path. Walks backward from `tail_anchor` in
//!   `BITS_PER_FRAME·tpb` strides, snapping each predicted position to
//!   the nearest captured IC edge within a ±`HSI_WALK_SNAP_BITS·tpb`
//!   window; emits `(prev, curr)` pairs for [`crate::dxl::uart::clock::Clock::on_byte_pair`].
//!
//! Both operations subtract the cache's `rx_edge_comp_ticks` from every
//! ring read so stored state, band checks, and emitted pairs all live in
//! wire-edge time rather than IC-filter-output time.

use super::anchor::{AnchorCache, TAIL_STARTS};
use crate::dxl::uart::BITS_PER_FRAME;
use crate::dxl::uart::poll_src::PollSrc;
use crate::ring::HwRing;

/// Falling-edge count per UART byte value, indexed by the byte. Edges
/// come from `1→0` transitions in the bit stream
/// `[idle=1, start=0, b.bit0..b.bit7, stop=1, idle=1]`: always one
/// `idle→start` falling edge, plus one for every `(bit_i=1, bit_{i+1}=0)`
/// pair in the data bits. Stop and trailing transitions never fall.
///
/// Used by [`anchor_at_tail`] to convert the parser's just-
/// consumed tail-byte slice into an expected total edge count so the
/// signature match locks against the exact per-byte edge pattern; codec
/// reaches it through [`edges_in_byte`] to sum the `d_min` shift over
/// bytes drained past CRC. Private — the table is the edge parser's own
/// wire-model knowledge.
const EDGES_PER_BYTE: [u8; 256] = {
    let mut t = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let b = i as u8;
        t[i] = 1 + (b & !(b >> 1) & 0x7F).count_ones() as u8;
        i += 1;
    }
    t
};

/// Falling-edge count for one wire byte. The codec's tail-anchor
/// back-search origin (`d_min`) sums this over the bytes DMA latched past
/// CRC in the RX ring — those bytes' edges sit between the ET head and
/// CRC's tail edge. Keeps [`EDGES_PER_BYTE`] private to the edge parser
/// while the byte-ring iteration stays with the codec that owns that ring.
pub(super) fn edges_in_byte(b: u8) -> u8 {
    EDGES_PER_BYTE[b as usize]
}

/// Max entry in [`EDGES_PER_BYTE`]: start bit + four `1→0` transitions in
/// alternating data bits (e.g. `0x55`, `0xAA`). Load-bearing in
/// [`edge_buf_len`]'s worst-case sizing.
pub const MAX_EDGES_PER_BYTE: u8 = 5;

/// Upper slack on the tail-anchor search window. `EDGES_PER_BYTE` sums to
/// the *minimum* edge count between two known ring positions on a clean
/// noise-free wire. Real wire only adds edges past that floor: in-flight
/// partial bytes (start bit captured before DMA commits the byte), EMI
/// spurious edges, back-slice residue from previous polls. Both search
/// slack (the anchor probe window `[D_min ..= D_min + PADDING]` in
/// [`anchor_at_tail`]) and per-poll ring headroom size against
/// this.
const PADDING_EDGES_AHEAD: u16 = 5;

/// Snap half-width for [`walk_pairs_back`], in bit-times. Sets
/// how far the predicted position can drift from the captured stamp before
/// the walk free-runs on that step. `2·bit` at 3M = 32 ticks ≈ 0.66 µs —
/// wide enough to absorb factory HSI drift plus IC-stamp jitter, tight
/// enough that spurious intra-byte edges outside the expected byte
/// boundary don't spoof a HIT.
const HSI_WALK_SNAP_BITS: u16 = 2;

/// Minimum edge-ring depth (in `u16` slots, rounded up to a power of two)
/// derived from the byte-ring size. Power-of-two so the chip-side
/// `% EDGE_BUF_LEN` math collapses to AND on the hardware-divider-free
/// CH32V006.
///
/// The edge ring must hold every falling-edge stamp produced by the bytes
/// in one poll batch. Under byte-ring HT/TC drain, one batch is at most
/// `rx_buf_len / 2` bytes (bytes accumulated between two half-transfer
/// events), each contributing at most [`MAX_EDGES_PER_BYTE`] edges. Add
/// [`PADDING_EDGES_AHEAD`] for noise + one in-flight partial-byte start-
/// bit edge whose byte is still shifting on the wire at poll entry.
///
/// [`walk_pairs_back`]'s reach beyond the batch is best-
/// effort: on missing edges it free-runs and emits fewer HSI drift pairs
/// (slower convergence, never incorrect). It lives in the headroom
/// `next_power_of_two` rounding leaves.
pub const fn edge_buf_len(rx_buf_len: usize) -> usize {
    let et_min = (rx_buf_len / 2) * MAX_EDGES_PER_BYTE as usize + PADDING_EDGES_AHEAD as usize;
    et_min.next_power_of_two()
}

/// Tail-signature band check. Intra-byte transitions (two edges inside one
/// byte) get a ±0.5·bit window — drift can't accumulate inside a single
/// byte, so the floor is capture jitter. Cross-byte transitions (last edge
/// of byte N → first edge of byte N+1) get ±1·bit — same as a wide Sync
/// band — to absorb factory HSI drift plus residual idle-gap jitter.
#[inline]
fn tail_delta_in_band(delta: u16, expected_bits: u16, tpb: u16, cross_byte: bool) -> bool {
    let actual_x2 = (delta as u32) * 2;
    let expected_x2 = (expected_bits as u32) * 2 * (tpb as u32);
    let tol_x2 = if cross_byte {
        2 * tpb as u32
    } else {
        tpb as u32
    };
    actual_x2.abs_diff(expected_x2) <= tol_x2
}

/// Falling-edge positions within a single UART byte, expressed as bit-time
/// offsets from the start bit. Always includes position 0 (the start
/// edge). Returns `(positions, count)` — at most 5 entries (start + up to
/// 4 intra-byte `1→0` transitions across `d0..d7`; stop=1 produces no
/// trailing edge). UART stream is LSB-first
/// `[idle=1, start=0, d0, d1, …, d7, stop=1]`; an edge lands at position
/// `p` iff bit at `p-1` is `1` and bit at `p` is `0`. Position 0 is the
/// `idle→start` transition (always present).
#[inline]
fn byte_edge_positions(b: u8) -> ([u8; 5], usize) {
    let mut positions = [0u8; 5];
    let mut n = 1;
    let mut prev = 0u8;
    let mut i = 0;
    while i < 8 {
        let cur = (b >> i) & 1;
        if prev == 1 && cur == 0 {
            positions[n] = (i + 1) as u8;
            n += 1;
        }
        prev = cur;
        i += 1;
    }
    (positions, n)
}

/// Lift a 16-bit IC stamp into the WireClock u32 domain. The contract
/// (see [`crate::traits::dxl::WireClock`]) guarantees the low 16 bits of
/// `ref_tick` equal the IC stamp captured at the same instant, so the
/// modular delta from `stamp` to `ref_tick as u16` is the elapsed ticks —
/// subtract that from `ref_tick` to recover the u32 reading at capture
/// time. **Caller must ensure `ref_tick − stamp_u32 < 65536`**; if the
/// reference sits more than a u16-wrap past the stamp the result aliases
/// to a value one wrap too high. See [`drain_ref`] for the per-source
/// reference correction the composite applies before calling.
#[inline]
fn lift(stamp: u16, ref_tick: u32) -> u32 {
    let delta = (ref_tick as u16).wrapping_sub(stamp) as u32;
    ref_tick.wrapping_sub(delta)
}

/// Correct an ISR-entry `now` back into the u16-wrap window above the tail
/// anchor so [`lift`] doesn't alias under sub-wrap baud math. At low baud
/// (9600), `now − stamp` at IDLE drain is `2·BITS_PER_FRAME·tpb = 100_000`
/// ticks > 65_536 (u16 wrap @ 48 MHz HCLK) — `lift(stamp, now)` silently
/// aliases one wrap too high (~1.365 ms drift). HT/TC drain runs ~1·tpb
/// after the start bit, always within one wrap.
///
/// At Idle we subtract `BITS_PER_FRAME · tpb` (= 10·tpb) — NOT `2·tpb·
/// BITS_PER_FRAME`. The exact IDLE elapsed is `2·BITS_PER_FRAME·tpb`
/// (the byte's own 10 bits + 10 idle threshold bits), so subtracting
/// 10·tpb lands `ref ≈ stamp + 10·tpb = packet_end_tick` — half-way
/// through the wrap window above `stamp_full`. That gives ±10·tpb
/// (= ±50_000 ticks at 9600 / ±160 at 3M) of slack on either side
/// without crossing into the next wrap, so off-by-one truncation in
/// the time→tick conversion can't slip `ref` below `stamp_full` (which
/// would alias DOWN one wrap, the dual of the original bug).
///
/// `[[no_idle_timing]]` compliance: the returned reference is only used
/// to disambiguate which u16-wrap window the IC stamp belongs to; the
/// lifted stamp itself is hardware-precise. IDLE is signalling which
/// formula to apply (Dma vs Idle), not measuring the packet's wire end.
#[inline]
fn drain_ref(now: u32, src: PollSrc, ticks_per_bit: u16) -> u32 {
    match src {
        PollSrc::ByteBatch => now,
        PollSrc::LineIdle => {
            let half = (BITS_PER_FRAME as u32).wrapping_mul(ticks_per_bit as u32);
            now.wrapping_sub(half)
        }
    }
}

// The wire-walker below is a set of stateless free functions — every
// operation threads a borrowed [`AnchorCache`] for the tail-anchor state
// it reads and writes. The codec's [`EdgeCapture`] owns the cache and the
// rings and drives these.
//
// [`EdgeCapture`]: super::edge_capture::EdgeCapture

/// Single-window check of the tail signature: anchor the FIRST tail
/// byte's start edge at `head_offset` slots back from the ring head,
/// then walk forward through the per-byte falling-edge pattern checking
/// each inter-edge delta. On match: returns each tail byte's start
/// tick (oldest at `[0]`, LAST tail byte's start = CRC byte's start
/// at `[TAIL_STARTS - 1]`). On miss: returns `None` with no state
/// mutation.
///
/// Cross-byte deltas (last edge of byte N → first edge of byte N+1)
/// allow ±1·bit; intra-byte deltas allow ±0.5·bit — same convention
/// as a wide/narrow band split. `anchor.rx_edge_comp_ticks` is
/// subtracted from every stamp at read-from-ring time so the return
/// value is wire-edge time.
fn try_anchor_at_tail_offset<const EDGE_BUF_LEN: usize>(
    edges: &HwRing<u16, EDGE_BUF_LEN>,
    anchor: &AnchorCache,
    head_offset: u16,
    tail_bytes: &[u8],
    ticks_per_bit: u16,
) -> Option<[u16; TAIL_STARTS]> {
    let rx_edge_comp = anchor.rx_edge_comp_ticks;
    let mut sig_idx: u16 = 0;
    let mut prev_tick: u16 = 0;
    let mut prev_bit_pos: u16 = 0;
    let mut starts = [0u16; TAIL_STARTS];
    let mut matched = false;

    let n_bytes = tail_bytes.len();
    let mut bi = 0;
    while bi < n_bytes {
        let b = tail_bytes[bi];
        let (positions, count) = byte_edge_positions(b);
        let byte_start_bit = (bi as u16).wrapping_mul(BITS_PER_FRAME);

        let mut k = 0;
        while k < count {
            let bit_pos = byte_start_bit.wrapping_add(positions[k] as u16);

            // Map signature index → ring offset. sig_idx 0 sits at
            // `head_offset` (oldest in the signature); sig_idx grows
            // toward the ring head as we walk forward.
            let recent_off = head_offset.wrapping_sub(sig_idx);
            let &e_raw = edges.recent(recent_off)?;
            let e = e_raw.wrapping_sub(rx_edge_comp);

            if sig_idx > 0 {
                let actual = e.wrapping_sub(prev_tick);
                let expected_bits = bit_pos.wrapping_sub(prev_bit_pos);
                let cross_byte = k == 0;
                if !tail_delta_in_band(actual, expected_bits, ticks_per_bit, cross_byte) {
                    return None;
                }
            }

            if k == 0 && bi < TAIL_STARTS {
                starts[bi] = e;
                if bi == n_bytes - 1 {
                    matched = true;
                }
            }

            prev_tick = e;
            prev_bit_pos = bit_pos;
            sig_idx = sig_idx.wrapping_add(1);
            k += 1;
        }
        bi += 1;
    }

    matched.then_some(starts)
}

/// Back-search the ET ring for the just-received tail bytes' wire-edge
/// signature. Called by the codec at every parser `Event::Crc` with
/// the last few raw wire bytes (pre-unstuff) the parser consumed and
/// `d_min` — the summed [`EDGES_PER_BYTE`] of bytes drained-but-not-
/// yet-parsed past CRC in the byte ring at Crc emit. On match,
/// `anchor.tail_anchor` is set to the LAST tail byte's start tick —
/// what [`packet_end_tick`] reads to derive `packet_end_tick
/// = anchor + 10·bit` in wire-edge time. On miss, `anchor.tail_anchor`
/// is cleared and the composite's downstream fallback path runs.
///
/// Search window: `[d_min .. d_min + PADDING_EDGES_AHEAD]` slots back
/// from head, where `d_min` locates CRC's tail edge in ET (bytes past
/// CRC in the ring contribute edges between head and CRC's tail
/// edge). The signature spans `total_edges - 1` slots deeper still,
/// so the actual probe offset walked by [`Self::try_anchor_at_tail_offset`]
/// is `d_min + total_edges - 1 + slack`. [`EDGES_PER_BYTE`] gives the
/// *minimum* edge count — real wire adds edges past that floor via
/// in-flight partial bytes / EMI, absorbed by the `PADDING_EDGES_AHEAD`
/// asymmetric slack.
///
/// The 4-byte tail signature includes both CRC bytes, so per-shift
/// false-positive entropy is ~2^-16 — comfortably below the packet
/// rate.
pub(super) fn anchor_at_tail<const EDGE_BUF_LEN: usize>(
    edges: &mut HwRing<u16, EDGE_BUF_LEN>,
    anchor: &mut AnchorCache,
    ticks_per_bit: u16,
    tail_bytes: &[u8],
    d_min: u16,
) -> bool {
    match search_tail_signature(edges, anchor, ticks_per_bit, tail_bytes, d_min) {
        Some((o, starts)) => {
            anchor.tail_anchor = Some(starts[TAIL_STARTS - 1]);
            anchor.tail_starts = starts;
            // `o` IS the oldest cached start's ring offset — the FIRST
            // tail byte's start edge, by the sig_idx=0 invariant in
            // `try_anchor_at_tail_offset`.
            anchor.tail_starts_oldest_ring_off = Some(o);
            true
        }
        None => {
            anchor.tail_anchor = None;
            anchor.tail_starts_oldest_ring_off = None;
            false
        }
    }
}

/// Window-bounded search for `tail_bytes`' falling-edge signature —
/// the shared core of [`anchor_at_tail`] (Crc-time drift anchor, writes
/// [`AnchorCache`]) and [`reply_tail_newest_start`] (FAST status-start
/// query, read-only). Returns the matched probe offset (= the FIRST
/// tail byte's start-edge ring offset) and the per-byte start ticks
/// (oldest at `[0]`; entries past `tail_bytes.len()` stay zeroed for
/// short slices). Mutates nothing beyond the ring's lap resync.
fn search_tail_signature<const EDGE_BUF_LEN: usize>(
    edges: &mut HwRing<u16, EDGE_BUF_LEN>,
    anchor: &AnchorCache,
    ticks_per_bit: u16,
    tail_bytes: &[u8],
    d_min: u16,
) -> Option<(u16, [u16; TAIL_STARTS])> {
    edges.reader().resync_if_lapped();
    let avail = edges.recent_count();

    let mut total_edges: u16 = 0;
    for &b in tail_bytes {
        total_edges = total_edges.wrapping_add(EDGES_PER_BYTE[b as usize] as u16);
    }
    // First tail byte's start edge sits this many slots back from head
    // when the newest tail edge is at offset `d_min`.
    let base = d_min.saturating_add(total_edges.saturating_sub(1));
    if total_edges < 2 || avail <= base {
        crate::log::trace!(
            "edge_parser: tail-anchor insufficient avail={} base={} total_edges={} d_min={}",
            avail,
            base,
            total_edges,
            d_min,
        );
        return None;
    }

    let max_offset = avail - 1;
    let lo = base.min(max_offset);
    let hi = base.saturating_add(PADDING_EDGES_AHEAD).min(max_offset);

    for o in lo..=hi {
        if let Some(starts) = try_anchor_at_tail_offset(edges, anchor, o, tail_bytes, ticks_per_bit)
        {
            crate::log::debug!(
                "edge_parser: tail-anchor match offset={} base={} d_min={} total={}",
                o,
                base,
                d_min,
                total_edges,
            );
            return Some((o, starts));
        }
    }

    crate::log::debug!(
        "edge_parser: tail-anchor miss base={} d_min={} total={}",
        base,
        d_min,
        total_edges,
    );
    None
}

/// Wire-end tick of the just-completed packet, lifted into the WireClock
/// u32 domain. Composite stamps `packet_end_tick` at the parser's
/// CRC-good event — the CRC byte's start sits at the tail anchor, the
/// wire-end one byte-time later. `now` / `src` route through
/// [`drain_ref`] so the lift stays sub-wrap at low baud.
pub(super) fn packet_end_tick(
    anchor: &AnchorCache,
    ticks_per_bit: u16,
    now: u32,
    src: PollSrc,
) -> Option<u32> {
    let frame_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
    let r = drain_ref(now, src, ticks_per_bit);
    anchor
        .tail_anchor
        .map(|t| lift(t, r).wrapping_add(frame_ticks))
}

/// Fallback packet-end estimate for the no-anchor case — the composite
/// calls this at Crc when [`packet_end_tick`] returns `None` (interference
/// or edge loss starved the tail-anchor back-search). Formula per `src`:
///
/// - [`PollSrc::ByteBatch`]: returns `now`. CRC byte landed in DMA
///   ~immediately before the poll, so `now` IS packet-end with negligible
///   ISR-entry offset.
/// - [`PollSrc::LineIdle`]: returns `now − BITS_PER_FRAME · ticks_per_bit`.
///   USART1 IDLE asserts one idle character after the last data byte's
///   stop bit, so back-date by that interval.
///
/// Bumps no state; the composite increments the telemetry counter via
/// [`crate::traits::dxl::Telemetry::record_edge_anchor_miss`] alongside.
///
/// The formulas coincide with [`drain_ref`] by construction — the
/// corrected lift reference IS the fallback estimate (both back-date
/// LineIdle by exactly one frame) — so this delegates rather than
/// duplicating the match.
pub(super) fn packet_end_tick_fallback(src: PollSrc, now: u32, ticks_per_bit: u16) -> u32 {
    drain_ref(now, src, ticks_per_bit)
}

/// Max ticks between the first Status byte's wire end (RXNE assertion at
/// its stop bit) and the `now` the status-start query receives — PFIC
/// trap-entry latency plus the RXNE-set-vs-stop-bit sampling skew.
/// 240 HCLK ticks = 5 µs at 48 MHz, generous against the measured ~1-2 µs
/// entry latencies elsewhere (`FAST_LAST_ENTRY_TICKS` class). Bounds the
/// acceptance window that separates the first Status byte's start edge
/// from an in-flight second byte's edge (too young) and stale
/// instruction-tail edges (too old).
const STATUS_START_ENTRY_LAG_TICKS: u32 = 240;

/// Start tick of the awaited Status packet's FIRST byte, resolved from
/// the newest ET-ring stamps at the byte's RXNE trap. The DXL 2.0 header
/// starts `0xFF` — exactly one falling edge (the start bit) — so the
/// byte's stamp sits one frame (+ trap lag) before `now`:
///
/// - If the second-newest stamp lands in that window AND the newest sits
///   one frame after it, the newest is the in-flight second header
///   byte's start edge (also `0xFF`, captured before the trap entered) —
///   take the second-newest.
/// - Else the newest stamp in the window is the byte itself.
/// - Nothing in the window → `None` (edge lost to EMI; the caller
///   retries on the next byte's trap via the multi-byte signature path).
///
/// `now` must be a wire-clock reading taken at the trap entry — the
/// window math assumes the stamp is at most one frame plus
/// [`STATUS_START_ENTRY_LAG_TICKS`] old, which also keeps the u16 lift
/// sub-wrap at every supported baud (10 bits at 9600 = 50 000 ticks).
/// Residual alias risk: an edge more than one u16 wrap old can lift into
/// the window only if the true first-byte edge was ALSO lost — a dual
/// failure below the EMI-degraded baseline, accepted.
pub(super) fn first_status_byte_start<const EDGE_BUF_LEN: usize>(
    edges: &mut HwRing<u16, EDGE_BUF_LEN>,
    anchor: &AnchorCache,
    ticks_per_bit: u16,
    now: u32,
) -> Option<u32> {
    edges.reader().resync_if_lapped();
    let frame_ticks = (ticks_per_bit as u32).wrapping_mul(BITS_PER_FRAME as u32);
    let window_lo = frame_ticks.wrapping_sub(ticks_per_bit as u32);
    let window_hi = frame_ticks.wrapping_add(STATUS_START_ENTRY_LAG_TICKS);
    let comp = anchor.rx_edge_comp_ticks;

    let lifted_age = |offset: u16| -> Option<(u32, u32)> {
        let &raw = edges.recent(offset)?;
        let lifted = lift(raw.wrapping_sub(comp), now);
        Some((lifted, now.wrapping_sub(lifted)))
    };

    let (l0, age0) = lifted_age(0)?;
    if let Some((l1, age1)) = lifted_age(1) {
        let gap = age1.wrapping_sub(age0);
        let paired = gap.abs_diff(frame_ticks) <= ticks_per_bit as u32;
        if paired && (window_lo..=window_hi).contains(&age1) {
            return Some(l1);
        }
    }
    (window_lo..=window_hi).contains(&age0).then_some(l0)
}

/// Start tick of the NEWEST of `tail_bytes` — the multi-byte arm of the
/// FAST status-start query, for traps that observe the Status packet
/// `n ≥ 2` bytes in (late watch enable, missed first-byte edge). Runs
/// the same window-bounded signature search as the Crc-time anchor with
/// `d_min = 0` (the bytes ARE the ring's newest), reads nothing from and
/// writes nothing to the [`AnchorCache`] beyond its edge compensation,
/// and lifts against the caller's fresh `now` — the newest byte's start
/// is ~one frame old, sub-wrap at every baud; the caller extrapolates
/// back to the packet's first byte in u32 ticks.
pub(super) fn reply_tail_newest_start<const EDGE_BUF_LEN: usize>(
    edges: &mut HwRing<u16, EDGE_BUF_LEN>,
    anchor: &AnchorCache,
    ticks_per_bit: u16,
    tail_bytes: &[u8],
    now: u32,
) -> Option<u32> {
    let (_, starts) = search_tail_signature(edges, anchor, ticks_per_bit, tail_bytes, 0)?;
    let newest = starts[tail_bytes.len() - 1];
    Some(lift(newest, now))
}

/// Retroactive integrator walk. Emits pairs going backward from
/// `tail_anchor`: first the three cached `(tail_starts[k],
/// tail_starts[k+1])` pairs from the signature validation (no ring
/// scan needed), then — for any remaining `n_pairs` — a proper ring
/// walk seeded at `tail_starts[0]`. Each ring step predicts the
/// next byte's start at `anchor − BITS_PER_FRAME·tpb` and snaps to
/// the nearest captured IC edge within `±HSI_WALK_SNAP_BITS·tpb`;
/// on no-match the walk free-runs to the prediction without
/// emitting a pair (a zero-drift synthetic pair would bias the
/// integrator toward the free-run rate).
///
/// Runs after `arm_tim2` at the parser's Crc event so the deadline
/// path pays only the tail-anchor cost, not the walk cost. `n_pairs`
/// comes from [`crate::dxl::uart::clock::Clock::samples_wanted_per_packet`];
/// `n_pairs == 0` is a no-op. `rx_edge_comp_ticks` is subtracted from
/// every stamp at read-from-ring time so emitted pairs are in wire-
/// edge time.
pub(super) fn walk_pairs_back<const EDGE_BUF_LEN: usize, const PAIRS_LEN: usize>(
    edges: &HwRing<u16, EDGE_BUF_LEN>,
    anchor: &AnchorCache,
    n_pairs: u8,
    ticks_per_bit: u16,
    out_pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
) {
    if n_pairs == 0 {
        return;
    }

    // Emit cached pairs first: the anchor's signature walk already
    // visited each byte's start edge and validated it against a
    // ±1·bit cross-byte band — tighter than the walker's ±2·bit
    // snap. So these pairs are strictly higher quality than a
    // ring-walk snap; use them whenever we've got them.
    let mut emitted: u8 = 0;
    if anchor.tail_anchor.is_some() {
        let cache_pairs = (TAIL_STARTS as u8) - 1;
        let cache_take = n_pairs.min(cache_pairs);
        let mut i = 0;
        while i < cache_take {
            let idx = i as usize;
            let _ = out_pairs.push((anchor.tail_starts[idx], anchor.tail_starts[idx + 1]));
            i += 1;
        }
        emitted = cache_take;
    }
    if emitted >= n_pairs {
        return;
    }

    // i32 keeps the signed distance metric honest at 9600 baud —
    // `byte_ticks = 50_000` overflows i16's positive range, so an
    // `(e - predicted) as i16` metric aliases on the very first scan
    // iter and every step free-runs. Consecutive edges within a
    // packet are strictly < 65_536 ticks apart at every supported
    // baud (max = 50_000 = one byte-time at 9600), so summing
    // per-hop u16 deltas into an i32 accumulator relative to the
    // walk seed gives an unaliased signed offset. No absolute-tick
    // lift needed.
    let byte_ticks = (ticks_per_bit as i32).wrapping_mul(BITS_PER_FRAME as i32);
    let snap = (ticks_per_bit as i32).wrapping_mul(HSI_WALK_SNAP_BITS as i32);
    let rx_edge_comp = anchor.rx_edge_comp_ticks;
    let avail = edges.recent_count();

    // Ring walk seeds at `tail_starts[0]` (the OLDEST cached start).
    // Its ring offset is recorded by `anchor_at_tail` on match;
    // without a match there's nothing to seed and we early-return.
    let Some(probe_start) = anchor.tail_starts_oldest_ring_off else {
        return;
    };
    if probe_start >= avail {
        return;
    }
    let seed_stamp = anchor.tail_starts[0];

    // `curr` is the wire-clock u16 stamp emitted as the newer half of
    // the next pair. `prev` is the last edge examined (or the seed if
    // no edge examined yet). `older_from_seed` is the i32 wire-clock
    // distance from `seed_stamp` (the walk start) to `prev`,
    // accumulated from per-hop deltas — persists across free-runs
    // so scans resume from the right offset in signed space.
    // `probe_offset` advances strictly older with each match
    // (free-runs don't consume ring edges).
    let mut curr = seed_stamp;
    let mut prev = seed_stamp;
    let mut older_from_seed: i32 = 0;
    let mut probe_offset: u16 = probe_start;

    let remaining = n_pairs - emitted;
    for k in 1..=remaining as u32 {
        let target_older = (k as i32).wrapping_mul(byte_ticks);

        // Scan older-ward from probe_offset for the edge closest to
        // `target_older` back from tail_anchor. Track a per-scan
        // (older, prev) copy so a mid-scan `break` doesn't leak
        // partial-accumulator state into the free-run case.
        let mut best: Option<(u16, u16, i32, i32)> = None; // (off, stamp, signed_delta, older_at_match)
        let mut scan_older = older_from_seed;
        let mut scan_prev = prev;
        let mut off = probe_offset;
        while off < avail {
            let Some(&e_raw) = edges.recent(off) else {
                break;
            };
            let e = e_raw.wrapping_sub(rx_edge_comp);
            let step = scan_prev.wrapping_sub(e) as i32;
            scan_older = scan_older.saturating_add(step);
            let signed_delta = target_older.saturating_sub(scan_older);
            let abs = signed_delta.unsigned_abs();

            if abs <= snap as u32 {
                let take = match best {
                    None => true,
                    Some((_, _, prev_sd, _)) => {
                        let prev_abs = prev_sd.unsigned_abs();
                        // Closest-to-prediction; tiebreak on
                        // at-or-before predicted (real start edge
                        // shifts NEXT byte start LATER on a chip-
                        // driven inter-byte gap, so `at-or-before`
                        // is the physical direction).
                        abs < prev_abs || (abs == prev_abs && signed_delta <= 0)
                    }
                };
                if take {
                    best = Some((off, e, signed_delta, scan_older));
                }
            }

            if signed_delta < -snap {
                break;
            }
            scan_prev = e;
            off = off.wrapping_add(1);
        }

        match best {
            Some((off, match_stamp, _, older_at_match)) => {
                let _ = out_pairs.push((match_stamp, curr));
                curr = match_stamp;
                prev = match_stamp;
                older_from_seed = older_at_match;
                probe_offset = off.wrapping_add(1);
            }
            None => {
                // Free-run: virtual position moves back by
                // `byte_ticks`, but no ring edge was consumed —
                // leave `prev`, `older_from_seed`, and
                // `probe_offset` untouched so the next iter's scan
                // resumes from the same accumulator state.
                curr = curr.wrapping_sub(byte_ticks as u16);
            }
        }
    }
}

/// Stage a matching falling-edge signature for `tail_bytes` into `edges`
/// so a follow-up [`anchor_at_tail`] resolves the tail anchor to
/// `anchor_tick`. Composite-test scaffolding — real code populates the
/// ET ring from the DMA1_CH7 IC-capture. Writes stamps oldest-first
/// starting at ring index 0; caller advances `write_seq` via
/// `on_publish(EDGE_BUF_LEN - total_edges)`. Assumes
/// `rx_edge_comp_ticks == 0` on the cache (test-baud default). Returns
/// the number of stamps written.
#[cfg(test)]
pub(super) fn stage_tail_signature_for_test<const EDGE_BUF_LEN: usize>(
    edges: &mut HwRing<u16, EDGE_BUF_LEN>,
    tail_bytes: &[u8],
    ticks_per_bit: u16,
    anchor_tick: u16,
) -> u16 {
    // Max footprint: TAIL_STARTS bytes × MAX_EDGES_PER_BYTE = 4 × 5 = 20.
    const MAX_TAIL_EDGES: usize = TAIL_STARTS * MAX_EDGES_PER_BYTE as usize;
    let mut stamps = [0u16; MAX_TAIL_EDGES];
    let mut n: u16 = 0;
    // Last tail byte's start edge sits at `anchor_tick`; earlier edges
    // step back by `(last_start_bit - bit_pos) * tpb`.
    let last_start_bit = ((tail_bytes.len().saturating_sub(1)) as u16).wrapping_mul(BITS_PER_FRAME);
    for (bi, &b) in tail_bytes.iter().enumerate() {
        let (positions, count) = byte_edge_positions(b);
        let byte_start_bit = (bi as u16).wrapping_mul(BITS_PER_FRAME);
        for &pos in positions.iter().take(count) {
            let bit_pos = byte_start_bit.wrapping_add(pos as u16);
            let delta_bits = last_start_bit.wrapping_sub(bit_pos);
            stamps[n as usize] = anchor_tick.wrapping_sub(delta_bits.wrapping_mul(ticks_per_bit));
            n = n.wrapping_add(1);
        }
    }
    edges.stage(0, &stamps[..n as usize]);
    n
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ring::HwRing;

    // 3 Mbaud at HCLK 48 MHz → ticks_per_bit = 16. One byte = 160 ticks.
    const TPB_3M: u16 = 16;
    const BYTE_TICKS_3M: u16 = 160;
    // 1 Mbaud at HCLK 48 MHz → ticks_per_bit = 48. One byte = 480 ticks.
    const TPB_1M: u16 = 48;
    // 9600 baud at HCLK 48 MHz → ticks_per_bit = 5000. One byte = 50000
    // ticks — inside u16 range, but the walker's signed-distance metric
    // aliases at i16 (50000 > i16::MAX). Regression fixture.
    const TPB_9600: u16 = 5000;
    const BYTE_TICKS_9600: u16 = 50000;

    /// Bundles the free-function walker calls with the [`AnchorCache`] they
    /// thread so test bodies read as `f.op(...)`. The `seed_*` helpers
    /// build cache state directly, replacing the deleted production
    /// force-shims.
    struct Fixture {
        anchor: AnchorCache,
    }

    impl Fixture {
        fn new() -> Self {
            Self {
                anchor: AnchorCache::new(),
            }
        }

        fn anchor_at_tail<const EDGE_BUF_LEN: usize>(
            &mut self,
            edges: &mut HwRing<u16, EDGE_BUF_LEN>,
            ticks_per_bit: u16,
            tail_bytes: &[u8],
            d_min: u16,
        ) -> bool {
            anchor_at_tail(edges, &mut self.anchor, ticks_per_bit, tail_bytes, d_min)
        }

        fn walk_pairs_back<const EDGE_BUF_LEN: usize, const PAIRS_LEN: usize>(
            &self,
            edges: &HwRing<u16, EDGE_BUF_LEN>,
            n_pairs: u8,
            ticks_per_bit: u16,
            out_pairs: &mut heapless::Vec<(u16, u16), PAIRS_LEN>,
        ) {
            walk_pairs_back(edges, &self.anchor, n_pairs, ticks_per_bit, out_pairs);
        }

        fn packet_end_tick(&self, ticks_per_bit: u16, now: u32, src: PollSrc) -> Option<u32> {
            packet_end_tick(&self.anchor, ticks_per_bit, now, src)
        }

        fn packet_end_tick_fallback(&self, src: PollSrc, now: u32, ticks_per_bit: u16) -> u32 {
            packet_end_tick_fallback(src, now, ticks_per_bit)
        }

        /// Seed the ring-walk start without the cache-emit path
        /// (`tail_anchor` left `None`) so tests exercise the ring-walk
        /// accumulator in isolation.
        fn seed_walker(&mut self, seed_stamp: u16, oldest_off: u16) {
            self.anchor.tail_starts[0] = seed_stamp;
            self.anchor.tail_starts_oldest_ring_off = Some(oldest_off);
        }

        /// Seed the cache-emit path AND ring seed for the "3 pairs from
        /// cache + walker for the rest" flow. `tail_anchor` = `starts[3]`.
        fn seed_cache(&mut self, starts: [u16; TAIL_STARTS], oldest_off: u16) {
            self.anchor.tail_starts = starts;
            self.anchor.tail_starts_oldest_ring_off = Some(oldest_off);
            self.anchor.tail_anchor = Some(starts[TAIL_STARTS - 1]);
        }
    }

    fn make() -> Fixture {
        Fixture::new()
    }

    /// Stage edges into a 16-slot HwRing and publish the producer head
    /// at `vals.len()`. Returns the populated ring.
    fn edges16(vals: &[u16]) -> HwRing<u16, 16> {
        let mut b: HwRing<u16, 16> = HwRing::new(0);
        b.stage(0, vals);
        b.on_publish(HwRing::<u16, 16>::LEN - vals.len() as u16);
        b
    }

    // ---------- lift / drain_ref ----------

    #[test]
    fn packet_end_tick_adds_10bit_after_lift() {
        let mut c = make();
        assert_eq!(
            c.packet_end_tick(TPB_3M, 1_000_000, PollSrc::ByteBatch),
            None
        );
        c.anchor.tail_anchor = Some(5000);
        let now = 0x0001_0000_u32 + 5000;
        assert_eq!(
            c.packet_end_tick(TPB_3M, now, PollSrc::ByteBatch),
            Some(now + BYTE_TICKS_3M as u32),
        );
    }

    #[test]
    fn packet_end_tick_idle_drain_stays_sub_wrap_at_9600() {
        // Regression: at 9600 baud (tpb = 5000 ticks @ 48 MHz HCLK) the
        // IDLE elapsed = 2·10·5000 = 100_000 ticks > 65_536 (u16 wrap).
        // Without the IDLE-aware reference, `lift` aliases one wrap too
        // high — that's the broadcast-Ping slot drift `timing/
        // ping_broadcast.rs` flagged at baud_idx 0. The corrected `ref`
        // lands at `stamp + 50_000`, sub-wrap, lifts cleanly.
        const TPB_9600: u16 = 5000;
        const BYTE_TICKS_9600: u32 = 10 * TPB_9600 as u32;
        let mut c = make();
        c.anchor.tail_anchor = Some(5000);
        let stamp_full = 0x0001_0000_u32 + 5000;
        let idle_elapsed = 2 * BITS_PER_FRAME as u32 * TPB_9600 as u32;
        let now_at_idle = stamp_full + idle_elapsed;
        assert!(
            now_at_idle - stamp_full > u16::MAX as u32,
            "guards regression scope"
        );
        assert_eq!(
            c.packet_end_tick(TPB_9600, now_at_idle, PollSrc::LineIdle),
            Some(stamp_full + BYTE_TICKS_9600),
        );
    }

    #[test]
    fn packet_end_tick_idle_robust_to_truncated_elapsed() {
        // Truncation regression: integer division in sim's ns→tick
        // conversion (and HCLK/baud rounding for non-integer tpb) can
        // shave one tick off the elapsed.
        const TPB_9600: u16 = 5000;
        const BYTE_TICKS_9600: u32 = 10 * TPB_9600 as u32;
        let mut c = make();
        c.anchor.tail_anchor = Some(5000);
        let stamp_full = 0x0001_0000_u32 + 5000;
        let truncated_elapsed = 2 * BITS_PER_FRAME as u32 * TPB_9600 as u32 - 1;
        let now_at_idle = stamp_full + truncated_elapsed;
        assert_eq!(
            c.packet_end_tick(TPB_9600, now_at_idle, PollSrc::LineIdle),
            Some(stamp_full + BYTE_TICKS_9600),
        );
    }

    #[test]
    fn fallback_dma_returns_now_unchanged() {
        let c = make();
        assert_eq!(
            c.packet_end_tick_fallback(PollSrc::ByteBatch, 1234, TPB_3M),
            1234
        );
        assert_eq!(c.packet_end_tick_fallback(PollSrc::ByteBatch, 0, TPB_3M), 0);
        assert_eq!(
            c.packet_end_tick_fallback(PollSrc::ByteBatch, u32::MAX, TPB_3M),
            u32::MAX,
        );
    }

    #[test]
    fn fallback_idle_back_dates_by_one_idle_char() {
        let c = make();
        assert_eq!(
            c.packet_end_tick_fallback(PollSrc::LineIdle, 1600, TPB_3M),
            1600 - BYTE_TICKS_3M as u32,
        );
    }

    #[test]
    fn fallback_idle_wraps_at_u32_boundary() {
        let c = make();
        let expected = 5_u32.wrapping_sub(BYTE_TICKS_3M as u32);
        assert_eq!(
            c.packet_end_tick_fallback(PollSrc::LineIdle, 5, TPB_3M),
            expected,
        );
    }

    #[test]
    fn reset_anchor_clears_state() {
        let mut c = make();
        c.anchor.tail_anchor = Some(1234);
        c.anchor.reset();
        assert_eq!(c.anchor.tail_anchor, None);
    }

    // ---------- FAST status-start query ----------

    #[test]
    fn status_start_single_edge_in_window_resolves() {
        let c = make();
        let mut edges = edges16(&[1000]);
        // RXNE trap enters one frame (+ some lag) after the start edge.
        let now = 1000 + BYTE_TICKS_3M as u32 + 50;
        assert_eq!(
            first_status_byte_start(&mut edges, &c.anchor, TPB_3M, now),
            Some(1000)
        );
    }

    #[test]
    fn status_start_prefers_paired_older_edge() {
        // The second header byte (also 0xFF) is in flight at trap entry;
        // its start edge is already captured one frame after the first
        // byte's. The query must take the OLDER of the frame-spaced pair.
        let c = make();
        let mut edges = edges16(&[1000, 1000 + BYTE_TICKS_3M]);
        let now = (1000 + BYTE_TICKS_3M) as u32 + BYTE_TICKS_3M as u32 + 30;
        assert_eq!(
            first_status_byte_start(&mut edges, &c.anchor, TPB_3M, now),
            Some(1000)
        );
    }

    #[test]
    fn status_start_ignores_unpaired_older_edge() {
        // An older edge NOT one frame away (instruction-tail residue)
        // must not steal the anchor from the in-window newest edge.
        let c = make();
        let mut edges = edges16(&[200, 1000]);
        let now = 1000 + BYTE_TICKS_3M as u32 + 50;
        assert_eq!(
            first_status_byte_start(&mut edges, &c.anchor, TPB_3M, now),
            Some(1000)
        );
    }

    #[test]
    fn status_start_stale_edge_is_none() {
        // Newest edge far older than one frame + trap lag: the awaited
        // byte's own edge was lost — refuse rather than mis-anchor.
        let c = make();
        let mut edges = edges16(&[1000]);
        let now = 1000 + 4 * BYTE_TICKS_3M as u32;
        assert_eq!(
            first_status_byte_start(&mut edges, &c.anchor, TPB_3M, now),
            None
        );
    }

    #[test]
    fn status_start_empty_ring_is_none() {
        let c = make();
        let mut edges = edges16(&[]);
        assert_eq!(
            first_status_byte_start(&mut edges, &c.anchor, TPB_3M, 1210),
            None
        );
    }

    #[test]
    fn status_start_sub_wrap_at_9600() {
        // One frame at 9600 = 50_000 ticks — near the u16 wrap. The lift
        // must land the stamp in the right wrap window off a trap-entry
        // `now` one frame + lag out.
        let c = make();
        let mut edges = edges16(&[1000]);
        let now = 0x0002_0000_u32 + 1000 + 10 * TPB_9600 as u32 + 100;
        assert_eq!(
            first_status_byte_start(&mut edges, &c.anchor, TPB_9600, now),
            Some(0x0002_0000 + 1000)
        );
    }

    #[test]
    fn reply_tail_newest_start_lifts_matched_signature() {
        let c = make();
        let tail = [0xFF, 0xFF, 0xFD, 0x00];
        let mut edges: HwRing<u16, 16> = HwRing::new(0);
        let n = stage_tail_signature_for_test(&mut edges, &tail, TPB_3M, 2000);
        edges.on_publish(HwRing::<u16, 16>::LEN - n);
        let now = 2000 + 300;
        assert_eq!(
            reply_tail_newest_start(&mut edges, &c.anchor, TPB_3M, &tail, now),
            Some(2000)
        );
    }

    #[test]
    fn reply_tail_newest_start_leaves_anchor_untouched() {
        // Read-only contract: the FAST query must never plant a tail
        // anchor the Crc-time drift path could mistake for its own.
        let mut c = make();
        c.anchor.tail_anchor = Some(777);
        let tail = [0xFF, 0xFF, 0xFD, 0x00];
        let mut edges: HwRing<u16, 16> = HwRing::new(0);
        let n = stage_tail_signature_for_test(&mut edges, &tail, TPB_3M, 2000);
        edges.on_publish(HwRing::<u16, 16>::LEN - n);
        reply_tail_newest_start(&mut edges, &c.anchor, TPB_3M, &tail, 2300);
        assert_eq!(c.anchor.tail_anchor, Some(777));
    }

    #[test]
    fn reply_tail_newest_start_two_byte_tail_resolves() {
        // FAST middle segments can be 3 wire bytes; a late watch enable
        // may have only 2 bytes to sign with.
        let c = make();
        let tail = [0x55, 0x00];
        let mut edges: HwRing<u16, 16> = HwRing::new(0);
        let n = stage_tail_signature_for_test(&mut edges, &tail, TPB_3M, 3000);
        edges.on_publish(HwRing::<u16, 16>::LEN - n);
        assert_eq!(
            reply_tail_newest_start(&mut edges, &c.anchor, TPB_3M, &tail, 3200),
            Some(3000)
        );
    }

    #[test]
    fn reply_tail_newest_start_without_enough_edges_is_none() {
        // Queried bytes need 10 edges but the ring only captured 5 —
        // the search must refuse, not fabricate. (Band-mismatch misses
        // are pinned by the anchor_at_tail tests; the search core is
        // shared.)
        let c = make();
        let staged = [0xFF, 0xFF, 0xFD, 0x00];
        let mut edges: HwRing<u16, 16> = HwRing::new(0);
        let n = stage_tail_signature_for_test(&mut edges, &staged, TPB_3M, 2000);
        edges.on_publish(HwRing::<u16, 16>::LEN - n);
        assert_eq!(
            reply_tail_newest_start(&mut edges, &c.anchor, TPB_3M, &[0x55, 0x55], 2300),
            None
        );
    }

    // ---------- tail-signature back-search ----------

    /// Synthesize the wire-edge timestamps for a back-to-back sequence of
    /// `tail_bytes` starting at base tick `t0`, bit-time `tpb`. Mirrors
    /// the runtime's [`byte_edge_positions`] table walk so the unit tests
    /// exercise the same per-byte arithmetic the production back-search
    /// validates against.
    fn tail_edges(tail_bytes: &[u8], t0: u16, tpb: u16) -> heapless::Vec<u16, 20> {
        let mut out: heapless::Vec<u16, 20> = heapless::Vec::new();
        for (bi, &b) in tail_bytes.iter().enumerate() {
            let byte_start_bit = (bi as u16).wrapping_mul(BITS_PER_FRAME);
            let (positions, count) = byte_edge_positions(b);
            for &pos in positions.iter().take(count) {
                let abs_bit = byte_start_bit.wrapping_add(pos as u16);
                let t = t0.wrapping_add(abs_bit.wrapping_mul(tpb));
                let _ = out.push(t);
            }
        }
        out
    }

    #[test]
    fn byte_edge_positions_ff_is_start_only() {
        let (p, n) = byte_edge_positions(0xFF);
        assert_eq!(n, 1);
        assert_eq!(p[0], 0);
    }

    #[test]
    fn byte_edge_positions_aa_walks_alternating_pairs() {
        // 0xAA LSB-first: d0..d7 = 0,1,0,1,0,1,0,1. Falling edges at start
        // + each 1→0 transition (stream positions 3, 5, 7).
        let (p, n) = byte_edge_positions(0xAA);
        assert_eq!(n, 4);
        assert_eq!(&p[..n], &[0, 3, 5, 7]);
    }

    #[test]
    fn byte_edge_positions_zero_is_start_only() {
        let (p, n) = byte_edge_positions(0x00);
        assert_eq!(n, 1);
        assert_eq!(p[0], 0);
    }

    #[test]
    fn tail_anchor_matches_4_byte_ff_tail_at_3m() {
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let edges_vec = tail_edges(&tail, 1000, TPB_3M);
        let mut edges = edges16(&edges_vec);
        assert!(c.anchor_at_tail(&mut edges, TPB_3M, &tail, 0));
        assert_eq!(
            c.anchor.tail_anchor,
            Some(1000u16.wrapping_add(30 * TPB_3M)),
        );
    }

    #[test]
    fn tail_anchor_matches_intra_byte_rich_tail() {
        let mut c = make();
        let tail = [0xAAu8, 0xFF, 0xFF, 0xFF];
        let edges_vec = tail_edges(&tail, 5000, TPB_3M);
        let mut edges = edges16(&edges_vec);
        assert!(c.anchor_at_tail(&mut edges, TPB_3M, &tail, 0));
        assert_eq!(
            c.anchor.tail_anchor,
            Some(5000u16.wrapping_add(30 * TPB_3M)),
        );
    }

    #[test]
    fn tail_anchor_matches_4_byte_ff_tail_at_1m() {
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let edges_vec = tail_edges(&tail, 2000, TPB_1M);
        let mut edges = edges16(&edges_vec);
        assert!(c.anchor_at_tail(&mut edges, TPB_1M, &tail, 0));
        assert_eq!(
            c.anchor.tail_anchor,
            Some(2000u16.wrapping_add(30 * TPB_1M)),
        );
    }

    #[test]
    fn tail_anchor_rejects_too_few_edges() {
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let mut edges = edges16(&[1000, 1160, 1320]);
        c.anchor.tail_anchor = Some(7777);
        assert!(!c.anchor_at_tail(&mut edges, TPB_3M, &tail, 0));
        assert_eq!(c.anchor.tail_anchor, None);
    }

    #[test]
    fn tail_anchor_rejects_wrong_byte_content() {
        let mut c = make();
        let real = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let claimed = [0xAAu8, 0xFF, 0xFF, 0xFF];
        let edges_vec = tail_edges(&real, 1000, TPB_3M);
        let mut edges = edges16(&edges_vec);
        c.anchor.tail_anchor = Some(7777);
        assert!(!c.anchor_at_tail(&mut edges, TPB_3M, &claimed, 0));
        assert_eq!(c.anchor.tail_anchor, None);
    }

    #[test]
    fn tail_anchor_rejects_perturbed_intra_byte_delta() {
        let mut c = make();
        let tail = [0xAAu8, 0xFF, 0xFF, 0xFF];
        let mut edges_vec = tail_edges(&tail, 1000, TPB_3M);
        // Shift the second intra-byte edge of 0xAA by +2·bit — outside
        // the ±0.5·bit intra-byte tolerance.
        edges_vec[1] = edges_vec[1].wrapping_add(2u16.wrapping_mul(TPB_3M));
        let mut edges = edges16(&edges_vec);
        assert!(!c.anchor_at_tail(&mut edges, TPB_3M, &tail, 0));
        assert_eq!(c.anchor.tail_anchor, None);
    }

    #[test]
    fn tail_anchor_padding_window_absorbs_one_extra_trailing_edge() {
        // Realistic tail (contains a non-0xFF byte with intra-byte edges)
        // so the first probe rejects the spurious newer window on band-
        // check mismatch before landing at the true offset.
        let mut c = make();
        let tail = [0xAAu8, 0xFF, 0xFF, 0xFF];
        let mut edges_vec = tail_edges(&tail, 1000, TPB_3M);
        // Simulate one in-flight edge past the CRC byte — the next byte's
        // start bit has landed but the byte itself hasn't been DMA'd, so
        // d_min stays 0 (D_min counts complete-byte edges only). Search
        // slack absorbs the extra edge.
        edges_vec
            .push(edges_vec[edges_vec.len() - 1].wrapping_add(BYTE_TICKS_3M))
            .unwrap();
        let mut edges = edges16(&edges_vec);
        assert!(c.anchor_at_tail(&mut edges, TPB_3M, &tail, 0));
        assert_eq!(
            c.anchor.tail_anchor,
            Some(1000u16.wrapping_add(30 * TPB_3M))
        );
    }

    #[test]
    fn tail_anchor_d_min_shift_matches_past_batch_bytes() {
        // Simulate a batch where two complete bytes (0xFF each — 1 edge
        // each) landed past CRC in the RX ring at Crc emit. d_min = 2
        // shifts the search deeper to skip those trailing edges.
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let mut edges_vec = tail_edges(&tail, 1000, TPB_3M);
        let crc_tail_edge = edges_vec[edges_vec.len() - 1];
        // Two extra bytes' start-bit edges (0xFF contributes 1 edge/byte)
        // one and two byte-times past the CRC byte's start edge.
        edges_vec
            .push(crc_tail_edge.wrapping_add(BYTE_TICKS_3M))
            .unwrap();
        edges_vec
            .push(crc_tail_edge.wrapping_add(2u16.wrapping_mul(BYTE_TICKS_3M)))
            .unwrap();
        let mut edges = edges16(&edges_vec);
        // Without d_min shift the naive probe lands on the trailing
        // edges; the search-slack window (5 edges) still swallows it
        // here, but the doc-contract is that d_min carries the exact
        // shift. Verify a d_min=2 call matches at the intended offset.
        assert!(c.anchor_at_tail(&mut edges, TPB_3M, &tail, 2));
        assert_eq!(
            c.anchor.tail_anchor,
            Some(1000u16.wrapping_add(30 * TPB_3M))
        );
    }

    // ---------- retroactive walk ----------

    #[test]
    fn walk_pairs_back_zero_is_noop() {
        let mut c = make();
        c.seed_walker(5000, 0);
        let edges = edges16(&[]);
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.walk_pairs_back(&edges, 0, TPB_3M, &mut pairs);
        assert!(pairs.is_empty());
    }

    #[test]
    fn walk_pairs_back_emits_backwards_pairs_on_clean_tail() {
        // 4-byte 0xFF tail — one edge per byte, spaced BYTE_TICKS_3M apart.
        // Walk from the CRC byte's start (t0+30·bit) back through the 3
        // preceding byte starts. Ring-walk-only mode (no cache).
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let edges_vec = tail_edges(&tail, 1000, TPB_3M);
        let edges = edges16(&edges_vec);
        let tail_anchor = 1000u16.wrapping_add(30 * TPB_3M);
        c.seed_walker(tail_anchor, 0);
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.walk_pairs_back(&edges, 3, TPB_3M, &mut pairs);
        assert_eq!(
            pairs.as_slice(),
            &[
                (
                    1000u16.wrapping_add(20 * TPB_3M),
                    1000u16.wrapping_add(30 * TPB_3M)
                ),
                (
                    1000u16.wrapping_add(10 * TPB_3M),
                    1000u16.wrapping_add(20 * TPB_3M)
                ),
                (1000u16, 1000u16.wrapping_add(10 * TPB_3M)),
            ]
        );
    }

    #[test]
    fn walk_pairs_back_free_runs_when_edge_missing() {
        // Only 2 tail edges present but ask for 3 pairs. First 2 hit;
        // third step's predicted position sits before the ring's oldest
        // edge, so the walk free-runs without emitting a pair.
        let mut c = make();
        let edges = edges16(&[
            1000u16.wrapping_add(20 * TPB_3M),
            1000u16.wrapping_add(30 * TPB_3M),
        ]);
        let tail_anchor = 1000u16.wrapping_add(30 * TPB_3M);
        c.seed_walker(tail_anchor, 0);
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.walk_pairs_back(&edges, 3, TPB_3M, &mut pairs);
        assert_eq!(
            pairs.as_slice(),
            &[(
                1000u16.wrapping_add(20 * TPB_3M),
                1000u16.wrapping_add(30 * TPB_3M)
            )],
        );
    }

    #[test]
    fn walk_pairs_back_snaps_within_window() {
        // Perturb each byte start by +1·bit (well inside the 2·bit snap
        // window). Walk still matches and emits pairs at the perturbed
        // stamps.
        let mut c = make();
        let e0 = 1000u16.wrapping_add(TPB_3M);
        let e1 = 1000u16.wrapping_add(10 * TPB_3M).wrapping_add(TPB_3M);
        let e2 = 1000u16.wrapping_add(20 * TPB_3M).wrapping_add(TPB_3M);
        let e3 = 1000u16.wrapping_add(30 * TPB_3M).wrapping_add(TPB_3M);
        let edges = edges16(&[e0, e1, e2, e3]);
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.seed_walker(e3, 0);
        c.walk_pairs_back(&edges, 3, TPB_3M, &mut pairs);
        assert_eq!(pairs.as_slice(), &[(e2, e3), (e1, e2), (e0, e1)]);
    }

    #[test]
    fn walk_pairs_back_emits_backwards_pairs_at_9600() {
        // Regression: at 9600 baud byte_ticks = 50000 > i16::MAX. The old
        // walker's `(e - predicted) as i16` aliased on the first scan iter
        // and broke out before scanning the correct predecessor edge, so
        // every pair emit at 9600 free-ran. i32 accumulator fixes it.
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let edges_vec = tail_edges(&tail, 1000, TPB_9600);
        let edges = edges16(&edges_vec);
        let tail_anchor = 1000u16.wrapping_add(30u16.wrapping_mul(TPB_9600));
        c.seed_walker(tail_anchor, 0);
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.walk_pairs_back(&edges, 3, TPB_9600, &mut pairs);
        let byte = BYTE_TICKS_9600;
        assert_eq!(
            pairs.as_slice(),
            &[
                (
                    1000u16.wrapping_add(2u16.wrapping_mul(byte)),
                    1000u16.wrapping_add(3u16.wrapping_mul(byte))
                ),
                (
                    1000u16.wrapping_add(byte),
                    1000u16.wrapping_add(2u16.wrapping_mul(byte))
                ),
                (1000u16, 1000u16.wrapping_add(byte)),
            ]
        );
    }

    #[test]
    fn walk_pairs_back_skips_out_of_window_edge() {
        // Predecessor stamp perturbed by +3·bit (outside 2·bit snap) —
        // walk should free-run past it (no pair emitted at that step).
        let mut c = make();
        let e0 = 1000u16;
        let e1 = 1000u16.wrapping_add(10 * TPB_3M);
        // e2 shifted way off: +5·bit (well outside snap).
        let e2 = 1000u16.wrapping_add(20 * TPB_3M).wrapping_add(5 * TPB_3M);
        let e3 = 1000u16.wrapping_add(30 * TPB_3M);
        let edges = edges16(&[e0, e1, e2, e3]);
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.seed_walker(e3, 0);
        c.walk_pairs_back(&edges, 3, TPB_3M, &mut pairs);
        // Step 1 (predicted ≈ e3-10·bit = expected e2 slot): e2 is out
        // of window; predicted position is close to e2's SHOULD-BE slot
        // but no edge lands there → free-run. Step 2 (predicted =
        // e2_expected - 10·bit = e1_expected): e1 hits → emit. Step 3:
        // e0 hits → emit. Result: 2 pairs, both after free-run.
        assert_eq!(pairs.len(), 2);
    }

    // ---------- anchor+walker cache emission ----------

    #[test]
    fn anchor_populates_tail_starts_cache_for_multi_edge_crc() {
        // Regression: CRC bytes can have multi-edge patterns. Anchor's
        // per-byte start-edge capture must record byte N's OLDEST edge
        // (its start), not its newest — otherwise `tail_starts[N-1]`
        // ≠ tail_anchor and the cache emits garbage pairs. Use 0x55 as
        // CRC_hi (5 edges/byte) so this only passes if `k == 0`
        // recording is correct.
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0x55];
        let edges_vec = tail_edges(&tail, 1000, TPB_3M);
        let mut edges = edges16(&edges_vec);
        assert!(c.anchor_at_tail(&mut edges, TPB_3M, &tail, 0));
        // tail_anchor tick == tail_starts[3] (last byte's start), and
        // the three cached pairs are consecutive byte-time deltas.
        assert_eq!(c.anchor.tail_anchor, Some(c.anchor.tail_starts[3]));
        for k in 1..TAIL_STARTS {
            let delta = c.anchor.tail_starts[k].wrapping_sub(c.anchor.tail_starts[k - 1]);
            assert_eq!(delta, 10 * TPB_3M, "byte {} spacing", k);
        }
    }

    #[test]
    fn walk_pairs_back_emits_cache_pairs_without_ring_scan() {
        // Steady-state target: emit the 3 anchor-cached pairs, then
        // stop (n_pairs == 3). No ring walk needed. Bypass anchor
        // signature validation by forcing state directly, using edges
        // the walker's ring-scan path would refuse to snap to — proves
        // the pairs came from the cache, not the ring.
        let mut c = make();
        let starts = [1000u16, 2000, 3000, 4000];
        c.seed_cache(starts, 0);
        let edges = edges16(&[0xDEAD, 0xBEEF]); // decoy — walker must ignore
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.walk_pairs_back(&edges, 3, TPB_3M, &mut pairs);
        assert_eq!(
            pairs.as_slice(),
            &[(1000, 2000), (2000, 3000), (3000, 4000)]
        );
    }

    #[test]
    fn walk_pairs_back_cache_plus_one_ring_pair() {
        // Steady in an ideal packet asks for 4 pairs. Cache emits 3;
        // walker walks 1 more from the OLDEST cached start
        // (`tail_starts[0]`) back through the ring.
        let mut c = make();
        let tail = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let edges_vec = tail_edges(&tail, 1000, TPB_3M);
        // Prepend one older edge (byte-time older than tail_starts[0])
        // that the walker snaps to for the 4th pair.
        let mut extended: heapless::Vec<u16, 20> = heapless::Vec::new();
        let _ = extended.push(1000u16.wrapping_sub(BYTE_TICKS_3M));
        for &e in edges_vec.iter() {
            let _ = extended.push(e);
        }
        let mut edges = edges16(&extended);
        assert!(c.anchor_at_tail(&mut edges, TPB_3M, &tail, 0));
        let starts = c.anchor.tail_starts;
        let mut pairs: heapless::Vec<(u16, u16), 8> = heapless::Vec::new();
        c.walk_pairs_back(&edges, 4, TPB_3M, &mut pairs);
        assert_eq!(pairs.len(), 4);
        // First 3 come from cache.
        assert_eq!(pairs[0], (starts[0], starts[1]));
        assert_eq!(pairs[1], (starts[1], starts[2]));
        assert_eq!(pairs[2], (starts[2], starts[3]));
        // 4th: walker snaps to the prepended older edge.
        let older = 1000u16.wrapping_sub(BYTE_TICKS_3M);
        assert_eq!(pairs[3], (older, starts[0]));
    }
}
