//! Drift-span tracker. Turns the drain-ISR entry stamps (byte-ring HT/TC
//! and line-IDLE) into `(d_ticks, d_bytes)` spans — the NDTR/byte-count
//! replacement for the deleted edge-pair walk. Each stamp records
//! `(now, published_cursor, flavor)`; a span is emitted only when the new
//! stamp pairs with the previous one over a contiguous, same-flavor burst
//! of Instruction bytes.

use crate::dxl::uart::poll_src::PollSrc;

/// Same-burst gate divisor. A span is rejected when its tick length
/// deviates from `d_bytes · byte_ticks` by more than `expected / 16`. Host
/// inter-packet gaps (≥ RDT + turnaround) blow this by orders of magnitude,
/// while the largest real trim offset (~2 %) passes with wide margin.
const SAME_BURST_GATE_DIV: u32 = 16;

/// Shortest per-byte window span the drift integrator will accept. A 1-byte
/// span at 3M has ±6250 ppm stamp quantization (±1 tick / 160), worse than
/// the ~2500 ppm trim step; 8 bytes brings it to ≤781 ppm (1 / 1280 ticks),
/// finer at every lower baud. Below this the span is dropped rather than fed
/// as a poison low-SNR sample.
const DRIFT_WINDOW_MIN_BYTES: u32 = 8;

struct Stamp {
    now: u32,
    cursor: u32,
    flavor: PollSrc,
}

/// One-stamp-of-history span emitter. Holds no bytes — the codec owns the
/// ring and hands in the published cursor.
pub(super) struct SpanTracker {
    last: Option<Stamp>,
}

impl SpanTracker {
    pub(super) const fn new() -> Self {
        Self { last: None }
    }

    /// Record a drain-ISR stamp and, when it pairs with the previous stamp
    /// over one contiguous same-flavor burst of Instruction bytes, return
    /// the `(d_ticks, d_bytes)` span for the drift integrator. `now` is
    /// already back-dated to the last-byte reference for the flavor (IDLE
    /// stamps subtract one frame upstream so both flavors share a
    /// reference). `instruction` is the codec's drift-sampling gate —
    /// foreign Status bytes are another servo's HSI-clocked TX and must
    /// never contribute ([[drift_sampling_instruction_only]]).
    pub(super) fn on_stamp(
        &mut self,
        now: u32,
        cursor: u32,
        flavor: PollSrc,
        byte_ticks: u32,
        instruction: bool,
    ) -> Option<(u32, u32)> {
        let prev = self.last.replace(Stamp {
            now,
            cursor,
            flavor,
        })?;
        // Same-flavor only: a mixed HT/TC↔IDLE span carries the differential
        // ISR-prologue latency as a systematic offset.
        if prev.flavor != flavor || !instruction {
            return None;
        }
        let d_bytes = cursor.wrapping_sub(prev.cursor);
        // Cursor is monotonic; a zero or wrapped-negative delta is not a span.
        if d_bytes == 0 || (d_bytes as i32) < 0 {
            return None;
        }
        let d_ticks = now.wrapping_sub(prev.now);
        let expected = d_bytes.wrapping_mul(byte_ticks);
        if d_ticks.abs_diff(expected) > expected / SAME_BURST_GATE_DIV {
            return None;
        }
        Some((d_ticks, d_bytes))
    }
}

/// One-span-per-burst sampler for the RXNE cold-start window. While the
/// window is open the driver records the FIRST per-byte wake's
/// `(now, cursor)` and keeps overwriting the LAST one; at the packet
/// boundary [`Self::settle`] emits a single long span from the pair — the
/// path that restores drift sampling for isolated short packets, which
/// trip only one drain ISR and so form no [`SpanTracker`] pair.
///
/// One span per burst by design: pairing consecutive per-byte wakes would
/// hand the integrator 1-byte spans whose ±quantization dwarfs the trim
/// step and biases the batch minimum. The accumulated span is instead
/// gated on [`DRIFT_WINDOW_MIN_BYTES`] and the same-burst rule before it
/// ever reaches the integrator.
pub(super) struct DriftWindow {
    /// `(now, cursor)` of the burst's first per-byte wake; `None` between
    /// bursts.
    first: Option<(u32, u32)>,
    /// `(now, cursor)` of the burst's most recent per-byte wake.
    last: Option<(u32, u32)>,
    pending: Option<(u32, u32)>,
}

impl DriftWindow {
    pub(super) const fn new() -> Self {
        Self {
            first: None,
            last: None,
            pending: None,
        }
    }

    /// Record one per-byte wake. The first wake of a burst anchors the
    /// span; every later wake advances its end. `cursor` is the published
    /// wire-byte cursor (monotonic).
    pub(super) fn record(&mut self, now: u32, cursor: u32) {
        if self.first.is_none() {
            self.first = Some((now, cursor));
        }
        self.last = Some((now, cursor));
    }

    /// Packet boundary — close the current burst's window. Emits a single
    /// span into [`Self::take_span`] iff the burst was an Instruction, the
    /// accumulated length clears [`DRIFT_WINDOW_MIN_BYTES`], and the tick
    /// length passes the same-burst gate; resets the anchor for the next
    /// burst either way. Returns whether a qualifying span was produced.
    pub(super) fn settle(&mut self, byte_ticks: u32, instruction: bool) -> bool {
        let (first, last) = match (self.first.take(), self.last.take()) {
            (Some(f), Some(l)) => (f, l),
            _ => return false,
        };
        if !instruction {
            return false;
        }
        let d_bytes = last.1.wrapping_sub(first.1);
        if d_bytes < DRIFT_WINDOW_MIN_BYTES || (d_bytes as i32) < 0 {
            return false;
        }
        let d_ticks = last.0.wrapping_sub(first.0);
        let expected = d_bytes.wrapping_mul(byte_ticks);
        if d_ticks.abs_diff(expected) > expected / SAME_BURST_GATE_DIV {
            return false;
        }
        self.pending = Some((d_ticks, d_bytes));
        true
    }

    /// Take the accumulated span for the drift integrator, if the last
    /// [`Self::settle`] produced one. Take-once.
    pub(super) fn take_span(&mut self) -> Option<(u32, u32)> {
        self.pending.take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BYTE_TICKS: u32 = 160; // 3M: 10 bits × 16 ticks.

    fn tracker() -> SpanTracker {
        SpanTracker::new()
    }

    #[test]
    fn first_stamp_never_emits() {
        let mut t = tracker();
        assert_eq!(
            t.on_stamp(1000, 0, PollSrc::ByteBatch, BYTE_TICKS, true),
            None
        );
    }

    #[test]
    fn contiguous_same_flavor_instruction_burst_emits_span() {
        let mut t = tracker();
        t.on_stamp(0, 0, PollSrc::ByteBatch, BYTE_TICKS, true);
        // 16 bytes later, 16 byte-times of ticks: a clean burst.
        assert_eq!(
            t.on_stamp(16 * BYTE_TICKS, 16, PollSrc::ByteBatch, BYTE_TICKS, true),
            Some((16 * BYTE_TICKS, 16))
        );
    }

    #[test]
    fn mixed_flavor_pair_is_rejected() {
        let mut t = tracker();
        t.on_stamp(0, 0, PollSrc::ByteBatch, BYTE_TICKS, true);
        assert_eq!(
            t.on_stamp(16 * BYTE_TICKS, 16, PollSrc::LineIdle, BYTE_TICKS, true),
            None
        );
    }

    #[test]
    fn non_instruction_span_is_rejected() {
        let mut t = tracker();
        t.on_stamp(0, 0, PollSrc::ByteBatch, BYTE_TICKS, true);
        assert_eq!(
            t.on_stamp(16 * BYTE_TICKS, 16, PollSrc::ByteBatch, BYTE_TICKS, false),
            None
        );
    }

    #[test]
    fn inter_packet_gap_blows_the_same_burst_gate() {
        let mut t = tracker();
        t.on_stamp(0, 0, PollSrc::ByteBatch, BYTE_TICKS, true);
        // 16 bytes but a millisecond of gap between stamps — rejected.
        assert_eq!(
            t.on_stamp(1_000_000, 16, PollSrc::ByteBatch, BYTE_TICKS, true),
            None
        );
    }

    #[test]
    fn small_trim_offset_passes_the_gate() {
        let mut t = tracker();
        t.on_stamp(0, 0, PollSrc::ByteBatch, BYTE_TICKS, true);
        // +2 % long span: within expected/16 (6.25 %).
        let d_ticks = 16 * BYTE_TICKS + (16 * BYTE_TICKS) / 50;
        assert_eq!(
            t.on_stamp(d_ticks, 16, PollSrc::ByteBatch, BYTE_TICKS, true),
            Some((d_ticks, 16))
        );
    }

    #[test]
    fn zero_byte_span_is_rejected_but_updates_history() {
        let mut t = tracker();
        t.on_stamp(0, 8, PollSrc::ByteBatch, BYTE_TICKS, true);
        // No new bytes: no span, but the stamp still refreshes history.
        assert_eq!(
            t.on_stamp(50, 8, PollSrc::ByteBatch, BYTE_TICKS, true),
            None
        );
        assert_eq!(
            t.on_stamp(
                8 * BYTE_TICKS + 50,
                16,
                PollSrc::ByteBatch,
                BYTE_TICKS,
                true
            ),
            Some((8 * BYTE_TICKS, 8))
        );
    }

    // ---------- drift window ----------

    fn window() -> DriftWindow {
        DriftWindow::new()
    }

    #[test]
    fn window_emits_one_span_for_a_short_instruction_burst() {
        let mut w = window();
        // 10-byte ping: wakes at cursors 1..=10, one byte-time apart.
        for i in 1..=10u32 {
            w.record(i * BYTE_TICKS, i);
        }
        assert!(w.settle(BYTE_TICKS, true));
        // first cursor 1, last cursor 10 → 9 whole bytes.
        assert_eq!(w.take_span(), Some((9 * BYTE_TICKS, 9)));
        // Take-once.
        assert_eq!(w.take_span(), None);
    }

    #[test]
    fn window_resets_between_bursts() {
        let mut w = window();
        for i in 1..=10u32 {
            w.record(i * BYTE_TICKS, i);
        }
        assert!(w.settle(BYTE_TICKS, true));
        let _ = w.take_span();
        // Next burst continues the monotonic cursor; the span must span
        // only the new burst, not straddle the reset.
        for i in 11..=20u32 {
            w.record(i * BYTE_TICKS, i);
        }
        assert!(w.settle(BYTE_TICKS, true));
        assert_eq!(w.take_span(), Some((9 * BYTE_TICKS, 9)));
    }

    #[test]
    fn window_below_min_bytes_is_dropped() {
        let mut w = window();
        // 8 wakes → 7 whole bytes, under the 8-byte floor.
        for i in 1..=8u32 {
            w.record(i * BYTE_TICKS, i);
        }
        assert!(!w.settle(BYTE_TICKS, true));
        assert_eq!(w.take_span(), None);
    }

    #[test]
    fn window_non_instruction_burst_is_dropped() {
        let mut w = window();
        for i in 1..=10u32 {
            w.record(i * BYTE_TICKS, i);
        }
        assert!(!w.settle(BYTE_TICKS, false));
        assert_eq!(w.take_span(), None);
    }

    #[test]
    fn window_inter_packet_gap_blows_the_same_burst_gate() {
        let mut w = window();
        w.record(0, 0);
        // 10 bytes of cursor but a millisecond of ticks between first and
        // last — a straddled inter-packet gap, rejected.
        w.record(1_000_000, 10);
        assert!(!w.settle(BYTE_TICKS, true));
        assert_eq!(w.take_span(), None);
    }

    #[test]
    fn window_settle_without_wakes_is_a_no_op() {
        let mut w = window();
        assert!(!w.settle(BYTE_TICKS, true));
        assert_eq!(w.take_span(), None);
    }
}
