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
}
