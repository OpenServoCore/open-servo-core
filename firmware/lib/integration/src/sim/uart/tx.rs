use std::cmp::Reverse;
use std::collections::BinaryHeap;

use osc_core::BaudRate;

use crate::sim::SimTime;
use crate::sim::uart::TxEncoder;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TxLogEntry {
    pub at: SimTime,
    pub byte: u8,
}

/// Where a pending byte's value comes from at drain time.
///
/// `Value` is a snapshot taken at queue time — suitable for callers whose
/// TX bytes never change after they're queued (the Host). `Offset` is an
/// index into the caller's TX buffer that [`UartTx::advance`] reads live —
/// any patch landing between queue and drain (Fast Last CRC) is honored.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum PendingSource {
    Value(u8),
    Offset(u32),
}

/// Layer 2 — scheduled TX port. Owns a [`TxEncoder`], a pending-byte queue,
/// and a transmission log. Devices push bytes via [`UartTx::queue_byte`] /
/// [`UartTx::queue_byte_indirect`] and call [`UartTx::advance`] when the
/// orchestrator reaches a scheduled time.
pub struct UartTx {
    encoder: TxEncoder,
    pending: BinaryHeap<Reverse<(SimTime, PendingSource)>>,
    tx_log: Vec<TxLogEntry>,
    /// End-of-frame time for the latest queued byte. A new queue whose
    /// `at` precedes this would model two byte streams driving the same TX
    /// peripheral concurrently — physically impossible (one shift register).
    wire_busy_until: SimTime,
}

impl UartTx {
    pub fn new(baud: BaudRate) -> Self {
        Self {
            encoder: TxEncoder::new(baud),
            pending: BinaryHeap::new(),
            tx_log: Vec::new(),
            wire_busy_until: SimTime::ZERO,
        }
    }

    pub fn bit_period_ns(&self) -> u64 {
        self.encoder.bit_period_ns()
    }

    pub fn tx_log(&self) -> &[TxLogEntry] {
        &self.tx_log
    }

    pub fn clear_log(&mut self) {
        self.tx_log.clear();
    }

    /// Stage `byte` for transmission starting at `at`. The value is
    /// snapshotted now; later mutations of any backing buffer do not
    /// affect what ships out. For callers whose TX bytes never get
    /// patched after queueing (e.g. the Host).
    pub fn queue_byte(&mut self, byte: u8, at: SimTime) {
        self.check_and_advance_wire_busy(at);
        self.pending.push(Reverse((at, PendingSource::Value(byte))));
    }

    /// Stage a byte for transmission starting at `at`, to be resolved
    /// from `tx_buf[offset]` at drain time. Models the DMA cursor read in
    /// production — patches landing between queue and drain (Fast Last
    /// chain-CRC) are honored. Pair with [`Self::advance`] passing the
    /// live buf slice.
    pub fn queue_byte_indirect(&mut self, offset: u32, at: SimTime) {
        self.check_and_advance_wire_busy(at);
        self.pending
            .push(Reverse((at, PendingSource::Offset(offset))));
    }

    fn check_and_advance_wire_busy(&mut self, at: SimTime) {
        assert!(
            at >= self.wire_busy_until,
            "UartTx: byte queued at {:?} overlaps in-flight frame ending at {:?} \
             — one UART TX peripheral cannot drive two byte streams concurrently",
            at,
            self.wire_busy_until,
        );
        self.wire_busy_until = at + 10 * self.bit_period_ns();
    }

    /// Time of the next queued byte, or `None` if idle.
    pub fn next_wake(&self) -> Option<SimTime> {
        self.pending.peek().map(|r| r.0.0)
    }

    /// Drain every queued byte whose start time equals `t`, encode each
    /// into level transitions, and append a [`TxLogEntry`] per byte.
    /// `Offset` entries read `tx_buf[offset]` LIVE — pass the caller's
    /// current TX buffer so any patches between
    /// [`Self::queue_byte_indirect`] and now are picked up. Callers with
    /// no `Offset` entries (e.g. the Host) may pass `&[]`.
    pub fn advance(&mut self, t: SimTime, tx_buf: &[u8]) -> Vec<(u64, bool)> {
        let mut out = Vec::new();
        while let Some(top) = self.pending.peek() {
            if top.0.0 != t {
                break;
            }
            let (at, source) = self.pending.pop().unwrap().0;
            let byte = match source {
                PendingSource::Value(b) => b,
                PendingSource::Offset(o) => tx_buf[o as usize],
            };
            self.tx_log.push(TxLogEntry { at, byte });
            out.extend(self.encoder.encode_byte(byte, at.as_ns()));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::defaults::DEFAULT_BAUD as BAUD;

    #[test]
    fn queue_then_advance_drains_due_byte_and_logs() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xFF, SimTime::ZERO);
        assert_eq!(tx.next_wake(), Some(SimTime::ZERO));

        let edges = tx.advance(SimTime::ZERO, &[]);
        assert_eq!(edges, vec![(0, false), (bp, true)]);
        assert_eq!(
            tx.tx_log(),
            &[TxLogEntry {
                at: SimTime::ZERO,
                byte: 0xFF
            }]
        );
        assert_eq!(tx.next_wake(), None);
    }

    #[test]
    fn advance_skips_bytes_not_due() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::from_ns(10 * bp));
        assert_eq!(tx.advance(SimTime::ZERO, &[]), vec![]);
        assert!(tx.tx_log().is_empty());
        assert_eq!(tx.next_wake(), Some(SimTime::from_ns(10 * bp)));
    }

    #[test]
    fn advance_drains_in_order_across_calls() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::ZERO);
        tx.queue_byte(0x55, SimTime::from_ns(10 * bp));

        tx.advance(SimTime::ZERO, &[]);
        tx.advance(SimTime::from_ns(10 * bp), &[]);
        assert_eq!(
            tx.tx_log(),
            &[
                TxLogEntry {
                    at: SimTime::ZERO,
                    byte: 0xAA,
                },
                TxLogEntry {
                    at: SimTime::from_ns(10 * bp),
                    byte: 0x55,
                },
            ],
        );
    }

    #[test]
    fn back_to_back_bytes_at_exact_frame_boundary_are_allowed() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::ZERO);
        tx.queue_byte(0x55, SimTime::from_ns(10 * bp));
    }

    #[test]
    #[should_panic(expected = "overlaps in-flight frame")]
    fn overlapping_queue_panics_within_frame() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::ZERO);
        // 1 bit_period into the first frame — frame ends at 10*bp.
        tx.queue_byte(0x55, SimTime::from_ns(bp));
    }

    #[test]
    #[should_panic(expected = "overlaps in-flight frame")]
    fn overlapping_queue_panics_one_ns_before_end() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::ZERO);
        tx.queue_byte(0x55, SimTime::from_ns(10 * bp - 1));
    }

    #[test]
    fn clear_log_drops_history_but_keeps_pending() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::ZERO);
        tx.advance(SimTime::ZERO, &[]);
        tx.queue_byte(0x55, SimTime::from_ns(10 * bp));

        tx.clear_log();
        assert!(tx.tx_log().is_empty());
        assert_eq!(tx.next_wake(), Some(SimTime::from_ns(10 * bp)));
    }

    #[test]
    fn indirect_reads_buf_at_advance_time_not_queue_time() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        let mut buf = [0xAA_u8, 0xBB];

        // Queue an indirect byte pointing at buf[1].
        tx.queue_byte_indirect(1, SimTime::from_ns(10 * bp));

        // Mutate the buf AFTER queueing — production patch_crc analogue.
        buf[1] = 0x55;

        // Drain at the scheduled time — emitted byte should be the
        // post-mutation value, proving deferred-read.
        tx.advance(SimTime::from_ns(10 * bp), &buf);
        assert_eq!(
            tx.tx_log(),
            &[TxLogEntry {
                at: SimTime::from_ns(10 * bp),
                byte: 0x55,
            }]
        );
    }

    #[test]
    fn indirect_and_value_entries_drain_in_time_order() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        let buf = [0x11_u8, 0x22];

        tx.queue_byte(0xAA, SimTime::ZERO);
        tx.queue_byte_indirect(0, SimTime::from_ns(10 * bp));
        tx.queue_byte(0xBB, SimTime::from_ns(20 * bp));
        tx.queue_byte_indirect(1, SimTime::from_ns(30 * bp));

        tx.advance(SimTime::ZERO, &buf);
        tx.advance(SimTime::from_ns(10 * bp), &buf);
        tx.advance(SimTime::from_ns(20 * bp), &buf);
        tx.advance(SimTime::from_ns(30 * bp), &buf);

        let bytes: Vec<u8> = tx.tx_log().iter().map(|e| e.byte).collect();
        assert_eq!(bytes, vec![0xAA, 0x11, 0xBB, 0x22]);
    }
}
