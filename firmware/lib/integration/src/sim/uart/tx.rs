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

/// Layer 2 — scheduled TX port. Owns a [`TxEncoder`], a pending-byte queue,
/// and a transmission log. Devices push bytes via [`UartTx::queue_byte`] and
/// call [`UartTx::advance`] when the orchestrator reaches a scheduled time.
pub struct UartTx {
    encoder: TxEncoder,
    pending: BinaryHeap<Reverse<(SimTime, u8)>>,
    tx_log: Vec<TxLogEntry>,
}

impl UartTx {
    pub fn new(baud: BaudRate) -> Self {
        Self {
            encoder: TxEncoder::new(baud),
            pending: BinaryHeap::new(),
            tx_log: Vec::new(),
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

    /// Stage `byte` for transmission starting at `at`. The first edge (start
    /// bit) fires at that instant; subsequent transitions follow one bit
    /// period apart.
    pub fn queue_byte(&mut self, byte: u8, at: SimTime) {
        self.pending.push(Reverse((at, byte)));
    }

    /// Time of the next queued byte, or `None` if idle.
    pub fn next_wake(&self) -> Option<SimTime> {
        self.pending.peek().map(|r| r.0.0)
    }

    /// Drain every queued byte whose start time equals `t`, encode each into
    /// level transitions, and append a [`TxLogEntry`] per byte. Returns the
    /// transitions in time order.
    pub fn advance(&mut self, t: SimTime) -> Vec<(u64, bool)> {
        let mut out = Vec::new();
        while let Some(top) = self.pending.peek() {
            if top.0.0 != t {
                break;
            }
            let (at, byte) = self.pending.pop().unwrap().0;
            self.tx_log.push(TxLogEntry { at, byte });
            out.extend(self.encoder.encode_byte(byte, at.as_ns()));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BAUD: BaudRate = BaudRate::B115200;

    #[test]
    fn queue_then_advance_drains_due_byte_and_logs() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xFF, SimTime::ZERO);
        assert_eq!(tx.next_wake(), Some(SimTime::ZERO));

        let edges = tx.advance(SimTime::ZERO);
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
        assert_eq!(tx.advance(SimTime::ZERO), vec![]);
        assert!(tx.tx_log().is_empty());
        assert_eq!(tx.next_wake(), Some(SimTime::from_ns(10 * bp)));
    }

    #[test]
    fn advance_drains_in_order_across_calls() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::ZERO);
        tx.queue_byte(0x55, SimTime::from_ns(10 * bp));

        tx.advance(SimTime::ZERO);
        tx.advance(SimTime::from_ns(10 * bp));
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
    fn clear_log_drops_history_but_keeps_pending() {
        let mut tx = UartTx::new(BAUD);
        let bp = tx.bit_period_ns();
        tx.queue_byte(0xAA, SimTime::ZERO);
        tx.advance(SimTime::ZERO);
        tx.queue_byte(0x55, SimTime::from_ns(10 * bp));

        tx.clear_log();
        assert!(tx.tx_log().is_empty());
        assert_eq!(tx.next_wake(), Some(SimTime::from_ns(10 * bp)));
    }
}
