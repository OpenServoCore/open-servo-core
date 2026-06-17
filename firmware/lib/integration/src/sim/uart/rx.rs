use std::collections::VecDeque;

use osc_core::BaudRate;

use crate::sim::SimTime;
use crate::sim::uart::{RxDecoder, RxEffect};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RxLogEntry {
    pub at: SimTime,
    pub kind: RxLogKind,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RxLogKind {
    Byte(u8),
    IdleGap,
}

/// Layer 2 — scheduled RX port. Owns a [`RxDecoder`], a pending-edge queue,
/// and a reception log. Devices push edges via [`UartRx::receive_edge`] and
/// call [`UartRx::advance`] when the orchestrator reaches a scheduled time.
pub struct UartRx {
    decoder: RxDecoder,
    pending_edges: VecDeque<(SimTime, bool)>,
    rx_log: Vec<RxLogEntry>,
}

impl UartRx {
    pub fn new(baud: BaudRate) -> Self {
        Self {
            decoder: RxDecoder::new(baud),
            pending_edges: VecDeque::new(),
            rx_log: Vec::new(),
        }
    }

    pub fn bit_period_ns(&self) -> u64 {
        self.decoder.bit_period_ns()
    }

    pub fn rx_log(&self) -> &[RxLogEntry] {
        &self.rx_log
    }

    pub fn rx_bytes(&self) -> Vec<u8> {
        self.rx_log
            .iter()
            .filter_map(|e| match e.kind {
                RxLogKind::Byte(b) => Some(b),
                RxLogKind::IdleGap => None,
            })
            .collect()
    }

    pub fn clear_log(&mut self) {
        self.rx_log.clear();
    }

    /// Buffer an inbound edge from the wire. The decoder consumes it during
    /// the next [`UartRx::advance`] that reaches `at`.
    pub fn receive_edge(&mut self, at: SimTime, rising: bool) {
        self.pending_edges.push_back((at, rising));
    }

    /// Earliest time the RX needs to be advanced: either the head of the
    /// pending edge queue or the decoder's next internal-tick wake
    /// (decode boundary / idle expiry), whichever is sooner.
    pub fn next_wake(&self) -> Option<SimTime> {
        let edge_at = self.pending_edges.front().map(|(t, _)| *t);
        let internal_at = self.decoder.next_internal_wake().map(SimTime::from_ns);
        [edge_at, internal_at].into_iter().flatten().min()
    }

    /// Drain every pending edge whose time equals `t` into the decoder, then
    /// fire one internal tick at `t` if a decode boundary or idle expiry is
    /// due. Returns the [`RxEffect`]s produced and appends each to the log.
    /// Callers that only care about logging may discard the return value.
    pub fn advance(&mut self, t: SimTime) -> Vec<RxEffect> {
        let mut out = Vec::new();
        while let Some(&(et, _)) = self.pending_edges.front() {
            if et != t {
                break;
            }
            let (_, rising) = self.pending_edges.pop_front().unwrap();
            self.decoder.on_edge(t.as_ns(), rising, &mut out);
        }
        if let Some(wake) = self.decoder.next_internal_wake()
            && wake <= t.as_ns()
        {
            self.decoder.on_internal_tick(t.as_ns(), &mut out);
        }
        log_effects(&mut self.rx_log, &out);
        out
    }
}

fn log_effects(log: &mut Vec<RxLogEntry>, effects: &[RxEffect]) {
    for eff in effects {
        let entry = match *eff {
            RxEffect::ByteComplete {
                byte,
                complete_at_ns,
            } => RxLogEntry {
                at: SimTime::from_ns(complete_at_ns),
                kind: RxLogKind::Byte(byte),
            },
            RxEffect::IdleDetected { at_ns } => RxLogEntry {
                at: SimTime::from_ns(at_ns),
                kind: RxLogKind::IdleGap,
            },
        };
        log.push(entry);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::uart::TxEncoder;

    const BAUD: BaudRate = BaudRate::B115200;

    fn drain_to(rx: &mut UartRx, until_ns: u64, step_ns: u64) {
        let mut t = 0u64;
        while t <= until_ns {
            let _ = rx.advance(SimTime::from_ns(t));
            t += step_ns;
        }
    }

    #[test]
    fn rx_log_mirrors_decoded_effects() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut rx = UartRx::new(BAUD);
        for (at_ns, rising) in enc.encode_byte(0x42, 0) {
            rx.receive_edge(SimTime::from_ns(at_ns), rising);
        }
        drain_to(&mut rx, 20 * bp, bp);
        assert_eq!(
            rx.rx_log(),
            &[
                RxLogEntry {
                    at: SimTime::from_ns(10 * bp),
                    kind: RxLogKind::Byte(0x42),
                },
                RxLogEntry {
                    at: SimTime::from_ns(20 * bp),
                    kind: RxLogKind::IdleGap,
                },
            ],
        );
        assert_eq!(rx.rx_bytes(), vec![0x42]);
    }

    #[test]
    fn advance_returns_effects_for_caller_routing() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut rx = UartRx::new(BAUD);
        for (at_ns, rising) in enc.encode_byte(0x42, 0) {
            rx.receive_edge(SimTime::from_ns(at_ns), rising);
        }
        for i in 0..=9u64 {
            let _ = rx.advance(SimTime::from_ns(i * bp));
        }
        let effects = rx.advance(SimTime::from_ns(10 * bp));
        assert_eq!(
            effects,
            vec![RxEffect::ByteComplete {
                byte: 0x42,
                complete_at_ns: 10 * bp,
            }],
        );
    }

    #[test]
    fn next_wake_yields_pending_edge_when_idle() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut rx = UartRx::new(BAUD);
        assert_eq!(rx.next_wake(), None);

        for (at_ns, rising) in enc.encode_byte(0x42, 5 * bp) {
            rx.receive_edge(SimTime::from_ns(at_ns), rising);
        }
        assert_eq!(rx.next_wake(), Some(SimTime::from_ns(5 * bp)));
    }

    #[test]
    fn next_wake_yields_internal_when_no_pending_edges() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut rx = UartRx::new(BAUD);

        for (at_ns, rising) in enc.encode_byte(0x42, 0) {
            rx.receive_edge(SimTime::from_ns(at_ns), rising);
        }
        for i in 0..=9u64 {
            let _ = rx.advance(SimTime::from_ns(i * bp));
        }
        assert_eq!(rx.next_wake(), Some(SimTime::from_ns(10 * bp)));
    }

    #[test]
    fn clear_log_drops_history() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut rx = UartRx::new(BAUD);
        for (at_ns, rising) in enc.encode_byte(0xAA, 0) {
            rx.receive_edge(SimTime::from_ns(at_ns), rising);
        }
        drain_to(&mut rx, 10 * bp, bp);
        assert!(!rx.rx_log().is_empty());
        rx.clear_log();
        assert!(rx.rx_log().is_empty());
    }
}
