use osc_core::BaudRate;

use crate::sim::uart::bit_period_ns;

/// Layer 1 — pure UART-frame encoder. Translates one byte into wire-edge
/// transitions at exact baud timing. No state beyond the bit period.
pub struct TxEncoder {
    bit_period_ns: u64,
}

impl TxEncoder {
    pub fn new(baud: BaudRate) -> Self {
        Self {
            bit_period_ns: bit_period_ns(baud),
        }
    }

    pub fn bit_period_ns(&self) -> u64 {
        self.bit_period_ns
    }

    /// Returns the level transitions for one UART frame containing `byte`
    /// starting at `byte_start_ns`. Idle high; start bit falls at
    /// `byte_start_ns`; data bits LSB-first; stop bit rises if the last data
    /// bit was 0. Time order.
    pub fn encode_byte(&self, byte: u8, byte_start_ns: u64) -> Vec<(u64, bool)> {
        let mut prev_level = true;
        let mut out = Vec::with_capacity(10);
        for i in 0..=9u8 {
            let level = match i {
                0 => false,
                1..=8 => (byte >> (i - 1)) & 1 != 0,
                9 => true,
                _ => unreachable!(),
            };
            if level != prev_level {
                let at = byte_start_ns + i as u64 * self.bit_period_ns;
                out.push((at, level));
                prev_level = level;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::defaults::DEFAULT_BAUD as BAUD;

    #[test]
    fn emits_start_and_first_transition_for_0xff() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        assert_eq!(enc.encode_byte(0xFF, 0), vec![(0, false), (bp, true)]);
    }

    #[test]
    fn emits_start_and_stop_only_for_0x00() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        assert_eq!(enc.encode_byte(0x00, 0), vec![(0, false), (9 * bp, true)]);
    }

    #[test]
    fn emits_alternating_transitions_for_0x55() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let edges = enc.encode_byte(0x55, 0);
        assert_eq!(edges.len(), 10);
        for (i, &(at, rising)) in edges.iter().enumerate() {
            assert_eq!(at, i as u64 * bp);
            assert_eq!(rising, i % 2 == 1);
        }
    }

    #[test]
    fn starts_at_offset() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let edges = enc.encode_byte(0xFF, 1_000_000);
        assert_eq!(edges, vec![(1_000_000, false), (1_000_000 + bp, true)]);
    }
}
