use osc_core::BaudRate;

use crate::sim::uart::bit_period_ns;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RxEffect {
    ByteComplete { byte: u8, complete_at_ns: u64 },
    IdleDetected { at_ns: u64 },
}

enum RxState {
    Idle,
    RxActive {
        start_ns: u64,
        edges: Vec<(u64, bool)>,
    },
    IdleDetectPending {
        idle_at_ns: u64,
    },
}

/// Layer 1 — pure UART RX state machine. Consumes edges and internal ticks
/// in chronological order; emits [`RxEffect`]s. No queue, no log; the caller
/// (a [`UartRx`](crate::sim::uart::UartRx) or a unit test) buffers events
/// upstream and decides what to do with the effects downstream.
pub struct RxDecoder {
    bit_period_ns: u64,
    idle_threshold_bits: u8,
    state: RxState,
}

impl RxDecoder {
    pub fn new(baud: BaudRate) -> Self {
        Self {
            bit_period_ns: bit_period_ns(baud),
            idle_threshold_bits: 10,
            state: RxState::Idle,
        }
    }

    pub fn bit_period_ns(&self) -> u64 {
        self.bit_period_ns
    }

    /// Feed one edge into the state machine. Pushes any newly-completed
    /// effects (byte decode, etc.) onto `out`.
    pub fn on_edge(&mut self, at_ns: u64, rising: bool, out: &mut Vec<RxEffect>) {
        match &mut self.state {
            RxState::Idle => {
                if !rising {
                    self.state = RxState::RxActive {
                        start_ns: at_ns,
                        edges: Vec::new(),
                    };
                }
            }
            RxState::RxActive { start_ns, edges } => {
                let decode_at = *start_ns + 10 * self.bit_period_ns;
                // Any edge past the stop bit (9·bp) belongs to the next
                // byte, not this one — data-bit samples max out at 8.5·bp,
                // so finalizing here doesn't change the decode. Without
                // this, a back-to-back start bit landing a few ns before
                // `decode_at` (typical of integer-tick scheduling at bauds
                // where the wall clock doesn't divide evenly into the
                // ref-clock period) gets absorbed into this byte's edge
                // list and silently dropped when the state transitions.
                let stop_bit_at = *start_ns + 9 * self.bit_period_ns;
                if at_ns >= stop_bit_at {
                    let byte = decode_byte(edges, *start_ns, self.bit_period_ns);
                    out.push(RxEffect::ByteComplete {
                        byte,
                        complete_at_ns: decode_at,
                    });
                    if !rising {
                        self.state = RxState::RxActive {
                            start_ns: at_ns,
                            edges: Vec::new(),
                        };
                    } else {
                        self.state = RxState::IdleDetectPending {
                            idle_at_ns: decode_at
                                + self.idle_threshold_bits as u64 * self.bit_period_ns,
                        };
                    }
                } else {
                    edges.push((at_ns, rising));
                }
            }
            RxState::IdleDetectPending { .. } => {
                if !rising {
                    self.state = RxState::RxActive {
                        start_ns: at_ns,
                        edges: Vec::new(),
                    };
                }
            }
        }
    }

    /// Fire one internal tick (no edge). Completes a byte whose decode
    /// boundary has passed, or transitions out of `IdleDetectPending`.
    pub fn on_internal_tick(&mut self, at_ns: u64, out: &mut Vec<RxEffect>) {
        match &mut self.state {
            RxState::Idle => {}
            RxState::RxActive { start_ns, edges } => {
                let decode_at = *start_ns + 10 * self.bit_period_ns;
                if at_ns >= decode_at {
                    let byte = decode_byte(edges, *start_ns, self.bit_period_ns);
                    out.push(RxEffect::ByteComplete {
                        byte,
                        complete_at_ns: decode_at,
                    });
                    self.state = RxState::IdleDetectPending {
                        idle_at_ns: decode_at
                            + self.idle_threshold_bits as u64 * self.bit_period_ns,
                    };
                }
            }
            RxState::IdleDetectPending { idle_at_ns } => {
                let due = *idle_at_ns;
                if at_ns >= due {
                    self.state = RxState::Idle;
                    out.push(RxEffect::IdleDetected { at_ns: due });
                }
            }
        }
    }

    pub fn next_internal_wake(&self) -> Option<u64> {
        match &self.state {
            RxState::Idle => None,
            RxState::RxActive { start_ns, .. } => Some(*start_ns + 10 * self.bit_period_ns),
            RxState::IdleDetectPending { idle_at_ns } => Some(*idle_at_ns),
        }
    }
}

fn decode_byte(edges: &[(u64, bool)], start_ns: u64, bit_period_ns: u64) -> u8 {
    let mut byte = 0u8;
    for n in 0..8u8 {
        let sample_ns = start_ns + bit_period_ns * 3 / 2 + n as u64 * bit_period_ns;
        let toggles = edges.iter().filter(|(t, _)| *t <= sample_ns).count();
        if toggles % 2 == 1 {
            byte |= 1 << n;
        }
    }
    byte
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::defaults::DEFAULT_BAUD as BAUD;
    use crate::sim::uart::TxEncoder;

    fn round_trip(byte: u8) -> u8 {
        let enc = TxEncoder::new(BAUD);
        let mut dec = RxDecoder::new(BAUD);
        let mut effects = Vec::new();
        for (at, rising) in enc.encode_byte(byte, 0) {
            dec.on_edge(at, rising, &mut effects);
        }
        dec.on_internal_tick(10 * enc.bit_period_ns(), &mut effects);
        let RxEffect::ByteComplete { byte, .. } = effects[0] else {
            panic!("expected ByteComplete");
        };
        byte
    }

    #[test]
    fn round_trips_full_byte_range_samples() {
        for byte in [0x00, 0x01, 0x55, 0xAA, 0x42, 0x7F, 0x80, 0xFF, 0xFD] {
            assert_eq!(round_trip(byte), byte, "byte {byte:#04x}");
        }
    }

    #[test]
    fn emits_idle_after_threshold_lapses() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut dec = RxDecoder::new(BAUD);
        let mut effects = Vec::new();
        for (at, rising) in enc.encode_byte(0xAA, 0) {
            dec.on_edge(at, rising, &mut effects);
        }
        dec.on_internal_tick(10 * bp, &mut effects);
        effects.clear();
        dec.on_internal_tick(20 * bp, &mut effects);
        assert_eq!(effects, vec![RxEffect::IdleDetected { at_ns: 20 * bp }]);
    }

    #[test]
    fn cancels_idle_on_next_start_bit() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut dec = RxDecoder::new(BAUD);
        let mut effects = Vec::new();
        for (at, rising) in enc.encode_byte(0x42, 0) {
            dec.on_edge(at, rising, &mut effects);
        }
        dec.on_internal_tick(10 * bp, &mut effects);
        effects.clear();
        let next_start = 15 * bp;
        for (at, rising) in enc.encode_byte(0xC3, next_start) {
            dec.on_edge(at, rising, &mut effects);
        }
        dec.on_internal_tick(next_start + 10 * bp, &mut effects);
        assert_eq!(
            effects,
            vec![RxEffect::ByteComplete {
                byte: 0xC3,
                complete_at_ns: next_start + 10 * bp,
            }],
        );
    }

    #[test]
    fn handles_back_to_back_bytes_with_sub_bp_overlap() {
        // Simulate integer-tick truncation at 57600 baud: the firmware's
        // `bytes_to_ticks(N)` rounds down, scheduling the next byte's start
        // bit a few ns before this byte's nominal `decode_at`. Real UART
        // hardware samples each bit at midpoint and treats any falling edge
        // past the stop bit as the next start bit — the decoder must do
        // the same, not absorb the early start bit into this byte's edges.
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut dec = RxDecoder::new(BAUD);
        let mut effects = Vec::new();
        for (at, rising) in enc.encode_byte(0x01, 0) {
            dec.on_edge(at, rising, &mut effects);
        }
        // Next byte's start bit lands 2 ns before this one's decode_at.
        let overlap_start = 10 * bp - 2;
        for (at, rising) in enc.encode_byte(0x00, overlap_start) {
            dec.on_edge(at, rising, &mut effects);
        }
        dec.on_internal_tick(overlap_start + 10 * bp, &mut effects);
        let bytes: Vec<u8> = effects
            .iter()
            .filter_map(|e| match e {
                RxEffect::ByteComplete { byte, .. } => Some(*byte),
                _ => None,
            })
            .collect();
        assert_eq!(bytes, vec![0x01, 0x00]);
    }

    #[test]
    fn handles_back_to_back_bytes_at_decode_boundary() {
        let enc = TxEncoder::new(BAUD);
        let bp = enc.bit_period_ns();
        let mut dec = RxDecoder::new(BAUD);
        let mut effects = Vec::new();
        for (at, rising) in enc.encode_byte(0xAA, 0) {
            dec.on_edge(at, rising, &mut effects);
        }
        for (at, rising) in enc.encode_byte(0x55, 10 * bp) {
            dec.on_edge(at, rising, &mut effects);
        }
        dec.on_internal_tick(20 * bp, &mut effects);
        let bytes: Vec<u8> = effects
            .iter()
            .filter_map(|e| match e {
                RxEffect::ByteComplete { byte, .. } => Some(*byte),
                _ => None,
            })
            .collect();
        assert_eq!(bytes, vec![0xAA, 0x55]);
    }
}
