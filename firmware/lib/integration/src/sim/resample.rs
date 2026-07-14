//! Cross-baud reception model: traffic sent at one rate, sampled by a
//! receiver UART at another. This is the physics behind baud-migration
//! garble (protocol sec 2 / sec 8): a slower talker's low runs stretch past
//! the faster receiver's 10-bit break bar (spurious breaks + multiplied
//! characters), while a faster talker's spans compress below the slower
//! receiver's framing (most bytes vanish or merge).
//!
//! One streaming machine per receiver: wire events feed bit spans in wall
//! order, gaps between events are idle-high by construction (one talker at
//! a time), and outputs are ring artifacts -- characters ring regardless of
//! framing violations (FE is only a wake, protocol sec 3.4), a qualified
//! break rings one 0x00 and wakes (F2/F3).
//!
//! Approximations, all conservative: the break bar is 10 receiver bits (LIN
//! LBD); a character completing inside a longer low run rings in addition
//! to the run's break byte; a character left sampling at a frame's end
//! completes against idle-high at the next feed or flush.

use osc_protocol::wire::BaudRate;

use super::core::bit_ticks;

/// One received ring artifact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxOut {
    /// A sampled character (framing violations still ring).
    Byte(u8),
    /// A qualified break span: one 0x00 rings, the wake fires.
    Break,
}

/// In-flight character: falling edge anchored, 9 samples (8 data + stop) at
/// `edge + rx_bit*(3 + 2k)/2`.
struct Sampler {
    edge: u64,
    taken: u8,
    acc: u8,
}

pub struct CrossRx {
    rate: BaudRate,
    rx_bit: u64,
    /// Waveform continuity cursor: `[last_end, next feed)` is idle-high.
    last_end: u64,
    /// Start of the running low span, when the line is low.
    low_since: Option<u64>,
    sampler: Option<Sampler>,
    /// All-zero character completed inside its own still-running low span:
    /// the run decides at close (break swallows it, a short run rings it).
    held_zero: bool,
}

impl CrossRx {
    pub fn new(rate: BaudRate) -> Self {
        Self {
            rate,
            rx_bit: bit_ticks(rate),
            last_end: 0,
            low_since: None,
            sampler: None,
            held_zero: false,
        }
    }

    /// The receiver retuned (deferred baud apply): all sampling state dies
    /// with the old clock.
    pub fn retune(&mut self, rate: BaudRate) {
        if rate != self.rate {
            *self = Self::new(rate);
        }
    }

    /// A data character sent at `tx`, event-stamped at its wire END (the
    /// sim's byte events carry end times).
    pub fn on_byte(&mut self, end: u64, byte: u8, tx: BaudRate, out: &mut Vec<RxOut>) {
        let tx_bit = bit_ticks(tx);
        let start = end.saturating_sub(10 * tx_bit);
        self.idle_gap(start, out);
        self.feed(true, start, tx_bit, out);
        for k in 0..8 {
            let low = byte >> k & 1 == 0;
            self.feed(low, start + (1 + k as u64) * tx_bit, tx_bit, out);
        }
        self.feed(false, start + 9 * tx_bit, tx_bit, out);
    }

    /// A break span `[start, start+dur)` of dominant low.
    pub fn on_break(&mut self, start: u64, dur: u64, out: &mut Vec<RxOut>) {
        self.idle_gap(start, out);
        self.feed(true, start, dur, out);
    }

    /// The line is idle-high from here on: complete or drop what's pending.
    pub fn flush(&mut self, out: &mut Vec<RxOut>) {
        let now = self.last_end;
        if let Some(t0) = self.low_since.take() {
            self.close_run(t0, now, out);
        }
        if let Some(s) = self.sampler.take()
            && s.taken > 0
        {
            let mut acc = s.acc;
            for k in s.taken..8 {
                acc |= 1 << k;
            }
            out.push(RxOut::Byte(acc));
        }
    }

    fn idle_gap(&mut self, start: u64, out: &mut Vec<RxOut>) {
        if start > self.last_end {
            let gap = start - self.last_end;
            self.feed(false, self.last_end, gap, out);
        }
    }

    fn feed(&mut self, low: bool, start: u64, dur: u64, out: &mut Vec<RxOut>) {
        let end = start + dur;
        if low {
            if self.low_since.is_none() {
                self.low_since = Some(start);
                if self.sampler.is_none() {
                    self.sampler = Some(Sampler {
                        edge: start,
                        taken: 0,
                        acc: 0,
                    });
                }
            }
        } else if let Some(t0) = self.low_since.take() {
            self.close_run(t0, start, out);
        }
        self.pump(low, end, out);
        self.last_end = end;
    }

    /// Consume sample instants that fall before `end` at the current level.
    fn pump(&mut self, low: bool, end: u64, out: &mut Vec<RxOut>) {
        while let Some(s) = self.sampler.as_mut() {
            let t = s.edge + self.rx_bit * (3 + 2 * s.taken as u64) / 2;
            if t >= end {
                return;
            }
            if s.taken < 8 && !low {
                s.acc |= 1 << s.taken;
            }
            s.taken += 1;
            if s.taken == 9 {
                let byte = s.acc;
                let edge = s.edge;
                self.sampler = None;
                if low && byte == 0 && self.low_since == Some(edge) {
                    // Still inside its own low run: break-or-zero at close.
                    self.held_zero = true;
                } else {
                    out.push(RxOut::Byte(byte));
                }
            }
        }
    }

    /// A low run `[t0, t1)` ended: break-qualify it (10 receiver bits).
    fn close_run(&mut self, t0: u64, t1: u64, out: &mut Vec<RxOut>) {
        let held = std::mem::take(&mut self.held_zero);
        if t1 - t0 >= 10 * self.rx_bit {
            // The run's own in-flight sampler dies with it; a sampler from
            // an earlier edge cannot exist here (it either completed inside
            // the run or was this run's own).
            if self.sampler.as_ref().is_some_and(|s| s.edge == t0) {
                self.sampler = None;
            }
            out.push(RxOut::Break);
        } else if held {
            out.push(RxOut::Byte(0));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn resample(bytes: &[u8], tx: BaudRate, rx: BaudRate) -> Vec<RxOut> {
        let mut m = CrossRx::new(rx);
        let bt = bit_ticks(tx) * 10;
        let mut out = Vec::new();
        for (k, &b) in bytes.iter().enumerate() {
            m.on_byte((k as u64 + 1) * bt, b, tx, &mut out);
        }
        m.flush(&mut out);
        out
    }

    #[test]
    fn slow_zero_byte_reads_as_break_at_the_faster_rate() {
        // 1M 0x00: start + 8 zero bits = 9 us low = 27 bits at 3M >= 10.
        let out = resample(&[0x00], BaudRate::B1000000, BaudRate::B3000000);
        assert_eq!(out, vec![RxOut::Break]);
    }

    #[test]
    fn slow_ff_reads_as_one_char() {
        // 1M 0xFF at 3M: start low = 3 bits -> samples 0,0 then highs = 0xFC.
        let out = resample(&[0xFF], BaudRate::B1000000, BaudRate::B3000000);
        assert_eq!(out, vec![RxOut::Byte(0xFC)]);
    }

    #[test]
    fn slow_alternating_byte_multiplies() {
        // 1M 0x55 at 3M: three characters (hand-derived sampling).
        let out = resample(&[0x55], BaudRate::B1000000, BaudRate::B3000000);
        assert_eq!(
            out,
            vec![RxOut::Byte(0x1C), RxOut::Byte(0x1C), RxOut::Byte(0xFC)]
        );
    }

    #[test]
    fn matched_rate_round_trips() {
        for b in [0x00u8, 0x5A, 0xFF, 0x01] {
            let out = resample(&[b], BaudRate::B1000000, BaudRate::B1000000);
            if b == 0 {
                // A matched-rate 0x00 is 9 low bits -- under the break bar.
                assert_eq!(out, vec![RxOut::Byte(0)], "byte {b:#04x}");
            } else {
                assert_eq!(out, vec![RxOut::Byte(b)], "byte {b:#04x}");
            }
        }
    }

    #[test]
    fn fast_traffic_mostly_vanishes_at_the_slower_rate() {
        // A 3M frame at 1M: spans compress to fractions of the receiver's
        // bit -- far fewer characters than sent, and no breaks.
        let sent = [0x00u8, 0x05, 0x30, 0x80, 0xA5, 0x11, 0x22, 0x33];
        let out = resample(&sent, BaudRate::B3000000, BaudRate::B1000000);
        assert!(out.len() < sent.len() / 2, "got {out:?}");
        assert!(!out.contains(&RxOut::Break), "got {out:?}");
    }

    #[test]
    fn slow_break_span_qualifies_at_the_faster_rate() {
        let mut m = CrossRx::new(BaudRate::B3000000);
        let mut out = Vec::new();
        // A 1M break: 14 bit-times dominant low.
        m.on_break(0, 14 * bit_ticks(BaudRate::B1000000), &mut out);
        m.flush(&mut out);
        assert_eq!(out, vec![RxOut::Break]);
    }

    #[test]
    fn fast_break_span_is_invisible_at_the_slower_rate() {
        let mut m = CrossRx::new(BaudRate::B1000000);
        let mut out = Vec::new();
        // A 3M break: 14 * 16 ticks = 4.7 receiver bits -- under the bar.
        m.on_break(0, 14 * bit_ticks(BaudRate::B3000000), &mut out);
        m.flush(&mut out);
        assert!(!out.contains(&RxOut::Break), "got {out:?}");
    }

    #[test]
    fn retune_resets_the_machine() {
        let mut m = CrossRx::new(BaudRate::B3000000);
        let mut out = Vec::new();
        m.on_byte(
            10 * bit_ticks(BaudRate::B1000000),
            0x55,
            BaudRate::B1000000,
            &mut out,
        );
        m.retune(BaudRate::B1000000);
        m.flush(&mut out);
        let n = out.len();
        m.flush(&mut out);
        assert_eq!(out.len(), n, "reset machine must hold no residue");
    }
}
