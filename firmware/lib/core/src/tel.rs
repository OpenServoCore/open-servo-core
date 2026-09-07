//! TEL stream payload layout: the bounded burst of osc-native status
//! frames (result = `Stream`, protocol sec 5.3) a servo emits on the main
//! bus after a committed nonzero `tel_count` write. Each frame is
//! break-delimited and osc-CRC-16 protected, so a corrupt frame is
//! detectable; this module owns only the payload bytes between INST and
//! CRC. Layout, all LE:
//!
//!   [0]    stream_seq  u8, increments per frame, wraps
//!   [1]    flags       bit 0 = LAST frame of the burst; rest reserved 0
//!   [2..4] valid       u16 bitmap, bit i = sample i window_valid
//!   [4..]  samples     the `tel_mask`-selected fields in bit order,
//!                      2 bytes each; count = payload remainder / sample_len

/// `tel_mask` bits, in canonical sample order. All v1 fields are 2 bytes.
pub const BIT_POS: u16 = 1 << 0;
pub const BIT_CURRENT: u16 = 1 << 1;
pub const BIT_CURRENT_TROUGH: u16 = 1 << 2;
pub const BIT_DUTY: u16 = 1 << 3;
pub const BIT_VDIFF: u16 = 1 << 4;
pub const BIT_VBUS: u16 = 1 << 5;
pub const MASK_ALL: u16 = 0x3F;

/// Payload flags bit 0: last frame of the burst; the line frees after it.
pub const FLAG_LAST: u8 = 1 << 0;

/// Fixed batch size: one frame carries up to 16 fast-tick samples (the
/// burst's last frame may carry fewer).
pub const STREAM_SAMPLES_MAX: usize = 16;

pub const STREAM_HDR: usize = 4;

pub const fn sample_len(mask: u16) -> usize {
    2 * (mask & MASK_ALL).count_ones() as usize
}

pub const STREAM_PAYLOAD_MAX: usize = STREAM_HDR + STREAM_SAMPLES_MAX * sample_len(MASK_ALL);

/// Reserved bits are invalid; mask 0 is valid (stream disarmed).
pub const fn mask_valid(mask: u16) -> bool {
    mask & !MASK_ALL == 0
}

/// One fast tick's streamable primitives, in device counts. `current` is
/// the signed bias-subtracted window sample (the fitter's domain, matching
/// `i_hat_counts`); `current_trough` and `vbus` stay raw unsigned like
/// their telemetry counterparts. `window_valid` (this tick's drive window
/// met the sampling floors) travels in the frame's `valid` bitmap, not per
/// sample.
#[derive(Copy, Clone, Debug, Default)]
pub struct TelSample {
    pub pos: u16,
    pub current: i16,
    pub current_trough: u16,
    pub duty_q15: i16,
    pub vdiff: i16,
    pub vbus: u16,
    pub window_valid: bool,
    /// Kernel fault mask nonzero this tick. Travels in the frame's INST
    /// ALERT bit (the fault contract), never in the payload.
    pub fault: bool,
}

/// The kernel's per-fast-tick stream hook; core never names the transport.
/// The sink owns its own gate: `on_tick` fires only while `active`.
pub trait TelStream {
    fn active(&self) -> bool;
    fn on_tick(&mut self, sample: &TelSample);
}

/// Stream absent (boards without a burst producer, tests).
impl TelStream for () {
    fn active(&self) -> bool {
        false
    }
    fn on_tick(&mut self, _sample: &TelSample) {}
}

/// Serialize the 4-byte stream header. Order-free vs the samples: it never
/// touches bytes past [`STREAM_HDR`], so an incremental encoder writes it
/// last, once the batch's `valid` bitmap is known.
pub fn encode_stream_hdr(
    stream_seq: u8,
    last: bool,
    valid: u16,
    buf: &mut [u8; STREAM_PAYLOAD_MAX],
) {
    buf[0] = stream_seq;
    buf[1] = if last { FLAG_LAST } else { 0 };
    buf[2..4].copy_from_slice(&valid.to_le_bytes());
}

/// Byte offset of sample `i` in a stream payload.
pub const fn sample_offset(mask: u16, i: usize) -> usize {
    STREAM_HDR + i * sample_len(mask)
}

/// Serialize one sample's mask-selected fields at `at`, returning the next
/// offset. `mask` reserved bits are ignored (callers gate on `mask_valid`);
/// `at` past the last sample slot clamps (caller contract, debug-asserted),
/// so the fixed-size buffer keeps every write structurally in-bounds.
pub fn encode_sample(
    mask: u16,
    s: &TelSample,
    buf: &mut [u8; STREAM_PAYLOAD_MAX],
    at: usize,
) -> usize {
    let m = mask & MASK_ALL;
    let cap = sample_offset(m, STREAM_SAMPLES_MAX - 1);
    debug_assert!(at <= cap);
    let mut n = if at > cap { cap } else { at };
    let mut put = |bit: u16, le: [u8; 2]| {
        if m & bit != 0 {
            buf[n] = le[0];
            buf[n + 1] = le[1];
            n += 2;
        }
    };
    put(BIT_POS, s.pos.to_le_bytes());
    put(BIT_CURRENT, s.current.to_le_bytes());
    put(BIT_CURRENT_TROUGH, s.current_trough.to_le_bytes());
    put(BIT_DUTY, s.duty_q15.to_le_bytes());
    put(BIT_VDIFF, s.vdiff.to_le_bytes());
    put(BIT_VBUS, s.vbus.to_le_bytes());
    n
}

/// Serialize one stream payload into `buf`, returning its length. `samples`
/// beyond [`STREAM_SAMPLES_MAX`] truncate (caller contract, debug-asserted).
/// Built on the same appenders the driver-side incremental encoder uses, so
/// the two paths cannot diverge.
pub fn encode_stream(
    mask: u16,
    stream_seq: u8,
    last: bool,
    samples: &[TelSample],
    buf: &mut [u8; STREAM_PAYLOAD_MAX],
) -> usize {
    debug_assert!(samples.len() <= STREAM_SAMPLES_MAX);
    let count = if samples.len() > STREAM_SAMPLES_MAX {
        STREAM_SAMPLES_MAX
    } else {
        samples.len()
    };
    let mut valid: u16 = 0;
    let mut n = STREAM_HDR;
    for (i, s) in samples[..count].iter().enumerate() {
        if s.window_valid {
            valid |= 1 << i;
        }
        n = encode_sample(mask, s, buf, n);
    }
    encode_stream_hdr(stream_seq, last, valid, buf);
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_len_counts_selected_fields() {
        assert_eq!(sample_len(0), 0);
        assert_eq!(sample_len(BIT_POS), 2);
        assert_eq!(sample_len(BIT_POS | BIT_DUTY | BIT_VDIFF), 6);
        assert_eq!(sample_len(MASK_ALL), 12);
        assert_eq!(STREAM_PAYLOAD_MAX, STREAM_HDR + 16 * 12);
    }

    #[test]
    fn reserved_bits_are_invalid() {
        assert!(mask_valid(0));
        assert!(mask_valid(MASK_ALL));
        assert!(!mask_valid(1 << 6));
        assert!(!mask_valid(1 << 15));
        assert!(!mask_valid(MASK_ALL | 1 << 6));
    }

    /// Frame wire time must not outrun the batch it carries: the largest
    /// frame (full mask, 16 samples, 196 B payload + 6 B frame overhead +
    /// break) fits 16 fast ticks at 3 Mbaud with >= 5% margin.
    #[test]
    fn largest_frame_fits_its_batch_window() {
        const WIRE_BYTES: usize = STREAM_PAYLOAD_MAX + 6 + 1;
        const { assert!(WIRE_BYTES * 10 <= STREAM_SAMPLES_MAX * 150 * 95 / 100) }
    }

    fn sample(i: usize) -> TelSample {
        TelSample {
            pos: 0x1000 + i as u16,
            current: -(i as i16) - 1,
            current_trough: 0xB000 + i as u16,
            duty_q15: 0x2000 + i as i16,
            vdiff: -300 - i as i16,
            vbus: 1800 + i as u16,
            window_valid: i.is_multiple_of(2),
            // varies across samples; the goldens below pin that it never
            // reaches the payload
            fault: i.is_multiple_of(3),
        }
    }

    /// Test-local decoder: splits one payload back into header + samples.
    fn decode(mask: u16, payload: &[u8]) -> (u8, u8, u16, heapless::Vec<TelSample, 16>) {
        let slen = sample_len(mask);
        let (seq, flags) = (payload[0], payload[1]);
        let valid = u16::from_le_bytes([payload[2], payload[3]]);
        let body = &payload[STREAM_HDR..];
        assert_eq!(body.len() % slen, 0);
        let mut out = heapless::Vec::new();
        for (i, ch) in body.chunks(slen).enumerate() {
            let mut s = TelSample {
                window_valid: valid & (1 << i) != 0,
                ..Default::default()
            };
            let mut off = 0;
            let mut take = |bit: u16| {
                if mask & bit != 0 {
                    let v = u16::from_le_bytes([ch[off], ch[off + 1]]);
                    off += 2;
                    v
                } else {
                    0
                }
            };
            s.pos = take(BIT_POS);
            s.current = take(BIT_CURRENT) as i16;
            s.current_trough = take(BIT_CURRENT_TROUGH);
            s.duty_q15 = take(BIT_DUTY) as i16;
            s.vdiff = take(BIT_VDIFF) as i16;
            s.vbus = take(BIT_VBUS);
            out.push(s).unwrap();
        }
        (seq, flags, valid, out)
    }

    #[test]
    fn encode_golden_full_mask_full_batch() {
        let samples: heapless::Vec<TelSample, 16> = (0..16).map(sample).collect();
        let mut buf = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(MASK_ALL, 0x42, false, &samples, &mut buf);
        assert_eq!(n, STREAM_PAYLOAD_MAX);
        // header: seq, flags (not LAST), valid = even sample indices
        assert_eq!(buf[..4], [0x42, 0x00, 0x55, 0x55]);
        // first sample, all six fields in bit order
        assert_eq!(
            buf[4..16],
            [
                0x00, 0x10, 0xFF, 0xFF, 0x00, 0xB0, 0x00, 0x20, 0xD4, 0xFE, 0x08, 0x07
            ]
        );
        // last sample (i = 15)
        assert_eq!(
            buf[184..196],
            [
                0x0F, 0x10, 0xF0, 0xFF, 0x0F, 0xB0, 0x0F, 0x20, 0xC5, 0xFE, 0x17, 0x07
            ]
        );
    }

    #[test]
    fn encode_subset_mask_packs_in_bit_order() {
        // ladder mask 0x1B: pos + current + duty + vdiff
        let mask = 0x1B;
        let samples = [sample(0), sample(1)];
        let mut buf = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(mask, 7, false, &samples, &mut buf);
        assert_eq!(n, STREAM_HDR + 2 * 8);
        assert_eq!(buf[..4], [7, 0, 0x01, 0x00]);
        assert_eq!(buf[4..12], [0x00, 0x10, 0xFF, 0xFF, 0x00, 0x20, 0xD4, 0xFE]);
        assert_eq!(
            buf[12..20],
            [0x01, 0x10, 0xFE, 0xFF, 0x01, 0x20, 0xD3, 0xFE]
        );
    }

    #[test]
    fn encode_partial_last_frame() {
        let samples = [sample(0), sample(1), sample(2)];
        let mut buf = [0xEEu8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(BIT_POS, 0xFF, true, &samples, &mut buf);
        assert_eq!(n, STREAM_HDR + 3 * 2);
        assert_eq!(buf[..4], [0xFF, FLAG_LAST, 0x05, 0x00]);
        assert_eq!(buf[4..10], [0x00, 0x10, 0x01, 0x10, 0x02, 0x10]);
        assert!(buf[10..].iter().all(|&b| b == 0xEE));
    }

    /// The driver-side incremental path (per-tick `encode_sample`, header
    /// last) is byte-identical to `encode_stream` for the same inputs.
    #[test]
    fn incremental_encode_matches_encode_stream() {
        for (mask, count) in [(MASK_ALL, 16), (0x1B, 16), (BIT_POS, 3), (MASK_ALL, 1)] {
            let samples: heapless::Vec<TelSample, 16> = (0..count).map(sample).collect();
            let mut whole = [0u8; STREAM_PAYLOAD_MAX];
            let n = encode_stream(mask, 7, true, &samples, &mut whole);

            let mut inc = [0u8; STREAM_PAYLOAD_MAX];
            let mut valid = 0u16;
            let mut at = STREAM_HDR;
            for (i, s) in samples.iter().enumerate() {
                if s.window_valid {
                    valid |= 1 << i;
                }
                assert_eq!(at, sample_offset(mask, i));
                at = encode_sample(mask, s, &mut inc, at);
            }
            encode_stream_hdr(7, true, valid, &mut inc);
            assert_eq!(at, n);
            assert_eq!(inc[..n], whole[..n]);
        }
    }

    #[test]
    fn round_trip_all_masks() {
        for mask in [MASK_ALL, 0x1B, BIT_VBUS] {
            let samples: heapless::Vec<TelSample, 16> = (0..5).map(sample).collect();
            let mut buf = [0u8; STREAM_PAYLOAD_MAX];
            let n = encode_stream(mask, 9, true, &samples, &mut buf);
            let (seq, flags, _, got) = decode(mask, &buf[..n]);
            assert_eq!((seq, flags), (9, FLAG_LAST));
            assert_eq!(got.len(), samples.len());
            for (g, w) in got.iter().zip(&samples) {
                assert_eq!(g.window_valid, w.window_valid);
                let m = |bit: u16, v: i32| if mask & bit != 0 { v } else { 0 };
                assert_eq!(g.pos as i32, m(BIT_POS, w.pos as i32));
                assert_eq!(g.current as i32, m(BIT_CURRENT, w.current as i32));
                assert_eq!(
                    g.current_trough as i32,
                    m(BIT_CURRENT_TROUGH, w.current_trough as i32)
                );
                assert_eq!(g.duty_q15 as i32, m(BIT_DUTY, w.duty_q15 as i32));
                assert_eq!(g.vdiff as i32, m(BIT_VDIFF, w.vdiff as i32));
                assert_eq!(g.vbus as i32, m(BIT_VBUS, w.vbus as i32));
            }
        }
    }
}
