//! TEL stream payload layout: the bounded burst of osc-native status
//! frames (result = `Stream`, protocol sec 5.3) a servo emits on the main
//! bus after a committed nonzero `tel_count` write. Each frame is
//! break-delimited and osc-CRC-16 protected, so a corrupt frame is
//! detectable; this module owns only the payload bytes between INST and
//! CRC. Layout, all LE:
//!
//!   [0]    stream_seq  u8, increments per frame, wraps
//!   [1]    flags       bit 0 = LAST frame of the burst; rest reserved 0
//!   [2..4] tick        u16, row 0's `sample_tick`, low half
//!   [4..]  rows        the `tel_mask`-selected fields per row, packed per
//!                      `osc_protocol::tel::Layout`; the count is in LEN
//!   then   valid       u32 bitmap, bit i = row i window_valid
//!          tail        u8, bit 0 = some row's fault; then a reserved 0
//!
//! Everything a row-0 head holds is known when row 0 lands; what only the
//! batch end knows rides behind the rows.

use core::mem::offset_of;

use osc_protocol::tel::Layout;
pub use osc_protocol::tel::{
    BIT_CURRENT, BIT_CURRENT_RAW, BIT_CURRENT_TROUGH, BIT_DUTY, BIT_NTC_RAW, BIT_POS, BIT_POS_LIN,
    BIT_VBUS, BIT_VBUS_RAW, BIT_VDIFF, BIT_VMOTOR_A, BIT_VMOTOR_B, DUTY_COUNTS_MAX, FIELDS,
    MASK_ALL, WIDE, row_bytes,
};
use osc_protocol::tel::{NARROW_BITS, NARROW_MASK, PAIR_BYTES, WORD_BYTES};
use osc_protocol::wire::{self, BaudRate, STREAM_TAIL_ALERT};

/// The TEL `duty` a logical Q15 duty drives at `pwm_arr`: the kernel's
/// `window::drive_ticks` width, negative in reverse. Hosts quantize a goal
/// with it to compare against the rows.
pub fn duty_counts(duty_q15: i16, pwm_arr: u16) -> i16 {
    let ticks = crate::estimator::window::drive_ticks(duty_q15, pwm_arr) as i16;
    if duty_q15 < 0 { -ticks } else { ticks }
}

/// Payload flags bit 0: last frame of the burst; the line frees after it.
pub const FLAG_LAST: u8 = 1 << 0;

/// Fixed batch size: one frame carries up to 16 fast-tick rows (the
/// burst's last frame may carry fewer).
pub const STREAM_SAMPLES_MAX: usize = wire::STREAM_SAMPLES_MAX;

pub const STREAM_HDR: usize = wire::STREAM_HDR;
pub const STREAM_TRAILER: usize = wire::STREAM_TRAILER;

/// The longest row the encoder and stager keep up with, the kernel parked
/// and at budget (DES `tel_widest_row_keeps_up_at_rest`): a 12 B row
/// drops rows there though the 3 M wire carries 13 B.
pub const ROW_BYTES_CPU_MAX: usize = 11;

/// The longest row a burst arms with at `baud`.
pub const fn row_bytes_max(baud: BaudRate) -> usize {
    let wire = wire::stream_row_bytes_max(baud, wire::STREAM_BUDGET_TICK_HZ);
    if wire < ROW_BYTES_CPU_MAX {
        wire
    } else {
        ROW_BYTES_CPU_MAX
    }
}

/// The longest row any rate admits: the buffers' row.
pub const ROW_BYTES_MAX: usize = row_bytes_max(wire::STREAM_BUDGET_BAUD);

pub const STREAM_PAYLOAD_MAX: usize =
    STREAM_HDR + STREAM_SAMPLES_MAX * ROW_BYTES_MAX + STREAM_TRAILER;

/// [`row_bytes_max`] by `BaudRate` index, as the identity block publishes it.
pub const ROW_BYTES_PER_RATE: [u8; osc_protocol::table::TEL_ROW_RATES] = {
    let mut r = [0; osc_protocol::table::TEL_ROW_RATES];
    let mut i = 0;
    while i < r.len() {
        if let Some(b) = BaudRate::from_idx(i as u8) {
            r[i] = row_bytes_max(b) as u8;
        }
        i += 1;
    }
    r
};

/// A burst under `mask` arms at `baud`: no reserved bit, a field, and a
/// row the rate's capacity carries.
pub const fn admits(mask: u16, baud: BaudRate) -> bool {
    mask & !MASK_ALL == 0 && mask != 0 && row_bytes(mask) <= row_bytes_max(baud)
}

/// One fast tick's streamable primitives, in device counts. `pos`,
/// `current_raw`, `current_trough`, `vmotor_a`, `vmotor_b`, `vbus_raw`,
/// `ntc_raw` are the tick's raw ADC frame; `duty` is the signed PWM compare
/// count the bridge applied; `current` (signed, bias-subtracted,
/// settle-gained, held - the fitter's domain, matching `i_hat_counts`),
/// `vdiff`, `vbus` and `pos_lin_q4` (the linearized pot, Q4) are kernel
/// conclusions, streamable to validate them against host re-derivations.
/// `window_valid` (this tick's drive window met the current floor)
/// travels in the frame's `valid` bitmap, not per sample; `tick` is
/// `sample_tick`'s low half, the frame's stamp when the row is its first.
///
/// The twelve words sit in `tel_mask` bit order at `2 * bit` (asserted
/// below): the encoder reads them by offset.
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct TelSample {
    pub pos: u16,
    pub current: i16,
    pub current_trough: u16,
    pub duty: i16,
    pub vdiff: i16,
    pub vbus: u16,
    pub current_raw: u16,
    pub vmotor_a: u16,
    pub vmotor_b: u16,
    pub vbus_raw: u16,
    pub ntc_raw: u16,
    pub pos_lin_q4: u16,
    pub tick: u16,
    pub window_valid: bool,
    /// Kernel fault mask nonzero this tick. The first row's is the frame's
    /// INST ALERT (the fault contract), the batch's OR the trailer's tail.
    pub fault: bool,
}

const _: () = {
    let at = [
        offset_of!(TelSample, pos),
        offset_of!(TelSample, current),
        offset_of!(TelSample, current_trough),
        offset_of!(TelSample, duty),
        offset_of!(TelSample, vdiff),
        offset_of!(TelSample, vbus),
        offset_of!(TelSample, current_raw),
        offset_of!(TelSample, vmotor_a),
        offset_of!(TelSample, vmotor_b),
        offset_of!(TelSample, vbus_raw),
        offset_of!(TelSample, ntc_raw),
        offset_of!(TelSample, pos_lin_q4),
    ];
    let mut b = 0;
    while b < FIELDS {
        assert!(at[b] == WORD_BYTES * b);
        b += 1;
    }
};

impl TelSample {
    /// The word at byte offset `at`, which a [`RowPlan`] keeps below
    /// `WORD_BYTES * FIELDS` and even.
    #[inline(always)]
    fn word(&self, at: u8) -> u16 {
        // SAFETY: `repr(C)` puts the twelve 2-byte words first in bit order
        // (asserted above); every offset comes from a `RowPlan`, built only
        // from bit indices below `FIELDS`.
        unsafe {
            (self as *const Self)
                .cast::<u8>()
                .add(at as usize)
                .cast::<u16>()
                .read()
        }
    }
}

/// The kernel's per-fast-tick stream hook; core never names the transport.
pub trait TelStream {
    /// The fields this tick's row carries, 0 when no row is owed: the
    /// kernel builds the sample only when nonzero.
    fn row_mask(&mut self) -> u16;
    fn on_tick(&mut self, sample: &TelSample);
}

/// Stream absent (boards without a burst producer, tests).
impl TelStream for () {
    fn row_mask(&mut self) -> u16 {
        0
    }
    fn on_tick(&mut self, _sample: &TelSample) {}
}

/// A mask's row as the encoder walks it, built once per arm: the sample
/// offsets of the 16-bit slots, then of the 12-bit pairs. Word-aligned so
/// the kernel's pickup copies it in four words.
#[repr(C, align(4))]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct RowPlan {
    at: [u8; FIELDS],
    words: u8,
    /// `words` plus two per pair: the end of `at`'s live prefix.
    end: u8,
    len: u8,
}

impl RowPlan {
    /// No field: the encoder writes nothing.
    pub const EMPTY: Self = Self {
        at: [0; FIELDS],
        words: 0,
        end: 0,
        len: 0,
    };

    /// None when `mask` sets a reserved bit or its row outgrows the
    /// buffers' row.
    pub const fn new(mask: u16) -> Option<Self> {
        let l = Layout::new(mask);
        if mask & !MASK_ALL != 0 || l.row_bytes() > ROW_BYTES_MAX {
            return None;
        }
        let end = l.words as usize + 2 * l.pairs as usize;
        let mut at = [0; FIELDS];
        let mut i = 0;
        while i < end && i < FIELDS {
            at[i] = l.order[i] * WORD_BYTES as u8;
            i += 1;
        }
        Some(Self {
            at,
            words: l.words,
            end: end as u8,
            len: l.row_bytes() as u8,
        })
    }

    pub const fn len(&self) -> usize {
        self.len as usize
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Serialize `s`'s planned fields into `row`; a `row` shorter than
    /// [`Self::len`] stays untouched.
    #[inline(always)]
    pub fn encode(&self, s: &TelSample, row: &mut [u8]) {
        if row.len() < self.len() {
            return;
        }
        let mut d = row.as_mut_ptr();
        // SAFETY: `new` keeps `words <= end <= FIELDS`, and the slots it
        // plans fill `2 * words + 3 * pairs` = `self.len()` bytes, which
        // `row` holds.
        unsafe {
            let mut at = self.at.as_ptr();
            let words = at.add(self.words as usize);
            let end = at.add(self.end as usize);
            while at < words {
                let [lo, hi] = s.word(*at).to_le_bytes();
                d.write(lo);
                d.add(1).write(hi);
                d = d.add(WORD_BYTES);
                at = at.add(1);
            }
            // `a | b << 12` as three bytes; bits past 12 never reach them
            const SPLIT: u32 = NARROW_BITS - u8::BITS;
            while at < end {
                let (a, b) = (s.word(*at), s.word(*at.add(1)));
                d.write(a as u8);
                d.add(1)
                    .write(((a & NARROW_MASK) >> u8::BITS) as u8 | (b << SPLIT) as u8);
                d.add(2).write((b >> SPLIT) as u8);
                d = d.add(PAIR_BYTES);
                at = at.add(2);
            }
        }
    }
}

/// Serialize the stream head: everything row 0 knows. Order-free vs the
/// rows: it never touches bytes past [`STREAM_HDR`].
pub fn encode_stream_head(
    stream_seq: u8,
    last: bool,
    tick: u16,
    buf: &mut [u8; STREAM_PAYLOAD_MAX],
) {
    buf[0] = stream_seq;
    buf[1] = if last { FLAG_LAST } else { 0 };
    buf[2..4].copy_from_slice(&tick.to_le_bytes());
}

/// Serialize the stream trailer at the front of `out`, which follows the
/// last row; an `out` shorter than [`STREAM_TRAILER`] stays untouched.
#[inline(always)]
pub fn encode_stream_trailer(valid: u32, alert: bool, out: &mut [u8]) {
    if let Some(t) = out.first_chunk_mut::<STREAM_TRAILER>() {
        let [v0, v1, v2, v3] = valid.to_le_bytes();
        let tail = if alert { STREAM_TAIL_ALERT } else { 0 };
        *t = [v0, v1, v2, v3, tail, 0];
    }
}

/// Serialize one stream payload into `buf`, returning its length; 0 for a
/// mask with no [`RowPlan`]. `samples` beyond [`STREAM_SAMPLES_MAX`]
/// truncate. The reference the driver's incremental encoder is pinned to.
pub fn encode_stream(
    mask: u16,
    stream_seq: u8,
    last: bool,
    samples: &[TelSample],
    buf: &mut [u8; STREAM_PAYLOAD_MAX],
) -> usize {
    let Some(plan) = RowPlan::new(mask) else {
        return 0;
    };
    let mut valid: u32 = 0;
    let mut alert = false;
    let mut n = STREAM_HDR;
    for (i, s) in samples.iter().take(STREAM_SAMPLES_MAX).enumerate() {
        valid |= (s.window_valid as u32) << i;
        alert |= s.fault;
        if let Some(row) = buf.get_mut(n..) {
            plan.encode(s, row);
        }
        n += plan.len();
    }
    let tick = samples.first().map_or(0, |s| s.tick);
    encode_stream_head(stream_seq, last, tick, buf);
    if let Some(out) = buf.get_mut(n..) {
        encode_stream_trailer(valid, alert, out);
    }
    n + STREAM_TRAILER
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::tel::decode_row;

    /// The six raw fields: the soak and capture default.
    const MASK_RAW: u16 =
        BIT_POS | BIT_CURRENT_RAW | BIT_CURRENT_TROUGH | BIT_DUTY | BIT_VMOTOR_A | BIT_VMOTOR_B;
    /// The ladder mask: pos, current, duty, vdiff.
    const MASK_LADDER: u16 = 0x1B;

    fn sample(i: usize) -> TelSample {
        TelSample {
            pos: 0x100 + i as u16,
            current: -(i as i16) - 1,
            current_trough: 0xB00 + i as u16,
            duty: -1200 + 100 * i as i16,
            vdiff: -300 - i as i16,
            vbus: 1800 + i as u16,
            current_raw: 0x0100 + i as u16,
            vmotor_a: 0x0A00 + i as u16,
            vmotor_b: 0x0B00 + i as u16,
            vbus_raw: 0x0C00 + i as u16,
            ntc_raw: 0x0D00 + i as u16,
            pos_lin_q4: 0x0E00 + i as u16,
            tick: 0x1230 + i as u16,
            window_valid: i.is_multiple_of(2),
            fault: i.is_multiple_of(3),
        }
    }

    fn words(s: &TelSample) -> [u16; FIELDS] {
        core::array::from_fn(|b| s.word((WORD_BYTES * b) as u8))
    }

    #[test]
    fn buffers_hold_the_widest_admitted_row() {
        assert_eq!(ROW_BYTES_MAX, 11);
        assert_eq!(STREAM_PAYLOAD_MAX, STREAM_HDR + 16 * 11 + STREAM_TRAILER);
        assert!(STREAM_PAYLOAD_MAX <= wire::MAX_PAYLOAD as usize);
    }

    /// Admission by row bytes at the current rate: the six raw fields (9 B)
    /// at 3 M only; 2 M carries 8 B rows, 1 M 3 B, 0.5 M none; 3 M stops at
    /// the encoder's 11 B short of the wire's 13.
    #[test]
    fn admission_is_by_row_bytes_at_the_rate() {
        assert_eq!(
            [
                BaudRate::B500000,
                BaudRate::B1000000,
                BaudRate::B2000000,
                BaudRate::B3000000
            ]
            .map(row_bytes_max),
            [0, 3, 8, 11]
        );
        assert!(admits(MASK_RAW, BaudRate::B3000000));
        assert!(!admits(MASK_RAW, BaudRate::B2000000));
        assert!(admits(MASK_LADDER, BaudRate::B2000000));
        assert!(!admits(MASK_LADDER, BaudRate::B1000000));
        assert!(admits(BIT_POS | BIT_CURRENT_RAW, BaudRate::B1000000));
        assert!(!admits(BIT_POS | BIT_CURRENT, BaudRate::B1000000));
        assert!(!admits(BIT_POS, BaudRate::B500000));
        assert!(!admits(0, BaudRate::B3000000));
        assert!(!admits(1 << 12 | BIT_POS, BaudRate::B3000000));
        // six raw fields and current: 11 B; eight 12-bit fields: 12 B
        assert!(admits(MASK_RAW | BIT_CURRENT, BaudRate::B3000000));
        assert!(!admits(MASK_ALL & !WIDE, BaudRate::B3000000));
    }

    /// The host detects the burst end through osc-protocol's mirror of the
    /// LAST bit; the two consts must never drift.
    #[test]
    fn flag_last_matches_the_protocol_mirror() {
        use osc_protocol::wire::{Inst, ResultCode, STREAM_FLAG_LAST, stream_last};
        assert_eq!(FLAG_LAST, STREAM_FLAG_LAST);
        let mut buf = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(BIT_POS, 0, true, &[sample(0)], &mut buf);
        let inst = Inst::status(ResultCode::Stream, false);
        assert!(stream_last(inst, buf[..n].into()));
        let n = encode_stream(BIT_POS, 0, false, &[sample(0)], &mut buf);
        assert!(!stream_last(inst, buf[..n].into()));
    }

    /// The protocol's reader takes back what the encoder wrote: the row
    /// count from the length, the stamp, the bitmap and the fault OR.
    #[test]
    fn encoded_frames_parse_through_the_protocol_reader() {
        use osc_protocol::tel::StreamFrame;
        let samples: heapless::Vec<TelSample, 16> = (1..6).map(sample).collect();
        let mut buf = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(MASK_RAW, 9, true, &samples, &mut buf);
        let f = StreamFrame::parse(MASK_RAW, &buf[..n]).expect("a frame");
        assert_eq!((f.seq, f.last, f.tick, f.len()), (9, true, 0x1231, 5));
        assert_eq!((f.valid, f.alert), (0b01010, true));
        let n = encode_stream(MASK_RAW, 9, false, &samples[..2], &mut buf);
        let f = StreamFrame::parse(MASK_RAW, &buf[..n]).expect("a frame");
        assert_eq!((f.last, f.len(), f.alert), (false, 2, false));
    }

    #[test]
    fn plans_refuse_reserved_bits() {
        assert!(RowPlan::new(1 << 12).is_none());
        assert!(RowPlan::new(MASK_ALL).is_none());
        assert_eq!(RowPlan::new(0).map(|p| p.len()), Some(0));
        assert_eq!(RowPlan::new(MASK_RAW).map(|p| p.len()), Some(9));
    }

    #[test]
    fn encode_golden_six_raw_fields_full_batch() {
        let samples: heapless::Vec<TelSample, 16> = (0..16).map(sample).collect();
        let mut buf = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(MASK_RAW, 0x42, false, &samples, &mut buf);
        assert_eq!(n, STREAM_HDR + 16 * 9 + STREAM_TRAILER);
        // head: seq, flags (not LAST), row 0's tick
        assert_eq!(buf[..4], [0x42, 0x00, 0x30, 0x12]);
        // row 0: (pos 0x100, trough 0xB00) (duty -1200 = 0xB50, raw 0x100)
        // (va 0xA00, vb 0xB00), each pair a | b << 12
        assert_eq!(
            buf[4..13],
            [0x00, 0x01, 0xB0, 0x50, 0x0B, 0x10, 0x00, 0x0A, 0xB0]
        );
        // row 15: duty +300 = 0x12C
        assert_eq!(
            buf[139..148],
            [0x0F, 0xF1, 0xB0, 0x2C, 0xF1, 0x10, 0x0F, 0xFA, 0xB0]
        );
        // trailer: valid = even rows, tail ALERT (rows 0, 3, ... faulted)
        assert_eq!(
            buf[148..154],
            [0x55, 0x55, 0x00, 0x00, STREAM_TAIL_ALERT, 0]
        );
    }

    #[test]
    fn encode_mixed_widths_words_first() {
        // ladder: current, vdiff wide; pos, duty pair
        let mut buf = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(MASK_LADDER, 7, false, &[sample(1)], &mut buf);
        assert_eq!(n, STREAM_HDR + 7 + STREAM_TRAILER);
        assert_eq!(buf[..4], [7, 0, 0x31, 0x12]);
        // current -2, vdiff -301, then pos 0x101 | duty -1100 (0xBB4) << 12
        assert_eq!(buf[4..11], [0xFE, 0xFF, 0xD3, 0xFE, 0x01, 0x41, 0xBB]);
        // an odd row: not valid, no fault
        assert_eq!(buf[11..17], [0; 6]);
    }

    #[test]
    fn encode_partial_last_frame() {
        let samples = [sample(0), sample(1), sample(2)];
        let mut buf = [0xEEu8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(BIT_POS, 0xFF, true, &samples, &mut buf);
        assert_eq!(n, STREAM_HDR + 3 * 2 + STREAM_TRAILER);
        assert_eq!(buf[..4], [0xFF, FLAG_LAST, 0x30, 0x12]);
        assert_eq!(buf[4..10], [0x00, 0x01, 0x01, 0x01, 0x02, 0x01]);
        assert_eq!(buf[10..16], [0x05, 0, 0, 0, STREAM_TAIL_ALERT, 0]);
        assert!(buf[16..].iter().all(|&b| b == 0xEE));
    }

    /// Every mask the buffers hold decodes back through the protocol's
    /// reference decoder, field for field.
    #[test]
    fn round_trip_every_plannable_mask() {
        let s = sample(5);
        let want = words(&s);
        for mask in 0..=MASK_ALL {
            let Some(plan) = RowPlan::new(mask) else {
                assert!(row_bytes(mask) > ROW_BYTES_MAX);
                continue;
            };
            let mut row = [0u8; ROW_BYTES_MAX];
            plan.encode(&s, &mut row);
            let got = decode_row(mask, &row[..plan.len()]).expect("decodes");
            for b in 0..FIELDS {
                let w = if mask & (1 << b) == 0 { 0 } else { want[b] };
                assert_eq!(got[b], w, "mask {mask:#x} field {b}");
            }
        }
    }
}
