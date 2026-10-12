//! sec 5.6 TEL rows (osc-servo field set): the field a `tel_mask` bit
//! selects, the width it rides at, and the row layout servo and host share.

use crate::wire;

pub const BIT_POS: u16 = 1 << 0;
pub const BIT_CURRENT: u16 = 1 << 1;
pub const BIT_CURRENT_TROUGH: u16 = 1 << 2;
pub const BIT_DUTY: u16 = 1 << 3;
pub const BIT_VDIFF: u16 = 1 << 4;
pub const BIT_VBUS: u16 = 1 << 5;
pub const BIT_CURRENT_RAW: u16 = 1 << 6;
pub const BIT_VMOTOR_A: u16 = 1 << 7;
pub const BIT_VMOTOR_B: u16 = 1 << 8;
pub const BIT_VBUS_RAW: u16 = 1 << 9;
pub const BIT_NTC_RAW: u16 = 1 << 10;
pub const BIT_POS_LIN: u16 = 1 << 11;
pub const MASK_ALL: u16 = 0xFFF;
pub const FIELDS: usize = 12;

/// The fields that ride in 16 bits: the kernel's conclusions. The raw ADC
/// codes and the duty ride in 12.
pub const WIDE: u16 = BIT_CURRENT | BIT_VDIFF | BIT_VBUS | BIT_POS_LIN;

/// Two's-complement fields; a 12-bit one sign-extends from bit 11.
pub const SIGNED: u16 = BIT_CURRENT | BIT_DUTY | BIT_VDIFF;

pub const NARROW_BITS: u32 = 12;
pub const NARROW_MASK: u16 = (1 << NARROW_BITS) - 1;

/// The largest duty magnitude, in PWM compare counts, a 12-bit word holds.
pub const DUTY_COUNTS_MAX: u16 = NARROW_MASK >> 1;

/// Bytes of one 16-bit slot and of one packed pair of 12-bit fields.
pub const WORD_BYTES: usize = 2;
pub const PAIR_BYTES: usize = 3;

/// A row in wire order: `words` 16-bit LE slots, then `pairs` 3-byte LE
/// pairs `a | b << 12`. `order` lists the field bit indices slot by slot.
/// The slots carry the wide fields and, when the mask selects an odd count
/// of 12-bit fields, the lowest of them (holding its value), each in bit
/// order; the other 12-bit fields pair up in bit order.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Layout {
    pub order: [u8; FIELDS],
    pub words: u8,
    pub pairs: u8,
}

impl Layout {
    pub const fn new(mask: u16) -> Self {
        let m = mask & MASK_ALL;
        let narrow = m & !WIDE;
        let lone = if narrow.count_ones() % 2 == 1 {
            narrow.isolate_lowest_one()
        } else {
            0
        };
        let (order, words) = push_bits([0; FIELDS], 0, (m & WIDE) | lone);
        let (order, n) = push_bits(order, words, narrow & !lone);
        Self {
            order,
            words: words as u8,
            pairs: ((n - words) / 2) as u8,
        }
    }

    pub const fn row_bytes(&self) -> usize {
        WORD_BYTES * self.words as usize + PAIR_BYTES * self.pairs as usize
    }
}

const fn push_bits(mut order: [u8; FIELDS], mut n: usize, bits: u16) -> ([u8; FIELDS], usize) {
    let bits = bits as u32;
    let mut b = 0;
    while bits >> b != 0 && n < FIELDS {
        if bits >> b & 1 != 0 {
            order[n] = b as u8;
            n += 1;
        }
        b += 1;
    }
    (order, n)
}

/// Bytes one row of `mask` takes on the wire: [`Layout::row_bytes`]
/// without the walk.
pub const fn row_bytes(mask: u16) -> usize {
    let m = mask & MASK_ALL;
    let narrow = (m & !WIDE).count_ones() as usize;
    WORD_BYTES * ((m & WIDE).count_ones() as usize + narrow % 2) + PAIR_BYTES * (narrow / 2)
}

/// Payload bytes of a frame of `rows` rows of `mask`.
pub const fn frame_len(mask: u16, rows: usize) -> usize {
    wire::STREAM_HDR + rows * row_bytes(mask) + wire::STREAM_TRAILER
}

/// The most rows a `mask` frame carries: what the payload and the `valid`
/// bitmap hold; 0 for a mask with no row.
pub const fn rows_max(mask: u16) -> usize {
    let row = row_bytes(mask);
    if row == 0 {
        return 0;
    }
    let fit = (wire::MAX_PAYLOAD as usize - wire::STREAM_HDR - wire::STREAM_TRAILER) / row;
    if fit < wire::STREAM_ROWS_MAX {
        fit
    } else {
        wire::STREAM_ROWS_MAX
    }
}

/// One TEL frame payload (sec 5.6), split: the head, the rows, the trailer.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StreamFrame<'a> {
    pub seq: u8,
    pub last: bool,
    /// The first row's `sample_tick`, low 16 bits.
    pub tick: u16,
    /// Bit i: row i measured a fresh drive window.
    pub valid: u32,
    /// Some row of the frame saw a fault.
    pub alert: bool,
    rows: &'a [u8],
    row_bytes: usize,
}

impl<'a> StreamFrame<'a> {
    /// None when `payload` cannot be a `mask` frame: shorter than head and
    /// trailer, a ragged row remainder, no row, or over [`rows_max`].
    pub fn parse(mask: u16, payload: &'a [u8]) -> Option<Self> {
        let row_bytes = row_bytes(mask);
        let (head, rest) = payload.split_first_chunk::<{ wire::STREAM_HDR }>()?;
        let (rows, trailer) = rest.split_last_chunk::<{ wire::STREAM_TRAILER }>()?;
        let n = rows.len().checked_div(row_bytes)?;
        if n == 0 || n > rows_max(mask) || n * row_bytes != rows.len() {
            return None;
        }
        let [seq, flags, t0, t1] = *head;
        let [v0, v1, v2, v3, tail, _] = *trailer;
        Some(Self {
            seq,
            last: flags & wire::STREAM_FLAG_LAST != 0,
            tick: u16::from_le_bytes([t0, t1]),
            valid: u32::from_le_bytes([v0, v1, v2, v3]),
            alert: tail & wire::STREAM_TAIL_ALERT != 0,
            rows,
            row_bytes,
        })
    }

    pub fn len(&self) -> usize {
        self.rows.len() / self.row_bytes
    }

    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    /// The packed rows in order, each [`decode_row`]'s input.
    pub fn rows(&self) -> core::slice::ChunksExact<'a, u8> {
        self.rows.chunks_exact(self.row_bytes)
    }
}

/// One row's fields by bit index, 0 where unselected: 12-bit fields
/// masked, the signed ones sign-extended. None when `row` is short.
pub fn decode_row(mask: u16, row: &[u8]) -> Option<[u16; FIELDS]> {
    let l = Layout::new(mask);
    let (words, pairs) = l.order.split_at_checked(l.words as usize)?;
    let (head, tail) = row.split_at_checked(WORD_BYTES * words.len())?;
    let tail = tail.get(..PAIR_BYTES * l.pairs as usize)?;
    let mut out = [0u16; FIELDS];
    let mut put = |f: u8, v: u16| {
        if let Some(o) = out.get_mut(f as usize) {
            *o = v;
        }
    };
    for (&b, &f) in head.as_chunks::<WORD_BYTES>().0.iter().zip(words) {
        put(f, u16::from_le_bytes(b));
    }
    let pairs = pairs.as_chunks::<2>().0;
    for (&[b0, b1, b2], &[fa, fb]) in tail.as_chunks::<PAIR_BYTES>().0.iter().zip(pairs) {
        let v = u32::from_le_bytes([b0, b1, b2, 0]);
        put(fa, v as u16);
        put(fb, (v >> NARROW_BITS) as u16);
    }
    for (f, o) in out.iter_mut().enumerate() {
        let bit = 1 << f;
        if mask & !WIDE & bit != 0 {
            let v = *o & NARROW_MASK;
            let shift = 16 - NARROW_BITS;
            *o = if SIGNED & bit != 0 {
                ((v << shift) as i16 >> shift) as u16
            } else {
                v
            };
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIX_RAW: u16 =
        BIT_POS | BIT_CURRENT_TROUGH | BIT_DUTY | BIT_CURRENT_RAW | BIT_VMOTOR_A | BIT_VMOTOR_B;

    #[test]
    fn row_bytes_follow_the_widths() {
        assert_eq!(row_bytes(0), 0);
        assert_eq!(row_bytes(BIT_POS), 2);
        assert_eq!(row_bytes(BIT_POS | BIT_CURRENT_RAW), 3);
        assert_eq!(row_bytes(BIT_CURRENT), 2);
        assert_eq!(row_bytes(SIX_RAW), 9);
        assert_eq!(frame_len(SIX_RAW, wire::STREAM_SAMPLES_MAX), 154);
        assert_eq!(row_bytes(0x1B), 7);
        assert_eq!(row_bytes(MASK_ALL), 4 * 2 + 4 * 3);
        assert_eq!(row_bytes(1 << 12), 0);
        for m in 0..=MASK_ALL {
            assert_eq!(row_bytes(m), Layout::new(m).row_bytes(), "{m:#x}");
        }
    }

    #[test]
    fn layout_puts_words_first_then_pairs() {
        let l = Layout::new(BIT_POS | BIT_CURRENT | BIT_DUTY | BIT_VDIFF | BIT_NTC_RAW);
        assert_eq!((l.words, l.pairs), (3, 1));
        assert_eq!(l.order[..5], [0, 1, 4, 3, 10]);
        let l = Layout::new(SIX_RAW);
        assert_eq!((l.words, l.pairs), (0, 3));
        assert_eq!(l.order[..6], [0, 2, 3, 6, 7, 8]);
    }

    /// Head [seq][flags][tick u16], rows, trailer [valid u32][tail][0]: the
    /// count comes from the length alone.
    #[test]
    fn stream_frame_takes_its_row_count_from_the_length() {
        let p = [
            0x42,
            wire::STREAM_FLAG_LAST,
            0x34,
            0x12,
            0x01,
            0x00,
            0x02,
            0x00,
            0x03,
            0x00,
            0x05,
            0x00,
            0x00,
            0x80,
            wire::STREAM_TAIL_ALERT,
            0,
        ];
        let f = StreamFrame::parse(BIT_POS, &p).expect("a frame");
        assert_eq!((f.seq, f.last, f.tick), (0x42, true, 0x1234));
        assert_eq!((f.valid, f.alert, f.len()), (0x8000_0005, true, 3));
        let mut rows = f.rows();
        assert_eq!(rows.next(), Some(&[1, 0][..]));
        assert_eq!(rows.nth(1), Some(&[3, 0][..]));
        assert_eq!(rows.next(), None);
        let f = StreamFrame::parse(BIT_POS | BIT_CURRENT, &p);
        assert!(f.is_none(), "a ragged remainder is not a frame");
    }

    #[test]
    fn stream_frame_rejects_empty_and_oversized_frames() {
        let empty = [0u8; wire::STREAM_HDR + wire::STREAM_TRAILER];
        assert!(StreamFrame::parse(BIT_POS, &empty).is_none());
        assert!(StreamFrame::parse(BIT_POS, &empty[1..]).is_none());
        assert!(StreamFrame::parse(0, &[0; 12]).is_none());
        assert_eq!(rows_max(BIT_POS), wire::STREAM_ROWS_MAX);
        assert_eq!(rows_max(SIX_RAW), 26);
        assert_eq!(rows_max(0), 0);
        let buf = [0u8; frame_len(BIT_POS, wire::STREAM_ROWS_MAX + 1)];
        let max = &buf[..frame_len(BIT_POS, wire::STREAM_ROWS_MAX)];
        assert_eq!(StreamFrame::parse(BIT_POS, max).map(|f| f.len()), Some(32));
        assert!(StreamFrame::parse(BIT_POS, &buf).is_none());
    }

    #[test]
    fn decode_reads_words_and_pairs() {
        let mask = BIT_POS | BIT_CURRENT | BIT_DUTY | BIT_VDIFF | BIT_NTC_RAW;
        // pos 0x123 (lone, a word), current -2, vdiff -300, duty -5 | ntc 0xABC << 12
        let row = [0x23, 0x01, 0xFE, 0xFF, 0xD4, 0xFE, 0xFB, 0xCF, 0xAB, 0xEE];
        let f = decode_row(mask, &row).expect("decodes");
        assert_eq!(f[0], 0x123);
        assert_eq!(f[1] as i16, -2);
        assert_eq!(f[3] as i16, -5);
        assert_eq!(f[4] as i16, -300);
        assert_eq!(f[10], 0xABC);
        assert_eq!(f[2], 0);
        assert!(decode_row(mask, &row[..8]).is_none());
    }
}
