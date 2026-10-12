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
    wire::STREAM_HDR + rows * row_bytes(mask)
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
        assert_eq!(frame_len(SIX_RAW, wire::STREAM_SAMPLES_MAX), 148);
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
