//! Reference osc-CRC-16 (`docs/osc-native-protocol.md` §3.2). Host/test only;
//! servos use the SPI CRC engine. Bitwise, no table — smallest code wins.
//!
//! Flavor: CRC-16/BUYPASS (poly `0x8005`, init `0x0000`, no reflection, no
//! output XOR) computed with each 2-byte pair swapped — bit-exact with what
//! the V006 SPI unit produces when DMA feeds 16-bit halfwords from the byte
//! buffer. The covered span is always even (PAD invariant, §3.1).
#![cfg(feature = "software-crc")]

const POLY: u16 = 0x8005;

#[inline]
fn update(mut crc: u16, byte: u8) -> u16 {
    crc ^= (byte as u16) << 8;
    for _ in 0..8 {
        crc = if crc & 0x8000 != 0 {
            (crc << 1) ^ POLY
        } else {
            crc << 1
        };
    }
    crc
}

/// osc-CRC-16 over `covered` (prefix byte included by the caller). Length must
/// be even.
#[inline]
pub fn osc_crc(covered: &[u8]) -> u16 {
    osc_crc_continue(0, covered)
}

/// Accumulate `chunk` into a running CRC. `chunk` length must be even, so pairs
/// never straddle calls — mirrors the hardware engine spanning DMA arms.
/// `osc_crc(x) == osc_crc_continue(0, x)`.
pub fn osc_crc_continue(mut crc: u16, chunk: &[u8]) -> u16 {
    debug_assert!(chunk.len().is_multiple_of(2), "osc-CRC covers an even span");
    let (pairs, _) = chunk.as_chunks::<2>();
    for &[lo, hi] in pairs {
        crc = update(crc, hi);
        crc = update(crc, lo);
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spec_vectors() {
        assert_eq!(osc_crc(&[0x00, 0x01, 0x03, 0x10]), 0x740A);
        assert_eq!(
            osc_crc(&[0x00, 0x05, 0x07, 0x30, 0x80, 0x01, 0x2C, 0x01]),
            0xC9AE
        );
        assert_eq!(
            osc_crc(&[0x00, 0x03, 0x07, 0x20, 0x00, 0x02, 0x08, 0x00]),
            0x1970
        );
        assert_eq!(
            osc_crc(&[0x00, 0x02, 0x07, 0x32, 0x00, 0x01, 0xAA, 0x00]),
            0x46A8
        );
    }

    #[test]
    fn split_at_even_boundary_matches_whole() {
        let v = [0x00, 0x05, 0x07, 0x30, 0x80, 0x01, 0x2C, 0x01];
        let whole = osc_crc(&v);
        for split in [0, 2, 4, 6, 8] {
            let (a, b) = v.split_at(split);
            let crc = osc_crc_continue(osc_crc_continue(0, a), b);
            assert_eq!(crc, whole, "split at {split}");
        }
    }
}
