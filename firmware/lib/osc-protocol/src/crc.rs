//! Reference osc-CRC-16 (`docs/osc-native-protocol.md` §3.2). Host/test only;
//! servos use the SPI CRC engine. Bitwise, no table — smallest code wins.
//!
//! Flavor: **CRC-16/ARC** — poly `0x8005` reflected (`0xA001`), init `0x0000`,
//! reflected input and output, no output XOR. Check: `crc("123456789") =
//! 0xBB3D`. Byte-wise and prefix-free; a leading `0x00` is a no-op (init = 0),
//! which is what lets hardware receivers feed the anchor-inclusive ring span
//! in place (§3.2).

#[cfg(feature = "software-crc")]
const POLY_REFLECTED: u16 = 0xA001;

#[cfg(feature = "software-crc")]
#[inline]
fn update(mut crc: u16, byte: u8) -> u16 {
    crc ^= byte as u16;
    for _ in 0..8 {
        crc = if crc & 1 != 0 {
            (crc >> 1) ^ POLY_REFLECTED
        } else {
            crc >> 1
        };
    }
    crc
}

/// osc-CRC-16 over `covered` (the frame bytes `ID .. PAD`; an anchor-inclusive
/// span with its leading `0x00` yields the same value).
#[cfg(feature = "software-crc")]
#[inline]
pub fn osc_crc(covered: &[u8]) -> u16 {
    osc_crc_continue(0, covered)
}

/// Accumulate `chunk` into a running CRC — byte-wise, so chunks may split
/// anywhere (mirrors the hardware engine spanning DMA arms).
/// `osc_crc(x) == osc_crc_continue(0, x)`.
#[cfg(feature = "software-crc")]
pub fn osc_crc_continue(mut crc: u16, chunk: &[u8]) -> u16 {
    for &b in chunk {
        crc = update(crc, b);
    }
    crc
}

/// Reverse the bit order of a 16-bit value. The V006 SPI CRC unit's register
/// holds the bit-reversed checksum in LSB-first mode (§3.2, spike
/// `spi_crc_lsb_copy`): chip providers apply this once per frame. Three
/// parallel in-byte swap stages, then the byte swap — no table, no multiply.
#[inline]
pub fn bitrev16(v: u16) -> u16 {
    let mut x = v as u32;
    x = ((x & 0x5555) << 1) | ((x >> 1) & 0x5555);
    x = ((x & 0x3333) << 2) | ((x >> 2) & 0x3333);
    x = ((x & 0x0f0f) << 4) | ((x >> 4) & 0x0f0f);
    (x as u16).swap_bytes()
}

#[cfg(all(test, feature = "software-crc"))]
mod tests {
    use super::*;

    #[test]
    fn catalog_check_value() {
        assert_eq!(osc_crc(b"123456789"), 0xBB3D);
    }

    #[test]
    fn spec_vectors() {
        assert_eq!(osc_crc(&[0x01, 0x03, 0x10]), 0xFC50);
        assert_eq!(osc_crc(&[0x05, 0x07, 0x30, 0x80, 0x01, 0x2C, 0x01]), 0xB3B1);
        assert_eq!(osc_crc(&[0x03, 0x07, 0x20, 0x00, 0x02, 0x08, 0x00]), 0x7015);
        assert_eq!(osc_crc(&[0x02, 0x07, 0x32, 0x00, 0x01, 0xAA, 0x00]), 0xD334);
    }

    #[test]
    fn leading_zero_is_a_noop() {
        assert_eq!(
            osc_crc(&[0x00, 0x01, 0x03, 0x10]),
            osc_crc(&[0x01, 0x03, 0x10])
        );
    }

    #[test]
    fn split_anywhere_matches_whole() {
        let v = [0x05, 0x07, 0x30, 0x80, 0x01, 0x2C, 0x01];
        let whole = osc_crc(&v);
        for split in 0..=v.len() {
            let (a, b) = v.split_at(split);
            let crc = osc_crc_continue(osc_crc_continue(0, a), b);
            assert_eq!(crc, whole, "split at {split}");
        }
    }

    #[test]
    fn bitrev16_matches_silicon_fingerprint() {
        // spi_crc_lsb_copy: TCRCR("12345678") = 0xB93C = bitrev16(ARC 0x3C9D).
        assert_eq!(osc_crc(b"12345678"), 0x3C9D);
        assert_eq!(bitrev16(0x3C9D), 0xB93C);
        assert_eq!(bitrev16(bitrev16(0x1234)), 0x1234);
    }
}
