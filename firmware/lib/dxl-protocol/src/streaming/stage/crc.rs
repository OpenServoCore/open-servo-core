//! CRC verdict stage: reads 2 trailing wire bytes (LE), compares against
//! the running accumulator.

use crate::crc::CrcUmts;
use crate::streaming::event::CrcResult;

pub(crate) struct CrcStage {
    cursor: u8,
    expected: u16,
}

impl CrcStage {
    pub(crate) fn new() -> Self {
        Self {
            cursor: 0,
            expected: 0,
        }
    }

    /// CRC bytes still to consume before the verdict event (`Good` / `Bad`).
    pub(crate) fn remaining(&self) -> u16 {
        2u16.saturating_sub(self.cursor as u16)
    }

    /// Consume up to 2 expected-CRC bytes from `slice`; return `(consumed,
    /// Some(verdict))` once both have been seen, or `(consumed, None)` if
    /// the slice runs out mid-CRC. The verdict compares the parsed LE
    /// u16 against `crc.finalize()` — no CRC update happens here (the
    /// trailing CRC bytes are not part of the covered region).
    pub(crate) fn feed<CRC: CrcUmts>(
        &mut self,
        slice: &[u8],
        crc: &CRC,
    ) -> (usize, Option<CrcResult>) {
        let mut consumed = 0;
        for &b in slice {
            match self.cursor {
                0 => {
                    self.expected = b as u16;
                    self.cursor = 1;
                    consumed += 1;
                }
                _ => {
                    self.expected |= (b as u16) << 8;
                    consumed += 1;
                    let verdict = if self.expected == crc.finalize() {
                        CrcResult::Good
                    } else {
                        CrcResult::Bad
                    };
                    return (consumed, Some(verdict));
                }
            }
        }
        (consumed, None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::SoftwareCrcUmts;

    type Crc = SoftwareCrcUmts;

    fn folded(bytes: &[u8]) -> Crc {
        let mut c = Crc::new();
        c.update(bytes);
        c
    }

    #[test]
    fn matching_crc_emits_good() {
        let body = [0xDE, 0xAD, 0xBE, 0xEF];
        let c = folded(&body);
        let crc = c.finalize().to_le_bytes();
        let mut s = CrcStage::new();
        assert_eq!(s.feed(&crc, &c), (2, Some(CrcResult::Good)));
    }

    #[test]
    fn mismatching_crc_emits_bad() {
        let c = folded(&[1, 2, 3]);
        let crc = c.finalize().to_le_bytes();
        let mut s = CrcStage::new();
        assert_eq!(
            s.feed(&[crc[0], crc[1] ^ 0xFF], &c),
            (2, Some(CrcResult::Bad))
        );
    }

    #[test]
    fn first_byte_does_not_finalize() {
        let c = folded(&[0xAA]);
        let mut s = CrcStage::new();
        assert_eq!(s.feed(&[0x12], &c), (1, None));
    }

    #[test]
    fn accumulator_is_not_consumed() {
        let c = folded(&[1, 2, 3, 4]);
        let before = c.finalize();
        let mut s = CrcStage::new();
        let _ = s.feed(&before.to_le_bytes(), &c);
        assert_eq!(c.finalize(), before);
    }

    #[test]
    fn slice_stops_at_verdict_boundary() {
        let c = folded(&[0xAA]);
        let crc = c.finalize().to_le_bytes();
        let mut s = CrcStage::new();
        // Pass 3 bytes — only 2 should be consumed.
        let (consumed, verdict) = s.feed(&[crc[0], crc[1], 0xCC], &c);
        assert_eq!(consumed, 2);
        assert_eq!(verdict, Some(CrcResult::Good));
    }
}
