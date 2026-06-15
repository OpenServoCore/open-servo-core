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

    pub(crate) fn feed<CRC: CrcUmts>(&mut self, b: u8, crc: &CRC) -> Option<CrcResult> {
        match self.cursor {
            0 => {
                self.expected = b as u16;
                self.cursor = 1;
                None
            }
            _ => {
                self.expected |= (b as u16) << 8;
                Some(if self.expected == crc.finalize() {
                    CrcResult::Good
                } else {
                    CrcResult::Bad
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;

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
        let crc = c.finalize();
        let mut s = CrcStage::new();
        assert!(s.feed(crc as u8, &c).is_none());
        assert_eq!(s.feed((crc >> 8) as u8, &c), Some(CrcResult::Good));
    }

    #[test]
    fn mismatching_crc_emits_bad() {
        let c = folded(&[1, 2, 3]);
        let crc = c.finalize();
        let mut s = CrcStage::new();
        assert!(s.feed(crc as u8, &c).is_none());
        assert_eq!(s.feed((crc >> 8) as u8 ^ 0xFF, &c), Some(CrcResult::Bad));
    }

    #[test]
    fn first_byte_does_not_finalize() {
        let c = folded(&[0xAA]);
        let mut s = CrcStage::new();
        assert!(s.feed(0x12, &c).is_none());
    }

    #[test]
    fn accumulator_is_not_consumed() {
        let c = folded(&[1, 2, 3, 4]);
        let before = c.finalize();
        let mut s = CrcStage::new();
        let _ = s.feed(before as u8, &c);
        let _ = s.feed((before >> 8) as u8, &c);
        assert_eq!(c.finalize(), before);
    }
}
