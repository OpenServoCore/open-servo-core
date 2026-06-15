//! Header-sync stage: KMP-style match of `FF FF FD 00`.

use crate::crc::CrcUmts;
use crate::wire::HEADER;

pub(crate) struct SyncStage {
    matched: u8,
}

impl SyncStage {
    pub(crate) fn new() -> Self {
        Self { matched: 0 }
    }

    /// KMP backoff (`f[1]=0, f[2]=1, f[3]=0`) locks `FF FF FF FD 00` onto the
    /// embedded header at offset 1. Seeds the CRC on match.
    pub(crate) fn feed<CRC: CrcUmts>(&mut self, b: u8, crc: &mut CRC) -> Option<()> {
        let m = self.matched as usize;
        if b == HEADER[m] {
            let new_m = m + 1;
            if new_m == HEADER.len() {
                crc.update(&HEADER);
                self.matched = 0;
                return Some(());
            }
            self.matched = new_m as u8;
        } else {
            match m {
                2 if b == HEADER[0] => {}
                3 if b == HEADER[0] => self.matched = 1,
                _ => self.matched = if b == HEADER[0] { 1 } else { 0 },
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::SoftwareCrcUmts;

    type Crc = SoftwareCrcUmts;

    fn feed_all(s: &mut SyncStage, crc: &mut Crc, bytes: &[u8]) -> Option<usize> {
        for (i, &b) in bytes.iter().enumerate() {
            if s.feed(b, crc).is_some() {
                return Some(i + 1);
            }
        }
        None
    }

    #[test]
    fn exact_preamble_completes_on_fourth_byte() {
        let (mut s, mut c) = (SyncStage::new(), Crc::new());
        assert_eq!(feed_all(&mut s, &mut c, &[0xFF, 0xFF, 0xFD, 0x00]), Some(4));
    }

    #[test]
    fn noise_then_preamble() {
        let (mut s, mut c) = (SyncStage::new(), Crc::new());
        let bytes = [0x00, 0x12, 0xFF, 0x34, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0x00];
        assert_eq!(feed_all(&mut s, &mut c, &bytes), Some(bytes.len()));
    }

    #[test]
    fn embedded_after_extra_ff() {
        let (mut s, mut c) = (SyncStage::new(), Crc::new());
        assert_eq!(
            feed_all(&mut s, &mut c, &[0xFF, 0xFF, 0xFF, 0xFD, 0x00]),
            Some(5)
        );
    }

    #[test]
    fn falls_back_on_ff_after_fd() {
        let (mut s, mut c) = (SyncStage::new(), Crc::new());
        assert_eq!(
            feed_all(&mut s, &mut c, &[0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0x00]),
            Some(7)
        );
    }

    #[test]
    fn partial_state_preserved_across_calls() {
        let (mut s, mut c) = (SyncStage::new(), Crc::new());
        assert!(s.feed(0xFF, &mut c).is_none());
        assert!(s.feed(0xFF, &mut c).is_none());
        assert!(s.feed(0xFD, &mut c).is_none());
        assert_eq!(s.feed(0x00, &mut c), Some(()));
    }

    #[test]
    fn seeds_crc_with_header_on_completion() {
        let (mut s, mut c) = (SyncStage::new(), Crc::new());
        let _ = feed_all(&mut s, &mut c, &[0xFF, 0xFF, 0xFD, 0x00]);

        let mut expected = Crc::new();
        expected.update(&HEADER);
        assert_eq!(c.finalize(), expected.finalize());
    }

    #[test]
    fn matched_resets_after_completion() {
        let (mut s, mut c) = (SyncStage::new(), Crc::new());
        let _ = feed_all(&mut s, &mut c, &[0xFF, 0xFF, 0xFD, 0x00]);
        assert_eq!(s.matched, 0);
    }
}
