use crc::{CRC_16_UMTS, Crc};

use super::crc::CrcUmts;

const ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

/// Reference software impl of [`CrcUmts`]. Gated by the `software-crc`
/// feature so the default protocol surface stays implementation-free — chips
/// bring their own impl (hardware CRC peripheral, table lookup, etc.).
pub struct SoftwareCrcUmts;

impl CrcUmts for SoftwareCrcUmts {
    #[inline]
    fn accumulate(seed: u16, bytes: &[u8]) -> u16 {
        let mut digest = ENGINE.digest_with_initial(seed);
        digest.update(bytes);
        digest.finalize()
    }
}
