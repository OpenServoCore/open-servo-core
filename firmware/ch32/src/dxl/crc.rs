use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::CrcUmts;

/// CH32V006 has no CRC peripheral, so this is software — `crc` crate's
/// table-driven impl over the DXL polynomial. Same wire algorithm as
/// `dxl_protocol::SoftwareCrcUmts`; lives here because the chip is the
/// authority on how its frames' CRCs get computed (would be swapped for a
/// peripheral-driven impl on chips that have one).
const ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

pub struct Ch32DxlCrc;

impl CrcUmts for Ch32DxlCrc {
    #[inline]
    fn accumulate(seed: u16, bytes: &[u8]) -> u16 {
        let mut digest = ENGINE.digest_with_initial(seed);
        digest.update(bytes);
        digest.finalize()
    }
}
