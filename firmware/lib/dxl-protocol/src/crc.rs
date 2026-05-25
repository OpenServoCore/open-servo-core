use crc::{Algorithm, Crc};

const DXL_CRC: Algorithm<u16> = Algorithm {
    width: 16,
    poly: 0x8005,
    init: 0x0000,
    refin: false,
    refout: false,
    xorout: 0x0000,
    check: 0xFEE8,
    residue: 0x0000,
};

const DXL: Crc<u16> = Crc::<u16>::new(&DXL_CRC);

pub fn crc16(bytes: &[u8]) -> u16 {
    DXL.checksum(bytes)
}

#[inline]
pub fn crc16_continue(seed: u16, bytes: &[u8]) -> u16 {
    let mut digest = DXL.digest_with_initial(seed);
    digest.update(bytes);
    digest.finalize()
}
