use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::CrcUmts;

/// CH32V006 has no CRC peripheral, so this is software — `crc` crate's
/// table-driven impl over the DXL polynomial. Same wire algorithm as
/// `dxl_protocol::SoftwareCrcUmts`; lives here because the chip is the
/// authority on how its frames' CRCs get computed (would be swapped for a
/// peripheral-driven impl on chips that have one).
const ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

#[derive(Copy, Clone, Debug)]
pub struct Ch32DxlCrc {
    state: u16,
}

impl Ch32DxlCrc {
    /// Construct an engine seeded with an arbitrary intermediate state —
    /// for resuming a chain CRC across discontiguous ring-walk slices that
    /// share a single running value.
    pub fn new_with_state(state: u16) -> Self {
        Self { state }
    }
}

impl CrcUmts for Ch32DxlCrc {
    fn new() -> Self {
        Self { state: 0 }
    }

    fn update(&mut self, bytes: &[u8]) {
        let mut digest = ENGINE.digest_with_initial(self.state);
        digest.update(bytes);
        self.state = digest.finalize();
    }

    fn finalize(&self) -> u16 {
        self.state
    }

    fn reset(&mut self) {
        self.state = 0;
    }
}

impl Default for Ch32DxlCrc {
    fn default() -> Self {
        Self::new()
    }
}
