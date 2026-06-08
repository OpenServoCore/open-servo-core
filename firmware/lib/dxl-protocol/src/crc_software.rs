use crc::{CRC_16_UMTS, Crc};

use crate::crc::CrcUmts;

const ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

/// Reference software impl of [`CrcUmts`], gated by the `software-crc`
/// feature. Chips bring their own (hardware peripheral, table lookup, etc.).
#[derive(Copy, Clone, Debug)]
pub struct SoftwareCrcUmts {
    state: u16,
}

impl SoftwareCrcUmts {
    /// Resume a chain CRC computed in a prior pass (e.g. discontiguous
    /// ring-walk slices sharing one running CRC).
    pub fn new_with_state(state: u16) -> Self {
        Self { state }
    }
}

impl CrcUmts for SoftwareCrcUmts {
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

impl Default for SoftwareCrcUmts {
    fn default() -> Self {
        Self::new()
    }
}
