use crc::{CRC_16_UMTS, Crc};

use super::crc::CrcUmts;

const ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

/// Reference software impl of [`CrcUmts`]. Gated by the `software-crc`
/// feature so the default protocol surface stays implementation-free — chips
/// bring their own impl (hardware CRC peripheral, table lookup, etc.).
#[derive(Copy, Clone, Debug)]
pub struct SoftwareCrcUmts {
    state: u16,
}

impl SoftwareCrcUmts {
    /// Construct an engine seeded with an arbitrary intermediate state —
    /// for resuming a chain CRC computed in a prior pass (e.g. discontiguous
    /// ring-walk slices that share a single running CRC).
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
