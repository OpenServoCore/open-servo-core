//! DXL CRC-16/UMTS provider. V006 has no CRC peripheral, so the impl is
//! software — `crc` crate's table-driven engine over the DXL polynomial.
//! A chip with a hardware CRC unit would swap this file with a peripheral-
//! driven implementation; the trait surface (`dxl_protocol::CrcUmts`) is
//! the same.

use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::CrcUmts;

const ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

#[derive(Copy, Clone, Debug)]
pub struct DxlCrc {
    state: u16,
}

impl DxlCrc {
    /// Construct an engine seeded with an arbitrary intermediate state —
    /// for resuming a chain CRC across discontiguous ring-walk slices that
    /// share a single running value.
    pub fn new_with_state(state: u16) -> Self {
        Self { state }
    }
}

impl CrcUmts for DxlCrc {
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

impl Default for DxlCrc {
    fn default() -> Self {
        Self::new()
    }
}
