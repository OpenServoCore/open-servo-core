//! DXL CRC-16/UMTS provider. V006 has no CRC peripheral, so the impl is
//! software — `crc` crate's table-driven engine over the DXL polynomial.
//! A chip with a hardware CRC unit would swap this file with a peripheral-
//! driven implementation; the trait surface (`dxl_protocol::CrcUmts`) is
//! the same.

use dxl_protocol::{CrcUmts, crc16_umts_continue};

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

    #[inline]
    fn update(&mut self, bytes: &[u8]) {
        self.state = crc16_umts_continue(self.state, bytes);
    }

    // The default's slice round-trip pays the `crc` crate's digest
    // setup/finalize PER BYTE — the framer fold and fused encode call this
    // once per wire byte on the reply-turnaround path.
    #[inline(always)]
    fn update_byte(&mut self, b: u8) {
        self.state = crc16_umts_continue(self.state, core::slice::from_ref(&b));
    }

    fn finalize(&self) -> u16 {
        self.state
    }

    fn reset(&mut self) {
        self.state = 0;
    }

    fn new_with_state(state: u16) -> Self {
        Self { state }
    }
}

impl Default for DxlCrc {
    fn default() -> Self {
        Self::new()
    }
}
