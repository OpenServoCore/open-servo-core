//! CRC-16/UMTS trait and reference impl.
//!
//! Chips inject their own [`CrcUmts`] so they can pick hardware vs.
//! software; the reference [`SoftwareCrcUmts`] lives in [`software`] behind
//! the `software-crc` feature.

#[cfg(feature = "software-crc")]
pub mod software;

#[cfg(feature = "software-crc")]
pub use software::SoftwareCrcUmts;

/// CRC-16/UMTS (poly 0x8005, init 0x0000, non-reflected; aliases
/// CRC-16/BUYPASS, CRC-16/VERIFONE) -- the variant DXL 2.0 uses. ROBOTIS
/// docs call this "CRC-16 (IBM/ANSI)" colloquially, but the true IBM-SDLC
/// variant is reflected and *not* what's on the wire.
///
/// Chips inject their own impl so they can pick hardware vs. software; a
/// reference software impl is available behind the `software-crc` feature.
pub trait CrcUmts: Sized {
    fn new() -> Self;
    fn update(&mut self, bytes: &[u8]);
    fn finalize(&self) -> u16;
    /// Reset to initial state -- equivalent to a fresh engine but cheaper
    /// for peripheral-backed impls.
    fn reset(&mut self);

    /// Fold a single byte. The fused single-pass encoder's hot loop calls this
    /// per param byte; peripheral- or table-backed impls override it to avoid
    /// the per-byte engine setup the default's slice round-trip would pay.
    fn update_byte(&mut self, b: u8) {
        self.update(core::slice::from_ref(&b));
    }
}

const fn crc16_umts_entry(index: u16) -> u16 {
    let mut crc = index << 8;
    let mut bit = 0;
    while bit < 8 {
        crc = if crc & 0x8000 != 0 {
            (crc << 1) ^ 0x8005
        } else {
            crc << 1
        };
        bit += 1;
    }
    crc
}

const fn crc16_umts_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        table[i] = crc16_umts_entry(i as u16);
        i += 1;
    }
    table
}

/// Byte-indexed CRC-16/UMTS step table (non-reflected, MSB-first).
const CRC16_UMTS_TABLE: [u16; 256] = crc16_umts_table();

/// Extend a running CRC-16/UMTS `state` over `bytes` and return the new
/// state. With `init = 0x0000` and no output XOR, the state IS the wire
/// value at any prefix — seed with `0` for a fresh packet, or with a state
/// carried from earlier bytes of the same packet.
///
/// A plain table step per byte with no engine object, so callers on
/// deterministic-latency paths (the Fast Status chain-CRC fold spins this
/// against a live 3M wire) pay exactly one table read + shift/xor per byte
/// and the whole loop inlines into the caller — a [`CrcUmts`] engine can't
/// promise that (its impl may live behind a call, or in a peripheral with
/// MMIO setup cost per update).
#[inline(always)]
pub fn crc16_umts_continue(state: u16, bytes: &[u8]) -> u16 {
    let mut crc = state;
    for &b in bytes {
        crc = (crc << 8) ^ CRC16_UMTS_TABLE[(((crc >> 8) as u8) ^ b) as usize];
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn continue_matches_the_umts_check_value() {
        // Standard CRC catalog check input; CRC-16/UMTS check = 0xFEE8.
        assert_eq!(crc16_umts_continue(0, b"123456789"), 0xFEE8);
    }

    #[test]
    fn continue_composes_across_split_updates() {
        let split = crc16_umts_continue(crc16_umts_continue(0, b"1234"), b"56789");
        assert_eq!(split, 0xFEE8);
    }

    #[cfg(feature = "software-crc")]
    #[test]
    fn continue_matches_the_engine_impl() {
        let bytes = [0xFF_u8, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x55, 0x00];
        let mut engine = SoftwareCrcUmts::new();
        engine.update(&bytes);
        assert_eq!(crc16_umts_continue(0, &bytes), engine.finalize());
    }
}
