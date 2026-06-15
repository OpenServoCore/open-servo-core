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
}
