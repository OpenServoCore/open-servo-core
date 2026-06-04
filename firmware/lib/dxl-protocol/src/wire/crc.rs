/// CRC-16/UMTS (poly 0x8005, init 0x0000, non-reflected; aliases
/// CRC-16/BUYPASS, CRC-16/VERIFONE) — the variant the DXL 2.0 wire format
/// uses. ROBOTIS docs call this "CRC-16 (IBM/ANSI)" colloquially, but the
/// true IBM-SDLC variant is reflected and not what's on the wire.
///
/// Chips inject their own impl so they can pick hardware vs. software; the
/// protocol layer only ever sees this trait. A reference software impl is
/// available behind the `software-crc` feature for tests and dev tooling.
pub trait CrcUmts {
    /// Fold `bytes` into the running CRC starting from `seed`. A whole-frame
    /// CRC is `accumulate(0, bytes)`; an incremental chain feeds the previous
    /// return value back in as `seed`.
    fn accumulate(seed: u16, bytes: &[u8]) -> u16;
}
