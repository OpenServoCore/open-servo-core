/// CRC-16/UMTS (poly 0x8005, init 0x0000, non-reflected; aliases
/// CRC-16/BUYPASS, CRC-16/VERIFONE) — the variant the DXL 2.0 wire format
/// uses. ROBOTIS docs call this "CRC-16 (IBM/ANSI)" colloquially, but the
/// true IBM-SDLC variant is reflected and not what's on the wire.
///
/// Streaming engine, shaped after [`crc32fast::Hasher`] / [`core::hash::Hasher`]
/// — construct, [`update`](Self::update) bytes into it, read with
/// [`finalize`](Self::finalize), [`reset`](Self::reset) to reuse. Chips inject
/// their own impl so they can pick hardware vs. software; a reference software
/// impl is available behind the `software-crc` feature for tests and dev
/// tooling.
pub trait CrcUmts: Sized {
    /// Construct an engine in the canonical CRC-16/UMTS initial state (0).
    fn new() -> Self;

    /// Fold `bytes` into the running state.
    fn update(&mut self, bytes: &[u8]);

    /// Read the current running state without consuming. The result is the
    /// finished CRC value when called after the last byte of a frame.
    fn finalize(&self) -> u16;

    /// Reset to the canonical initial state. Equivalent to constructing a
    /// fresh engine, but cheaper for impls backed by a peripheral.
    fn reset(&mut self);
}
