//! Consumer-owned finalize interface for the chain-CRC fold.

/// Sink the [`FoldEngine`] finalizes the chain CRC into. Consumer-owned
/// (driver-pattern §5.1): the fold engine folds predecessor wire bytes but
/// never sees the reply buffer — the codec's TX half implements this so the
/// engine mixes our own reply bytes and patches the trailing CRC slot
/// without importing or reaching into a sibling sub-driver.
///
/// [`FoldEngine`]: super::FoldEngine
pub trait CrcPatchSink {
    /// Our own encoded reply bytes, excluding the trailing 2-byte
    /// placeholder CRC slot — the tail the chain CRC folds last. Empty when
    /// no reply has been encoded yet.
    fn own_reply_bytes(&self) -> &[u8];

    /// Overwrite the trailing 2-byte placeholder CRC slot with the finalized
    /// chain CRC (little-endian).
    fn patch_crc(&mut self, crc: u16);
}
