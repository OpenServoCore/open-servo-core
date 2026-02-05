//! Debug I/O abstraction for REPL and debug output.
//!
//! This trait abstracts the underlying transport (RTT, UART, USB-CDC, etc.)
//! so the core debug shell logic is portable across different boards.

/// Non-blocking byte-oriented debug channel.
///
/// Implemented by board crates using RTT, UART, USB-CDC, etc.
/// The core debug shell is generic over this trait.
pub trait DebugIo {
    /// Try to read a single byte; returns None if no data is available.
    fn try_read(&mut self) -> Option<u8>;

    /// Best-effort write. Should not block or allocate.
    fn write(&mut self, data: &[u8]);

    /// Optionally flush buffered data (can be a no-op).
    fn flush(&mut self) {}
}
