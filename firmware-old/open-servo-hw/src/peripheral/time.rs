//! System time traits.

/// System time for timing and delays.
pub trait SystemTime {
    /// Get current time in microseconds.
    ///
    /// Wraps at ~71 minutes for u32.
    fn now_us(&self) -> u32;
}
