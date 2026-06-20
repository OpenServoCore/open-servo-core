use osc_core::BaudRate;

/// Computes the bit period in nanoseconds for `baud`.
pub fn bit_period_ns(baud: BaudRate) -> u64 {
    1_000_000_000 / baud.as_hz() as u64
}

/// 8N1 frame time in nanoseconds (start + 8 data + stop = 10 bits).
/// Used by timing tests as the wire-stride between adjacent bytes.
pub fn byte_time_ns(baud: BaudRate) -> u64 {
    10 * bit_period_ns(baud)
}
