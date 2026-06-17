use osc_core::BaudRate;

/// Computes the bit period in nanoseconds for `baud`.
pub fn bit_period_ns(baud: BaudRate) -> u64 {
    1_000_000_000 / baud.as_hz() as u64
}
