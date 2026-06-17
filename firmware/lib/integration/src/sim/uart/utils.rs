/// Computes the bit period in nanoseconds for `baud`.
pub fn bit_period_ns(baud: u32) -> u64 {
    1_000_000_000 / baud as u64
}
