use crate::BaudRate;

/// Bytes of inter-slot bus turnaround / worst-case stuffing margin added on
/// top of each Status frame in a Sync/Bulk reply train.
pub const SLOT_MARGIN: u32 = 4;

/// Q16.16 multiply; `us_per_byte_q16` is precomputed so RV32EC avoids `__udivdi3`.
pub fn bytes_to_us(bytes: u32, baud: BaudRate) -> u32 {
    ((bytes as u64 * baud.us_per_byte_q16() as u64) >> 16) as u32
}

/// Q8.8 µs (1 unit = 1/256 µs ≈ 3.9 ns). For chain Last fire scheduling, where
/// flooring to whole µs costs up to ~0.67 µs at 3M (10/baud not integer µs) —
/// enough to fire ahead of the predecessor's last stop bit.
pub fn bytes_to_us_q88(bytes: u32, baud: BaudRate) -> u32 {
    ((bytes as u64 * baud.us_per_byte_q16() as u64) >> 8) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_to_us_at_3mbaud_floors() {
        // 14 bytes * 10 / 3 = 46.66… → 46 (truncated by Q16.16 floor).
        // Chain Last math uses bytes_to_us_q88 to recover the lost precision;
        // bytes_to_us stays at floor for slot-period / stall callers.
        assert_eq!(bytes_to_us(14, BaudRate::B3000000), 46);
    }

    #[test]
    fn bytes_to_us_q88_matches_q88_truth() {
        // 14 bytes * 10/3 µs = 46.667 µs. Q8.8 = floor(46.667 × 256) = 11946.
        assert_eq!(bytes_to_us_q88(14, BaudRate::B3000000), 11946);
        // 1M is exact: 11 bytes = 110 µs = 28160 q88.
        assert_eq!(bytes_to_us_q88(11, BaudRate::B1000000), 11 * 10 * 256);
        // 0 bytes is 0 q88.
        assert_eq!(bytes_to_us_q88(0, BaudRate::B3000000), 0);
    }
}
