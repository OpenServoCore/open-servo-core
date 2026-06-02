use crate::BaudRate;

// DXL 2.0 Status frame overhead: header(4) + id(1) + length(2) + instr(1) + err(1) + crc(2) = 11
// bytes; the 4-byte margin covers worst-case byte stuffing and inter-slot bus turnaround.
const STATUS_OVERHEAD: u32 = 11;
const SLOT_MARGIN: u32 = 4;

pub fn slot_period_us(baud: BaudRate, param_len: u16) -> u32 {
    let bytes = STATUS_OVERHEAD + param_len as u32 + SLOT_MARGIN;
    bytes_to_us(bytes, baud)
}

pub fn sync_slot_index(ids: &[u8], our_id: u8) -> Option<usize> {
    ids.iter().position(|&id| id == our_id)
}

pub fn bulk_slot_delay_us(body: &[u8], our_id: u8, baud: BaudRate) -> Option<u32> {
    let mut bytes = 0u32;
    for tup in body.chunks_exact(5) {
        if tup[0] == our_id {
            return Some(bytes_to_us(bytes, baud));
        }
        let len = u16::from_le_bytes([tup[3], tup[4]]) as u32;
        bytes = bytes
            .saturating_add(STATUS_OVERHEAD)
            .saturating_add(len)
            .saturating_add(SLOT_MARGIN);
    }
    None
}

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
    fn slot_period_us_at_1mbaud() {
        // (11 + 0 + 4) * 10 / 1 = 150 µs
        assert_eq!(slot_period_us(BaudRate::B1000000, 0), 150);
        // (11 + 4 + 4) * 10 / 1 = 190 µs
        assert_eq!(slot_period_us(BaudRate::B1000000, 4), 190);
    }

    #[test]
    fn slot_period_us_at_57600_baud() {
        // (11 + 16 + 4) * 10_000_000 / 57600 ≈ 5381 µs
        assert_eq!(slot_period_us(BaudRate::B57600, 16), 5381);
    }

    #[test]
    fn slot_period_us_handles_u16_max_without_overflow() {
        // (11 + 65535 + 4) * 10_000_000 fits in u64; final u32 cast is the only risk.
        let us = slot_period_us(BaudRate::B1000000, u16::MAX);
        assert_eq!(us, (11 + 65535 + 4) * 10);
    }

    #[test]
    fn sync_slot_index_returns_position_or_none() {
        assert_eq!(sync_slot_index(&[1, 2, 3], 2), Some(1));
        assert_eq!(sync_slot_index(&[1, 2, 3], 1), Some(0));
        assert_eq!(sync_slot_index(&[1, 2, 3], 9), None);
        assert_eq!(sync_slot_index(&[], 1), None);
    }

    #[test]
    fn bulk_slot_delay_us_first_slot_is_zero() {
        let body = [5, 0, 0, 4, 0, 7, 0, 0, 4, 0];
        assert_eq!(bulk_slot_delay_us(&body, 5, BaudRate::B1000000), Some(0));
    }

    #[test]
    fn bulk_slot_delay_us_sums_preceding_slot_periods() {
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0];
        let expected = slot_period_us(BaudRate::B1000000, 4);
        assert_eq!(
            bulk_slot_delay_us(&body, 2, BaudRate::B1000000),
            Some(expected)
        );
    }

    #[test]
    fn bulk_slot_delay_us_missing_id_returns_none() {
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 4, 0];
        assert_eq!(bulk_slot_delay_us(&body, 9, BaudRate::B1000000), None);
    }

    #[test]
    fn bulk_slot_delay_us_ignores_trailing_partial_tuple() {
        let body = [1, 0, 0, 4, 0, 9, 9, 9];
        assert_eq!(bulk_slot_delay_us(&body, 1, BaudRate::B1000000), Some(0));
        assert_eq!(bulk_slot_delay_us(&body, 2, BaudRate::B1000000), None);
    }

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
