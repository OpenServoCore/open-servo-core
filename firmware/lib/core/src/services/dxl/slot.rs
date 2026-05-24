use crate::BaudRate;

// DXL 2.0 Status frame overhead: header(4) + id(1) + length(2) + instr(1) + err(1) + crc(2) = 11
// bytes; the 4-byte margin covers worst-case byte stuffing and inter-slot bus turnaround.
const STATUS_OVERHEAD: u32 = 11;
const SLOT_MARGIN: u32 = 4;
const BITS_PER_BYTE: u32 = 10;

pub fn slot_period_us(baud: BaudRate, param_len: u16) -> u32 {
    let bytes = STATUS_OVERHEAD + param_len as u32 + SLOT_MARGIN;
    let us = (bytes as u64) * (BITS_PER_BYTE as u64) * 1_000_000 / baud.as_hz() as u64;
    us as u32
}

pub fn sync_slot_index(ids: &[u8], our_id: u8) -> Option<usize> {
    ids.iter().position(|&id| id == our_id)
}

pub fn bulk_slot_delay_us(body: &[u8], our_id: u8, baud: BaudRate) -> Option<u32> {
    let mut delay = 0u32;
    for tup in body.chunks_exact(5) {
        if tup[0] == our_id {
            return Some(delay);
        }
        let len = u16::from_le_bytes([tup[3], tup[4]]);
        delay = delay.saturating_add(slot_period_us(baud, len));
    }
    None
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
}
