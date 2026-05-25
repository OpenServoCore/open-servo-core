use crate::BaudRate;

// DXL 2.0 Status frame overhead: header(4) + id(1) + length(2) + instr(1) + err(1) + crc(2) = 11
// bytes; the 4-byte margin covers worst-case byte stuffing and inter-slot bus turnaround.
const STATUS_OVERHEAD: u32 = 11;
const SLOT_MARGIN: u32 = 4;
const BITS_PER_BYTE: u32 = 10;

// Fast Sync/Bulk Read coalesce all slave replies into one Status frame:
//   FF FF FD 00 FE LEN_lo LEN_hi 0x55  ERR_0 ID_0 data_0  ERR_1 ID_1 data_1 ... CRC_lo CRC_hi
// Slot 0 emits the 8-byte fixed header plus its ERR+ID prefix (10 bytes total)
// then payload_0; each slot k≥1 emits ERR+ID (2 bytes) then payload_k; CRC is
// appended by the last slot.
const FAST_SLOT0_PREFIX: u32 = 10;
const FAST_SLOT_PREFIX: u32 = 2;

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

/// Fast Sync Read variant: every slot carries the same payload length.
/// (For varied per-slot lengths use `fast_bulk_slot_delay_us`.)
pub fn fast_sync_slot_delay_us(slot_index: usize, payload_len: u16, baud: BaudRate) -> u32 {
    if slot_index == 0 {
        return 0;
    }
    let payload = payload_len as u32;
    let bytes =
        FAST_SLOT0_PREFIX + payload + (slot_index as u32 - 1) * (FAST_SLOT_PREFIX + payload);
    bytes_to_us(bytes, baud)
}

/// Fast Bulk Read variant: walks `body`'s 5-byte `(id, addr, length)` tuples
/// to sum each preceding slot's payload. Trailing partial tuples are ignored.
pub fn fast_bulk_slot_delay_us(slot_index: usize, body: &[u8], baud: BaudRate) -> u32 {
    if slot_index == 0 {
        return 0;
    }
    let mut bytes = 0u32;
    for (i, tup) in body.chunks_exact(5).enumerate().take(slot_index) {
        let payload = u16::from_le_bytes([tup[3], tup[4]]) as u32;
        let prefix = if i == 0 {
            FAST_SLOT0_PREFIX
        } else {
            FAST_SLOT_PREFIX
        };
        bytes = bytes.saturating_add(prefix).saturating_add(payload);
    }
    bytes_to_us(bytes, baud)
}

fn bytes_to_us(bytes: u32, baud: BaudRate) -> u32 {
    ((bytes as u64) * (BITS_PER_BYTE as u64) * 1_000_000 / baud.as_hz() as u64) as u32
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
    fn fast_sync_slot_delay_us_slot_zero_is_zero() {
        assert_eq!(fast_sync_slot_delay_us(0, 4, BaudRate::B1000000), 0);
        assert_eq!(fast_sync_slot_delay_us(0, 128, BaudRate::B3000000), 0);
    }

    #[test]
    fn fast_sync_slot_delay_us_matches_byte_time_at_1mbaud() {
        // slot 1: 10 + 4 = 14 bytes * 10 / 1e6 = 140 µs.
        assert_eq!(fast_sync_slot_delay_us(1, 4, BaudRate::B1000000), 140);
        // slot 2: 14 + (2 + 4) = 20 bytes * 10 / 1e6 = 200 µs.
        assert_eq!(fast_sync_slot_delay_us(2, 4, BaudRate::B1000000), 200);
    }

    #[test]
    fn fast_sync_slot_delay_us_at_3mbaud_floors() {
        // 14 bytes * 10 / 3 = 46.66… → 46
        assert_eq!(fast_sync_slot_delay_us(1, 4, BaudRate::B3000000), 46);
    }

    #[test]
    fn fast_bulk_slot_delay_us_slot_zero_is_zero() {
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0];
        assert_eq!(fast_bulk_slot_delay_us(0, &body, BaudRate::B1000000), 0);
    }

    #[test]
    fn fast_bulk_slot_delay_us_matches_uniform_helper() {
        // Three slots, all length 4 — should equal the Fast Sync uniform helper.
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 4, 0, 3, 0, 0, 4, 0];
        for k in 0..3 {
            assert_eq!(
                fast_bulk_slot_delay_us(k, &body, BaudRate::B1000000),
                fast_sync_slot_delay_us(k, 4, BaudRate::B1000000)
            );
        }
    }

    #[test]
    fn fast_bulk_slot_delay_us_sums_varied_lengths() {
        // slot 0: len=4, slot 1: len=8, slot 2: len=2.
        // bytes to slot 1: FAST_SLOT0_PREFIX(10) + 4 = 14
        // bytes to slot 2: 14 + FAST_SLOT_PREFIX(2) + 8 = 24
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0, 3, 0, 0, 2, 0];
        assert_eq!(fast_bulk_slot_delay_us(1, &body, BaudRate::B1000000), 140);
        assert_eq!(fast_bulk_slot_delay_us(2, &body, BaudRate::B1000000), 240);
    }

    #[test]
    fn fast_bulk_slot_delay_us_ignores_trailing_partial_tuple() {
        // Two complete tuples + 3 stray bytes.
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0, 9, 9, 9];
        assert_eq!(fast_bulk_slot_delay_us(2, &body, BaudRate::B1000000), 240);
    }
}
