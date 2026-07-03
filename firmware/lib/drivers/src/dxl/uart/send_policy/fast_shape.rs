//! Chain-shape arithmetic for DXL instruction targeting and Sync/Bulk/Fast
//! chain wire layout — pure functions from header + slot-walk state to
//! wire-byte offsets and slot positions. No state of its own; consumers
//! thread theirs.

use dxl_protocol::SlotPosition;
use dxl_protocol::streaming::InstructionHeader;
use dxl_protocol::wire::{
    BROADCAST_ID, CRC_BYTES, FAST_RESPONSE_SLOT_BYTES, FAST_RESPONSE_SLOT0_BYTES, PACKET_LEN_MIN,
    PING_STATUS_PARAM_BYTES, RESPONSE_HEADER_BYTES,
};

/// Wire bytes of a single Ping Status reply. Multi-servo broadcast Ping
/// convention positions servo N's reply at `N × PING_STATUS_FRAME_BYTES`
/// wire bytes past wire-end so the servos don't collide.
pub(super) const PING_STATUS_FRAME_BYTES: u32 =
    (RESPONSE_HEADER_BYTES + PING_STATUS_PARAM_BYTES + CRC_BYTES) as u32;

/// Wire bytes a Fast First emission consumes for a per-slot `length`: chain
/// header + slot 0's `error + id` prefix + data, no trailing CRC
/// (successors continue).
pub(super) fn fast_first_bytes(length: u32) -> u32 {
    FAST_RESPONSE_SLOT0_BYTES as u32 + length
}

/// Wire bytes a Fast Middle emission consumes for a per-slot `length`:
/// `error + id` prefix + data.
pub(super) fn fast_middle_bytes(length: u32) -> u32 {
    FAST_RESPONSE_SLOT_BYTES as u32 + length
}

/// True when the instruction's target is the chip's own ID or BROADCAST.
pub(super) fn target_addressable(h: &InstructionHeader, id: u8) -> bool {
    let target = h.target();
    target.as_byte() == id || target.as_byte() == BROADCAST_ID
}

/// Wire bytes preceding slot `k` in a Fast Sync Read chain with uniform
/// per-slot register length. Slot 0 sits at offset 0 (it carries the chain
/// header). Slot k > 0 follows a First emission and `k-1` Middle emissions.
pub(super) fn fast_bytes_before(k: u8, length: u32) -> u32 {
    if k == 0 {
        0
    } else {
        fast_first_bytes(length) + ((k as u32) - 1) * fast_middle_bytes(length)
    }
}

/// Wire length carried by the Fast chain Status header — bytes from `INST`
/// through trailing CRC inclusive. Encoder-input for both Only and First
/// `packet_length`. Per-slot wire shape is `err(1) + id(1) + data(L_k)`,
/// so the sum is `INST(1) + CRC(2) + n·2 + Σ L_k`.
pub(super) fn fast_chain_packet_length(n_total: u8, chain_data_bytes: u32) -> u16 {
    (PACKET_LEN_MIN as u32 + FAST_RESPONSE_SLOT_BYTES as u32 * (n_total as u32) + chain_data_bytes)
        as u16
}

/// Map a chain `(slot_index, total_slots, packet_length)` to the
/// [`SlotPosition`] the encoder consumes. `packet_length` on First/Only is
/// the chain's advertised wire length — emitted straight onto the wire by
/// [`dxl_protocol::encoder::SlotEncoder::emit`].
pub(super) fn compute_fast_position(k: u8, n_total: u8, packet_length: u16) -> SlotPosition {
    if n_total == 1 {
        SlotPosition::Only { packet_length }
    } else if k == 0 {
        SlotPosition::First { packet_length }
    } else if k + 1 == n_total {
        SlotPosition::Last { crc: 0 }
    } else {
        SlotPosition::Middle
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::test_support::TEST_ID;
    use dxl_protocol::Id;

    #[test]
    fn ping_status_frame_is_fourteen_wire_bytes() {
        // header(4) + id + len(2) + inst + err + model(2) + fw + crc(2).
        assert_eq!(PING_STATUS_FRAME_BYTES, 14);
    }

    #[test]
    fn fast_first_carries_chain_header_plus_slot_prefix() {
        // FF FF FD 00 + id + len(2) + inst + err + servo_id = 10, then data.
        assert_eq!(fast_first_bytes(2), 12);
        assert_eq!(fast_middle_bytes(2), 4);
    }

    #[test]
    fn fast_bytes_before_walks_first_then_middles() {
        assert_eq!(fast_bytes_before(0, 2), 0);
        assert_eq!(fast_bytes_before(1, 2), 12);
        assert_eq!(fast_bytes_before(3, 2), 12 + 2 * 4);
    }

    #[test]
    fn fast_chain_packet_length_counts_inst_crc_and_slots() {
        // INST(1) + CRC(2) + n·(err+id) + Σ data.
        assert_eq!(fast_chain_packet_length(1, 2), 7);
        assert_eq!(fast_chain_packet_length(3, 6), 15);
    }

    #[test]
    fn compute_fast_position_maps_the_chain_walk() {
        assert_eq!(
            compute_fast_position(0, 1, 7),
            SlotPosition::Only { packet_length: 7 }
        );
        assert_eq!(
            compute_fast_position(0, 3, 15),
            SlotPosition::First { packet_length: 15 }
        );
        assert_eq!(compute_fast_position(1, 3, 15), SlotPosition::Middle);
        assert_eq!(
            compute_fast_position(2, 3, 15),
            SlotPosition::Last { crc: 0 }
        );
    }

    #[test]
    fn target_addressable_accepts_own_id_and_broadcast_only() {
        let own = InstructionHeader::Ping {
            id: Id::new(TEST_ID),
        };
        let broadcast = InstructionHeader::Ping {
            id: Id::new(BROADCAST_ID),
        };
        let foreign = InstructionHeader::Ping { id: Id::new(0x42) };
        assert!(target_addressable(&own, TEST_ID));
        assert!(target_addressable(&broadcast, TEST_ID));
        assert!(!target_addressable(&foreign, TEST_ID));
    }
}
