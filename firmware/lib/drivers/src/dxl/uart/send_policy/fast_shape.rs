//! Chain-shape arithmetic for DXL instruction targeting and Sync/Bulk/Fast
//! chain wire layout — pure functions from header + slot-walk state to
//! wire-byte offsets and slot positions. No state of its own; consumers
//! thread theirs.

use dxl_protocol::streaming::InstructionHeader;
use dxl_protocol::wire::{BROADCAST_ID, CRC_BYTES, RESPONSE_HEADER_BYTES};
use dxl_protocol::{Id, SlotPosition};

/// Wire bytes of a single Ping Status reply — `RESPONSE_HEADER_BYTES`
/// (header(4) + id + len(2) + inst + err = 9) + 3 payload bytes
/// (model_lo + model_hi + firmware) + CRC_BYTES (2). Multi-servo broadcast
/// Ping convention positions servo N's reply at `N × PING_STATUS_FRAME_BYTES`
/// wire bytes past wire-end so the servos don't collide.
pub(crate) const PING_STATUS_FRAME_BYTES: u32 = RESPONSE_HEADER_BYTES as u32 + 3 + CRC_BYTES as u32;

/// Slot-header bytes a Fast First/Only emission carries (`FF FF FD 00` +
/// BROADCAST id + length(2) + Status instruction = 8 bytes). Sized to match
/// `emit_slot_header` in `dxl-protocol`.
const FAST_SLOT_HEADER_BYTES: u32 = 8;

/// Per-slot body bytes excluding the trailing CRC: `error(1) + id(1) + data`.
fn fast_slot_body_bytes(length: u32) -> u32 {
    2 + length
}

/// Wire bytes a Fast First emission consumes for a per-slot `length`: header +
/// body, no trailing CRC (successors continue).
pub(super) fn fast_first_bytes(length: u32) -> u32 {
    FAST_SLOT_HEADER_BYTES + fast_slot_body_bytes(length)
}

/// Wire bytes a Fast Middle emission consumes for a per-slot `length`: body
/// only.
pub(super) fn fast_middle_bytes(length: u32) -> u32 {
    fast_slot_body_bytes(length)
}

/// Target id carried in any Instruction header. Mirrors
/// `osc_core::services::dxl::dispatcher::header_target` — copied here to keep
/// osc-drivers self-contained per the driver-pattern doc.
pub(crate) fn header_target(h: &InstructionHeader) -> Id {
    use InstructionHeader::*;
    match *h {
        Ping { id }
        | Read { id, .. }
        | Write { id, .. }
        | RegWrite { id, .. }
        | Action { id }
        | Reboot { id }
        | FactoryReset { id, .. }
        | Clear { id, .. }
        | ControlTableBackup { id, .. }
        | SyncRead { id, .. }
        | SyncWrite { id, .. }
        | BulkRead { id }
        | BulkWrite { id }
        | FastSyncRead { id, .. }
        | FastBulkRead { id }
        | Raw { id, .. } => id,
    }
}

/// True when the instruction's target is the chip's own ID or BROADCAST.
pub(super) fn target_addressable(h: &InstructionHeader, id: u8) -> bool {
    let target = header_target(h);
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
/// so the sum is `INST(1) + n·2 + Σ L_k + CRC(2)`.
pub(super) fn fast_chain_packet_length(n_total: u8, chain_data_bytes: u32) -> u16 {
    (3 + 2 * (n_total as u32) + chain_data_bytes) as u16
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
