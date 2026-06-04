use super::bytes::Bytes;

pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
pub const BROADCAST_ID: u8 = 0xFE;

/// Cap on the wire `Length` field. Bounds how long the parser waits on a
/// phantom header in random traffic; real frames stay well under this.
pub const MAX_LENGTH: usize = 1024;

/// Bytes before the parameter region of any request packet:
/// `HEADER(4) + ID(1) + LENGTH(2) + INSTRUCTION(1)`.
pub const REQUEST_HEADER_BYTES: usize = 8;

/// Bytes before the parameter region of any Status response packet:
/// `HEADER(4) + ID(1) + LENGTH(2) + INSTRUCTION(1) + ERROR(1)`.
pub const RESPONSE_HEADER_BYTES: usize = 9;

/// Trailing CRC bytes on every framed packet.
pub const CRC_BYTES: usize = 2;

/// Per-slave entry width in SyncRead / FastSyncRead request bodies: just `id`.
pub const SYNC_REQUEST_SLOT_BYTES: usize = 1;

/// Per-slave entry width in BulkRead / FastBulkRead request bodies:
/// `id(1) + addr_le16(2) + len_le16(2)`.
pub const BULK_REQUEST_SLOT_BYTES: usize = 5;

/// Bytes before slot 0's payload in a Fast response chain:
/// `RESPONSE_HEADER_BYTES + slave_id(1)` — slot 0 reuses the response
/// header's ERROR byte, then adds the slave ID ahead of its data.
pub const FAST_RESPONSE_SLOT0_BYTES: usize = RESPONSE_HEADER_BYTES + 1;

/// Bytes before slot `k > 0`'s payload in a Fast response chain:
/// `ERROR(1) + slave_id(1)`.
pub const FAST_RESPONSE_SLOT_BYTES: usize = 2;

/// Wire-layer view of a single validated DXL frame: header, ID, length, and
/// CRC are accounted for, but the instruction byte has not been resolved to
/// the `Instruction` enum and `params` has not been interpreted. Use this as
/// the boundary between wire parsing and typed decode.
#[derive(Copy, Clone, Debug)]
pub struct RawFrame<'a> {
    pub id: u8,
    pub instruction: u8,
    pub params: Bytes<'a>,
}
