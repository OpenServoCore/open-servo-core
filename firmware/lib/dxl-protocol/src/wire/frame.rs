pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
pub const BROADCAST_ID: u8 = 0xFE;

/// Three-byte logical pattern (HEADER[0..3]) that triggers byte stuffing —
/// the encoder inserts `STUFFING_BYTE` after every occurrence in the
/// payload so a fake header can't appear mid-frame.
pub const STUFFING_TRIGGER: [u8; 3] = [HEADER[0], HEADER[1], HEADER[2]];

/// Escape byte inserted after every `STUFFING_TRIGGER` on the wire and
/// stripped back out by the decoder.
pub const STUFFING_BYTE: u8 = 0xFD;

/// Minimum value of the wire `Length` field: `INSTRUCTION(1) + CRC(2)`.
/// A `Length` below this can't describe a real frame.
pub const PACKET_LEN_MIN: usize = 1 + CRC_BYTES;

/// Sanity cap on the wire `Length` field — purely a parser guard, not a
/// buffer size (callers size their accumulators independently). Bounds how
/// long the parser stays committed to a phantom header in random traffic;
/// real frames stay well under this.
pub const PACKET_LEN_GUARD: usize = 1024;

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
