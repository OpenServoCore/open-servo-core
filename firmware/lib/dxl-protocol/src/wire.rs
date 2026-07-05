//! Framing constants shared by the frame emitters and the decoder.
//!
//! The stuffing byte-window lives inside [`crate::encode`].

pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
pub const BROADCAST_ID: u8 = 0xFE;

/// Minimum `Length` field value: `INSTRUCTION(1) + CRC(2)`.
pub const PACKET_LEN_MIN: usize = 1 + CRC_BYTES;

/// Parser-side sanity cap on the wire `Length` field -- bounds how long the
/// parser stays committed to a phantom header in random traffic. Not a
/// buffer size (callers size accumulators independently).
pub const PACKET_LEN_GUARD: usize = 1024;

/// `HEADER(4) + ID(1) + LENGTH(2) + INSTRUCTION(1)`.
pub const REQUEST_HEADER_BYTES: usize = 8;

/// `HEADER(4) + ID(1) + LENGTH(2) + INSTRUCTION(1) + ERROR(1)`.
pub const RESPONSE_HEADER_BYTES: usize = 9;

pub const CRC_BYTES: usize = 2;

/// Ping Status parameters: `model_number_le16(2) + firmware_version(1)`.
pub const PING_STATUS_PARAM_BYTES: usize = 3;

/// SyncRead/FastSyncRead per-slave entry: `id(1)`.
pub const SYNC_REQUEST_SLOT_BYTES: usize = 1;

/// BulkRead/FastBulkRead per-slave entry: `id(1) + addr_le16(2) + len_le16(2)`.
pub const BULK_REQUEST_SLOT_BYTES: usize = 5;

/// Bytes before slot 0's payload in a Fast response chain: slot 0 reuses
/// the response header's ERROR byte and adds the slave ID ahead of data.
pub const FAST_RESPONSE_SLOT0_BYTES: usize = RESPONSE_HEADER_BYTES + 1;

/// Bytes before slot `k > 0`'s payload: `ERROR(1) + slave_id(1)`.
pub const FAST_RESPONSE_SLOT_BYTES: usize = 2;
