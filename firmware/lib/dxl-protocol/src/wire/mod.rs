//! Wire-layer primitives: framing constants, CRC, write buffer trait.

mod buf;
mod crc;
#[cfg(feature = "software-crc")]
mod crc_software;
mod frame;

pub use buf::{WriteBuf, WriteError};
pub use crc::CrcUmts;
#[cfg(feature = "software-crc")]
pub use crc_software::SoftwareCrcUmts;
pub use frame::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, CRC_BYTES, FAST_RESPONSE_SLOT_BYTES,
    FAST_RESPONSE_SLOT0_BYTES, HEADER, PACKET_LEN_GUARD, PACKET_LEN_MIN, REQUEST_HEADER_BYTES,
    RESPONSE_HEADER_BYTES, STUFFING_BYTE, STUFFING_TRIGGER, SYNC_REQUEST_SLOT_BYTES,
};
