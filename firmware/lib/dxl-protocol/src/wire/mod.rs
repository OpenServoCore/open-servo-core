//! Wire layer: framing, length/CRC validation, byte stuffing. Knows nothing
//! about instruction-typed packets — emits `RawFrame` for typed decode to
//! interpret.

mod buf;
mod bytes;
mod crc;
#[cfg(feature = "software-crc")]
mod crc_software;
mod frame;
mod parser;
mod rx_view;
mod stuffing;
mod writer;

pub use buf::{WriteBuf, WriteError};
pub use bytes::{ByteIter, Bytes, Overflow};
pub use crc::CrcUmts;
#[cfg(feature = "software-crc")]
pub use crc_software::SoftwareCrcUmts;
pub(crate) use frame::RawFrame;
pub use frame::{
    BROADCAST_ID, BULK_REQUEST_SLOT_BYTES, CRC_BYTES, FAST_RESPONSE_SLOT_BYTES,
    FAST_RESPONSE_SLOT0_BYTES, HEADER, MAX_LENGTH, REQUEST_HEADER_BYTES, RESPONSE_HEADER_BYTES,
    SYNC_REQUEST_SLOT_BYTES,
};
pub use parser::ParseError;
pub(crate) use parser::parse_raw;
pub use rx_view::RxView;
pub(crate) use writer::write_raw;
