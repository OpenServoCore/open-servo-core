#![no_std]
//! DXL 2.0 wire-format codec.
//!
//! [`streaming::Parser`] walks wire bytes and emits typed
//! [`Event`](streaming::Event)s per protocol field. [`InstructionEncoder`],
//! [`StatusEncoder`], and [`SlotEncoder`] emit the three frame shapes.

mod buf;
pub mod crc;
pub mod encoder;
pub mod streaming;
pub mod types;
pub mod wire;

#[cfg(test)]
mod test_util;

pub use buf::{WriteBuf, WriteError};
pub use crc::CrcUmts;
#[cfg(feature = "software-crc")]
pub use crc::SoftwareCrcUmts;
pub use encoder::{InstructionEncoder, SlotEncoder, StatusEncoder};
pub use types::{
    BulkReadEntry, ErrorCode, Id, Instruction, PingStatus, Slot, SlotPosition, Status, StatusError,
};
