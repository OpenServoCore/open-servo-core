#![no_std]

mod buf;
mod bytes;
mod crc;
mod instruction;
mod parser;
mod writer;

pub use buf::WriteBuf;
pub use bytes::{ByteIter, Bytes, Overflow};
pub use crc::crc16;
pub use instruction::Instruction;
pub use parser::{BROADCAST_ID, HEADER, MAX_LENGTH, Packet, ParseError, parse_one};
pub use writer::{WriteError, write};
