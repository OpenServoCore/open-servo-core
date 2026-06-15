//! Streaming DXL 2.0 parser -- byte-range-to-event iterator.
//!
//! Fed wire bytes; emits typed [`Event`]s per protocol field. Owns no
//! payload buffer -- payload data flows as ring-relative `(offset, length)`
//! ranges. State is fixed scalars on the order of 32 bytes.
//!
//! See `docs/dxl-streaming-rx.md` §3 for the design.

#![allow(dead_code)]

mod event;
mod parser;
mod stage;

pub use event::{
    CrcResult, Event, HeaderEvent, InstructionHeader, InstructionPayload, PayloadEvent, ResyncKind,
    StatusHeader, StatusPayload,
};
pub use parser::{EventStream, Parser};
