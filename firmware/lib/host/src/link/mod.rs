//! The link layer: record formats over a reliable ordered byte pipe plus
//! the chip-agnostic server that bridges pipe records to the engine.
//! Depends on `engine`; the engine never depends on this (settled).

pub mod record;
pub mod server;

pub use server::{AdapterRequest, LinkServer, RecordSink};
