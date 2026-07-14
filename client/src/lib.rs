//! osc-client -- the host-side client for the osc-native bus (host-band
//! phase 4). Transport-generic: an async [`Client`] drives the adapter's
//! record pipe over any [`pipe::Pipe`]; [`blocking`] wraps it for
//! synchronous callers. Deliberately model-agnostic: register access is raw
//! addr+len, protocol-normative common registers arrive as osc-protocol
//! consts, per-model typed maps are a later band.
//!
//! All protocol timing lives in the adapter's engine; the only clock here is
//! a coarse pipe guard that catches an unplugged adapter.

pub mod blocking;
mod client;
mod error;
pub mod mgmt;
pub mod pipe;
pub mod session;

#[cfg(feature = "fake-adapter")]
pub mod fake;
#[cfg(feature = "nusb")]
pub mod nusb;

pub use client::{Chain, Client, Ping, Reply, Status};
pub use error::{Error, LinkError, RejectReason};

pub use osc_host::engine::Outcome;
pub use osc_protocol::wire::{BaudRate, Id, Inst, MgmtOp, Opcode, ResultCode};
