//! RTT communication module.
//!
//! Handles probe-rs session management, RTT channel I/O, and defmt decoding.

pub mod defmt_decoder;
pub mod session;
pub mod transport;

pub use defmt_decoder::DefmtDecoder;
pub use session::{DebugProbeInfo, RttEvent, RttSession};
pub use transport::{RpcClient, RpcResponse};
