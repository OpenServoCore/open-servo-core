//! Discrete-event simulator for the DXL bus.
//!
//! Each device owns its own clock and queue, advertises its next scheduled
//! event time, and on `advance` returns `Effect`s the engine routes back to
//! peers. `Wire` is the delivery medium: it routes per-bit edges between
//! devices in [`DeviceRegistry`].

pub mod decode;
pub mod defaults;
pub mod effect;
pub mod engine;
pub mod host;
pub mod registry;
pub mod servo;
pub mod source;
pub mod time;
pub mod uart;
pub mod wire;

pub use decode::{
    FastStatus, FastStatusCrc, format_hex, parse_fast_bulk_status, parse_fast_sync_status,
    parse_status, parse_status_stream,
};
pub use effect::Effect;
pub use engine::Sim;
pub use host::Host;
pub use registry::{DeviceId, DeviceRegistry};
pub use servo::{
    DEFAULT_DXL_ID, DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, DEFAULT_RDT_US, Servo,
};
pub use source::EventSource;
pub use time::{Clock, SimTime};
pub use uart::{RxDecoder, RxEffect, RxLogEntry, RxLogKind, TxEncoder, TxLogEntry, UartRx, UartTx};
pub use wire::Wire;
