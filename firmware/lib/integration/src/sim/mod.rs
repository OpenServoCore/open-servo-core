//! Discrete-event simulator for the DXL bus.
//!
//! Reactor + actor model: `Sim` owns the shared transport (`Bus`) and the
//! device registry, and is the only place that knows everyone. Actors —
//! `Host` and `Servo` — implement the `EventSource` conductor surface and
//! have no compile-time knowledge of each other. `Sim::settle` is the
//! reactor loop; predicate waits (`Host::wait_for_reply`) drive termination
//! instead of wall-clock budgets.

pub mod bus;
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

pub use bus::Bus;
pub use decode::{
    FastStatus, FastStatusCrc, format_hex, parse_fast_bulk_status, parse_fast_sync_status,
    parse_status, parse_status_stream,
};
pub use defaults::{DEFAULT_BAUD, DEFAULT_RDT_NS, DEFAULT_RDT_US};
pub use effect::Effect;
pub use engine::Sim;
pub use host::{HOST_ABSOLUTE_CAP, HOST_FIRST_BYTE_TIMEOUT, HOST_INTER_BYTE_TIMEOUT, Host};
pub use registry::{DeviceId, DeviceRegistry};
pub use servo::{DEFAULT_DXL_ID, DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, Servo};
pub use source::EventSource;
pub use time::{Clock, HsiClock, SimTime};
pub use uart::{
    RxDecoder, RxEffect, RxLogEntry, RxLogKind, TxEncoder, TxLogEntry, UartRx, UartTx,
    bit_period_ns, byte_time_ns,
};
pub use wire::Wire;
