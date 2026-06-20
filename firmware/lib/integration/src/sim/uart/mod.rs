//! Per-bit UART line model in two layers.
//!
//! Layer 1 — pure transforms over wire-level events:
//!   * [`TxEncoder`] — byte → 10-bit-frame level transitions
//!   * [`RxDecoder`] — edges + internal ticks → [`RxEffect`]s
//!
//! Layer 2 — scheduled ports devices plug into the simulator:
//!   * [`UartTx`] — owns a [`TxEncoder`], a pending-byte queue, and a TX log
//!   * [`UartRx`] — owns a [`RxDecoder`], a pending-edge queue, and an RX log
//!
//! Layer 2 is what `Host` and (later) `Servo` consume. Layer 1 stays usable
//! directly for unit tests that drive single events.
//!
//! A UART frame is 10 bits: a falling start bit, 8 LSB-first data bits, and
//! a rising stop bit. Idle level is high. The wire carries level transitions
//! only — consecutive same-level bits emit no edge.

pub mod rx;
pub mod rx_decoder;
pub mod tx;
pub mod tx_encoder;
pub mod utils;

pub use rx::{RxLogEntry, RxLogKind, UartRx};
pub use rx_decoder::{RxDecoder, RxEffect};
pub use tx::{TxLogEntry, UartTx};
pub use tx_encoder::TxEncoder;
pub use utils::{bit_period_ns, byte_time_ns};
