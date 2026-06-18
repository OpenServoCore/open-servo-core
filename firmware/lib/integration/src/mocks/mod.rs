//! Spy-harness wrappers around `osc-drivers`'s mockall-generated mocks.
//!
//! Each constructor returns `(MockX, XState)`. The `MockX` is the
//! mockall-generated mock from `osc_drivers::mocks`, pre-wired by
//! `mock_x()` so its methods record into the state companion. Tests
//! stage stateful returns via `XState::stage_*` setters and assert the
//! recorded call sequence via `XState::operations()`.
//!
//! Unit tests in `osc-drivers` that prefer mockall's expectation-based
//! API (`m.expect_*().with(...).times(...)`) should construct
//! `MockX::new()` directly instead of going through these helpers — the
//! spy wiring this module installs and per-call expectations don't
//! compose cleanly on the same mock instance.

pub mod bus;
pub mod clock;
pub mod dma;
pub mod primitives;
pub mod rx;
pub mod scheduler;

pub use bus::{TxBusState, mock_tx_bus};
pub use clock::{ClockTrimState, UsartBaudState, mock_clock_trim, mock_usart_baud};
pub use dma::{EdgeDmaState, mock_edge_dma};
pub use primitives::{
    DigitalOutState, MonotonicState, WireClockState, mock_digital_out, mock_monotonic,
    mock_wire_clock,
};
pub use rx::{RxDmaState, mock_rx_dma};
pub use scheduler::{
    FastLastSchedulerState, TxSchedulerState, mock_fast_last_scheduler, mock_tx_scheduler,
};
