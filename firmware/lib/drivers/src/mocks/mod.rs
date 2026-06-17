//! Recording mocks for driver unit tests and downstream integration crates.
//! Each per-role child module owns one or more of the driver-owned interfaces.
//! Tests assert against the recorded call log on each mock; cross-crate
//! integration tests (under `osc-integration`) consume the same mocks via the
//! `mocks` feature flag.
//!
//! `std` is pulled into the crate root behind the same cfg gate that guards
//! this module (see `lib.rs`), so `Vec` is available in tests and under the
//! `mocks` feature without making the whole crate `std`-bound.

use dxl_protocol::SoftwareCrcUmts;

use crate::traits::dxl::Providers;

pub mod bus;
pub mod clock;
pub mod dma;
pub mod primitives;
pub mod rx;
pub mod scheduler;

pub use bus::{MockTxBus, TxBusOp};
pub use clock::{MockClockTrim, MockUsartBaud};
pub use dma::{EdgeDmaOp, MockEdgeDma};
pub use primitives::{MockDigitalOut, MockMonotonic};
pub use rx::MockRxDma;
pub use scheduler::{FastLastSchedulerOp, MockFastLastScheduler, MockTxScheduler, ScheduleOp};

/// Driver-side `Providers` impl for tests — bundles every mock provider into
/// the single super-trait the [`DxlUart`] composite consumes. Tests
/// instantiate `DxlUart<TestProviders, …>` directly; the mocks' recording
/// state is reached through the composite's accessors / debug `pub` fields
/// (e.g. `bus.scheduler.log`, `bus.fast_last.scheduler().log`).
///
/// [`DxlUart`]: crate::dxl::uart::DxlUart
pub struct TestProviders;

impl Providers for TestProviders {
    type UsartBaud = MockUsartBaud;
    type ClockTrim = MockClockTrim;
    type EdgeDma = MockEdgeDma;
    type RxDma = MockRxDma;
    type TxScheduler = MockTxScheduler;
    type TxBus = MockTxBus;
    type FastLastScheduler = MockFastLastScheduler;
    type Crc = SoftwareCrcUmts;
}
