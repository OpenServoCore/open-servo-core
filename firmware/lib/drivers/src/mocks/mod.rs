//! mockall-generated mocks for the driver-owned interfaces. Each per-role
//! child module declares one or more `mock!` blocks; associated constants
//! are set inside the impl block per mockall's syntax for traits with
//! consts.
//!
//! `std` is pulled into the crate root behind the same cfg gate that
//! guards this module (see `lib.rs`), so submodules can use `std`-backed
//! types without making the whole crate `std`-bound.
//!
//! Higher-level test ergonomics — state companions, spy harnesses,
//! `mock_x()` constructors — live in the `osc-integration` crate; this
//! module is intentionally opinion-free about how downstream tests
//! consume the mocks.

use dxl_protocol::SoftwareCrcUmts;

use crate::traits::dxl::Providers;

pub mod bus;
pub mod clock;
pub mod dma;
pub mod primitives;
pub mod rx;
pub mod scheduler;
pub mod wire_clock;

pub use bus::{MockTxBus, TxBusOp};
pub use clock::{MockClockTrim, MockUsartBaud};
pub use dma::MockEdgeDma;
pub use primitives::{MockDigitalOut, MockMonotonic};
pub use rx::MockRxDma;
pub use scheduler::{FastLastSchedulerOp, MockFastLastScheduler, MockTxScheduler, ScheduleOp};
pub use wire_clock::MockWireClock;

/// Driver-side `Providers` impl for tests — bundles every mock provider into
/// the single super-trait the [`DxlUart`] composite consumes. Tests
/// instantiate `DxlUart<TestProviders, …>` directly; downstream harnesses
/// reach the mock state through their own wiring (e.g. the integration
/// crate's `mock_x()` constructors).
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
    type WireClock = MockWireClock;
    type Crc = SoftwareCrcUmts;
}
