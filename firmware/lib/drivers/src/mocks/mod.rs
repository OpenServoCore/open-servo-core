//! mockall-generated mocks for the driver-owned interfaces. One per-role
//! child module per trait file (mirroring `traits/` and `traits/dxl/`),
//! each declaring one `mock!` block plus its op-log enum where the role
//! has one; associated constants are set inside the impl block per
//! mockall's syntax for traits with consts.
//!
//! `std` is pulled into the crate root behind the same cfg gate that
//! guards this module (see `lib.rs`), so submodules can use `std`-backed
//! types without making the whole crate `std`-bound.
//!
//! Higher-level test ergonomics — state companions, spy harnesses,
//! `mock_x()` constructors — live in `dxl::uart::test_support` (this
//! crate) and the `osc-integration` crate; this module is intentionally
//! opinion-free about how downstream tests consume the mocks.

use dxl_protocol::SoftwareCrcUmts;

use crate::traits::dxl::Providers;

pub mod clock_trim;
pub mod digital_out;
pub mod fast_last_scheduler;
pub mod monotonic;
pub mod rx_dma;
pub mod telemetry;
pub mod tx_bus;
pub mod tx_scheduler;
pub mod usart_baud;
pub mod wire_clock;

pub use clock_trim::MockClockTrim;
pub use digital_out::MockDigitalOut;
pub use fast_last_scheduler::{FastLastSchedulerOp, MockFastLastScheduler};
pub use monotonic::MockMonotonic;
pub use rx_dma::MockRxDma;
pub use telemetry::MockTelemetry;
pub use tx_bus::{MockTxBus, TxBusOp};
pub use tx_scheduler::{MockTxScheduler, ScheduleOp};
pub use usart_baud::MockUsartBaud;
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
    type RxDma = MockRxDma;
    type TxScheduler = MockTxScheduler;
    type TxBus = MockTxBus;
    type FastLastScheduler = MockFastLastScheduler;
    type WireClock = MockWireClock;
    type Telemetry = MockTelemetry;
    type Crc = SoftwareCrcUmts;
}
