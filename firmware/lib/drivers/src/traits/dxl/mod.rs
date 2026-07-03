//! DXL-over-UART transport interfaces. Owned by the DXL driver; chip-side
//! providers implement these over real peripherals (production) or
//! recording mocks (tests).

use dxl_protocol::CrcUmts;

mod clock_trim;
mod edge_dma;
mod fast_last_scheduler;
mod rx_dma;
mod tx_bus;
mod tx_scheduler;
mod usart_baud;
mod wire_clock;

pub use clock_trim::ClockTrim;
pub use edge_dma::EdgeDma;
pub use fast_last_scheduler::FastLastScheduler;
pub use rx_dma::{DmaFlags, RxDma};
pub use tx_bus::TxBus;
pub use tx_scheduler::{SendKind, TxScheduler};
pub use usart_baud::UsartBaud;
pub use wire_clock::WireClock;

/// Role-shaped bundle of every chip-side leaf interface
/// [`crate::dxl::uart::DxlUart`] consumes. One associated type per leaf
/// trait below so the composite's signature collapses from nine type
/// parameters to one while each sub-driver
/// ([`Clock`], [`Codec`], [`FastLast`]) stays narrowly typed and still
/// documents exactly what hardware it depends on. Per driver-pattern §5.4.
///
/// Chip-side providers don't implement this trait directly — they
/// implement each leaf trait on their own zero-sized type, and the
/// chip-family crate's `runtime::registry` bundles them into a single
/// `Providers` impl that maps each associated type to the matching ZST.
///
/// [`Clock`]: crate::dxl::uart::clock::Clock
/// [`Codec`]: crate::dxl::uart::codec::Codec
/// [`FastLast`]: crate::dxl::uart::fast_last::FastLast
pub trait Providers {
    type UsartBaud: UsartBaud;
    type ClockTrim: ClockTrim;
    type EdgeDma: EdgeDma;
    type RxDma: RxDma;
    type TxScheduler: TxScheduler;
    type TxBus: TxBus;
    type FastLastScheduler: FastLastScheduler;
    type WireClock: WireClock;
    type Crc: CrcUmts;
}
