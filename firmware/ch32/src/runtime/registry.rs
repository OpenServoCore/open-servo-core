//! Driver registry — owns the static storage for each driver *instance*
//! and exposes typed accessors. Driver types themselves stay pure (no
//! statics, no `install`, no `get`); the registry is the only place that
//! knows which instance a driver type plays in this build.
//!
//! Adding an instance is a three-line change here (cell, install line,
//! accessor) — the driver type stays untouched.
//!
//! Each instance has its own [`SyncUnsafeCell`], so simultaneous mutable
//! access to two different instances (e.g. main loop touching `stat_led`
//! while a DXL ISR touches `dxl_uart`) doesn't pass through a shared
//! `&mut Drivers` — no aliasing UB.

use core::cell::SyncUnsafeCell;

use osc_drivers::Level;
use osc_drivers::dxl::uart::DxlUart;
use osc_drivers::dxl::uart::clock::Clock;
use osc_drivers::dxl::uart::codec::Codec;
use osc_drivers::dxl::uart::codec::rx::{edge_buf_len, rx_buf_len};
use osc_drivers::dxl::uart::fast_last::FastLast;
use osc_drivers::led::Led;
use osc_drivers::traits::dxl::Providers;

use crate::ConfigDefaults;
use crate::cfg::board_wiring::BoardWiring;
use crate::providers::clock_trim::ClockTrim;
use crate::providers::digital_out::DigitalOut;
use crate::providers::dxl_crc::DxlCrc;
use crate::providers::dxl_tx_bus::DxlTxBus;
use crate::providers::dxl_tx_scheduler::DxlTxScheduler;
use crate::providers::edge_dma::EdgeDma;
use crate::providers::fast_last_scheduler::FastLastScheduler;
use crate::providers::monotonic::Monotonic;
use crate::providers::rx_dma::RxDma;
use crate::providers::usart_baud::UsartBaud;
use crate::providers::wire_clock::WireClock;

/// Concrete instantiations for this chip. The driver types stay generic
/// in `osc-drivers`; this is the single spot that binds them to specific
/// providers and storage sizes. See the `DxlUart` doc for what each const
/// generic means; values track `docs/dxl-hw-timed-transport.md`.
type StatLed = Led<DigitalOut, Monotonic>;

/// Bundle of every chip-side provider the `DxlUart` composite consumes.
/// Maps each `Providers` associated type to its zero-sized provider impl
/// (one per `providers/<role>.rs` file) so the composite collapses to a
/// single `Providers`-bound type parameter per driver-pattern §5.4.
pub struct DxlUartProviders;

impl Providers for DxlUartProviders {
    type UsartBaud = UsartBaud;
    type ClockTrim = ClockTrim;
    type EdgeDma = EdgeDma;
    type RxDma = RxDma;
    type TxScheduler = DxlTxScheduler;
    type TxBus = DxlTxBus;
    type FastLastScheduler = FastLastScheduler;
    type WireClock = WireClock;
    type Crc = DxlCrc;
}

/// Anchor back-search depth target (in edges). Single design knob —
/// `osc_drivers::dxl::uart::codec::rx::{edge_buf_len, rx_buf_len}` derive
/// the matching ring sizes. See the docstring on
/// [`osc_drivers::dxl::uart::codec::rx::sync_lookback_edges`] for the
/// CPU / RAM cost per increment. V006 (48 MHz, 8 KiB SRAM) uses 59 →
/// `EDGE_BUF_LEN = 128`, `RX_BUF_LEN = 64`.
pub(crate) const DXL_SYNC_LOOKBACK_EDGES: u16 = 59;
/// DMA1_CH5 RX-byte ring depth, derived from [`DXL_SYNC_LOOKBACK_EDGES`].
pub(crate) const DXL_RX_BUF_LEN: usize = rx_buf_len(DXL_SYNC_LOOKBACK_EDGES);
/// DMA1_CH7 edge-timestamp ring depth, derived from
/// [`DXL_SYNC_LOOKBACK_EDGES`].
pub(crate) const DXL_EDGE_BUF_LEN: usize = edge_buf_len(DXL_SYNC_LOOKBACK_EDGES);
/// DMA1_CH4 TX-source buffer depth — mirrors
/// `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES` so the driver-owned
/// buffer can hold any Status / Slot reply the dispatcher emits.
pub(crate) const DXL_TX_BUF_LEN: usize = osc_core::services::dxl::limits::DXL_TX_MAX_BYTES;
type DxlUartCh = DxlUart<DxlUartProviders, DXL_RX_BUF_LEN, DXL_EDGE_BUF_LEN, DXL_TX_BUF_LEN>;

struct Cells {
    dbg: SyncUnsafeCell<Option<DigitalOut>>,
    stat_led: SyncUnsafeCell<Option<StatLed>>,
    dxl_uart: SyncUnsafeCell<Option<DxlUartCh>>,
}

static CELLS: Cells = Cells {
    dbg: SyncUnsafeCell::new(None),
    stat_led: SyncUnsafeCell::new(None),
    dxl_uart: SyncUnsafeCell::new(None),
};

pub struct Drivers;

// `SAFETY:` prose in each fn's doc comment is the project convention; the
// markdown-`# Safety`-section form clippy expects would add a one-off style
// island here. Keep the convention; allow the lint.
#[allow(clippy::missing_safety_doc)]
impl Drivers {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly once.
    pub unsafe fn install(w: &BoardWiring, defaults: &ConfigDefaults) {
        // SAFETY: see fn doc.
        let dbg = unsafe { &mut *CELLS.dbg.get() };
        debug_assert!(dbg.is_none(), "Drivers: dbg already installed");
        *dbg = Some(DigitalOut::new(w.dbg.pin(), Level::Low));

        // SAFETY: see fn doc.
        let stat_led = unsafe { &mut *CELLS.stat_led.get() };
        debug_assert!(stat_led.is_none(), "Drivers: stat_led already installed");
        *stat_led = Some(Led::new(
            DigitalOut::new(crate::cfg::chip::STAT_LED_PIN, Level::High),
            Monotonic,
        ));

        // SAFETY: see fn doc.
        let dxl_uart = unsafe { &mut *CELLS.dxl_uart.get() };
        debug_assert!(dxl_uart.is_none(), "Drivers: dxl_uart already installed");
        *dxl_uart = Some(DxlUart::new(
            Codec::new(EdgeDma),
            Clock::new(defaults.dxl_baud, UsartBaud, ClockTrim),
            RxDma,
            DxlTxScheduler::default(),
            DxlTxBus,
            FastLast::new(FastLastScheduler::default()),
            WireClock,
            defaults.dxl_id,
            (defaults.dxl_return_delay_2us as u32) * 2,
        ));
    }

    /// SAFETY: bringup installs `dbg` before any ISR runs; runtime access is
    /// from DXL-side ISRs at PFIC HIGH or main-loop with those IRQs masked.
    ///
    /// Only used under `--features bench`; kept always-available so the API
    /// doesn't change with the feature.
    #[inline(always)]
    #[allow(dead_code)]
    pub unsafe fn dbg() -> &'static mut DigitalOut {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.dbg.get() };
        debug_assert!(cell.is_some(), "Drivers::dbg() before install");
        // SAFETY: bringup ensures Some before any ISR fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup installs `stat_led` before main loop runs; runtime
    /// access is from main-loop callers only.
    #[inline(always)]
    pub unsafe fn stat_led() -> &'static mut StatLed {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.stat_led.get() };
        debug_assert!(cell.is_some(), "Drivers::stat_led() before install");
        // SAFETY: bringup ensures Some before main loop runs.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup installs `dxl_uart` before any IRQ runs; runtime
    /// access is from DMA1_CH7 HT/TC and USART1 IDLE ISRs (both PFIC
    /// HIGH, so same-priority serialization keeps the composite's interior
    /// state race-free).
    #[inline(always)]
    #[allow(dead_code)]
    pub unsafe fn dxl_uart() -> &'static mut DxlUartCh {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.dxl_uart.get() };
        debug_assert!(cell.is_some(), "Drivers::dxl_uart() before install");
        // SAFETY: bringup ensures Some before any IRQ fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }
}
