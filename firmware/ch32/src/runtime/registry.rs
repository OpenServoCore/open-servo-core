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
use osc_drivers::led::Led;

use crate::ConfigDefaults;
use crate::cfg::board_wiring::BoardWiring;
use crate::providers;

/// Concrete instantiations for this chip. The driver types stay generic
/// in `osc-drivers`; this is the single spot that binds them to specific
/// providers and storage sizes. See the `DxlUart` doc for what each const
/// generic means; values track `docs/dxl-hw-timed-transport.md`.
type StatLed = Led<providers::digital_out::DigitalOut, providers::monotonic::Monotonic>;
/// Streaming-decoder accumulator size — `osc-core`'s dispatcher uses this
/// value for its sibling decoder (`services::dxl::api::DXL_DECODER_CAP`).
/// Keep them in sync.
pub(crate) const DXL_DECODER_CAP: usize = 256;
/// DMA1_CH5 RX-byte ring depth (doc §8.1) — also the BT-ring depth inside
/// the RX sub-driver (doc §8.3 requires them to match).
pub(crate) const DXL_RX_BUF_LEN: usize = 64;
/// DMA1_CH7 edge-timestamp ring depth — option A in doc §8.4.
pub(crate) const DXL_EDGE_BUF_LEN: usize = 128;
type DxlUartCh = DxlUart<
    providers::usart_baud::UsartBaud,
    providers::clock_trim::ClockTrim,
    providers::dma_ring::DmaRing,
    providers::dxl_crc::DxlCrc,
    DXL_DECODER_CAP,
    DXL_RX_BUF_LEN,
    DXL_EDGE_BUF_LEN,
>;

struct Cells {
    dbg: SyncUnsafeCell<Option<providers::digital_out::DigitalOut>>,
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
        *dbg = Some(providers::digital_out::DigitalOut::new(w.dbg, Level::Low));

        // SAFETY: see fn doc.
        let stat_led = unsafe { &mut *CELLS.stat_led.get() };
        debug_assert!(stat_led.is_none(), "Drivers: stat_led already installed");
        *stat_led = Some(Led::new(
            providers::digital_out::DigitalOut::new(w.stat_led, Level::High),
            providers::monotonic::Monotonic,
        ));

        // SAFETY: see fn doc.
        let dxl_uart = unsafe { &mut *CELLS.dxl_uart.get() };
        debug_assert!(dxl_uart.is_none(), "Drivers: dxl_uart already installed");
        *dxl_uart = Some(DxlUart::new(
            Codec::new(providers::dma_ring::DmaRing),
            Clock::new(
                defaults.dxl_baud,
                providers::usart_baud::UsartBaud,
                providers::clock_trim::ClockTrim,
            ),
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
    pub unsafe fn dbg() -> &'static mut providers::digital_out::DigitalOut {
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
