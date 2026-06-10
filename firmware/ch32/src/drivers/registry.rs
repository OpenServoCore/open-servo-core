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
//! while a DXL ISR touches `dxl_clock`) doesn't pass through a shared
//! `&mut Drivers` — no aliasing UB.

use core::cell::SyncUnsafeCell;

use crate::ConfigDefaults;
use crate::adapters;
use crate::board::BoardWiring;
use crate::drivers::dxl::clock::DxlClock;
use crate::drivers::dxl::rx::DxlRx;
use crate::drivers::led::Led;
use crate::types::Level;

struct Cells {
    dbg: SyncUnsafeCell<Option<adapters::gpio::Output>>,
    stat_led: SyncUnsafeCell<Option<Led>>,
    dxl_clock: SyncUnsafeCell<Option<DxlClock>>,
    dxl_rx: SyncUnsafeCell<Option<DxlRx>>,
}

static CELLS: Cells = Cells {
    dbg: SyncUnsafeCell::new(None),
    stat_led: SyncUnsafeCell::new(None),
    dxl_clock: SyncUnsafeCell::new(None),
    dxl_rx: SyncUnsafeCell::new(None),
};

pub struct Drivers;

impl Drivers {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly once.
    pub unsafe fn install(w: &BoardWiring, defaults: &ConfigDefaults) {
        // SAFETY: see fn doc.
        let dbg = unsafe { &mut *CELLS.dbg.get() };
        debug_assert!(dbg.is_none(), "Drivers: dbg already installed");
        *dbg = Some(adapters::gpio::Output::new(w.dbg, Level::Low));

        // SAFETY: see fn doc.
        let stat_led = unsafe { &mut *CELLS.stat_led.get() };
        debug_assert!(stat_led.is_none(), "Drivers: stat_led already installed");
        *stat_led = Some(Led::new(
            adapters::gpio::Output::new(w.stat_led, Level::High),
            adapters::systick::SysTick,
        ));

        // SAFETY: see fn doc.
        let dxl_clock = unsafe { &mut *CELLS.dxl_clock.get() };
        debug_assert!(dxl_clock.is_none(), "Drivers: dxl_clock already installed");
        *dxl_clock = Some(DxlClock::new(
            defaults.dxl_baud,
            adapters::usart::Usart1,
            adapters::rcc::HsiTrim,
        ));

        // SAFETY: see fn doc.
        let dxl_rx = unsafe { &mut *CELLS.dxl_rx.get() };
        debug_assert!(dxl_rx.is_none(), "Drivers: dxl_rx already installed");
        *dxl_rx = Some(DxlRx::new(adapters::dma::Ch7));
    }

    /// SAFETY: bringup installs `dbg` before any ISR runs; runtime access is
    /// from DXL-side ISRs at PFIC HIGH or main-loop with those IRQs masked.
    ///
    /// Only used under `--features bench`; kept always-available so the API
    /// doesn't change with the feature.
    #[inline(always)]
    #[allow(dead_code)]
    pub unsafe fn dbg() -> &'static mut adapters::gpio::Output {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.dbg.get() };
        debug_assert!(cell.is_some(), "Drivers::dbg() before install");
        // SAFETY: bringup ensures Some before any ISR fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup installs `stat_led` before main loop runs; runtime
    /// access is from main-loop callers only.
    #[inline(always)]
    pub unsafe fn stat_led() -> &'static mut Led {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.stat_led.get() };
        debug_assert!(cell.is_some(), "Drivers::stat_led() before install");
        // SAFETY: bringup ensures Some before main loop runs.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup installs `dxl_clock` before any IRQ runs; runtime
    /// access is from DXL-side ISRs at PFIC HIGH (same-priority serialization).
    #[inline(always)]
    #[allow(dead_code)]
    pub unsafe fn dxl_clock() -> &'static mut DxlClock {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.dxl_clock.get() };
        debug_assert!(cell.is_some(), "Drivers::dxl_clock() before install");
        // SAFETY: bringup ensures Some before any IRQ fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup installs `dxl_rx` before any IRQ runs; runtime access
    /// is from DMA1_CH7 HT/TC and USART1 IDLE ISRs (both PFIC HIGH, so
    /// same-priority serialization keeps the classifier's interior state
    /// race-free).
    #[inline(always)]
    #[allow(dead_code)]
    pub unsafe fn dxl_rx() -> &'static mut DxlRx {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.dxl_rx.get() };
        debug_assert!(cell.is_some(), "Drivers::dxl_rx() before install");
        // SAFETY: bringup ensures Some before any IRQ fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }
}
