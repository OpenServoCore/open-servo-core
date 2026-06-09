//! Generic output-pin driver — owns a single push-pull GPIO output.
//!
//! Two static instances live alongside the type:
//!
//! - [`OutputPin::dbg`] — bench scope-marker pin; pulsed from DXL hot path
//!   under `--features bench` via [`crate::bench::dbg_pulse`].
//! - [`OutputPin::stat_led`] — front-panel activity LED; the blink policy
//!   that decides when to switch lives in [`crate::stat_led`].
//!
//! Pure-method API per `docs/driver-pattern.md` §3. `OutputPin::new(...)`
//! always returns a fully-initialized driver; uninit state lives at the
//! static cell as `SyncUnsafeCell<Option<OutputPin>>`.

use core::cell::SyncUnsafeCell;

use crate::hal::Pin;
use crate::hal::gpio::{self, Level, PinMode};

pub struct OutputPin {
    pin: Pin,
}

impl OutputPin {
    pub fn new(pin: Pin, initial: Level) -> Self {
        gpio::configure(pin, PinMode::OUTPUT_PUSH_PULL);
        gpio::set_level(pin, initial);
        Self { pin }
    }

    #[inline(always)]
    pub fn set(&mut self, level: Level) {
        gpio::set_level(self.pin, level);
    }

    /// High then Low, back-to-back. Intended for scope markers with a Low
    /// resting state; callers with inverted polarity should `set` explicitly.
    /// Only used under `--features bench` today; kept always-available so the
    /// API doesn't change with the feature.
    #[inline(always)]
    #[allow(dead_code)]
    pub fn pulse(&mut self) {
        gpio::set_level(self.pin, Level::High);
        gpio::set_level(self.pin, Level::Low);
    }
}

static DBG: SyncUnsafeCell<Option<OutputPin>> = SyncUnsafeCell::new(None);
static STAT_LED: SyncUnsafeCell<Option<OutputPin>> = SyncUnsafeCell::new(None);

impl OutputPin {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly once.
    pub unsafe fn install_dbg(pin: Pin, initial: Level) {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *DBG.get() };
        debug_assert!(cell.is_none(), "OutputPin::DBG already installed");
        *cell = Some(OutputPin::new(pin, initial));
    }

    /// SAFETY: bringup installs DBG before any ISR runs; runtime access is
    /// from DXL-side ISRs at PFIC HIGH or main-loop with those IRQs masked.
    ///
    /// Only used under `--features bench`; kept always-available so the API
    /// doesn't change with the feature.
    #[inline(always)]
    #[allow(dead_code)]
    pub unsafe fn dbg() -> &'static mut Self {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *DBG.get() };
        debug_assert!(cell.is_some(), "OutputPin::dbg() before install");
        // SAFETY: bringup ensures Some before any ISR fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly once.
    pub unsafe fn install_stat_led(pin: Pin, initial: Level) {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *STAT_LED.get() };
        debug_assert!(cell.is_none(), "OutputPin::STAT_LED already installed");
        *cell = Some(OutputPin::new(pin, initial));
    }

    /// SAFETY: bringup installs STAT_LED before main loop starts; runtime
    /// access is from `stat_led::poll` on the main loop only.
    #[inline(always)]
    pub unsafe fn stat_led() -> &'static mut Self {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *STAT_LED.get() };
        debug_assert!(cell.is_some(), "OutputPin::stat_led() before install");
        // SAFETY: bringup ensures Some before main loop runs.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }
}
