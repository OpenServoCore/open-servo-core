//! Generic status-LED driver — drives a single output pin in one of a few
//! patterns. Mechanism only; no knowledge of *what* signals it represents.
//!
//! Callers set the pattern with [`StatLed::set_pattern`] when their model
//! changes; the main loop calls [`StatLed::poll`] every iteration to advance
//! the pin level by elapsed time.

use core::cell::SyncUnsafeCell;

use crate::drivers::output_pin::OutputPin;
use crate::hal::Pin;
use crate::hal::gpio::Level;
use crate::hal::systick::{self, TICKS_PER_US};

/// What the LED should be doing.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Pattern {
    SolidOn,
    #[allow(dead_code)]
    SolidOff,
    /// Symmetric 50%-duty blink at the given full-cycle period.
    Blink {
        period_us: u32,
    },
}

pub struct StatLed {
    pin: OutputPin,
    pattern: Pattern,
    phase_tick: u32,
    on: bool,
}

impl StatLed {
    pub fn new(pin: Pin) -> Self {
        Self {
            pin: OutputPin::new(pin, Level::High),
            pattern: Pattern::SolidOn,
            phase_tick: 0,
            on: true,
        }
    }

    /// Change the pattern. Resets the phase so the new pattern's first
    /// half-cycle starts now; a no-op if the pattern is already current.
    pub fn set_pattern(&mut self, pattern: Pattern) {
        if self.pattern != pattern {
            self.pattern = pattern;
            self.phase_tick = systick::ticks();
        }
    }

    /// Main-loop tick. Updates the pin level if the current pattern + elapsed
    /// time call for a transition; cheap no-op otherwise.
    pub fn poll(&mut self) {
        let now = systick::ticks();
        let target_on = match self.pattern {
            Pattern::SolidOn => true,
            Pattern::SolidOff => false,
            Pattern::Blink { period_us } => {
                let half_period_ticks = (period_us / 2) * TICKS_PER_US;
                if now.wrapping_sub(self.phase_tick) >= half_period_ticks {
                    self.phase_tick = now;
                    !self.on
                } else {
                    self.on
                }
            }
        };
        if target_on != self.on {
            self.pin
                .set(if target_on { Level::High } else { Level::Low });
            self.on = target_on;
        }
    }
}

static STAT_LED: SyncUnsafeCell<Option<StatLed>> = SyncUnsafeCell::new(None);

impl StatLed {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly once.
    pub unsafe fn install(pin: Pin) {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *STAT_LED.get() };
        debug_assert!(cell.is_none(), "StatLed: already installed");
        *cell = Some(StatLed::new(pin));
    }

    /// SAFETY: bringup installs StatLed before main loop runs; runtime
    /// access is from main-loop callers only.
    #[inline(always)]
    pub unsafe fn get() -> &'static mut Self {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *STAT_LED.get() };
        debug_assert!(cell.is_some(), "StatLed accessed before install");
        // SAFETY: bringup ensures Some before main loop runs.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }
}
