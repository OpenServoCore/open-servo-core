//! Generic LED driver -- drives a single output pin in one of a few
//! patterns. Mechanism only; no knowledge of *what* signals it represents.
//!
//! Callers set the pattern with [`Led::set_pattern`] when their model
//! changes; the main loop calls [`Led::poll`] every iteration to advance
//! the pin level by elapsed time.

use crate::traits::{DigitalOut, Level, Monotonic};

/// What the LED should be doing.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Pattern {
    SolidOn,
    SolidOff,
    /// Symmetric 50%-duty blink at the given full-cycle period.
    Blink {
        period_us: u32,
    },
}

pub struct Led<P: DigitalOut, M: Monotonic> {
    pin: P,
    clock: M,
    pattern: Pattern,
    phase_tick: u32,
    on: bool,
    active: Level,
}

impl<P: DigitalOut, M: Monotonic> Led<P, M> {
    /// `active` is the level that lights the LED; `pin` must already sit at
    /// the other one, since the driver starts dark and writes only on change.
    pub fn new(pin: P, clock: M, active: Level) -> Self {
        Self {
            pin,
            clock,
            pattern: Pattern::SolidOff,
            phase_tick: 0,
            on: false,
            active,
        }
    }

    /// Change the pattern. Resets the phase so the new pattern's first
    /// half-cycle starts now; a no-op if the pattern is already current.
    pub fn set_pattern(&mut self, pattern: Pattern) {
        if self.pattern != pattern {
            self.pattern = pattern;
            self.phase_tick = self.clock.ticks();
        }
    }

    /// Main-loop tick. Updates the pin level if the current pattern + elapsed
    /// time call for a transition; cheap no-op otherwise.
    pub fn poll(&mut self) {
        let now = self.clock.ticks();
        let target_on = match self.pattern {
            Pattern::SolidOn => true,
            Pattern::SolidOff => false,
            Pattern::Blink { period_us } => {
                let half_period_ticks = (period_us / 2) * M::TICKS_PER_US;
                if now.wrapping_sub(self.phase_tick) >= half_period_ticks {
                    self.phase_tick = now;
                    !self.on
                } else {
                    self.on
                }
            }
        };
        if target_on != self.on {
            self.pin.set(if target_on {
                self.active
            } else {
                self.active.inverted()
            });
            self.on = target_on;
        }
    }
}

#[cfg(test)]
impl<P: DigitalOut, M: Monotonic> Led<P, M> {
    pub(crate) fn phase_tick_for_test(&self) -> u32 {
        self.phase_tick
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::mocks::{MockDigitalOut, MockMonotonic};
    use alloc::vec::Vec;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    // MockMonotonic uses 1 tick = 1 us, so test arithmetic stays in us.

    #[derive(Clone, Default)]
    struct PinState {
        log: Rc<RefCell<Vec<Level>>>,
    }
    impl PinState {
        fn log(&self) -> Vec<Level> {
            self.log.borrow().clone()
        }
    }
    fn mk_pin() -> (MockDigitalOut, PinState) {
        let state = PinState::default();
        let mut m = MockDigitalOut::new();
        {
            let log = state.log.clone();
            m.expect_set()
                .returning_st(move |lv| log.borrow_mut().push(lv));
        }
        (m, state)
    }

    #[derive(Clone, Default)]
    struct ClockState {
        now: Rc<Cell<u32>>,
    }
    impl ClockState {
        fn set_now(&self, v: u32) {
            self.now.set(v);
        }
    }
    fn mk_clock() -> (MockMonotonic, ClockState) {
        let state = ClockState::default();
        let mut m = MockMonotonic::new();
        {
            let n = state.now.clone();
            m.expect_ticks().returning_st(move || n.get());
        }
        (m, state)
    }

    fn led_with(
        pattern: Pattern,
        active: Level,
    ) -> (Led<MockDigitalOut, MockMonotonic>, PinState, ClockState) {
        let (pin, pin_state) = mk_pin();
        let (clock, clock_state) = mk_clock();
        let mut led = Led::new(pin, clock, active);
        led.set_pattern(pattern);
        (led, pin_state, clock_state)
    }

    #[test]
    fn solid_off_leaves_the_parked_pin_alone() {
        let (mut led, pin, _clock) = led_with(Pattern::SolidOff, Level::High);
        led.poll();
        assert!(pin.log().is_empty());
    }

    #[test]
    fn solid_on_drives_the_active_level() {
        let (mut led, pin, _clock) = led_with(Pattern::SolidOn, Level::High);
        led.poll();
        assert_eq!(pin.log(), [Level::High]);
        let (mut led, pin, _clock) = led_with(Pattern::SolidOn, Level::Low);
        led.poll();
        assert_eq!(pin.log(), [Level::Low]);
    }

    #[test]
    fn blink_toggles_after_half_period() {
        let period_us = 100;
        let half = period_us / 2;
        let (mut led, pin, clock) = led_with(Pattern::Blink { period_us }, Level::Low);
        // Just before half period: no toggle yet.
        clock.set_now(half - 1);
        led.poll();
        assert!(pin.log().is_empty());
        // At the boundary: lit, at the active-low level.
        clock.set_now(half);
        led.poll();
        assert_eq!(pin.log(), [Level::Low]);
        // Second half later: dark again.
        clock.set_now(2 * half);
        led.poll();
        assert_eq!(pin.log(), [Level::Low, Level::High]);
    }

    #[test]
    fn set_pattern_same_value_is_noop() {
        let (mut led, _pin, clock) = led_with(Pattern::SolidOn, Level::High);
        clock.set_now(12345);
        let phase_before = led.phase_tick_for_test();
        led.set_pattern(Pattern::SolidOn);
        assert_eq!(led.phase_tick_for_test(), phase_before);
    }
}
