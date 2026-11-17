//! The tick's own guard, from the main loop. The tick enters on the
//! injected group's end; a converter or timer that stops ending it runs no
//! tick, and the bridge holds the last compares the kernel wrote. The main
//! loop watches `sample_tick` on every wake and, once it has held still for
//! `STILL_MAX`, writes the drive off through the kernel and latches the
//! sensor fault (`Kernel::sensor_lost`).

use crate::hal::clocks::HCLK_HZ;
use crate::runtime::burst;

/// The longest `sample_tick` may hold still under a running main loop, 4
/// ms. The longest hold that is not a fault is a shunt burst's, under 1.3
/// ms (`burst::UNTICKED_MAX`). A flash SAVE stalls the main loop with the
/// core, and the tick, the highest vector, runs before the main loop does
/// again, so a stall never shows as a hold; nor does a body over the
/// period, which delays the next tick by under a period.
pub const STILL_MAX: u32 = HCLK_HZ / 250;

const _: () = assert!(STILL_MAX >= 3 * burst::UNTICKED_MAX);

pub struct EntryWatch {
    tick: u32,
    since: u32,
    tripped: bool,
}

impl EntryWatch {
    pub const fn new(now: u32, tick: u32) -> Self {
        Self {
            tick,
            since: now,
            tripped: false,
        }
    }

    /// `now` is SysTick read before `tick`, `sample_tick`, so a tick that
    /// lands between the two reads shows as movement, never as a hold.
    /// True once a hold passes `STILL_MAX`, once per hold.
    #[inline(always)]
    pub fn stopped(&mut self, now: u32, tick: u32) -> bool {
        if tick != self.tick {
            self.tick = tick;
            self.since = now;
            self.tripped = false;
            return false;
        }
        if self.tripped || now.wrapping_sub(self.since) <= STILL_MAX {
            return false;
        }
        self.tripped = true;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tick every period never trips, however long it runs, across the
    /// SysTick wrap.
    #[test]
    fn a_ticking_kernel_never_trips() {
        let period = HCLK_HZ / crate::cfg::chip::MOTOR_PWM_FREQ_HZ;
        let t0 = u32::MAX - 100 * period;
        let mut w = EntryWatch::new(t0, 0);
        for k in 1..1000u32 {
            let now = t0.wrapping_add(k * period);
            assert!(!w.stopped(now, k));
            assert!(!w.stopped(now.wrapping_add(period - 1), k));
        }
    }

    /// A burst's hold, and anything up to the bound, is no stop.
    #[test]
    fn a_hold_up_to_the_bound_is_no_stop() {
        let mut w = EntryWatch::new(0, 7);
        assert!(!w.stopped(burst::UNTICKED_MAX, 7));
        assert!(!w.stopped(STILL_MAX, 7));
        assert!(!w.stopped(STILL_MAX + 1, 8));
        assert!(!w.stopped(2 * STILL_MAX + 1, 8));
    }

    /// A hold past the bound trips once; the next tick re-arms the watch,
    /// and a second hold trips again.
    #[test]
    fn a_hold_past_the_bound_trips_once_and_a_tick_rearms_it() {
        let mut w = EntryWatch::new(1000, 7);
        assert!(w.stopped(1001 + STILL_MAX, 7));
        assert!(!w.stopped(1002 + STILL_MAX, 7));
        assert!(!w.stopped(10 * STILL_MAX, 7));
        assert!(!w.stopped(10 * STILL_MAX, 8));
        assert!(w.stopped(11 * STILL_MAX + 1, 8));
    }
}
