//! Kernel tick load, overruns and lost ticks from SysTick stamps (HCLK)
//! taken at the tick interrupt's entry and exit. Pure arithmetic:
//! `runtime::isr` stamps, keeps the one `TickLoad` and publishes into
//! TELEMETRY `health`: the counters every 16 ticks, the mean every 4096.
//! The window's lost count also reaches the kernel (`Kernel::lost_ticks`),
//! whose time-integrating phases span it.

use crate::cfg::chip;
use crate::hal::clocks::HCLK_HZ;

/// Kernel period, SysTick ticks. SysTick and the PWM timer share HCLK, so
/// the period is exact and lateness accumulates without drift.
const PERIOD: u32 = HCLK_HZ / chip::MOTOR_PWM_FREQ_HZ;
const RECIP_SHIFT: u32 = 12;
/// Q15 share per SysTick tick, scaled by `RECIP_SHIFT` and rounded up so a
/// whole period reads exactly 32768.
const RECIP: u32 = (32768u32 << RECIP_SHIFT).div_ceil(PERIOD);
/// Largest cost whose product with `RECIP` fits u32, far past saturation.
const COST_MAX: u32 = u32::MAX / RECIP;
const SHORT_LOG2: u32 = 4;
const SHORT: u8 = 1 << SHORT_LOG2;
const LONG_LOG2: u32 = 12;
const SHORTS_PER_LONG: u16 = 1 << (LONG_LOG2 - SHORT_LOG2);
/// Bounds the lost-tick loop per short window.
const LOST_MAX: u16 = 16;

const _: () = assert!(COST_MAX > 2 * PERIOD);
const _: () = assert!((COST_MAX as u64) << LONG_LOG2 <= u32::MAX as u64);

/// Share of the kernel period that `cost` SysTick ticks take, Q15,
/// saturating at 65535.
#[inline(always)]
pub fn share_q15(cost: u32) -> u16 {
    ((cost.min(COST_MAX) * RECIP) >> RECIP_SHIFT).min(u16::MAX as u32) as u16
}

/// Folds the SysTick ticks `elapsed` between two short closes into `debt`,
/// the lateness behind the earliest phase seen, and returns the new debt
/// and the ticks lost: each whole period of lateness is a tick whose
/// pending flag merged with the next. A hold-off past the bound resyncs to
/// zero debt.
#[inline(always)]
pub fn catch_up(debt: u32, elapsed: u32) -> (u32, u16) {
    let mut debt = debt
        .wrapping_add(elapsed)
        .saturating_sub(SHORT as u32 * PERIOD);
    let mut lost = 0;
    while debt >= PERIOD && lost < LOST_MAX {
        debt -= PERIOD;
        lost += 1;
    }
    if debt >= PERIOD {
        debt = 0;
    }
    (debt, lost)
}

/// One 16-tick window.
pub struct Window {
    pub over: u16,
    pub lost: u16,
    /// The mean share of the 4096 ticks, on the close that ends them.
    pub mean_q15: Option<u16>,
}

pub struct TickLoad {
    /// Entry stamp of the last short window's closing tick.
    mark: u32,
    debt: u32,
    /// Costs of the long window.
    sum: u32,
    over: u16,
    n: u8,
    shorts: u16,
    primed: bool,
}

impl TickLoad {
    pub const fn new() -> Self {
        Self {
            mark: 0,
            debt: 0,
            sum: 0,
            over: 0,
            n: 0,
            shorts: 0,
            primed: false,
        }
    }

    /// A shunt burst capture runs no kernel tick: the partial short window
    /// is dropped so none straddles the capture, and the next close only
    /// sets the mark. The long window carries on, so the dropped ticks'
    /// costs stay in its sum.
    #[inline(always)]
    pub fn skip(&mut self) {
        self.primed = false;
        self.over = 0;
        self.n = 0;
    }

    /// Accounts one tick from its entry and exit stamps; the tick that
    /// closes a short window returns it.
    #[inline(always)]
    pub fn tick(&mut self, entry: u32, exit: u32) -> Option<Window> {
        let cost = exit.wrapping_sub(entry);
        self.sum = self.sum.wrapping_add(cost);
        if cost > PERIOD {
            self.over += 1;
        }
        self.n += 1;
        if self.n < SHORT {
            return None;
        }
        Some(self.close(entry))
    }

    /// The 16 gaps since the last close are judged together. The first
    /// close after boot or a `skip` only sets the mark.
    #[inline(always)]
    fn close(&mut self, entry: u32) -> Window {
        let lost = if self.primed {
            let (debt, lost) = catch_up(self.debt, entry.wrapping_sub(self.mark));
            self.debt = debt;
            lost
        } else {
            self.primed = true;
            self.debt = 0;
            0
        };
        self.mark = entry;
        let over = self.over;
        self.over = 0;
        self.n = 0;
        self.shorts += 1;
        let mean_q15 = if self.shorts == SHORTS_PER_LONG {
            let mean = share_q15(self.sum >> LONG_LOG2);
            self.sum = 0;
            self.shorts = 0;
            Some(mean)
        } else {
            None
        };
        Window {
            over,
            lost,
            mean_q15,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec::Vec;

    use super::*;

    const fn tenths(n: u32) -> u32 {
        PERIOD * n / 10
    }

    #[test]
    fn period_is_2400_systick_ticks() {
        assert_eq!(PERIOD, 2400);
    }

    #[test]
    fn share_spans_zero_to_saturation() {
        assert_eq!(share_q15(0), 0);
        assert_eq!(share_q15(PERIOD / 2), 16384);
        assert_eq!(share_q15(PERIOD), 32768);
        assert_eq!(share_q15(2 * PERIOD - 1), 65523);
        assert_eq!(share_q15(2 * PERIOD), 65535);
        assert_eq!(share_q15(COST_MAX + 1), 65535);
        assert_eq!(share_q15(u32::MAX), 65535);
    }

    /// One short window of `costs` whose closing tick enters at `close_at`;
    /// the entries before it do not matter to the window.
    fn window(load: &mut TickLoad, close_at: u32, costs: [u32; 16]) -> Window {
        let (last, early) = costs.split_last().unwrap();
        for &c in early {
            assert!(load.tick(0, c).is_none());
        }
        load.tick(close_at, close_at.wrapping_add(*last)).unwrap()
    }

    /// Lost ticks per window for windows closing at `t0 + closes[i]` tenths
    /// of a period.
    fn lost(t0: u32, closes: &[u32]) -> Vec<u16> {
        let mut load = TickLoad::new();
        closes
            .iter()
            .map(|&c| window(&mut load, t0.wrapping_add(tenths(c)), [100; 16]).lost)
            .collect()
    }

    #[test]
    fn a_late_close_that_the_next_catches_up_costs_nothing() {
        assert_eq!(lost(0, &[0, 160, 320, 480]), [0, 0, 0, 0]);
        assert_eq!(lost(0, &[0, 167, 320]), [0, 0, 0]);
    }

    #[test]
    fn a_window_a_period_long_holds_a_lost_tick() {
        assert_eq!(lost(0, &[0, 170]), [0, 1]);
        assert_eq!(lost(0, &[0, 170, 340]), [0, 1, 1]);
        assert_eq!(lost(0, &[0, 170, 330]), [0, 1, 0]);
    }

    #[test]
    fn a_long_hold_off_counts_the_bound_and_resyncs() {
        assert_eq!(lost(0, &[0, 360, 520]), [0, LOST_MAX, 0]);
        assert_eq!(catch_up(0, 36 * PERIOD), (0, LOST_MAX));
    }

    #[test]
    fn early_closes_never_bank_credit() {
        assert_eq!(lost(0, &[0, 150, 320]), [0, 0, 1]);
    }

    #[test]
    fn a_window_across_the_systick_wrap() {
        assert_eq!(lost(u32::MAX - 5 * PERIOD, &[0, 170]), [0, 1]);
    }

    #[test]
    fn only_a_tick_past_the_period_counts_over() {
        let mut load = TickLoad::new();
        assert_eq!(window(&mut load, 0, [PERIOD; 16]).over, 0);
        let mut costs = [100; 16];
        costs[3] = PERIOD + 1;
        costs[15] = 5 * PERIOD;
        assert_eq!(window(&mut load, 16 * PERIOD, costs).over, 2);
        let w = window(&mut load, 32 * PERIOD, [100; 16]);
        assert_eq!(w.over, 0, "over resets per short window");
    }

    #[test]
    fn the_mean_covers_4096_ticks() {
        let mut load = TickLoad::new();
        let mut means = Vec::new();
        for s in 0..2 * SHORTS_PER_LONG as u32 {
            let cost = if s < 128 { PERIOD / 4 } else { 3 * PERIOD / 4 };
            let w = window(&mut load, s * 16 * PERIOD, [cost; 16]);
            if let Some(m) = w.mean_q15 {
                means.push((s, m));
            }
        }
        assert_eq!(means, [(255, 16384), (511, 24576)]);
    }

    #[test]
    fn after_a_capture_the_first_window_only_sets_the_mark() {
        let mut load = TickLoad::new();
        window(&mut load, 0, [100; 16]);
        window(&mut load, tenths(167), [100; 16]);
        for _ in 0..5 {
            load.tick(0, 2 * PERIOD);
        }
        load.skip();
        let w = window(&mut load, 100 * PERIOD, [100; 16]);
        assert_eq!((w.over, w.lost), (0, 0), "the partial window is gone");
        let w = window(&mut load, 100 * PERIOD + tenths(165), [100; 16]);
        assert_eq!(w.lost, 0, "the old debt is gone");
        let w = window(&mut load, 100 * PERIOD + tenths(335), [100; 16]);
        assert_eq!(w.lost, 1);
    }
}
