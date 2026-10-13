//! Kernel tick load, overruns and lost ticks from SysTick stamps (HCLK)
//! taken at the tick interrupt's entry and exit. Pure arithmetic:
//! `runtime::isr` stamps, keeps the one `TickLoad` and publishes into
//! TELEMETRY `health`: the counters every 20 ticks, the mean every 4000.

use osc_servo_core::kernel::{HEALTH_CLOSE_PHASE, HEALTH_CLOSE_TICKS};

use crate::cfg::chip;
use crate::hal::clocks::HCLK_HZ;

/// Kernel period, SysTick ticks. SysTick and the PWM timer share HCLK, so
/// the period is exact and lateness accumulates without drift.
const PERIOD: u32 = HCLK_HZ / chip::MOTOR_PWM_FREQ_HZ;
const SHORT: u8 = HEALTH_CLOSE_TICKS as u8;
const SHORTS_PER_LONG: u16 = 200;
const LONG: u32 = SHORT as u32 * SHORTS_PER_LONG as u32;
/// Q15 share per SysTick tick of a long window's sum, Q32 so the mean is
/// the product's high word; rounded up so whole periods read exactly
/// 32768.
const MEAN_RECIP: u64 = (32768u64 << 32).div_ceil(LONG as u64 * PERIOD as u64);
/// The count a fresh window starts at: the first close lands on the boot
/// count's first `HEALTH_CLOSE_PHASE` tick (`kernel::health_close_at`).
const FIRST: u8 = SHORT - 1 - HEALTH_CLOSE_PHASE;
/// Bounds the lost-tick loop per short window.
const LOST_MAX: u16 = 16;

const _: () = assert!(SHORT as u32 == HEALTH_CLOSE_TICKS && HEALTH_CLOSE_PHASE < SHORT);
const _: () = assert!(MEAN_RECIP <= u32::MAX as u64);

/// Share of the kernel period a long window's `sum` of SysTick ticks
/// takes, Q15, saturating at 65535.
#[inline(always)]
pub fn mean_q15(sum: u32) -> u16 {
    let q = ((sum as u64 * MEAN_RECIP) >> 32) as u32;
    q.min(u16::MAX as u32) as u16
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

/// One short window.
pub struct Window {
    pub over: u16,
    pub lost: u16,
    /// The mean share of the long window's ticks, on the close that ends
    /// them.
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
            n: FIRST,
            shorts: 0,
            primed: false,
        }
    }

    /// A shunt burst capture runs no kernel tick: the partial short
    /// window's overruns are dropped and the next close only sets the mark.
    /// The tick count carries on, so a close stays on its grid of kernel
    /// ticks (`kernel::health_close_lands`), and so does the long window's
    /// sum.
    #[inline(always)]
    pub fn skip(&mut self) {
        self.primed = false;
        self.over = 0;
    }

    /// Accounts one tick from its entry and exit stamps; the tick that
    /// closes a short window returns it.
    #[inline(always)]
    pub fn tick(&mut self, entry: u32, exit: u32) -> Option<Window> {
        let cost = exit.wrapping_sub(entry);
        self.sum = self.sum.wrapping_add(cost);
        if cost > PERIOD {
            core::hint::cold_path();
            self.over += 1;
        }
        self.n += 1;
        if self.n < SHORT {
            return None;
        }
        core::hint::cold_path();
        Some(self.close(entry))
    }

    /// The gaps since the last close are judged together. The first close
    /// after boot or a `skip` only sets the mark.
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
            let mean = mean_q15(self.sum);
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

    use osc_servo_core::budget::PHASES;
    use osc_servo_core::kernel::health_close_at;

    use super::*;

    const fn tenths(n: u32) -> u32 {
        PERIOD * n / 10
    }

    const N: usize = SHORT as usize;
    /// Ticks in a fresh load's first window.
    const FIRST_TICKS: usize = (SHORT - FIRST) as usize;

    #[test]
    fn period_is_2400_systick_ticks() {
        assert_eq!(PERIOD, 2400);
    }

    /// The published unit is the Q15 share of the period, 32768 a whole
    /// one: a long window of any whole per-tick cost reads that cost's
    /// share exactly, rounded down, up to saturation.
    #[test]
    fn the_mean_reads_the_q15_share_of_the_period() {
        assert_eq!(LONG, 4000);
        for cost in 0..2 * PERIOD {
            assert_eq!(
                mean_q15(cost * LONG),
                (cost * 32768 / PERIOD) as u16,
                "cost {cost}"
            );
        }
        assert_eq!(mean_q15(0), 0);
        assert_eq!(mean_q15(PERIOD / 2 * LONG), 16384);
        assert_eq!(mean_q15(PERIOD * LONG), 32768);
        assert_eq!(mean_q15(2 * PERIOD * LONG), 65535);
        assert_eq!(mean_q15(u32::MAX), 65535);
    }

    /// One short window of `costs` whose closing tick enters at `close_at`;
    /// the entries before it do not matter to the window.
    fn window<const K: usize>(load: &mut TickLoad, close_at: u32, costs: [u32; K]) -> Window {
        let (last, early) = costs.split_last().unwrap();
        for &c in early {
            assert!(load.tick(0, c).is_none());
        }
        load.tick(close_at, close_at.wrapping_add(*last)).unwrap()
    }

    /// A fresh load past its first close, which entered at `t0`.
    fn primed(t0: u32) -> TickLoad {
        let mut load = TickLoad::new();
        window(&mut load, t0, [100; FIRST_TICKS]);
        load
    }

    /// Lost ticks per window for windows closing at `t0 + closes[i]` tenths
    /// of a period, the first a fresh load's.
    fn lost(t0: u32, closes: &[u32]) -> Vec<u16> {
        let mut load = TickLoad::new();
        let mut out = Vec::new();
        for (i, &c) in closes.iter().enumerate() {
            let at = t0.wrapping_add(tenths(c));
            let w = if i == 0 {
                window(&mut load, at, [100; FIRST_TICKS])
            } else {
                window(&mut load, at, [100; N])
            };
            out.push(w.lost);
        }
        out
    }

    #[test]
    fn a_late_close_that_the_next_catches_up_costs_nothing() {
        assert_eq!(lost(0, &[0, 200, 400, 600]), [0, 0, 0, 0]);
        assert_eq!(lost(0, &[0, 207, 400]), [0, 0, 0]);
    }

    #[test]
    fn a_window_a_period_long_holds_a_lost_tick() {
        assert_eq!(lost(0, &[0, 210]), [0, 1]);
        assert_eq!(lost(0, &[0, 210, 420]), [0, 1, 1]);
        assert_eq!(lost(0, &[0, 210, 410]), [0, 1, 0]);
    }

    #[test]
    fn a_long_hold_off_counts_the_bound_and_resyncs() {
        assert_eq!(lost(0, &[0, 400, 600]), [0, LOST_MAX, 0]);
        assert_eq!(catch_up(0, 40 * PERIOD), (0, LOST_MAX));
    }

    #[test]
    fn early_closes_never_bank_credit() {
        assert_eq!(lost(0, &[0, 190, 400]), [0, 0, 1]);
    }

    #[test]
    fn a_window_across_the_systick_wrap() {
        assert_eq!(lost(u32::MAX - 5 * PERIOD, &[0, 210]), [0, 1]);
    }

    #[test]
    fn only_a_tick_past_the_period_counts_over() {
        let mut load = TickLoad::new();
        assert_eq!(window(&mut load, 0, [PERIOD; FIRST_TICKS]).over, 0);
        let mut costs = [100; N];
        costs[3] = PERIOD + 1;
        costs[N - 1] = 5 * PERIOD;
        assert_eq!(window(&mut load, 20 * PERIOD, costs).over, 2);
        let w = window(&mut load, 40 * PERIOD, [100; N]);
        assert_eq!(w.over, 0, "over resets per short window");
    }

    /// The fresh load's first window opens the first long one; every later
    /// mean covers the 4000 ticks of its 200 windows.
    #[test]
    fn the_mean_covers_4000_ticks() {
        let mut load = primed(0);
        let mut s = 1;
        while window(&mut load, s * 20 * PERIOD, [100; N])
            .mean_q15
            .is_none()
        {
            s += 1;
        }
        assert_eq!(s, SHORTS_PER_LONG as u32 - 1);
        let mut means = Vec::new();
        for k in 0..2 * SHORTS_PER_LONG as u32 {
            let cost = if k < 100 { PERIOD / 4 } else { 3 * PERIOD / 4 };
            let w = window(&mut load, (s + 1 + k) * 20 * PERIOD, [cost; N]);
            if let Some(m) = w.mean_q15 {
                means.push((k, m));
            }
        }
        assert_eq!(means, [(199, 16384), (399, 24576)]);
    }

    #[test]
    fn after_a_capture_the_first_window_only_sets_the_mark() {
        let mut load = primed(0);
        window(&mut load, tenths(207), [100; N]);
        for _ in 0..5 {
            load.tick(0, 2 * PERIOD);
        }
        load.skip();
        let w = window(&mut load, 100 * PERIOD, [100; N - 5]);
        assert_eq!(
            (w.over, w.lost),
            (0, 0),
            "the partial window's overruns are gone"
        );
        let w = window(&mut load, 100 * PERIOD + tenths(205), [100; N]);
        assert_eq!(w.lost, 0, "the old debt is gone");
        let w = window(&mut load, 100 * PERIOD + tenths(415), [100; N]);
        assert_eq!(w.lost, 1);
    }

    /// The kernel's executed ticks and this count advance together, a
    /// capture pausing both: whatever captures land, every close lands on
    /// the boot count's PUBLISH ticks, one in twenty, so never on a SLOW,
    /// thermometer or refresh phase.
    #[test]
    fn a_capture_keeps_the_close_on_the_publish_phase() {
        let mut load = TickLoad::new();
        let mut closes = Vec::new();
        for t in 0..200u32 {
            if t % 37 == 5 {
                load.skip();
                load.skip();
            }
            if load.tick(t * PERIOD, t * PERIOD + 100).is_some() {
                closes.push(t);
            }
        }
        let publish = |&t: &u32| t as usize % PHASES == HEALTH_CLOSE_PHASE as usize;
        assert!(closes.iter().all(publish), "{closes:?}");
        assert!(closes.iter().all(|&t| health_close_at(t as u64)));
        assert_eq!(closes.len(), 200 / N);
    }
}
