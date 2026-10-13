//! OpenLoop duty ceiling (control-theory "Limits"): current limiting for the
//! one mode with no current loop, from the shunt alone - no gains, no
//! identified constant, no multiply. One state variable, three regions:
//! under `lim - lim/8` the ceiling slews up at `UP_Q15` per tick, inside the
//! band `[lim - lim/8, lim]` it holds (no limit cycle), over `lim` it drops
//! by the overage times `1 << DOWN_SHIFT`. A goal cut applies at once. A
//! ceiling under the shunt window floor applies `base` instead: below the
//! floor the shunt sees nothing, so the ceiling cannot be trusted there.

/// Ceiling rise per FAST tick, Q15 (0.39 %/tick).
pub const UP_Q15: u16 = 128;
/// Ceiling drop per count over the limit, as a shift: `over << DOWN_SHIFT`.
pub const DOWN_SHIFT: u32 = 3;
/// The ceiling rises only below `lim - (lim >> BAND_SHIFT)`.
pub const BAND_SHIFT: u32 = 3;

pub struct DutyLimiter {
    ceil_q15: u16,
    /// A drop held the ceiling under the goal since the last `take_pinned`.
    pinned: bool,
}

impl DutyLimiter {
    pub const fn new() -> Self {
        Self {
            ceil_q15: 0,
            pinned: false,
        }
    }

    pub fn reset(&mut self, floor_q15: u16) {
        self.ceil_q15 = floor_q15;
        self.pinned = false;
    }

    /// One FAST tick; returns the duty magnitude to apply. `i_abs` is the
    /// window-valid current magnitude, 0 for an invalid window (the same
    /// honest zero the current loop uses). A goal at or under the floor
    /// comes back unchanged while `base_q15 >= floor_q15`.
    pub fn step(
        &mut self,
        goal_mag: u16,
        i_abs: u32,
        lim: u32,
        floor_q15: u16,
        base_q15: u16,
    ) -> u16 {
        if i_abs < lim - (lim >> BAND_SHIFT) {
            self.ceil_q15 = self.ceil_q15.saturating_add(UP_Q15);
        } else {
            let over = i_abs
                .saturating_sub(lim)
                .min((u16::MAX >> DOWN_SHIFT) as u32);
            self.ceil_q15 = self.ceil_q15.saturating_sub((over << DOWN_SHIFT) as u16);
            self.pinned |= self.ceil_q15 < goal_mag;
        }
        self.ceil_q15 = self.ceil_q15.min(goal_mag);
        if self.ceil_q15 < floor_q15 {
            base_q15.min(goal_mag)
        } else {
            self.ceil_q15
        }
    }

    pub fn take_pinned(&mut self) -> bool {
        core::mem::take(&mut self.pinned)
    }
}

impl Default for DutyLimiter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FLOOR: u16 = 4356;
    const LIM: u32 = 280;
    const GOAL: u16 = 20971;

    fn at_floor() -> DutyLimiter {
        let mut l = DutyLimiter::new();
        l.reset(FLOOR);
        l
    }

    #[test]
    fn blind_goal_passes_raw() {
        let mut l = at_floor();
        for goal in [0, 1, 3000, FLOOR - 1, FLOOR] {
            assert_eq!(l.step(goal, 0, LIM, FLOOR, FLOOR), goal);
            assert_eq!(l.step(goal, 1000, LIM, FLOOR, FLOOR), goal);
        }
        assert_eq!(l.step(3000, 1000, 0, FLOOR, FLOOR), 3000);
    }

    #[test]
    fn ceiling_rises_at_the_up_rate_from_the_floor() {
        let mut l = at_floor();
        assert_eq!(l.step(GOAL, 0, LIM, FLOOR, FLOOR), FLOOR + UP_Q15);
        assert_eq!(l.step(GOAL, 100, LIM, FLOOR, FLOOR), FLOOR + 2 * UP_Q15);
        let band_lo = LIM - (LIM >> BAND_SHIFT);
        assert_eq!(
            l.step(GOAL, band_lo - 1, LIM, FLOOR, FLOOR),
            FLOOR + 3 * UP_Q15
        );
        let mut d = 0;
        for _ in 0..200 {
            d = l.step(GOAL, 0, LIM, FLOOR, FLOOR);
        }
        assert_eq!(d, GOAL, "the rise ends at the goal");
    }

    #[test]
    fn ceiling_holds_inside_the_band() {
        let mut l = at_floor();
        let d = l.step(GOAL, 0, LIM, FLOOR, FLOOR);
        let band_lo = LIM - (LIM >> BAND_SHIFT);
        for i in [band_lo, 260, LIM] {
            assert_eq!(l.step(GOAL, i, LIM, FLOOR, FLOOR), d);
        }
    }

    #[test]
    fn ceiling_drops_by_the_overage() {
        let mut l = at_floor();
        for _ in 0..20 {
            l.step(GOAL, 0, LIM, FLOOR, FLOOR);
        }
        let d = l.step(GOAL, LIM, LIM, FLOOR, FLOOR);
        assert_eq!(l.step(GOAL, LIM + 10, LIM, FLOOR, FLOOR), d - 80);
        assert!(l.take_pinned());
    }

    #[test]
    fn goal_cut_applies_at_once() {
        let mut l = at_floor();
        for _ in 0..50 {
            l.step(GOAL, 0, LIM, FLOOR, FLOOR);
        }
        assert_eq!(l.step(8000, 0, LIM, FLOOR, FLOOR), 8000);
        assert_eq!(
            l.step(GOAL, 0, LIM, FLOOR, FLOOR),
            8000 + UP_Q15,
            "a raised goal slews from the cut"
        );
    }

    #[test]
    fn zero_limit_never_rises() {
        let mut l = at_floor();
        for _ in 0..10 {
            assert_eq!(l.step(GOAL, 0, 0, FLOOR, FLOOR), FLOOR);
        }
        assert_eq!(
            l.step(GOAL, 5, 0, FLOOR, FLOOR),
            FLOOR,
            "under the floor: base"
        );
    }

    #[test]
    fn ceiling_under_the_floor_applies_the_base() {
        let mut l = at_floor();
        let base = 3000;
        assert_eq!(l.step(GOAL, LIM + 100, LIM, FLOOR, base), base);
        assert_eq!(
            l.step(2000, LIM + 100, LIM, FLOOR, base),
            2000,
            "base never exceeds the goal"
        );
        let mut d = 0;
        for _ in 0..20 {
            d = l.step(GOAL, 0, LIM, FLOOR, base);
        }
        assert!(
            d > FLOOR,
            "the rise back past the floor is the re-probe: {d}"
        );
    }

    #[test]
    fn slew_alone_is_never_pinned() {
        let mut l = at_floor();
        for _ in 0..200 {
            l.step(GOAL, 0, LIM, FLOOR, FLOOR);
        }
        assert!(!l.take_pinned());
        l.step(GOAL, LIM, LIM, FLOOR, FLOOR);
        assert!(!l.take_pinned(), "the goal reached, a hold is not a pin");
        l.step(GOAL, LIM + 1, LIM, FLOOR, FLOOR);
        assert!(l.take_pinned());
        assert!(!l.take_pinned(), "take clears");
    }
}
