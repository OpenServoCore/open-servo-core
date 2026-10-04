//! Shunt zero-current bias tracker.

/// The driver state a zero-current shunt sample was read in. The driver's
/// own supply current returns through the shunt while it is awake, so the
/// two zeros sit apart by a step the servo learns (`BiasTracker`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Zero {
    /// `MotorCmd::Disabled`: the driver asleep, torque off or a fault.
    Asleep,
    /// The driver awake with no winding current crossing the shunt: a
    /// brake, a coast, or a settled Slow-decay brake trough.
    Awake,
}

/// Sub-count bits of the state: the update's dead band is an eighth of a
/// count at `ALPHA_SHIFT`.
const FRAC: u32 = 16;
/// Alpha = 2^-13 per sample: an 8192-tick time constant, 0.41 s at 20 kHz.
/// The offset moves with board temperature, over minutes.
pub const ALPHA_SHIFT: u32 = 13;
/// A zero not fed for this many ticks (3.3 s at 20 kHz, 8 time constants)
/// is stale; ages saturate here.
pub const STALE_TICKS: u16 = u16::MAX;

/// Zero-current output of the current-sense chain, raw ADC counts: two slow
/// EWMAs seeded from the boot rest measurement, the awake zero every
/// current reading subtracts and the asleep zero torque off reads. A sample
/// feeds its own zero. While the other zero is stale it moves by the same
/// delta, so the awake-minus-asleep step carries a drift seen in one state
/// only into the other. While the other is fresh it holds, and the pair
/// learns the step from the two states read at one temperature.
pub struct BiasTracker {
    awake_q: i32,
    asleep_q: i32,
    awake_age: u16,
    asleep_age: u16,
}

impl BiasTracker {
    pub const fn new() -> Self {
        Self {
            awake_q: 0,
            asleep_q: 0,
            awake_age: STALE_TICKS,
            asleep_age: STALE_TICKS,
        }
    }

    /// Both zeros at the boot rest measurement, the step unknown (0).
    pub fn seed(&mut self, counts: u16) {
        self.awake_q = (counts as i32) << FRAC;
        self.asleep_q = self.awake_q;
    }

    /// One tick: `zero` names the state `sample` read zero current in, or
    /// is `None` when the shunt may carry current. Returns the awake zero.
    pub fn update(&mut self, zero: Option<Zero>, sample: u16) -> u16 {
        self.awake_age = self.awake_age.saturating_add(1);
        self.asleep_age = self.asleep_age.saturating_add(1);
        if let Some(zero) = zero {
            let (fed, other, fed_age, other_age) = match zero {
                Zero::Awake => (
                    &mut self.awake_q,
                    &mut self.asleep_q,
                    &mut self.awake_age,
                    self.asleep_age,
                ),
                Zero::Asleep => (
                    &mut self.asleep_q,
                    &mut self.awake_q,
                    &mut self.asleep_age,
                    self.awake_age,
                ),
            };
            let delta = (((sample as i32) << FRAC) - *fed) >> ALPHA_SHIFT;
            *fed += delta;
            if other_age == STALE_TICKS {
                *other += delta;
            }
            *fed_age = 0;
        }
        self.counts()
    }

    /// The awake zero, rounded to nearest.
    pub fn counts(&self) -> u16 {
        ((self.awake_q + (1 << (FRAC - 1))) >> FRAC).max(0) as u16
    }

    #[cfg(test)]
    fn asleep_counts(&self) -> u16 {
        ((self.asleep_q + (1 << (FRAC - 1))) >> FRAC).max(0) as u16
    }
}

impl Default for BiasTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TAU: u32 = 1 << ALPHA_SHIFT;

    fn feed(b: &mut BiasTracker, zero: Option<Zero>, sample: u16, n: u32) -> u16 {
        let mut v = b.counts();
        for _ in 0..n {
            v = b.update(zero, sample);
        }
        v
    }

    #[test]
    fn seeds_both_zeros_from_boot_value() {
        let mut b = BiasTracker::new();
        b.seed(1234);
        assert_eq!(b.counts(), 1234);
        assert_eq!(b.asleep_counts(), 1234);
        assert_eq!(b.update(Some(Zero::Awake), 1234), 1234);
        assert_eq!(b.update(Some(Zero::Asleep), 1234), 1234);
    }

    #[test]
    fn constant_input_steady_state() {
        for zero in [Zero::Awake, Zero::Asleep] {
            let mut b = BiasTracker::new();
            b.seed(2048);
            for _ in 0..4 * TAU {
                assert_eq!(b.update(Some(zero), 2048), 2048);
            }
        }
    }

    #[test]
    fn step_converges_at_the_time_constant() {
        for (from, to) in [(2048u16, 2088u16), (2088, 2048), (0, 2048)] {
            let mut b = BiasTracker::new();
            b.seed(from);
            let v = feed(&mut b, Some(Zero::Awake), to, TAU) as i32;
            // one time constant: 1 - (1 - 2^-13)^8192 = 63.2% of the step
            let expect = from as i32 + ((to as i32 - from as i32) * 632) / 1000;
            assert!((v - expect).abs() <= 1, "{from}->{to}: {v} vs {expect}");
            // 12 tau leaves well under the half count `counts` rounds away
            let v = feed(&mut b, Some(Zero::Awake), to, 11 * TAU);
            assert_eq!(v, to, "{from}->{to} after 12 tau");
        }
    }

    #[test]
    fn a_tick_with_no_zero_holds_both() {
        let mut b = BiasTracker::new();
        b.seed(1000);
        assert_eq!(feed(&mut b, None, 3000, 4 * TAU), 1000);
        assert_eq!(b.asleep_counts(), 1000);
    }

    #[test]
    fn a_stale_zero_moves_with_the_fed_one() {
        let mut b = BiasTracker::new();
        b.seed(1000);
        // never awake: the asleep drift carries the awake zero, step 0
        assert_eq!(feed(&mut b, Some(Zero::Asleep), 900, 12 * TAU), 900);
        assert_eq!(b.asleep_counts(), 900);
        // and a long awake run carries the asleep zero the same way
        let mut b = BiasTracker::new();
        b.seed(1000);
        assert_eq!(feed(&mut b, None, 0, STALE_TICKS as u32), 1000);
        assert_eq!(feed(&mut b, Some(Zero::Awake), 1100, 12 * TAU), 1100);
        assert_eq!(b.asleep_counts(), 1100);
    }

    #[test]
    fn the_step_is_learned_across_a_fresh_transition_and_kept() {
        let mut b = BiasTracker::new();
        b.seed(460);
        // torque off at rest, then awake straight after: the asleep zero
        // is fresh and holds while the awake zero learns 10 over it
        feed(&mut b, Some(Zero::Asleep), 460, 64);
        assert_eq!(feed(&mut b, Some(Zero::Awake), 470, 12 * TAU), 470);
        assert_eq!(b.asleep_counts(), 460);
        // torque off past the stale window while the board warms: the
        // awake zero keeps the step over the drifting rest
        assert_eq!(
            feed(&mut b, Some(Zero::Asleep), 460, STALE_TICKS as u32),
            470
        );
        assert_eq!(feed(&mut b, Some(Zero::Asleep), 260, 12 * TAU), 270);
        assert_eq!(b.asleep_counts(), 260);
    }

    #[test]
    fn a_fresh_awake_zero_holds_while_the_asleep_zero_learns() {
        let mut b = BiasTracker::new();
        b.seed(1000);
        // a long awake run leaves the asleep zero stale at the old value
        feed(&mut b, None, 0, STALE_TICKS as u32);
        assert_eq!(feed(&mut b, Some(Zero::Awake), 900, 12 * TAU), 900);
        // asleep right after: the awake zero holds, the asleep one learns
        assert_eq!(feed(&mut b, Some(Zero::Asleep), 890, 6 * TAU), 900);
        assert_eq!(b.asleep_counts(), 890);
    }

    #[test]
    fn counts_never_wraps_below_zero() {
        let mut b = BiasTracker::new();
        b.seed(5);
        feed(&mut b, Some(Zero::Asleep), 5, 64);
        // the awake zero learns 3 under the asleep one
        feed(&mut b, Some(Zero::Awake), 2, 12 * TAU);
        // then a stale awake zero follows the asleep one down to 0
        feed(&mut b, None, 0, STALE_TICKS as u32);
        assert_eq!(feed(&mut b, Some(Zero::Asleep), 0, 12 * TAU), 0);
    }
}
