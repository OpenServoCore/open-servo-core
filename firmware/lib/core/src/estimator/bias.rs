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

/// Sub-count bits of the state: the update's dead band is a 64th of a
/// count at `ALPHA_SHIFT`.
const FRAC: u32 = 16;
/// Alpha = 2^-10 per sample, one sample a control period: a 1024-period
/// time constant, 0.51 s at 2 kHz. The offset moves with board
/// temperature, over minutes.
pub const ALPHA_SHIFT: u32 = 10;
/// A zero not fed for this many SLOW periods (3.3 s at 62.5 Hz, 6 time
/// constants) is stale.
pub const STALE_PERIODS: u8 = 205;

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
    /// SLOW periods left before each zero is stale, 0 once it is; a feed
    /// resets its own to `STALE_PERIODS`.
    awake_fresh: u8,
    asleep_fresh: u8,
}

impl BiasTracker {
    pub const fn new() -> Self {
        Self {
            awake_q: 0,
            asleep_q: 0,
            awake_fresh: 0,
            asleep_fresh: 0,
        }
    }

    /// Both zeros at the boot rest measurement, the step unknown (0).
    pub fn seed(&mut self, counts: u16) {
        self.awake_q = (counts as i32) << FRAC;
        self.asleep_q = self.awake_q;
    }

    /// One sample, at most one a control period: `zero` names the state
    /// `sample` read zero current in. A period whose shunt may carry
    /// current feeds nothing. Returns the awake zero.
    pub fn update(&mut self, zero: Zero, sample: u16) -> u16 {
        match zero {
            Zero::Awake => feed(
                sample,
                &mut self.awake_q,
                &mut self.awake_fresh,
                &mut self.asleep_q,
                self.asleep_fresh,
            ),
            Zero::Asleep => feed(
                sample,
                &mut self.asleep_q,
                &mut self.asleep_fresh,
                &mut self.awake_q,
                self.awake_fresh,
            ),
        }
        self.counts()
    }

    /// One aging step of both zeros, once a SLOW period, fed or not.
    pub fn age(&mut self) {
        self.awake_fresh = self.awake_fresh.saturating_sub(1);
        self.asleep_fresh = self.asleep_fresh.saturating_sub(1);
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

/// One sample into `fed`; `other` follows while stale. Each `update` arm
/// inlines this against its own fields, so the feed addresses them
/// directly instead of through a pair of pointers picked by the match.
#[inline(always)]
fn feed(sample: u16, fed: &mut i32, fed_fresh: &mut u8, other: &mut i32, other_fresh: u8) {
    let delta = (((sample as i32) << FRAC) - *fed) >> ALPHA_SHIFT;
    *fed += delta;
    if other_fresh == 0 {
        *other += delta;
    }
    *fed_fresh = STALE_PERIODS;
}

impl Default for BiasTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::DECIM_SLOW;

    /// One time constant, in samples.
    const TAU: u32 = 1 << ALPHA_SHIFT;

    /// `n` samples of `sample` into `zero`, none of them aging.
    fn feed(b: &mut BiasTracker, zero: Zero, sample: u16, n: u32) -> u16 {
        let mut v = b.counts();
        for _ in 0..n {
            v = b.update(zero, sample);
        }
        v
    }

    /// `n` SLOW periods with no sample.
    fn age(b: &mut BiasTracker, n: u32) {
        for _ in 0..n {
            b.age();
        }
    }

    /// `n` SLOW periods of `sample` into `zero` every control period, as
    /// the kernel runs the tracker.
    fn run(b: &mut BiasTracker, zero: Zero, sample: u16, n: u32) -> u16 {
        for _ in 0..n {
            feed(b, zero, sample, DECIM_SLOW as u32);
            b.age();
        }
        b.counts()
    }

    #[test]
    fn seeds_both_zeros_from_boot_value() {
        let mut b = BiasTracker::new();
        b.seed(1234);
        assert_eq!(b.counts(), 1234);
        assert_eq!(b.asleep_counts(), 1234);
        assert_eq!(b.update(Zero::Awake, 1234), 1234);
        assert_eq!(b.update(Zero::Asleep, 1234), 1234);
    }

    #[test]
    fn constant_input_steady_state() {
        for zero in [Zero::Awake, Zero::Asleep] {
            let mut b = BiasTracker::new();
            b.seed(2048);
            for _ in 0..4 * TAU {
                assert_eq!(b.update(zero, 2048), 2048);
            }
        }
    }

    #[test]
    fn step_converges_at_the_time_constant() {
        for (from, to) in [(2048u16, 2088u16), (2088, 2048), (0, 2048)] {
            let mut b = BiasTracker::new();
            b.seed(from);
            let v = feed(&mut b, Zero::Awake, to, TAU) as i32;
            // one time constant: 1 - (1 - 2^-10)^1024 = 63.2% of the step
            let expect = from as i32 + ((to as i32 - from as i32) * 632) / 1000;
            assert!((v - expect).abs() <= 1, "{from}->{to}: {v} vs {expect}");
            // 12 tau leaves well under the half count `counts` rounds away
            let v = feed(&mut b, Zero::Awake, to, 11 * TAU);
            assert_eq!(v, to, "{from}->{to} after 12 tau");
        }
    }

    #[test]
    fn aging_alone_holds_both_zeros() {
        let mut b = BiasTracker::new();
        b.seed(1000);
        b.update(Zero::Awake, 1000);
        b.update(Zero::Asleep, 1000);
        age(&mut b, 4 * STALE_PERIODS as u32);
        assert_eq!(b.counts(), 1000);
        assert_eq!(b.asleep_counts(), 1000);
    }

    #[test]
    fn a_stale_zero_moves_with_the_fed_one() {
        let mut b = BiasTracker::new();
        b.seed(1000);
        // never awake: the asleep drift carries the awake zero, step 0
        assert_eq!(feed(&mut b, Zero::Asleep, 900, 12 * TAU), 900);
        assert_eq!(b.asleep_counts(), 900);
        // and a long awake run carries the asleep zero the same way
        let mut b = BiasTracker::new();
        b.seed(1000);
        b.update(Zero::Asleep, 1000);
        age(&mut b, STALE_PERIODS as u32);
        assert_eq!(feed(&mut b, Zero::Awake, 1100, 12 * TAU), 1100);
        assert_eq!(b.asleep_counts(), 1100);
    }

    #[test]
    fn a_zero_is_stale_after_stale_periods_unfed() {
        for (fed, held) in [(Zero::Asleep, Zero::Awake), (Zero::Awake, Zero::Asleep)] {
            let read = |b: &BiasTracker| match held {
                Zero::Awake => (b.counts(), b.asleep_counts()),
                Zero::Asleep => (b.asleep_counts(), b.counts()),
            };
            let mut b = BiasTracker::new();
            b.seed(1000);
            b.update(held, 1000);
            // one SLOW period short of the horizon: the held zero stays put
            // however many samples the other takes, as no sample ages it
            age(&mut b, STALE_PERIODS as u32 - 1);
            feed(&mut b, fed, 900, 12 * TAU);
            assert_eq!(read(&b), (1000, 900), "{held:?} one period short");
            // the last period: stale, it moves by the fed zero's delta
            b.age();
            feed(&mut b, fed, 800, 12 * TAU);
            assert_eq!(read(&b), (900, 800), "{held:?} stale");
        }
    }

    #[test]
    fn a_sample_restarts_its_own_horizon() {
        let mut b = BiasTracker::new();
        b.seed(1000);
        b.update(Zero::Awake, 1000);
        age(&mut b, STALE_PERIODS as u32 - 1);
        b.update(Zero::Awake, 1000);
        age(&mut b, STALE_PERIODS as u32 - 1);
        assert_eq!(feed(&mut b, Zero::Asleep, 900, 12 * TAU), 1000);
        assert_eq!(b.asleep_counts(), 900);
    }

    #[test]
    fn the_step_is_learned_across_a_fresh_transition_and_kept() {
        let mut b = BiasTracker::new();
        b.seed(460);
        // torque off at rest, then awake straight after: the asleep zero
        // is fresh and holds while the awake zero learns 10 over it
        feed(&mut b, Zero::Asleep, 460, 64);
        assert_eq!(feed(&mut b, Zero::Awake, 470, 12 * TAU), 470);
        assert_eq!(b.asleep_counts(), 460);
        // torque off past the stale horizon while the board warms: the
        // awake zero keeps the step over the drifting rest
        assert_eq!(run(&mut b, Zero::Asleep, 460, STALE_PERIODS as u32), 470);
        assert_eq!(feed(&mut b, Zero::Asleep, 260, 12 * TAU), 270);
        assert_eq!(b.asleep_counts(), 260);
    }

    #[test]
    fn a_fresh_awake_zero_holds_while_the_asleep_zero_learns() {
        let mut b = BiasTracker::new();
        b.seed(1000);
        // an awake run from boot carries the asleep zero, stale since the
        // seed, to the awake value
        assert_eq!(feed(&mut b, Zero::Awake, 900, 12 * TAU), 900);
        assert_eq!(b.asleep_counts(), 900);
        // asleep right after, for 6 time constants, inside the horizon:
        // the awake zero holds, the asleep one learns
        let slow_periods = 6 * TAU / DECIM_SLOW as u32;
        assert!(slow_periods < STALE_PERIODS as u32);
        assert_eq!(run(&mut b, Zero::Asleep, 890, slow_periods), 900);
        assert_eq!(b.asleep_counts(), 890);
    }

    #[test]
    fn counts_never_wraps_below_zero() {
        let mut b = BiasTracker::new();
        b.seed(5);
        feed(&mut b, Zero::Asleep, 5, 64);
        // the awake zero learns 3 under the asleep one
        feed(&mut b, Zero::Awake, 2, 12 * TAU);
        // then a stale awake zero follows the asleep one down to 0
        age(&mut b, STALE_PERIODS as u32);
        assert_eq!(feed(&mut b, Zero::Asleep, 0, 12 * TAU), 0);
    }
}
