//! Shunt zero-current bias tracker.

/// Zero-current output of the current-sense chain, in raw ADC counts:
/// seeded from the boot rest measurement, then followed as an EWMA of the
/// Slow-decay trough shunt sample (`window::trough_is_brake` gates the
/// feed). Alpha = 1/128; `state_q8` keeps 8 sub-LSB bits so the dead band
/// where the update rounds to zero is under half a count, and `counts`
/// rounds to nearest, so a constant input is reproduced exactly.
#[derive(Default)]
pub struct BiasTracker {
    state_q8: i32,
    seeded: bool,
}

impl BiasTracker {
    pub const fn new() -> Self {
        Self {
            state_q8: 0,
            seeded: false,
        }
    }

    pub fn seed(&mut self, counts: u16) {
        self.state_q8 = (counts as i32) << 8;
        self.seeded = true;
    }

    /// Folds one offset sample in; the first sample seeds if nothing has.
    pub fn update(&mut self, trough: u16) -> u16 {
        let x = (trough as i32) << 8;
        if !self.seeded {
            self.state_q8 = x;
            self.seeded = true;
        } else {
            self.state_q8 += (x - self.state_q8) >> 7;
        }
        self.counts()
    }

    pub fn counts(&self) -> u16 {
        ((self.state_q8 + (1 << 7)) >> 8) as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seeds_from_boot_value() {
        let mut b = BiasTracker::new();
        b.seed(1234);
        assert_eq!(b.counts(), 1234);
        assert_eq!(b.update(1234), 1234);
    }

    #[test]
    fn first_sample_seeds_when_unseeded() {
        let mut b = BiasTracker::new();
        assert_eq!(b.update(2000), 2000);
        assert_eq!(b.update(2000), 2000);
    }

    #[test]
    fn constant_input_steady_state() {
        let mut b = BiasTracker::new();
        b.seed(2048);
        for _ in 0..512 {
            assert_eq!(b.update(2048), 2048);
        }
    }

    #[test]
    fn step_converges_at_one_over_128() {
        for (from, to) in [(2048u16, 2088u16), (2088, 2048), (0, 2048)] {
            let mut b = BiasTracker::new();
            b.seed(from);
            let mut v = 0;
            for _ in 0..128 {
                v = b.update(to) as i32;
            }
            // one time constant: 1 - (127/128)^128 = 63.4% of the step
            let expect = from as i32 + ((to as i32 - from as i32) * 634) / 1000;
            assert!((v - expect).abs() <= 1, "{from}->{to}: {v} vs {expect}");
            // 10 tau leaves under 0.1 count of the largest step: inside the
            // half-count dead band, rounded away
            for _ in 128..1280 {
                v = b.update(to) as i32;
            }
            assert_eq!(v, to as i32, "{from}->{to} after 10 tau");
        }
    }
}
