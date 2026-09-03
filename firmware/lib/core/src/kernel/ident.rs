//! Identification aggregates (spec IDENT row): sum/min/max of per-tick
//! current, drive-window differential, and commanded duty, folded over an
//! own /16 fast-tick window - independent of DECIM_MED so the host fitter
//! gets decimation-free statistics without streaming every tick. The window
//! is a power of two so the mean is an arithmetic shift, no divide;
//! truncation is toward -inf for negative sums (sub-LSB, irrelevant to a
//! fit). The kernel feeds LAST-VALID current/vdiff through invalid windows
//! rather than zeros: zeros would drag the means toward 0, last-valid keeps
//! them unbiased at steady state, and identification runs at healthy duties
//! where the windows are valid anyway.

/// Window length as a shift: 16 fast ticks, mean = sum >> IDENT_SHIFT.
pub const IDENT_SHIFT: u32 = 4;
const IDENT_WINDOW: u8 = 1 << IDENT_SHIFT;

/// One published window; `agg_seq` increments per window (wrapping) so the
/// host pairs a consistent set across the six fields.
pub struct IdentOut {
    pub i_mean_counts: i16,
    pub i_min_counts: i16,
    pub i_max_counts: i16,
    pub vdiff_mean: i16,
    pub duty_mean_q15: i16,
    pub agg_seq: u16,
}

pub struct IdentAgg {
    ctr: u8,
    i_sum: i32,
    i_min: i16,
    i_max: i16,
    vdiff_sum: i32,
    duty_sum: i32,
    seq: u16,
}

/// A mean of 16 i16 samples always fits i16 (|sum| <= 16 * 32768); the
/// clamp is a no-panic belt on that invariant.
fn mean_i16(sum: i32) -> i16 {
    let m = sum >> IDENT_SHIFT;
    debug_assert!((i16::MIN as i32..=i16::MAX as i32).contains(&m));
    m.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

impl IdentAgg {
    pub const fn new() -> Self {
        Self {
            ctr: 0,
            i_sum: 0,
            i_min: i16::MAX,
            i_max: i16::MIN,
            vdiff_sum: 0,
            duty_sum: 0,
            seq: 0,
        }
    }

    /// Fold one fast tick; returns the finished window every 16th call.
    /// Sums cannot overflow: 16 x |i16| <= 2^19.
    pub fn sample(&mut self, i: i16, vdiff: i16, duty: i16) -> Option<IdentOut> {
        self.i_sum += i as i32;
        self.i_min = self.i_min.min(i);
        self.i_max = self.i_max.max(i);
        self.vdiff_sum += vdiff as i32;
        self.duty_sum += duty as i32;
        self.ctr += 1;
        if self.ctr < IDENT_WINDOW {
            return None;
        }
        self.seq = self.seq.wrapping_add(1);
        let out = IdentOut {
            i_mean_counts: mean_i16(self.i_sum),
            i_min_counts: self.i_min,
            i_max_counts: self.i_max,
            vdiff_mean: mean_i16(self.vdiff_sum),
            duty_mean_q15: mean_i16(self.duty_sum),
            agg_seq: self.seq,
        };
        self.ctr = 0;
        self.i_sum = 0;
        self.i_min = i16::MAX;
        self.i_max = i16::MIN;
        self.vdiff_sum = 0;
        self.duty_sum = 0;
        Some(out)
    }
}

impl Default for IdentAgg {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_length_and_stats() {
        let mut agg = IdentAgg::new();
        for n in 0..15 {
            assert!(agg.sample(100 + n, 50, 8000).is_none());
        }
        let out = agg.sample(-60, 50, 8000).expect("16th sample publishes");
        // sum = 100+..+114 - 60 = 1545 -> floor mean 96
        assert_eq!(out.i_mean_counts, 96);
        assert_eq!(out.i_min_counts, -60);
        assert_eq!(out.i_max_counts, 114);
        assert_eq!(out.vdiff_mean, 50);
        assert_eq!(out.duty_mean_q15, 8000);
        assert_eq!(out.agg_seq, 1);
    }

    #[test]
    fn negative_mean_truncates_toward_neg_inf() {
        let mut agg = IdentAgg::new();
        for _ in 0..15 {
            agg.sample(0, 0, 0);
        }
        let out = agg.sample(-5, -5, -5).expect("publish");
        assert_eq!(out.i_mean_counts, -1);
        assert_eq!(out.vdiff_mean, -1);
        assert_eq!(out.duty_mean_q15, -1);
    }

    #[test]
    fn extremes_survive_the_full_window() {
        let mut agg = IdentAgg::new();
        let mut out = None;
        for _ in 0..16 {
            out = agg.sample(i16::MIN, i16::MIN, i16::MIN);
        }
        let out = out.expect("publish");
        assert_eq!(out.i_mean_counts, i16::MIN);
        assert_eq!(out.i_min_counts, i16::MIN);
        assert_eq!(out.i_max_counts, i16::MIN);
    }

    #[test]
    fn seq_wraps() {
        let mut agg = IdentAgg::new();
        let mut last = 0u16;
        for _ in 0..(u16::MAX as u32 + 1) {
            for _ in 0..15 {
                agg.sample(0, 0, 0);
            }
            last = agg.sample(0, 0, 0).expect("publish").agg_seq;
        }
        assert_eq!(last, 0, "seq wrapped through 65535");
    }
}
