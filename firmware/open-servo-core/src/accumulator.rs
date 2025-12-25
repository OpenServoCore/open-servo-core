//! Fast-tick accumulator for windowed statistics.
//!
//! Collects position and current samples during fast ticks (10kHz),
//! producing aggregated snapshots for medium-tick (1kHz) processing.

use open_servo_math::{CentiDeg, DegPerSec10, MilliAmp};

/// Snapshot of accumulated fast-tick data for medium-tick processing.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MediumSnapshot {
    /// Window-averaged velocity (position delta / window time).
    pub velocity_dps10: DegPerSec10,
    /// Average current over the window (if any current samples).
    pub current_avg: Option<MilliAmp>,
    /// Peak absolute current over the window.
    pub current_peak_abs: Option<MilliAmp>,
    /// Last position sample in the window.
    pub pos_last: CentiDeg,
    /// Number of samples in this window.
    pub sample_count: u16,
}

/// Accumulates fast-tick samples between medium ticks.
#[derive(Debug, Clone, Copy)]
pub struct FastAccumulator {
    sample_count: u16,
    pos_first: Option<CentiDeg>,
    pos_last: Option<CentiDeg>,
    current_sum_ma: i32,
    current_count: u16,
    current_peak_abs_ma: i16,
}

impl FastAccumulator {
    /// Create a new accumulator with zeroed state.
    pub const fn new() -> Self {
        Self {
            sample_count: 0,
            pos_first: None,
            pos_last: None,
            current_sum_ma: 0,
            current_count: 0,
            current_peak_abs_ma: 0,
        }
    }

    /// Reset accumulator state (call on fault/disengage to avoid stale data).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Record a sample from fast_tick.
    pub fn observe(&mut self, position: CentiDeg, current: Option<MilliAmp>) {
        // Track first position for velocity calculation
        if self.pos_first.is_none() {
            self.pos_first = Some(position);
        }
        self.pos_last = Some(position);
        self.sample_count = self.sample_count.saturating_add(1);

        // Accumulate current if available
        if let Some(ma) = current {
            let ma_i32 = ma.as_ma() as i32;
            self.current_sum_ma = self.current_sum_ma.saturating_add(ma_i32);
            self.current_count = self.current_count.saturating_add(1);

            // Track peak absolute current (use i32 abs to avoid i16::MIN overflow)
            let abs_ma = ma_i32.abs().min(i16::MAX as i32) as i16;
            if abs_ma > self.current_peak_abs_ma {
                self.current_peak_abs_ma = abs_ma;
            }
        }
    }

    /// Compute snapshot and reset accumulator.
    ///
    /// `fast_dt_us`: Per-sample time in microseconds (e.g., 100us for 10kHz).
    /// Returns `None` if no samples were accumulated.
    pub fn take_snapshot(&mut self, fast_dt_us: u32) -> Option<MediumSnapshot> {
        if self.sample_count == 0 {
            return None;
        }

        let pos_first = self.pos_first?;
        let pos_last = self.pos_last?;

        // Compute window velocity: delta_cdeg * 100_000 / window_dt_us
        // Units: cdeg * (1e5) / us = (deg/100) * 1e5 / (s/1e6) = deg * 1e9 / (100 * s)
        // Simplify: deg/s * 10 = DegPerSec10 units
        let window_dt_us = fast_dt_us.saturating_mul(self.sample_count as u32);

        // Defensive: if dt is zero (shouldn't happen), return None to avoid div-by-zero
        if window_dt_us == 0 {
            self.reset();
            return None;
        }

        let delta_cdeg = pos_last.as_cdeg() as i32 - pos_first.as_cdeg() as i32;
        // vel = delta_cdeg * 100_000 / window_dt_us
        let vel_raw = delta_cdeg.saturating_mul(100_000) / window_dt_us as i32;
        let vel_clamped = vel_raw.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        let velocity_dps10 = DegPerSec10::from_dps10(vel_clamped);

        // Compute average current
        let current_avg = if self.current_count > 0 {
            let avg_raw = self.current_sum_ma / self.current_count as i32;
            let avg_clamped = avg_raw.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            Some(MilliAmp::from_ma(avg_clamped))
        } else {
            None
        };

        // Peak current (only if we had current samples)
        let current_peak_abs = if self.current_count > 0 {
            Some(MilliAmp::from_ma(self.current_peak_abs_ma))
        } else {
            None
        };

        let snapshot = MediumSnapshot {
            velocity_dps10,
            current_avg,
            current_peak_abs,
            pos_last,
            sample_count: self.sample_count,
        };

        // Reset for next window
        self.reset();

        Some(snapshot)
    }
}

impl Default for FastAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_accumulator_returns_none() {
        let mut acc = FastAccumulator::new();
        assert!(acc.take_snapshot(100).is_none());
    }

    #[test]
    fn test_single_sample_zero_velocity() {
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(1000), None);

        let snap = acc.take_snapshot(100).unwrap();
        // Single sample: pos_first == pos_last, so velocity = 0
        assert_eq!(snap.velocity_dps10.as_dps10(), 0);
        assert_eq!(snap.pos_last.as_cdeg(), 1000);
        assert_eq!(snap.sample_count, 1);
        assert!(snap.current_avg.is_none());
    }

    #[test]
    fn test_velocity_calculation() {
        // 10 samples at 100us each = 1000us window
        // pos_first=0, pos_last=100cdeg = 1deg movement
        // velocity = 1deg / 0.001s = 1000 deg/s = 10000 DegPerSec10
        let mut acc = FastAccumulator::new();

        for i in 0..10 {
            let pos = CentiDeg::from_cdeg(i * 100 / 9); // Spread 0 to ~100
            acc.observe(pos, None);
        }
        // Force exact endpoints for test
        acc.pos_first = Some(CentiDeg::from_cdeg(0));
        acc.pos_last = Some(CentiDeg::from_cdeg(100));

        let snap = acc.take_snapshot(100).unwrap();
        // delta=100cdeg, window=1000us
        // vel = 100 * 100_000 / 1000 = 10_000_000 / 1000 = 10000
        assert_eq!(snap.velocity_dps10.as_dps10(), 10000);
    }

    #[test]
    fn test_velocity_clamping_positive() {
        let mut acc = FastAccumulator::new();
        // Large movement in short time
        acc.observe(CentiDeg::from_cdeg(0), None);
        acc.observe(CentiDeg::from_cdeg(i16::MAX), None);

        let snap = acc.take_snapshot(1).unwrap(); // 1us per sample = 2us total
                                                  // delta=32767cdeg, window=2us
                                                  // vel = 32767 * 100_000 / 2 = huge, should clamp to i16::MAX
        assert_eq!(snap.velocity_dps10.as_dps10(), i16::MAX);
    }

    #[test]
    fn test_velocity_clamping_negative() {
        let mut acc = FastAccumulator::new();
        // Large negative movement
        acc.observe(CentiDeg::from_cdeg(i16::MAX), None);
        acc.observe(CentiDeg::from_cdeg(i16::MIN), None);

        let snap = acc.take_snapshot(1).unwrap();
        // Should clamp to i16::MIN
        assert_eq!(snap.velocity_dps10.as_dps10(), i16::MIN);
    }

    #[test]
    fn test_current_average() {
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(100)));
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(200)));
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(300)));

        let snap = acc.take_snapshot(100).unwrap();
        // Average of 100, 200, 300 = 200
        assert_eq!(snap.current_avg.unwrap().as_ma(), 200);
    }

    #[test]
    fn test_current_peak_abs_positive() {
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(100)));
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(500)));
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(200)));

        let snap = acc.take_snapshot(100).unwrap();
        assert_eq!(snap.current_peak_abs.unwrap().as_ma(), 500);
    }

    #[test]
    fn test_current_peak_abs_negative() {
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(100)));
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(-500)));
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(200)));

        let snap = acc.take_snapshot(100).unwrap();
        // Peak abs should be 500 (from -500)
        assert_eq!(snap.current_peak_abs.unwrap().as_ma(), 500);
    }

    #[test]
    fn test_current_peak_abs_i16_min() {
        // Test that i16::MIN doesn't overflow when taking abs
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(i16::MIN)));

        let snap = acc.take_snapshot(100).unwrap();
        // i16::MIN.abs() would overflow, but we use i32 abs clamped to i16::MAX
        assert_eq!(snap.current_peak_abs.unwrap().as_ma(), i16::MAX);
    }

    #[test]
    fn test_reset_clears_state() {
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(100), Some(MilliAmp::from_ma(200)));
        acc.observe(CentiDeg::from_cdeg(200), Some(MilliAmp::from_ma(300)));

        acc.reset();

        assert!(acc.take_snapshot(100).is_none());
    }

    #[test]
    fn test_take_snapshot_resets_accumulator() {
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(100), None);

        let _ = acc.take_snapshot(100);

        // Should be empty after take_snapshot
        assert!(acc.take_snapshot(100).is_none());
    }

    #[test]
    fn test_mixed_current_samples() {
        // Some samples have current, some don't
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(0), Some(MilliAmp::from_ma(100)));
        acc.observe(CentiDeg::from_cdeg(10), None);
        acc.observe(CentiDeg::from_cdeg(20), Some(MilliAmp::from_ma(300)));

        let snap = acc.take_snapshot(100).unwrap();
        // Only 2 current samples: avg = (100 + 300) / 2 = 200
        assert_eq!(snap.current_avg.unwrap().as_ma(), 200);
        assert_eq!(snap.sample_count, 3);
    }

    #[test]
    fn test_zero_dt_returns_none() {
        // Defensive: if fast_dt_us is 0, window_dt_us would be 0, avoid div-by-zero
        let mut acc = FastAccumulator::new();
        acc.observe(CentiDeg::from_cdeg(0), None);
        acc.observe(CentiDeg::from_cdeg(100), None);

        // Pass 0 for fast_dt_us - should return None and reset
        assert!(acc.take_snapshot(0).is_none());

        // Accumulator should be reset (subsequent call also returns None)
        assert!(acc.take_snapshot(100).is_none());
    }
}
