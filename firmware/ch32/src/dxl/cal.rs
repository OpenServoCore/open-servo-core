//! Slave-side HSI drift filter.
//!
//! Snoops `(observed_ticks, wire_bytes, byte_time_ticks)` of every non-Status
//! master packet, batches `BATCH_K` samples into a single i32 division for
//! the ppm sample, drives an EMA in ppm-space (no division), and queues a
//! coarse-trim step at ±½-step hysteresis plus a Q8.8 µs fine residual on
//! every batch close.
//!
//! Per-packet hot path is pure additions + one outlier-clip comparison; no
//! division until `BATCH_K` samples have accumulated. Caller glue
//! (`Ch32Bus::poll`) drains [`CalApply`] into the chip's pending atomics.

/// Samples accumulated per batched ppm sample. Trades latency for SNR; at
/// 1 kHz master traffic the batch period is ~8 ms.
const BATCH_K: u8 = 8;

/// EMA shift: α = 1 / (1 << ALPHA_SHIFT) per batch. With BATCH_K=8 and
/// shift=5, the time constant is ~32 batches ≈ 256 samples (~250 ms at
/// 1 kHz traffic) — well above thermal-drift bandwidth.
const ALPHA_SHIFT: u32 = 5;

/// Wire-byte horizon for fine-trim residual translation: matches the
/// master-side `Calibrator(n_target=128)` default so explicit CAL and
/// dynamic snoop produce consistent `clock_fine_trim_us` semantics.
const N_TARGET: u32 = 128;

/// Output of one batched apply step. `trim_step` is a signed step the caller
/// adds to the current HSITRIM delta (clamping is the caller's job); zero
/// means the EMA stayed inside the ±½-step deadband and only fine changed.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct CalApply {
    pub trim_step: i8,
    pub fine_us_q88: i16,
}

/// Snapshot of the filter's most recent observation + most recent batched
/// apply. Surfaces in the CAL Status reply so the master can see what the
/// chip decided.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct CalState {
    pub observed_ticks: u32,
    pub nominal_ticks: u32,
    pub applied_trim_delta: i8,
    pub applied_fine_trim_us: i16,
}

pub struct Cal {
    ppm_per_step: u32,
    ticks_per_us: u32,

    err_ticks_sum: i32,
    nominal_ticks_sum: u32,
    samples: u8,

    ema_ppm: i32,
    baseline_ppm: i32,

    has_observation: bool,
    last_observed_ticks: u32,
    last_nominal_ticks: u32,
    last_applied_trim_step: i8,
    last_applied_fine_us_q88: i16,
}

impl Cal {
    pub const fn new(ppm_per_step: u32, ticks_per_us: u32) -> Self {
        Self {
            ppm_per_step,
            ticks_per_us,
            err_ticks_sum: 0,
            nominal_ticks_sum: 0,
            samples: 0,
            ema_ppm: 0,
            baseline_ppm: 0,
            has_observation: false,
            last_observed_ticks: 0,
            last_nominal_ticks: 0,
            last_applied_trim_step: 0,
            last_applied_fine_us_q88: 0,
        }
    }

    /// Feed one packet's wire-timing into the filter. Returns `Some(apply)`
    /// when the batch closed and the caller should drain into the pending
    /// atomics. The "no apply this call" return covers both mid-batch
    /// samples and outlier-clipped ones.
    pub fn observe(
        &mut self,
        observed_ticks: u32,
        wire_bytes: u32,
        byte_time_ticks: u32,
    ) -> Option<CalApply> {
        if wire_bytes < 2 || byte_time_ticks == 0 {
            return None;
        }
        let nominal = (wire_bytes - 1).saturating_mul(byte_time_ticks);
        let err = (observed_ticks as i32).wrapping_sub(nominal as i32);

        // Latch last observation for the CAL reply path even on the outlier
        // branch — gives the master visibility into corrupt timing.
        self.has_observation = true;
        self.last_observed_ticks = observed_ticks;
        self.last_nominal_ticks = nominal;

        // Drop samples >25% off nominal: multi-packet bursts caught by the
        // first-tick latch (T_first from packet N-k, T_last from packet N)
        // and noise events. Pure comparison, no division.
        if err.unsigned_abs() > nominal / 4 {
            return None;
        }

        self.err_ticks_sum = self.err_ticks_sum.saturating_add(err);
        self.nominal_ticks_sum = self.nominal_ticks_sum.saturating_add(nominal);
        self.samples += 1;
        if self.samples < BATCH_K {
            return None;
        }

        // One i64 division per batch — softdiv-cheap on rv32ec at this rate.
        let sample_ppm = ((self.err_ticks_sum as i64) * 1_000_000)
            .checked_div(self.nominal_ticks_sum as i64)
            .unwrap_or(0) as i32;
        self.err_ticks_sum = 0;
        self.nominal_ticks_sum = 0;
        self.samples = 0;

        // EMA in ppm space — pure shift, no division.
        let diff = sample_ppm.wrapping_sub(self.ema_ppm);
        self.ema_ppm = self.ema_ppm.wrapping_add(diff >> ALPHA_SHIFT);

        // ±½-step deadband around `baseline_ppm` (the EMA at the last coarse
        // apply). Keeps HSITRIM from hunting on noise.
        let d_ppm = self.ema_ppm.wrapping_sub(self.baseline_ppm);
        let half_step = (self.ppm_per_step / 2) as i32;
        let trim_step = if d_ppm.unsigned_abs() >= half_step as u32 {
            let step = d_ppm / self.ppm_per_step as i32;
            self.baseline_ppm = self
                .baseline_ppm
                .wrapping_add(step.wrapping_mul(self.ppm_per_step as i32));
            step.clamp(i8::MIN as i32, i8::MAX as i32) as i8
        } else {
            0
        };

        // Fine residual: -(residual_ppm × N_TARGET × byte_time_us) in Q8.8.
        // Sign matches `clock_fine_trim_us`: + advances fire, − retards. The
        // master-side `cal.py` derivation is the reference; we mirror it in
        // fixed point.
        //
        // q88 = -residual_ppm × N_TARGET × byte_time_ticks × 256
        //         / (ticks_per_us × 1_000_000)
        let residual_ppm = self.ema_ppm.wrapping_sub(self.baseline_ppm);
        let num = -(residual_ppm as i64)
            * (N_TARGET as i64)
            * (byte_time_ticks as i64)
            * 256;
        let den = (self.ticks_per_us as i64) * 1_000_000;
        let fine_us_q88 = (num / den).clamp(i16::MIN as i64, i16::MAX as i64) as i16;

        self.last_applied_trim_step = trim_step;
        self.last_applied_fine_us_q88 = fine_us_q88;

        Some(CalApply {
            trim_step,
            fine_us_q88,
        })
    }

    /// Most recent observation + most recent batched apply, or `None` until
    /// the first packet has been observed.
    pub fn snapshot(&self) -> Option<CalState> {
        if !self.has_observation {
            return None;
        }
        Some(CalState {
            observed_ticks: self.last_observed_ticks,
            nominal_ticks: self.last_nominal_ticks,
            applied_trim_delta: self.last_applied_trim_step,
            applied_fine_trim_us: self.last_applied_fine_us_q88,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // CH32V006-ish: ~4500 ppm/step, 48 ticks/µs.
    const PPM_PER_STEP: u32 = 4500;
    const TICKS_PER_US: u32 = 48;
    // 1 Mbps at 48 MHz HCLK → brr=48, byte_time = 480 ticks.
    const BYTE_TIME_TICKS: u32 = 480;
    // Long-enough sample to be useful: 64 wire bytes → nominal = 63 × 480.
    const WIRE_BYTES: u32 = 64;
    const NOMINAL: u32 = (WIRE_BYTES - 1) * BYTE_TIME_TICKS;

    fn drift_observed(ppm: i32) -> u32 {
        // observed = nominal × (1 + ppm/1e6); for small ppm, integer-safe.
        let delta = ((NOMINAL as i64) * (ppm as i64) / 1_000_000) as i32;
        (NOMINAL as i32 + delta) as u32
    }

    #[test]
    fn single_sample_does_not_apply() {
        let mut c = Cal::new(PPM_PER_STEP, TICKS_PER_US);
        assert!(c.observe(NOMINAL, WIRE_BYTES, BYTE_TIME_TICKS).is_none());
        assert!(c.snapshot().is_some());
    }

    #[test]
    fn batch_close_emits_apply_with_zero_step_at_zero_drift() {
        let mut c = Cal::new(PPM_PER_STEP, TICKS_PER_US);
        let mut last = None;
        for _ in 0..BATCH_K {
            last = c.observe(NOMINAL, WIRE_BYTES, BYTE_TIME_TICKS);
        }
        let apply = last.expect("batch close should emit");
        assert_eq!(apply.trim_step, 0);
        assert_eq!(apply.fine_us_q88, 0);
    }

    #[test]
    fn sustained_drift_converges_and_steps_trim() {
        let mut c = Cal::new(PPM_PER_STEP, TICKS_PER_US);
        let drift = 22_500_i32; // +5 steps worth
        let obs = drift_observed(drift);
        let mut applied_steps: i32 = 0;
        for _ in 0..500 {
            if let Some(a) = c.observe(obs, WIRE_BYTES, BYTE_TIME_TICKS) {
                applied_steps += a.trim_step as i32;
            }
        }
        // Should have stepped trim within ±1 of expected 5 steps (round-to-nearest).
        assert!(
            (4..=6).contains(&applied_steps),
            "expected ~5 step applies, got {applied_steps}"
        );
    }

    #[test]
    fn deadband_holds_small_drift() {
        let mut c = Cal::new(PPM_PER_STEP, TICKS_PER_US);
        // Drift well inside ±½-step (2250 ppm).
        let obs = drift_observed(1_500);
        let mut applied = 0_i32;
        let mut nonzero_fine = false;
        for _ in 0..500 {
            if let Some(a) = c.observe(obs, WIRE_BYTES, BYTE_TIME_TICKS) {
                applied += a.trim_step as i32;
                if a.fine_us_q88 != 0 {
                    nonzero_fine = true;
                }
            }
        }
        assert_eq!(applied, 0, "coarse should stay quiet inside deadband");
        // Fine still tracks: residual_ppm ≈ 1500 → -1500 × 128 × 10 µs / 1e6
        // = -1.92 µs ≈ -492 in Q8.8 (sign retards fire to slow effective rate).
        assert!(nonzero_fine, "fine residual should track the inside-deadband drift");
    }

    #[test]
    fn outlier_clipped_does_not_poison_batch() {
        let mut c = Cal::new(PPM_PER_STEP, TICKS_PER_US);
        // First a clean batch's worth of zero-drift samples but with a single
        // huge outlier inserted — outlier dropped, others accumulate normally.
        for i in 0..BATCH_K {
            let obs = if i == 3 {
                NOMINAL * 4 // 300% off — clipped
            } else {
                NOMINAL
            };
            c.observe(obs, WIRE_BYTES, BYTE_TIME_TICKS);
        }
        // 7 valid samples in batch; one more good sample closes it.
        let apply = c.observe(NOMINAL, WIRE_BYTES, BYTE_TIME_TICKS);
        assert!(apply.is_some(), "outlier-clipped sample should not consume batch slot");
        let a = apply.unwrap();
        // Zero drift in valid samples → no coarse step, fine = 0.
        assert_eq!(a.trim_step, 0);
        assert_eq!(a.fine_us_q88, 0);
    }

    #[test]
    fn snapshot_reflects_last_observation() {
        let mut c = Cal::new(PPM_PER_STEP, TICKS_PER_US);
        assert!(c.snapshot().is_none());
        c.observe(12_345, WIRE_BYTES, BYTE_TIME_TICKS);
        let s = c.snapshot().unwrap();
        assert_eq!(s.observed_ticks, 12_345);
        assert_eq!(s.nominal_ticks, NOMINAL);
    }
}
