//! Authoritative `ticks_per_bit` for the DXL transport — how long one UART
//! bit lasts on the wire, in TIM2 ticks (HCLK). The RX classifier window
//! check, the fire scheduler's wire-end math, and the snoop tick all read it.
//! Everything stays in ticks; µs conversion only at the edge (telemetry).
//!
//! Two inputs determine the value, both committed at `on_tx_complete` (USART
//! can't change BRR mid-frame):
//! - Baud, staged by control-table writes via [`Clock::stage_baud`].
//! - HSITRIM step, derived from drift samples via
//!   [`Clock::on_drift_sample`]. The integrator stages a ±1 step when the
//!   running average crosses ~½ step; one step ≈ 0.4 ticks at the 11·bit
//!   classifier-window edge, easily inside the ±10% tolerance.

use osc_core::BaudRate;

use crate::traits::{ClockTrim, UsartBaud};

const DRIFT_MIN_SAMPLES: u16 = 32;

/// Round-to-nearest BRR divisor for a baud, given the USART's clock rate.
/// Folds to a literal whenever both arguments are const at the call site.
pub(crate) const fn brr_for(clock_hz: u32, baud_hz: u32) -> u32 {
    (clock_hz + baud_hz / 2) / baud_hz
}

pub struct Clock<U: UsartBaud, T: ClockTrim> {
    usart: U,
    trim: T,

    baud: BaudRate,
    ticks_per_bit: u16,

    /// Sum of `delta_ticks << 8` across the current batch. Q8 keeps sub-tick
    /// resolution at high baud (spec=16 → threshold ≈ 0.64 ticks/32 samples).
    drift_sum_q8: i32,
    /// `(DRIFT_THRESHOLD_Q20 × ticks_per_bit × DRIFT_MIN_SAMPLES) >> 12`,
    /// precomputed at baud change. Per-sample path becomes shift + add +
    /// compare — no `__divsi3`.
    drift_threshold_q8: u32,
    trim_delta: i8,
    drift_samples: u16,

    pending_baud: Option<BaudRate>,
    pending_trim_delta: Option<i8>,
}

impl<U: UsartBaud, T: ClockTrim> Clock<U, T> {
    /// Relative frequency shift per trim step, in Q20 (1 unit = 1/2^20).
    /// Power-of-2 scale (instead of ppm) so the threshold compute is a pure
    /// shift. Folded to a literal at monomorphization time.
    const TRIM_PER_STEP_Q20: u32 = ((T::STEP_HZ as u64 * (1 << 20)) / T::HZ as u64) as u32;
    /// Step iff average drift exceeds half a step — below it, the actuator
    /// would overshoot and the post-step error would be worse than pre-step.
    const DRIFT_THRESHOLD_Q20: u32 = Self::TRIM_PER_STEP_Q20 / 2;

    /// BRR divisor for the given baud at this USART's clock rate. Each arm
    /// folds to a literal at monomorphization — no runtime divide (matters
    /// on RV32EC, which has no hardware `div`).
    #[inline]
    fn brr(baud: BaudRate) -> u32 {
        match baud {
            BaudRate::B9600 => const { brr_for(U::CLOCK_HZ, BaudRate::B9600.as_hz()) },
            BaudRate::B57600 => const { brr_for(U::CLOCK_HZ, BaudRate::B57600.as_hz()) },
            BaudRate::B115200 => const { brr_for(U::CLOCK_HZ, BaudRate::B115200.as_hz()) },
            BaudRate::B1000000 => const { brr_for(U::CLOCK_HZ, BaudRate::B1000000.as_hz()) },
            BaudRate::B2000000 => const { brr_for(U::CLOCK_HZ, BaudRate::B2000000.as_hz()) },
            BaudRate::B3000000 => const { brr_for(U::CLOCK_HZ, BaudRate::B3000000.as_hz()) },
        }
    }

    pub fn new(baud: BaudRate, usart: U, trim: T) -> Self {
        let ticks_per_bit = Self::brr(baud) as u16;
        Self {
            usart,
            trim,
            baud,
            ticks_per_bit,
            trim_delta: 0,
            pending_baud: None,
            pending_trim_delta: None,
            drift_sum_q8: 0,
            drift_threshold_q8: Self::drift_threshold_q8(ticks_per_bit),
            drift_samples: 0,
        }
    }

    /// `|drift_sum_q8|` value that flags a worth-stepping batch. Derives from
    /// the half-step rule expressed in Q8 ticks: the `× 256` (Q8 scale) and
    /// the `/ 2^20` (Q20 → ratio) collapse to a single `>> 12`.
    const fn drift_threshold_q8(ticks_per_bit: u16) -> u32 {
        let prod =
            Self::DRIFT_THRESHOLD_Q20 as u64 * ticks_per_bit as u64 * DRIFT_MIN_SAMPLES as u64;
        (prod >> 12) as u32
    }

    #[allow(dead_code)]
    pub fn stage_baud(&mut self, baud: BaudRate) {
        if baud != self.baud {
            self.pending_baud = Some(baud);
        }
    }

    #[allow(dead_code)]
    pub fn on_tx_complete(&mut self) {
        let mut reset_integrator = false;
        if let Some(baud) = self.pending_baud.take() {
            let brr = Self::brr(baud);
            self.usart.set_baud(brr);
            self.baud = baud;
            self.ticks_per_bit = brr as u16;
            self.drift_threshold_q8 = Self::drift_threshold_q8(self.ticks_per_bit);
            reset_integrator = true;
        }
        if let Some(delta) = self.pending_trim_delta.take() {
            self.trim.apply_delta(delta);
            self.trim_delta = delta;
            reset_integrator = true;
        }
        if reset_integrator {
            self.drift_sum_q8 = 0;
            self.drift_samples = 0;
        }
    }

    /// Inter-byte BT delta from the codec. Doc §10.7.1: only deltas in
    /// `[9·tpb, 11·tpb]` measure a real 10-bit byte-time; GAP-class deltas
    /// would inject re-anchor distance and poison the running average, so
    /// they're filtered here rather than in the composite. Per
    /// driver-pattern §4 the HIT window belongs with the driver that owns
    /// `ticks_per_bit` — this one — not with the routing.
    #[allow(dead_code)]
    pub fn on_byte_pair(&mut self, prev_ts: u16, curr_ts: u16) {
        let byte_ticks = curr_ts.wrapping_sub(prev_ts);
        let lo = self.ticks_per_bit.wrapping_mul(9);
        let hi = self.ticks_per_bit.wrapping_mul(11);
        if byte_ticks < lo || byte_ticks > hi {
            return;
        }

        let observed_tpb = (byte_ticks / 10) as i32;
        let drift = observed_tpb - self.ticks_per_bit as i32;
        self.drift_sum_q8 = self.drift_sum_q8.saturating_add(drift << 8);
        self.drift_samples += 1;
        if self.drift_samples < DRIFT_MIN_SAMPLES {
            return;
        }

        if self.drift_sum_q8.unsigned_abs() >= self.drift_threshold_q8 {
            // Servo fast → observed ticks/bit short → HSITRIM must drop.
            let step: i8 = if self.drift_sum_q8 > 0 { -1 } else { 1 };
            let new_delta = (self.trim_delta as i32 + step as i32)
                .clamp(T::DELTA_MIN as i32, T::DELTA_MAX as i32) as i8;
            if new_delta != self.trim_delta {
                self.pending_trim_delta = Some(new_delta);
            }
        }
        self.drift_sum_q8 = 0;
        self.drift_samples = 0;
    }

    #[inline(always)]
    #[allow(dead_code)]
    pub fn ticks_per_bit(&self) -> u16 {
        self.ticks_per_bit
    }

    /// Integer-µs duration of `bytes` wire bytes at the current baud.
    /// Used by the composite's slot-offset math: `delay = rdt + bytes_to_us(offset)`.
    #[allow(dead_code)]
    pub fn bytes_to_us(&self, bytes: u32) -> u32 {
        let ticks = (bytes as u64) * (self.ticks_per_bit as u64) * 10;
        (ticks * 1_000_000 / U::CLOCK_HZ as u64) as u32
    }

    /// Q8.8 µs duration of `bytes` wire bytes. Used by the Fast Last fire
    /// path where sub-µs precision matters at 3 Mbaud.
    #[allow(dead_code)]
    pub fn bytes_to_us_q88(&self, bytes: u32) -> u32 {
        let ticks = (bytes as u64) * (self.ticks_per_bit as u64) * 10;
        (ticks * (1_000_000 * 256) / U::CLOCK_HZ as u64) as u32
    }
}

#[cfg(test)]
impl<U: UsartBaud, T: ClockTrim> Clock<U, T> {
    /// Total drift samples accepted since the last batch close. Lets
    /// cross-module tests (e.g. [`super::super::DxlUart`]'s drift-feed
    /// path) verify that samples actually reached the integrator without
    /// having to drive a full `DRIFT_MIN_SAMPLES` batch.
    pub(crate) fn drift_samples(&self) -> u16 {
        self.drift_samples
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{FakeClockTrim, FakeUsartBaud};

    // 9600 baud → BRR 5000 — high spec gives ample headroom for sub-tick
    // drift math without bumping against the i32 range or the deadband.
    const TEST_BAUD: BaudRate = BaudRate::B9600;
    const SPEC: u16 = 5000;

    fn clock(baud: BaudRate) -> Clock<FakeUsartBaud, FakeClockTrim> {
        Clock::new(baud, FakeUsartBaud::default(), FakeClockTrim::default())
    }

    /// Feed one byte-time observation expressed as ticks-per-bit via
    /// `on_byte_pair`. `prev = 0`, `curr = observed_tpb * 10` lands a delta
    /// in the `[9·spec, 11·spec]` HIT window for `observed_tpb ≈ spec`, so
    /// the integrator sees the same input the production codec produces.
    fn feed(c: &mut Clock<FakeUsartBaud, FakeClockTrim>, observed_tpb: u16) {
        c.on_byte_pair(0, observed_tpb * 10);
    }

    #[test]
    fn new_computes_ticks_per_bit_from_brr() {
        assert_eq!(clock(BaudRate::B3000000).ticks_per_bit, 16);
        assert_eq!(clock(BaudRate::B1000000).ticks_per_bit, 48);
        assert_eq!(clock(BaudRate::B9600).ticks_per_bit, 5000);
    }

    #[test]
    fn stage_baud_records_change() {
        let mut c = clock(TEST_BAUD);
        c.stage_baud(BaudRate::B3000000);
        assert_eq!(c.pending_baud, Some(BaudRate::B3000000));
    }

    #[test]
    fn stage_baud_is_noop_for_same_baud() {
        let mut c = clock(TEST_BAUD);
        c.stage_baud(TEST_BAUD);
        assert_eq!(c.pending_baud, None);
    }

    #[test]
    fn single_sample_does_not_stage_trim() {
        let mut c = clock(TEST_BAUD);
        feed(&mut c, SPEC + 100);
        assert_eq!(c.pending_trim_delta, None);
        assert_eq!(c.drift_samples, 1);
    }

    #[test]
    fn batch_at_zero_drift_does_not_stage_trim() {
        let mut c = clock(TEST_BAUD);
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC);
        }
        assert_eq!(c.pending_trim_delta, None);
    }

    #[test]
    fn batch_within_deadband_does_not_stage_trim() {
        // delta=3 per sample → ~600 ppm, well under the ~1250 ppm half-step.
        let mut c = clock(TEST_BAUD);
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC + 3);
        }
        assert_eq!(c.pending_trim_delta, None);
    }

    #[test]
    fn sustained_positive_drift_stages_trim_down() {
        // delta=10 per sample → ~2000 ppm → above threshold; servo runs fast.
        let mut c = clock(TEST_BAUD);
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC + 10);
        }
        assert_eq!(c.pending_trim_delta, Some(-1));
    }

    #[test]
    fn sustained_negative_drift_stages_trim_up() {
        let mut c = clock(TEST_BAUD);
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC - 10);
        }
        assert_eq!(c.pending_trim_delta, Some(1));
    }

    #[test]
    fn integrator_resets_after_batch_close() {
        let mut c = clock(TEST_BAUD);
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC + 10);
        }
        assert_eq!(c.drift_sum_q8, 0);
        assert_eq!(c.drift_samples, 0);
    }

    #[test]
    fn clamp_holds_trim_at_upper_bound() {
        let mut c = clock(TEST_BAUD);
        c.trim_delta = FakeClockTrim::DELTA_MAX;
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC - 100); // would step trim up
        }
        // At MAX already → clamp keeps new == current → no stage.
        assert_eq!(c.pending_trim_delta, None);
    }

    #[test]
    fn out_of_window_byte_pair_is_dropped() {
        // delta > 11·tpb → GAP class (e.g. inter-packet gap). The integrator
        // must not see it — the BT pair walk inside `Codec::poll_one`
        // doesn't pre-filter; that's this method's job.
        let mut c = clock(TEST_BAUD);
        // 12·SPEC = 60_000 → above hi=55_000.
        c.on_byte_pair(0, 60_000);
        assert_eq!(c.drift_samples, 0);
    }

    #[test]
    fn under_window_byte_pair_is_dropped() {
        // delta < 9·tpb → SKIP class (intra-byte edge). Same drop.
        let mut c = clock(TEST_BAUD);
        // 8·SPEC = 40_000 → below lo=45_000.
        c.on_byte_pair(0, 40_000);
        assert_eq!(c.drift_samples, 0);
    }

    #[test]
    fn drift_threshold_q8_matches_formula() {
        type C = Clock<FakeUsartBaud, FakeClockTrim>;
        assert_eq!(C::drift_threshold_q8(16), 163);
        assert_eq!(C::drift_threshold_q8(48), 491);
        assert_eq!(C::drift_threshold_q8(5000), 51171);
    }

    #[test]
    fn on_tx_complete_applies_staged_baud_to_usart() {
        let mut c = clock(TEST_BAUD);
        c.stage_baud(BaudRate::B3000000);
        c.on_tx_complete();
        let expected_brr = brr_for(FakeUsartBaud::CLOCK_HZ, BaudRate::B3000000.as_hz());
        assert_eq!(c.usart.log, [expected_brr]);
        assert_eq!(c.baud, BaudRate::B3000000);
        assert_eq!(c.ticks_per_bit, 16);
    }

    #[test]
    fn on_tx_complete_applies_staged_trim_to_rcc() {
        let mut c = clock(TEST_BAUD);
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC + 10);
        }
        // Sustained positive drift staged trim_delta = -1; commit it now.
        c.on_tx_complete();
        assert_eq!(c.trim.log, [-1]);
        assert_eq!(c.trim_delta, -1);
    }

    #[test]
    fn on_tx_complete_is_noop_with_nothing_pending() {
        let mut c = clock(TEST_BAUD);
        c.on_tx_complete();
        assert!(c.usart.log.is_empty());
        assert!(c.trim.log.is_empty());
    }

    #[test]
    fn bytes_to_us_at_1m_is_ten_us_per_byte() {
        let c = clock(BaudRate::B1000000);
        assert_eq!(c.bytes_to_us(1), 10);
        assert_eq!(c.bytes_to_us(5), 50);
    }

    #[test]
    fn bytes_to_us_at_3m_truncates_to_three_us_per_byte() {
        // 10 bits / 3 Mbaud = 3.33 µs/byte → 3 µs integer.
        let c = clock(BaudRate::B3000000);
        assert_eq!(c.bytes_to_us(1), 3);
        assert_eq!(c.bytes_to_us(3), 10); // 9.99 µs → 9? actually (3*16*10*1e6/48e6)=10
    }

    #[test]
    fn bytes_to_us_q88_at_3m_preserves_sub_us_resolution() {
        // (1 * 16 * 10 * 256_000_000 / 48_000_000) = 853 (Q8.8) ≈ 3.33 µs.
        // Linear in bytes — no truncation accumulates across multipliers.
        let c = clock(BaudRate::B3000000);
        assert_eq!(c.bytes_to_us_q88(1), 853);
        assert_eq!(c.bytes_to_us_q88(3), 2560);
    }
}
