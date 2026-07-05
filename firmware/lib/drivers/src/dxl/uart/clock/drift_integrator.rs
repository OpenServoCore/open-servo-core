//! HSI drift integrator for the DXL transport — a two-phase control law
//! that corrects the servo oscillator against the host's byte-timing.
//! Fed by NDTR/byte-count spans via [`DriftIntegrator::on_span`], commits
//! its correction at the RX packet boundary via
//! [`DriftIntegrator::on_rx_packet_end`]:
//!
//! - **Boot** ([`DriftPhase::Boot`], active until the first batch close from
//!   [`DriftIntegrator::new`]): batch closes at [`DRIFT_MIN_SAMPLES_BOOT`]
//!   spans with a full-step deadband and an emit cap of 16 register steps
//!   (full envelope). Lands ±20 000 ppm factory drift in a single batch so
//!   non-Fast commands work as soon as enough RX traffic has flowed.
//! - **Steady** ([`DriftPhase::Steady`], every batch after the first): batch
//!   closes at [`DRIFT_MIN_SAMPLES_STEADY`] spans with a half-step deadband
//!   and an emit cap of 4 steps. Bounds noise risk and squeezes the residual
//!   gap before the host issues Fast commands.
//!
//! Each span is `(d_ticks, d_bytes)`: the servo-clock ticks elapsed and the
//! whole wire bytes received between two same-flavor drain-ISR stamps of one
//! contiguous burst (see [`super::super::codec`]'s span tracker). The
//! per-span drift is `d_ticks − d_bytes · byte_ticks_spec`; a fast HSI reads
//! more ticks per byte, so a positive drift means the servo runs fast.
//!
//! HSI drift is intrinsic to the oscillator, not the divider — baud changes
//! ([`DriftIntegrator::on_baud_change`]) re-derive `byte_ticks_spec` but
//! never re-engage boot mode.

use core::marker::PhantomData;

use osc_core::BaudRate;

use super::drift_consts::{DRIFT_MIN_SAMPLES_BOOT, DRIFT_MIN_SAMPLES_STEADY};
use crate::dxl::uart::BITS_PER_FRAME;
use crate::traits::dxl::{ClockTrim, UsartBaud};

/// Boot-phase emission cap = full register range. One batch of spans over a
/// real ±20 000 ppm factory drift produces an 8-step estimate that commits
/// cleanly to the envelope clamp in a single emit.
const EMIT_CAP_STEPS_BOOT: i32 = 16;
/// Steady-phase emission cap. Bounds noise sensitivity: a single anomalous
/// batch can't swing the applied correction past
/// `EMIT_CAP_STEPS_STEADY × STEP_PPM` ppm. Magnitude-aware otherwise — the
/// integrator estimates accumulated drift in ppm, emits the opposing
/// magnitude clamped to the cap, then re-samples.
const EMIT_CAP_STEPS_STEADY: i32 = 4;

pub struct DriftIntegrator<U: UsartBaud, T: ClockTrim> {
    trim: T,
    /// `U::CLOCK_HZ` never leaves the reference-clock derivation of
    /// `ticks_per_bit` in [`super::baud_cache::BaudCache`]; `PhantomData`
    /// only binds `U` so this type stays parameterized alongside the cache.
    _usart: PhantomData<U>,
    /// Nominal ticks per bit at the current baud — `BITS_PER_FRAME × this`
    /// is `byte_ticks_spec`, the tick count one wire byte should take on a
    /// perfectly-trimmed clock. The per-span drift subtracts it.
    ticks_per_bit: u16,

    state: DriftPhase,
    /// Robust batch statistic: the span with the smallest per-byte drift
    /// seen this batch. Entry-latency noise only ever *delays* a stamp, so
    /// it can only inflate `d_ticks` — the minimum per-byte drift is the
    /// least-corrupted estimate. Stored raw as `(drift_ticks, d_bytes)` and
    /// compared by cross-multiply so no per-span division is needed (RV32EC
    /// has no hardware divide); one division converts it to ppm at close.
    best: Option<SpanDrift>,
    span_count: u16,
    /// Currently-applied absolute correction relative to factory cal, in
    /// ppm. Mirrors the value last handed to `T::apply_ppm`; clamped to
    /// `T::ENVELOPE_PPM` before emission so we never request a correction
    /// the provider would saturate on.
    applied_ppm: i32,
    pending_applied_ppm: Option<i32>,
}

/// One batch's minimum per-byte drift, kept raw so the ppm conversion (the
/// one division) defers to batch close.
#[derive(Copy, Clone)]
struct SpanDrift {
    /// `d_ticks − d_bytes · byte_ticks_spec` for the span, in servo ticks.
    drift_ticks: i32,
    /// The span's whole-byte length; always > 0 for a stored span.
    d_bytes: u32,
}

/// Two-phase drift control state. Boot lands factory drift in one batch;
/// the one-shot transition to Steady tightens the deadband for the rest of
/// the integrator's lifetime. HSI drift is the oscillator's, not the
/// divider's, so baud changes never re-engage Boot.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum DriftPhase {
    Boot,
    Steady,
}

impl DriftPhase {
    /// Spans that close a batch. Boot's short window lands factory drift
    /// fast; steady's longer window clears sampling noise.
    fn min_samples(self) -> u16 {
        match self {
            Self::Boot => DRIFT_MIN_SAMPLES_BOOT,
            Self::Steady => DRIFT_MIN_SAMPLES_STEADY,
        }
    }

    /// Emission cap in trim steps — see [`EMIT_CAP_STEPS_BOOT`] /
    /// [`EMIT_CAP_STEPS_STEADY`].
    fn emit_cap_steps(self) -> i32 {
        match self {
            Self::Boot => EMIT_CAP_STEPS_BOOT,
            Self::Steady => EMIT_CAP_STEPS_STEADY,
        }
    }

    /// Worth-emitting deadband on the batch's ppm estimate. Boot: full-step
    /// (`STEP_PPM`), conservative over the shorter window. Steady: half-step
    /// (`STEP_PPM / 2`), tighter once the longer window has the SNR margin.
    fn threshold_ppm(self, step_ppm: u32) -> u32 {
        match self {
            Self::Boot => step_ppm,
            Self::Steady => step_ppm >> 1,
        }
    }
}

impl<U: UsartBaud, T: ClockTrim> DriftIntegrator<U, T> {
    pub fn new(_baud: BaudRate, ticks_per_bit: u16, trim: T) -> Self {
        Self {
            trim,
            _usart: PhantomData,
            ticks_per_bit,
            state: DriftPhase::Boot,
            best: None,
            span_count: 0,
            applied_ppm: 0,
            pending_applied_ppm: None,
        }
    }

    #[inline(always)]
    fn is_boot(&self) -> bool {
        matches!(self.state, DriftPhase::Boot)
    }

    /// `byte_ticks_spec` — the tick count a wire byte should take on a
    /// perfectly-trimmed clock at the current baud.
    #[inline(always)]
    fn byte_ticks_spec(&self) -> i64 {
        BITS_PER_FRAME as i64 * self.ticks_per_bit as i64
    }

    // -- events -----------------------------------------------------------------

    /// A committed baud change from the composite. Re-derives
    /// `byte_ticks_spec` and drops the in-flight batch; the phase is
    /// untouched (drift is the oscillator's, not the divider's).
    pub fn on_baud_change(&mut self, _baud: BaudRate, ticks_per_bit: u16) {
        self.ticks_per_bit = ticks_per_bit;
        self.reset_batch();
    }

    /// One NDTR/byte-count span from the codec's span tracker — `d_ticks`
    /// servo-clock ticks and `d_bytes` whole wire bytes over one contiguous
    /// same-flavor burst of Instruction bytes. Folds the span into the
    /// batch's minimum per-byte drift and closes the batch once it has
    /// enough spans.
    ///
    /// The tracker's same-burst gate has already bounded `d_ticks` to within
    /// `d_bytes · byte_ticks / 16` of nominal, so `drift_ticks` here is a
    /// small honest offset, never an inter-packet gap.
    pub fn on_span(&mut self, d_ticks: u32, d_bytes: u32) {
        if d_bytes == 0 {
            return;
        }
        let drift = (d_ticks as i64 - d_bytes as i64 * self.byte_ticks_spec())
            .clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        let cand = SpanDrift {
            drift_ticks: drift,
            d_bytes,
        };
        self.best = Some(match self.best {
            // Compare per-byte drift `a/da < b/db` by cross-multiply (both
            // `d_bytes` > 0, so the inequality direction is preserved).
            Some(cur)
                if (cand.drift_ticks as i64 * cur.d_bytes as i64)
                    < (cur.drift_ticks as i64 * cand.d_bytes as i64) =>
            {
                cand
            }
            Some(cur) => cur,
            None => cand,
        });
        self.span_count += 1;
        if self.span_count >= self.state.min_samples() {
            self.close_batch();
        }
    }

    /// Close one batch: convert the minimum per-byte drift to ppm, stage the
    /// capped counter-correction if it clears the deadband, and reset the
    /// accumulator for the next window.
    #[inline]
    fn close_batch(&mut self) {
        if let Some(best) = self.best {
            let ppm = self.batch_ppm(best);
            if ppm.unsigned_abs() >= self.state.threshold_ppm(T::STEP_PPM) {
                // Magnitude-aware: estimate accumulated drift in whole trim
                // steps, emit the opposing correction capped to bound noise
                // risk. Servo fast → positive drift → counter ppm DOWN.
                let drift_steps = round_div(ppm, T::STEP_PPM as i32);
                let cap = self.state.emit_cap_steps();
                let capped = (-drift_steps).clamp(-cap, cap);
                let nudge_ppm = capped.saturating_mul(T::STEP_PPM as i32);
                let new_applied = self
                    .applied_ppm
                    .saturating_add(nudge_ppm)
                    .clamp(T::ENVELOPE_PPM.0, T::ENVELOPE_PPM.1);
                if new_applied != self.applied_ppm {
                    self.pending_applied_ppm = Some(new_applied);
                }
            }
        }
        self.reset_batch();
    }

    /// The batch's minimum per-byte drift as a signed ppm estimate. The one
    /// division of the whole integrator (RV32EC has no hardware divide, but
    /// a single 64-bit divide at batch close is acceptable — the per-span
    /// path stays divide-free).
    fn batch_ppm(&self, best: SpanDrift) -> i32 {
        let denom = best.d_bytes as i64 * self.byte_ticks_spec();
        if denom == 0 {
            return 0;
        }
        (best.drift_ticks as i64 * 1_000_000 / denom) as i32
    }

    fn reset_batch(&mut self) {
        self.best = None;
        self.span_count = 0;
    }

    /// RX-side packet boundary — applies any pending trim correction and
    /// transitions boot→steady. Trim writes HSITRIM directly (no BRR
    /// mid-frame constraint), so the earliest safe apply point is the first
    /// quiet moment after the integrator has folded a packet's spans.
    /// Called by the driver at every poll boundary — own-Crc,
    /// foreign-SkipComplete, and Bad-CRC / Resync alike, so foreign-
    /// instruction sampling per [[drift_sampling_instruction_only]]
    /// converges the same as own. Idempotent: with no pending correction and
    /// already in [`DriftPhase::Steady`], the call is a no-op.
    pub fn on_rx_packet_end(&mut self) {
        if let Some(applied) = self.pending_applied_ppm.take() {
            self.trim.apply_ppm(applied);
            self.applied_ppm = applied;
        }
        if self.is_boot() {
            self.state = DriftPhase::Steady;
            self.reset_batch();
        }
    }
}

/// Signed integer division rounded half-away-from-zero, so a batch just past
/// the deadband produces `±1` step instead of truncating to 0.
fn round_div(num: i32, den: i32) -> i32 {
    let half = den / 2;
    if num >= 0 {
        (num + half) / den
    } else {
        (num - half) / den
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::dxl::uart::test_support::{ClockTrimState, mk_clock_trim};
    use crate::mocks::{MockClockTrim, MockUsartBaud};

    type TestIntegrator = DriftIntegrator<MockUsartBaud, MockClockTrim>;

    /// `ticks_per_bit` at B9600, HCLK 48 MHz — matches the mock's CLOCK_HZ.
    const SPEC_9600: u16 = 5000;
    /// A representative span byte length (one RX_BUF_LEN/2 batch at the
    /// V006 default ring). Drift is per-byte, so any positive length works.
    const SPAN_BYTES: u32 = 16;

    fn make(baud: BaudRate, ticks_per_bit: u16) -> (TestIntegrator, ClockTrimState) {
        let (trim, t_state) = mk_clock_trim();
        (DriftIntegrator::new(baud, ticks_per_bit, trim), t_state)
    }

    fn integrator_9600() -> TestIntegrator {
        make(BaudRate::B9600, SPEC_9600).0
    }

    /// Feed one span whose per-byte drift is `drift_ppm` ppm relative to the
    /// nominal byte time — the sign convention the hardware span carries
    /// (positive = HSI fast = more ticks per byte).
    fn feed_ppm(c: &mut TestIntegrator, drift_ppm: i32) {
        let spec = BITS_PER_FRAME as i64 * SPEC_9600 as i64;
        let nominal = SPAN_BYTES as i64 * spec;
        let d_ticks = nominal + (nominal * drift_ppm as i64) / 1_000_000;
        c.on_span(d_ticks as u32, SPAN_BYTES);
    }

    // ---------- boot integrator ----------

    #[test]
    fn single_span_does_not_stage_ppm() {
        let mut c = integrator_9600();
        feed_ppm(&mut c, 20_000);
        assert_eq!(c.pending_applied_ppm, None);
        assert_eq!(c.span_count, 1);
    }

    #[test]
    fn boot_batch_at_zero_drift_stages_nothing() {
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, 0);
        }
        assert_eq!(c.pending_applied_ppm, None);
    }

    #[test]
    fn boot_batch_within_deadband_stages_nothing() {
        // 1000 ppm < the full-step (2500 ppm) boot deadband.
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, 1000);
        }
        assert_eq!(c.pending_applied_ppm, None);
    }

    #[test]
    fn boot_batch_positive_drift_stages_negative_ppm() {
        // +20 000 ppm (HSI fast) → 8 steps → counter −8 → −20 000 ppm.
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, 20_000);
        }
        assert_eq!(c.pending_applied_ppm, Some(-20_000));
    }

    #[test]
    fn boot_batch_negative_drift_stages_positive_ppm() {
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, -20_000);
        }
        assert_eq!(c.pending_applied_ppm, Some(20_000));
    }

    #[test]
    fn integrator_resets_after_batch_close() {
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, 20_000);
        }
        assert!(c.best.is_none());
        assert_eq!(c.span_count, 0);
    }

    #[test]
    fn min_statistic_rejects_a_one_sided_inflated_span() {
        // Entry-latency delay only inflates d_ticks; the minimum per-byte
        // drift is the honest estimate. A batch of true-20 000-ppm spans
        // with one badly-delayed (inflated) span still lands at −8 steps.
        let mut c = integrator_9600();
        for i in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, if i == 0 { 60_000 } else { 20_000 });
        }
        assert_eq!(c.pending_applied_ppm, Some(-20_000));
    }

    // ---------- envelope clamp ----------

    #[test]
    fn upper_envelope_bound_clamps_positive_nudge() {
        // −40 000 ppm drift → 16 steps (at the boot emit cap) → counter
        // +16 → +40 000 ppm, clamped to the +37 500 envelope rail.
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, -40_000);
        }
        assert_eq!(c.pending_applied_ppm, Some(MockClockTrim::ENVELOPE_PPM.1));
    }

    // ---------- boot → steady transition ----------

    #[test]
    fn on_rx_packet_end_transitions_out_of_boot() {
        let mut c = integrator_9600();
        assert!(c.is_boot());
        c.on_rx_packet_end();
        assert!(!c.is_boot());
    }

    #[test]
    fn steady_batch_closes_at_20_spans_not_6() {
        let mut c = integrator_9600();
        c.on_rx_packet_end(); // boot → steady, batch reset.
        // +5000 ppm = 2 steps, inside the 4-step steady cap.
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, 5000);
        }
        assert_eq!(
            c.pending_applied_ppm, None,
            "6 spans don't close a steady batch"
        );
        for _ in DRIFT_MIN_SAMPLES_BOOT..DRIFT_MIN_SAMPLES_STEADY {
            feed_ppm(&mut c, 5000);
        }
        assert_eq!(c.pending_applied_ppm, Some(-5000));
    }

    // ---------- trim apply ----------

    #[test]
    fn on_rx_packet_end_applies_pending_ppm_to_trim() {
        let (mut c, t_state) = make(BaudRate::B9600, SPEC_9600);
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed_ppm(&mut c, 20_000);
        }
        assert_eq!(c.pending_applied_ppm, Some(-20_000));
        c.on_rx_packet_end();
        assert_eq!(t_state.apply_ppm_log(), alloc::vec![-20_000]);
        assert_eq!(c.applied_ppm, -20_000);
        assert_eq!(c.pending_applied_ppm, None);
    }

    // ---------- steady emit cap ----------

    #[test]
    fn steady_emit_cap_clamps_to_four_steps() {
        // +15 000 ppm → 6 steps, clamped to the 4-step steady cap =
        // −10 000 ppm even though the estimate says −6.
        let mut c = integrator_9600();
        c.on_rx_packet_end(); // boot → steady
        for _ in 0..DRIFT_MIN_SAMPLES_STEADY {
            feed_ppm(&mut c, 15_000);
        }
        assert_eq!(
            c.pending_applied_ppm,
            Some(-(EMIT_CAP_STEPS_STEADY * MockClockTrim::STEP_PPM as i32))
        );
    }

    // ---------- baud change ----------

    #[test]
    fn on_baud_change_resets_batch() {
        let mut c = integrator_9600();
        feed_ppm(&mut c, 20_000);
        assert_eq!(c.span_count, 1);
        c.on_baud_change(BaudRate::B3000000, 16);
        assert!(c.best.is_none());
        assert_eq!(c.span_count, 0);
    }

    #[test]
    fn drift_measure_scales_with_baud_byte_ticks() {
        // Same +20 000 ppm span at 3M (tpb 16) still lands −8 steps: the
        // per-byte drift is baud-independent because byte_ticks_spec tracks
        // the divisor.
        let (mut c, _) = make(BaudRate::B3000000, 16);
        let spec = BITS_PER_FRAME as i64 * 16;
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            let nominal = SPAN_BYTES as i64 * spec;
            let d_ticks = nominal + (nominal * 20_000) / 1_000_000;
            c.on_span(d_ticks as u32, SPAN_BYTES);
        }
        assert_eq!(c.pending_applied_ppm, Some(-20_000));
    }

    #[test]
    fn zero_byte_span_is_ignored() {
        let mut c = integrator_9600();
        c.on_span(1234, 0);
        assert_eq!(c.span_count, 0);
        assert!(c.best.is_none());
    }
}
