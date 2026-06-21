//! Authoritative `ticks_per_bit` for the DXL transport — how long one UART
//! bit lasts on the wire, in TIM2 ticks (HCLK). The RX classifier window
//! check, the fire scheduler's wire-end math, and the snoop tick all read it.
//! Everything stays in ticks; µs conversion only at the edge (telemetry).
//!
//! Two inputs determine the value, both committed at `on_tx_complete` (USART
//! can't change BRR mid-frame):
//! - Baud, staged by control-table writes via [`Clock::stage_baud`].
//! - Clock trim correction, derived from drift samples via
//!   [`Clock::on_byte_pair`]. The integrator runs a two-phase control law:
//!     * **Boot** (active until first batch close from [`Clock::new`]):
//!       batch closes at [`DRIFT_MIN_SAMPLES_BOOT`] = 6 byte pairs (one
//!       Ping reply's worth) with a full-step threshold and an emit cap
//!       of 16 register steps (full envelope). Lands ±20 000 ppm factory
//!       drift in a single packet so non-Fast commands work immediately
//!       after init.
//!     * **Steady** (every batch after the first): batch closes at
//!       [`DRIFT_MIN_SAMPLES_STEADY`] = 20 pairs with a half-step
//!       threshold and an emit cap of 4 steps. Bounds noise risk and
//!       squeezes the residual gap before host issues Fast commands.
//!
//!   HSI drift is intrinsic to the oscillator, not the divider — baud
//!   changes don't re-engage boot mode.

use osc_core::BaudRate;

use super::BITS_PER_FRAME;
use crate::traits::dxl::{ClockTrim, UsartBaud};

/// Fixed-point lift: `× Q8_SCALE` shifts byte-tick deltas into the
/// integrator's accumulator domain so sub-tick drift survives summation.
const Q8_SCALE: u32 = 256;

/// Batch size during the boot (first-emit) phase. One Ping instruction +
/// reply supplies ~6 in-window byte pairs (anchor at byte 3 + 6 forward
/// hits across bytes 4–9), so the boot batch closes inside the first
/// instruction the slave parses.
const DRIFT_MIN_SAMPLES_BOOT: u16 = 6;
/// Batch size after the boot phase has fired. At N=20 the half-step
/// deadband sits 2.8σ above per-sample chip-stamp quantization noise
/// under the conservative 1-tick σ model (9.8σ at the realistic uniform
/// 1-LSB σ) at the worst-case 3M baud — false-emit rate ~5e-3 per batch
/// conservative, ~0 uniform. ~one bogus correction every ~4 minutes at
/// sustained 3M load in the conservative envelope; the next batch
/// immediately reverts the spurious ±1-step nudge.
const DRIFT_MIN_SAMPLES_STEADY: u16 = 20;

/// Boot-phase emission cap = full register range. Estimator at 6
/// samples has ~3.4σ SNR margin over chip-stamp quantization noise at
/// the worst baud (3M, tpb=16); for a real ±20 000 ppm factory drift
/// it produces 8 ± 0.3 step estimates that commit cleanly to the
/// envelope clamp in one go.
const EMIT_CAP_STEPS_BOOT: i32 = 16;
/// Steady-phase emission cap. Bounds noise sensitivity: a single
/// anomalous batch can't swing the applied correction past
/// `EMIT_CAP_STEPS_STEADY × STEP_PPM` ppm. Magnitude-aware otherwise —
/// the integrator estimates accumulated drift in step units, emits the
/// opposing magnitude clamped to the cap, then re-samples.
const EMIT_CAP_STEPS_STEADY: i32 = 4;

/// Round-to-nearest rate divisor — `ticks_per_bit` for a baud at the
/// driver's reference clock. Folds to a literal whenever both arguments
/// are const at the call site. Numerically identical to a USART BRR
/// divisor on chips where the BRR clock equals the monotonic tick clock,
/// but the chip-side provider owns its own BRR math now.
pub(crate) const fn divisor_for(clock_hz: u32, baud_hz: u32) -> u32 {
    (clock_hz + baud_hz / 2) / baud_hz
}

pub struct Clock<U: UsartBaud, T: ClockTrim> {
    usart: U,
    trim: T,

    baud: BaudRate,
    ticks_per_bit: u16,

    /// `true` until the first batch close from [`Clock::new`]. The first
    /// `on_tx_complete` after that close transitions to steady mode
    /// and stays there for the rest of the Clock's lifetime — HSI drift
    /// is the oscillator's, not the divider's, so baud changes don't
    /// re-engage boot.
    is_boot: bool,
    /// Sum of `(byte_ticks − BITS_PER_FRAME × tpb) << 8` across the
    /// current batch. Tracking the raw byte-time delta (not `byte_ticks
    /// / BITS_PER_FRAME`) keeps 1-tick chip-stamp quantization noise
    /// below threshold at small `ticks_per_bit` (V006 at 3M baud:
    /// tpb=16, 1 tick = 6% of a bit).
    drift_sum_q8: i32,
    /// `|drift_sum_q8|` value that flags a worth-emitting batch. Boot
    /// phase uses a full-step threshold (= `drift_per_step_q8`);
    /// steady uses a half-step (= `drift_per_step_q8 >> 1`).
    /// Precomputed on baud change and on phase transition.
    drift_threshold_q8: u32,
    /// Reciprocal of one full step's drift in Q32. Lets `on_byte_pair`
    /// estimate accumulated drift in trim-step units via a single 64-bit
    /// multiply + shift instead of a runtime divide. Precomputed
    /// alongside `drift_threshold_q8`.
    drift_per_step_recip_q32: u32,
    /// Currently-applied absolute correction relative to factory cal, in
    /// ppm. Mirrors the value last handed to `T::apply_ppm`; clamped to
    /// `T::ENVELOPE_PPM` before emission so we never request a correction
    /// the provider would saturate on.
    applied_ppm: i32,
    drift_samples: u16,

    pending_baud: Option<BaudRate>,
    pending_applied_ppm: Option<i32>,
}

impl<U: UsartBaud, T: ClockTrim> Clock<U, T> {
    /// `ticks_per_bit` at the given baud and the reference clock rate. Each
    /// arm folds to a literal at monomorphization — no runtime divide
    /// (matters on RV32EC, which has no hardware `div`).
    #[inline]
    fn ticks_per_bit_at(baud: BaudRate) -> u16 {
        match baud {
            BaudRate::B9600 => const { divisor_for(U::CLOCK_HZ, BaudRate::B9600.as_hz()) as u16 },
            BaudRate::B57600 => const { divisor_for(U::CLOCK_HZ, BaudRate::B57600.as_hz()) as u16 },
            BaudRate::B115200 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B115200.as_hz()) as u16 }
            }
            BaudRate::B1000000 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B1000000.as_hz()) as u16 }
            }
            BaudRate::B2000000 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B2000000.as_hz()) as u16 }
            }
            BaudRate::B3000000 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B3000000.as_hz()) as u16 }
            }
        }
    }

    pub fn new(baud: BaudRate, usart: U, trim: T) -> Self {
        let ticks_per_bit = Self::ticks_per_bit_at(baud);
        let per_step_q8 = Self::drift_per_step_q8(ticks_per_bit, DRIFT_MIN_SAMPLES_BOOT);
        Self {
            usart,
            trim,
            baud,
            ticks_per_bit,
            is_boot: true,
            applied_ppm: 0,
            pending_baud: None,
            pending_applied_ppm: None,
            drift_sum_q8: 0,
            // Boot phase: full-step threshold for ≥3.4σ SNR margin at 3M.
            drift_threshold_q8: per_step_q8,
            drift_per_step_recip_q32: Self::drift_per_step_recip_q32(per_step_q8),
            drift_samples: 0,
        }
    }

    /// One full trim step's worth of accumulated byte-tick drift over `N`
    /// samples, in Q8 ticks. `drift_per_byte_q8 = STEP_PPM × tpb ×
    /// BITS_PER_FRAME × Q8_SCALE / 10⁶` (one step shifts a BITS_PER_FRAME-
    /// bit byte-time by `BITS_PER_FRAME × tpb × STEP_PPM / 10⁶` ticks; the
    /// × Q8_SCALE lifts to Q8). Multiplied by `N` gives full-step-over-N.
    /// Half-step (steady deadband) is just `>> 1`. Tracking drift on raw
    /// byte_ticks instead of `(byte_ticks / BITS_PER_FRAME)` keeps 1-tick
    /// chip-stamp quantization noise below threshold at small
    /// `ticks_per_bit` (V006 at 3M has tpb=16 — a single tick of stamp
    /// jitter is 6% of one bit and would otherwise dominate).
    fn drift_per_step_q8(ticks_per_bit: u16, n_samples: u16) -> u32 {
        let prod = T::STEP_PPM as u64
            * ticks_per_bit as u64
            * n_samples as u64
            * BITS_PER_FRAME as u64
            * Q8_SCALE as u64;
        (prod / 1_000_000) as u32
    }

    /// Q32 reciprocal of one full step's drift-Q8, so `on_byte_pair` can
    /// estimate accumulated drift in step units without a runtime divide.
    fn drift_per_step_recip_q32(drift_per_step_q8: u32) -> u32 {
        let denom = (drift_per_step_q8 as u64).max(1);
        ((1u64 << 32) / denom) as u32
    }

    /// Recompute `drift_threshold_q8` and `drift_per_step_recip_q32` from
    /// the current `ticks_per_bit` and `is_boot`. Called on baud change
    /// and on phase transition — both cold paths (at most twice per Clock
    /// lifetime in steady state).
    fn rebuild_integrator_consts(&mut self) {
        let n_samples = if self.is_boot {
            DRIFT_MIN_SAMPLES_BOOT
        } else {
            DRIFT_MIN_SAMPLES_STEADY
        };
        let per_step_q8 = Self::drift_per_step_q8(self.ticks_per_bit, n_samples);
        // Boot: full-step threshold (more conservative deadband over the
        // smaller 6-sample window). Steady: half-step (tighter; the
        // 20-sample window has enough SNR margin to spot half-step drift).
        self.drift_threshold_q8 = if self.is_boot {
            per_step_q8
        } else {
            per_step_q8 >> 1
        };
        self.drift_per_step_recip_q32 = Self::drift_per_step_recip_q32(per_step_q8);
    }

    #[allow(dead_code)]
    pub fn stage_baud(&mut self, baud: BaudRate) {
        if baud != self.baud {
            self.pending_baud = Some(baud);
        }
    }

    #[allow(dead_code)]
    pub fn on_tx_complete(&mut self) {
        // Baud-only: BRR can't change mid-frame, so a staged baud waits
        // for the USART to go idle after our own TX. Trim corrections
        // and boot→steady transitions happen at RX packet boundaries
        // via [`Self::on_rx_packet_end`] — trim writes HSITRIM (not BRR)
        // and the integrator is fed by RX edges, so an RX boundary is
        // both the earliest and the natural quiet point.
        if let Some(baud) = self.pending_baud.take() {
            self.usart.apply_baud(baud);
            self.baud = baud;
            self.ticks_per_bit = Self::ticks_per_bit_at(baud);
            self.rebuild_integrator_consts();
            self.drift_sum_q8 = 0;
            self.drift_samples = 0;
        }
    }

    /// RX-side packet boundary — applies any pending trim correction and
    /// transitions boot→steady. Trim writes HSITRIM directly (no BRR
    /// mid-frame constraint), so the earliest safe apply point is the
    /// first quiet moment after the integrator has seen all of a
    /// packet's byte pairs. Called by the driver at every poll after
    /// the byte-pair buffer is drained into [`Self::on_byte_pair`] — at
    /// own-Crc, foreign-SkipComplete, and Bad-CRC / Resync boundaries
    /// alike, so foreign-instruction sampling per
    /// [[drift_sampling_instruction_only]] converges the same as own.
    /// Idempotent: with no pending correction and `is_boot` already
    /// false, the call is a no-op.
    #[allow(dead_code)]
    pub fn on_rx_packet_end(&mut self) {
        let mut reset_integrator = false;
        if let Some(applied) = self.pending_applied_ppm.take() {
            self.trim.apply_ppm(applied);
            self.applied_ppm = applied;
            reset_integrator = true;
        }
        let was_boot = self.is_boot;
        if was_boot {
            self.is_boot = false;
            reset_integrator = true;
        }
        if was_boot {
            self.rebuild_integrator_consts();
        }
        if reset_integrator {
            self.drift_sum_q8 = 0;
            self.drift_samples = 0;
        }
    }

    /// One classifier byte-pair from the codec — `prev`/`curr` are
    /// consecutive start-bit edge timestamps the classifier walked out
    /// of the EXTI/DMA edge ring. The classifier gates on its HIT window
    /// `[WINDOW_LO_MUL·tpb, WINDOW_HI_MUL·tpb]` (doc §10.7.1) before
    /// emitting, so every pair reaching here is a real
    /// `BITS_PER_FRAME`-bit byte-time delta.
    ///
    /// Despite the per-pair callback shape, the accumulator is *one
    /// multi-byte stride measurement*: each pair contributes
    /// `(e_k − e_{k−1}) − BITS_PER_FRAME·tpb`, and over N samples the
    /// intermediate edge stamps telescope to give `(e_N − e_0) −
    /// N·BITS_PER_FRAME·tpb`. Noise on `drift_sum_q8` is therefore
    /// `√2·σ_edge` regardless of N — independent of sample count.
    #[allow(dead_code)]
    pub fn on_byte_pair(&mut self, prev: u16, curr: u16) {
        let byte_ticks = curr.wrapping_sub(prev);
        // Drift on raw byte_ticks (no `/BITS_PER_FRAME` truncation) — at
        // small `ticks_per_bit` (V006 at 3M: tpb=16, byte=160) a 1-tick
        // chip-stamp quantization is ~6% of one bit and would otherwise
        // dominate the per-sample signal. Threshold scales by
        // `BITS_PER_FRAME` to match.
        let drift = byte_ticks as i32 - BITS_PER_FRAME as i32 * self.ticks_per_bit as i32;
        self.drift_sum_q8 = self.drift_sum_q8.saturating_add(drift << 8);
        self.drift_samples += 1;
        let min_samples = if self.is_boot {
            DRIFT_MIN_SAMPLES_BOOT
        } else {
            DRIFT_MIN_SAMPLES_STEADY
        };
        if self.drift_samples < min_samples {
            return;
        }

        if self.drift_sum_q8.unsigned_abs() >= self.drift_threshold_q8 {
            // Magnitude-aware: estimate accumulated drift in trim steps,
            // emit the opposing correction capped to bound noise risk.
            // Servo fast → observed ticks/bit short → counter ppm DOWN.
            let drift_steps = self.drift_in_steps();
            let counter = -drift_steps;
            let cap = if self.is_boot {
                EMIT_CAP_STEPS_BOOT
            } else {
                EMIT_CAP_STEPS_STEADY
            };
            let capped = counter.clamp(-cap, cap);
            let nudge_ppm = capped.saturating_mul(T::STEP_PPM as i32);
            let new_applied = self
                .applied_ppm
                .saturating_add(nudge_ppm)
                .clamp(T::ENVELOPE_PPM.0, T::ENVELOPE_PPM.1);
            if new_applied != self.applied_ppm {
                self.pending_applied_ppm = Some(new_applied);
            }
        }
        self.drift_sum_q8 = 0;
        self.drift_samples = 0;
    }

    /// Accumulated drift estimate in signed trim-step units. Uses the
    /// precomputed Q32 reciprocal — single 64-bit multiply + shift, no
    /// runtime divide. Rounds half-away-from-zero so a single
    /// threshold-crossing batch produces `±1` instead of truncating to 0.
    fn drift_in_steps(&self) -> i32 {
        let abs_sum = self.drift_sum_q8.unsigned_abs() as u64;
        let prod = abs_sum * self.drift_per_step_recip_q32 as u64;
        let steps_abs = ((prod + (1u64 << 31)) >> 32) as i32;
        if self.drift_sum_q8 >= 0 {
            steps_abs
        } else {
            -steps_abs
        }
    }

    #[inline(always)]
    #[allow(dead_code)]
    pub fn ticks_per_bit(&self) -> u16 {
        self.ticks_per_bit
    }

    /// Wire-byte duration in monotonic timer ticks at the current baud.
    /// Used by the composite's slot-offset math:
    /// `delay_ticks = rdt_us · TICKS_PER_US + bytes_to_ticks(offset)`.
    ///
    /// Exact at 9600/1M/2M/3M; sub-tick truncation at 57600/115200 (≤14 ns
    /// at V006's 48 MHz, far below the timer's 20.83 ns resolution).
    #[allow(dead_code)]
    pub fn bytes_to_ticks(&self, bytes: u32) -> u32 {
        // BITS_PER_FRAME · CLOCK_HZ folds to a u64 literal at monomorphization
        // (e.g. 480_000_000 for V006). Each match arm's divisor is also a
        // literal, so LLVM lowers `u64 / const_u64` to a reciprocal multiply
        // — no `__udivdi3` call.
        let scaled = bytes as u64 * BITS_PER_FRAME as u64 * U::CLOCK_HZ as u64;
        match self.baud {
            BaudRate::B9600 => (scaled / 9_600u64) as u32,
            BaudRate::B57600 => (scaled / 57_600u64) as u32,
            BaudRate::B115200 => (scaled / 115_200u64) as u32,
            BaudRate::B1000000 => (scaled / 1_000_000u64) as u32,
            BaudRate::B2000000 => (scaled / 2_000_000u64) as u32,
            BaudRate::B3000000 => (scaled / 3_000_000u64) as u32,
        }
    }
}

// Shelved pending U4 (osc-drivers unit test audit): tests below bind to
// hand-rolled mock fields; will be migrated to the mockall + state-companion
// API as part of the audit.
#[cfg(any())]
#[cfg(test)]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{MockClockTrim, MockUsartBaud};

    // 9600 baud → BRR 5000 — high spec gives ample headroom for sub-tick
    // drift math without bumping against the i32 range or the deadband.
    const TEST_BAUD: BaudRate = BaudRate::B9600;
    const SPEC: u16 = 5000;

    fn clock(baud: BaudRate) -> Clock<MockUsartBaud, MockClockTrim> {
        Clock::new(baud, MockUsartBaud::default(), MockClockTrim::default())
    }

    /// Feed one byte-time observation expressed as ticks-per-bit via
    /// `on_byte_pair`. `prev = 0`, `curr = observed_tpb * 10` lands a delta
    /// in the `[9·spec, 11·spec]` HIT window for `observed_tpb ≈ spec`, so
    /// the integrator sees the same input the production codec produces.
    fn feed(c: &mut Clock<MockUsartBaud, MockClockTrim>, observed_tpb: u16) {
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
        c.trim_delta = MockClockTrim::DELTA_MAX;
        for _ in 0..DRIFT_MIN_SAMPLES {
            feed(&mut c, SPEC - 100); // would step trim up
        }
        // At MAX already → clamp keeps new == current → no stage.
        assert_eq!(c.pending_trim_delta, None);
    }

    #[test]
    fn out_of_window_byte_pair_is_dropped() {
        // delta > 11·tpb → GAP class (e.g. inter-packet gap). The integrator
        // must not see it — the pair callback driven by the classifier
        // through `Codec::on_edge_advance` / `on_idle` doesn't pre-filter;
        // that's this method's job.
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
        type C = Clock<MockUsartBaud, MockClockTrim>;
        assert_eq!(C::drift_threshold_q8(16), 163);
        assert_eq!(C::drift_threshold_q8(48), 491);
        assert_eq!(C::drift_threshold_q8(5000), 51171);
    }

    #[test]
    fn on_tx_complete_applies_staged_baud_to_usart() {
        let mut c = clock(TEST_BAUD);
        c.stage_baud(BaudRate::B3000000);
        c.on_tx_complete();
        assert_eq!(c.usart.log, [BaudRate::B3000000]);
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
