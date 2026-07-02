//! HSI drift integrator for the DXL transport — a two-phase control law
//! that corrects the servo oscillator against the host's byte-pair timing.
//! Fed by classifier byte pairs via [`DriftIntegrator::on_byte_pair`],
//! commits its correction at the RX packet boundary via
//! [`DriftIntegrator::on_rx_packet_end`]:
//!
//! - **Boot** ([`DriftPhase::Boot`], active until the first batch close from
//!   [`DriftIntegrator::new`]): batch closes at [`DRIFT_MIN_SAMPLES_BOOT`] =
//!   6 byte pairs (one Ping reply's worth) with a full-step threshold and an
//!   emit cap of 16 register steps (full envelope). Lands ±20 000 ppm
//!   factory drift in a single packet so non-Fast commands work immediately
//!   after init.
//! - **Steady** ([`DriftPhase::Steady`], every batch after the first): batch
//!   closes at [`DRIFT_MIN_SAMPLES_STEADY`] = 20 pairs with a half-step
//!   threshold and an emit cap of 4 steps. Bounds noise risk and squeezes
//!   the residual gap before the host issues Fast commands.
//!
//! HSI drift is intrinsic to the oscillator, not the divider — baud changes
//! ([`DriftIntegrator::on_baud_change`]) rebuild the per-baud constants but
//! never re-engage boot mode.

use core::marker::PhantomData;

use osc_core::BaudRate;

use super::divisor_for;
use crate::dxl::uart::BITS_PER_FRAME;
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

/// Steady-phase drain cap, in byte pairs per packet. Caller (the driver)
/// reads via [`DriftIntegrator::samples_wanted_per_packet`] and truncates
/// the per-packet pair buffer before draining into
/// [`DriftIntegrator::on_byte_pair`]. At 4/packet ×
/// [`DRIFT_MIN_SAMPLES_STEADY`] = 20 → 5 packets fill a batch (vs 1–2
/// today), trading convergence latency for ~2–4 µs/cycle of drain work.
/// Per-packet skew tolerance unchanged: the integrator is already
/// noise-tolerant by design, and the missed pairs are time-uniform so no
/// systematic bias.
const STEADY_SAMPLES_PER_PACKET: u8 = 4;

/// Boot-phase drain cap, in byte pairs per packet. Matches
/// [`DRIFT_MIN_SAMPLES_BOOT`] so a single Ping's worth of pairs closes
/// the boot batch in one packet (the walker's cache emits 3 pairs from
/// the tail signature, ring walk adds the rest). Bounded — not
/// `u8::MAX` — so the walker doesn't spin ~250 free-run iterations per
/// Crc waiting for edges past the packet's boundary.
const BOOT_SAMPLES_PER_PACKET: u8 = DRIFT_MIN_SAMPLES_BOOT as u8;

/// Bundle of integrator constants that vary with `(baud, is_boot)`.
/// Grouped so a single `const { ... }` block folds all three fields at
/// monomorphization — see [[perf_optimization_workflow]] and
/// [`DriftIntegrator::drift_consts_at`] for why per-baud tabulation is
/// needed: the underlying `u64 / const_u64` divides don't lower to
/// reciprocal multiplies on RV32EC+Zmmul (no hardware `div`) and would
/// otherwise call `__udivdi3` on every baud change.
#[derive(Copy, Clone)]
struct DriftConsts {
    per_step_q8: u32,
    recip_q32: u32,
    window_recip_q32: u32,
}

/// Const-evaluable formula for one `(step_ppm, tpb, n_samples)` triple.
/// Runs at monomorphization when wrapped in `const { ... }`; the u64
/// divides fold to literals so no library call survives to runtime.
const fn compute_drift_consts(step_ppm: u32, ticks_per_bit: u16, n_samples: u16) -> DriftConsts {
    let per_step_q8 = {
        let prod = step_ppm as u64
            * ticks_per_bit as u64
            * n_samples as u64
            * BITS_PER_FRAME as u64
            * Q8_SCALE as u64;
        (prod / 1_000_000) as u32
    };
    let recip_q32 = {
        let denom = if per_step_q8 == 0 {
            1u64
        } else {
            per_step_q8 as u64
        };
        ((1u64 << 32) / denom) as u32
    };
    let window_recip_q32 = {
        let window = DRIFT_MIN_SAMPLES_STEADY as u64 * BITS_PER_FRAME as u64 * ticks_per_bit as u64;
        let denom = if window == 0 { 1u64 } else { window };
        ((1u64 << 32) / denom) as u32
    };
    DriftConsts {
        per_step_q8,
        recip_q32,
        window_recip_q32,
    }
}

pub struct DriftIntegrator<U: UsartBaud, T: ClockTrim> {
    trim: T,
    /// The reference clock rate (`U::CLOCK_HZ`) parameterizes the per-baud
    /// integrator constants even though the USART instance itself lives in
    /// [`super::baud_cache::BaudCache`]. `PhantomData` binds `U` so the
    /// const-fold arms in [`Self::drift_consts_at`] resolve `U::CLOCK_HZ`
    /// at monomorphization ([[const_fold_integrator_rebuild_divides]]).
    _usart: PhantomData<U>,
    baud: BaudRate,
    ticks_per_bit: u16,

    state: DriftPhase,
    /// Sum of `(byte_ticks − BITS_PER_FRAME × tpb) << 8` across the
    /// current batch. Tracking the raw byte-time delta (not `byte_ticks
    /// / BITS_PER_FRAME`) keeps 1-tick chip-stamp quantization noise
    /// below threshold at small `ticks_per_bit` (V006 at 3M baud:
    /// tpb=16, 1 tick = 6% of a bit).
    drift_sum_q8: i32,
    drift_samples: u16,
    /// `|drift_sum_q8|` value that flags a worth-emitting batch. Boot
    /// phase uses a full-step threshold (= `drift_per_step_q8`);
    /// steady uses a half-step (= `drift_per_step_q8 >> 1`).
    /// Precomputed on baud change and on phase transition.
    drift_threshold_q8: u32,
    /// One full trim step's worth of accumulated byte-tick drift over N
    /// samples, in Q8.8 ticks. Used at batch close to compute the
    /// sub-step residual (`drift_sum - actual_applied_steps × step_q8`).
    drift_per_step_q8: u32,
    /// Reciprocal of one full step's drift in Q32. Lets `on_byte_pair`
    /// estimate accumulated drift in trim-step units via a single 64-bit
    /// multiply + shift instead of a runtime divide. Precomputed
    /// alongside `drift_threshold_q8`.
    drift_per_step_recip_q32: u32,
    /// Reciprocal of the steady-phase integrator window (`N_steady × BPF ×
    /// tpb` HCLK ticks) in Q32. Lets [`Self::projected_phase_error_hclk`]
    /// scale the residual by `distance_hclk / window` via multiply + shift
    /// instead of a runtime divide. Precomputed alongside the other
    /// integrator consts (baud change is the only mutator).
    window_recip_q32: u32,
    /// Predicted next-window drift_sum after the most recent batch's apply
    /// takes effect — i.e. the sub-step residual the integer-step
    /// quantization couldn't cancel. Signed Q8.8 byte-ticks; sign matches
    /// drift convention (positive = HSI fast). Persists across batches
    /// until the next batch close overwrites it; consumers (the TX
    /// scheduler's phase-adjust math) read the latest estimate.
    residual_q8: i32,
    /// Currently-applied absolute correction relative to factory cal, in
    /// ppm. Mirrors the value last handed to `T::apply_ppm`; clamped to
    /// `T::ENVELOPE_PPM` before emission so we never request a correction
    /// the provider would saturate on.
    applied_ppm: i32,
    pending_applied_ppm: Option<i32>,
}

/// Two-phase drift control state. Boot lands factory drift in one packet;
/// the one-shot transition to Steady tightens the deadband for the rest of
/// the integrator's lifetime. HSI drift is the oscillator's, not the
/// divider's, so baud changes never re-engage Boot.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
enum DriftPhase {
    Boot,
    Steady,
}

impl<U: UsartBaud, T: ClockTrim> DriftIntegrator<U, T> {
    pub fn new(baud: BaudRate, ticks_per_bit: u16, trim: T) -> Self {
        let consts = Self::drift_consts_at(baud, true);
        Self {
            trim,
            _usart: PhantomData,
            baud,
            ticks_per_bit,
            state: DriftPhase::Boot,
            drift_sum_q8: 0,
            drift_samples: 0,
            // Boot phase: full-step threshold for ≥3.4σ SNR margin at 3M.
            drift_threshold_q8: consts.per_step_q8,
            drift_per_step_q8: consts.per_step_q8,
            drift_per_step_recip_q32: consts.recip_q32,
            window_recip_q32: consts.window_recip_q32,
            residual_q8: 0,
            applied_ppm: 0,
            pending_applied_ppm: None,
        }
    }

    #[inline(always)]
    fn is_boot(&self) -> bool {
        matches!(self.state, DriftPhase::Boot)
    }

    /// Return the precomputed integrator triple for `(baud, is_boot)`.
    /// One `const { ... }` block per arm folds the u64 divides at
    /// monomorphization — runtime cost is a 12-arm dispatch with baked
    /// literals, no `__udivdi3`. Formula reference:
    /// `drift_per_step_q8 = STEP_PPM × tpb × N × BITS_PER_FRAME × Q8_SCALE /
    /// 10⁶` (one step shifts a `BITS_PER_FRAME`-bit byte-time by
    /// `BITS_PER_FRAME × tpb × STEP_PPM / 10⁶` ticks; × Q8_SCALE lifts to
    /// Q8; × N gives full-step-over-N). Half-step (steady deadband) is
    /// applied by the caller via `>> 1`. Tracking drift on raw byte_ticks
    /// keeps 1-tick chip-stamp quantization noise below threshold at
    /// small `ticks_per_bit` (V006 at 3M: tpb=16, 1 tick = 6% of a bit).
    fn drift_consts_at(baud: BaudRate, is_boot: bool) -> DriftConsts {
        macro_rules! arm {
            ($baud:expr, $n:expr) => {
                const {
                    compute_drift_consts(
                        T::STEP_PPM,
                        divisor_for(U::CLOCK_HZ, $baud.as_hz()) as u16,
                        $n,
                    )
                }
            };
        }
        match (baud, is_boot) {
            (BaudRate::B9600, true) => arm!(BaudRate::B9600, DRIFT_MIN_SAMPLES_BOOT),
            (BaudRate::B9600, false) => arm!(BaudRate::B9600, DRIFT_MIN_SAMPLES_STEADY),
            (BaudRate::B57600, true) => arm!(BaudRate::B57600, DRIFT_MIN_SAMPLES_BOOT),
            (BaudRate::B57600, false) => arm!(BaudRate::B57600, DRIFT_MIN_SAMPLES_STEADY),
            (BaudRate::B115200, true) => arm!(BaudRate::B115200, DRIFT_MIN_SAMPLES_BOOT),
            (BaudRate::B115200, false) => arm!(BaudRate::B115200, DRIFT_MIN_SAMPLES_STEADY),
            (BaudRate::B1000000, true) => arm!(BaudRate::B1000000, DRIFT_MIN_SAMPLES_BOOT),
            (BaudRate::B1000000, false) => arm!(BaudRate::B1000000, DRIFT_MIN_SAMPLES_STEADY),
            (BaudRate::B2000000, true) => arm!(BaudRate::B2000000, DRIFT_MIN_SAMPLES_BOOT),
            (BaudRate::B2000000, false) => arm!(BaudRate::B2000000, DRIFT_MIN_SAMPLES_STEADY),
            (BaudRate::B3000000, true) => arm!(BaudRate::B3000000, DRIFT_MIN_SAMPLES_BOOT),
            (BaudRate::B3000000, false) => arm!(BaudRate::B3000000, DRIFT_MIN_SAMPLES_STEADY),
        }
    }

    /// Recompute integrator constants from the current `(baud, is_boot)`.
    /// Called on baud change (in [`Self::on_baud_change`]) and on phase
    /// transition (in [`Self::on_rx_packet_end`]) — both cold paths (at
    /// most twice per integrator lifetime in steady state).
    fn rebuild_integrator_consts(&mut self) {
        let consts = Self::drift_consts_at(self.baud, self.is_boot());
        // Boot: full-step threshold (more conservative deadband over the
        // smaller 6-sample window). Steady: half-step (tighter; the
        // 20-sample window has enough SNR margin to spot half-step drift).
        self.drift_threshold_q8 = if self.is_boot() {
            consts.per_step_q8
        } else {
            consts.per_step_q8 >> 1
        };
        self.drift_per_step_q8 = consts.per_step_q8;
        self.drift_per_step_recip_q32 = consts.recip_q32;
        self.window_recip_q32 = consts.window_recip_q32;
    }

    // -- events -----------------------------------------------------------------

    /// A committed baud change from the composite. Rebinds the per-baud
    /// integrator constants and resets the in-flight batch; the phase is
    /// untouched (drift is the oscillator's, not the divider's). The stale
    /// residual is dropped — it was measured against the old
    /// `drift_per_step_q8`.
    pub fn on_baud_change(&mut self, baud: BaudRate, ticks_per_bit: u16) {
        self.baud = baud;
        self.ticks_per_bit = ticks_per_bit;
        self.rebuild_integrator_consts();
        self.drift_sum_q8 = 0;
        self.drift_samples = 0;
        self.residual_q8 = 0;
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
        let min_samples = if self.is_boot() {
            DRIFT_MIN_SAMPLES_BOOT
        } else {
            DRIFT_MIN_SAMPLES_STEADY
        };
        if self.drift_samples < min_samples {
            return;
        }

        // Steps the upcoming apply will actually shift HSITRIM by (signed,
        // same direction as `applied_ppm` delta). 0 below threshold; may
        // be less than `capped` magnitude if the envelope clamps.
        let mut applied_steps: i32 = 0;
        if self.drift_sum_q8.unsigned_abs() >= self.drift_threshold_q8 {
            // Magnitude-aware: estimate accumulated drift in trim steps,
            // emit the opposing correction capped to bound noise risk.
            // Servo fast → observed ticks/bit short → counter ppm DOWN.
            let drift_steps = self.drift_in_steps();
            let counter = -drift_steps;
            let cap = if self.is_boot() {
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
            applied_steps = (new_applied - self.applied_ppm) / T::STEP_PPM as i32;
            if new_applied != self.applied_ppm {
                self.pending_applied_ppm = Some(new_applied);
            }
        }
        // Predicted next-window drift_sum after the apply takes effect —
        // the sub-step residual that integer-step quantization couldn't
        // cancel. Applying +1 step nudges HSI faster by STEP_PPM ppm,
        // which adds `drift_per_step_q8` to next window's drift_sum (per
        // the `drift_per_step_q8` derivation). For signed `applied_steps`:
        //
        //   predicted = drift_sum + applied_steps × drift_per_step_q8
        //
        // Example: drift_sum = +30 (HSI fast), applied_steps = -1 (slowing
        // direction), drift_per_step_q8 = 24 → predicted = +6, still
        // slightly fast — quantization under-corrected by 6.
        let delta = (applied_steps as i64 * self.drift_per_step_q8 as i64)
            .clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        self.residual_q8 = self.drift_sum_q8.saturating_add(delta);
        self.drift_sum_q8 = 0;
        self.drift_samples = 0;
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
    /// Idempotent: with no pending correction and already in
    /// [`DriftPhase::Steady`], the call is a no-op.
    pub fn on_rx_packet_end(&mut self) {
        let mut reset_integrator = false;
        if let Some(applied) = self.pending_applied_ppm.take() {
            self.trim.apply_ppm(applied);
            self.applied_ppm = applied;
            reset_integrator = true;
        }
        if self.is_boot() {
            self.state = DriftPhase::Steady;
            reset_integrator = true;
            self.rebuild_integrator_consts();
            // Boot batch close stored a residual accumulated over only
            // `DRIFT_MIN_SAMPLES_BOOT` (6) pairs, but the projection scales
            // by the steady window (20 pairs). Drop it; the first steady
            // batch overwrites in ~20 byte pairs of wall-clock and host-
            // side Fast TX is unlikely to fire in that gap (host has to
            // finish init protocol first).
            self.residual_q8 = 0;
        }
        if reset_integrator {
            self.drift_sum_q8 = 0;
            self.drift_samples = 0;
        }
    }

    // -- accessors --------------------------------------------------------------

    /// Max byte pairs per packet the driver should drain into
    /// [`Self::on_byte_pair`]. Boot: [`BOOT_SAMPLES_PER_PACKET`] — one
    /// Ping's worth of pairs closes the boot batch in a single packet.
    /// Steady: [`STEADY_SAMPLES_PER_PACKET`] — drift is slow, so a
    /// sparser feed across more packets is fine, and the saved drain
    /// work shows up as ~2–4 µs/cycle on the chip-side floor.
    ///
    /// Per [[drift_sampling_instruction_only]]: both own + foreign
    /// Instruction packets feed (gated by `hsi_active` upstream at the
    /// codec); Status frames never contribute regardless of cap.
    pub fn samples_wanted_per_packet(&self) -> u8 {
        if self.is_boot() {
            BOOT_SAMPLES_PER_PACKET
        } else {
            STEADY_SAMPLES_PER_PACKET
        }
    }

    /// Project the integrator's pending residual to a chip-HCLK-tick
    /// offset that, added to a wall-clock-relative deadline, lands the
    /// fire moment on the host's reference. Sign: positive = HSI fast =
    /// chip-clock ticks accumulate ahead of wall clock → deadline must
    /// shift LATER (added) by the projected amount. Linear scaling: the
    /// integrator's window-summed residual scales by `distance_hclk /
    /// window` to the deadline distance, then strips Q8.
    ///
    /// Uses the precomputed [`Self::window_recip_q32`] reciprocal — a
    /// `__udivdi3` here would land on every scheduled deadline, and
    /// RV32EC has no hardware divide. The wide multiply lowers to a
    /// single `__multi3` call (~tens of cycles) instead.
    pub fn projected_phase_error_hclk(&self, distance_hclk: u32) -> i32 {
        if self.residual_q8 == 0 || distance_hclk == 0 {
            return 0;
        }
        // residual_q8 × distance / (window × Q8_SCALE)
        //   = residual_q8 × distance × window_recip_q32 >> (32 + 8)
        //   = residual_q8 × distance × window_recip_q32 >> 40
        //
        // u128 keeps the multiply correct for the rare saturated
        // residual_q8 (near i32::MAX) case without silently truncating.
        let abs_res = self.residual_q8.unsigned_abs() as u128;
        let prod = abs_res * distance_hclk as u128 * self.window_recip_q32 as u128;
        let mag = (prod >> 40).min(i32::MAX as u128) as i32;
        if self.residual_q8 >= 0 { mag } else { -mag }
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

    fn make(baud: BaudRate, ticks_per_bit: u16) -> (TestIntegrator, ClockTrimState) {
        let (trim, t_state) = mk_clock_trim();
        (DriftIntegrator::new(baud, ticks_per_bit, trim), t_state)
    }

    fn integrator_9600() -> TestIntegrator {
        make(BaudRate::B9600, SPEC_9600).0
    }

    /// Feed one classifier byte pair where `observed_tpb` sets `byte_ticks =
    /// observed_tpb * BITS_PER_FRAME`. Per-sample drift is
    /// `BITS_PER_FRAME × (observed_tpb − tpb)`.
    fn feed(c: &mut TestIntegrator, observed_tpb: u16) {
        c.on_byte_pair(0, observed_tpb.wrapping_mul(BITS_PER_FRAME));
    }

    // ---------- boot integrator ----------
    //
    // At B9600: tpb = 5000; boot batch = 6 pairs; drift_per_step_q8 =
    // STEP_PPM · tpb · 6 · BITS_PER_FRAME · Q8_SCALE / 10⁶ = 2500 · 5000 · 6
    // · 10 · 256 / 10⁶ = 192_000. Full-step boot deadband = 192_000.

    #[test]
    fn single_sample_does_not_stage_ppm() {
        let mut c = integrator_9600();
        feed(&mut c, SPEC_9600 + 40);
        assert_eq!(c.pending_applied_ppm, None);
        assert_eq!(c.drift_samples, 1);
    }

    #[test]
    fn boot_batch_at_zero_drift_stages_nothing() {
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600);
        }
        assert_eq!(c.pending_applied_ppm, None);
    }

    #[test]
    fn boot_batch_within_deadband_stages_nothing() {
        // Per-sample drift = 40 (raw ticks); 6 · 40 · 256 = 61_440 <
        // 192_000 full-step boot deadband.
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 + 4);
        }
        assert_eq!(c.pending_applied_ppm, None);
    }

    #[test]
    fn boot_batch_positive_drift_stages_negative_ppm() {
        // Per-sample drift = 400; 6 · 400 · 256 = 614_400 > 192_000.
        // drift_in_steps rounds to 3; counter = −3; nudge = −7500 ppm.
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 + 40);
        }
        assert_eq!(c.pending_applied_ppm, Some(-7500));
    }

    #[test]
    fn boot_batch_negative_drift_stages_positive_ppm() {
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 - 40);
        }
        assert_eq!(c.pending_applied_ppm, Some(7500));
    }

    #[test]
    fn integrator_resets_after_batch_close() {
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 + 40);
        }
        assert_eq!(c.drift_sum_q8, 0);
        assert_eq!(c.drift_samples, 0);
    }

    // ---------- envelope clamp ----------

    #[test]
    fn upper_envelope_bound_clamps_positive_nudge() {
        // Per-sample drift = −2000; 6 · −2000 · 256 = −3_072_000; magnitude
        // matches 16 · drift_per_step_q8 → drift_in_steps = 16 (at emit cap
        // ceiling), counter = +16, nudge = +40_000. Envelope upper bound
        // 37_500 clamps → stage = Some(37_500).
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 - 200);
        }
        assert_eq!(c.pending_applied_ppm, Some(MockClockTrim::ENVELOPE_PPM.1));
    }

    // ---------- boot → steady transition ----------

    #[test]
    fn samples_wanted_per_packet_reflects_phase() {
        let mut c = integrator_9600();
        assert_eq!(c.samples_wanted_per_packet(), BOOT_SAMPLES_PER_PACKET);
        c.on_rx_packet_end();
        assert_eq!(c.samples_wanted_per_packet(), STEADY_SAMPLES_PER_PACKET);
    }

    #[test]
    fn on_rx_packet_end_transitions_out_of_boot() {
        let mut c = integrator_9600();
        assert!(c.is_boot());
        c.on_rx_packet_end();
        assert!(!c.is_boot());
    }

    #[test]
    fn steady_batch_closes_at_20_pairs_not_6() {
        let mut c = integrator_9600();
        c.on_rx_packet_end(); // boot → steady, integrator reset.
        // 6 pairs at boot-trip drift no longer close a batch in steady.
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 + 40);
        }
        assert_eq!(c.pending_applied_ppm, None);
        // Fill to the 20-pair steady window; drift trips the (halved) deadband.
        for _ in DRIFT_MIN_SAMPLES_BOOT..DRIFT_MIN_SAMPLES_STEADY {
            feed(&mut c, SPEC_9600 + 40);
        }
        assert_eq!(c.pending_applied_ppm, Some(-7500));
    }

    // ---------- trim apply ----------

    #[test]
    fn on_rx_packet_end_applies_pending_ppm_to_trim() {
        let (mut c, t_state) = make(BaudRate::B9600, SPEC_9600);
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 + 40);
        }
        assert_eq!(c.pending_applied_ppm, Some(-7500));
        c.on_rx_packet_end();
        assert_eq!(t_state.apply_ppm_log(), alloc::vec![-7500]);
        assert_eq!(c.applied_ppm, -7500);
        assert_eq!(c.pending_applied_ppm, None);
    }

    // ---------- projected_phase_error_hclk ----------

    #[test]
    fn projected_phase_error_zero_with_no_residual() {
        let c = integrator_9600();
        assert_eq!(c.projected_phase_error_hclk(100_000), 0);
    }

    #[test]
    fn projected_phase_error_zero_at_zero_distance() {
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 + 40);
        }
        assert_ne!(c.residual_q8, 0);
        assert_eq!(c.projected_phase_error_hclk(0), 0);
    }

    #[test]
    fn projected_phase_error_sign_matches_residual() {
        // Positive drift → positive drift_sum → integer quantization leaves a
        // positive residual after the −3 emit; projected error is positive.
        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 + 40);
        }
        assert!(c.residual_q8 > 0);
        assert!(c.projected_phase_error_hclk(100_000) > 0);

        let mut c = integrator_9600();
        for _ in 0..DRIFT_MIN_SAMPLES_BOOT {
            feed(&mut c, SPEC_9600 - 40);
        }
        assert!(c.residual_q8 < 0);
        assert!(c.projected_phase_error_hclk(100_000) < 0);
    }
}
