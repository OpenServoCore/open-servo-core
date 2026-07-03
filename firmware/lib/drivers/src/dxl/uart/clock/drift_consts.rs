//! Per-baud integrator constants for [`DriftIntegrator`], const-folded at
//! monomorphization. Split from the integrator so the control law reads as
//! policy and this file as arithmetic.
//!
//! [`DriftIntegrator`]: super::drift_integrator::DriftIntegrator

use osc_core::BaudRate;

use super::divisor_for;
use crate::dxl::uart::BITS_PER_FRAME;
use crate::traits::dxl::{ClockTrim, UsartBaud};

/// Bit width of the integrator's fixed-point lift.
pub(super) const Q8_SHIFT: u32 = 8;
/// `× Q8_SCALE` shifts byte-tick deltas into the integrator's accumulator
/// domain so sub-tick drift survives summation.
pub(super) const Q8_SCALE: u32 = 1 << Q8_SHIFT;

/// Batch size during the boot (first-emit) phase. One Ping instruction +
/// reply supplies ~6 in-window byte pairs (anchor at byte 3 + 6 forward
/// hits across bytes 4–9), so the boot batch closes inside the first
/// instruction the servo parses.
pub(super) const DRIFT_MIN_SAMPLES_BOOT: u16 = 6;
/// Batch size after the boot phase's one-shot emission. At N=20 the half-step
/// deadband sits 2.8σ above per-sample chip-stamp quantization noise
/// under the conservative 1-tick σ model (9.8σ at the realistic uniform
/// 1-LSB σ) at the worst-case 3M baud — false-emit rate ~5e-3 per batch
/// conservative, ~0 uniform. ~one bogus correction every ~4 minutes at
/// sustained 3M load in the conservative envelope; the next batch
/// immediately reverts the spurious ±1-step nudge.
pub(super) const DRIFT_MIN_SAMPLES_STEADY: u16 = 20;

/// Bundle of integrator constants that vary with `(baud, is_boot)`.
/// Grouped so a single `const { ... }` block folds all three fields at
/// monomorphization — see [[perf_optimization_workflow]] and
/// [`drift_consts_at`] for why per-baud tabulation is needed: the
/// underlying `u64 / const_u64` divides don't lower to reciprocal
/// multiplies on RV32EC+Zmmul (no hardware `div`) and would otherwise
/// call `__udivdi3` on every baud change.
#[derive(Copy, Clone)]
pub(super) struct DriftConsts {
    pub(super) per_step_q8: u32,
    pub(super) recip_q32: u32,
    pub(super) window_recip_q32: u32,
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

/// Return the precomputed integrator triple for `(baud, is_boot)`.
/// One `const { ... }` block per arm folds the u64 divides at
/// monomorphization — runtime cost is a 12-arm dispatch with baked
/// literals, no `__udivdi3`. Formula reference:
/// `drift_per_step_q8 = STEP_PPM × tpb × N × BITS_PER_FRAME × Q8_SCALE /
/// 10⁶` (one step shifts a `BITS_PER_FRAME`-bit byte-time by
/// `BITS_PER_FRAME × tpb × STEP_PPM / 10⁶` ticks; × Q8_SCALE lifts to
/// Q8; × N gives full-step-over-N). Half-step (steady deadband) is
/// applied by the caller. Tracking drift on raw byte_ticks keeps 1-tick
/// chip-stamp quantization noise below threshold at small
/// `ticks_per_bit` (V006 at 3M: tpb=16, 1 tick = 6% of a bit).
pub(super) fn drift_consts_at<U: UsartBaud, T: ClockTrim>(
    baud: BaudRate,
    is_boot: bool,
) -> DriftConsts {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{MockClockTrim, MockUsartBaud};

    #[test]
    fn b9600_steady_triple_matches_hand_computation() {
        // STEP_PPM 2500, tpb 5000 (48 MHz / 9600), N 20:
        // per_step_q8 = 2500 · 5000 · 20 · 10 · 256 / 10⁶ = 640_000;
        // recip_q32 = 2³² / 640_000 = 6710;
        // window_recip_q32 = 2³² / (20 · 10 · 5000) = 4294.
        let c = drift_consts_at::<MockUsartBaud, MockClockTrim>(BaudRate::B9600, false);
        assert_eq!(c.per_step_q8, 640_000);
        assert_eq!(c.recip_q32, 6710);
        assert_eq!(c.window_recip_q32, 4294);
    }
}
