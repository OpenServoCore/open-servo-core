//! Clock-trim controller (`docs/osc-native-protocol.md` sec 9.3): folds
//! windowed cadence error into chip trim steps. One rule covers both phases
//! -- `steps = round(err / step_effect)` -- so the first window after boot
//! takes a multi-step jump while steady-state windows round to zero:
//! round-to-nearest IS the deadband, half a step wide, which is the physical
//! optimum a stepped trim can hold.
//!
//! The step effect is SELF-MEASURED: chip trim steps are nonuniform per chip
//! (bench: 1.4-3.2k ppm/step against the 2.5k nominal), and a fixed deadband
//! either limit-cycles on coarse-step chips or under-trims fine-step ones.
//! Every applied correction is a free plant measurement -- the next window's
//! error shift, divided by the steps taken -- so the deadband scales itself
//! to THIS chip within one correction.
//!
//! CONTRACT (chip-independent): the output is signed trim steps from the
//! chip's factory default where **positive slows the oscillator**. The chip
//! adapter owns the mapping to its trim register's direction and never leaks
//! it upward -- V006 HSITRIM runs higher = faster, so its adapter negates
//! (`hal::rcc::apply_clock_trim`); a same-sign mapping turns this loop's
//! negative feedback positive and rails the chip in `STEPS_MAX` clamps
//! (the trim-runaway probe pins this).

/// Self-measured step-effect acceptance band, ppm per step. A window
/// polluted by traffic noise or a thermal shift between sparse windows can
/// imply an absurd plant gain; readings outside the band are discarded and
/// the previous estimate kept.
const STEP_PPM_MIN: i32 = 800;
const STEP_PPM_MAX: i32 = 4000;

/// Largest correction one window may take. Bounds a garbage window's damage
/// to something the next window walks back, and keeps any single over-the-air
/// rate jump well inside UART framing tolerance.
const STEPS_MAX: i32 = 4;

/// Accumulated-trim rail, steps from the factory default (the chip register
/// clamps harder; this keeps the controller's own state windup-free).
const TOTAL_MAX: i32 = 15;

pub struct TrimLoop {
    /// Applied trim, in chip steps from the factory default.
    total: i8,
    /// Plant gain: ppm of clock shift per trim step, seeded at the chip's
    /// nominal and replaced by measurement after the first correction.
    step_ppm: i32,
    /// Previous window's error and the correction it drew -- the pair the
    /// next window turns into a step-effect measurement.
    last_ppm: i32,
    last_steps: i32,
}

impl TrimLoop {
    pub const fn new(step_ppm_nominal: u32) -> Self {
        Self {
            total: 0,
            step_ppm: step_ppm_nominal as i32,
            last_ppm: 0,
            last_steps: 0,
        }
    }

    /// One measurement window: `err_ticks` of local-clock excess over
    /// `span_ticks` of nominal wire-byte span (positive = local clock fast).
    /// Returns the new trim total when it changed -- the caller applies it to
    /// the oscillator (positive = slower).
    pub fn on_window(&mut self, err_ticks: i32, span_ticks: u32) -> Option<i8> {
        if span_ticks == 0 {
            return None; // defensive: no span, no reading
        }
        let ppm = ppm(err_ticks, span_ticks);
        if self.last_steps != 0 {
            // Truncating division by the +/-1..=4 step count: |a/b| == |a|/|b|
            // signed by XOR; constant divisors lower to mulhu magic, no libcall.
            let delta = self.last_ppm - ppm;
            let mag = delta.unsigned_abs();
            let q = match self.last_steps.unsigned_abs() {
                1 => mag,
                2 => mag / 2,
                3 => mag / 3,
                _ => mag / 4,
            };
            let observed = if (delta < 0) == (self.last_steps < 0) {
                q as i32
            } else {
                -(q as i32)
            };
            if (STEP_PPM_MIN..=STEP_PPM_MAX).contains(&observed) {
                self.step_ppm = observed;
            }
        }
        // round(ppm / step_ppm) clamped to +/-STEPS_MAX, division-free: the
        // half-step boundaries sit at odd multiples of step_ppm, so the
        // clamped quotient is STEPS_MAX threshold compares.
        let d = self.step_ppm as u32;
        let t = 2 * ppm.unsigned_abs();
        let mut mag = 0;
        for k in 0..STEPS_MAX as u32 {
            mag += (t >= (2 * k + 1) * d) as i32;
        }
        let steps = if ppm < 0 { -mag } else { mag };
        let total = (self.total as i32 + steps).clamp(-TOTAL_MAX, TOTAL_MAX);
        // Record what was APPLIED, not what was asked: at the rail the plant
        // moved less than `steps`, and the step-effect division must match.
        let applied = total - self.total as i32;
        crate::bench::trim_probe(|p| {
            p.windows += 1;
            p.tw_ppm = ppm;
            p.tw_effect = self.step_ppm;
            p.tw_applied = applied;
            p.tw_total = total;
        });
        self.last_ppm = ppm;
        self.last_steps = applied;
        if applied == 0 {
            return None;
        }
        self.total = total as i8;
        Some(self.total)
    }
}

/// Exact `err * 1_000_000 / span`, truncated toward zero. Hand-rolled
/// restoring division: the chip has no divider and the soft-arith gate bans
/// the libcall. Every feeder gates |err| <= span and 1e6 < 2^20, so the
/// quotient fits 20 bits; every u64 op here lowers inline on rv32+zmmul.
pub(super) fn ppm(err: i32, span: u32) -> i32 {
    debug_assert!(span != 0);
    let neg = err < 0;
    let mut rem = err.unsigned_abs() as u64 * 1_000_000;
    let mut q: u32 = 0;
    // Out-of-domain net: |err| > span means an upstream gate broke;
    // saturate so the downstream sanity bands discard the reading.
    if rem >= (span as u64) << 20 {
        q = (1 << 20) - 1;
        rem = 0;
    }
    let mut dshift = (span as u64) << 19;
    let mut bit: u32 = 1 << 19;
    while bit != 0 {
        if rem >= dshift {
            rem -= dshift;
            q |= bit;
        }
        dshift >>= 1;
        bit >>= 1;
    }
    if neg { -(q as i32) } else { q as i32 }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-to-nearest signed division, divisor positive -- the original
    /// closed form the threshold compares in `on_window` replaced. Kept as a
    /// test oracle only.
    fn round_div_reference(n: i32, d: i32) -> i32 {
        let half = d / 2;
        if n >= 0 {
            (n + half) / d
        } else {
            (n - half) / d
        }
    }

    /// Closed-loop plant: a chip with a true clock offset and a true (possibly
    /// off-nominal) step effect, measured through noiseless windows.
    struct Plant {
        offset_ppm: i32,
        step_ppm: i32,
        trim: TrimLoop,
        applied: i32,
    }

    impl Plant {
        fn new(offset_ppm: i32, step_ppm: i32) -> Self {
            Self {
                offset_ppm,
                step_ppm,
                trim: TrimLoop::new(2500),
                applied: 0,
            }
        }

        fn residual(&self) -> i32 {
            self.offset_ppm - self.applied * self.step_ppm
        }

        fn window(&mut self) -> Option<i8> {
            let span = 1_000_000u32;
            let err = self.residual(); // err_ticks == ppm at this span
            let out = self.trim.on_window(err, span);
            if let Some(total) = out {
                self.applied = total as i32;
            }
            out
        }
    }

    #[test]
    fn first_window_takes_the_acquire_jump() {
        let mut p = Plant::new(5200, 2500);
        assert_eq!(p.window(), Some(2));
        assert_eq!(p.residual(), 200);
    }

    #[test]
    fn steady_state_rounds_to_zero_inside_half_a_step() {
        let mut p = Plant::new(1200, 2500);
        assert_eq!(p.window(), None);
        assert_eq!(p.window(), None);
    }

    #[test]
    fn coarse_step_chip_locks_without_limit_cycling() {
        // 3.2k-ppm steps under the 2.5k seed: one overshoot, then the
        // measured gain widens the deadband and the loop holds.
        let mut p = Plant::new(1700, 3200);
        assert_eq!(p.window(), Some(1));
        assert_eq!(p.residual(), -1500);
        for _ in 0..8 {
            assert_eq!(p.window(), None);
        }
        assert_eq!(p.trim.step_ppm, 3200);
    }

    #[test]
    fn fine_step_chip_converges_to_its_own_optimum() {
        let mut p = Plant::new(5000, 1400);
        p.window(); // acquire on the nominal seed
        for _ in 0..8 {
            p.window();
        }
        assert_eq!(p.trim.step_ppm, 1400);
        assert!(p.residual().abs() <= 700, "residual {}", p.residual());
        for _ in 0..8 {
            assert_eq!(p.window(), None);
        }
    }

    #[test]
    fn negative_offset_converges_symmetrically() {
        let mut p = Plant::new(-5000, 1400);
        for _ in 0..8 {
            p.window();
        }
        assert!(p.residual().abs() <= 700, "residual {}", p.residual());
        assert!(p.applied < 0);
    }

    #[test]
    fn rail_pins_without_windup() {
        // An offset past the throw: the total pins at the rail and stays
        // responsive (backs off immediately once the sign flips).
        let mut p = Plant::new(60_000, 2500);
        for _ in 0..12 {
            p.window();
        }
        assert_eq!(p.applied, 15);
        assert_eq!(p.window(), None); // at the rail: applied == 0, no windup
        p.offset_ppm = 0;
        let out = p.window();
        assert!(out.is_some() && p.applied < 15, "applied {}", p.applied);
    }

    #[test]
    fn absurd_gain_readings_are_rejected() {
        let mut p = Plant::new(5000, 2500);
        p.window();
        // Pollute the next window: a thermal jump mimics a huge plant gain.
        p.offset_ppm -= 9000;
        p.window();
        assert_eq!(p.trim.step_ppm, 2500, "seed survives the outlier");
    }

    #[test]
    fn zero_span_reads_nothing() {
        let mut t = TrimLoop::new(2500);
        assert_eq!(t.on_window(1000, 0), None);
    }

    #[test]
    fn ppm_matches_reference_division() {
        // Bit-identical to the i64 quotient across the helper's domain
        // (|err| <= span); errs are generated FROM each span so the bound
        // holds by construction and the saturation net never trips.
        fn reference(err: i32, span: u32) -> i32 {
            (err as i64 * 1_000_000 / span as i64) as i32
        }
        let spans: [u32; 11] = [
            1,
            2,
            3,
            16,
            4_000,
            16_000,
            153_600,
            4_300_000,
            8_600_000,
            2_147_483_647,
            u32::MAX,
        ];
        let mut cases = 0u32;
        for &span in &spans {
            let bound = (span as i64).min(i32::MAX as i64);
            // Boundary and interior magnitudes, each clamped into [0, bound].
            let mags = [
                0,
                1,
                2,
                (span / 16) as i64,
                (span / 2) as i64,
                bound - 1,
                bound,
            ];
            for &m in &mags {
                if !(0..=bound).contains(&m) {
                    continue;
                }
                let err = m as i32;
                assert_eq!(
                    ppm(err, span),
                    reference(err, span),
                    "err {err} span {span}"
                );
                assert_eq!(
                    ppm(-err, span),
                    reference(-err, span),
                    "err {} span {span}",
                    -err
                );
                cases += 2;
            }
        }
        assert!(cases >= 100, "grid too small: {cases}");

        // Explicit saturation path (|err| > span, OUTSIDE the domain): the
        // helper clamps to +/-(2^20 - 1) rather than dividing, carrying the
        // input sign. The magnitude sits far past every downstream sanity band
        // (drift 8000 ppm, step-effect 4000 ppm) so the reading is discarded.
        let sat = ppm(1_000_000, 4);
        assert!(sat >= 1 << 19, "saturation too small: {sat}");
        assert_eq!(sat, (1 << 20) - 1);
        // 8000 = clock::DRIFT_SANITY_PPM (the widest downstream band); the
        // saturated magnitude clears it and the STEP_PPM band by >100x.
        assert!(sat.unsigned_abs() > 8_000);
        assert!(sat > STEP_PPM_MAX);
        assert_eq!(ppm(-1_000_000, 4), -((1 << 20) - 1));
    }

    #[test]
    fn steps_threshold_matches_round_div_reference() {
        // The four-compare step selector must equal round_div_reference
        // clamped to +/-STEPS_MAX across every plausible (d, n).
        fn threshold(n: i32, d: u32) -> i32 {
            let t = 2 * n.unsigned_abs();
            let mag =
                (t >= d) as i32 + (t >= 3 * d) as i32 + (t >= 5 * d) as i32 + (t >= 7 * d) as i32;
            if n < 0 { -mag } else { mag }
        }
        for d in (STEP_PPM_MIN..=STEP_PPM_MAX).step_by(37) {
            for n in (-(9 * d)..=(9 * d)).step_by(53) {
                assert_eq!(
                    threshold(n, d as u32),
                    round_div_reference(n, d).clamp(-STEPS_MAX, STEPS_MAX),
                    "d {d} n {n}"
                );
            }
        }
        // Exact half-step ties, even and odd d, both signs -- the load-bearing
        // boundary the coarse sweep above straddles rather than lands on.
        for &d in &[800i32, 801, 2500, 2501, 4000] {
            for k in 0..=4 {
                let n = k * d + d / 2;
                assert_eq!(
                    threshold(n, d as u32),
                    round_div_reference(n, d).clamp(-STEPS_MAX, STEPS_MAX),
                    "tie d {d} n {n}"
                );
                assert_eq!(
                    threshold(-n, d as u32),
                    round_div_reference(-n, d).clamp(-STEPS_MAX, STEPS_MAX),
                    "tie d {d} n {}",
                    -n
                );
            }
        }
    }

    #[test]
    fn gain_constant_divisor_matches_truncating_division() {
        // The match-on-|steps| gain divisor must equal signed truncating
        // division for every divisor the loop can record.
        fn gain(delta: i32, steps: i32) -> i32 {
            let mag = delta.unsigned_abs();
            let q = match steps.unsigned_abs() {
                1 => mag,
                2 => mag / 2,
                3 => mag / 3,
                _ => mag / 4,
            };
            if (delta < 0) == (steps < 0) {
                q as i32
            } else {
                -(q as i32)
            }
        }
        for delta in (-20_000..=20_000).step_by(37) {
            for steps in [-4, -3, -2, -1, 1, 2, 3, 4] {
                assert_eq!(
                    gain(delta, steps),
                    delta / steps,
                    "delta {delta} steps {steps}"
                );
            }
        }
    }
}
