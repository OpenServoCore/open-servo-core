//! Clock-trim controller (`docs/osc-native-protocol.md` §9.3): folds windowed
//! cadence error into chip trim steps. One rule covers both phases — `steps =
//! round(err / step_effect)` — so the first window after boot takes a
//! multi-step jump while steady-state windows round to zero: round-to-nearest
//! IS the deadband, half a step wide, which is the physical optimum a stepped
//! trim can hold.
//!
//! The step effect is SELF-MEASURED: chip trim steps are nonuniform per chip
//! (bench 2026-07-11: 1.4–3.2k ppm/step against the 2.5k nominal), and a
//! fixed deadband either limit-cycles on coarse-step chips or under-trims
//! fine-step ones. Every applied correction is a free plant measurement — the
//! next window's error shift, divided by the steps taken — so the deadband
//! scales itself to THIS chip within one correction.
//!
//! CONTRACT (chip-independent): the output is signed trim steps from the
//! chip's factory default where **positive slows the oscillator**. The chip
//! adapter owns the mapping to its trim register's direction and never leaks
//! it upward — V006 HSITRIM runs higher = faster, so its adapter negates
//! (`hal::rcc::apply_clock_trim`); a same-sign mapping turns this loop's
//! negative feedback positive and rails the chip in `STEPS_MAX` clamps
//! (silicon 2026-07-11: the trim-runaway probe).

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
    /// Previous window's error and the correction it drew — the pair the
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
    /// Returns the new trim total when it changed — the caller applies it to
    /// the oscillator (positive = slower).
    pub fn on_window(&mut self, err_ticks: i32, span_ticks: u32) -> Option<i8> {
        if span_ticks == 0 {
            return None; // defensive: no span, no reading
        }
        let ppm = ((err_ticks as i64 * 1_000_000) / span_ticks as i64) as i32;
        if self.last_steps != 0 {
            let observed = (self.last_ppm - ppm) / self.last_steps;
            if (STEP_PPM_MIN..=STEP_PPM_MAX).contains(&observed) {
                self.step_ppm = observed;
            }
        }
        let steps = round_div(ppm, self.step_ppm).clamp(-STEPS_MAX, STEPS_MAX);
        let total = (self.total as i32 + steps).clamp(-TOTAL_MAX, TOTAL_MAX);
        // Record what was APPLIED, not what was asked: at the rail the plant
        // moved less than `steps`, and the step-effect division must match.
        let applied = total - self.total as i32;
        self.last_ppm = ppm;
        self.last_steps = applied;
        if applied == 0 {
            return None;
        }
        self.total = total as i8;
        Some(self.total)
    }
}

/// Round-to-nearest signed division, divisor positive.
fn round_div(n: i32, d: i32) -> i32 {
    let half = d / 2;
    if n >= 0 {
        (n + half) / d
    } else {
        (n - half) / d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
