//! CAL trim under ruler noise (protocol sec 9.3): a simulated chip with a
//! true per-code trim step table, measured through the real ruler and
//! `TrimLoop`, judged on the chip's TRUE clock error and the codes it
//! visits, never on the loop's own estimates.
//!
//! Every bound is the ceiling of the ideal stepped trim: perfect step
//! knowledge, round-to-nearest on the same noisy reading. Its truth after a
//! decision lies within half a step plus that train's noise, so it leaves
//! `band_ppm` only on a noise draw past `BAND_K` sd, and its codes inside a
//! window span at most the codes `BAND_K` sd of noise can reach.

use super::ClockDiscipline;
use std::collections::VecDeque;

/// V006 SysTick at HCLK 48 MHz.
const TICKS_PER_US: u32 = 48;
/// V006 HSITRIM nominal: 60 kHz per step on the 24 MHz HSI.
const STEP_NOMINAL_PPM: u32 = 2_500;
/// The bench CAL train (tools/bench hardware trim.rs).
const GAP_US: u16 = 400;
const GAPS: u8 = 8;
const SPAN_US: f64 = GAP_US as f64 * GAPS as f64;
/// drivers `trim::STEPS_MAX` and `trim::TOTAL_MAX`.
const STEPS_MAX: i32 = 4;
const TOTAL_MAX: i32 = 15;
const CODES: usize = 2 * TOTAL_MAX as usize;
/// Trains a boot anchor may take (tools/bench `ANCHOR_TRAINS_MAX`).
const SETTLE_TRAINS: usize = (TOTAL_MAX as usize).div_ceil(STEPS_MAX as usize) + 3;
/// Local ticks between trains: only moves the stamps through the u32 wrap.
const CAL_PERIOD_TICKS: u32 = 480_000_017;
/// One 0x00 character's low time in bits: the break the wake stamps.
const BREAK_BITS: f64 = 10.0;
const BOOT_BAUD: f64 = 1_000_000.0;
/// One BRR step off 1M (tools/bench `DETUNE_BAUD`).
const DETUNE_BAUD: f64 = 993_103.0;

/// Bench chip: median self-measured step effect 2968 ppm/step.
const COARSE_STEP_PPM: f64 = 3_000.0;
/// Finest bench chip step.
const FINE_STEP_PPM: f64 = 1_400.0;
/// Per-code step spread of the nonuniform chip, +/- fraction (bringup kb
/// trim-validation model).
const STEP_SPREAD: f64 = 0.15;
const CHIPS: [Steps; 3] = [
    Steps::Uniform(COARSE_STEP_PPM),
    Steps::Nonuniform(COARSE_STEP_PPM),
    Steps::Uniform(FINE_STEP_PPM),
];

/// Per-train ruler noise sd the gate runs at (bringup kb trim-stepfx
/// model: 415-600 ppm, not yet bench-measured).
const GATE_NOISE_PPM: f64 = 600.0;
const BAND_K: f64 = 3.0;
/// P(|z| > BAND_K) for a standard normal.
const NOISE_TAIL_P: f64 = 0.0027;
const SWING_WINDOW: usize = 6;

const BOOT_REACH_STEPS: f64 = 8.0;
/// Boot reach that keeps an 8-step jump inside the rail on the finest
/// nonuniform code.
const JUMP_REACH_STEPS: f64 = 3.0;
const EPISODES: usize = 100;
const JUDGED_TRAINS: usize = 100;
const SEED: u64 = 0x05C_CA1;

/// SplitMix64.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in (0, 1].
    fn unit(&mut self) -> f64 {
        ((self.next() >> 11) + 1) as f64 / (1u64 << 53) as f64
    }

    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn gauss(&mut self) -> f64 {
        let (u, v) = (self.unit(), self.unit());
        (-2.0 * u.ln()).sqrt() * (core::f64::consts::TAU * v).cos()
    }
}

#[derive(Clone, Copy, Debug)]
enum Steps {
    Uniform(f64),
    Nonuniform(f64),
}

/// True clock error at code 0 (ppm, positive = fast) and the ppm each code
/// step slows it: `steps[c + TOTAL_MAX]` lies between codes `c` and `c + 1`.
struct Chip {
    offset_ppm: f64,
    steps: [f64; CODES],
}

impl Chip {
    /// Booted within +/-`reach` nominal steps of its optimum code.
    fn new(kind: Steps, reach: f64, rng: &mut Rng) -> Self {
        let steps = core::array::from_fn(|_| match kind {
            Steps::Uniform(s) => s,
            Steps::Nonuniform(s) => s * (1.0 + rng.uniform(-STEP_SPREAD, STEP_SPREAD)),
        });
        let mut chip = Self {
            offset_ppm: 0.0,
            steps,
        };
        let reach = reach * chip.nominal();
        chip.offset_ppm = rng.uniform(-reach, reach);
        chip
    }

    fn nominal(&self) -> f64 {
        self.steps.iter().sum::<f64>() / CODES as f64
    }

    fn band_ppm(&self, noise_ppm: f64) -> f64 {
        let coarsest = self.steps.iter().copied().fold(0.0, f64::max);
        coarsest.max(STEP_NOMINAL_PPM as f64) / 2.0 + BAND_K * noise_ppm
    }

    /// Codes `BAND_K` sd of noise can spread the ideal trim across.
    fn swing_max(&self, noise_ppm: f64) -> i32 {
        let finest = self.steps.iter().copied().fold(f64::MAX, f64::min);
        (2.0 * BAND_K * noise_ppm / finest).ceil() as i32
    }

    fn clock_ppm(&self, code: i32) -> f64 {
        let at = |c: i32| self.steps[(c + TOTAL_MAX) as usize];
        if code >= 0 {
            self.offset_ppm - (0..code).map(at).sum::<f64>()
        } else {
            self.offset_ppm + (code..0).map(at).sum::<f64>()
        }
    }
}

/// The chip adapter between the loop's total and the trim code. `Slows` is
/// the contract (positive = slower); `Speeds` is the same-sign HSITRIM
/// mapping; `Late` applies each decision one train late.
#[derive(Clone, Copy)]
enum Adapter {
    Slows,
    Speeds,
    Late,
}

struct Servo {
    chip: Chip,
    clock: ClockDiscipline,
    adapter: Adapter,
    total: i32,
    prev: i32,
    ticks: u32,
}

impl Servo {
    fn new(chip: Chip, adapter: Adapter) -> Self {
        Self {
            chip,
            clock: ClockDiscipline::new(STEP_NOMINAL_PPM),
            adapter,
            total: 0,
            prev: 0,
            ticks: u32::MAX - CAL_PERIOD_TICKS / 3,
        }
    }

    fn code(&self) -> i32 {
        match self.adapter {
            Adapter::Slows => self.total,
            Adapter::Speeds => -self.total,
            Adapter::Late => self.prev,
        }
    }

    fn err_ppm(&self) -> f64 {
        self.chip.clock_ppm(self.code())
    }

    /// One CAL train paced by the host crystal: every break stamp carries
    /// the break's low time at the host's baud and a Gaussian service
    /// latency sized so the train reads with sd `noise_ppm`. Returns the
    /// step-effect sample the loop accepted, if any, and the true mean
    /// step of the codes the previous decision crossed.
    fn train(&mut self, rng: &mut Rng, noise_ppm: f64, baud: f64) -> Option<(i32, f64)> {
        let estimate = self.clock.trim.step_ppm();
        let (from, to) = (self.prev, self.total);
        let rate = TICKS_PER_US as f64 * (1.0 + self.err_ppm() / 1e6);
        let break_us = BREAK_BITS * 1e6 / baud;
        let latency_sd_us = noise_ppm * 1e-6 * SPAN_US / core::f64::consts::SQRT_2;
        self.clock.pending_cal = Some((GAP_US, GAPS));
        for k in 0..=GAPS {
            let t_us = k as f64 * GAP_US as f64 + break_us + latency_sd_us * rng.gauss();
            let now = self.ticks.wrapping_add((t_us * rate).round() as i64 as u32);
            self.clock.on_cal_break(now, TICKS_PER_US);
        }
        self.ticks = self.ticks.wrapping_add(CAL_PERIOD_TICKS);
        self.prev = self.total;
        if let Some(total) = self.clock.poll() {
            self.total = total as i32;
        }
        let accepted = self.clock.trim.step_ppm();
        (accepted != estimate && from != to).then(|| {
            let true_step =
                (self.chip.clock_ppm(from) - self.chip.clock_ppm(to)) / (to - from) as f64;
            (accepted, true_step)
        })
    }
}

#[derive(Default, Debug)]
struct Tally {
    judged: usize,
    out_of_band: usize,
    railed: usize,
    swings: usize,
    collapsed: usize,
    worst_ppm: f64,
}

impl Tally {
    fn judge(
        &mut self,
        s: &Servo,
        sample: Option<(i32, f64)>,
        window: &mut VecDeque<i32>,
        noise_ppm: f64,
    ) {
        let code = s.code();
        window.push_back(code);
        if window.len() > SWING_WINDOW {
            window.pop_front();
        }
        let (lo, hi) = window
            .iter()
            .fold((i32::MAX, i32::MIN), |(lo, hi), &c| (lo.min(c), hi.max(c)));
        let err = s.err_ppm().abs();
        self.judged += 1;
        self.out_of_band += (err > s.chip.band_ppm(noise_ppm)) as usize;
        self.railed += (s.total.abs() >= TOTAL_MAX) as usize;
        self.swings += (hi - lo > s.chip.swing_max(noise_ppm)) as usize;
        self.worst_ppm = self.worst_ppm.max(err);
        if let Some((accepted, true_step)) = sample {
            self.collapsed += ((accepted as f64) < true_step / 2.0) as usize;
        }
    }

    /// The ideal trim's ceilings: a train leaves the band, and a window
    /// swings wider than `swing_max`, only on a noise draw past BAND_K sd.
    fn holds(&self) -> bool {
        let n = self.judged as f64;
        let window_tail = 1.0 - (1.0 - NOISE_TAIL_P).powi(SWING_WINDOW as i32);
        self.railed == 0
            && self.out_of_band as f64 <= NOISE_TAIL_P * n
            && self.swings as f64 <= window_tail * n
    }
}

/// `EPISODES` chips settled at the boot baud, then judged for `JUDGED_TRAINS` at `baud`.
fn soak(kind: Steps, noise_ppm: f64, adapter: Adapter, baud: f64) -> Tally {
    let mut rng = Rng(SEED);
    let mut t = Tally::default();
    for _ in 0..EPISODES {
        let chip = Chip::new(kind, BOOT_REACH_STEPS, &mut rng);
        let mut s = Servo::new(chip, adapter);
        let mut window = VecDeque::new();
        for _ in 0..SETTLE_TRAINS {
            s.train(&mut rng, noise_ppm, BOOT_BAUD);
        }
        for _ in 0..JUDGED_TRAINS {
            let sample = s.train(&mut rng, noise_ppm, baud);
            t.judge(&s, sample, &mut window, noise_ppm);
        }
    }
    t
}

#[test]
fn cal_trim_holds_the_true_clock_at_600_ppm_ruler_noise() {
    for kind in CHIPS {
        let t = soak(kind, GATE_NOISE_PPM, Adapter::Slows, BOOT_BAUD);
        assert!(t.holds(), "{kind:?}: {t:?}");
    }
}

/// Settled chips jumped by +/-4 and +/-8 nominal steps: episodes that miss
/// the band for every one of ceil(|j| / STEPS_MAX) + 1 trains (the ideal
/// trim's count, plus one to re-identify the step), and the ideal's ceiling
/// on them, a noise draw past BAND_K sd on any of those trains.
fn recovery(noise_ppm: f64) -> (usize, f64) {
    let mut rng = Rng(SEED);
    let (mut misses, mut ceiling) = (0, 0.0);
    for kind in CHIPS {
        for jump in [4i32, -4, 8, -8] {
            let limit = jump.unsigned_abs().div_ceil(STEPS_MAX as u32) + 1;
            for _ in 0..EPISODES {
                let chip = Chip::new(kind, JUMP_REACH_STEPS, &mut rng);
                let mut s = Servo::new(chip, Adapter::Slows);
                for _ in 0..SETTLE_TRAINS {
                    s.train(&mut rng, noise_ppm, BOOT_BAUD);
                }
                s.chip.offset_ppm += jump as f64 * s.chip.nominal();
                let band = s.chip.band_ppm(noise_ppm);
                let back = (0..limit).any(|_| {
                    s.train(&mut rng, noise_ppm, BOOT_BAUD);
                    s.err_ppm().abs() <= band
                });
                misses += !back as usize;
                ceiling += 1.0 - (1.0 - NOISE_TAIL_P).powi(limit as i32);
            }
        }
    }
    (misses, ceiling)
}

/// An estimator that accepts a step-effect sample only within BAND_K sd
/// of the true step takes one outside that window, collapsed below half
/// the step among them, only on a draw past BAND_K sd: at most
/// NOISE_TAIL_P of trains, one sample per train. Seeds 1-20: today's loop
/// peaks at 15 per 10k (3000 nonuniform), a 200 ppm acceptance floor
/// takes 47-83 on the 1400 chip and fails all 20.
///
/// Finding: today's loop accepts collapsed samples on 3000 chips at up to
/// 15 per 10k at 600 ppm, and its worst true error reaches 8.2-9.8k ppm
/// (about 3 steps) against ~3.5k for the ideal loop.
#[test]
fn cal_trim_rejects_collapsed_step_estimates_at_600_ppm_ruler_noise() {
    for kind in CHIPS {
        let t = soak(kind, GATE_NOISE_PPM, Adapter::Slows, BOOT_BAUD);
        assert!(
            t.collapsed as f64 <= NOISE_TAIL_P * t.judged as f64,
            "{kind:?}: {t:?}"
        );
    }
}

#[test]
fn cal_trim_recovers_from_four_and_eight_step_jumps() {
    let (misses, ceiling) = recovery(GATE_NOISE_PPM);
    assert!(
        misses as f64 <= ceiling,
        "{misses} misses, ceiling {ceiling:.1}"
    );
}

#[test]
fn host_detune_never_moves_a_noiseless_trim() {
    for kind in CHIPS {
        let mut rng = Rng(SEED);
        for _ in 0..EPISODES {
            let chip = Chip::new(kind, BOOT_REACH_STEPS, &mut rng);
            let mut s = Servo::new(chip, Adapter::Slows);
            for _ in 0..SETTLE_TRAINS {
                s.train(&mut rng, 0.0, BOOT_BAUD);
            }
            let anchor = s.total;
            for _ in 0..JUDGED_TRAINS {
                s.train(&mut rng, 0.0, DETUNE_BAUD);
                assert_eq!(s.total, anchor, "{kind:?}");
            }
        }
    }
}

#[test]
fn host_detune_holds_the_true_clock_at_600_ppm_ruler_noise() {
    for kind in CHIPS {
        let t = soak(kind, GATE_NOISE_PPM, Adapter::Slows, DETUNE_BAUD);
        assert!(t.holds(), "{kind:?}: {t:?}");
    }
}

#[test]
fn judge_flags_a_same_sign_trim_adapter() {
    let t = soak(
        Steps::Uniform(COARSE_STEP_PPM),
        GATE_NOISE_PPM,
        Adapter::Speeds,
        BOOT_BAUD,
    );
    assert!(t.railed > 0 && !t.holds(), "{t:?}");
}

#[test]
fn judge_flags_a_trim_applied_one_train_late() {
    let t = soak(
        Steps::Uniform(COARSE_STEP_PPM),
        GATE_NOISE_PPM,
        Adapter::Late,
        BOOT_BAUD,
    );
    assert!(t.railed == 0 && !t.holds(), "{t:?}");
}

/// The ruler-noise ladder: `cargo test noise_ladder -- --ignored --nocapture`.
#[test]
#[ignore]
fn noise_ladder() {
    for kind in CHIPS {
        for noise in [260.0, 415.0, 600.0, 900.0] {
            let t = soak(kind, noise, Adapter::Slows, BOOT_BAUD);
            std::println!(
                "{kind:?} {noise}: holds {}, out of band {}, swings {}, collapsed {}, \
                 worst true err {:.0} ppm",
                t.holds(),
                t.out_of_band,
                t.swings,
                t.collapsed,
                t.worst_ppm
            );
        }
    }
    for noise in [260.0, 415.0, 600.0, 900.0] {
        let (misses, ceiling) = recovery(noise);
        std::println!("recovery {noise}: {misses} misses, ceiling {ceiling:.1}");
    }
}
