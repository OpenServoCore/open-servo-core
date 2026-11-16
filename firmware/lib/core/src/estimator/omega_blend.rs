//! Velocity-loop feedback: the pot observer's speed in the mean, the
//! back-EMF boxcar for what changes fast. The boxcar's level is not
//! trusted - the brush drop wanders, and under a driven loop it reads
//! over the pot - so the loop takes only what the boxcar says beyond its
//! own standing difference from the observer:
//!
//! ```text
//! g = boxcar - observer, limited          a period with a boxcar
//! d += (g - d) >> D_SHIFT                 the standing part, those periods
//! c += ((boxcar ? g - d : 0) - c) >> 2    the lead
//! omega_hat = observer + c
//! ```
//!
//! The standing part holds across the gaps between boxcars: under a
//! closed loop a run of boxcars lasts a few periods, and a part cleared at
//! every gap never forms. The lead keeps a boxcar that comes or goes from
//! stepping the loop's speed by the whole fast part at once. With no
//! boxcar the lead runs out and the loop reads the observer alone.

use crate::math::{sat_add, sat_clamp, sat_sub};

/// The standing part's corner, 2^D_SHIFT periods with a boxcar.
pub const D_SHIFT: u32 = 7;
const _: () = assert!(super::bemf::BRUSH_AVG_SHIFT == D_SHIFT - 1);
/// The lead's corner, 2^C_SHIFT periods.
const C_SHIFT: u32 = 2;

/// The limit on the boxcar's difference from the observer, c/s: `g` and
/// the standing part stay under 2^30 csQ16, so their difference is exact.
const G_LIM_CPS: i32 = 16383;

/// States, csQ16.
#[derive(Default)]
pub struct OmegaBlend {
    d_q16: i32,
    c_q16: i32,
}

impl OmegaBlend {
    pub const fn new() -> Self {
        Self { d_q16: 0, c_q16: 0 }
    }

    /// Zero state; the kernel calls it where the velocity loop's
    /// integrator clears.
    #[inline(always)]
    pub fn reset(&mut self) {
        self.d_q16 = 0;
        self.c_q16 = 0;
    }

    /// One MEDIUM-tick update -> omega_hat csQ16. `bemf_cps` is the boxcar
    /// result (None = no boxcar this period), `flipped` whether its close
    /// saw the drive's sign change, `obs_q16` the observer's omega.
    #[inline(always)]
    pub fn step(&mut self, bemf_cps: Option<i32>, flipped: bool, obs_q16: i32) -> i32 {
        if flipped {
            // every volts term of the boxcar lies along the drive, so its
            // standing error turns with it
            self.d_q16 = -self.d_q16;
        }
        let fast = match bemf_cps {
            Some(w) => {
                let g = sat_clamp(sat_sub(w, obs_q16 >> 16), -G_LIM_CPS, G_LIM_CPS) << 16;
                self.d_q16 += (g - self.d_q16) >> D_SHIFT;
                g - self.d_q16
            }
            None => 0,
        };
        // the fast part and the lead each fit i32, their difference does
        // not
        let e = sat_sub(fast, self.c_q16);
        // rounded away from zero: the lead lands on its input, so with no
        // boxcar it runs out to 0 from either side
        let up = (e > 0) as i32;
        self.c_q16 += ((e - up) >> C_SHIFT) + up;
        sat_add(obs_q16, self.c_q16)
    }

    /// The boxcar's standing difference from the observer, csQ16.
    pub fn standing_q16(&self) -> i32 {
        self.d_q16
    }

    /// What the loop's speed carries over the observer's, csQ16.
    pub fn lead_q16(&self) -> i32 {
        self.c_q16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const G_LIM_Q16: i64 = (G_LIM_CPS as i64) << 16;
    /// Periods for the standing part to form on boxcars half the time:
    /// eight of its corners.
    const FORM: u32 = 16 << D_SHIFT;
    /// Periods without a boxcar that run any lead out.
    const RUN_OUT: u32 = 73;

    const fn q16(cps: i32) -> i32 {
        cps << 16
    }

    fn lcg(seed: u32) -> impl FnMut() -> u32 {
        let mut s = seed;
        move || {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            s >> 16
        }
    }

    /// The form in i64: nothing in it can wrap, and it saturates only
    /// where the i32 form does.
    #[derive(Default)]
    struct Wide {
        d: i64,
        c: i64,
        /// Steps whose lead input less the lead left i32.
        saturated: u32,
    }

    impl Wide {
        fn step(&mut self, bemf_cps: Option<i32>, flipped: bool, obs_q16: i32) -> i32 {
            const I32: (i64, i64) = (i32::MIN as i64, i32::MAX as i64);
            if flipped {
                self.d = -self.d;
            }
            let fast = match bemf_cps {
                Some(w) => {
                    let gap = w as i64 - (obs_q16 >> 16) as i64;
                    let g = gap.clamp(-(G_LIM_CPS as i64), G_LIM_CPS as i64) << 16;
                    self.d += (g - self.d) >> D_SHIFT;
                    g - self.d
                }
                None => 0,
            };
            let e = (fast - self.c).clamp(I32.0, I32.1);
            self.saturated += (e != fast - self.c) as u32;
            let round = (1i64 << C_SHIFT) - 1;
            self.c += if e > 0 {
                (e + round) >> C_SHIFT
            } else {
                e >> C_SHIFT
            };
            (obs_q16 as i64 + self.c).clamp(I32.0, I32.1) as i32
        }
    }

    /// The loop's mean speed is the observer's whatever the boxcar's
    /// level. A boxcar that reads 400 c/s off the shaft on two periods in
    /// four, over an observer that dithers 64 c/s about it: once the
    /// standing part has formed the loop's speed is within 8 c/s of the
    /// observer's in the mean, where the boxcar's level handed over on
    /// its periods would leave 200. The fast part still reaches the loop:
    /// behind a clear the lead carries more than half the gap.
    #[test]
    fn a_steady_cruise_reads_the_observer_in_the_mean() {
        const JUDGE: u32 = 400;
        let mut worst = 0;
        for (shaft, over) in [
            (1500, 400),
            (-1500, -400),
            (1500, -400),
            (-1500, 400),
            (300, 4000),
            (-9000, -2500),
        ] {
            let mut blend = OmegaBlend::new();
            let mut next = lcg(0x1234_5678 ^ shaft as u32);
            let (mut sum, mut first) = (0i64, 0i32);
            for period in 0..FORM + JUDGE {
                let dither = (next() as i32 % (129 << 8)) - (64 << 8);
                let obs = q16(shaft) + (dither << 8);
                let boxcar = (period % 4 < 2).then_some(shaft + over);
                let lead = blend.step(boxcar, false, obs) - obs;
                if period < 16 {
                    first = first.max(lead * over.signum());
                }
                if period >= FORM {
                    sum += lead as i64;
                }
            }
            let mean = sum / JUDGE as i64;
            assert!(
                mean.abs() <= 8 << 16,
                "shaft {shaft}, {over} over: the loop reads {} c/s off the observer",
                mean >> 16
            );
            worst = worst.max(mean.abs());
            assert!(
                first > q16(over.abs()) / 2,
                "shaft {shaft}, {over} over: the lead took {} c/s of the gap",
                first >> 16
            );
            let standing = blend.standing_q16() >> 16;
            assert!((standing - over).abs() <= 64, "standing part {standing}");
        }
        assert!(worst > 0, "a mean of exactly the observer's pins nothing");
    }

    /// A boxcar that comes or goes never steps the loop's speed: on any
    /// pattern of periods with and without one, behind a clear, the
    /// loop's speed moves off the observer's by at most a quarter of the
    /// gap a period, where the boxcar handed over at once steps by the
    /// whole gap. The first boxcar behind a clear takes that quarter.
    #[test]
    fn a_validity_change_never_steps_the_loops_speed() {
        const OBS: i32 = q16(1500);
        for over in [540, -540, 16383, -16383, 3, -3] {
            let quarter = (q16(over).abs() + 3) / 4;
            let mut blend = OmegaBlend::new();
            let mut next = lcg(0x9e37_79b9 ^ over as u32);
            let (mut was, mut largest, mut changes) = (0, 0, 0u32);
            let mut fed = false;
            for period in 0..4000 {
                let valid = next() % 3 != 0;
                changes += (valid != fed) as u32;
                fed = valid;
                let lead = blend.step(valid.then_some(1500 + over), false, OBS) - OBS;
                let step = (lead - was).abs();
                assert!(
                    step <= quarter,
                    "{over} over, period {period}: a step of {step}"
                );
                largest = largest.max(step);
                was = lead;
            }
            assert!(changes > 1000, "{changes} changes");
            assert!(
                largest >= quarter - (quarter >> D_SHIFT) - 1,
                "{over} over: the largest step is {largest} of {quarter}"
            );
        }
    }

    /// Without a boxcar the loop reads the observer: bit for bit on a
    /// fresh blend, and behind a run of boxcars once the lead has run
    /// out. Each period without one leaves at most three quarters of the
    /// lead, (3/4)^16 of it after sixteen, 1.0 %, and nothing after
    /// `RUN_OUT`, from either sign and at any observer speed. The
    /// standing part holds through the gap.
    #[test]
    fn without_a_boxcar_the_loop_reads_the_observer() {
        const OBS: [i32; 7] = [
            0,
            q16(1500),
            q16(-1500),
            q16(32767),
            q16(-32767),
            i32::MAX,
            i32::MIN,
        ];
        let hat = |obs: i32, lead: i32| {
            (obs as i64 + lead as i64).clamp(i32::MIN as i64, i32::MAX as i64) as i32
        };
        for over in [540, -540, 16383, -16383, 30000, -30000, 1, -1] {
            let mut blend = OmegaBlend::new();
            for obs in OBS {
                assert_eq!(blend.step(None, false, obs), obs, "fresh, {obs}");
            }
            for _ in 0..8 {
                blend.step(Some(over), false, 0);
            }
            let (standing, last) = (blend.standing_q16(), blend.lead_q16());
            let gap = q16(over.clamp(-G_LIM_CPS, G_LIM_CPS));
            assert!(
                last.signum() == over.signum() && last.abs() > gap.abs() / 2,
                "{over} over: a lead of {last}"
            );
            let mut lead = last;
            for n in 1..=RUN_OUT {
                let obs = OBS[n as usize % OBS.len()];
                let got = blend.step(None, false, obs);
                let now = blend.lead_q16();
                assert_eq!(got, hat(obs, now), "period {n}");
                assert_eq!(
                    now.abs(),
                    lead.abs() - (lead.abs() + 3) / 4,
                    "{over} over, period {n}: {lead} to {now}"
                );
                assert!(now == 0 || now.signum() == last.signum());
                lead = now;
                if n == 16 {
                    let bound = (last.abs() as i64 * 3i64.pow(16)) >> 32;
                    assert!(lead.abs() as i64 <= bound, "{over} over: {lead} at 16");
                }
            }
            assert_eq!(lead, 0, "{over} over: the lead ran out");
            for n in 0..100 {
                let obs = OBS[n % OBS.len()];
                assert_eq!(blend.step(None, false, obs), obs, "{over} over, {obs}");
            }
            assert_eq!(blend.standing_q16(), standing, "{over} over");
        }
    }

    /// The standing part turns with the drive. Every volts term of the
    /// boxcar lies along the drive, so a boxcar that reads 500 c/s over
    /// the shaft one way reads 500 c/s under it the other way. At the
    /// flip the standing part changes sign, exactly and once, and the
    /// boxcars of the drive the other way leave the loop on the observer;
    /// a standing part that kept its sign hands the loop more than the
    /// whole gap.
    #[test]
    fn the_standing_part_turns_with_the_drive() {
        for dir in [1, -1] {
            let (mut blend, mut kept) = (OmegaBlend::new(), OmegaBlend::new());
            for _ in 0..FORM {
                for b in [&mut blend, &mut kept] {
                    b.step(Some(dir * 2000), false, q16(dir * 1500));
                }
            }
            let formed = blend.standing_q16();
            assert!((formed - q16(dir * 500)).abs() < q16(1), "formed {formed}");
            assert!(blend.lead_q16().abs() < q16(1));

            // the flipped half is void, the two behind it start up
            blend.step(None, true, q16(dir * 700));
            kept.step(None, false, q16(dir * 700));
            assert_eq!(blend.standing_q16(), -formed);
            assert_eq!(kept.standing_q16(), formed);
            for obs in [0, -dir * 700] {
                for b in [&mut blend, &mut kept] {
                    b.step(None, false, q16(obs));
                }
            }
            assert_eq!(blend.standing_q16(), -formed, "one flip, one turn");

            let (mut turned, mut unturned) = (0, 0);
            for _ in 0..16 {
                let obs = q16(-dir * 1500);
                turned = turned.max((blend.step(Some(-dir * 2000), false, obs) - obs).abs());
                unturned = unturned.max((kept.step(Some(-dir * 2000), false, obs) - obs).abs());
            }
            assert!(turned < q16(1), "drive {dir}: {} c/s off", turned >> 16);
            assert!(unturned > q16(500), "drive {dir}: {} c/s", unturned >> 16);
        }
    }

    /// Nothing wraps, for any boxcar, any observer speed and any run of
    /// flips: the i32 form is the i64 form bit for bit, the standing part
    /// stays inside the gap's limit and the lead inside twice it. The runs
    /// are long enough to rail both states, and a flip against a railed
    /// lead takes the lead's input less the lead past i32, where the form
    /// saturates.
    #[test]
    fn the_extremes_never_wrap() {
        const BOXCARS: [Option<i32>; 12] = [
            None,
            Some(i32::MIN),
            Some(i32::MAX),
            Some(-32768),
            Some(32767),
            Some(-16383),
            Some(16383),
            Some(0),
            Some(-1),
            Some(1),
            None,
            None,
        ];
        const OBS: [i32; 8] = [
            i32::MIN,
            i32::MAX,
            q16(-32767),
            q16(32767),
            0,
            -1,
            q16(-9000),
            q16(9000),
        ];
        let (mut blend, mut wide) = (OmegaBlend::new(), Wide::default());
        let mut next = lcg(0xdead_beef);
        let (mut d_max, mut c_max, mut hats) = (0i64, 0i64, [false; 2]);
        for _ in 0..4000 {
            let boxcar = BOXCARS[next() as usize % BOXCARS.len()];
            let obs = OBS[next() as usize % OBS.len()];
            let flipped = next() % 4 == 0;
            for n in 0..1 + next() % 300 {
                let flip = flipped && n == 0;
                let got = blend.step(boxcar, flip, obs);
                assert_eq!(got, wide.step(boxcar, flip, obs), "{boxcar:?} {obs} {flip}");
                assert_eq!(blend.standing_q16() as i64, wide.d);
                assert_eq!(blend.lead_q16() as i64, wide.c);
                d_max = d_max.max(wide.d.abs());
                c_max = c_max.max(wide.c.abs());
                hats[0] |= got == i32::MIN;
                hats[1] |= got == i32::MAX;
            }
        }
        assert!(d_max <= G_LIM_Q16 && d_max > G_LIM_Q16 - (G_LIM_Q16 >> 8));
        assert!(c_max <= 2 * G_LIM_Q16 && c_max > G_LIM_Q16);
        assert!(wide.saturated > 0, "no step saturated");
        assert_eq!(hats, [true; 2], "the loop's speed railed both ways");
    }
}
