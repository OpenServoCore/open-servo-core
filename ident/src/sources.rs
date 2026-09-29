//! Where every plant input gain synthesis uses came from. The winding's R
//! and L come from the E8 burst when one of its routes promotes
//! ([`InductanceResult::route`]); otherwise R falls back to the E2 end-stop
//! stall and L to the configured default, and without a stall to the
//! winding the servo carries from an earlier identification ([`stored`]).
//! The rest have one source each.
//!
//! The winding carries two R. `r_vpc` is duty x rail over current at the
//! current limit: every stall-safe duty plans from it and `r_q12` carries
//! it, so the firmware's stall cap and back-EMF estimate read the same
//! number. `r_loop_vpc` is the slope of the V-I line with the bridge: the
//! current loop's plant, synthesized into `i_ki`. Only the free-shaft
//! burst tells them apart; every other source has one R for both.

use crate::exp::inductance::{BurstRoute, InductanceResult};
use crate::exp::resistance::ResistanceResult;
use crate::exp::rl::Scales;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Source {
    /// E8 held at a stop, promoted.
    BurstHeld,
    /// E8 on the free shaft from rest, promoted.
    Burst,
    /// E2, the end-stop stall, run because E8 declined.
    StallFallback,
    /// Read off the servo: an earlier identification measured it.
    Stored,
    /// The configured default: nothing measured it.
    Default,
    /// E3, the steady-state ladder.
    Ladder,
    /// E4, the duty-step transients.
    Inertia,
    /// E0, the torque-off noise floor.
    Bias,
}

impl Source {
    pub fn as_str(self) -> &'static str {
        match self {
            Source::BurstHeld => "burst, held",
            Source::Burst => "burst, free",
            Source::StallFallback => "resistance fallback",
            Source::Stored => "stored on the servo",
            Source::Default => "default",
            Source::Ladder => "ladder",
            Source::Inertia => "inertia",
            Source::Bias => "bias",
        }
    }
}

/// The winding's R and L as gain synthesis and the plan take them.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Winding {
    /// `r_vpc` in ohms; None when R came from E2, which fits vcounts per
    /// ccount only.
    pub r_ohm: Option<f64>,
    /// What the stall-safe duties and `r_q12` take, vcounts per ccount.
    pub r_vpc: f64,
    pub r_from: Source,
    /// The current loop's plant R, vcounts per ccount.
    pub r_loop_vpc: f64,
    pub l_h: f64,
    pub l_from: Source,
}

/// True when the stall has to run: E8 was not run, or both its routes
/// declined.
pub fn needs_stall(e8: Option<&InductanceResult>) -> bool {
    !e8.is_some_and(|r| r.promotable())
}

/// E8's winding from the route that promotes, else E2's R with
/// `l_default_h`. None when E8 declined and no stall ran. `sc` converts
/// E8's ohms to the table's units; a recording too old to carry the scales
/// has no E8 to promote.
pub fn winding(
    e8: Option<&InductanceResult>,
    e2: Option<&ResistanceResult>,
    sc: Option<&Scales>,
    l_default_h: f64,
) -> Option<Winding> {
    let from = |r: BurstRoute| match r {
        BurstRoute::Held => Source::BurstHeld,
        BurstRoute::Free => Source::Burst,
    };
    match e8.and_then(|x| x.route().zip(x.winding_terms())).zip(sc) {
        Some(((route, t), sc)) => Some(Winding {
            r_ohm: Some(t.r_plan_ohm),
            r_vpc: sc.r_vpc(t.r_plan_ohm),
            r_from: from(route),
            r_loop_vpc: sc.r_vpc(t.r_loop_ohm),
            l_h: t.l_h,
            l_from: from(route),
        }),
        None => e2.map(|x| Winding {
            r_ohm: None,
            r_vpc: x.r_vpc,
            r_from: Source::StallFallback,
            r_loop_vpc: x.r_vpc,
            l_h: l_default_h,
            l_from: Source::Default,
        }),
    }
}

/// The winding the servo carries. `r_q12` is what the plan takes, vcounts
/// per ccount in Q12. The current loop's R and L have no fields of their
/// own: they ride in the current PI, whose kp is w_ci L and whose ki is
/// w_ci R per fast tick, so `f_ci`, the crossover the gains were
/// synthesized at, reads both back. Resynthesized at another crossover the
/// PI keeps its zero and moves by the ratio. None unless all four fields
/// and the crossover are set.
pub fn stored(
    r_q12: u16,
    i_kp_q88: u16,
    i_ki_q412: u16,
    tick_hz: u16,
    f_ci: f64,
    sc: &Scales,
) -> Option<Winding> {
    if r_q12 == 0 || i_kp_q88 == 0 || i_ki_q412 == 0 || tick_hz == 0 || f_ci <= 0.0 {
        return None;
    }
    let r_vpc = r_q12 as f64 / 4096.0;
    let w_ci = core::f64::consts::TAU * f_ci;
    let l_cd = i_kp_q88 as f64 / 256.0 / w_ci;
    Some(Winding {
        r_ohm: Some(r_vpc * sc.v_term_per_count / sc.amps_per_count),
        r_vpc,
        r_from: Source::Stored,
        r_loop_vpc: i_ki_q412 as f64 / 4096.0 * tick_hz as f64 / w_ci,
        l_h: l_cd * sc.v_term_per_count / sc.amps_per_count,
        l_from: Source::Stored,
    })
}

/// How far, as a fraction of the stored R, the burst's rough R may sit
/// before the stored winding is called into question.
pub const STALE_R: f64 = 0.25;

/// The burst's rough R, ohms - its V/I at the current limit, or its pairs
/// estimate without one - when it is more than [`STALE_R`] away from the
/// stored winding's R.
pub fn stale(e8: Option<&InductanceResult>, stored: &Winding) -> Option<f64> {
    let rough = e8.and_then(|x| {
        x.wave
            .as_ref()
            .ok()
            .and_then(|w| w.at_limit)
            .map(|a| a.v_over_i_ohm)
            .or(x.volts.r_pair_ohm)
            .or(x.r_pair_ohm)
    })?;
    let r = stored.r_ohm?;
    ((rough - r).abs() > STALE_R * r).then_some(rough)
}

/// What a servo with nothing to fall back on is told when the burst
/// declined, `reason` in a few plain words.
pub fn unmeasured(reason: &str) -> String {
    format!(
        "this motor's winding could not be measured ({reason}). The servo is using the safe \
         value for micro servos: it is protected, it moves more gently, and its speed reading is \
         less exact. Charge the battery, make sure the horn turns freely, and run identification \
         again."
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::burst::{Capture, Chans};
    use crate::exp::Guarded;
    use crate::exp::inductance::{Cfg as InductanceCfg, FitCfg, Inductance};
    use crate::exp::resistance::{Resistance, ResistanceCfg};
    use crate::exp::testkit::{FakeServo, SynthBurst, pump};
    use crate::gains::DEFAULT_L_HENRIES;
    use crate::units::SenseParams;

    fn scales() -> Scales {
        let s = SenseParams {
            shunt_r_mohm: 60,
            gain_milli: 15_000,
            vmotor_div_top: 6_800,
            vmotor_div_bot: 3_300,
            vdd_mv: 3_300,
            tick_hz: 20_100,
        };
        Scales::from_sense(&s, 15_000, 10_000).unwrap()
    }

    /// The front of `osc ident run`: E8, then E2 only if E8 declined, then
    /// the winding terms gain synthesis takes.
    fn front(
        servo: &mut FakeServo,
        chans: Chans,
    ) -> (InductanceResult, Option<ResistanceResult>, Winding) {
        let params = crate::exp::testkit::rig();
        let cfg = InductanceCfg {
            repeats: 1,
            i_max_a: 1.0,
            chans,
            fit: FitCfg::default().with_limit(I_LIM_A),
            ..InductanceCfg::default()
        };
        let mut e8 = Guarded::new(Inductance::new(cfg, &params, scales()), params);
        pump(&mut e8, servo, 200_000);
        assert!(e8.abort().is_none());
        let e8 = e8.into_inner().fit().expect("E8 fits");
        let e2 = needs_stall(Some(&e8)).then(|| {
            let p = params.without_pos_guard();
            let mut e = Guarded::new(Resistance::new(ResistanceCfg::default(), &p), p);
            pump(&mut e, servo, 200_000);
            assert!(e.abort().is_none());
            e.into_inner().fit().expect("E2 fits")
        });
        let sc = scales();
        let w = winding(Some(&e8), e2.as_ref(), Some(&sc), DEFAULT_L_HENRIES).expect("winding");
        (e8, e2, w)
    }

    /// The bench servo's 280-count limit, amps.
    const I_LIM_A: f64 = 280.0 * 3.3 / 4096.0 / (15.0 * 0.060);

    /// USB behind 1.3 ohm and 100 uF, the supply the pre-arm rail misreads.
    fn usb_plant() -> SynthBurst {
        SynthBurst {
            r: 4.0,
            l: 0.6e-3,
            v0: 0.2,
            settle_us: 2.0,
            v_rail: 4.37,
            r_src: 1.3,
            ..SynthBurst::board_d().with_bridge()
        }
    }

    /// The plant's own duty x rail over current, settled at `i`: the bulk
    /// capacitor averages the supply's current, D i, through its source.
    fn plant_v_over_i(p: &SynthBurst, i: f64) -> f64 {
        let mut d = 0.0;
        for _ in 0..50 {
            let vc = p.v_rail - p.r_src * d * i;
            d = (p.v0 + (p.r + 2.0 * p.rds) * i) / (vc - p.r_shunt * i);
        }
        d * p.v_rail / i
    }

    /// The burst reads the line and hands the plan V/I at the limit, the
    /// current loop the slope with the bridge, both with L.
    #[test]
    fn a_promoted_burst_supplies_r_and_l_and_the_stall_never_runs() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        servo.burst = usb_plant();
        let (e8, e2, w) = front(&mut servo, Chans::Driven);
        assert!(e8.promotable(), "{:?}", e8.blocking());
        assert!(e2.is_none(), "E2 ran behind a promoted E8");
        assert_eq!((w.r_from, w.l_from), (Source::Burst, Source::Burst));
        let plant = usb_plant();
        let r = w.r_ohm.unwrap();
        let want = plant_v_over_i(&plant, I_LIM_A);
        assert!((r - want).abs() / want < 0.03, "V/I {r} of {want}");
        assert!((w.l_h - 0.6e-3).abs() / 0.6e-3 < 0.05, "L {}", w.l_h);
        assert!((w.r_vpc - scales().r_vpc(r)).abs() < 1e-12);
        let wave = e8.wave.as_ref().expect("the waveform fit");
        assert!(
            (wave.fit.r_ohm - plant.r).abs() / plant.r < 0.05,
            "{:?}",
            wave.fit
        );
        let at = wave.at_limit.expect("read at the limit");
        assert_eq!(at.v_over_i_ohm, r);
        assert!((w.r_loop_vpc - scales().r_vpc(at.slope_ohm)).abs() < 1e-12);
        assert!(
            w.r_loop_vpc < w.r_vpc,
            "the slope sits under V/I at the limit"
        );
    }

    #[test]
    fn a_declined_burst_hands_r_to_the_stall_and_l_to_the_default() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        servo.burst = usb_plant();
        // no voltage channel: nothing measured the winding's voltage, so
        // there is no waveform to fit
        let (e8, e2, w) = front(&mut servo, Chans::Fixed(0));
        assert_eq!(e8.blocking(), vec!["waveform"]);
        assert_eq!(
            e8.reason(),
            "no capture from rest sampled the driven terminal"
        );
        let e2 = e2.expect("E2 runs behind a declined E8");
        assert_eq!(w.r_from, Source::StallFallback);
        assert_eq!(w.l_from, Source::Default);
        assert_eq!(w.r_vpc, e2.r_vpc);
        assert!((w.r_vpc - 3.37).abs() / 3.37 < 0.05, "E2 R {}", w.r_vpc);
        assert_eq!(w.l_h, DEFAULT_L_HENRIES);
        assert!(w.r_ohm.is_none());
    }

    /// The promotion order: held first, the free shaft when the held route
    /// declines, E2 only when both do.
    #[test]
    fn a_seated_burst_outranks_the_free_shaft() {
        use crate::exp::inductance::{BurstRoute, FitCfg, fit_captures};
        let plant = usb_plant();
        let at = |q: i16| SynthBurst {
            chans: Chans::Driven.for_step(q),
            ..plant.clone()
        };
        let mut free = Vec::new();
        for pct in [20i32, 30, 40] {
            for sgn in [1i32, -1] {
                let q = (sgn * pct * 32767 / 100) as i16;
                free.push(at(q).capture(q, 0));
            }
        }
        let hold = -(12 * 32767 / 100) as i16;
        let seated = |pcts: &[i32]| -> Vec<Capture> {
            let mut v = vec![at(hold).capture(0, 0)];
            for pct in pcts {
                for _ in 0..2 {
                    let q = -(pct * 32767 / 100) as i16;
                    let mut c = at(q).capture(q, hold);
                    c.meta.seated = true;
                    v.push(c);
                }
            }
            v
        };
        let sc = scales();
        let at = FitCfg::default().with_limit(I_LIM_A);
        let fit = |caps: Vec<Capture>| fit_captures(&caps, &sc, &at).unwrap();
        let w =
            |r: &InductanceResult| winding(Some(r), None, Some(&sc), DEFAULT_L_HENRIES).unwrap();

        let both = fit([free.clone(), seated(&[20, 30, 40])].concat());
        assert_eq!(both.route(), Some(BurstRoute::Held));
        assert_eq!(w(&both).r_from, Source::BurstHeld);
        assert!(!needs_stall(Some(&both)));

        // one step duty is too thin for the held route: the free shaft stands in
        let thin = fit([free.clone(), seated(&[30])].concat());
        assert_eq!(thin.held.blocking(), vec!["captures"]);
        assert_eq!(thin.route(), Some(BurstRoute::Free));
        assert_eq!(
            (w(&thin).r_from, w(&thin).l_from),
            (Source::Burst, Source::Burst)
        );

        let alone = fit(seated(&[30]));
        assert_eq!(alone.route(), None);
        assert!(needs_stall(Some(&alone)));
    }

    /// A winding identified once, synthesized and encoded, then read back
    /// off the table: the plan's R, the loop's R and L come back as they
    /// went in, and synthesized again at the same crossover they encode to
    /// the same fields. The burst's two R differ, V/I at the limit 5.33 ohm
    /// over a slope of 4.59: each comes back as itself.
    #[test]
    fn stored_winding_writes_back_unchanged() {
        use crate::gains::{self, BwTargets, PlantParams};
        let sc = scales();
        let (r_ohm, r_loop_ohm, l_h) = (5.33, 4.59, 0.79e-3);
        let l_cd = |l: f64| gains::l_cd_from_si(l, 60, 15_000, 6_800, 3_300).unwrap();
        let plant = |r_vpc: f64, r_loop_vpc: f64, l: f64| PlantParams {
            r_vpc,
            r_loop_vpc,
            ke_vpc: 0.15,
            fc: 20.0,
            fv: 0.001,
            b: 0.1,
            sigma_theta: 1.0,
            l_cd: l_cd(l),
            tick_hz: 20_100.0,
            f_med: 2_010.0,
        };
        let t = BwTargets::default();
        let first = gains::encode(&gains::synthesize(
            &plant(sc.r_vpc(r_ohm), sc.r_vpc(r_loop_ohm), l_h),
            &t,
        ));
        let read = |r_q12: u16, kp: u16, ki: u16| stored(r_q12, kp, ki, 20_100, t.f_ci, &sc);
        let w = read(first.r_q12.raw, first.i_kp_q88.raw, first.i_ki_q412.raw)
            .expect("a stored winding");
        assert_eq!((w.r_from, w.l_from), (Source::Stored, Source::Stored));
        assert!((w.r_ohm.unwrap() - r_ohm).abs() / r_ohm < 1e-3, "{w:?}");
        let loop_ohm = w.r_loop_vpc / sc.r_vpc(1.0);
        assert!((loop_ohm - r_loop_ohm).abs() / r_loop_ohm < 5e-3, "{w:?}");
        assert!((w.l_h - l_h).abs() / l_h < 5e-3, "{w:?}");
        let again = gains::encode(&gains::synthesize(&plant(w.r_vpc, w.r_loop_vpc, w.l_h), &t));
        assert_eq!(again.r_q12.raw, first.r_q12.raw);
        assert_eq!(again.i_ki_q412.raw, first.i_ki_q412.raw);
        assert_eq!(again.i_kp_q88.raw, first.i_kp_q88.raw);

        assert!(read(0, first.i_kp_q88.raw, first.i_ki_q412.raw).is_none());
        assert!(read(first.r_q12.raw, 0, first.i_ki_q412.raw).is_none());
        assert!(read(first.r_q12.raw, first.i_kp_q88.raw, 0).is_none());
    }

    /// The bench run promotes its line: the plan and `r_q12` take V/I at
    /// the limit, so the stop cap the plan allows draws exactly the limit
    /// on the measured line, where planning from the slope would stall a
    /// fifth under it. The current loop's ki takes the slope, kp the L.
    #[test]
    fn stall_safe_duties_plan_from_v_over_i_at_the_limit() {
        use crate::exp::inductance::{FitCfg, fit_captures};
        use crate::exp::testkit::{board_d_scales, mg90_2s};
        use crate::gains::{self, BwTargets, PlantParams};
        use crate::limits::{DutyPlan, ServoLimits};
        let sc = board_d_scales();
        let cfg = FitCfg::default().with_limit(I_LIM_A);
        let e8 = fit_captures(&mg90_2s(), &sc, &cfg).expect("the bench run fits");
        let w = winding(Some(&e8), None, Some(&sc), DEFAULT_L_HENRIES).expect("promoted");
        assert_eq!((w.r_from, w.l_from), (Source::Burst, Source::Burst));
        let wave = e8.wave.as_ref().expect("the waveform fit");
        let at = wave.at_limit.expect("read at the limit");
        assert_eq!(w.r_ohm, Some(at.v_over_i_ohm));
        assert!((w.r_vpc - sc.r_vpc(at.v_over_i_ohm)).abs() < 1e-12);
        assert!((w.r_loop_vpc - sc.r_vpc(at.slope_ohm)).abs() < 1e-12);
        assert_eq!(w.l_h, wave.fit.l_h);

        let lim = ServoLimits {
            i_lim: 280,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 0,
            vbus: (wave.rail_v / sc.v_term_per_count).round() as u16,
            window_floor_q15: 4356,
            amps_per_count: sc.amps_per_count,
        };
        let rail = lim.vbus as f64 * sc.v_term_per_count;
        let plan = DutyPlan::new(&lim, w.r_vpc, None);
        let i_at = |duty: f64| {
            let (mut lo, mut hi) = (0.0, 2.0);
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if wave.duty_volts(mid) < duty * rail {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            lo
        };
        let limit = I_LIM_A;
        assert!((i_at(plan.stop_cap) / limit - 1.0).abs() < 1e-3, "{plan:?}");
        let by_slope = DutyPlan::new(&lim, sc.r_vpc(wave.fit.r_ohm), None);
        let under = i_at(by_slope.stop_cap) / limit;
        assert!(
            (0.75..0.85).contains(&under),
            "the slope plans {under} of the limit"
        );

        let plant = PlantParams {
            r_vpc: w.r_vpc,
            r_loop_vpc: w.r_loop_vpc,
            ke_vpc: 0.15,
            fc: 20.0,
            fv: 0.001,
            b: 0.1,
            sigma_theta: 1.0,
            l_cd: sc.r_vpc(w.l_h),
            tick_hz: 20_100.0,
            f_med: 2_010.0,
        };
        let t = BwTargets::default();
        let g = gains::synthesize(&plant, &t);
        let w_ci = core::f64::consts::TAU * t.f_ci;
        assert_eq!(g.r_vpc, w.r_vpc);
        assert!((g.i_ki - w_ci * w.r_loop_vpc / 20_100.0).abs() < 1e-12);
        assert!((g.i_kp - w_ci * sc.r_vpc(w.l_h)).abs() < 1e-12);
    }

    /// A servo with nothing to fall back on hears why in a few plain words
    /// and what to do, in the third person.
    #[test]
    fn a_winding_that_cannot_be_measured_says_so_in_plain_words() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        servo.burst = usb_plant();
        let params = crate::exp::testkit::rig();
        let cfg = InductanceCfg {
            repeats: 1,
            i_max_a: 1.0,
            chans: Chans::Fixed(0),
            fit: FitCfg::default().with_limit(I_LIM_A),
            ..InductanceCfg::default()
        };
        let mut e8 = Guarded::new(Inductance::new(cfg, &params, scales()), params);
        pump(&mut e8, &mut servo, 200_000);
        let e8 = e8.into_inner().fit().expect("E8 fits");
        assert!(!e8.promotable());
        let say = unmeasured(&e8.reason());
        assert_eq!(
            say,
            "this motor's winding could not be measured (no capture from rest sampled the \
             driven terminal). The servo is using the safe value for micro servos: it is \
             protected, it moves more gently, and its speed reading is less exact. Charge the \
             battery, make sure the horn turns freely, and run identification again."
        );
        let words: Vec<&str> = say
            .split(|c: char| !c.is_ascii_alphabetic() && c != '\'')
            .collect();
        for w in ["I", "we", "We", "our", "us"] {
            assert!(!words.contains(&w), "{w} in {say}");
        }
        assert!(say.is_ascii());
    }

    #[test]
    fn no_burst_and_no_stall_is_no_winding() {
        assert!(needs_stall(None));
        assert!(winding(None, None, Some(&scales()), DEFAULT_L_HENRIES).is_none());
    }
}
