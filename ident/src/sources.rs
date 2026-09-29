//! Where every plant input gain synthesis uses came from. The winding's R
//! and L come from the E8 burst when one of its routes promotes
//! ([`InductanceResult::route`]); otherwise R falls back to the E2 end-stop
//! stall and L to the configured default, and without a stall to the
//! winding the servo carries from an earlier identification ([`stored`]).
//! The rest have one source each.

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

/// The winding's R and L as gain synthesis takes them.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Winding {
    /// Ohms; None when R came from E2, which fits vcounts per ccount only.
    pub r_ohm: Option<f64>,
    pub r_vpc: f64,
    pub r_from: Source,
    pub l_h: f64,
    pub l_from: Source,
}

/// True when the stall has to run: E8 was not run, or both its routes
/// declined.
pub fn needs_stall(e8: Option<&InductanceResult>) -> bool {
    !e8.is_some_and(|r| r.promotable())
}

/// E8's R and L from the route that promotes, else E2's R with
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
    match e8.and_then(|x| x.route().zip(x.gain_r_l())).zip(sc) {
        Some(((route, (r, l)), sc)) => Some(Winding {
            r_ohm: Some(r),
            r_vpc: sc.r_vpc(r),
            r_from: from(route),
            l_h: l,
            l_from: from(route),
        }),
        None => e2.map(|x| Winding {
            r_ohm: None,
            r_vpc: x.r_vpc,
            r_from: Source::StallFallback,
            l_h: l_default_h,
            l_from: Source::Default,
        }),
    }
}

/// The winding the servo carries: R is `r_q12`, vcounts per ccount in
/// Q12. L has no field of its own; it rides in the current PI, whose kp
/// is w_ci L and whose ki is w_ci R per fast tick, so kp over ki is L/R
/// in fast ticks whatever crossover set them. None unless all four are
/// set.
pub fn stored(
    r_q12: u16,
    i_kp_q88: u16,
    i_ki_q412: u16,
    tick_hz: u16,
    sc: &Scales,
) -> Option<Winding> {
    if r_q12 == 0 || i_kp_q88 == 0 || i_ki_q412 == 0 || tick_hz == 0 {
        return None;
    }
    let r_vpc = r_q12 as f64 / 4096.0;
    let r_ohm = r_vpc * sc.v_term_per_count / sc.amps_per_count;
    let tau_s = (i_kp_q88 as f64 / 256.0) / (i_ki_q412 as f64 / 4096.0 * tick_hz as f64);
    Some(Winding {
        r_ohm: Some(r_ohm),
        r_vpc,
        r_from: Source::Stored,
        l_h: tau_s * r_ohm,
        l_from: Source::Stored,
    })
}

/// How far, as a fraction of the stored R, the burst's rough R may sit
/// before the stored winding is called into question.
pub const STALE_R: f64 = 0.25;

/// The burst's rough R, ohms - its pairs estimate - when it is more than
/// [`STALE_R`] away from the stored winding's R.
pub fn stale(e8: Option<&InductanceResult>, stored: &Winding) -> Option<f64> {
    let rough = e8.and_then(|x| x.volts.r_pair_ohm.or(x.r_pair_ohm))?;
    let r = stored.r_ohm?;
    ((rough - r).abs() > STALE_R * r).then_some(rough)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::burst::{Capture, Chans};
    use crate::exp::Guarded;
    use crate::exp::inductance::{Cfg as InductanceCfg, Inductance};
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

    #[test]
    fn a_promoted_burst_supplies_r_and_l_and_the_stall_never_runs() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        servo.burst = usb_plant();
        let (e8, e2, w) = front(&mut servo, Chans::Driven);
        assert!(e8.promotable(), "{:?}", e8.blocking());
        assert!(e2.is_none(), "E2 ran behind a promoted E8");
        assert_eq!((w.r_from, w.l_from), (Source::Burst, Source::Burst));
        let r = w.r_ohm.unwrap();
        assert!((r - 4.0).abs() / 4.0 < 0.03, "R {r}");
        assert!((w.l_h - 0.6e-3).abs() / 0.6e-3 < 0.05, "L {}", w.l_h);
        assert!((w.r_vpc - scales().r_vpc(r)).abs() < 1e-12);
    }

    #[test]
    fn a_declined_burst_hands_r_to_the_stall_and_l_to_the_default() {
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        servo.burst = usb_plant();
        // no voltage channel on a soft supply: the supply gate declines
        let (e8, e2, w) = front(&mut servo, Chans::Fixed(0));
        assert_eq!(e8.blocking(), vec!["supply"]);
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
        let fit = |caps: Vec<Capture>| fit_captures(&caps, &sc, &FitCfg::default()).unwrap();
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
    /// off the table: R and L come back as they went in, and synthesized
    /// again at the same crossover they encode to the same fields.
    #[test]
    fn stored_winding_writes_back_unchanged() {
        use crate::gains::{self, BwTargets, PlantParams};
        let sc = scales();
        let (r_ohm, l_h) = (4.9, 0.6e-3);
        let l_cd = |l: f64| gains::l_cd_from_si(l, 60, 15_000, 6_800, 3_300).unwrap();
        let plant = |r_vpc: f64, l: f64| PlantParams {
            r_vpc,
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
        let first = gains::encode(&gains::synthesize(&plant(sc.r_vpc(r_ohm), l_h), &t));
        let w = stored(
            first.r_q12.raw,
            first.i_kp_q88.raw,
            first.i_ki_q412.raw,
            20_100,
            &sc,
        )
        .expect("a stored winding");
        assert_eq!((w.r_from, w.l_from), (Source::Stored, Source::Stored));
        assert!((w.r_ohm.unwrap() - r_ohm).abs() / r_ohm < 1e-3, "{w:?}");
        assert!((w.l_h - l_h).abs() / l_h < 5e-3, "{w:?}");
        let again = gains::encode(&gains::synthesize(&plant(w.r_vpc, w.l_h), &t));
        assert_eq!(again.r_q12.raw, first.r_q12.raw);
        assert_eq!(again.i_ki_q412.raw, first.i_ki_q412.raw);
        assert_eq!(again.i_kp_q88.raw, first.i_kp_q88.raw);

        assert!(stored(0, first.i_kp_q88.raw, first.i_ki_q412.raw, 20_100, &sc).is_none());
        assert!(stored(first.r_q12.raw, 0, first.i_ki_q412.raw, 20_100, &sc).is_none());
        assert!(stored(first.r_q12.raw, first.i_kp_q88.raw, 0, 20_100, &sc).is_none());
    }

    #[test]
    fn no_burst_and_no_stall_is_no_winding() {
        assert!(needs_stall(None));
        assert!(winding(None, None, Some(&scales()), DEFAULT_L_HENRIES).is_none());
    }
}
