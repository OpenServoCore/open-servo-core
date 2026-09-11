//! Where every plant input gain synthesis uses came from. The winding's R
//! and L come from the E8 burst when it promotes
//! ([`InductanceResult::promotable`]); otherwise R falls back to the E2
//! end-stop stall and L to the configured default. The rest have one source
//! each.

use crate::exp::inductance::InductanceResult;
use crate::exp::resistance::ResistanceResult;
use crate::exp::rl::Scales;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Source {
    /// E8, the high-rate burst, promoted.
    Burst,
    /// E2, the end-stop stall, run because E8 declined.
    StallFallback,
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
            Source::Burst => "E8 burst",
            Source::StallFallback => "E2 fallback",
            Source::Default => "default",
            Source::Ladder => "E3 ladder",
            Source::Inertia => "E4 inertia",
            Source::Bias => "E0 bias",
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

/// True when the stall has to run: E8 was not run, or it declined.
pub fn needs_stall(e8: Option<&InductanceResult>) -> bool {
    !e8.is_some_and(|r| r.promotable())
}

/// E8's R and L when it promotes, else E2's R with `l_default_h`. None when
/// E8 declined and no stall ran. `sc` converts E8's ohms to the table's
/// units; a recording too old to carry the scales has no E8 to promote.
pub fn winding(
    e8: Option<&InductanceResult>,
    e2: Option<&ResistanceResult>,
    sc: Option<&Scales>,
    l_default_h: f64,
) -> Option<Winding> {
    match e8.and_then(InductanceResult::gain_r_l).zip(sc) {
        Some(((r, l), sc)) => Some(Winding {
            r_ohm: Some(r),
            r_vpc: sc.r_vpc(r),
            r_from: Source::Burst,
            l_h: l,
            l_from: Source::Burst,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::burst::Chans;
    use crate::exp::inductance::{Cfg as InductanceCfg, Inductance};
    use crate::exp::resistance::{Resistance, ResistanceCfg};
    use crate::exp::testkit::{FakeServo, SynthBurst, pump};
    use crate::exp::{Guarded, RigParams};
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
        let params = RigParams::default();
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

    #[test]
    fn no_burst_and_no_stall_is_no_winding() {
        assert!(needs_stall(None));
        assert!(winding(None, None, Some(&scales()), DEFAULT_L_HENRIES).is_none());
    }
}
