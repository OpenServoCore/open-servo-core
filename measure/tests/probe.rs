//! The probe step: the rise fit on recorded bursts, part A's judge and
//! part B's ladder on synthetic data, and the step's bus driver on the
//! client's simulated servo for the ways out it can reach.

mod common;

use common::path;
use osc_measure::capture::burst::{self, Capture};
use osc_measure::fit::rise::{self, Lags, Scales};
use serde_json::Value;

fn sources() -> Value {
    serde_json::from_str(&common::text("testdata/bursts/sources.json")).expect("sources")
}

fn stage1(s: &Value) -> Scales {
    let v = |k: &str| s["scales"][k]["value"].as_f64().expect(k);
    let lag = |k: &str| s["scales"]["lags"][k]["value"].as_f64().expect(k);
    Scales {
        counts_per_a: v("counts_per_a"),
        terminal_counts_per_v: v("terminal_counts_per_v"),
        terminal_offset_counts: v("terminal_offset_counts"),
        window_scale: v("window_scale"),
        lags: Lags {
            on: lag("on"),
            tap_on: lag("tap_on"),
            off: lag("off"),
            settle: lag("settle"),
        },
    }
}

fn recorded(name: &str) -> Capture {
    let text = std::fs::read_to_string(path(&format!("testdata/bursts/{name}"))).expect(name);
    burst::from_csv(&text).expect(name)
}

/// The notebook's per-burst R and tau, on its stage 1 scales and each
/// pool's edge term, from the recorded bursts at all three seats.
#[test]
fn the_rise_fit_matches_the_notebook_on_recorded_bursts() {
    let s = sources();
    let scales = stage1(&s);
    let bursts = s["bursts"].as_object().expect("bursts");
    assert!(bursts.len() >= 4);
    for (name, b) in bursts {
        let text = std::fs::read(path(&format!("testdata/bursts/{name}"))).expect(name);
        assert_eq!(
            osc_measure::set::file_sha1(&text),
            b["sha1"].as_str().expect("sha1"),
            "{name} is the recorded file"
        );
        let cap = recorded(name);
        let nb = &b["notebook"];
        let sign = b["sign"].as_f64().expect("sign");
        let t = rise::periods(
            &cap,
            sign,
            &scales,
            nb["edge_volt_ticks"].as_f64().expect("edge"),
        )
        .expect("periods");
        let period_s = rise::pwm_period_s(cap.meta.pwm_arr);
        let f = rise::fit(&t, period_s, scales.window_scale).expect("fits");
        let (r, tau) = (
            nb["r_ohm"].as_f64().expect("r"),
            nb["tau_us"].as_f64().expect("tau"),
        );
        println!(
            "{name}: R {:.4} ohm (notebook {r:.4}), L {:.4} mH, tau {:.2} us (notebook {tau:.2}), {} pairs",
            f.r_ohm,
            f.l_h * 1e3,
            f.tau_s * 1e6,
            f.pairs
        );
        assert_eq!(
            f.pairs as u64,
            nb["pairs"].as_u64().expect("pairs"),
            "{name}"
        );
        assert!(
            (f.r_ohm / r - 1.0).abs() < 0.01,
            "{name}: R {} against {r}",
            f.r_ohm
        );
        assert!(
            (f.tau_s * 1e6 / tau - 1.0).abs() < 0.01,
            "{name}: tau {} against {tau}",
            f.tau_s * 1e6
        );
    }
}

mod synthetic {
    use osc_measure::capture::burst::{CHANS_DIFF, Capture, Meta, SAMPLES, frame_len};
    use osc_measure::fit::rise::{self, Lags, Scales};
    use osc_measure::steps::Verdict;
    use osc_measure::steps::probe::{AFacts, BURSTS_A_GROUP, PartA, Read, Shot, judge, shots};

    const ARR: u16 = 1200;
    const ZERO: f64 = 1206.0;
    const CPA: f64 = 1108.5;
    const CPV: f64 = 248.0;
    const RAIL: f64 = 5.0;
    const LAGS: Lags = Lags {
        on: 44.0,
        tap_on: 39.5,
        off: 14.0,
        settle: 72.0,
    };

    /// A winding of `r` ohm and `l` henry stepped from rest at `q15` by a
    /// bridge of sign `bridge`, sampled as the burst samples: the shunt
    /// reads the winding's current inside the window (its zero between),
    /// the driven tap the rail inside the window, both taps ground in the
    /// brake. `dead` drives nothing; `full_rail` drives every tick.
    fn burst(q15: i16, bridge: f64, r: f64, l: f64, dead: bool, full_rail: bool) -> Capture {
        let period = 2.0 * ARR as f64;
        let (start_cnt, step_index) = (600u16, 480u16);
        let tick = |k: usize| start_cnt as f64 + 52.0 * k as f64 + 27.0;
        let half = rise::half_ticks(q15, ARR);
        let trough = period * (tick(step_index as usize) / period).ceil();
        let tc = |t: f64| {
            ((t - ARR as f64).rem_euclid(period) + period / 2.0).rem_euclid(period) - period / 2.0
        };
        let live = |t: f64, lag: f64| {
            !dead && t > trough && (full_rail || (tc(t) >= -half + lag && tc(t) <= half + LAGS.off))
        };
        let end = tick(SAMPLES - 1) as usize + 1;
        let dt = 1.0 / 48e6;
        let mut i = vec![0.0; end + 1];
        for t in 0..end {
            let v = if live(t as f64, LAGS.tap_on) {
                RAIL
            } else {
                0.0
            };
            i[t + 1] = i[t] + (v - r * i[t]) / l * dt;
        }
        let samples = (0..SAMPLES)
            .map(|k| {
                let t = tick(k);
                let at = i[t.floor() as usize];
                let code = match k % 4 {
                    0 | 2 => ZERO + if live(t, LAGS.on) { CPA * at } else { 0.0 },
                    lane => {
                        if t <= trough {
                            773.0
                        } else {
                            let driven = (lane == 1) == (bridge > 0.0);
                            if driven && live(t, LAGS.tap_on) {
                                CPV * RAIL
                            } else {
                                0.0
                            }
                        }
                    }
                };
                code.round() as u16
            })
            .collect();
        Capture {
            samples,
            meta: Meta {
                pre_q15: 0,
                step_q15: q15,
                step_index,
                start_cnt,
                pwm_arr: ARR,
                start_dir: 0,
                restore_dir: 0,
                vbus_raw: 0,
                bias: ZERO as u16,
                chans: CHANS_DIFF,
                frame_len: frame_len(CHANS_DIFF) as u8,
                vmotor_bias: 773,
                pos: 2000,
                seated: false,
                drive_polarity: true,
            },
        }
    }

    fn facts() -> AFacts {
        AFacts {
            rail_v: RAIL,
            zero_counts: ZERO,
            polarity: true,
            scales: Scales {
                counts_per_a: CPA,
                terminal_counts_per_v: CPV,
                terminal_offset_counts: 0.0,
                window_scale: 1.0,
                lags: LAGS,
            },
        }
    }

    fn reads(r: f64, l: f64, dead_minus: bool, full_rail: bool) -> Vec<Read> {
        reads_by(|_| (r, l), dead_minus, full_rail)
    }

    /// Part A's bursts, each on the winding `rl` gives its shot.
    fn reads_by(rl: impl Fn(&Shot) -> (f64, f64), dead_minus: bool, full_rail: bool) -> Vec<Read> {
        shots(4.8, RAIL)
            .into_iter()
            .map(|s| {
                let (r, l) = rl(&s);
                Read {
                    shot: s,
                    cap: burst(
                        s.duty_q15,
                        s.dir as f64,
                        r,
                        l,
                        dead_minus && s.dir < 0,
                        full_rail,
                    ),
                    after_counts: ZERO,
                }
            })
            .collect()
    }

    const R: f64 = 4.2;
    const L: f64 = 0.82e-3;
    /// A rotor resting where a brush bridges two segments: the winding
    /// reads low, R and L alike, as the bench's outlier bursts did.
    const LOW: f64 = 0.81;

    /// Part A with the bursts `low` read on the low winding.
    fn with_low(low: &[usize]) -> PartA {
        judge(
            &facts(),
            &reads_by(
                |s| {
                    if low.contains(&s.k) {
                        (LOW * R, LOW * L)
                    } else {
                        (R, L)
                    }
                },
                false,
                false,
            ),
        )
    }

    fn check<'a>(a: &'a PartA, name: &str) -> &'a osc_measure::steps::Check {
        a.checks.iter().find(|c| c.name == name).expect(name)
    }

    fn aside(a: &PartA) -> Vec<usize> {
        a.aside().map(|r| r.shot.k).collect()
    }

    #[test]
    fn part_a_reads_r_and_l_of_a_synthetic_rl_rise() {
        let (r, l) = (4.2, 0.82e-3);
        let a = judge(&facts(), &reads(r, l, false, false));
        for c in &a.checks {
            println!("{}", c.said());
        }
        assert!(a.accepted() && !a.bridge_fault);
        let p = a.r_plan_free().expect("a fit part B is priced from");
        println!(
            "R {:.4} ohm, L {:.4} mH, tau {:.1} us, intercept {:+.4} V, {} pairs",
            p.r_ohm,
            p.l_h * 1e3,
            p.tau_s * 1e6,
            p.v0_v,
            p.pairs
        );
        assert!((p.r_ohm / r - 1.0).abs() < 0.10, "R {}", p.r_ohm);
        assert!((p.l_h / l - 1.0).abs() < 0.10, "L {}", p.l_h);
        assert!((a.period_s - 50e-6).abs() < 1e-15, "{}", a.period_s);
        assert_eq!(a.reads.len(), 4 * BURSTS_A_GROUP);
        assert_eq!(a.reads.iter().filter(|r| r.shot.in_fit).count(), 14);
        assert_eq!(a.reads.iter().filter(|r| r.kept()).count(), 14);
        assert_eq!(aside(&a), [0usize; 0]);
        assert_eq!(check(&a, "bursts set aside").verdict, Verdict::NotJudged);
    }

    /// The bursts of a sign and level run in a row, the first after each
    /// sign change out of the fit.
    #[test]
    fn part_a_fires_four_bursts_a_sign_and_level() {
        let s = shots(4.8, RAIL);
        let at: Vec<(i8, usize)> = s.iter().map(|s| (s.dir, s.level)).collect();
        let want: Vec<(i8, usize)> = [(1, 0), (1, 1), (-1, 1), (-1, 0)]
            .into_iter()
            .flat_map(|g| [g; BURSTS_A_GROUP])
            .collect();
        assert_eq!(at, want);
        let out: Vec<usize> = s.iter().filter(|s| !s.in_fit).map(|s| s.k).collect();
        assert_eq!(out, [0, 8]);
    }

    /// One burst on the low winding in a group of four is set aside and
    /// named; the signs read alike, and the planning R is the clean one.
    #[test]
    fn one_low_burst_is_set_aside_and_the_r_stays_clean() {
        let clean = with_low(&[]);
        let a = with_low(&[5]);
        for r in &a.reads {
            println!("{}", r.said(RAIL, a.period_s));
        }
        for c in &a.checks {
            println!("{}", c.said());
        }
        assert_eq!(aside(&a), [5]);
        let five = &a.reads[5];
        assert!(five.move_share > 0.2, "{}", five.move_share);
        assert!(
            five.said(RAIL, a.period_s)
                .contains("(set aside: its first window current is +"),
            "{}",
            five.said(RAIL, a.period_s)
        );
        assert_eq!(
            check(&a, "the rotor held still for the bursts").verdict,
            Verdict::Pass
        );
        assert!(a.accepted());
        let (p, c) = (
            a.r_plan_free().expect("priced"),
            clean.r_plan_free().expect("priced"),
        );
        println!("R {:.4} ohm against the clean {:.4}", p.r_ohm, c.r_ohm);
        assert!(
            (p.r_ohm / c.r_ohm - 1.0).abs() < 1e-3,
            "{} {}",
            p.r_ohm,
            c.r_ohm
        );
        assert!((p.l_h / c.l_h - 1.0).abs() < 1e-3, "{} {}", p.l_h, c.l_h);
        assert!((p.v0_v - c.v0_v).abs() < 1e-3, "{} {}", p.v0_v, c.v0_v);
    }

    /// Two low bursts in one group of four leave it no current to compare:
    /// the still check fails and says what to do.
    #[test]
    fn two_low_bursts_in_a_group_refuse_with_the_sentence() {
        let a = with_low(&[5, 6]);
        let c = check(&a, "the rotor held still for the bursts");
        println!("{}", c.said());
        assert_eq!(c.verdict, Verdict::Fail);
        assert!(
            c.value
                .contains("the rotor did not hold still for the bursts at 32% and the + sign")
                && c.value.contains("turn the horn a little and start again"),
            "{}",
            c.value
        );
        assert!(!a.accepted() && !a.bridge_fault);
        assert!(a.r_plan_free().is_none());
    }

    /// Three set aside across the groups are reported, never refuse.
    #[test]
    fn three_set_aside_are_reported() {
        let a = with_low(&[2, 5, 13]);
        assert_eq!(aside(&a), [2, 5, 13]);
        let c = check(&a, "bursts set aside");
        println!("{}", c.said());
        assert_eq!(c.verdict, Verdict::NotJudged);
        assert!(
            c.value.starts_with("3 of 16 set aside")
                && c.value.contains("brush bridges two segments"),
            "{}",
            c.value
        );
        assert_eq!(
            check(&a, "both signs read the same current").verdict,
            Verdict::NotJudged
        );
        assert!(a.accepted() && a.r_plan_free().is_some());
    }

    /// The two signs reading different currents, and an L / R past the
    /// board's old range, are reported: part A is accepted and prices
    /// part B.
    #[test]
    fn signs_apart_and_a_long_l_over_r_are_reported_not_refused() {
        let short = 0.7 * R * 50e-6;
        let a = judge(
            &facts(),
            &reads_by(
                |s| {
                    if s.dir < 0 {
                        (1.3 * R, 1.3 * short)
                    } else {
                        (R, short)
                    }
                },
                false,
                false,
            ),
        );
        for c in &a.checks {
            println!("{}", c.said());
        }
        let lr = check(&a, "L / R");
        assert_eq!(lr.verdict, Verdict::NotJudged, "{}", lr.said());
        assert!(lr.value.contains("PWM periods"), "{}", lr.value);
        let signs = check(&a, "both signs read the same current");
        assert_eq!(signs.verdict, Verdict::NotJudged, "{}", signs.said());
        assert!(a.accepted() && a.r_plan_free().is_some(), "{:?}", a.checks);
    }

    #[test]
    fn a_bridge_fault_ends_before_part_b() {
        let (r, l) = (4.2, 0.82e-3);
        for (dead, full, word) in [
            (true, false, "one half of the bridge drives nothing"),
            (false, true, "a full-rail pulse at partial duty"),
        ] {
            let a = judge(&facts(), &reads(r, l, dead, full));
            let bridge = a
                .checks
                .iter()
                .find(|c| c.name == "the bridge drives both signs")
                .expect("checked");
            assert!(
                a.bridge_fault && bridge.value.contains(word),
                "{}",
                bridge.value
            );
            assert!(!a.accepted());
            assert!(a.r_plan_free().is_none(), "no R prices part B: {word}");
        }
    }
}

/// The two bench runs the old signs check refused, each for one burst
/// reading about 0.81 x its neighbors' R: on its first window current that
/// burst alone is over the set aside share off the same level's others,
/// both signs. The old layout ran two bursts a sign and level, too few for
/// the rule as part A now applies it.
#[test]
fn the_recorded_low_burst_is_the_one_the_first_window_rule_flags() {
    use osc_measure::fit::seated::{self, SET_ASIDE_SHARE};

    let s: Value =
        serde_json::from_str(&common::text("testdata/probe/sources.json")).expect("sources");
    let v = |k: &str| s["scales"][k].as_f64().expect(k);
    let lag = |k: &str| s["scales"]["lags_ticks"][k].as_f64().expect(k);
    let scales = Scales {
        counts_per_a: v("counts_per_a"),
        terminal_counts_per_v: v("terminal_counts_per_v"),
        terminal_offset_counts: v("terminal_offset_counts"),
        window_scale: v("window_scale"),
        lags: Lags {
            on: lag("on"),
            tap_on: lag("tap_on"),
            off: lag("off"),
            settle: lag("settle"),
        },
    };
    for (run, x) in s["runs"].as_object().expect("runs") {
        let outlier = x["outlier"].as_u64().expect("outlier") as usize;
        let mut firsts = Vec::new();
        let mut rs = Vec::new();
        for (k, level) in x["levels"].as_array().expect("levels").iter().enumerate() {
            let name = format!("burst-{k}.csv");
            let text = std::fs::read(path(&format!("testdata/probe/{run}/{name}"))).expect(&name);
            assert_eq!(
                osc_measure::set::file_sha1(&text),
                x["sha1"][&name].as_str().expect("sha1"),
                "{run}/{name} is the recorded file"
            );
            let cap = burst::from_csv(std::str::from_utf8(&text).expect("text")).expect(&name);
            let t = rise::periods(&cap, seated::wiring_sign(&cap), &scales, 0.0).expect("periods");
            let f = rise::fit(&t, rise::pwm_period_s(cap.meta.pwm_arr), 1.0).expect("fit");
            firsts.push((level.as_u64().expect("level"), t[0].i_a));
            rs.push(f.r_ohm);
        }
        let shares = seated::off_others(&firsts);
        for (k, m) in shares.iter().enumerate() {
            println!(
                "{run} burst {k}: first window {:.1} mA ({:+.1}% off its level's others), R {:.3} ohm",
                1e3 * firsts[k].1,
                100.0 * m,
                rs[k]
            );
        }
        let flagged: Vec<usize> = (0..shares.len())
            .filter(|k| seated::moved(shares[*k]).is_some())
            .collect();
        assert_eq!(flagged, [outlier], "{run}: {shares:?}");
        assert!(
            shares[outlier] > SET_ASIDE_SHARE,
            "{run}: reads the winding low"
        );
    }
}

/// A probe result's free-rotor R can feed no set field and no rules
/// input: its names are none the rules read, and the measured inputs read
/// off it hold nothing.
#[test]
fn no_set_field_or_rules_input_is_fed_from_the_probes_r() {
    use osc_measure::fit::rise::Rise;
    use osc_measure::inputs::Measured;
    use osc_measure::steps::probe::estimates;
    use osc_measure::{Outcome, build};

    let rise = Rise {
        r_ohm: 4.25,
        l_h: 0.85e-3,
        tau_s: 0.2e-3,
        v0_v: 0.01,
        pairs: 48,
        resid_v: vec![],
    };
    let est = serde_json::Value::Object(estimates(Some(&rise), Some(0.9 * 4.25)));
    assert!(est["r_plan_free_ohm"]["value"] == 4.25, "{est}");
    let read: Measured = serde_json::from_value(est.clone()).expect("reads as measured inputs");
    assert!(read.r_ohm.is_none() && read.l_mh.is_none(), "{read:?}");

    let inputs = common::inputs();
    let set = build(&inputs);
    let mut read_by_rules = Vec::new();
    for (_, o) in &set.fields {
        if let Outcome::Bound(e) = o {
            read_by_rules.extend(e.inputs.iter().cloned());
        }
    }
    assert!(
        read_by_rules.contains(&"measured.r_ohm"),
        "the rules read R"
    );
    for key in est.as_object().expect("map").keys() {
        assert!(
            !read_by_rules
                .iter()
                .any(|i| i.ends_with(&format!(".{key}"))),
            "{key} is read by a rule"
        );
    }
    let mut fed = inputs.clone();
    fed.measured = read;
    let none = osc_measure::Inputs {
        measured: Measured::default(),
        ..inputs
    };
    let (a, b) = (build(&fed), build(&none));
    for (name, o) in &a.fields {
        assert_eq!(
            format!("{o:?}"),
            format!("{:?}", b.get(name).expect(name)),
            "{name} moved with the probe's R"
        );
    }
}
