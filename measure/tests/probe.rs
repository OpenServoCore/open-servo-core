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
    use osc_measure::steps::probe::{AFacts, Read, judge, shots};

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
        shots(4.8, RAIL)
            .into_iter()
            .map(|s| Read {
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
            })
            .collect()
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
        assert_eq!(a.reads.iter().filter(|r| r.shot.in_fit).count(), 6);
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
