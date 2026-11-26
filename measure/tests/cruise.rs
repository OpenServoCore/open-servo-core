//! The cruise fit on the record's own cruises: stage 5 cruise-friction's
//! two sittings on mg90-d (`testdata/cruise`, written with their expected
//! values by its `gen.py`, which runs the notebook's own cells). Each leg's
//! replay and held mean against the notebook's, then the step's line on
//! the first sitting against the line of record, and the joint line.

use std::collections::BTreeMap;

use osc_measure::capture::meta::gunzip;
use osc_measure::capture::rows::TelFrame;
use osc_measure::fit::cruise::{self, Leg, LegRead, Point, Poll, Reading};
use osc_measure::fit::window::Settle;
use osc_measure::scorecard::Lin;
use osc_measure::steps::Verdict;
use serde_json::Value;

fn dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata/cruise")
}

fn expected() -> Value {
    serde_json::from_slice(&std::fs::read(dir().join("expected.json")).expect("expected"))
        .expect("json")
}

/// A stored leg: where it is from, its polls and its rows.
struct Stored {
    run: String,
    capture: String,
    seg: i64,
    goal: f64,
    before: Poll,
    after: Poll,
    t_read_cc: f64,
    frames: Vec<TelFrame>,
}

fn sums(v: &Value) -> Vec<i64> {
    let mut acc = 0;
    v.as_array()
        .expect("steps")
        .iter()
        .map(|x| {
            acc += x.as_i64().expect("int");
            acc
        })
        .collect()
}

fn poll(v: &Value) -> Poll {
    let n = |k: &str| v[k].as_f64().unwrap_or_else(|| panic!("{k}"));
    Poll {
        vbus_raw: n("vbus_raw"),
        t_winding_cc: n("t_winding_cc"),
        bias_counts: n("current_bias_counts"),
    }
}

fn legs() -> Vec<Stored> {
    let bytes = gunzip(&std::fs::read(dir().join("legs.json.gz")).expect("legs")).expect("gzip");
    let doc: Value = serde_json::from_slice(&bytes).expect("json");
    doc.as_array()
        .expect("legs")
        .iter()
        .map(|l| {
            let valid: Vec<bool> = l["window_valid"]
                .as_str()
                .expect("flags")
                .bytes()
                .map(|b| b == b'1')
                .collect();
            let pos = sums(&l["pos"]);
            let duty = sums(&l["duty_counts"]);
            let mut raw = sums(&l["current_raw"]).into_iter();
            let first = l["first_tick"].as_u64().expect("first");
            assert_eq!(valid.len() as u64, l["n"].as_u64().expect("n"));
            let frames = (0..valid.len())
                .map(|k| TelFrame {
                    tick: first + k as u64,
                    window_valid: valid[k],
                    pos: Some(pos[k] as u16),
                    duty_counts: Some(duty[k] as i16),
                    current_raw: if valid[k] {
                        Some(raw.next().expect("a read row's current") as u16)
                    } else {
                        None
                    },
                    ..TelFrame::default()
                })
                .collect();
            assert!(raw.next().is_none(), "every stored current is a read row's");
            Stored {
                run: l["run"].as_str().expect("run").into(),
                capture: l["capture"].as_str().expect("capture").into(),
                seg: l["seg"].as_i64().expect("seg"),
                goal: l["goal"].as_f64().expect("goal"),
                before: poll(&l["before"]),
                after: poll(&l["after"]),
                t_read_cc: l["t_read_cc"].as_f64().expect("t_read"),
                frames,
            }
        })
        .collect()
}

fn lin(e: &Value) -> Lin {
    let ints = |k: &str| -> Vec<i64> {
        e[k].as_array()
            .expect(k)
            .iter()
            .map(|v| v.as_i64().expect(k))
            .collect()
    };
    let points: Vec<i16> = ints("points").into_iter().map(|p| p as i16).collect();
    let points: [i16; 256] = points.try_into().expect("256 points");
    let slots: Vec<u8> = ints("slots").into_iter().map(|s| s as u8).collect();
    Lin::new(Some(&points), Some(&slots))
}

/// The reading a sitting's legs replay through: the set's words and the
/// sitting's own.
fn reading(e: &Value, run: &str) -> Reading {
    let c = &e["constants"];
    let s = &e["sittings"][run];
    let n = |v: &Value, k: &str| v[k].as_f64().unwrap_or_else(|| panic!("{k}"));
    let w = |v: &Value, k: &str| n(v, k) as u16;
    Reading {
        counts_per_a: n(c, "counts_per_a"),
        g_window_q016: w(s, "g_window_q016"),
        i_kp_q88: w(c, "i_kp_q88"),
        i_ki_q412: w(c, "i_ki_q412"),
        ke_vpc_q: w(c, "ke_vpc_q"),
        r_hat_q12: w(s, "r_hat_q12"),
        j_ff_q88: n(c, "j_ff_q88"),
        fric_fc_counts: n(s, "fric_fc_counts"),
        fric_fv_q016: n(s, "fric_fv_q016"),
        pwm_arr: w(c, "pwm_arr"),
        tick_hz: n(c, "tick_hz"),
        bridge_min_ticks: n(c, "bridge_min_ticks"),
        settle: Settle {
            start_ticks: w(c, "settle_start_ticks"),
            q15: c["settle_q15"]
                .as_array()
                .expect("settle")
                .iter()
                .map(|v| v.as_u64().expect("q15") as u16)
                .collect(),
        },
        counts_per_motor_rad: n(c, "counts_per_motor_rad"),
        ceiling_cps: n(s, "velocity_limit_cps"),
    }
}

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol,
        "{what}: {got} against {want} (tol {tol})"
    );
}

/// Every stored leg read, beside the notebook's numbers for it.
fn read_all() -> (Value, Vec<(Value, LegRead)>) {
    let e = expected();
    let lin = lin(&e);
    let by: BTreeMap<(String, String, i64), Value> = e["legs"]
        .as_array()
        .expect("legs")
        .iter()
        .map(|l| {
            let k = (
                l["run"].as_str().expect("run").to_string(),
                l["capture"].as_str().expect("capture").to_string(),
                l["seg"].as_i64().expect("seg"),
            );
            (k, l.clone())
        })
        .collect();
    let stored = legs();
    assert_eq!(stored.len(), by.len());
    let reads = stored
        .iter()
        .map(|s| {
            let r = reading(&e, &s.run);
            let leg = Leg {
                goal_cps: s.goal,
                rows: &s.frames,
                before: s.before,
                after: s.after,
                t_read_cc: s.t_read_cc,
            };
            let got = cruise::read(&leg, &lin, &r).expect("read");
            (by[&(s.run.clone(), s.capture.clone(), s.seg)].clone(), got)
        })
        .collect();
    (e, reads)
}

/// Each leg against the notebook's `leg_numbers` and `inertia_ma`: the
/// phase found exactly, the replay's spreads, the held mean, the read
/// rows' mean, the integrator's share, the speed and its halves, the
/// inertia current and the steady mean, to rounding.
#[test]
fn every_leg_reads_as_the_notebook() {
    let (_, reads) = read_all();
    let mut worst: BTreeMap<&str, f64> = BTreeMap::new();
    for (want, got) in &reads {
        let n = |k: &str| want[k].as_f64().unwrap_or(f64::NAN);
        let at = format!(
            "{} {} seg {}",
            want["run"].as_str().unwrap_or(""),
            want["capture"].as_str().unwrap_or(""),
            want["seg"]
        );
        assert_eq!(got.phase, want["phase"].as_i64().expect("phase"), "{at}");
        assert_eq!(got.dir as i64, want["dir"].as_i64().expect("dir"), "{at}");
        assert_eq!(
            got.rows as i64,
            want["rows"].as_i64().expect("rows"),
            "{at}"
        );
        let held_in = got.held_in_ms.unwrap_or(f64::NAN);
        for (name, v, k, tol) in [
            ("held", got.held_ma, "held_ma", 1e-9),
            ("read", got.read_ma, "read_ma", 1e-9),
            ("read share", got.read_share, "read_share", 1e-12),
            ("line", got.line_ma, "line_ma", 1e-9),
            ("integrator", got.integ_ma, "integ_ma", 1e-9),
            ("sd in", got.sd_in_ma, "sd_in_ma", 1e-9),
            ("sd worst", got.sd_worst_ma, "sd_worst_ma", 1e-9),
            ("cps", got.speed_cps, "cps", 1e-6),
            ("halves", 100.0 * got.halves, "v2_v1_pct", 1e-6),
            ("held in", held_in, "held_in_ms", 1e-9),
            ("inertia", got.inertia_ma, "inertia_ma", 1e-9),
            ("steady", got.held_ma - got.inertia_ma, "steady_ma", 1e-9),
        ] {
            close(&format!("{at} {name}"), v, n(k), tol);
            let d = worst.entry(name).or_insert(0.0);
            *d = d.max((v - n(k)).abs());
        }
    }
    println!("{} legs, worst differences: {worst:?}", reads.len());
}

fn word(x: f64) -> i64 {
    (x.abs() + 0.5).floor().copysign(x) as i64
}

/// The step's line on the first sitting's whole legs is the line of
/// record, the notebook's `LINE`: the top speed's up leg still gaining
/// 5.55% across its steady rows is left out, named; the other 19, held
/// means against the goal, give 37.64 mA + 12.73 per 1000 c/s, the words
/// 43 / 947; the line's checks pass.
#[test]
fn the_steps_line_is_the_line_of_record() {
    let (e, reads) = read_all();
    let g3: Vec<&(Value, LegRead)> = reads.iter().filter(|(w, _)| w["run"] == "g3").collect();
    let fails = |l: &LegRead| l.fails(l.goal_cps.abs() == 4025.0);
    let aside: Vec<String> = g3
        .iter()
        .filter(|(_, l)| !fails(l).is_empty())
        .map(|(w, l)| format!("{} seg {}: {:?}", w["capture"], w["seg"], fails(l)))
        .collect();
    println!("set aside: {aside:?}");
    assert_eq!(aside.len(), 1, "{aside:?}");
    assert!(
        aside[0].contains("c4025") && aside[0].contains("+5.55% across its steady rows"),
        "{aside:?}"
    );
    let kept: Vec<&&(Value, LegRead)> = g3.iter().filter(|(_, l)| fails(l).is_empty()).collect();
    assert!(kept.iter().all(|(w, _)| w["in_line"] == true));
    let points: Vec<Point> = kept
        .iter()
        .map(|(_, l)| Point {
            goal_cps: l.goal_cps.abs(),
            dir: l.dir,
            i_ma: l.held_ma,
        })
        .collect();
    let c = &e["constants"];
    let cpmr = c["counts_per_motor_rad"].as_f64().expect("cpmr");
    let cpa = c["counts_per_a"].as_f64().expect("cpa");
    let got = cruise::line(&points, cpmr).expect("line");
    let x = &e["line"];
    let n = |k: &str| x[k].as_f64().unwrap_or_else(|| panic!("{k}"));
    println!(
        "the line: {} legs, {:.4} +/- {:.4} mA + {:.4} +/- {:.4} per 1000 c/s ({:.5} per rad/s), rms {:.3}; the notebook's {:.4} + {:.4}",
        got.legs,
        got.coulomb_ma,
        got.coulomb_se_ma,
        got.viscous_ma_per_kcps,
        got.viscous_se_ma_per_kcps,
        got.viscous_ma_s_per_rad,
        got.rms_ma,
        n("intercept_ma"),
        n("slope_ma_per_kcps")
    );
    assert_eq!(got.legs as u64, x["legs"].as_u64().expect("legs"));
    for (name, v, k, tol) in [
        ("coulomb", got.coulomb_ma, "intercept_ma", 1e-9),
        ("coulomb se", got.coulomb_se_ma, "intercept_se_ma", 1e-9),
        (
            "viscous",
            got.viscous_ma_per_kcps,
            "slope_ma_per_kcps",
            1e-9,
        ),
        (
            "viscous se",
            got.viscous_se_ma_per_kcps,
            "slope_se_ma_per_kcps",
            1e-9,
        ),
        (
            "viscous per rad/s",
            got.viscous_ma_s_per_rad,
            "viscous_ma_s_per_rad",
            1e-12,
        ),
        ("rms", got.rms_ma, "rms_ma", 1e-9),
    ] {
        close(&format!("line {name}"), v, n(k), tol);
    }
    let words = [
        word(got.coulomb_ma * cpa / 1e3),
        word(got.viscous_ma_per_kcps / 1e3 * cpa / 1e3 * 65536.0),
    ];
    assert_eq!(words, [43, 947]);
    let checks = cruise::checks(&points, Some(&got), 4025.0);
    for c in &checks {
        println!("  {}", c.said());
    }
    assert!(
        checks.iter().all(|c| c.verdict == Verdict::NotJudged),
        "{checks:?}"
    );
}

/// The joint line, reported by the notebook and not the step's: both
/// sittings, the first's 4025 legs out, each leg's reported inertia
/// current off its held mean, against the notebook's `J`.
#[test]
fn the_joint_line_is_the_notebooks() {
    let (e, reads) = read_all();
    let cpmr = e["constants"]["counts_per_motor_rad"]
        .as_f64()
        .expect("cpmr");
    let points: Vec<Point> = reads
        .iter()
        .filter(|(w, _)| w["in_joint"] == true)
        .map(|(_, l)| Point {
            goal_cps: l.goal_cps.abs(),
            dir: l.dir,
            i_ma: l.held_ma - l.inertia_ma,
        })
        .collect();
    let got = cruise::line(&points, cpmr).expect("line");
    let x = &e["joint"];
    let n = |k: &str| x[k].as_f64().unwrap_or_else(|| panic!("{k}"));
    assert_eq!(got.legs as u64, x["legs"].as_u64().expect("legs"));
    close("joint coulomb", got.coulomb_ma, n("intercept_ma"), 1e-9);
    close(
        "joint viscous",
        got.viscous_ma_per_kcps,
        n("slope_ma_per_kcps"),
        1e-9,
    );
}
