//! The cruise fit on recorded rows: closed-loop Velocity cruises of the
//! mg90-d, trimmed to the rows the fit reads (`testdata/mg90-d/cruise`,
//! written with their expected values by its `gen.py`, a Python reference
//! of the method). Each leg's numbers against the reference, then the
//! line against the reference and against the analyst's lines of the
//! same captures.

use std::collections::BTreeMap;

use osc_measure::Board;
use osc_measure::capture::meta::gunzip;
use osc_measure::capture::rows::TelFrame;
use osc_measure::fit::cruise::{
    self, HALVES_FROM_SHARE, Leg, LegRead, Line, Point, Reading, band, settled_span,
};
use osc_measure::fit::window::Settle;
use osc_measure::scorecard::Lin;
use osc_measure::steps::Verdict;
use serde_json::Value;

fn dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata/mg90-d/cruise")
}

fn expected() -> Value {
    serde_json::from_slice(&std::fs::read(dir().join("expected.json")).expect("expected"))
        .expect("json")
}

/// A stored leg: its numbers and its rows, each row's tick its place in
/// the leg.
struct Stored {
    run: String,
    seg: i64,
    goal: f64,
    rows: usize,
    bias: f64,
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
            let every = l["vdiff_every"].as_u64().expect("every") as usize;
            let vdiff: Vec<i64> = l["vdiff"]
                .as_array()
                .expect("vdiff")
                .iter()
                .map(|v| v.as_i64().expect("int"))
                .collect();
            let first = l["first"].as_u64().expect("first");
            let frames = (0..valid.len())
                .map(|k| {
                    let tap = (k % every == 0).then(|| vdiff[k / every]);
                    TelFrame {
                        tick: first + k as u64,
                        window_valid: valid[k],
                        pos: Some(pos[k] as u16),
                        duty_counts: Some(duty[k] as i16),
                        current_raw: if valid[k] {
                            Some(raw.next().expect("a read row's current") as u16)
                        } else {
                            None
                        },
                        vmotor_a: tap.map(|v| (2048 + v) as u16),
                        vmotor_b: tap.map(|_| 2048),
                        ..TelFrame::default()
                    }
                })
                .collect();
            assert!(raw.next().is_none(), "every stored current is a read row's");
            Stored {
                run: l["run"].as_str().expect("run").into(),
                seg: l["seg"].as_i64().expect("seg"),
                goal: l["goal"].as_f64().expect("goal"),
                rows: l["rows"].as_u64().expect("rows") as usize,
                bias: l["bias_counts"].as_f64().expect("bias"),
                frames,
            }
        })
        .collect()
}

fn lin(e: &Value) -> Lin {
    let points: Vec<i16> = e["points"]
        .as_array()
        .expect("points")
        .iter()
        .map(|v| v.as_i64().expect("point") as i16)
        .collect();
    let points: [i16; 256] = points.try_into().expect("256 points");
    let slots: Vec<u8> = e["slots"]
        .as_array()
        .expect("slots")
        .iter()
        .map(|v| v.as_u64().expect("slot") as u8)
        .collect();
    Lin::new(Some(&points), Some(&slots))
}

fn reading(e: &Value) -> Reading {
    let c = &e["constants"];
    let n = |k: &str| c[k].as_f64().unwrap_or_else(|| panic!("{k}"));
    let board: Board = serde_json::from_str(
        &std::fs::read_to_string(
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("boards/osc-dev-v006.json"),
        )
        .expect("board"),
    )
    .expect("board json");
    let w = board.window_read.expect("the board's crest read");
    assert_eq!(board.bridge_min_ticks, Some(n("bridge_min_ticks")));
    Reading {
        counts_per_a: n("counts_per_a"),
        g_window_q016: n("g_window_q016"),
        j_ff_q88: n("j_ff_q88"),
        pwm_arr: n("pwm_arr"),
        tick_hz: n("tick_hz"),
        medium_hz: n("tick_hz") / 10.0,
        v_window_min_ticks: n("v_window_min_ticks") as u16,
        bridge_min_ticks: n("bridge_min_ticks"),
        settle: Settle {
            start_ticks: w.settle_start_ticks,
            q15: w.settle_q15,
        },
        counts_per_motor_rad: n("counts_per_deg") * 180.0 / core::f64::consts::PI / n("gear_ratio"),
        ceiling_cps: n("ceiling_cps"),
    }
}

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol,
        "{what}: {got} against {want} (tol {tol})"
    );
}

fn leg(s: &Stored) -> Leg<'_> {
    Leg {
        goal_cps: s.goal,
        rows: &s.frames,
        planned_rows: s.rows,
        bias_counts: s.bias,
    }
}

/// A leg's reference values by name, and the leg as read.
type Read = (BTreeMap<&'static str, f64>, Result<LegRead, String>);

/// Every leg read as the step reads it: the windows by speed, then each
/// leg over its speed's window.
fn read_all(stored: &[Stored], lin: &Lin, r: &Reading, travel: (f64, f64)) -> Vec<Read> {
    let b = band(travel);
    let mut windows = BTreeMap::new();
    for g in [300.0, 2150.0, 4025.0] {
        let spans: Vec<(i8, (f64, f64))> = stored
            .iter()
            .filter(|s| s.goal.abs() == g)
            .filter_map(|s| Some((leg(s).dir(), settled_span(&leg(s), lin, b)?)))
            .collect();
        windows.insert(g as i64, cruise::window(&spans).expect("a window"));
    }
    stored
        .iter()
        .map(|s| {
            let w = windows[&(s.goal.abs() as i64)];
            let mut m = BTreeMap::new();
            m.insert("window_lo", w.0);
            m.insert("window_hi", w.1);
            (m, cruise::read(&leg(s), lin, w, r))
        })
        .collect()
}

fn line_of(
    stored: &[Stored],
    reads: &[Result<LegRead, String>],
    r: &Reading,
    low: bool,
) -> Option<Line> {
    let points: Vec<Point> = stored
        .iter()
        .zip(reads)
        .filter_map(|(s, l)| l.as_ref().ok().map(|l| (s, l)))
        .filter(|(_, l)| l.fails(r).is_empty())
        .filter(|(s, _)| low || s.goal.abs() >= HALVES_FROM_SHARE * r.ceiling_cps)
        .map(|(s, l)| Point {
            goal_cps: s.goal.abs(),
            dir: l.dir,
            rad_s: l.rad_s,
            i_ma: l.i_ma,
        })
        .collect();
    cruise::line(&points)
}

fn said(l: &Line) -> String {
    format!(
        "{:.2} +/- {:.2} mA + {:.5} +/- {:.5} mA per rad/s over {:.0} to {:.0} rad/s, {} legs, residuals {}",
        l.coulomb_ma,
        l.coulomb_se_ma,
        l.viscous_ma_s_per_rad,
        l.viscous_se,
        l.rad_s.0,
        l.rad_s.1,
        l.legs,
        l.speeds
            .iter()
            .map(|s| format!("{:+.2} at {:.0}", s.residual_ma, s.goal_cps))
            .collect::<Vec<_>>()
            .join(", ")
    )
}

/// The table and slots read a raw pot as the notebooks' do.
#[test]
fn the_fixture_reads_the_pot_as_the_notebooks() {
    let e = expected();
    let lin = lin(&e);
    for s in e["lin_samples"].as_array().expect("samples") {
        let raw = s[0].as_u64().expect("raw") as u16;
        close(
            &format!("lin({raw})"),
            lin.counts(raw),
            s[1].as_f64().expect("lin"),
            1e-9,
        );
    }
}

/// Each leg's cut, reading, all-tick estimate, speed, inertia and
/// verdict equal the reference's on the same rows.
#[test]
fn each_leg_reads_as_the_reference() {
    let e = expected();
    let (lin, r) = (lin(&e), reading(&e));
    let travel = (
        e["constants"]["travel_counts"][0].as_f64().expect("lo"),
        e["constants"]["travel_counts"][1].as_f64().expect("hi"),
    );
    let stored = legs();
    let reads = read_all(&stored, &lin, &r, travel);
    let want = e["legs"].as_array().expect("legs");
    assert_eq!(want.len(), stored.len());
    for ((s, (w, got)), x) in stored.iter().zip(&reads).zip(want) {
        let at = format!("{} seg {}", s.run, s.seg);
        assert_eq!(x["run"], s.run.as_str(), "{at}");
        assert_eq!(x["seg"], s.seg, "{at}");
        let ew = &e["windows"][format!("{}", s.goal.abs())];
        close(
            &format!("{at} window lo"),
            w["window_lo"],
            ew[0].as_f64().expect("lo"),
            1e-9,
        );
        close(
            &format!("{at} window hi"),
            w["window_hi"],
            ew[1].as_f64().expect("hi"),
            1e-9,
        );
        match (got, x.get("kept_rows")) {
            (Err(why), None) => {
                println!("{at}: not read, {why} (reference: {})", x["fail"]);
                continue;
            }
            (Ok(_), None) => panic!("{at}: read, the reference did not ({})", x["fail"]),
            (Err(why), Some(_)) => panic!("{at}: not read ({why}), the reference read it"),
            (Ok(l), Some(_)) => {
                let n = |k: &str| x[k].as_f64().unwrap_or_else(|| panic!("{at} {k}"));
                assert_eq!(l.rows as f64, n("kept_rows"), "{at}");
                assert_eq!(l.first_tick as f64, n("first_kept"), "{at}");
                close(&format!("{at} v_on"), l.v_on_counts, n("v_on_counts"), 1e-9);
                close(
                    &format!("{at} read share"),
                    l.read_share,
                    n("read_share"),
                    1e-12,
                );
                close(
                    &format!("{at} read mean"),
                    l.i_read_ma,
                    n("i_read_ma"),
                    1e-6,
                );
                let all = l.all_ticks.expect("an all-tick fit");
                assert_eq!(
                    (all.lag as f64, all.tau_us),
                    (n("lag"), n("tau_us")),
                    "{at}"
                );
                close(&format!("{at} all-tick"), all.counts, n("i_all_ma"), 1e-6);
                close(&format!("{at} used"), l.i_used_ma, n("i_used_ma"), 1e-6);
                close(&format!("{at} speed"), l.speed_cps, n("cps"), 1e-6);
                close(&format!("{at} halves"), l.halves, n("halves"), 1e-9);
                close(
                    &format!("{at} inertia"),
                    l.i_inertia_ma,
                    n("i_inertia_ma"),
                    1e-6,
                );
                close(&format!("{at} current"), l.i_ma, n("i_ma"), 1e-6);
                let fails = l.fails(&r);
                assert_eq!(fails.is_empty(), x["valid"] == true, "{at}: {fails:?}");
                println!(
                    "{at}: {:.0} rows, {:.1} rad/s, read {:.2}{}, {:.2} mA, inertia {:+.2}, halves {:+.2}%: {}",
                    l.rows,
                    l.rad_s,
                    l.read_share,
                    if l.all_ticks_used { " (all-tick)" } else { "" },
                    l.i_ma,
                    l.i_inertia_ma,
                    100.0 * l.halves,
                    if fails.is_empty() {
                        "valid".into()
                    } else {
                        fails.join("; ")
                    }
                );
            }
        }
    }
}

/// The line the step ships, every valid leg's, the 300 c/s legs
/// anchoring the offset, reproduces the reference and the analyst's line
/// with the 300 c/s legs within its error bars, and its checks pass. The
/// line without them is held to the analyst's line without them, the one
/// over 1500 to 4025 c/s.
#[test]
fn the_line_reproduces_the_analysts_within_its_error_bars() {
    let e = expected();
    let (lin, r) = (lin(&e), reading(&e));
    let travel = (110.0, 3730.0);
    let stored = legs();
    let reads: Vec<Result<LegRead, String>> = read_all(&stored, &lin, &r, travel)
        .into_iter()
        .map(|(_, l)| l)
        .collect();
    let points: Vec<Point> = stored
        .iter()
        .zip(&reads)
        .filter_map(|(s, l)| {
            l.as_ref()
                .ok()
                .filter(|l| l.fails(&r).is_empty())
                .map(|l| (s, l))
        })
        .map(|(s, l)| Point {
            goal_cps: s.goal.abs(),
            dir: l.dir,
            rad_s: l.rad_s,
            i_ma: l.i_ma,
        })
        .collect();
    let shipped = cruise::line(&points).expect("the step's line");
    assert_eq!(
        Some(&shipped),
        line_of(&stored, &reads, &r, true).as_ref(),
        "the line over every valid leg is the one with the 300 c/s legs"
    );
    let without = line_of(&stored, &reads, &r, false).expect("a line without 300");
    println!("the step's line: {}", said(&shipped));
    println!(
        "  errors: coulomb {:.2} mA from {}, legs {:.2}, speeds {:?}; viscous {:.5} from {}",
        shipped.coulomb_se_ma,
        shipped.coulomb_se_from,
        shipped.coulomb_se_legs_ma,
        shipped.coulomb_se_speeds_ma,
        shipped.viscous_se,
        shipped.viscous_se_from
    );
    println!("without the 300 c/s legs: {}", said(&without));
    for (got, key) in [(&shipped, "line_with_low"), (&without, "line")] {
        let x = &e[key];
        for (name, v, k, tol) in [
            ("coulomb", got.coulomb_ma, "coulomb_ma", 1e-6),
            ("coulomb se", got.coulomb_se_ma, "coulomb_se_ma", 1e-6),
            (
                "coulomb se legs",
                got.coulomb_se_legs_ma,
                "coulomb_se_legs_ma",
                1e-6,
            ),
            (
                "viscous",
                got.viscous_ma_s_per_rad,
                "viscous_ma_s_per_rad",
                1e-9,
            ),
            ("viscous se", got.viscous_se, "viscous_se", 1e-9),
            (
                "viscous se legs",
                got.viscous_se_legs,
                "viscous_se_legs",
                1e-9,
            ),
        ] {
            close(&format!("{key} {name}"), v, x[k].as_f64().expect(k), tol);
        }
        let from = |k: &str| x[k].as_str().expect(k).to_string();
        let ours = |f: &str| {
            if f == cruise::SE_SPEEDS {
                "speeds"
            } else {
                "legs"
            }
        };
        assert_eq!(ours(got.coulomb_se_from), from("coulomb_se_from"), "{key}");
        assert_eq!(ours(got.viscous_se_from), from("viscous_se_from"), "{key}");
        assert_eq!(got.legs as u64, x["legs"].as_u64().expect("legs"), "{key}");
    }
    assert_eq!(shipped.coulomb_se_from, cruise::SE_SPEEDS);
    for (got, key, span) in [
        (&shipped, "line_with_low", "with the 300 c/s legs"),
        (&without, "line", "over 400 to 1076 rad/s"),
    ] {
        let a = &e["analyst"][key];
        let n = |k: &str| a[k].as_f64().expect(k);
        println!(
            "the analyst's {key}: {:.2} +/- {:.2} mA + {:.5} +/- {:.5} mA per rad/s, {span}",
            n("coulomb_ma"),
            n("coulomb_se_ma"),
            n("viscous_ma_s_per_rad"),
            n("viscous_se")
        );
        close(
            &format!("{key}: the offset against the analyst's"),
            got.coulomb_ma,
            n("coulomb_ma"),
            n("coulomb_se_ma"),
        );
        close(
            &format!("{key}: the slope against the analyst's"),
            got.viscous_ma_s_per_rad,
            n("viscous_ma_s_per_rad"),
            n("viscous_se"),
        );
    }
    let checks = cruise::checks(&points, Some(&shipped), r.ceiling_cps);
    for c in &checks {
        println!("  {}", c.said());
    }
    assert!(
        checks.iter().all(|c| c.verdict == Verdict::Pass),
        "{checks:?}"
    );
}
