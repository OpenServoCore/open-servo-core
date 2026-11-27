//! The stops step's seated read on recorded bursts: the record's seated
//! set, the reverse groups at the low stop of bench capture B, through the
//! period-pair fit on stage 1's sense scales and the board's edge lags,
//! window scale and terminal offset; a stage 3 stop burst as the
//! identification notebook reads it; and a stops step's own bursts whose
//! high stop read bridged, replayed through the bridged rule and the
//! step's judge.

mod common;

use std::path::{Path, PathBuf};

use common::path;
use osc_measure::Board;
use osc_measure::capture::burst;
use osc_measure::fit::rise::{self, Lags, Scales};
use osc_measure::fit::seated::{self, SeatedRead, Shot};
use osc_measure::steps::Verdict;
use osc_measure::steps::polarity::Stop;
use osc_measure::steps::shunt::Shunted;
use osc_measure::steps::stops::{self, SeatShot};
use serde_json::Value;

/// The seated set of record: B's three seatings, the groups that press
/// into the stop, and its pooled median, ohm.
const SEATS: [&str; 3] = ["seat-1", "seat-2", "seat-3"];
const GROUPS: [(&str, usize); 3] = [("g3-rev-40", 1), ("g4-rev-25", 0), ("g5-rev-25-rested", 0)];
const R_RECORD_OHM: f64 = 4.2174;

/// The stage 3 identification notebook's stop seat, burst 1 of its 40 %
/// forward group, as its pass reads it (stages/3-servo/identification
/// cell 43, printed to the milliohm).
const STOP_40_R_OHM: f64 = 4.341;

fn board() -> Board {
    common::load("boards/osc-dev-v006.json")
}

fn sources() -> Value {
    serde_json::from_str(&common::text("testdata/bursts/sources.json")).expect("sources")
}

/// The scales the step reads its bursts through, on stage 1's measured
/// sense scales in place of the nominal ones the servo reports.
fn stage1() -> Scales {
    let s = sources();
    let v = |k: &str| s["scales"][k]["value"].as_f64().expect(k);
    stops::seated_scales(&board(), v("counts_per_a"), v("terminal_counts_per_v"))
        .expect("the board reads a seated burst")
}

/// R of one burst through `s`, the edge term left out as the seated read
/// leaves it.
fn r_of(cap: &burst::Capture, s: &Scales) -> f64 {
    let t = rise::periods(cap, seated::wiring_sign(cap), s, 0.0).expect("periods");
    rise::fit(&t, rise::pwm_period_s(cap.meta.pwm_arr), s.window_scale)
        .expect("fits")
        .r_ohm
}

/// The same scales without the two board terms.
fn bare(s: &Scales) -> Scales {
    Scales {
        terminal_offset_counts: 0.0,
        window_scale: 1.0,
        ..*s
    }
}

#[test]
fn the_board_carries_stage_1s_burst_terms() {
    let s = sources();
    let v = |k: &str| s["scales"][k]["value"].as_f64().expect(k);
    let lag = |k: &str| s["scales"]["lags"][k]["value"].as_f64().expect(k);
    let b = board();
    assert_eq!(b.window_scale, Some(v("window_scale")));
    assert_eq!(b.terminal_offset_counts, Some(v("terminal_offset_counts")));
    let sc = stage1();
    assert_eq!(sc.window_scale, v("window_scale"));
    assert_eq!(sc.terminal_offset_counts, v("terminal_offset_counts"));
    assert_eq!(
        sc.lags,
        Lags {
            on: lag("on"),
            tap_on: lag("tap_on"),
            off: lag("off"),
            settle: lag("settle"),
        }
    );
    for key in ["window_scale", "terminal_offset_counts"] {
        let mut v: Value = common::load("boards/osc-dev-v006.json");
        v.as_object_mut().expect("board").remove(key);
        let b: Board = serde_json::from_value(v).expect("board");
        let e = stops::seated_scales(&b, 1135.0, 248.0).expect_err(key);
        assert!(e.0.contains(key), "{key}: {}", e.0);
    }
}

#[test]
fn a_stage_3_stop_burst_reads_the_identification_notebooks_r() {
    let name = "stop-g2-fwd-40-burst-1.csv";
    let cap = burst::from_csv(&common::text(&format!("testdata/bursts/{name}"))).expect(name);
    let s = stage1();
    let (r, was) = (r_of(&cap, &s), r_of(&cap, &bare(&s)));
    println!(
        "{name}: R {r:.5} ohm (notebook {STOP_40_R_OHM}), {was:.5} without the board terms ({:+.2}%)",
        100.0 * (r / was - 1.0)
    );
    assert!((r - STOP_40_R_OHM).abs() < 5e-4, "R {r:.5}");
}

/// Opt-in, OSC_NOTEBOOKS names a notebooks checkout: the identification
/// notebook's seated pass, its stop seat's groups and bursts as its result
/// names them, reads its burst R through the step's scales.
#[test]
fn the_identification_pass_reads_the_notebooks_burst_r() {
    let Some(nb) = std::env::var_os("OSC_NOTEBOOKS") else {
        println!("OSC_NOTEBOOKS unset: skipped");
        return;
    };
    let nb = PathBuf::from(nb);
    let res: Value = serde_json::from_str(
        &std::fs::read_to_string(
            nb.join("stages/3-servo/identification/results/mg90-d__dev-v006-2A-D.json"),
        )
        .expect("the identification result"),
    )
    .expect("json");
    let want = res["r_burst_ohm"]["value"].as_f64().expect("r_burst_ohm");
    let pass = &res["pass_bursts"];
    let ks: Vec<usize> = pass["bursts"]
        .as_array()
        .expect("bursts")
        .iter()
        .map(|k| k.as_u64().expect("k") as usize)
        .collect();
    let dir = nb.join("telemetry/mg90-d__dev-v006-2A-D/inductance/stop-6");
    let groups = pass["groups"].as_array().expect("groups");
    let mut shots = Vec::new();
    for (g, name) in groups.iter().enumerate() {
        let name = name.as_str().expect("group");
        for k in 0..=*ks.iter().max().expect("a burst") {
            let p = dir.join(name).join(format!("burst-{k}.csv"));
            let text =
                std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
            shots.push(Shot {
                group: g,
                stop: 0,
                level: name.ends_with("-40") as usize,
                ntc_c: 25.0,
                cap: burst::from_csv(&text).unwrap_or_else(|e| panic!("{}: {e}", p.display())),
            });
        }
    }
    let s = stage1();
    let mean = |s: &Scales| {
        let r = seated::read(&shots, s);
        let x: Vec<f64> = r
            .bursts
            .iter()
            .filter(|b| ks.contains(&b.k))
            .map(|b| b.fit.as_ref().expect("a fit").r_ohm)
            .collect();
        assert_eq!(x.len(), groups.len() * ks.len());
        x.iter().sum::<f64>() / x.len() as f64
    };
    let (r, was) = (mean(&s), mean(&bare(&s)));
    println!(
        "the pass: R {r:.4} ohm (notebook {want:.4}), {was:.4} without the board terms ({:+.2}%)",
        100.0 * (r / was - 1.0)
    );
    assert!((r - want).abs() < 5e-5, "R {r:.6} against {want:.6}");
}

/// Every burst of the seated set under `root`, a group a folder.
fn shots(root: &Path) -> Vec<Shot> {
    let mut out = Vec::new();
    for (s, seat) in SEATS.iter().enumerate() {
        for (g, (name, level)) in GROUPS.iter().enumerate() {
            for k in 0.. {
                let p = root.join(seat).join(name).join(format!("burst-{k}.csv"));
                let Ok(text) = std::fs::read_to_string(&p) else {
                    break;
                };
                out.push(Shot {
                    group: 3 * s + g,
                    stop: 0,
                    level: *level,
                    ntc_c: 25.0,
                    cap: burst::from_csv(&text).unwrap_or_else(|e| panic!("{}: {e}", p.display())),
                });
            }
        }
    }
    out
}

fn pooled(root: &Path) -> f64 {
    let all = shots(root);
    assert_eq!(all.len(), 45, "five bursts a group");
    let mut r = seated::read(&all, &stage1());
    let kept: Vec<usize> = r.kept().map(|b| 5 * b.group + b.k).collect();
    let before = r.r(|_| true).expect("a pool");
    r.set_aside_bridged();
    assert_eq!(r.bridged().count(), 0, "a clean run keeps every burst");
    assert_eq!(
        kept,
        r.kept().map(|b| 5 * b.group + b.k).collect::<Vec<_>>()
    );
    assert_eq!(Some(before), r.r(|_| true));
    for b in &r.bursts {
        println!(
            "group {} burst {}: {} R {:.4} ohm, first window {:.4} A ({:+.1}%){}",
            b.group,
            b.k,
            b.step_q15,
            b.fit.as_ref().map_or(f64::NAN, |f| f.r_ohm),
            b.i_first_a,
            100.0 * b.move_share,
            b.aside
                .as_ref()
                .map_or(String::new(), |a| format!(", set aside: {}", a.said()))
        );
    }
    assert_eq!(
        r.kept().count(),
        36,
        "the first of each group set aside, nothing else"
    );
    let p = r.r(|_| true).expect("a pool");
    let l = r.l(|_| true).expect("a pool");
    println!(
        "pooled R {:.4} ohm (record {R_RECORD_OHM}), sd {:.2}%, within groups {:.2}%; L {:.4} mH",
        p.median,
        100.0 * p.sd_share,
        100.0 * r.r_sd_within_groups().expect("groups"),
        l.median * 1e3
    );
    p.median
}

#[test]
fn the_seated_pool_matches_the_record_on_its_bursts() {
    let r = pooled(&path("testdata/seated"));
    assert!(
        (r / R_RECORD_OHM - 1.0).abs() < 0.01,
        "pooled R {r:.4} against the record's {R_RECORD_OHM}"
    );
}

/// The same on the bringup checkout's own capture folder, when
/// `OSC_BRINGUP` names the checkout: the committed bursts are its files.
#[test]
fn the_seated_pool_matches_the_record_on_the_full_capture() {
    let Some(root) = std::env::var_os("OSC_BRINGUP") else {
        println!("OSC_BRINGUP unset: skipped");
        return;
    };
    let full = PathBuf::from(root).join("captures/ident-slice2/B/seated");
    for seat in SEATS {
        for (name, _) in GROUPS {
            for k in 0..5 {
                let rel = format!("{seat}/{name}/burst-{k}.csv");
                assert_eq!(
                    std::fs::read(full.join(&rel)).expect(&rel),
                    std::fs::read(path("testdata/seated").join(&rel)).expect(&rel),
                    "{rel} is the recorded file"
                );
            }
        }
    }
    let r = pooled(&full);
    assert!((r / R_RECORD_OHM - 1.0).abs() < 0.01, "pooled R {r:.4}");
}

/// A stops step's own run, its bursts as `burst-K.csv` beside its
/// `stops.json`: each burst's shot and seat as the step took it, and the
/// scales it read them on.
fn step_run(root: &Path) -> (Value, Vec<Shot>, Vec<SeatShot>, Scales) {
    let res: Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("stops.json")).expect("result"))
            .expect("json");
    let sc = &res["seated"]["scales"];
    let n = |v: &Value| v.as_f64().expect("a number");
    let scales = Scales {
        counts_per_a: n(&sc["counts_per_a"]),
        terminal_counts_per_v: n(&sc["terminal_counts_per_v"]),
        terminal_offset_counts: n(&sc["terminal_offset_counts"]),
        window_scale: n(&sc["window_scale"]),
        lags: Lags {
            on: n(&sc["lags_ticks"]["on"]),
            tap_on: n(&sc["lags_ticks"]["tap_on"]),
            off: n(&sc["lags_ticks"]["off"]),
            settle: n(&sc["lags_ticks"]["settle"]),
        },
    };
    let (mut shots, mut seats) = (Vec::new(), Vec::new());
    for (k, b) in res["seated"]["bursts"]
        .as_array()
        .expect("bursts")
        .iter()
        .enumerate()
    {
        let text = std::fs::read_to_string(root.join(format!("burst-{k}.csv"))).expect("burst");
        let cap = burst::from_csv(&text).expect("a burst");
        let seat = b["seat"].as_u64().expect("seat") as usize;
        let level = stops::LEVELS
            .iter()
            .position(|l| *l == n(&b["level_share"]))
            .expect("level");
        let stop = if b["stop"] == "low" {
            Stop::Low
        } else {
            Stop::High
        };
        shots.push(Shot {
            group: (seat - 1) * stops::LEVELS.len() + level,
            stop: (stop == Stop::High) as usize,
            level,
            ntc_c: n(&b["ntc_c"]),
            cap: cap.clone(),
        });
        seats.push(SeatShot {
            seat,
            stop,
            level,
            ntc_c: n(&b["ntc_c"]),
            permit: b["permit"] == true,
            shunted: Shunted {
                cap,
                after_counts: 0.0,
                times: (0.0, 0.0),
            },
            after_a: n(&b["after_a"]),
        });
    }
    (res, shots, seats, scales)
}

/// The stops' and the pooled R, ohm.
fn rs(r: &SeatedRead) -> [f64; 3] {
    [
        r.r(|b| b.stop == 0).expect("low").median,
        r.r(|b| b.stop == 1).expect("high").median,
        r.r(|_| true).expect("pooled").median,
    ]
}

fn verdict(checks: &[osc_measure::steps::Check], name: &str) -> (Verdict, String) {
    let c = checks
        .iter()
        .find(|c| c.name == name)
        .unwrap_or_else(|| panic!("{name}: {checks:?}"));
    (c.verdict, c.value.clone())
}

/// The run the two stops' R once refused: its bursts replay to the R the
/// step printed, and the bridged rule sets aside the five 40 % bursts at
/// the high stop that read 2.90 to 3.52 ohm and nothing at the low stop.
#[test]
fn the_bridged_rule_replays_a_run_refused_by_the_stops_r() {
    let root = path("testdata/seated/bridged-high");
    let src: Value =
        serde_json::from_str(&common::text("testdata/seated/bridged-high/sources.json"))
            .expect("sources");
    for (name, sha) in src["sha1"].as_object().expect("sha1") {
        assert_eq!(
            osc_measure::set::file_sha1(&std::fs::read(root.join(name)).expect("file")),
            sha.as_str().expect("sha1"),
            "{name} is the recorded file"
        );
    }
    let (res, shots, seats, scales) = step_run(&root);
    assert_eq!(shots.len(), 32);
    let mut r = seated::read(&shots, &scales);
    for (b, x) in r
        .bursts
        .iter()
        .zip(res["seated"]["bursts"].as_array().expect("bursts"))
    {
        let f = b.fit.as_ref().expect("a fit");
        assert!((f.r_ohm - x["r_ohm"].as_f64().expect("r")).abs() < 1e-9);
        assert_eq!(b.kept(), x["kept"] == true);
    }
    let was = rs(&r);
    let judged = stops::judge_r(&r, &seats);
    assert!(
        judged.iter().all(|c| c.verdict != Verdict::Fail),
        "the stops' R apart is reported: {judged:?}"
    );
    r.set_aside_bridged();
    let now = rs(&r);
    println!("burst seat stop level k R_ohm kept why_not");
    for (k, (b, s)) in r.bursts.iter().zip(&seats).enumerate() {
        println!(
            "{k:2} {} {:4} {:.0}% {} {:.4} {} {}",
            s.seat,
            s.stop.word(),
            100.0 * stops::LEVELS[b.level],
            b.k,
            b.fit.as_ref().map_or(f64::NAN, |f| f.r_ohm),
            b.kept(),
            b.aside.as_ref().map_or(String::new(), |a| a.said())
        );
    }
    println!(
        "R low {:.4} -> {:.4}, high {:.4} -> {:.4}, pooled {:.4} -> {:.4} ohm; kept {}",
        was[0],
        now[0],
        was[1],
        now[1],
        was[2],
        now[2],
        r.kept().count()
    );
    let cut: Vec<usize> = r
        .bursts
        .iter()
        .enumerate()
        .filter(|(_, b)| matches!(b.aside, Some(seated::Aside::Bridged { .. })))
        .map(|(k, _)| k)
        .collect();
    assert_eq!(
        cut,
        [14, 15, 29, 30, 31],
        "the 40 % bursts at the high stop that read low"
    );
    assert_eq!(now[0], was[0], "nothing set aside at the low stop");
    assert!((now[1] - 4.0295).abs() < 5e-4, "high {:.4}", now[1]);
    let judged = stops::judge_r(&r, &seats);
    for c in &judged {
        println!("  {}", c.said());
    }
    assert!(
        judged.iter().all(|c| c.verdict != Verdict::Fail),
        "{judged:?}"
    );
    let (v, said) = verdict(&judged, "kept bursts in the fit");
    assert!(said.contains("5 more set aside as bridged"), "{said}");
    assert_eq!(v, Verdict::NotJudged, "{said}");
}

/// The same rule on a clean stops run of the bench servo, its own
/// bursts, when `OSC_BRINGUP` names the checkout: nothing is set aside.
#[test]
fn the_bridged_rule_keeps_a_clean_step_run_whole() {
    let Some(root) = std::env::var_os("OSC_BRINGUP") else {
        println!("OSC_BRINGUP unset: skipped");
        return;
    };
    let run = PathBuf::from(root)
        .join("captures/ident-onsets/run2/runs/00000000ffffffff76e5bc220ef6abcd/20261123T115428Z");
    let dir = std::env::temp_dir().join(format!("osc-clean-run-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("dir");
    std::fs::copy(run.join("results/stops.json"), dir.join("stops.json")).expect("result");
    for k in 0..24 {
        let n = format!("burst-{k}.csv");
        std::fs::copy(run.join("captures/stops").join(&n), dir.join(&n)).expect(&n);
    }
    let (_, shots, _, scales) = step_run(&dir);
    let mut r = seated::read(&shots, &scales);
    let (kept, was) = (r.kept().count(), r.r(|_| true));
    r.set_aside_bridged();
    println!("{kept} kept, pooled {:?}", was.map(|s| s.median));
    assert_eq!(r.bridged().count(), 0, "{:?}", r.bursts);
    assert_eq!(r.r(|_| true), was);
    let _ = std::fs::remove_dir_all(&dir);
}
