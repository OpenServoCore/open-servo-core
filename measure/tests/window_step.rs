//! The window step's rows to its judge, on recorded onsets: the plan's 24
//! onsets of session A (testdata/window), each recorded segment taken as
//! one of the step's shots, gated as one session, fitted and judged; the
//! fixture's g comes back.

mod common;

use common::{path, sweep};
use osc_measure::capture::meta::gunzip;
use osc_measure::fit::window::{Method, Scales, Settle, on_rail_line};
use osc_measure::steps::Verdict;
use osc_measure::steps::window::{DIRECTIONS, SET, Shot, judge};
use serde_json::Value;

fn json(rel: &str) -> Value {
    serde_json::from_str(&common::text(rel)).expect(rel)
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("{v} is not a number"))
}

/// The recorded segments in record order, one shot each.
fn shots(src: &Value) -> Vec<Shot> {
    let mut out = Vec::new();
    for c in src["caps"].as_object().expect("caps").values() {
        let gz = std::fs::read(path(&format!(
            "testdata/window/{}",
            c["file"].as_str().expect("file")
        )))
        .expect("fixture");
        let text = String::from_utf8(gunzip(&gz).expect("gzip")).expect("utf8");
        for (_, duty, rows) in sweep(&text) {
            out.push(Shot {
                n: out.len() + 1,
                set: 0,
                duty_q15: duty,
                start: f64::NAN,
                on_s: f64::NAN,
                rows,
            });
        }
    }
    out
}

/// The fixture's stage 1 scales at the captures' rail, as the parity test
/// builds them.
fn stage1(src: &Value, exp: &Value) -> Scales {
    let s = &src["scales"];
    let rail = f(&exp["sub"]["rail_v"]);
    let line = |key: &str| {
        let p = |i: usize| (f(&s["rail_line"][i]["v"]), f(&s["rail_line"][i][key]));
        on_rail_line(rail, p(0), p(1))
    };
    Scales {
        counts_per_a: f(&s["counts_per_a"]),
        terminal_counts_per_v: f(&s["terminal_counts_per_v"]),
        terminal_offset_counts: f(&s["terminal_offset_counts"]),
        tick_hz: 20_000.0,
        pwm_arr: 1200,
        sample_ticks: f(&s["sample_ticks"]),
        lag_on_ticks: line("tap_on"),
        lag_off_ticks: line("off"),
        r_brake_ohm: f(&s["r_brake_ohm"]),
        v_window_min_ticks: f(&s["v_window_min_ticks"]) as u16,
        settle: Settle {
            start_ticks: f(&s["settle_start_ticks"]) as u16,
            q15: s["settle_q15"]
                .as_array()
                .expect("q15")
                .iter()
                .map(|v| f(v) as u16)
                .collect(),
        },
    }
}

fn close(what: &str, got: f64, want: f64, rel: f64) {
    assert!(
        (got - want).abs() <= rel * want.abs(),
        "{what}: {got:e} against {want:e}"
    );
}

#[test]
fn the_steps_judge_returns_the_fixtures_g_from_the_recorded_onsets() {
    let src = json("testdata/window/sources.json");
    let exp = json("testdata/window/expected.json");
    let shots = shots(&src);
    assert_eq!(shots.len(), SET);
    let s = stage1(&src, &exp);
    let j = judge(&shots, &s, &Method::NOTEBOOK, 3725);
    let w = j.window.as_ref().expect("fits");
    let e = &exp["sub"];
    println!(
        "replay: {} of {} gated, g {:.4} mS (fixture {:.4}), over the mean {:.4} mS (fixture {:.4}), L {:.3} mH, R {:.2} ohm",
        j.onsets.iter().filter(|o| o.ok).count(),
        shots.len(),
        w.g_s * 1e3,
        f(&e["g_s"]) * 1e3,
        w.g_over_mean_s * 1e3,
        f(&e["g_over_mean_s"]) * 1e3,
        w.l_h * 1e3,
        w.r_ohm
    );
    assert!(j.aside.is_empty(), "{:?}", j.aside);
    close("g", w.g_s, f(&e["g_s"]), 1e-9);
    close(
        "g over the mean",
        w.g_over_mean_s,
        f(&e["g_over_mean_s"]),
        1e-9,
    );
    close("L", w.l_h, f(&e["l_h"]), 1e-12);
    close("R", w.r_ohm, f(&e["r_ohm"]), 1e-12);
    for c in &j.checks {
        println!("  {}", c.said());
        let warned = c.name == DIRECTIONS && c.verdict == Verdict::Warn;
        assert!(c.verdict == Verdict::Pass || warned, "{}", c.said());
    }

    // the step's own scales: the nominal sense block and the board's lags
    let board: osc_measure::Board = common::load("boards/osc-dev-v006.json");
    let (edges, read) = (
        board.burst_edges.expect("edges"),
        board.window_read.expect("window_read"),
    );
    let nominal = Scales {
        counts_per_a: 14.884 * 0.060 / (3.3 / 4096.0),
        terminal_counts_per_v: 4096.0 / 3.3 * 1600.0 / 8000.0,
        terminal_offset_counts: 0.0,
        lag_on_ticks: edges.tap_on_ticks,
        lag_off_ticks: edges.off_ticks,
        sample_ticks: read.sample_ticks,
        r_brake_ohm: read.r_brake_ohm,
        settle: Settle {
            start_ticks: read.settle_start_ticks,
            q15: read.settle_q15.clone(),
        },
        ..s.clone()
    };
    assert_eq!(
        nominal.settle, s.settle,
        "the board's settle gain is the fixture's"
    );
    let n = judge(&shots, &nominal, &Method::NOTEBOOK, 3725);
    let wn = n.window.as_ref().expect("fits");
    println!(
        "on the step's nominal scales: g {:.4} mS, over the mean {:.4} mS, field {}",
        wn.g_s * 1e3,
        wn.g_over_mean_s * 1e3,
        osc_measure::fit::window::field_q016(wn.g_over_mean_s, &nominal)
    );
    for c in &n.checks {
        println!("  {}", c.said());
    }
}
