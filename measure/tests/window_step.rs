//! The window step's rows to its judge, on recorded onsets: the plan's 24
//! onsets of session A (testdata/window), each recorded segment taken as
//! one of the step's shots, gated as one session, fitted and judged; the
//! fixture's g comes back.

mod common;

use common::{path, sweep};
use osc_measure::capture::meta::gunzip;
use osc_measure::fit::window::{Method, Scales, Settle, on_rail_line};
use osc_measure::ntc::{self, Parts};
use osc_measure::steps::Verdict;
use osc_measure::steps::rest::Sense;
use osc_measure::steps::window::{self, SET, Shot, judge};
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

/// The dev board's sense chain as its CALIB holds it.
fn sense() -> Sense {
    let ntc = Parts {
        pullup_ohm: 10_000.0,
        r25_ohm: 10_000.0,
        beta: 3950.0,
    };
    Sense {
        shunt_r_mohm: 60.0,
        gain_milli: 14_884.0,
        vdd_mv: 3300.0,
        vmotor_div_top: 6400.0,
        vmotor_div_bot: 1600.0,
        vbus_div_top: 6400.0,
        vbus_div_bot: 1600.0,
        rail_drop_mv: 250.0,
        bias_nom: 773.0,
        tick_hz: 20_000.0,
        words: ntc::design(&ntc, &ntc::STAGE_1).expect("designs"),
        ntc,
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
        assert!(
            matches!(c.verdict, Verdict::Pass | Verdict::NotJudged),
            "{}",
            c.said()
        );
    }

    // the step's own scales: the nominal sense block, the board's terminal
    // offset, lags and crest read as the servo publishes them; the
    // fixture's differ in the sense scales and the lags on the captures'
    // rail, which stage 1 measured, and in the two terms the words round
    let board = common::board();
    let nominal = window::scales(&sense(), &board, s.pwm_arr, s.v_window_min_ticks)
        .expect("the board reads onsets");
    assert_eq!(
        Scales {
            counts_per_a: s.counts_per_a,
            terminal_counts_per_v: s.terminal_counts_per_v,
            lag_on_ticks: s.lag_on_ticks,
            lag_off_ticks: s.lag_off_ticks,
            terminal_offset_counts: s.terminal_offset_counts,
            r_brake_ohm: s.r_brake_ohm,
            ..nominal.clone()
        },
        s,
        "the step's other terms are the fixture's"
    );
    assert!((nominal.terminal_offset_counts - s.terminal_offset_counts).abs() <= 1.0 / 32.0);
    assert!((nominal.r_brake_ohm - s.r_brake_ohm).abs() <= 0.5e-3);
    let mut w = common::board_record().registers;
    w.window_scale_q15 = 0;
    let bare = osc_measure::Board::decode(&w).expect("decodes");
    let e = window::scales(&sense(), &bare, s.pwm_arr, s.v_window_min_ticks).expect_err("refused");
    assert!(e.0.contains("window_scale_q15 reads 0"), "{}", e.0);
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

/// The recorded onsets refit with the terminal offset and the brake loop
/// as the servo publishes them: g and its share over the mean move under
/// 1e-4 of the stage 1 floats' fit.
#[test]
fn the_window_onsets_refit_through_the_published_words() {
    let src = json("testdata/window/sources.json");
    let exp = json("testdata/window/expected.json");
    let shots = shots(&src);
    let s = stage1(&src, &exp);
    let c = common::board().characterization.expect("stage 1");
    let p = Scales {
        terminal_offset_counts: c.terminal_offset_counts,
        r_brake_ohm: c.window_read.r_brake_ohm,
        ..s.clone()
    };
    let fit = |s: &Scales| {
        let j = judge(&shots, s, &Method::NOTEBOOK, 3725);
        let w = j.window.expect("fits");
        (w.g_s, w.g_over_mean_s)
    };
    let (was, now) = (fit(&s), fit(&p));
    println!(
        "g {:.6} -> {:.6} mS ({:+.2e}), over the mean {:.6} -> {:.6} mS ({:+.2e})",
        was.0 * 1e3,
        now.0 * 1e3,
        now.0 / was.0 - 1.0,
        was.1 * 1e3,
        now.1 * 1e3,
        now.1 / was.1 - 1.0
    );
    assert!((now.0 / was.0 - 1.0).abs() < 1e-4, "g");
    assert!((now.1 / was.1 - 1.0).abs() < 1e-4, "g over the mean");
}
