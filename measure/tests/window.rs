//! The window-term fit against the stage 3 window-term notebook: the 24
//! onsets of the plan's check, trimmed into testdata/window, and every
//! recorded onset from a notebooks checkout named by `OSC_NOTEBOOKS`
//! (skipped when it is unset).

mod common;

use std::path::Path;

use common::{path, sweep};
use osc_measure::capture::meta::gunzip;
use osc_measure::capture::rows::TelFrame;
use osc_measure::fit::window::{
    self, Method, Onset, Scales, Segment, Settle, Window, acceptance, field_q016, on_rail_line,
};
use serde_json::Value;

/// The plan's check: 24 onsets return the record's term within 5 %.
const RECORD_G_S: f64 = 3.93e-3;

fn json(rel: &str) -> Value {
    serde_json::from_str(&common::text(rel)).expect(rel)
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("{v} is not a number"))
}

/// The stage 1 scales of sources.json, the edge lags at `rail_v`.
fn scales(src: &Value, tick_hz: f64, pwm_arr: u16, rail_v: f64) -> Scales {
    let s = &src["scales"];
    let line = |key: &str| {
        let p = |i: usize| (f(&s["rail_line"][i]["v"]), f(&s["rail_line"][i][key]));
        on_rail_line(rail_v, p(0), p(1))
    };
    Scales {
        counts_per_a: f(&s["counts_per_a"]),
        terminal_counts_per_v: f(&s["terminal_counts_per_v"]),
        terminal_offset_counts: f(&s["terminal_offset_counts"]),
        tick_hz,
        pwm_arr,
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

fn segments(sw: &[(u32, i16, Vec<TelFrame>)]) -> Vec<Segment<'_>> {
    sw.iter()
        .map(|(seg, cmd, rows)| Segment {
            seg: *seg,
            cmd_duty_q15: *cmd,
            rows,
        })
        .collect()
}

fn close(what: &str, got: f64, want: f64, rel: f64) {
    let tol = rel * want.abs().max(1e-300);
    assert!(
        (got - want).abs() <= tol,
        "{what}: {got:e} against the notebook's {want:e}"
    );
}

/// The gate's readings of `ons` against the notebook's, by capture name
/// and segment.
fn same_gates(ons: &[Onset], names: &[String], exp: &Value) {
    for o in ons {
        let e = exp["gates"]
            .as_array()
            .expect("gates")
            .iter()
            .find(|g| g["cap"] == names[o.cap].as_str() && g["seg"] == o.seg)
            .unwrap_or_else(|| panic!("{} {} not in the notebook", names[o.cap], o.seg));
        let what = format!("{} segment {}", names[o.cap], o.seg);
        assert_eq!(o.sgn as i64, e["sgn"].as_i64().expect("sgn"), "{what}");
        assert_eq!(
            o.pre_rows as u64,
            e["pre_rows"].as_u64().expect("pre"),
            "{what}"
        );
        assert_eq!(o.pre_vd_max, e["pre_vd_max"].as_f64(), "{what}");
        assert_eq!(o.moved, e["moved"].as_f64(), "{what}");
        assert_eq!(
            o.same_sign,
            e["same_sign"].as_bool().expect("sign"),
            "{what}"
        );
        let seq: Vec<u16> = e["seq"]
            .as_array()
            .expect("seq")
            .iter()
            .map(|v| f(v) as u16)
            .collect();
        assert_eq!(o.seq, seq, "{what}");
        assert_eq!(o.ok, e["ok"].as_bool().expect("ok"), "{what}");
    }
}

/// The fit against the notebook's on the same onsets; `boot_rel` bounds
/// the bootstrap sd, which draws on another generator.
fn same_fit(w: &Window, s: &Scales, names: &[String], exp: &Value, r_q12: u16, boot_rel: f64) {
    assert_eq!(w.profiles.len() as u64, exp["n"].as_u64().expect("n"));
    for (p, e) in w
        .profiles
        .iter()
        .zip(exp["onsets"].as_array().expect("onsets"))
    {
        let what = format!("{} segment {}", names[p.cap], p.seg);
        assert_eq!(
            (names[p.cap].as_str(), p.seg as u64, p.sgn as i64),
            (
                e["cap"].as_str().expect("cap"),
                e["seg"].as_u64().expect("seg"),
                e["sgn"].as_i64().expect("sgn")
            ),
            "onsets in the notebook's order"
        );
        for (k, (y, ye)) in p
            .y_counts
            .iter()
            .zip(e["y_counts"].as_array().expect("y"))
            .enumerate()
        {
            assert!((y - f(ye)).abs() <= 1e-9, "{what} period {k}: {y} counts");
        }
        close(&format!("{what} V_on"), p.v_on_v, f(&e["v_on_v"]), 1e-12);
        close(
            &format!("{what} g pinned"),
            p.g_pinned_s,
            f(&e["g_pinned_s"]),
            1e-9,
        );
    }
    let h: Vec<u16> = exp["h"]
        .as_array()
        .expect("h")
        .iter()
        .map(|v| f(v) as u16)
        .collect();
    assert_eq!(w.h, h);
    close("L", w.l_h, f(&exp["l_h"]), 1e-12);
    close("R", w.r_ohm, f(&exp["r_ohm"]), 1e-12);
    close("g", w.g_s, f(&exp["g_s"]), 1e-9);
    close("rms", w.rms_a, f(&exp["rms_a"]), 1e-9);
    close("V_on", w.v_on_v, f(&exp["v_on_v"]), 1e-12);
    close("lateness", w.lateness_s, f(&exp["lateness_s"]), 1e-9);
    close(
        "g over the mean",
        w.g_over_mean_s,
        f(&exp["g_over_mean_s"]),
        1e-9,
    );
    close("L / R", w.tau_periods, f(&exp["tau_periods"]), 1e-9);
    assert_eq!(w.on_grid_edge, exp["edge"].as_bool().expect("edge"));
    for (key, got) in [("resid_a", &w.resid_a), ("winding_a", &w.winding_a)] {
        let want = exp[key].as_array().expect(key);
        assert_eq!(got.len(), want.len());
        for (k, (a, b)) in got.iter().zip(want).enumerate() {
            assert!(
                (a - f(b)).abs() <= 1e-12,
                "{key} period {k}: {a:e} against {b}"
            );
        }
    }
    for (name, d) in [("forward", w.forward), ("reverse", w.reverse)] {
        let d = d.expect(name);
        let e = &exp["split"][name];
        assert_eq!(d.n as u64, e["n"].as_u64().expect("n"), "{name}");
        close(&format!("{name} free"), d.g_free_s, f(&e["g_free_s"]), 1e-9);
        close(
            &format!("{name} pinned"),
            d.g_pinned_s,
            f(&e["g_pinned_s"]),
            1e-9,
        );
    }
    let sd = w.g_sd_s.expect("draws");
    close("bootstrap sd", sd, f(&exp["boot_sd_s"]), boot_rel);
    assert_eq!(field_q016(w.g_over_mean_s, s), f(&exp["field"]));
    let checks = acceptance(w, s, r_q12);
    let split = &exp["split"];
    let d = |k: &str| f(&split["forward"][k]) - f(&split["reverse"][k]);
    let dirs = d("g_free_s").abs().max(d("g_pinned_s").abs()) / f(&exp["g_s"]) * 100.0;
    let want = [
        f(&exp["rms_a"]) * 1e3,
        f(&exp["tau_periods"]),
        dirs,
        f(&exp["g_s"]) * 1e3,
        f(&exp["fence"]),
        exp["edge"].as_bool().expect("edge") as u8 as f64,
    ];
    assert_eq!(checks.len(), want.len());
    for (c, v) in checks.iter().zip(want) {
        close(c.name, c.value.expect(c.name), v, 1e-9);
        println!(
            "{}: {:.4} {}: {}",
            c.name,
            c.value.expect(c.name),
            c.unit,
            if c.pass { "pass" } else { "FAIL" }
        );
    }
}

fn report(what: &str, w: &Window, exp: &Value) {
    println!(
        "{what}: {} onsets, g {:.4} mS (notebook {:.4}), + lateness {:.4} = {:.4} mS (notebook {:.4}, \
         record {:.4}), sd {:.4} mS (notebook {:.4}, {} draws), L {:.2} mH, R {:.2} ohm, rms {:.3} mA",
        w.profiles.len(),
        w.g_s * 1e3,
        f(&exp["g_s"]) * 1e3,
        w.lateness_s * 1e3,
        w.g_over_mean_s * 1e3,
        f(&exp["g_over_mean_s"]) * 1e3,
        RECORD_G_S * 1e3,
        w.g_sd_s.unwrap_or(f64::NAN) * 1e3,
        f(&exp["boot_sd_s"]) * 1e3,
        w.draws,
        w.l_h * 1e3,
        w.r_ohm,
        w.rms_a * 1e3
    );
}

/// The 24 onsets of the plan's check: gated, fitted and judged as the
/// notebook does, and within 5 % of the record.
#[test]
fn the_window_fit_matches_the_notebook_on_the_plans_24_onsets() {
    let src = json("testdata/window/sources.json");
    let exp = json("testdata/window/expected.json");
    let caps = src["caps"].as_object().expect("caps");
    let names: Vec<String> = caps.keys().cloned().collect();
    let mut sweeps = Vec::new();
    for c in caps.values() {
        let gz = std::fs::read(path(&format!(
            "testdata/window/{}",
            c["file"].as_str().expect("file")
        )))
        .expect("fixture");
        sweeps.push(sweep(
            &String::from_utf8(gunzip(&gz).expect("gzip")).expect("utf8"),
        ));
    }
    let first = caps.values().next().expect("a capture");
    let vpv = f(&src["scales"]["terminal_counts_per_v"]);
    let rail = caps
        .values()
        .map(|c| f(&c["vbus_counts"]) / vpv)
        .sum::<f64>()
        / caps.len() as f64;
    let s = scales(
        &src,
        f(&first["tick_hz"]),
        f(&first["pwm_period_ticks"]) as u16,
        rail,
    );
    let exp_sub = &exp["sub"];
    close("rail", rail, f(&exp_sub["rail_v"]), 1e-12);
    close("ON lag", s.lag_on_ticks, f(&exp_sub["lags"][0]), 1e-12);
    close("OFF lag", s.lag_off_ticks, f(&exp_sub["lags"][1]), 1e-12);
    let m = Method::NOTEBOOK;
    let mut ons: Vec<Onset> = Vec::new();
    for (i, sw) in sweeps.iter().enumerate() {
        ons.extend(window::onsets(i, &segments(sw), &m).expect("onsets"));
    }
    assert_eq!(ons.len(), 24);
    let ramp = window::gate(&mut ons, &m).expect("a ramp");
    assert_eq!(&ramp[..3], &[68, 73, 78]);
    same_gates(&ons, &names, &exp);
    assert!(ons.iter().all(|o| o.ok));
    let w = window::fit(&ons, &s, &m).expect("fits");
    report("the plan's 24", &w, exp_sub);
    let r_q12 = f(&first["r_q12"]) as u16;
    same_fit(&w, &s, &names, exp_sub, r_q12, 0.1);
    close("against the record", w.g_over_mean_s, RECORD_G_S, 0.05);
}

/// Every recorded onset of both sessions, gated by session and pooled, as
/// the notebook's result was fitted.
#[test]
fn the_window_fit_matches_the_notebook_on_every_recorded_onset() {
    let Some(root) = std::env::var_os("OSC_NOTEBOOKS") else {
        println!("OSC_NOTEBOOKS unset: the full parity is skipped");
        return;
    };
    let src = json("testdata/window/sources.json");
    let exp = json("testdata/window/expected.json");
    let data = Path::new(&root).join("telemetry/mg90-d__dev-v006-2A-D/window-term");
    let vpv = f(&src["scales"]["terminal_counts_per_v"]);
    let m = Method::NOTEBOOK;
    let (mut names, mut all, mut rails, mut metas) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for (session, folder) in [("A", "onsets-A"), ("brake", "onsets-A-brake")] {
        let mut ons = Vec::new();
        for k in 1..=3 {
            let dir = data.join(format!("{folder}/ol-{k}"));
            let text = std::fs::read_to_string(dir.join("sweep.csv")).expect("sweep.csv");
            let meta: Value = serde_json::from_str(
                &std::fs::read_to_string(dir.join("meta.json")).expect("meta"),
            )
            .expect("meta json");
            rails.push(f(&meta["vbus_counts"]) / vpv);
            let sw = sweep(&text);
            ons.extend(window::onsets(names.len(), &segments(&sw), &m).expect("onsets"));
            names.push(format!("{session}/ol-{k}"));
            metas.push(meta);
        }
        window::gate(&mut ons, &m).expect("a ramp");
        all.extend(ons);
    }
    assert_eq!(all.len(), 108);
    same_gates(&all, &names, &exp);
    assert_eq!(all.iter().filter(|o| o.ok).count(), 105);
    let rail = rails.iter().sum::<f64>() / rails.len() as f64;
    let s = scales(
        &src,
        f(&metas[0]["tick_hz"]),
        f(&metas[0]["pwm_period_ticks"]) as u16,
        rail,
    );
    let w = window::fit(&all, &s, &m).expect("fits");
    report("every recorded onset", &w, &exp["full"]);
    same_fit(
        &w,
        &s,
        &names,
        &exp["full"],
        f(&metas[0]["drive"]["r_q12"]) as u16,
        0.1,
    );
    close(
        "against the record",
        w.g_over_mean_s,
        f(&exp["record"]),
        1e-9,
    );
}
