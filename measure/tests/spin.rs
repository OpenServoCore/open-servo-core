//! The spin fits against the notebooks: the ripple counters, Ke from the
//! ladder's coasts and J from the run-ups, on trimmed copies of recorded
//! captures (`testdata/spin`, written with their expected values by
//! `testdata/spin/gen.py` running the notebooks' own reductions). With
//! OSC_NOTEBOOKS naming a notebooks checkout, the same reductions run on
//! its whole datasets against `testdata/spin/full.json`.

use std::path::{Path, PathBuf};

use osc_measure::capture::meta::gunzip;
use osc_measure::capture::rows::{CSV_HEADER, TelFrame};
use osc_measure::fit::coast::{self, CoastKe, KePooled, Pooled};
use osc_measure::fit::median;
use osc_measure::fit::runup::{self, Inertia, Run, RunUp};
use osc_measure::ripple;
use serde_json::Value;

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata/spin")
}

fn json(name: &str) -> Value {
    let p = dir().join(name);
    serde_json::from_str(&std::fs::read_to_string(&p).expect("expected values")).expect("json")
}

/// One segment of a sweep: its number, commanded duty, direction, rows.
struct Seg {
    seg: i64,
    cmd: i64,
    dir: i8,
    rows: Vec<TelFrame>,
}

/// A sweep CSV, osc-lab's `sweep.csv` (gzipped when the name says so).
fn sweep(path: &Path) -> Vec<Seg> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let bytes = if path.extension().is_some_and(|e| e == "gz") {
        gunzip(&bytes).expect("gzip")
    } else {
        bytes
    };
    let text = String::from_utf8(bytes).expect("utf8");
    let mut lines = text.lines();
    assert_eq!(lines.next(), Some(CSV_HEADER), "{}", path.display());
    let mut out: Vec<Seg> = Vec::new();
    for line in lines {
        let c: Vec<&str> = line.split(',').collect();
        let n = |k: usize| -> Option<f64> {
            c.get(k)
                .filter(|s| !s.is_empty())
                .map(|s| s.parse::<f64>().expect("number"))
        };
        let u = |k: usize| n(k).map(|v| v as u16);
        let i = |k: usize| n(k).map(|v| v as i16);
        let f = TelFrame {
            tick: n(3).expect("tick") as u64,
            window_valid: n(4) == Some(1.0),
            pos: u(5),
            current: i(6),
            current_trough: u(7),
            duty_counts: i(8),
            vdiff: i(9),
            vbus: u(10),
            current_raw: u(11),
            vmotor_a: u(12),
            vmotor_b: u(13),
            vbus_raw: u(14),
            ntc_raw: u(15),
            vref_raw: u(16),
            ..TelFrame::default()
        };
        let (seg, cmd, d) = (
            n(0).expect("seg") as i64,
            n(1).expect("cmd") as i64,
            n(2).expect("dir") as i8,
        );
        match out.last_mut() {
            Some(s) if s.seg == seg => s.rows.push(f),
            _ => out.push(Seg {
                seg,
                cmd,
                dir: d,
                rows: vec![f],
            }),
        }
    }
    out
}

/// oscnb `coast.pairs`: each zero-command segment after the first with
/// the segment before it, `(direction, spin, coast)`.
fn pairs(segs: &[Seg]) -> Vec<(i8, &[TelFrame], &[TelFrame])> {
    segs.iter()
        .filter(|s| s.seg > 0 && s.cmd == 0)
        .map(|co| {
            let sp = segs
                .iter()
                .find(|s| s.seg == co.seg - 1)
                .expect("the spin before");
            (co.dir, sp.rows.as_slice(), co.rows.as_slice())
        })
        .collect()
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("not a number: {v}"))
}

fn u(v: &Value) -> usize {
    v.as_u64().unwrap_or_else(|| panic!("not a count: {v}")) as usize
}

/// Relative agreement, for numbers the two languages reach by the same
/// arithmetic in another order.
const REL: f64 = 1e-9;

#[track_caller]
fn close(got: f64, want: f64, rel: f64, what: &str) {
    let tol = rel * want.abs().max(1e-300);
    assert!(
        (got - want).abs() <= tol,
        "{what}: {got} against {want} (rel {:e})",
        (got / want - 1.0).abs()
    );
}

fn coast_scales(s: &Value) -> coast::Scales {
    coast::Scales {
        counts_per_v: f(&s["counts_per_v"]),
        per_rev: u(&s["per_rev"]),
        fs: f(&s["fs"]),
    }
}

fn runup_scales(s: &Value) -> runup::Scales {
    runup::Scales {
        counts_per_a: f(&s["counts_per_a"]),
        drive_offset_counts: f(&s["drive_offset_counts"]),
        counts_per_v: f(&s["counts_per_v"]),
        terminal_offset_counts: f(&s["terminal_offset_counts"]),
        pwm_arr: f(&s["pwm_arr"]),
        window_s: f(&s["window_s"]),
        r_ohm: f(&s["r_ohm"]),
        l_h: f(&s["l_h"]),
        ke: f(&s["ke"]),
        i_drag0_a: f(&s["i_drag0_a"]),
        per_rev: u(&s["per_rev"]),
        fs: f(&s["fs"]),
    }
}

fn opt(v: &Value) -> Option<f64> {
    v.as_f64()
}

fn check_coast(got: &CoastKe, want: &Value, at: &str) {
    assert_eq!(got.dir as i64, want["dir"].as_i64().expect("dir"), "{at}");
    assert_eq!(f64::from(got.level), f(&want["level"]), "{at} level");
    assert_eq!(got.cut, u(&want["cut"]), "{at} cut");
    close(
        got.e_first,
        f(&want["e_first"]),
        REL,
        &format!("{at} e_first"),
    );
    assert_eq!(
        got.turned,
        want["turned"].as_bool().expect("turned"),
        "{at} turned"
    );
    assert_eq!(got.open_rows, u(&want["open_rows"]), "{at} open rows");
    assert_eq!(got.f_seed, opt(&want["f_seed"]), "{at} seed");
    assert_eq!(
        got.stop,
        want["stop"].as_u64().map(|v| v as usize),
        "{at} stop"
    );
    match (got.spin_rev_s, opt(&want["spin_rev_s"])) {
        (Some(a), Some(b)) => close(a, b, REL, &format!("{at} spin rev/s")),
        (a, b) => assert_eq!(a, b, "{at} spin rev/s"),
    }
    let w = &want["window"];
    match got.window {
        None => assert!(w.is_null(), "{at}: no window, the notebook has one"),
        Some(g) => {
            assert_eq!(
                (g.i0, g.i1, g.cycles),
                (u(&w["i0"]), u(&w["i1"]), u(&w["cycles"])),
                "{at} window"
            );
            for (name, a) in [
                ("rev_s", g.rev_s),
                ("e_win", g.e_win),
                ("e_cut", g.e_cut),
                ("dedt", g.dedt),
                ("ke", g.ke),
                ("decel_frac", g.decel_frac),
            ] {
                close(a, f(&w[name]), REL, &format!("{at} {name}"));
            }
        }
    }
    assert_eq!(
        got.gated,
        want["gated"].as_bool().expect("gated"),
        "{at} gated"
    );
}

fn check_pooled(got: &Pooled, want: &Value, at: &str) {
    assert_eq!(got.n, u(&want["n"]), "{at} n");
    close(got.mean, f(&want["mean"]), REL, &format!("{at} mean"));
    match (got.sd, opt(&want["sd"])) {
        (Some(a), Some(b)) => close(a, b, 1e-7, &format!("{at} sd")),
        (a, b) => assert_eq!(a, b, "{at} sd"),
    }
}

fn check_ke_pooled(got: &KePooled, want: &Value) {
    check_pooled(&got.all, &want["all"], "Ke");
    for (d, p) in &got.by_dir {
        check_pooled(p, &want["by_dir"][d.to_string()], &format!("Ke {d:+}"));
    }
    assert_eq!(
        got.by_dir.len(),
        want["by_dir"].as_object().expect("dirs").len()
    );
    for (l, p) in &got.by_level {
        check_pooled(p, &want["by_level"][l.to_string()], &format!("Ke at {l}"));
    }
    assert_eq!(
        got.by_level.len(),
        want["by_level"].as_object().expect("levels").len()
    );
}

/// Every coast of one sweep through [`coast::coast_ke`].
fn ladder_coasts(segs: &[Seg], zero_e: f64, s: &coast::Scales) -> Vec<CoastKe> {
    pairs(segs)
        .into_iter()
        .map(|(d, sp, co)| coast::coast_ke(sp, co, d, zero_e, s).expect("a coast reads"))
        .collect()
}

fn check_ladder(
    files: &[Value],
    load: &dyn Fn(&Value) -> Vec<Seg>,
    s: &coast::Scales,
) -> Vec<CoastKe> {
    let mut all = Vec::new();
    for file in files {
        let got = ladder_coasts(&load(file), f(&file["zero_e"]), s);
        let want = file["coasts"].as_array().expect("coasts");
        assert_eq!(got.len(), want.len(), "{file}");
        for (k, (g, w)) in got.iter().zip(want).enumerate() {
            check_coast(g, w, &format!("{} coast {k}", file["from"]));
        }
        all.extend(got);
    }
    all
}

fn check_runups(want: &Value, reads: &[RunUp], got: &Inertia) {
    let runs = want["runs"].as_array().expect("runs");
    assert_eq!((reads.len(), got.runs.len()), (runs.len(), runs.len()));
    for (k, ((r, j), w)) in reads.iter().zip(&got.runs).zip(runs).enumerate() {
        let at = format!("run-up {k}");
        assert_eq!(r.dir as i64, w["dir"].as_i64().expect("dir"), "{at}");
        assert_eq!(
            (r.rows, r.tc.len(), r.added, r.removed, r.dropouts),
            (
                u(&w["rows"]),
                u(&w["crossings"]),
                u(&w["added"]),
                u(&w["removed"]),
                u(&w["dropouts"])
            ),
            "{at} count"
        );
        close(
            r.tc[0],
            f(&w["tc_first"]),
            REL,
            &format!("{at} first crossing"),
        );
        close(
            r.tc[r.tc.len() - 1],
            f(&w["tc_last"]),
            REL,
            &format!("{at} last crossing"),
        );
        assert_eq!(
            (r.windows.len(), r.windows_ok.iter().filter(|o| **o).count()),
            (u(&w["windows"]), u(&w["windows_ok_local"])),
            "{at} windows"
        );
        assert_eq!(
            (r.fits.len(), r.fits_ok.iter().filter(|o| **o).count()),
            (u(&w["fits"]), u(&w["fits_res_ok"])),
            "{at} fits"
        );
        assert_eq!(
            (r.coast.fits.len(), r.coast.stop, r.coast.open_rows),
            (u(&w["coast_fits"]), u(&w["stop"]), u(&w["coast_open_rows"])),
            "{at} coast"
        );
        close(r.coast.e0, f(&w["e0"]), REL, &format!("{at} E0"));
        assert_eq!(
            (j.fits_kept, j.windows_flagged),
            (u(&w["fits_kept"]), u(&w["windows_flagged"])),
            "{at} kept"
        );
        close(j.j.expect("a J"), f(&w["j"]), REL, &format!("{at} J"));
    }
    check_pooled(&got.j, &want["j"], "J");
    for (d, p) in &got.by_dir {
        check_pooled(p, &want["by_dir"][d.to_string()], &format!("J {d:+}"));
    }
    let fr = want["friction"].as_array().expect("friction");
    assert_eq!(got.friction.len(), fr.len());
    for (g, w) in got.friction.iter().zip(fr) {
        let at = format!("friction {:+}", g.dir);
        assert_eq!(
            (g.dir as i64, g.bins),
            (w["dir"].as_i64().expect("dir"), u(&w["bins"])),
            "{at}"
        );
        for (name, a) in [
            ("a0", g.a0),
            ("a1", g.a1),
            ("tc", g.tc),
            ("b", g.b),
            ("rms", g.rms),
        ] {
            close(a, f(&w[name]), 1e-7, &format!("{at} {name}"));
        }
    }
}

fn inertia_of(
    files: &[Value],
    load: &dyn Fn(&Value) -> Vec<Seg>,
    s: &runup::Scales,
) -> (Vec<RunUp>, Inertia) {
    let mut reads = Vec::new();
    for file in files {
        let segs = load(file);
        for (d, sp, co) in pairs(&segs) {
            let run = Run {
                dir: d,
                spin: sp,
                coast: co,
                zero_i: f(&file["zero_i"]),
                zero_e: f(&file["zero_e"]),
            };
            reads.push(runup::read_run(&run, s).expect("a run-up reads"));
        }
    }
    let got = runup::inertia(&reads, s).expect("J");
    (reads, got)
}

fn fixture(file: &Value) -> Vec<Seg> {
    sweep(&dir().join(file["file"].as_str().expect("file")))
}

#[test]
fn the_counters_match_the_notebooks_on_a_spin() {
    let e = json("expected.json");
    let c = &e["checks"];
    let segs = sweep(&dir().join(c["file"].as_str().expect("file")));
    let (_, sp, co) = pairs(&segs)[0];
    assert_eq!(coast::cut_row(co), Some(u(&c["cut"])));
    // the ladder's spin window: the held duty's last 100 ms, half the held
    // rows at most, 20 ms at least
    let duty = |f: &TelFrame| f64::from(f.duty_counts.expect("duty").unsigned_abs());
    let half = median(&sp[sp.len() / 2..].iter().map(duty).collect::<Vec<_>>()).expect("held");
    let held: Vec<f64> = sp
        .iter()
        .filter(|f| duty(f) == half)
        .map(|f| f64::from(f.current_raw.expect("current")))
        .collect();
    let n = (held.len() / 2).clamp(400, 2000).min(held.len());
    let raw = &held[held.len() - n..];
    let fs = f(&e["scales"]["fs"]);
    let (lo, hi) = ripple::BAND_HZ;
    assert_eq!(ripple::line_freq(raw, lo, hi, fs), Some(f(&c["line_hz"])));
    let f0 = f(&c["seed_hz"]);
    let list = |v: &Value| -> Vec<f64> { v.as_array().expect("list").iter().map(f).collect() };
    let rows = |got: &[f64], want: &[f64], what: &str| {
        assert_eq!(got.len(), want.len(), "{what}");
        for (a, b) in got.iter().zip(want) {
            assert!((a - b).abs() < 1e-9, "{what}: {a} against {b}");
        }
    };
    rows(
        &ripple::crossings(raw, f0, ripple::BAND_FRAC, fs),
        &list(&c["crossings"]),
        "crossings",
    );
    let cnt = ripple::count_cycles(raw, f0, ripple::BAND_FRAC, fs).expect("counts");
    assert_eq!(cnt.cycles, u(&c["count_cycles"]));
    close(cnt.hz, f(&c["count_hz"]), REL, "count");
    let tr = ripple::tracked_crossings(raw, 0, raw.len() as f64, f0, ripple::TRACK_CYCLES, fs);
    rows(&tr, &list(&c["tracked"]), "tracked");
    let mut gapped = tr.clone();
    gapped.drain(10..12);
    let r = ripple::repair(&gapped);
    assert_eq!(
        (r.added, r.removed),
        (u(&c["repaired_added"]), u(&c["repaired_removed"]))
    );
    rows(&r.tc, &list(&c["repaired"]), "repaired");
}

#[test]
fn ke_matches_the_notebooks_on_the_ladder_fixture() {
    let e = json("expected.json");
    let s = coast_scales(&e["scales"]);
    let files = e["ladder"]["files"].as_array().expect("files");
    let all = check_ladder(files, &fixture, &s);
    let p = coast::pooled(&all).expect("pooled");
    check_ke_pooled(&p, &e["ladder"]["pooled"]);
    // 12 of the ladder's coasts give the record within 1 %
    let record = f(&e["record"]["ke_mv_per_rev_s"]);
    let ke = coast::mv_per_rev_s(p.all.mean);
    assert_eq!(p.all.n, 12);
    assert!(
        (ke / record - 1.0).abs() < 0.01,
        "Ke {ke} mV per rev/s against the record {record}"
    );
    assert!(
        all.iter().any(|c| !c.gated),
        "the fixture holds a coast the gate refuses"
    );
    // E0 over the drive's tach: a check that carries the tach's 20 ms
    // spread, about 3 % a coast, so only its mean is held
    let cut: Vec<f64> = all
        .iter()
        .filter(|c| c.gated)
        .filter_map(|c| c.ke_cut())
        .collect();
    let m = cut.iter().sum::<f64>() / cut.len() as f64;
    assert!(
        cut.len() == 12 && (m / p.all.mean - 1.0).abs() < 0.01,
        "{m} over {}",
        cut.len()
    );
}

#[test]
fn j_matches_the_notebooks_on_the_runup_fixture() {
    let e = json("expected.json");
    let s = runup_scales(&e["scales"]);
    let files = e["runup"]["files"].as_array().expect("files");
    let (reads, got) = inertia_of(files, &fixture, &s);
    check_runups(&e["runup"], &reads, &got);
    let record = f(&e["record"]["j_kg_m2"]);
    assert!(
        (got.j.mean / record - 1.0).abs() < 0.10,
        "J {} against the record {record}",
        got.j.mean
    );
}

/// Opt-in: the whole datasets of a notebooks checkout, OSC_NOTEBOOKS.
#[test]
fn the_whole_datasets_match_the_notebooks() {
    let Some(nb) = std::env::var_os("OSC_NOTEBOOKS") else {
        eprintln!("OSC_NOTEBOOKS unset: the full parity run is skipped");
        return;
    };
    let nb = PathBuf::from(nb);
    let full = json("full.json");
    let load = |file: &Value| {
        sweep(
            &nb.join(file["from"].as_str().expect("from"))
                .join("sweep.csv"),
        )
    };
    let files = full["ladder"]["files"].as_array().expect("files");
    let all = check_ladder(files, &load, &coast_scales(&full["scales"]));
    let p = coast::pooled(&all).expect("pooled");
    check_ke_pooled(&p, &full["ladder"]["pooled"]);
    let record = &full["record"];
    assert_eq!(p.all.n as f64, f(&record["ke_coasts_gated"]));
    let ke = coast::mv_per_rev_s(p.all.mean);
    let want = f(&record["ke_mv_per_rev_s"]);
    assert!((ke / want - 1.0).abs() < 0.01, "Ke {ke} against {want}");
    let s = runup_scales(&full["scales"]);
    let files = full["runup"]["files"].as_array().expect("files");
    let (reads, got) = inertia_of(files, &load, &s);
    check_runups(&full["runup"], &reads, &got);
    let want = f(&record["j_kg_m2"]);
    assert!(
        (got.j.mean / want - 1.0).abs() < 0.10,
        "J {} against {want}",
        got.j.mean
    );
}

/// The step's judge on the ladder fixture, its rows cut as the step cuts
/// its own captures (a spin's drive rows, its coast's rows from the zero
/// duty's COMMIT): the coasts' Ke pools to the fixture's at the
/// nameplate's six events a revolution.
#[test]
fn the_spin_steps_judge_reads_the_fixtures_ke() {
    use osc_measure::fit::runup::RowScales;
    use osc_measure::steps::spins::{self, JudgeCfg, Spin};
    let e = json("expected.json");
    let sc = &e["scales"];
    let files = e["ladder"]["files"].as_array().expect("files");
    let cfg = |zero_i: f64, zero_e: f64, per_rev: u32| JudgeCfg {
        fs: f(&sc["fs"]),
        polarity: true,
        zero_i,
        zero_e,
        row: RowScales {
            counts_per_a: f(&sc["counts_per_a"]),
            drive_offset_counts: f(&sc["drive_offset_counts"]),
            counts_per_v: f(&sc["counts_per_v"]),
            terminal_offset_counts: f(&sc["terminal_offset_counts"]),
            pwm_arr: f(&sc["pwm_arr"]),
            window_s: 0.0,
        },
        window: None,
        r_ohm: f(&sc["r_ohm"]),
        l_h: f(&sc["l_h"]),
        per_rev,
        i_drag0_a: None,
    };
    let mut coasts = Vec::new();
    for file in files {
        let segs = fixture(file);
        let ps = pairs(&segs);
        let spins: Vec<Spin> = ps
            .iter()
            .enumerate()
            .map(|(k, (d, sp, co))| Spin {
                n: k + 1,
                level: 0.25,
                dir: *d,
                drive: sp,
                coast: co,
            })
            .collect();
        let (zi, ze) = (f(&file["zero_i"]), f(&file["zero_e"]));
        let j = spins::judge(&spins, &cfg(zi, ze, 6));
        assert_eq!(j.per_rev.value, 6);
        let want = file["coasts"].as_array().expect("coasts");
        for (k, ((_, got), w)) in j.coasts.iter().zip(want).enumerate() {
            check_coast(
                got.as_ref().expect("a coast reads"),
                w,
                &format!("{} judged coast {k}", file["from"]),
            );
            coasts.push(got.clone().expect("reads"));
        }
    }
    let p = coast::pooled(&coasts).expect("pooled");
    check_ke_pooled(&p, &e["ladder"]["pooled"]);
    println!(
        "judged Ke {:.4} mV per rev/s over {} gated coasts",
        coast::mv_per_rev_s(p.all.mean),
        p.all.n
    );
}
