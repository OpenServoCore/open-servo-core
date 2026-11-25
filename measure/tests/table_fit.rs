//! The table step's estimators against its offline check (bringup
//! `kb/scratch/ident-table`), on recorded rows (`testdata/table`, written
//! with the reference rules' values by `testdata/table/gen.py`): the 20 %
//! one-way passes of bringup capture A give the position table, the crawl
//! and ladder hold rows the wide codes. Each number is held within 10 % of the offline check's and
//! close to the reference rules' own on the same rows.

use std::path::PathBuf;

use osc_measure::capture::meta::gunzip;
use osc_measure::fit::postable::{
    self, DirectionFit, SMOOTH_WEIGHT, Segment, Span, Table, agreement, common_span,
    direction_term, fit_direction, gains, mean_table, repeat, residual_counts,
};
use osc_measure::fit::widecodes::{self, Finding, HALF_THIN_ROWS, State, THIN_ROWS, find_in};
use osc_servo_core::pos_lut::{self, INTERVALS};
use serde_json::Value;

const FS: f64 = 20_000.0;
/// The bench pair's stops as the offline check took them.
const STOPS: (u16, u16) = (110, 3730);
/// The stops of the table of record (`testdata/mg90-d/pos_lut.json`).
const RECORD_STOPS: (u16, u16) = (82, 3792);
/// The mg90-d motor's commutations a revolution (oscnb `motors`).
const RIPPLE_PER_REV: u32 = 6;

fn path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

fn json(rel: &str) -> Value {
    let bytes = std::fs::read(path(rel)).unwrap_or_else(|e| panic!("{rel}: {e}"));
    let bytes = if rel.ends_with(".gz") {
        gunzip(&bytes).expect("gzip")
    } else {
        bytes
    };
    serde_json::from_slice(&bytes).expect("json")
}

fn expected() -> Value {
    json("testdata/table/expected.json")
}

fn f(v: &Value) -> f64 {
    v.as_f64().expect("number")
}

fn record_slots() -> [u8; 16] {
    let v = json("testdata/mg90-d/wide-codes.json");
    let s: Vec<u8> = v["slots"]
        .as_array()
        .expect("slots")
        .iter()
        .map(|x| x.as_u64().expect("slot") as u8)
        .collect();
    s.try_into().expect("16 slots")
}

fn gear_ratio() -> f64 {
    f(&json("testdata/mg90-d/nameplate.json")["gear_ratio"])
}

fn cyc_deg() -> f64 {
    360.0 / (RIPPLE_PER_REV as f64 * gear_ratio())
}

/// `got` within `share` of `want`.
fn near(label: &str, got: f64, want: f64, share: f64) {
    assert!(
        (got / want - 1.0).abs() <= share,
        "{label}: {got:.4}, want {want:.4} within {:.0} %",
        100.0 * share
    );
}

/// The four passes in capture order (pass-1 up, pass-1 down, pass-2 up,
/// pass-2 down), each a rung.
fn passes() -> Vec<(String, Segment)> {
    let slots = record_slots();
    let mut out = Vec::new();
    for name in ["pass-1", "pass-2"] {
        let rel = format!("testdata/table/{name}.sweep.csv.gz");
        let text =
            String::from_utf8(gunzip(&std::fs::read(path(&rel)).expect("rows")).expect("gzip"))
                .expect("utf8");
        let mut lines = text.lines();
        assert_eq!(
            lines.next(),
            Some("seg,dir,tick,pos,duty_counts,current_raw")
        );
        let rows: Vec<[i64; 6]> = lines
            .map(|l| {
                let v: Vec<i64> = l.split(',').map(|c| c.parse().expect("integer")).collect();
                v.try_into().expect("six columns")
            })
            .collect();
        for seg in [1, 3] {
            let r: Vec<&[i64; 6]> = rows.iter().filter(|r| r[0] == seg).collect();
            let pos: Vec<u16> = r.iter().map(|r| r[3] as u16).collect();
            let duty: Vec<i16> = r.iter().map(|r| r[4] as i16).collect();
            let cur: Vec<u16> = r.iter().map(|r| r[5] as u16).collect();
            let s = postable::segment(&pos, &cur, &duty, &slots, FS).expect("segment");
            out.push((format!("{name}/{seg}"), s));
        }
    }
    out
}

struct Fitted {
    span: Span,
    up: DirectionFit,
    down: DirectionFit,
    table: Table,
}

fn fit(segs: &[Segment], span: Span) -> Fitted {
    let up = fit_direction(segs, 1, &span, SMOOTH_WEIGHT).expect("up");
    let down = fit_direction(segs, -1, &span, SMOOTH_WEIGHT).expect("down");
    let table = mean_table(&up, &down).expect("table");
    Fitted {
        span,
        up,
        down,
        table,
    }
}

fn all() -> (Vec<Segment>, Fitted) {
    let segs: Vec<Segment> = passes().into_iter().map(|(_, s)| s).collect();
    let span = common_span(&segs).expect("span");
    let fitted = fit(&segs, span);
    (segs, fitted)
}

#[test]
fn each_pass_is_read_as_the_reference_reads_it() {
    let exp = expected();
    for (name, s) in passes() {
        let e = &exp["table"]["passes"][&name];
        assert_eq!(s.dir as i64, e["sign"].as_i64().expect("sign"), "{name}");
        assert_eq!(s.f_seed, f(&e["f"]), "{name}");
        assert_eq!(
            s.crossings as u64,
            e["crossings"].as_u64().expect("n"),
            "{name}"
        );
        assert_eq!(
            s.x.len() as u64,
            e["samples"].as_u64().expect("n"),
            "{name}"
        );
        assert_eq!(s.odd, f(&e["odd"]), "{name}");
        assert!(s.odd <= postable::ODD_MAX && s.speed_spread < 0.2, "{name}");
    }
    let p = passes();
    for (a, b) in [(0, 2), (1, 3)] {
        let g = agreement(&p[a].1, &p[b].1).expect("they overlap");
        assert!(g.holds(), "{} against {}: {g:?}", p[a].0, p[b].0);
    }
}

#[test]
fn the_mean_table_matches_the_offline_check() {
    let exp = &expected()["table"];
    let cyc = cyc_deg();
    assert!((cyc / f(&exp["cyc_deg"]) - 1.0).abs() < 1e-9);
    let (_, t) = all();
    let knots = t.span.knots();
    assert_eq!(
        (knots[0], knots[knots.len() - 1]),
        (f(&exp["knots"][0]), f(&exp["knots"][1]))
    );
    let mean: Vec<f64> = exp["mean"]
        .as_array()
        .expect("mean")
        .iter()
        .map(f)
        .collect();
    let worst = t
        .table
        .angle
        .iter()
        .zip(&mean)
        .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));
    assert!(
        worst < 1e-3,
        "the mean table off the reference by {worst} cycles"
    );

    // the step refuses a direction over 4 counts rms
    for (fit, key) in [(&t.up, "up"), (&t.down, "down")] {
        near(
            key,
            fit.residual_rms * cyc,
            f(&exp["residual_deg"][key]),
            0.01,
        );
        let counts = residual_counts(fit, &t.table);
        assert!(counts <= 4.0, "{key} residual {counts:.2} counts");
    }

    let to_deg = t.table.mean_slope() * cyc;
    let term = direction_term(&t.up, &t.down, &t.table);
    near("up less down rms", term.rms * to_deg, 0.294, 0.10);
    near("up less down max", term.max * to_deg, 0.73, 0.10);
    near(
        "up less down rms",
        term.rms * to_deg,
        f(&exp["up_less_down_deg"]["rms"]),
        0.01,
    );
    assert!(term.rms < 8.0, "{term:?}");

    let scale = t
        .table
        .scale(Some(RIPPLE_PER_REV), Some(gear_ratio()))
        .expect("scale");
    near("counts a degree", scale, 19.01, 0.10);
    near("counts a degree", scale, f(&exp["counts_per_deg"]), 1e-3);
}

#[test]
fn one_round_trip_repeats_the_other() {
    let exp = &expected()["table"];
    let (segs, t) = all();
    let trip = |k: usize| fit(&segs[2 * k..2 * k + 2], t.span);
    let (a, b) = (trip(0), trip(1));
    let r = repeat(&a.table, &b.table).expect("same span");
    let to_deg = a.table.mean_slope() * cyc_deg();
    near("trip line out", r.line_out * to_deg, 0.038, 0.10);
    near("trip slope", r.slope, 0.022, 0.10);
    near(
        "trip line out",
        r.line_out * to_deg,
        f(&exp["trip_repeat"]["line_out_deg"]),
        0.02,
    );
    near(
        "trip slope",
        r.slope,
        f(&exp["trip_repeat"]["slope_rms"]),
        0.02,
    );
    // the step's repeat line: each trip against the pooled table
    for one in [&a, &b] {
        let r = repeat(&one.table, &t.table).expect("same span");
        assert!(r.line_out <= 2.0 && r.slope <= 0.05, "{r:?}");
    }
}

/// Zone mean |slope error| against the camera lookup of record: each
/// 48-count slope over its span mean, over the lookup's the same way.
fn zone_errors(span: &Span, angle: &[f64]) -> Vec<(f64, f64)> {
    let text =
        std::fs::read_to_string(path("testdata/table/lookup-of-record.csv")).expect("lookup");
    let (c, a): (Vec<f64>, Vec<f64>) = text
        .lines()
        .skip(1)
        .map(|l| {
            let (x, y) = l.split_once(',').expect("two columns");
            (
                x.parse::<f64>().expect("counts"),
                y.parse::<f64>().expect("deg"),
            )
        })
        .unzip();
    let grid = span.slope_grid();
    let lookup = |x: f64| osc_measure::fit::interp(x, &c, &a, f64::NAN, f64::NAN);
    let gl: Vec<f64> = grid
        .iter()
        .map(|&x| (lookup(x + 24.0) - lookup(x - 24.0)) / 48.0)
        .collect();
    let gt = span.slopes(angle);
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    let (ml, mt) = (mean(&gl), mean(&gt));
    [(1570.0, 1650.0), (2200.0, 2250.0)]
        .iter()
        .map(|&(lo, hi)| {
            let pick = |e: &dyn Fn(usize) -> f64| {
                let v: Vec<f64> = (0..grid.len())
                    .filter(|&j| grid[j] >= lo && grid[j] <= hi)
                    .map(|j| e(j).abs())
                    .collect();
                mean(&v)
            };
            (
                pick(&|j| (gt[j] / mt) / (gl[j] / ml) - 1.0),
                pick(&|j| ml / gl[j] - 1.0),
            )
        })
        .collect()
}

#[test]
fn the_table_cures_the_fast_zones_against_the_camera_lookup() {
    let exp = &expected()["table"]["zone_slope_error"];
    let (_, t) = all();
    let z = zone_errors(&t.span, &t.table.angle);
    for ((table, straight), (key, note, note_straight)) in
        z.iter().zip([("z1", 0.085, 0.410), ("z2", 0.070, 0.243)])
    {
        near(key, *table, note, 0.10);
        near(key, *straight, note_straight, 0.10);
        near(key, *table, f(&exp[key]["mean"]), 0.02);
        assert!(*table < 0.5 * straight, "{key}");
    }
}

#[test]
fn each_direction_sees_the_zone_excess_and_its_split() {
    let (_, t) = all();
    let grid = t.span.slope_grid();
    let excess = |angle: &[f64], lo: f64, hi: f64| {
        let g = t.span.slopes(angle);
        let m = g.iter().sum::<f64>() / g.len() as f64;
        let v: Vec<f64> = (0..g.len())
            .filter(|&j| grid[j] >= lo && grid[j] <= hi)
            .map(|j| m / g[j] - 1.0)
            .collect();
        v.iter().sum::<f64>() / v.len() as f64
    };
    for ((lo, hi), up, down) in [
        ((1600.0, 1650.0), 0.50, 0.51),
        ((2200.0, 2250.0), 0.13, 0.22),
        ((1890.0, 1975.0), 0.37, 0.68),
    ] {
        near("up excess", excess(&t.up.angle, lo, hi), up, 0.10);
        near("down excess", excess(&t.down.angle, lo, hi), down, 0.10);
    }
}

#[test]
fn the_points_pass_the_firmwares_validate() {
    let exp = &expected()["table"];
    let (_, t) = all();
    let p = t.table.points(STOPS.0, STOPS.1);
    let mut full = [0i16; INTERVALS + 1];
    full[..INTERVALS].copy_from_slice(&p);
    assert_eq!(pos_lut::validate(&full, STOPS.0, STOPS.1), Ok(()));
    let (lo, hi) = gains(&p);
    assert_eq!(
        (lo as i64, hi as i64),
        (
            exp["gain_q4"][0].as_i64().expect("gain"),
            exp["gain_q4"][1].as_i64().expect("gain")
        )
    );
    assert!(lo >= 8 && hi <= 32, "{lo}..{hi}");
    let max = p.iter().map(|c| c.unsigned_abs()).max().expect("points");
    assert_eq!(max as i64, exp["points_max_abs"].as_i64().expect("max"));
    assert!(t.span.coverage(STOPS.0, STOPS.1) > 0.45);
}

/// A set's two halves' histograms, and their sum, with each one's swept
/// range.
fn hold_set(name: &str) -> [(Vec<u32>, u16, u16); 3] {
    let v = json("testdata/table/widecodes.json.gz");
    let half = |k: &str| {
        let h = &v[name][k];
        let lo = h["lo"].as_u64().expect("lo") as usize;
        let mut out = vec![0u32; widecodes::CODES];
        for (j, n) in h["rows"].as_array().expect("rows").iter().enumerate() {
            out[lo + j] = n.as_u64().expect("rows") as u32;
        }
        (out, lo as u16, h["hi"].as_u64().expect("hi") as u16)
    };
    let (even, odd) = (half("even"), half("odd"));
    let whole: Vec<u32> = even.0.iter().zip(&odd.0).map(|(a, b)| a + b).collect();
    let (lo, hi) = (even.1.min(odd.1), even.2.max(odd.2));
    [(whole, lo, hi), even, odd]
}

/// Every carry of `got` as the reference rule reads it on the same rows.
fn same_as_reference(label: &str, got: &Finding, exp: &Value) {
    for c in &got.carries {
        let k = c.slot.to_string();
        match c.state {
            State::Unreached => {
                assert!(
                    exp["carries"].get(&k).is_none() && exp["thin"].get(&k).is_none(),
                    "{label} {k}"
                )
            }
            State::Thin => assert_eq!(c.rows_ref, f(&exp["thin"][&k]), "{label} {k}"),
            State::Wide | State::NotWide => {
                let e = &exp["carries"][&k];
                assert_eq!(got.slots[c.slot] as f64, f(&e["slot"]), "{label} {k}");
                assert_eq!(c.skipped as f64, f(&e["skipped"]), "{label} {k}");
                assert_eq!(c.rows_ref, f(&e["rows_ref"]), "{label} {k}");
                assert!((c.ratio - f(&e["ratio"])).abs() < 1e-9, "{label} {k}");
            }
        }
    }
}

#[test]
fn the_ladder_set_reads_the_slots_of_record_within_one_unit() {
    let exp = &expected()["widecodes"]["ladder"];
    let [whole, even, odd] = hold_set("ladder");
    let all = find_in(&whole.0, whole.1, whole.2, THIN_ROWS).expect("found");
    let mut want = [0u8; 16];
    (want[3], want[7], want[9]) = (88, 88, 48);
    assert_eq!(all.slots, want);
    same_as_reference("ladder", &all, &exp["whole"]);
    for (h, key) in [(&even, "even"), (&odd, "odd")] {
        let f = find_in(&h.0, h.1, h.2, THIN_ROWS).expect("found");
        same_as_reference(key, &f, &exp[key]);
    }
    let halves = |h: &(Vec<u32>, u16, u16)| find_in(&h.0, h.1, h.2, HALF_THIN_ROWS).expect("found");
    assert_eq!(
        widecodes::check(
            &all,
            &halves(&even),
            &halves(&odd),
            RECORD_STOPS.0,
            RECORD_STOPS.1
        ),
        Ok(())
    );
    // every carry it reaches within one unit of the record (80 at 1023)
    let record = record_slots();
    for c in all.carries.iter().filter(|c| c.state != State::Unreached) {
        assert!(all.slots[c.slot].abs_diff(record[c.slot]) <= 8, "{c:?}");
    }
    assert_eq!(all.carries[1].state, State::Unreached);
}

#[test]
fn the_crawl_set_names_carry_3_thin() {
    let exp = &expected()["widecodes"]["crawl"];
    let [whole, even, odd] = hold_set("crawl");
    let all = find_in(&whole.0, whole.1, whole.2, THIN_ROWS).expect("found");
    let mut want = [0u8; 16];
    (want[7], want[9]) = (88, 48);
    assert_eq!(all.slots, want);
    assert_eq!(all.carries[3].state, State::Thin);
    assert_eq!(all.carries[3].rows_ref, 59.5);
    same_as_reference("crawl", &all, &exp["whole"]);
    for (h, key) in [(&even, "even"), (&odd, "odd")] {
        let f = find_in(&h.0, h.1, h.2, THIN_ROWS).expect("found");
        same_as_reference(key, &f, &exp[key]);
    }
    let halves = |h: &(Vec<u32>, u16, u16)| find_in(&h.0, h.1, h.2, HALF_THIN_ROWS).expect("found");
    assert_eq!(
        widecodes::check(
            &all,
            &halves(&even),
            &halves(&odd),
            RECORD_STOPS.0,
            RECORD_STOPS.1
        ),
        Ok(())
    );
}
