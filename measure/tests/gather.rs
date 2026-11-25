//! The gather on one accepted run of the bench servo (`testdata/gather`:
//! the rest, probe and stops results as the steps wrote them, and window
//! and spins results carrying the record's values), then the rules on what
//! it gathered, register by register against the set of record.

mod common;

use std::collections::BTreeMap;

use common::{descriptor, load, text};
use osc_measure::gather::{self, RunResults, StepResult};
use osc_measure::inputs::{Measured, ThermalInputs, Tuning};
use osc_measure::rules::speed_loop;
use osc_measure::set::{self, FromRules, TableFile};
use osc_measure::{Board, Inputs, Nameplate, Outcome, Policy, build};
use serde_json::Value;

fn result(name: &str) -> StepResult {
    let rel = format!("testdata/gather/{name}.json");
    StepResult::parse(&format!("results/{name}.json"), text(&rel).as_bytes()).expect(name)
}

fn doc(name: &str) -> Value {
    load(&format!("testdata/gather/{name}.json"))
}

fn edited(name: &str, edit: impl FnOnce(&mut Value)) -> StepResult {
    let mut v = doc(name);
    edit(&mut v);
    StepResult::parse(
        &format!("results/{name}.json"),
        &serde_json::to_vec(&v).expect("json"),
    )
    .expect(name)
}

fn run() -> RunResults {
    RunResults {
        rest: Some(result("rest")),
        probe: Some(result("probe")),
        stops: Some(result("stops")),
        window: Some(result("window")),
        spins: Some(result("spins")),
        cruise: None,
        table: None,
    }
}

fn nameplate() -> Nameplate {
    load("testdata/mg90-d/nameplate.json")
}

fn inputs_of(measured: Measured) -> Inputs {
    Inputs {
        board: load::<Board>("boards/osc-dev-v006.json"),
        measured,
        nameplate: nameplate(),
        policy: load::<Policy>("testdata/mg90-d/policy.json"),
        thermal: load::<ThermalInputs>("testdata/mg90-d/thermal.json"),
        tuning: Tuning::default(),
    }
}

#[derive(serde::Deserialize)]
struct RecordField {
    value: i64,
}

#[derive(serde::Deserialize)]
struct Record {
    fields: BTreeMap<String, RecordField>,
}

/// Every value names its result file, key and sha1; the gathered file
/// loads as the rules' `Measured`, and the rules on it, the speed gain at
/// the starting one, give a set file with its straight table.
#[test]
fn the_gathered_inputs_load_and_the_rules_give_a_set() {
    let g = gather::gather(&run(), &nameplate()).expect("gathers");
    for l in &g.lines {
        println!("{l}");
    }
    let shas: BTreeMap<String, String> = load::<Value>("testdata/gather/sources.json")["files"]
        .as_object()
        .expect("files")
        .iter()
        .map(|(k, v)| (format!("results/{k}"), v["sha1"].as_str().unwrap().into()))
        .collect();
    for (name, q) in g.measured.as_object().expect("object") {
        let s = &q["source"];
        let file = s["file"].as_str().expect("file");
        assert_eq!(
            s["sha1"].as_str(),
            shas.get(file).map(String::as_str),
            "{name}"
        );
        assert!(!s["key"].as_str().unwrap_or("").is_empty(), "{name}");
    }

    let text = gather::render(&g.measured);
    let measured: Measured = serde_json::from_str(&text).expect("loads as Measured");
    assert!(measured.speed_kp_verified.is_none() && measured.shelf.is_none());
    assert!(measured.breakaway_ma.is_none());
    let rail = measured.rail_counts_per_v.as_ref().expect("rail");
    assert_eq!(rail.value, 248.24242424242425);
    assert_eq!(rail.source.file, "results/rest.json");
    assert_eq!(
        measured.rail_offset_counts.as_ref().map(|q| q.value),
        Some(0.0)
    );
    let r = measured.r_ohm.as_ref().expect("r");
    assert!((r.value - 4.187787930981532).abs() < 1e-12);
    // the larger of the burst spread (0.0058 x R) and half the stops' gap
    let half = (4.224129621497565 - 4.038807310729814) / 2.0;
    assert!((r.hi.unwrap() - r.value - half).abs() < 1e-12, "{r:?}");
    assert!((measured.l_mh.as_ref().unwrap().value - 0.8534002319061124).abs() < 1e-12);
    let c = measured.r_board_c.as_ref().unwrap().value;
    assert!((c - (21.04279655976103 + 21.085935179399826) / 2.0).abs() < 1e-9);

    let table_text = gather::render(&g.table);
    let table = TableFile::parse(&table_text).expect("the table file parses");
    assert!(table.points.iter().all(|&k| k == 0));
    assert_eq!(table.stops, (107, 3729));
    let cpd = measured.table.as_ref().unwrap().counts_per_deg;
    assert_eq!(table.counts_per_deg, Some(cpd));

    let mut inputs = inputs_of(measured);
    let k0 = speed_loop::starting_gain(&inputs).expect("a starting gain");
    inputs.measured.speed_kp_verified = Some(k0);
    let rules = build(&inputs);
    let (file, notes) = set::from_rules(
        &rules,
        &FromRules {
            pair: "mg90-d__osc-dev-v006",
            nameplate: &inputs.nameplate,
            counts_per_deg: Some(cpd),
            table: Some(&table),
            slots: None,
            files: &[],
        },
        &descriptor(),
    )
    .expect("the rules give a set");
    for n in &notes {
        println!("note: {n}");
    }
    assert_eq!(file.table().0, "IDENTITY");
    assert_eq!(file.slots(), Ok(None));

    let record: Record = load("testdata/mg90-d/set-of-record.json");
    println!("starting gain {k0:.5} ({} q88)", (k0 * 256.0).round());
    println!("| register | record | gathered | difference |");
    println!("|---|---:|---:|---:|");
    let mut names: Vec<&str> = rules.fields.iter().map(|(n, _)| *n).collect();
    for n in record.fields.keys() {
        if !names.contains(&n.as_str()) {
            names.push(n);
        }
    }
    for name in names {
        let rec = record.fields.get(name).map(|f| f.value);
        let ours = match rules.get(name) {
            Some(Outcome::Bound(e)) => e.value.to_string(),
            Some(Outcome::Held(m)) => format!("held ({})", m.input),
            Some(Outcome::Refused(r)) => format!("refused ({})", r.why),
            None => "no rule".into(),
        };
        let diff = match (rec, rules.value(name)) {
            (Some(0), Some(0)) => "0".to_string(),
            (Some(a), Some(b)) if a != 0 => format!("{:+.1}%", 100.0 * (b - a) as f64 / a as f64),
            _ => "-".into(),
        };
        println!(
            "| {name} | {} | {ours} | {diff} |",
            rec.map_or("-".into(), |v| v.to_string())
        );
    }
    let held: Vec<&str> = rules
        .fields
        .iter()
        .filter(|(_, o)| matches!(o, Outcome::Held(_)))
        .map(|(n, _)| *n)
        .collect();
    assert_eq!(held, ["fric_breakaway_counts"], "held");
    assert_eq!(rules.value("pos_min_phys_counts"), Some(107));
    assert_eq!(rules.value("pos_max_phys_counts"), Some(3729));
    assert_eq!(rules.value("drive_polarity"), Some(1));
}

/// Without the window and spins results the gather still runs and says
/// what is missing; the rules then hold every field that reads them.
#[test]
fn a_run_without_window_or_spins_says_what_is_missing() {
    let mut r = run();
    r.window = None;
    r.spins = None;
    let g = gather::gather(&r, &nameplate()).expect("gathers");
    let text = g.lines.join("\n");
    assert!(text.contains("g_window_ms: missing"), "{text}");
    assert!(text.contains("ke_v_s_per_rad, j_kg_m2: missing"), "{text}");
    let m = g.measured.as_object().unwrap();
    for k in [
        "g_window_ms",
        "ke_v_s_per_rad",
        "j_kg_m2",
        "cruise_coulomb_ma",
    ] {
        assert!(!m.contains_key(k), "{k}");
    }
    let measured: Measured = serde_json::from_value(g.measured).expect("loads");
    let inputs = inputs_of(measured);
    assert!(speed_loop::starting_gain(&inputs).is_err());
    let set = build(&inputs);
    for f in [
        "g_window_q016",
        "fric_fc_counts",
        "fric_fv_q016",
        "b_i_q313",
        "v_kp_q88",
    ] {
        assert!(matches!(set.get(f), Some(Outcome::Held(_))), "{f}");
    }
}

/// A rest result older than its rail scale leaves the two rail fields out,
/// said in a line; the rules then hold the undervolt floor and the fold.
#[test]
fn a_rest_result_without_the_rail_scale_leaves_the_rail_out() {
    let mut r = run();
    r.rest = Some(edited("rest", |v| {
        v.as_object_mut()
            .expect("object")
            .remove("rail_counts_per_v_nominal");
    }));
    let g = gather::gather(&r, &nameplate()).expect("gathers");
    assert!(
        g.lines
            .iter()
            .any(|l| l.starts_with("rail_counts_per_v, rail_offset_counts: missing")),
        "{:?}",
        g.lines
    );
    let m = g.measured.as_object().expect("object");
    assert!(!m.contains_key("rail_counts_per_v") && !m.contains_key("rail_offset_counts"));
    let set = build(&inputs_of(
        serde_json::from_value(g.measured).expect("loads"),
    ));
    for f in ["v_undervolt_counts", "rail_fold_q8"] {
        assert!(matches!(set.get(f), Some(Outcome::Held(_))), "{f}");
    }
}

/// A missing required result, or one not accepted, is refused naming the
/// step to run; so is a later step's result not accepted.
#[test]
fn a_step_missing_or_not_accepted_is_refused_naming_the_step() {
    for drop in ["rest", "probe", "stops"] {
        let mut r = run();
        match drop {
            "rest" => r.rest = None,
            "probe" => r.probe = None,
            _ => r.stops = None,
        }
        let e = gather::gather(&r, &nameplate()).expect_err(drop);
        assert!(
            e.0.contains(&format!("results/{drop}.json")) && e.0.contains("osc-ident step stops"),
            "{e}"
        );
    }
    let mut r = run();
    r.stops = Some(edited("stops", |v| v["verdict"] = "refused".into()));
    let e = gather::gather(&r, &nameplate()).expect_err("refused");
    assert!(
        e.0.contains("is refused, not accepted") && e.0.contains("osc-ident step stops"),
        "{e}"
    );
    let mut r = run();
    r.window = Some(edited("window", |v| v["verdict"] = "aborted".into()));
    let e = gather::gather(&r, &nameplate()).expect_err("refused");
    assert!(e.0.contains("window step"), "{e}");
    let mut r = run();
    r.probe = Some(result("rest"));
    let e = gather::gather(&r, &nameplate()).expect_err("refused");
    assert!(e.0.contains("not the probe step's"), "{e}");

    let mut np = nameplate();
    np.travel_deg = None;
    let e = gather::gather(&run(), &np).expect_err("refused");
    assert!(e.0.contains("travel_deg"), "{e}");
}

/// The sign the probe proved is the one gathered: the stored sign where it
/// proved right, flipped where it proved wrong.
#[test]
fn the_polarity_is_the_one_the_probe_proved() {
    let g = gather::gather(&run(), &nameplate()).expect("gathers");
    assert_eq!(g.measured["drive_polarity"]["value"], true);
    assert!(g.lines.iter().any(|l| l.contains("proved right")));

    let mut r = run();
    r.probe = Some(edited("probe", |v| {
        v["drive_polarity"]["stored"] = true.into();
        v["drive_polarity"]["found"] = false.into();
        v["drive_polarity"]["stored_right"] = false.into();
    }));
    r.stops = Some(edited("stops", |v| {
        v["drive_polarity"]["driven"] = false.into();
        v["drive_polarity"]["corrected_volatile"] = true.into();
    }));
    let g = gather::gather(&r, &nameplate()).expect("gathers");
    assert_eq!(g.measured["drive_polarity"]["value"], false);
    assert_eq!(
        g.measured["drive_polarity"]["source"]["key"],
        "drive_polarity.found"
    );
    assert!(g.lines.iter().any(|l| l.contains("proved wrong, flipped")));

    // the stops step drove on another sign than the probe proved
    let mut r = run();
    r.stops = Some(edited("stops", |v| {
        v["drive_polarity"]["driven"] = false.into()
    }));
    let e = gather::gather(&r, &nameplate()).expect_err("refused");
    assert!(e.0.contains("another sign"), "{e}");
}

/// The stops round inward, the low one up and the high one down, and the
/// table's scale is their span over the nameplate's travel.
#[test]
fn the_stops_round_inward() {
    let g = gather::gather(&run(), &nameplate()).expect("gathers");
    assert_eq!(g.measured["stops"]["raw_min"], 107);
    assert_eq!(g.measured["stops"]["raw_max"], 3729);

    let mut r = run();
    r.stops = Some(edited("stops", |v| {
        v["stops"]["low"]["median_counts"] = 82.25.into();
        v["stops"]["high"]["median_counts"] = 3791.75.into();
    }));
    let g = gather::gather(&r, &nameplate()).expect("gathers");
    assert_eq!(g.measured["stops"]["raw_min"], 83);
    assert_eq!(g.measured["stops"]["raw_max"], 3791);
    let travel = nameplate().travel_deg.unwrap();
    let cpd = g.measured["table"]["counts_per_deg"].as_f64().unwrap();
    assert_eq!(cpd, (3791.0 - 83.0) / travel);

    let mut r = run();
    r.stops = Some(edited("stops", |v| {
        v["stops"]["low"]["median_counts"] = 500.5.into();
        v["stops"]["high"]["median_counts"] = 500.7.into();
    }));
    let e = gather::gather(&r, &nameplate()).expect_err("no travel");
    assert!(e.0.contains("no travel"), "{e}");
}

/// The friction line is the cruise step's where the run holds it
/// accepted, a line saying so; else the spins', a line saying why and
/// that the spin line reads high at low speed. A cruise on another sense
/// block, or another step's file, is refused.
#[test]
fn the_friction_line_is_the_cruise_steps_where_it_was_accepted() {
    let mut r = run();
    r.cruise = Some(result("cruise"));
    let g = gather::gather(&r, &nameplate()).expect("gathers");
    let m = &g.measured;
    assert_eq!(m["cruise_coulomb_ma"]["value"], 39.4237);
    assert_eq!(m["cruise_coulomb_ma"]["lo"], 38.2465);
    assert_eq!(m["cruise_viscous_ma_s_per_rad"]["value"], 0.040364);
    assert_eq!(
        m["cruise_coulomb_ma"]["source"]["file"],
        "results/cruise.json"
    );
    let text = g.lines.join("\n");
    assert!(
        text.contains("friction line: from the cruise step's closed-loop cruises")
            && text.contains("reads high at low speed"),
        "{text}"
    );

    for (edit, why) in [
        (
            Some(edited("cruise", |v| v["verdict"] = "refused".into())),
            "as the cruise step's results/cruise.json is refused",
        ),
        (None, "as the run holds no results/cruise.json"),
    ] {
        let mut r = run();
        r.cruise = edit;
        let g = gather::gather(&r, &nameplate()).expect("gathers");
        assert_eq!(
            g.measured["cruise_coulomb_ma"]["source"]["file"],
            "results/spins.json"
        );
        let text = g.lines.join("\n");
        assert!(
            text.contains(&format!(
                "friction line: from the spin step's open-loop spins, {why}"
            )) && text.contains("reads high at low speed"),
            "{text}"
        );
    }

    let mut r = run();
    r.cruise = Some(edited("cruise", |v| {
        v["scales"]["counts_per_a"] = 1135.0.into()
    }));
    let e = gather::gather(&r, &nameplate()).expect_err("refused");
    assert!(e.0.contains("another sense block"), "{e}");
    let mut r = run();
    r.cruise = Some(edited("cruise", |v| v["step"] = "spins".into()));
    let e = gather::gather(&r, &nameplate()).expect_err("refused");
    assert!(e.0.contains("not the cruise step's"), "{e}");
}

/// An accepted table step's slots, points and scale are the ones gathered,
/// each naming `results/table.json`, its key and sha1; the rules on them
/// give a set whose table is LIVE on those points and whose slots are the
/// step's, written together. Without the step the straight table and the
/// servo's slots stay, said in a line.
#[test]
fn gather_carries_the_table_steps_points_slots_and_scale() {
    let tabled = result("table");
    let mut r = run();
    r.table = Some(tabled.clone());
    let g = gather::gather(&r, &nameplate()).expect("gathers");
    let src = &g.measured["table"]["source"];
    assert_eq!(src["file"], "results/table.json");
    assert_eq!(src["sha1"], tabled.sha1.as_str());
    assert!(
        src["key"]
            .as_str()
            .is_some_and(|k| k.starts_with("scale.counts_per_deg (the ripple")),
        "{src}"
    );
    let cpd = tabled.doc["scale"]["counts_per_deg"]
        .as_f64()
        .expect("scale");
    assert_eq!(g.measured["table"]["counts_per_deg"].as_f64(), Some(cpd));
    let table = TableFile::parse(&gather::render(&g.table)).expect("the table file parses");
    let want: Vec<i64> = tabled.doc["points_by_page"]
        .as_array()
        .expect("pages")
        .iter()
        .flat_map(|p| {
            p.as_array()
                .expect("page")
                .iter()
                .map(|v| v.as_i64().expect("point"))
        })
        .collect();
    assert_eq!(
        table.points.iter().map(|&k| k as i64).collect::<Vec<_>>(),
        want
    );
    assert!(table.points.iter().any(|&k| k != 0));
    assert_eq!(table.stops, (107, 3729));
    assert_eq!(table.counts_per_deg, Some(cpd));
    assert_eq!(g.table["source"]["file"], "results/table.json");
    let wide = g.wide_codes.as_ref().expect("the slots file");
    let slots: Vec<u8> = wide["slots"]
        .as_array()
        .expect("slots")
        .iter()
        .map(|v| v.as_u64().expect("slot") as u8)
        .collect();
    assert_eq!(slots, [0, 48, 0, 80, 0, 0, 0, 88, 0, 48, 0, 0, 0, 0, 0, 0]);
    assert_eq!(wide["source"]["key"], "slots");
    assert!(
        g.lines
            .iter()
            .any(|l| l.starts_with("wide-code slots [0, 48")),
        "{:?}",
        g.lines
    );

    let measured: Measured =
        serde_json::from_str(&gather::render(&g.measured)).expect("loads as Measured");
    let mut inputs = inputs_of(measured);
    inputs.measured.speed_kp_verified = Some(speed_loop::starting_gain(&inputs).expect("gain"));
    let rules = build(&inputs);
    let (file, _) = set::from_rules(
        &rules,
        &FromRules {
            pair: "mg90-d__osc-dev-v006",
            nameplate: &inputs.nameplate,
            counts_per_deg: Some(cpd),
            table: Some(&table),
            slots: Some(&slots),
            files: &[],
        },
        &descriptor(),
    )
    .expect("the rules give a set");
    assert_eq!(file.table().0, "LIVE");
    assert_eq!(file.table().1, set::points_sha1(&table.points));
    assert_eq!(file.slots(), Ok(Some(slots)));

    let g = gather::gather(&run(), &nameplate()).expect("gathers");
    assert!(g.wide_codes.is_none());
    assert!(
        g.lines
            .iter()
            .any(|l| l.starts_with("wide-code slots: not gathered")
                && l.contains("results/table.json")),
        "{:?}",
        g.lines
    );
}

/// A table step built against other stops, or not accepted, refuses the
/// gather naming the step.
#[test]
fn a_table_step_on_other_stops_or_refused_is_refused() {
    let mut r = run();
    r.table = Some(edited("table", |v| v["stops_used"]["raw_min"] = 110.into()));
    let e = gather::gather(&r, &nameplate()).expect_err("other stops");
    assert!(
        e.0.contains("built against stops 110") && e.0.contains("table step"),
        "{e}"
    );
    let mut r = run();
    r.table = Some(edited("table", |v| v["verdict"] = "refused".into()));
    let e = gather::gather(&r, &nameplate()).expect_err("refused");
    assert!(e.0.contains("run the table step again"), "{e}");
}
