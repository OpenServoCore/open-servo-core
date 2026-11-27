//! The recorded run (`testdata/run`): one no-flag `osc-ident run` on the
//! bench servo, trimmed (`testdata/run/TRIMMED.md`). The web's report is
//! specified on it, so what the gather writes must stay what the folder
//! holds: a change to the gathered files' shape fails here until the
//! folder's gathered files are regenerated (`testdata/run/TRIMMED.md`).

mod common;

use common::{load, path, text};
use osc_measure::Nameplate;
use osc_measure::gather::{self, RunResults, SPINS_RESULT, StepResult, WINDOW_RESULT};
use osc_measure::steps::{cruise, probe, rest, stops, table};
use serde_json::Value;

const RUN: &str = "testdata/run/00000000ffffffff76e5bc220ef6abcd/20261127T130203Z";

/// The neutral root the folder's paths were rewritten to: the gathered
/// files name each result under it.
fn root_as_run() -> String {
    let s: Value = load("testdata/run/sources.json");
    s["00000000ffffffff76e5bc220ef6abcd/20261127T130203Z"]["root_as_run"]
        .as_str()
        .expect("root_as_run")
        .to_string()
}

/// The run's results read as the run read them, the cruise step's only
/// where `cruised`: the first gather ran before the cruise step.
fn results(cruised: bool) -> RunResults {
    let root = root_as_run();
    let r = |rel: &str| {
        let bytes = std::fs::read(path(&format!("{RUN}/{rel}"))).expect(rel);
        Some(StepResult::parse(&format!("{root}/{rel}"), &bytes).expect(rel))
    };
    RunResults {
        rest: r(rest::RESULT),
        probe: r(probe::RESULT),
        stops: r(stops::RESULT),
        window: r(WINDOW_RESULT),
        spins: r(SPINS_RESULT),
        cruise: if cruised { r(cruise::RESULT) } else { None },
        table: r(table::RESULT),
    }
}

/// The gather over the run's results gives the files of its `inputs/`
/// (gathered after the spins) and of its `final/inputs/` (after the
/// cruise step), byte for byte.
#[test]
fn the_recorded_run_gathers_to_its_own_inputs() {
    let np: Nameplate = load(&format!("{RUN}/inputs/nameplate.json"));
    for (folder, cruised) in [("inputs", false), ("final/inputs", true)] {
        let g = gather::gather(&results(cruised), &np).expect(folder);
        let wide = g.wide_codes.as_ref().expect("the table step's slots");
        for (file, doc) in [
            ("measured.json", &g.measured),
            ("pos_lut.json", &g.table),
            ("wide-codes.json", wide),
            ("cross-checks.json", &g.cross_checks),
        ] {
            let recorded = text(&format!("{RUN}/{folder}/{file}"));
            assert!(
                gather::render(doc) == recorded,
                "{folder}/{file} differs from the recorded run's:\n{}",
                gather::render(doc)
            );
        }
    }
}
