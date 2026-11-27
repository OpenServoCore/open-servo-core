//! The rules against the mg90-d set of record, fed the notebooks' results
//! of record (`testdata/mg90-d`, each value with its source file's sha1),
//! the dev board's description, and the record's thermal values as user
//! inputs.

use std::collections::BTreeMap;
use std::path::PathBuf;

use osc_measure::inputs::{Measured, ShelfRecord, ThermalInputs, Tuning};
use osc_measure::rules::{Kind, rules};
use osc_measure::{Board, Inputs, Missing, Nameplate, Outcome, Policy, build};
use serde::Deserialize;
use serde::de::DeserializeOwned;

fn path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

fn load<T: DeserializeOwned>(rel: &str) -> T {
    let text = std::fs::read_to_string(path(rel)).unwrap_or_else(|e| panic!("{rel}: {e}"));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("{rel}: {e}"))
}

fn inputs() -> Inputs {
    let mut measured: Measured = load("testdata/mg90-d/measured.json");
    measured.shelf = Some(load::<ShelfRecord>("testdata/mg90-d/shelf.json"));
    Inputs {
        board: load::<Board>("boards/osc-dev-v006.json"),
        measured,
        nameplate: load::<Nameplate>("testdata/mg90-d/nameplate.json"),
        policy: load::<Policy>("testdata/mg90-d/policy.json"),
        thermal: load::<ThermalInputs>("testdata/mg90-d/thermal.json"),
        tuning: Tuning::default(),
    }
}

#[derive(Deserialize)]
struct RecordField {
    value: i64,
    band: Option<(i64, i64)>,
    class: String,
}

#[derive(Deserialize)]
struct Record {
    fields: BTreeMap<String, RecordField>,
}

fn record() -> Record {
    load("testdata/mg90-d/set-of-record.json")
}

fn bound(set: &osc_measure::Set, field: &str) -> i64 {
    match set.get(field) {
        Some(Outcome::Bound(e)) => e.value,
        other => panic!("{field}: {other:?}"),
    }
}

fn held(set: &osc_measure::Set, field: &str) -> String {
    match set.get(field) {
        Some(Outcome::Held(Missing { input })) => input.clone(),
        other => panic!("{field} is not held: {other:?}"),
    }
}

fn refused(set: &osc_measure::Set, field: &str) -> String {
    match set.get(field) {
        Some(Outcome::Refused(r)) => r.why.clone(),
        other => panic!("{field} is not refused: {other:?}"),
    }
}

/// No result on record is the field's quantity, a driven ramp's
/// breakaway: the set of record holds its 0 with no source, and the rule
/// holds it the same way.
const HELD_OF_RECORD: &str = "fric_breakaway_counts";
/// Tuning parameters newer than the set of record: off unless the user
/// sets them.
const TUNING: [&str; 2] = ["pot_steady_counts_per_a", "pot_fast_counts_per_a"];

#[test]
fn rules_reproduce_the_set_of_record() {
    let set = build(&inputs());
    let record = record();
    assert_eq!(record.fields.len(), 63);
    let mut equal = 0;
    let mut wrong = Vec::new();
    for (name, want) in &record.fields {
        if name == HELD_OF_RECORD {
            assert_eq!(want.class, "none");
            assert_eq!(held(&set, name), "measured.breakaway_ma");
            continue;
        }
        match set.get(name) {
            Some(Outcome::Bound(e)) if e.value == want.value => equal += 1,
            other => wrong.push(format!("{name}: record {}, rules {other:?}", want.value)),
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
    assert_eq!(equal, 62);
    for name in TUNING {
        assert_eq!(bound(&set, name), 0);
    }
    assert_eq!(set.fields.len(), record.fields.len() + TUNING.len());

    // The CONFIG the bench ran is the set's.
    let gate: BTreeMap<String, serde_json::Value> = load("testdata/mg90-d/config-main.json");
    let mut config = 0;
    for (name, v) in gate.iter().filter(|(n, _)| *n != "source") {
        assert_eq!(Some(bound(&set, name)), v.as_i64(), "{name}");
        config += 1;
    }
    assert_eq!(config, 39);
}

/// A thermal band of record came from the winding measurement; a user
/// enters the value alone.
const THERMAL_BANDED: [&str; 3] = ["thermal_tau_s", "thermal_tau2_s", "th_rise_q016"];

#[test]
fn bands_reproduce() {
    let set = build(&inputs());
    let mut same = 0;
    for (name, want) in &record().fields {
        let got = match set.get(name) {
            Some(Outcome::Bound(e)) => e.band,
            _ => None,
        };
        if THERMAL_BANDED.contains(&name.as_str()) {
            assert!(want.band.is_some() && got != want.band, "{name}: {got:?}");
            continue;
        }
        assert_eq!(got, want.band, "{name}");
        same += want.band.is_some() as usize;
    }
    assert_eq!(same, 24);
}

#[test]
fn a_value_that_does_not_fit_is_refused_never_clamped() {
    let mut i = inputs();
    i.policy.current_loop_hz = Some(1e6);
    let set = build(&i);
    assert!(refused(&set, "i_kp_q88").contains("does not fit its 0..65535"));

    // Fits its u16 storage, over the firmware's own speed cap.
    let mut i = inputs();
    for row in &mut i.nameplate.no_load {
        row.seconds /= 5.0;
    }
    let set = build(&i);
    assert!(refused(&set, "speed_ceiling_cps").contains("does not fit its 0..32767"));

    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(500.0);
    assert!(refused(&build(&i), "velocity_limit_cps").contains("over speed_ceiling_cps 8050"));

    let mut i = inputs();
    if let Some(q) = i.measured.r_board_c.as_mut() {
        q.value = f64::NAN;
    }
    assert!(refused(&build(&i), "r_q12").contains("which no field holds"));
}

#[test]
fn a_missing_input_holds_the_field_and_names_it() {
    let mut i = inputs();
    i.measured.l_mh = None;
    let set = build(&i);
    assert_eq!(held(&set, "i_kp_q88"), "measured.l_mh");
    assert_eq!(held(&set, "v_kp_q88"), "measured.l_mh");
    assert_eq!(held(&set, "p_kp_q88"), "measured.l_mh");
    assert_eq!(bound(&set, "i_ki_q412"), 1176);

    let mut i = inputs();
    i.nameplate.stall.clear();
    i.nameplate.derate_speed = None;
    let set = build(&i);
    assert_eq!(held(&set, "current_limit_counts"), "nameplate.stall");
    assert_eq!(held(&set, "speed_ceiling_cps"), "nameplate.derate_speed");

    let mut i = inputs();
    i.thermal.slow_tau_s = None;
    i.thermal.slow_gain_c_per_w = None;
    let set = build(&i);
    assert_eq!(held(&set, "thermal_tau2_s"), "thermal.slow_tau_s");
    assert_eq!(held(&set, "th_rise2_q016"), "thermal.slow_gain_c_per_w");
    assert_eq!(bound(&set, "th_rise_q016"), 434);
    i.thermal = load("testdata/mg90-d/thermal.json");
    i.thermal.slow_tau_s = None;
    assert!(refused(&build(&i), "th_rise2_q016").contains("entered whole"));

    // No margin record: the speed gains wait for a closed-loop check,
    // which may only lower the starting gain.
    let mut i = inputs();
    i.measured.shelf = None;
    let set = build(&i);
    assert!(held(&set, "v_kp_q88").starts_with("measured.speed_kp_verified"));
    i.measured.speed_kp_verified = Some(50.0 / 256.0);
    let set = build(&i);
    assert_eq!(bound(&set, "v_kp_q88"), 50);
    assert!(bound(&set, "p_kp_q88") < 5026);
    i.measured.speed_kp_verified = Some(1.0);
    assert!(refused(&build(&i), "v_kp_q88").contains("only lowers it"));
}

/// The soft limits are an application's: a margin share of 0, or none,
/// leaves both at 0, off, whatever the stops; a share over 0 sits them
/// that share of the span inside each stop, as the set of record.
#[test]
fn no_soft_margin_leaves_the_soft_limits_off() {
    for share in [None, Some(0.0)] {
        let mut i = inputs();
        i.policy.soft_margin_share = share;
        let set = build(&i);
        assert_eq!(bound(&set, "pos_min_soft_counts"), 0, "{share:?}");
        assert_eq!(bound(&set, "pos_max_soft_counts"), 0, "{share:?}");
        assert_eq!(bound(&set, "pos_min_phys_counts"), 82);
        assert_eq!(bound(&set, "pos_max_phys_counts"), 3792);
        i.measured.stops = None;
        let set = build(&i);
        assert_eq!(bound(&set, "pos_min_soft_counts"), 0, "no stops needed");
        assert_eq!(bound(&set, "pos_max_soft_counts"), 0, "no stops needed");
    }
    let set = build(&inputs());
    assert_eq!(bound(&set, "pos_min_soft_counts"), 314);
    assert_eq!(bound(&set, "pos_max_soft_counts"), 3560);
    let mut i = inputs();
    i.policy.soft_margin_share = Some(0.125);
    let set = build(&i);
    assert_eq!(bound(&set, "pos_min_soft_counts"), 82 + 464);
    assert_eq!(bound(&set, "pos_max_soft_counts"), 3792 - 464);
}

#[test]
fn field_formats_match_the_descriptor() {
    #[derive(Deserialize)]
    struct Field {
        name: String,
        addr: u32,
        width: u32,
        access: String,
        kind: String,
        min: Option<i64>,
        max: Option<i64>,
        variants: Option<Vec<Variant>>,
    }
    #[derive(Deserialize)]
    struct Variant {
        name: String,
        value: i64,
    }
    #[derive(Deserialize)]
    struct Descriptor {
        fields: Vec<Field>,
    }
    const COMMS: [&str; 3] = ["id", "baud_rate_idx", "response_deadline_us"];
    const NOT_RULES: [&str; 1] = ["adc_wide_mid_q4"];
    const CONTROL_BASE: u32 = 0x180;

    let d: Descriptor = load("../descriptors/osc-servo/0.1.json");
    let writable: Vec<&Field> = d
        .fields
        .iter()
        .filter(|f| f.access == "rw" && f.addr < CONTROL_BASE)
        .filter(|f| !COMMS.contains(&f.name.as_str()) && !NOT_RULES.contains(&f.name.as_str()))
        .collect();
    assert_eq!(writable.len(), 65);
    let all: Vec<_> = rules().collect();
    assert_eq!(all.len(), writable.len());
    for f in writable {
        let mut of = all.iter().filter(|r| r.field == f.name);
        let (Some(r), None) = (of.next(), of.next()) else {
            panic!("{}: not one rule", f.name)
        };
        let kind = match r.format.kind {
            Kind::Bool => "bool",
            Kind::Enum(_) => "enum",
            k if k.signed() => "int",
            _ => "uint",
        };
        assert_eq!(
            (kind, r.format.kind.width()),
            (f.kind.as_str(), f.width),
            "{}",
            f.name
        );
        assert_eq!((r.format.min, r.format.max), (f.min, f.max), "{}", f.name);
        if let Kind::Enum(variants) = r.format.kind {
            let want: Vec<(&str, i64)> = f
                .variants
                .iter()
                .flatten()
                .map(|v| (v.name.as_str(), v.value))
                .collect();
            assert_eq!(variants, &want[..], "{}", f.name);
        }
    }
}

#[test]
fn release_sits_under_the_yield_current() {
    let set = build(&inputs());
    assert!(bound(&set, "stall_release_counts") < bound(&set, "yield_current_counts"));

    for release in [350.0, 400.0] {
        let mut i = inputs();
        i.policy.stall_release_ma = Some(release);
        assert!(
            refused(&build(&i), "stall_release_counts").contains("not under the yield current")
        );
    }
    // 0 reads as the cap.
    let mut i = inputs();
    i.policy.yield_current_ma = Some(0.0);
    i.policy.stall_release_ma = Some(400.0);
    assert_eq!(bound(&build(&i), "stall_release_counts"), 454);

    let mut i = inputs();
    i.policy.yield_current_ma = None;
    assert!(refused(&build(&i), "stall_release_counts").contains("not bound"));
}

/// A policy file written before the rename names the yield current
/// `i_cont_ma`; it still reads.
#[test]
fn a_policy_naming_the_old_key_still_reads() {
    let p: Policy = serde_json::from_str(r#"{"i_cont_ma": 350.0}"#).expect("parses");
    assert_eq!(p.yield_current_ma, Some(350.0));
}

#[test]
fn a_board_with_its_ntc_on_the_can_has_no_body_node() {
    let mut i = inputs();
    i.board.ntc_site = osc_measure::board::NtcSite::Can;
    assert!(refused(&build(&i), "thermal_tau2_s").contains("on the can"));
    i.thermal.slow_tau_s = None;
    i.thermal.slow_gain_c_per_w = None;
    let set = build(&i);
    assert_eq!(
        (bound(&set, "thermal_tau2_s"), bound(&set, "th_rise2_q016")),
        (0, 0)
    );
}
