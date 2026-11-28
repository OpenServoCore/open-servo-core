//! The rules against the mg90-d set of record, fed the notebooks' results
//! of record (`testdata/mg90-d`, each value with its source file's sha1),
//! the board facts the bench servo publishes, and the record profile's heat
//! model.

use std::collections::BTreeMap;
use std::path::PathBuf;

use osc_measure::board;
use osc_measure::inputs::{Measured, ShelfRecord, ThermalInputs, Tuning};
use osc_measure::policy::{CurrentLoop, Observer, PolicyFile, PotJump, StallRelease, Yield};
use osc_measure::rules::{Kind, rules};
use osc_measure::{Inputs, Missing, Nameplate, Outcome, Policy, build};
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
        board: load::<board::Record>("testdata/mg90-d/board.json")
            .board()
            .expect("the board"),
        measured,
        nameplate: load::<Nameplate>("testdata/mg90-d/nameplate.json"),
        policy: load::<Policy>("testdata/mg90-d/policy.json"),
        thermal: thermal(),
        tuning: Tuning::default(),
    }
}

fn thermal() -> ThermalInputs {
    let p: osc_ident_inputs::ServoProfile = load("testdata/mg90-d/profile.json");
    let h = p.thermal.expect("the profile's heat model");
    ThermalInputs::of_profile(&h, "testdata/mg90-d/profile.json", "")
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
/// The soft limits of record were the policy's share of the stops' span
/// inside each stop, 314 / 3560; the rules now center the rated travel on
/// the first contacts (`the_soft_window_is_the_rated_travel_centered_on_the_contacts`).
const SOFT_SINCE_RECORD: [(&str, i64); 2] =
    [("pos_min_soft_counts", 126), ("pos_max_soft_counts", 3748)];
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
        if let Some((_, now)) = SOFT_SINCE_RECORD.iter().find(|(f, _)| f == name) {
            assert_eq!(bound(&set, name), *now, "{name}");
            continue;
        }
        match set.get(name) {
            Some(Outcome::Bound(e)) if e.value == want.value => equal += 1,
            other => wrong.push(format!("{name}: record {}, rules {other:?}", want.value)),
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
    assert_eq!(equal, 60);
    for name in TUNING {
        assert_eq!(bound(&set, name), 0);
    }
    assert_eq!(set.fields.len(), record.fields.len() + TUNING.len());

    // The CONFIG the bench ran is the set's.
    let gate: BTreeMap<String, serde_json::Value> = load("testdata/mg90-d/config-main.json");
    let mut config = 0;
    for (name, v) in gate.iter().filter(|(n, _)| *n != "source") {
        let want = match SOFT_SINCE_RECORD.iter().find(|(f, _)| f == name) {
            Some((_, now)) => Some(*now),
            None => v.as_i64(),
        };
        assert_eq!(Some(bound(&set, name)), want, "{name}");
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

fn clamp(set: &osc_measure::Set, field: &str) -> String {
    match set.get(field) {
        Some(Outcome::Bound(e)) => e
            .clamp
            .clone()
            .unwrap_or_else(|| panic!("{field}: no clamp")),
        other => panic!("{field}: {other:?}"),
    }
}

/// A value its storage cannot hold is refused, never stored at the end of
/// the width: a gain past it would be another loop.
#[test]
fn a_value_its_storage_cannot_hold_is_refused() {
    let mut i = inputs();
    i.policy.current_loop = Some(CurrentLoop::Hz(1e6));
    let set = build(&i);
    assert!(refused(&set, "i_kp_q88").contains("does not fit its 0..65535"));

    let mut i = inputs();
    if let Some(q) = i.measured.r_board_c.as_mut() {
        q.value = f64::NAN;
    }
    assert!(refused(&build(&i), "r_q12").contains("which no field holds"));
}

/// A value past the firmware's own bound on the field (`Format::within`)
/// is stored at that bound, and the set says what the rule gave: a speed
/// ceiling over the trajectory's speed cap.
#[test]
fn a_value_past_the_firmwares_bound_is_stored_at_it_and_said() {
    let mut i = inputs();
    for row in &mut i.nameplate.no_load {
        row.seconds /= 5.0;
    }
    let set = build(&i);
    assert_eq!(bound(&set, "speed_ceiling_cps"), 32767);
    let c = clamp(&set, "speed_ceiling_cps");
    assert!(
        c.starts_with("speed_ceiling.nameplate gives 40253.")
            && c.ends_with("past the firmware's 0..32767: stored as 32767"),
        "{c}"
    );
    assert!(set.clamps()[0].starts_with("speed_ceiling_cps: speed_ceiling.nameplate gives"));
    // the move speed is under the stored ceiling: nothing else moves
    assert_eq!(set.clamps().len(), 1, "{:?}", set.clamps());
    assert!(build(&inputs()).clamps().is_empty());
}

/// A move speed over the speed ceiling cannot act: it is stored at the
/// ceiling, and said.
#[test]
fn a_move_speed_over_the_ceiling_is_stored_at_the_ceiling() {
    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(500.0);
    let set = build(&i);
    assert_eq!(bound(&set, "speed_ceiling_cps"), 8050);
    assert_eq!(bound(&set, "velocity_limit_cps"), 8050);
    assert_eq!(
        clamp(&set, "velocity_limit_cps"),
        "the move speed gives 10063 count/s, over speed_ceiling_cps 8050, where it cannot act: stored as 8050"
    );
    // At the ceiling in deg/s it lands one count over: the ceiling, a
    // derated cap, rounds down, the move speed to the nearest.
    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(400.0);
    let set = build(&i);
    assert_eq!(bound(&set, "velocity_limit_cps"), 8050);
    assert_eq!(
        set.clamps(),
        [
            "velocity_limit_cps: the move speed gives 8051 count/s, over speed_ceiling_cps 8050, where it cannot act: stored as 8050"
        ]
    );
    i.policy.velocity_limit_deg_s = Some(399.9);
    assert!(build(&i).clamps().is_empty());
}

/// A current limit over the board's overcurrent trip cannot act, since the
/// trip fires first: it is stored at the trip, its band too, and the press
/// fold's release is then judged against the limit as stored.
#[test]
fn a_current_limit_over_the_board_trip_is_stored_at_the_trip() {
    let set = build(&inputs());
    assert_eq!(bound(&set, "current_limit_counts"), 1043);
    let mut i = inputs();
    i.board.oc_trip_ma = 800.0;
    let set = build(&i);
    let trip = bound(&set, "oc_trip_counts");
    assert_eq!(trip, 908);
    assert_eq!(bound(&set, "current_limit_counts"), trip);
    let Some(Outcome::Bound(e)) = set.get("current_limit_counts") else {
        panic!("bound")
    };
    assert!(
        e.band.is_some_and(|(lo, hi)| lo <= hi && hi <= trip),
        "{:?}",
        e.band
    );
    assert_eq!(
        clamp(&set, "current_limit_counts"),
        "the derate gives 1043 ccounts, over the board's overcurrent trip oc_trip_counts 908, which fires before the limit folds: stored as 908"
    );
    // the yield current 0 reads as the cap: the cap the servo holds
    i.policy.yield_ = Some(Yield::Ma(0.0));
    i.policy.stall_release = Some(StallRelease::Ma(850.0));
    let why = refused(&build(&i), "stall_release_counts");
    assert!(why.contains("not under the yield current 908"), "{why}");
}

/// The current limit is the derate of the stated stall volts over the
/// seated R: a stated stall current no longer binds it. The record states
/// none, so its limit is the record's either way.
#[test]
fn a_stated_stall_current_does_not_bind_the_current_limit() {
    let i = inputs();
    assert!(i.nameplate.stall.iter().all(|s| s.current_a.is_none()));
    let set = build(&i);
    assert_eq!(bound(&set, "current_limit_counts"), 1043);
    for a in [0.5, 2.0] {
        let mut i = inputs();
        for s in &mut i.nameplate.stall {
            s.current_a = Some(a);
        }
        let set = build(&i);
        assert_eq!(bound(&set, "current_limit_counts"), 1043, "{a} A");
        let Some(Outcome::Bound(e)) = set.get("current_limit_counts") else {
            panic!("bound")
        };
        assert!(e.inputs.contains(&"measured.r_ohm"), "{:?}", e.inputs);
    }
    let mut i = inputs();
    i.measured.r_ohm = None;
    i.nameplate.stall[0].current_a = Some(0.9);
    assert_eq!(held(&build(&i), "current_limit_counts"), "measured.r_ohm");
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
    i.thermal = thermal();
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

/// The record's stops as first contacts, 82 / 3792 table counts at
/// 20.1266 counts/deg: the rated 180 deg is 3622.8 counts centered on 1937,
/// more than K = 1.0 deg inside each contact. The window follows the
/// contacts, not the pins: pins moved out to the grid leave it where it
/// is. A narrower soft travel narrows it about the same middle; a wider one
/// that does not fit K inside each contact is clamped and said, as is one
/// under the least window, 2 K (oscnb `SOFT_FLOOR_MARGINS`). No contacts,
/// no soft limits.
#[test]
fn the_soft_window_is_the_rated_travel_centered_on_the_contacts() {
    let soft = |set: &osc_measure::Set| {
        (
            bound(set, "pos_min_soft_counts"),
            bound(set, "pos_max_soft_counts"),
        )
    };
    let mut i = inputs();
    assert_eq!(i.policy.soft_travel_deg, None);
    assert_eq!(i.nameplate.rated_travel_deg, Some(180.0));
    let set = build(&i);
    assert_eq!(soft(&set), (126, 3748));
    assert!(set.clamps().is_empty(), "{:?}", set.clamps());
    match set.get("pos_min_soft_counts") {
        Some(Outcome::Bound(e)) => assert_eq!(
            e.inputs,
            [
                "nameplate.rated_travel_deg",
                "measured.stops",
                "measured.table",
                "nameplate.brushes",
                "nameplate.rotor_segments",
                "nameplate.gear_ratio"
            ]
        ),
        other => panic!("{other:?}"),
    }

    let mut pinned = i.clone();
    if let Some(s) = pinned.measured.stops.as_mut() {
        (s.raw_min, s.raw_max) = (80, 3808);
    }
    let set = build(&pinned);
    assert_eq!(soft(&set), (126, 3748));
    assert_eq!(
        (
            bound(&set, "pos_min_phys_counts"),
            bound(&set, "pos_max_phys_counts")
        ),
        (80, 3808)
    );

    i.policy.soft_travel_deg = Some(90.0);
    let set = build(&i);
    // 45 deg of 20.1266 counts each side of 1937
    assert_eq!(soft(&set), (1031, 2843));
    assert!(set.clamps().is_empty());

    i.policy.soft_travel_deg = Some(185.0);
    let set = build(&i);
    assert_eq!(soft(&set), (102, 3772));
    assert!(clamp(&set, "pos_min_soft_counts").contains("clamped"));
    assert!(clamp(&set, "pos_max_soft_counts").contains("clamped"));

    i.policy.soft_travel_deg = Some(0.5);
    let set = build(&i);
    assert_eq!(soft(&set), (1917, 1957));
    assert!(clamp(&set, "pos_min_soft_counts").contains("under the least window, 2.00 deg"));

    let mut uncontacted = i.clone();
    if let Some(s) = uncontacted.measured.stops.as_mut() {
        s.contact_counts = None;
    }
    let set = build(&uncontacted);
    assert_eq!(
        held(&set, "pos_min_soft_counts"),
        "measured.stops.contact_counts"
    );

    i.measured.stops = None;
    let set = build(&i);
    assert_eq!(held(&set, "pos_min_soft_counts"), "measured.stops");
    assert_eq!(held(&set, "pos_max_soft_counts"), "measured.stops");
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
        i.policy.stall_release = Some(StallRelease::Ma(release));
        assert!(
            refused(&build(&i), "stall_release_counts").contains("not under the yield current")
        );
    }
    // 0 reads as the cap.
    let mut i = inputs();
    i.policy.yield_ = Some(Yield::Ma(0.0));
    i.policy.stall_release = Some(StallRelease::Ma(400.0));
    assert_eq!(bound(&build(&i), "stall_release_counts"), 454);

    let mut i = inputs();
    i.policy.yield_ = None;
    assert!(refused(&build(&i), "stall_release_counts").contains("not bound"));
}

/// A policy file written before the forms names each of the six values by
/// its unit, a bare number, and one before the rename names the yield
/// current `i_cont_ma`; each still reads, as the fixed form. A form is one
/// key of its two; anything else is refused naming both, as is a value
/// stated both ways.
#[test]
fn a_policy_naming_the_old_keys_still_reads() {
    let read = |text: &str| {
        serde_json::from_str::<PolicyFile>(text)
            .map_err(|e| e.to_string())
            .and_then(PolicyFile::policy)
    };
    assert_eq!(
        read(r#"{"i_cont_ma": 350.0}"#).map(|p| p.yield_),
        Ok(Some(Yield::Ma(350.0)))
    );
    let old = read(
        r#"{"observer_hz": 15.0, "current_loop_hz": 1000, "yield_current_ma": 350.0,
            "stall_release_ma": 126.0, "stall_omega_max_deg_s": 24.84,
            "sensor_delta_max_counts": 64}"#,
    )
    .expect("reads");
    let forms: Policy = serde_json::from_str(
        r#"{"observer": {"hz": 15.0}, "current_loop": {"hz": 1000}, "yield": {"ma": 350.0},
            "stall_release": {"ma": 126.0}, "stall_speed": {"deg_s": 24.84},
            "pot_jump": {"counts": 64}}"#,
    )
    .expect("parses");
    assert_eq!(old, forms);
    assert_eq!(old.observer, Some(Observer::Hz(15.0)));
    assert_eq!(old.pot_jump, Some(PotJump::Counts(64.0)));
    let rules: Policy = serde_json::from_str(
        r#"{"yield": {"torque_kg_cm": null}, "pot_jump": {"multiple_of_step4": 4}}"#,
    )
    .expect("parses");
    assert_eq!(rules.yield_, Some(Yield::TorqueKgCm(None)));
    assert_eq!(rules.pot_jump, Some(PotJump::MultipleOfStep4(4.0)));

    for (text, said) in [
        (
            r#"{"current_loop": 1000.0}"#,
            "expected one of {\"hz\": ...}, {\"multiple_of_electrical_corner\": ...}",
        ),
        (
            r#"{"observer": {"hz": 15.0, "x": 1}}"#,
            "a map of more keys than \"hz\": it is one of",
        ),
        (
            r#"{"pot_jump": {"multiple_of_step2": 4}}"#,
            "\"multiple_of_step2\": it is one of {\"counts\": ...}, {\"multiple_of_step4\": ...}",
        ),
        (
            r#"{"yield": {"a": 1}}"#,
            "\"a\": it is one of {\"ma\": ...}, {\"torque_kg_cm\": ...}",
        ),
        (r#"{"yield": {"ma": null}}"#, "\"ma\" with no value"),
    ] {
        let e = serde_json::from_str::<Policy>(text)
            .expect_err(text)
            .to_string();
        assert!(e.contains(said), "{text}: {e}");
    }
    let e = read(r#"{"observer": {"hz": 15}, "observer_hz": 15}"#).expect_err("both ways");
    assert!(e.starts_with("The policy states observer twice"), "{e}");
}

/// On a board whose NTC sits on the can, the NTC is the body node: the
/// profile's slow node binds 0 and the set says why; the rotor node stays.
#[test]
fn a_board_with_its_ntc_on_the_can_has_no_body_node() {
    let mut i = inputs();
    i.board.ntc_site = osc_measure::board::NtcSite::Can;
    let set = build(&i);
    assert_eq!(
        (bound(&set, "thermal_tau2_s"), bound(&set, "th_rise2_q016")),
        (0, 0)
    );
    assert_eq!(bound(&set, "th_rise_q016"), 434);
    let said = set.clamps();
    for f in ["thermal_tau2_s", "th_rise2_q016"] {
        assert!(
            said.iter()
                .any(|c| c.starts_with(&format!("{f}: 0")) && c.contains("sits on the can")),
            "{f}: {said:?}"
        );
    }
    i.thermal.slow_tau_s = None;
    i.thermal.slow_gain_c_per_w = None;
    let set = build(&i);
    assert_eq!(
        (bound(&set, "thermal_tau2_s"), bound(&set, "th_rise2_q016")),
        (0, 0)
    );
    assert!(set.clamps().is_empty(), "{:?}", set.clamps());
}

/// A gain over 0 that rounds to a rise of 0 would turn its node off, so it
/// is refused; a time constant past the estimate's longest leak is stored
/// as given.
#[test]
fn a_heat_node_never_stores_off_from_a_gain_over_0() {
    let mut i = inputs();
    i.thermal.fast_gain_c_per_w.as_mut().expect("gain").value = 0.001;
    let e = refused(&build(&i), "th_rise_q016");
    assert!(e.contains("the node would be off"), "{e}");
    let mut i = inputs();
    i.thermal.slow_tau_s.as_mut().expect("tau").value = 900.0;
    assert_eq!(bound(&build(&i), "thermal_tau2_s"), 900);
}

/// The resolve line's longest leak is the kernel's: 2^15 SLOW periods.
#[test]
fn the_longest_leak_a_profile_line_names_is_the_kernels() {
    use osc_servo_core::kernel::phases::loops::limits::{SLOW_PERIOD_MS, TAU_SHIFT_MAX};
    let ms = (1u64 << TAU_SHIFT_MAX) * u64::from(SLOW_PERIOD_MS);
    assert_eq!(
        (ms / 1000) as f64,
        osc_ident_inputs::resolve::LONGEST_LEAK_S
    );
}
