//! The Default policy's rule forms (notebooks `default-policy.json`) over
//! the record's measured inputs: the rules give the values notebooks #178
//! pins on the record (`tests/test_constants.py`), and the record's own
//! policy, each rule-bound value fixed, still gives the set of record.

mod common;

use common::{inputs, text};
use osc_ident_inputs::{read, resolve, upgrade};
use osc_measure::nameplate::GearClass;
use osc_measure::policy::{StallRelease, Yield};
use osc_measure::{Inputs, Outcome, Quantity, Source, build};

/// The record's still-shaft speed, count/s: the largest observer speed
/// over 308 hold segments of the scorecard, read off a replay of the
/// observer by the bringup analysis of the protection screens. No step of
/// a run measures it yet.
const STILL_OF_THE_ANALYSIS: f64 = 115.0;
/// The worst 4-step pot run of the closed-loop rows of record, raw counts,
/// as notebooks `screens.step_run` reads it.
const STEP4_OF_RECORD: f64 = 15.0;

const LOOP_RATES: [&str; 6] = [
    "i_kp_q88",
    "i_ki_q412",
    "i_kaw_q412",
    "l1_q016",
    "l2_q88",
    "l3_q88",
];

fn term(value: f64, unit: &str, what: &str) -> Quantity {
    Quantity {
        value,
        lo: None,
        hi: None,
        sd: None,
        n: None,
        unit: unit.into(),
        source: Source {
            file: what.into(),
            key: String::new(),
            sha1: None,
        },
    }
}

/// The record's measured inputs under a version 1 policy file over the
/// record's profile, resolved as a run resolves them.
fn under(policy: &str) -> Inputs {
    let mut i = inputs();
    let u = upgrade(
        read::profile(&text("testdata/mg90-d/profile.json")).expect("profile"),
        Some(read::policy(&text(policy), "policy").expect("policy")),
    )
    .expect("upgrades");
    let r = resolve(&u.profile, &u.policy, Some(&i.board.facts())).expect("resolves");
    i.nameplate = r.nameplate;
    i.policy = r.policy;
    i
}

fn default() -> Inputs {
    under("testdata/mg90-d/default-policy.json")
}

fn bound(i: &Inputs, field: &str) -> i64 {
    match build(i).get(field) {
        Some(Outcome::Bound(e)) => e.value,
        other => panic!("{field}: {other:?}"),
    }
}

fn outcome(i: &Inputs, field: &str) -> Outcome {
    build(i).get(field).cloned().expect(field)
}

/// The loop rates over their plant corners, R / (2 pi L) and
/// Ke^2 / (2 pi J R) of record; metal with no yield torque, the cap and the
/// release at 0.36 of the cap in A; the stall speed and the pot jump at
/// four times what the analysis read.
#[test]
fn default_policy_on_the_record_inputs_gives_the_rule_values() {
    let mut i = default();
    let m = &i.measured;
    let (r, l) = (
        m.r_ohm.as_ref().unwrap().value,
        m.l_mh.as_ref().unwrap().value,
    );
    let (ke, j) = (
        m.ke_v_s_per_rad.as_ref().unwrap().value,
        m.j_kg_m2.as_ref().unwrap().value,
    );
    let tau = 2.0 * core::f64::consts::PI;
    assert_eq!((12.5 * r / (tau * l / 1000.0)).round() / 10.0, 1013.6);
    assert_eq!((190.0 * ke * ke / (tau * j * r)).round() / 100.0, 15.12);

    let rates: Vec<i64> = LOOP_RATES.iter().map(|f| bound(&i, f)).collect();
    assert_eq!(rates, [292, 1192, 2384, 8704, 3229, 86]);
    assert_eq!(bound(&i, "yield_current_counts"), 0);
    assert_eq!(bound(&i, "stall_release_counts"), 376);

    // neither the still speed nor the pot run is measured: the two fields
    // are held, naming the term, and the servo keeps what it holds
    for (field, term) in [
        ("stall_omega_max_cps", "measured.still_speed_cps"),
        ("sensor_delta_max", "measured.pot_step4_counts"),
    ] {
        match outcome(&i, field) {
            Outcome::Held(m) => assert_eq!(m.input, term, "{field}"),
            other => panic!("{field}: {other:?}"),
        }
    }
    i.measured.still_speed_cps = Some(term(STILL_OF_THE_ANALYSIS, "count/s", "the analysis"));
    i.measured.pot_step4_counts = Some(term(STEP4_OF_RECORD, "raw counts", "the rows of record"));
    assert_eq!(bound(&i, "stall_omega_max_cps"), 460);
    assert_eq!(bound(&i, "sensor_delta_max"), 60);

    // against the record's fixed values
    let rec = under("testdata/mg90-d/policy-of-record.json");
    let was: Vec<i64> = LOOP_RATES.iter().map(|f| bound(&rec, f)).collect();
    assert_eq!(was, [289, 1176, 2352, 8640, 3180, 84]);
    for (field, value) in [
        ("yield_current_counts", 397),
        ("stall_release_counts", 143),
        ("stall_omega_max_cps", 500),
        ("sensor_delta_max", 64),
    ] {
        assert_eq!(bound(&rec, field), value, "{field}");
    }
}

/// A yield torque is the torque ratio: half the 1.8 kg cm the sheet states
/// at 4.8 V is half of 4.8 V over R, on either train, and the release its
/// share of that. With no torque a plastic train takes the plastic torque
/// limit, which is not known yet, and a torque over the limit is refused
/// without `above_plastic_limit`.
#[test]
fn a_yield_torque_is_the_torque_ratio_and_a_plastic_train_takes_its_limit() {
    let base = default();
    let r = base.measured.r_ohm.as_ref().unwrap().value;
    let cpa = base.measured.current_counts_per_a.as_ref().unwrap().value;
    let counts = |a: f64| (a * cpa).round() as i64;
    for gear in [GearClass::Metal, GearClass::Plastic, GearClass::Unknown] {
        let mut i = default();
        i.nameplate.gear_class = Some(gear);
        i.policy.yield_ = Some(Yield::TorqueKgCm(Some(0.9)));
        assert_eq!(
            bound(&i, "yield_current_counts"),
            counts(0.5 * 4.8 / r),
            "{gear:?}"
        );
        assert_eq!(
            bound(&i, "stall_release_counts"),
            counts(0.36 * 0.5 * 4.8 / r),
            "{gear:?}"
        );
    }

    let mut soft = default();
    soft.nameplate.gear_class = Some(GearClass::Plastic);
    match outcome(&soft, "yield_current_counts") {
        Outcome::Held(m) => assert_eq!(m.input, "policy.plastic_torque_limit_kg_cm"),
        other => panic!("{other:?}"),
    }
    soft.policy.plastic_torque_limit_kg_cm = Some(0.6);
    assert_eq!(
        bound(&soft, "yield_current_counts"),
        counts(0.6 / 1.8 * 4.8 / r)
    );
    soft.policy.yield_ = Some(Yield::TorqueKgCm(Some(0.9)));
    match outcome(&soft, "yield_current_counts") {
        Outcome::Refused(why) => assert_eq!(
            why.why,
            "policy.yield: a yield torque of 0.9 kg cm is over the plastic torque limit, 0.6 kg cm, and above_plastic_limit is not true: plastic teeth strip there when the horn is blocked"
        ),
        other => panic!("{other:?}"),
    }
    soft.policy.above_plastic_limit = true;
    assert_eq!(bound(&soft, "yield_current_counts"), counts(0.5 * 4.8 / r));

    // the record's own fixed values go through as they are
    let mut metal = default();
    metal.policy.yield_ = Some(Yield::Ma(350.0));
    assert_eq!(bound(&metal, "yield_current_counts"), 397);
    assert_eq!(bound(&metal, "stall_release_counts"), counts(0.36 * 0.35));
    metal.policy.stall_release = Some(StallRelease::Ma(126.0));
    assert_eq!(bound(&metal, "stall_release_counts"), 143);
}

/// The record's own policy, every rule-bound value fixed, gives the set of
/// record's values on every field it binds; the Default moves only the six
/// rule-bound values and the loop gains they set.
#[test]
fn the_default_moves_only_what_its_rules_bind() {
    let rec = build(&under("testdata/mg90-d/policy-of-record.json"));
    let mut i = default();
    i.measured.still_speed_cps = Some(term(STILL_OF_THE_ANALYSIS, "count/s", "the analysis"));
    i.measured.pot_step4_counts = Some(term(STEP4_OF_RECORD, "raw counts", "the rows of record"));
    let def = build(&i);
    let mut moved: Vec<&str> = rec
        .fields
        .iter()
        .filter(|(f, _)| rec.value(f) != def.value(f))
        .map(|(f, _)| *f)
        .collect();
    moved.sort_unstable();
    assert_eq!(
        moved,
        [
            "i_kaw_q412",
            "i_ki_q412",
            "i_kp_q88",
            "l1_q016",
            "l2_q88",
            "l3_q88",
            "sensor_delta_max",
            "stall_omega_max_cps",
            "stall_release_counts",
            "yield_current_counts",
        ]
    );
}
