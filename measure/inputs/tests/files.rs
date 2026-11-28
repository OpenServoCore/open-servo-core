//! The profile and policy files: the record's at both versions, the
//! migration of version 0, the refusals, and what `resolve` gives.

use osc_ident_inputs::migrate::{PolicyIn, ProfileIn, slug};
use osc_ident_inputs::nameplate::GearClass;
use osc_ident_inputs::policy::{StallResponse, Switch};
use osc_ident_inputs::profile::{FieldSource, HeatModel};
use osc_ident_inputs::resolve::{Form, Value};
use osc_ident_inputs::tuning::{
    CurrentLimit, CurrentLoop, MoveSpeed, Observer, PotJump, SpeedCeiling, SpeedUp, StallRelease,
    StallSpeed, Yield,
};
use osc_ident_inputs::{
    BoardFacts, ServoProfile, TuningPolicy, read, resolve, upgrade, upgrade_with, version,
};

const NAMEPLATE: &str = include_str!("../../testdata/mg90-d/nameplate.json");
const POLICY: &str = include_str!("../../testdata/mg90-d/policy.json");
const PROFILE: &str = include_str!("../../testdata/mg90-d/profile.json");
const RECORD_POLICY: &str = include_str!("../../testdata/mg90-d/policy-of-record.json");
const DEFAULT_POLICY: &str = include_str!("../../testdata/mg90-d/default-policy.json");
const THERMAL: &str = include_str!("../../testdata/mg90-d/thermal.json");
const RECORD_HEAT: HeatModel = HeatModel {
    fast_tau_s: 2.3752191114749404,
    fast_gain_c_per_w: 20.44571009950305,
    slow_tau_s: 128.92,
    slow_gain_c_per_w: 76.7616,
};
const BOARD: BoardFacts = BoardFacts {
    tick_hz: 20_000.0,
    oc_trip_us: Some(400.0),
};

fn v0() -> (ProfileIn, PolicyIn) {
    (
        read::profile(NAMEPLATE).expect("nameplate"),
        read::policy(POLICY, "policy").expect("policy"),
    )
}

fn v1() -> (ServoProfile, TuningPolicy) {
    let ProfileIn::V1(p) = read::profile(PROFILE).expect("profile") else {
        panic!("version 1")
    };
    let PolicyIn::V1(q) = read::policy(RECORD_POLICY, "unused").expect("policy") else {
        panic!("version 1")
    };
    (p, *q)
}

/// Every value of the version 0 pair lands in its version 1 place: the
/// nameplate's run voltage and derates in the policy, its name in the key,
/// its source as provenance; the policy's overcurrent window leaves it.
#[test]
fn each_version_0_field_moves_to_its_version_1_place() {
    let (np, pol) = v0();
    let from = FieldSource {
        how: Some("file".into()),
        file: Some("thermal.json".into()),
        ..FieldSource::default()
    };
    let heat = read::thermal(THERMAL).expect("thermal.json");
    let u = upgrade_with(np, Some(pol), Some((heat, from.clone()))).expect("upgrades");
    assert_eq!(u.profile.thermal, Some(RECORD_HEAT));
    assert_eq!(u.profile.provenance.fields["thermal"], from);
    assert!(
        u.notes[0].ends_with("and the thermal.json beside it as its thermal"),
        "{:?}",
        u.notes
    );
    let (p, q) = (&u.profile, &u.policy);
    assert_eq!(p.key, "mg90-d");
    assert_eq!(p.model.name, "mg90-d");
    assert_eq!(p.gear_ratio, Some(308.0533));
    assert_eq!((p.brushes, p.rotor_segments), (Some(2), Some(3)));
    assert_eq!(p.stall.len(), 2);
    assert!(
        p.provenance
            .document
            .as_deref()
            .unwrap()
            .starts_with("Vorpal Robotics wiki")
    );
    let f = &p.provenance.fields;
    assert_eq!(
        f["source"].sha1.as_deref(),
        Some("9b890b42877683522fdb9909bdc5bdafe0a7bd4e")
    );
    assert!(
        f["rated_travel_deg"]
            .note
            .as_deref()
            .is_some_and(|n| n.starts_with("the MG90 is sold as a 180 deg servo"))
    );
    assert_eq!(p.rated_travel_deg, Some(180.0));
    assert_eq!(
        f["registry"].file.as_deref(),
        Some("notebooks/oscnb/servos.py")
    );

    let r = &q.protection;
    assert_eq!(r.run_v, Some(5.0));
    assert_eq!(r.speed_ceiling, Some(SpeedCeiling::ShareOfNoLoad([2, 3])));
    assert_eq!(r.current_limit, Some(CurrentLimit::ShareOfStall(0.8)));
    assert_eq!(r.yield_, Some(Yield::Ma(350.0)));
    assert_eq!(r.stall_release, Some(StallRelease::Ma(126.0)));
    assert_eq!(r.stall_response, Some(StallResponse::Yield));
    assert_eq!(r.stall_time_ms, Some(500.0));
    assert_eq!(r.stall_speed, Some(StallSpeed::DegS(24.84)));
    assert_eq!(r.cutoff_c, Some(90.0));
    let e = r.position_error.expect("position error");
    assert_eq!((e.deg, e.time_ms), (Some(9.9), Some(100.0)));
    assert_eq!(r.duty_max_share, Some(1.0));
    assert_eq!(r.collision_trip, Some(Switch::Off));
    assert_eq!(q.motion.move_speed, Some(MoveSpeed::DegS(200.0)));
    assert_eq!(q.motion.speed_up, Some(SpeedUp::DegS2(4000.0)));
    assert_eq!(q.motion.soft_travel_deg, None);
    assert_eq!(q.motion.center_offset_deg, None);
    assert_eq!(q.loops.current_loop, Some(CurrentLoop::Hz(1000.0)));
    assert_eq!(q.loops.observer, Some(Observer::Hz(15.0)));
    assert_eq!(q.loops.position_split, Some(7.104));
    assert_eq!(q.loops.hold_band_floor_counts, Some(4.0));
    assert_eq!(q.sensor.pot_jump, Some(PotJump::Counts(64.0)));
    assert_eq!(q.sensor.bad_samples, Some(4.0));
    assert_eq!(q.behavior.brush_term, Some(Switch::Off));
    assert_eq!(
        q.source.as_ref().and_then(|s| s.sha1.as_deref()),
        Some("06b6b91594ab18cfb66f785e121c975196361bce")
    );
    assert_eq!(u.notes.len(), 2, "{:?}", u.notes);
    assert!(u.notes[1].contains("oc_trip_samples 8"), "{:?}", u.notes);

    // the migrated pair is the record's version 1 pair but for what
    // version 0 never said
    let (rp, rq) = v1();
    let mut p = u.profile.clone();
    p.key = rp.key.clone();
    p.revision = rp.revision;
    p.model = rp.model.clone();
    p.stall = rp.stall.clone();
    p.output_stopper = rp.output_stopper;
    p.provenance = rp.provenance.clone();
    assert_eq!(p, rp);
    let mut q = u.policy.clone();
    q.key = rq.key.clone();
    q.label = rq.label.clone();
    q.source = rq.source.clone();
    assert_eq!(q, rq);
}

/// Version 0 and version 1 of the record resolve to the same absolute
/// values: what the version 0 files said, the overcurrent window now the
/// board's.
#[test]
fn both_versions_of_the_record_resolve_to_the_version_0_values() {
    let (np, pol) = v0();
    let u = upgrade(np.clone(), Some(pol.clone())).expect("upgrades");
    let from_v0 = resolve(&u.profile, &u.policy, Some(&BOARD)).expect("resolves");
    let (p, q) = v1();
    let from_v1 = resolve(&p, &q, Some(&BOARD)).expect("resolves");
    let (ProfileIn::V0 { nameplate, .. }, PolicyIn::V0 { policy, .. }) = (np, pol) else {
        panic!("version 0")
    };
    assert_eq!(from_v0.policy, *policy);
    assert_eq!(from_v1.policy, *policy);
    for n in [&from_v0.nameplate, &from_v1.nameplate] {
        assert_eq!(n.run_v, nameplate.run_v);
        assert_eq!(n.derate_speed, nameplate.derate_speed);
        assert_eq!(n.derate_other, nameplate.derate_other);
        assert_eq!(n.no_load, nameplate.no_load);
        let volts = |s: &[osc_ident_inputs::nameplate::Stall]| -> Vec<(f64, Option<f64>)> {
            s.iter().map(|r| (r.at_v, r.current_a)).collect()
        };
        assert_eq!(volts(&n.stall), volts(&nameplate.stall));
        assert_eq!(n.document, nameplate.document);
        assert_eq!(n.rated_travel_deg, nameplate.rated_travel_deg);
    }
    assert_eq!(from_v0.nameplate.stall, nameplate.stall);
    // the profile states the stall torque the yield torque is taken over
    let torques: Vec<Option<f64>> = from_v1
        .nameplate
        .stall
        .iter()
        .map(|s| s.torque_kg_cm)
        .collect();
    assert_eq!(torques, [Some(1.8), Some(2.2)]);
    assert_eq!(from_v1.nameplate.name, "vorpal-mg90");
    let oc = from_v1
        .report
        .fields
        .iter()
        .find(|l| l.field == "oc_trip")
        .expect("oc_trip");
    assert_eq!(
        (oc.form.clone(), oc.value.clone()),
        (Form::Board, Value::Number(8.0))
    );
    let without = resolve(&p, &q, None).expect("resolves");
    assert_eq!(without.policy.oc_trip_samples, None);
}

/// The shares resolve from the profile alone, as the record's round trip:
/// two thirds of 600 deg/s, half of that, reached in 0.05 s.
#[test]
fn the_shares_resolve_to_the_records_values() {
    let (p, mut q) = v1();
    q.motion.move_speed = Some(MoveSpeed::ShareOfCeiling(0.5));
    q.motion.speed_up = Some(SpeedUp::S(0.05));
    let r = resolve(&p, &q, Some(&BOARD)).expect("resolves");
    assert_eq!(r.policy.velocity_limit_deg_s, Some(200.0));
    assert_eq!(r.policy.accel_limit_deg_s2, Some(4000.0));
    let ceiling = r
        .report
        .fields
        .iter()
        .find(|l| l.field == "protection.speed_ceiling")
        .unwrap();
    assert_eq!(
        ceiling.note.as_deref(),
        Some("400 deg/s at the 4.8 V rating")
    );
    q.motion.move_speed = None;
    let e = resolve(&p, &q, Some(&BOARD)).expect_err("a time with no move speed");
    assert!(e.contains("motion.move_speed"), "{e}");
}

/// A move speed over the speed ceiling cannot act: it resolves to the
/// ceiling, the speed-up time reaches that, and the report's line says
/// what was stated and what is stored.
#[test]
fn a_move_speed_over_the_ceiling_resolves_to_the_ceiling_and_says_so() {
    let (p, mut q) = v1();
    let line = |r: &osc_ident_inputs::Resolved| {
        r.report
            .fields
            .iter()
            .find(|l| l.field == "motion.move_speed")
            .cloned()
            .unwrap()
    };
    q.motion.move_speed = Some(MoveSpeed::ShareOfCeiling(1.5));
    q.motion.speed_up = Some(SpeedUp::S(0.05));
    let r = resolve(&p, &q, Some(&BOARD)).expect("resolves");
    assert_eq!(r.policy.velocity_limit_deg_s, Some(400.0));
    assert_eq!(r.policy.accel_limit_deg_s2, Some(8000.0));
    let l = line(&r);
    assert_eq!(l.note.as_deref(), Some("600 deg/s"));
    assert_eq!(
        l.clamp.as_deref(),
        Some(
            "600 deg/s is over the 400 deg/s speed ceiling, where it cannot act: stored as 400 deg/s"
        )
    );
    assert_eq!(
        r.report.clamps(),
        [
            "motion.move_speed: 600 deg/s is over the 400 deg/s speed ceiling, where it cannot act: stored as 400 deg/s"
        ]
    );

    q.motion.move_speed = Some(MoveSpeed::DegS(500.0));
    let r = resolve(&p, &q, Some(&BOARD)).expect("resolves");
    assert_eq!(r.policy.velocity_limit_deg_s, Some(400.0));
    assert!(line(&r).clamp.is_some());

    for under in [MoveSpeed::DegS(400.0), MoveSpeed::ShareOfCeiling(0.75)] {
        q.motion.move_speed = Some(under);
        let r = resolve(&p, &q, Some(&BOARD)).expect("resolves");
        assert!(r.report.clamps().is_empty(), "{under:?}");
        let json = serde_json::to_value(&r.report).expect("json");
        assert!(
            json["fields"]
                .as_array()
                .unwrap()
                .iter()
                .all(|l| l.get("clamp").is_none())
        );
    }
}

/// The current limit resolves to the stated volts over the identified R
/// whether or not the profile states a stall current; a stated one is
/// named as reported beside it, never as what binds.
#[test]
fn the_current_limit_is_the_stated_volts_over_r_a_stated_current_beside_it() {
    let (mut p, q) = v1();
    let note = |p: &ServoProfile| {
        resolve(p, &q, Some(&BOARD))
            .expect("resolves")
            .report
            .fields
            .iter()
            .find(|l| l.field == "protection.current_limit")
            .and_then(|l| l.note.clone())
            .unwrap()
    };
    let vr = "of the 4.8 V rating over the identified R, set by the run";
    assert_eq!(note(&p), vr);
    for s in &mut p.stall {
        s.current_a = Some(0.9);
    }
    assert_eq!(
        note(&p),
        format!("{vr}; the 0.9 A the profile states there is reported beside it")
    );
}

/// The Default policy's rule forms resolve as they are stated, each line
/// saying what the run sets it from; on metal no yield torque is the cap.
#[test]
fn the_default_policys_rule_forms_resolve_to_what_the_run_sets() {
    let (p, _) = v1();
    let PolicyIn::V1(q) = read::policy(DEFAULT_POLICY, "unused").expect("policy") else {
        panic!("version 1")
    };
    let r = resolve(&p, &q, Some(&BOARD)).expect("resolves");
    let o = &r.policy;
    assert_eq!(o.yield_, Some(Yield::TorqueKgCm(None)));
    assert_eq!(o.stall_release, Some(StallRelease::ShareOfYield(0.36)));
    assert_eq!(o.stall_speed, Some(StallSpeed::MultipleOfStillSpeed(4.0)));
    assert_eq!(
        o.current_loop,
        Some(CurrentLoop::MultipleOfElectricalCorner(1.25))
    );
    assert_eq!(o.observer, Some(Observer::MultipleOfMechanicalCorner(1.9)));
    assert_eq!(o.pot_jump, Some(PotJump::MultipleOfStep4(4.0)));
    let line = |f: &str| r.report.fields.iter().find(|l| l.field == f).unwrap();
    let y = line("protection.yield");
    assert_eq!(
        (&y.form, &y.value, y.note.as_deref()),
        (
            &Form::Share,
            &Value::Text("the cap".into()),
            Some("no yield torque on a metal train: the cap, no fold")
        )
    );
    for f in [
        "protection.stall_release",
        "protection.stall_speed",
        "loops.current_loop",
        "loops.observer",
        "sensor.pot_jump",
    ] {
        let l = line(f);
        assert_eq!(l.form, Form::Share, "{f}");
        assert!(
            l.note.as_deref().unwrap().ends_with("set by the run"),
            "{f}"
        );
    }

    // a yield torque is the torque ratio, and at or over the current
    // limit's share of the stall torque the press never trips
    let mut t = (*q).clone();
    t.protection.yield_ = Some(Yield::TorqueKgCm(Some(0.9)));
    let r = resolve(&p, &t, Some(&BOARD)).expect("resolves");
    let l = r
        .report
        .fields
        .iter()
        .find(|l| l.field == "protection.yield")
        .unwrap();
    assert_eq!(
        l.note.as_deref(),
        Some(
            "over the 1.8 kg cm stall torque at 4.8 V, times 4.8 V over the identified R, set by the run"
        )
    );
    t.protection.yield_ = Some(Yield::TorqueKgCm(Some(1.5)));
    let r = resolve(&p, &t, Some(&BOARD)).expect("resolves");
    let l = r
        .report
        .fields
        .iter()
        .find(|l| l.field == "protection.yield")
        .unwrap();
    assert!(
        l.note
            .as_deref()
            .unwrap()
            .ends_with("the press never trips"),
        "{l:?}"
    );

    // a plastic train takes the plastic torque limit, not known yet, and a
    // torque over it only with the checkbox
    let mut soft = p.clone();
    soft.gear_class = Some(GearClass::Plastic);
    let e = resolve(&soft, &q, Some(&BOARD)).expect_err("not known");
    assert_eq!(
        e,
        "The yield current a stalled horn folds to cannot be set: the plastic torque limit, the yield torque of a plastic train, is not known yet"
    );
    let mut t = (*q).clone();
    t.protection.plastic_torque_limit_kg_cm = Some(0.6);
    let r = resolve(&soft, &t, Some(&BOARD)).expect("the limit");
    let l = r
        .report
        .fields
        .iter()
        .find(|l| l.field == "protection.yield")
        .unwrap();
    assert_eq!(l.value, Value::Number(0.6));
    assert!(
        l.note
            .as_deref()
            .unwrap()
            .starts_with("the plastic torque limit, over the 1.8 kg cm")
    );
    t.protection.yield_ = Some(Yield::TorqueKgCm(Some(0.9)));
    let e = resolve(&soft, &t, Some(&BOARD)).expect_err("over the limit");
    assert!(
        e.ends_with("plastic teeth strip there when the horn is blocked"),
        "{e}"
    );
    t.protection.above_plastic_limit = true;
    resolve(&soft, &t, Some(&BOARD)).expect("with the checkbox");

    // a torque, a share or a multiple of 0 or under is refused
    type Set = fn(&mut TuningPolicy);
    let cases: [(&str, Set); 6] = [
        ("protection.yield", |q| {
            q.protection.yield_ = Some(Yield::TorqueKgCm(Some(0.0)))
        }),
        ("protection.stall_release", |q| {
            q.protection.stall_release = Some(StallRelease::ShareOfYield(-0.1))
        }),
        ("protection.stall_speed", |q| {
            q.protection.stall_speed = Some(StallSpeed::MultipleOfStillSpeed(0.0))
        }),
        ("loops.current_loop", |q| {
            q.loops.current_loop = Some(CurrentLoop::MultipleOfElectricalCorner(f64::NAN))
        }),
        ("loops.observer", |q| {
            q.loops.observer = Some(Observer::MultipleOfMechanicalCorner(0.0))
        }),
        ("sensor.pot_jump", |q| {
            q.sensor.pot_jump = Some(PotJump::MultipleOfStep4(0.0))
        }),
    ];
    for (field, set) in cases {
        let mut t = (*q).clone();
        set(&mut t);
        let e = resolve(&p, &t, Some(&BOARD)).expect_err(field);
        assert!(e.contains(&format!("the policy's {field} is")), "{e}");
    }
}

/// Each fact the run needs, taken out of the profile, is refused naming
/// the field, the consequence first.
#[test]
fn each_required_profile_field_is_refused_with_its_message() {
    let (p, q) = v1();
    assert!(p.check().is_ok() && q.check().is_ok());
    type Take = fn(&mut ServoProfile);
    let cases: [(&str, &str, Take); 12] = [
        ("gear_class", "Without the gear material", |p| {
            p.gear_class = None
        }),
        (
            "gear_ratio",
            "Without the gear ratio the run cannot turn motor turns into horn degrees",
            |p| p.gear_ratio = Some(0.0),
        ),
        (
            "rated_travel_deg",
            "The run sets the soft limits to the rated travel",
            |p| p.rated_travel_deg = Some(361.0),
        ),
        (
            "count_rises_ccw",
            "Without it the run cannot tell which stop is which",
            |p| p.count_rises_ccw = None,
        ),
        (
            "brushes and rotor_segments",
            "Without both counts the run cannot measure the angle table",
            |p| p.brushes = None,
        ),
        ("brushes and rotor_segments", "Without both counts", |p| {
            p.rotor_segments = Some(0)
        }),
        ("operating_v", "The run needs the maker's range", |p| {
            p.operating_v = None
        }),
        (
            "no_load",
            "The speed ceiling that protects the brushes cannot be set",
            |p| p.no_load.clear(),
        ),
        ("no_load", "The speed ceiling", |p| {
            p.no_load.iter_mut().for_each(|r| r.at_v = 9.0)
        }),
        ("stall", "The current limit cannot be set", |p| {
            p.stall.clear()
        }),
        (
            "rated_travel_deg",
            "The run sets the soft limits to the rated travel",
            |p| p.rated_travel_deg = None,
        ),
        ("key", "The key names", |p| p.key = "Vorpal MG90".into()),
    ];
    for (field, consequence, take) in cases {
        let mut p = p.clone();
        take(&mut p);
        let e = resolve(&p, &q, None).expect_err(field);
        assert!(e.starts_with(consequence), "{field}: {e}");
        assert!(e.contains(field), "{field}: {e}");
    }
    type TakeQ = fn(&mut TuningPolicy);
    let cases: [(&str, TakeQ); 3] = [
        ("protection.run_v", |q| q.protection.run_v = None),
        ("protection.speed_ceiling", |q| {
            q.protection.speed_ceiling = Some(SpeedCeiling::ShareOfNoLoad([4, 3]))
        }),
        ("protection.current_limit", |q| {
            q.protection.current_limit = None
        }),
    ];
    for (field, take) in cases {
        let mut q = q.clone();
        take(&mut q);
        let e = resolve(&p, &q, None).expect_err(field);
        assert!(
            e.contains(&format!("the policy has no usable {field} (")),
            "{e}"
        );
    }
}

/// A file newer than this tool, or of another kind, is refused naming its
/// schema; a version 1 file with a key this version does not know is
/// refused naming the key.
#[test]
fn a_file_with_a_higher_version_is_refused() {
    let newer = PROFILE.replacen("osc.servo-profile/1", "osc.servo-profile/2", 1);
    let e = read::profile(&newer).expect_err("newer");
    assert!(e.contains("version 2") && e.contains("up to 1"), "{e}");
    let e = read::policy(&RECORD_POLICY.replacen("/1\"", "/7\"", 1), "k").expect_err("newer");
    assert!(e.contains("osc.tuning-policy version 7"), "{e}");
    let e = read::profile(RECORD_POLICY).expect_err("a policy is no profile");
    assert!(e.contains("not a osc.servo-profile file"), "{e}");
    assert_eq!(version(None, "osc.servo-profile/1"), Ok(0));
    let e = read::profile(&PROFILE.replacen("\"brushes\"", "\"brush\"", 1)).expect_err("unknown");
    assert!(e.contains("unknown field `brush`"), "{e}");
}

/// A version 0 nameplate beside a version 1 policy: the policy's choices
/// bind; a choice both state differently is refused.
#[test]
fn a_version_0_nameplate_beside_a_version_1_policy_must_agree_with_it() {
    let (_, q) = v1();
    let np = read::profile(NAMEPLATE).expect("nameplate");
    let u = upgrade(np.clone(), Some(PolicyIn::V1(Box::new(q.clone())))).expect("they agree");
    assert_eq!(u.policy, q);
    let mut other = q.clone();
    other.protection.run_v = Some(6.0);
    let e = upgrade(np.clone(), Some(PolicyIn::V1(Box::new(other)))).expect_err("they disagree");
    assert!(e.contains("run_v 5") && e.contains("the policy 6"), "{e}");
    let mut less = q;
    less.protection.current_limit = None;
    let u = upgrade(np.clone(), Some(PolicyIn::V1(Box::new(less)))).expect("filled");
    assert_eq!(
        u.policy.protection.current_limit,
        Some(CurrentLimit::ShareOfStall(0.8))
    );
    // alone it carries only its own three choices, all the steps read
    let u = upgrade(np, None).expect("alone");
    let r = resolve(&u.profile, &u.policy, None).expect("resolves");
    assert_eq!(r.nameplate.run_v, Some(5.0));
    assert_eq!(r.policy.yield_, None);
}

#[test]
fn a_name_becomes_a_key() {
    assert_eq!(slug("mg90-d"), "mg90-d");
    assert_eq!(slug("  Vorpal MG90 (metal)! "), "vorpal-mg90-metal");
    assert_eq!(slug(&"a".repeat(50)).len(), 40);
}

/// The record profile states the record's heat model, and it resolves to a
/// line a value, each from the profile, not identified.
#[test]
fn the_profiles_heat_model_resolves_to_four_profile_lines() {
    let (p, q) = v1();
    assert_eq!(p.thermal, Some(RECORD_HEAT));
    let r = resolve(&p, &q, Some(&BOARD)).expect("resolves");
    assert_eq!(r.thermal, Some(RECORD_HEAT));
    let lines: Vec<_> = r
        .report
        .fields
        .iter()
        .filter(|l| l.form == Form::Profile && l.field.starts_with("thermal."))
        .collect();
    let named: Vec<_> = lines.iter().map(|l| (l.field, l.unit)).collect();
    assert_eq!(
        named,
        [
            ("thermal.fast_tau_s", "s"),
            ("thermal.fast_gain_c_per_w", "C/W"),
            ("thermal.slow_tau_s", "s"),
            ("thermal.slow_gain_c_per_w", "C/W"),
        ]
    );
    assert_eq!(lines[0].value, Value::Number(2.3752191114749404));
    for l in &lines {
        assert!(
            l.note
                .as_deref()
                .is_some_and(|n| n.starts_with("from the profile, not identified")),
            "{l:?}"
        );
        assert_eq!(l.clamp, None);
    }
    assert!(
        lines[3]
            .note
            .as_deref()
            .unwrap()
            .ends_with("the run sets th_rise2_q016 from it over the identified R")
    );
    assert!(r.report.clamps().is_empty(), "{:?}", r.report.clamps());

    // past the estimate's longest leak it is stored as given, and said
    let mut long = p.clone();
    long.thermal.as_mut().unwrap().slow_tau_s = 900.0;
    let r = resolve(&long, &q, Some(&BOARD)).expect("resolves");
    let l = r
        .report
        .fields
        .iter()
        .find(|l| l.field == "thermal.slow_tau_s")
        .unwrap();
    assert_eq!(l.value, Value::Number(900.0));
    assert!(
        l.note.as_deref().unwrap().contains("runs at 524 s"),
        "{l:?}"
    );

    // none states no line
    let mut none = p;
    none.thermal = None;
    let r = resolve(&none, &q, Some(&BOARD)).expect("resolves");
    assert!(
        !r.report
            .fields
            .iter()
            .any(|l| l.form == Form::Profile && l.field.starts_with("thermal."))
    );
}

/// A time constant under 0.5 s stores as 0 and turns its node off, so the
/// profile is refused; a gain of 0 likewise.
#[test]
fn a_heat_value_the_estimate_cannot_run_is_refused() {
    let (p, _) = v1();
    let mut short = p.clone();
    short.thermal.as_mut().unwrap().slow_tau_s = 0.3;
    let e = short.check().expect_err("0.3 s");
    assert!(
        e.contains("thermal.slow_tau_s is 0.3 s") && e.contains("at least 0.5 s"),
        "{e}"
    );
    let mut flat = p.clone();
    flat.thermal.as_mut().unwrap().fast_gain_c_per_w = 0.0;
    let e = flat.check().expect_err("no gain");
    assert!(e.contains("each node's gain over 0"), "{e}");
    let mut half = p;
    half.thermal.as_mut().unwrap().fast_tau_s = 0.5;
    half.check().expect("0.5 s");
    let e = osc_ident_inputs::no_heat_model();
    for f in [
        "fast_tau_s",
        "fast_gain_c_per_w",
        "slow_tau_s",
        "slow_gain_c_per_w",
    ] {
        assert!(e.contains(f), "{e}");
    }
}

/// The version 0 thermal.json reads as the record's four values; one value
/// short is refused naming it, and a version 1 profile, which states its
/// own, takes none beside it.
#[test]
fn a_version_0_thermal_json_folds_into_the_profile_only_beside_a_version_0_nameplate() {
    assert_eq!(read::thermal(THERMAL), Ok(RECORD_HEAT));
    let short = THERMAL.replacen("\"slow_tau_s\"", "\"slow_tau\"", 1);
    let e = read::thermal(&short).expect_err("short");
    assert!(e.contains("has no slow_tau_s"), "{e}");
    let profile = read::profile(PROFILE).expect("profile");
    let e = upgrade_with(profile, None, Some((RECORD_HEAT, FieldSource::default())))
        .expect_err("a version 1 profile states its own");
    assert!(e.contains("one source"), "{e}");
    let (np, pol) = v0();
    let u = upgrade(np, Some(pol)).expect("without one");
    assert_eq!(u.profile.thermal, None);
}

const STOP_TO_STOP: &str = include_str!("../../testdata/mg90-d/nameplate-stop-to-stop.json");

/// A version 1 profile that states travel_deg, the stop-to-stop travel, is
/// refused in plain words naming the rated travel to enter, not serde's
/// unknown field; the field never serializes.
#[test]
fn a_profile_stating_travel_deg_is_refused_naming_the_rated_travel() {
    let with = PROFILE.replacen(
        "\"rated_travel_deg\": 180.0,",
        "\"travel_deg\": 184.3328808989794,",
        1,
    );
    let ProfileIn::V1(p) = read::profile(&with).expect("parses") else {
        panic!("version 1")
    };
    let e = p.check().expect_err("refused");
    assert!(
        e.starts_with("the profile states travel_deg, the stop-to-stop travel")
            && e.contains("enter rated_travel_deg"),
        "{e}"
    );
    let (_, q) = v1();
    assert_eq!(resolve(&p, &q, None).expect_err("refused"), e);
    // read through the upgrade, as an import does, it is said before it drops
    let u = upgrade(read::profile(&with).expect("parses"), None).expect("upgrades");
    assert_eq!(u.notes, std::slice::from_ref(&e));
    let json = serde_json::to_value(&p).expect("json");
    assert!(json.get("travel_deg").is_none(), "{json}");
    // beside the rating it is refused all the same: it is still not read
    let both = PROFILE.replacen(
        "\"rated_travel_deg\": 180.0,",
        "\"rated_travel_deg\": 180.0, \"travel_deg\": 184.3,",
        1,
    );
    let ProfileIn::V1(p) = read::profile(&both).expect("parses") else {
        panic!("version 1")
    };
    assert_eq!(p.check().expect_err("refused"), e);
}

/// A version 0 nameplate's travel_deg is the travel stop to stop, so it is
/// never carried as the rating, only noted; the upgraded profile is refused
/// until the rating is entered. One stating rated_travel_deg carries it.
#[test]
fn a_version_0_travel_deg_is_noted_never_carried() {
    let np = read::profile(STOP_TO_STOP).expect("nameplate");
    let u = upgrade(np, None).expect("upgrades");
    assert_eq!(u.profile.rated_travel_deg, None);
    assert!(
        u.notes.iter().any(|n| n == "the version 0 nameplate's travel_deg (184.33) is the travel stop to stop; the profile takes the maker's rated travel: enter rated_travel_deg"),
        "{:?}",
        u.notes
    );
    let e = resolve(&u.profile, &u.policy, None).expect_err("no rating");
    assert!(
        e.starts_with("The run sets the soft limits to the rated travel")
            && e.contains("no usable rated_travel_deg"),
        "{e}"
    );
    let u = upgrade(read::profile(NAMEPLATE).expect("nameplate"), None).expect("upgrades");
    assert_eq!(u.profile.rated_travel_deg, Some(180.0));
    assert!(
        !u.notes.iter().any(|n| n.contains("travel_deg")),
        "{:?}",
        u.notes
    );
}

/// The soft window's width is the policy's motion.soft_travel_deg where it
/// states one, else the profile's rated travel; over 0 and at most a turn
/// is all the input asks, the run narrowing it to the stops. A policy that
/// states the retired soft_margin_share is refused naming soft_travel_deg;
/// a version 0 one drops it with a note.
#[test]
fn the_soft_travel_is_the_policys_else_the_rated_travel() {
    let (p, mut q) = v1();
    let line = |r: &osc_ident_inputs::Resolved| {
        r.report
            .fields
            .iter()
            .find(|l| l.field == "motion.soft_travel_deg")
            .cloned()
            .expect("a line")
    };
    // none stated stays none, so the rules name the profile's rated travel
    // as what binds; the report line gives its value
    let r = resolve(&p, &q, None).expect("resolves");
    assert_eq!(r.policy.soft_travel_deg, None);
    let l = line(&r);
    assert_eq!((&l.form, &l.value), (&Form::Profile, &Value::Number(180.0)));
    assert!(
        l.note
            .as_deref()
            .is_some_and(|n| n.starts_with("the profile's rated_travel_deg")),
        "{l:?}"
    );
    for deg in [120.0, 190.0, 360.0] {
        q.motion.soft_travel_deg = Some(deg);
        let r = resolve(&p, &q, None).expect("resolves");
        assert_eq!(r.policy.soft_travel_deg, Some(deg));
        assert_eq!(line(&r).form, Form::Stated);
    }
    for deg in [0.0, -10.0, 361.0, f64::NAN] {
        q.motion.soft_travel_deg = Some(deg);
        let e = resolve(&p, &q, None).expect_err("refused");
        assert!(e.contains("motion.soft_travel_deg is"), "{e}");
    }

    let with = RECORD_POLICY.replacen(
        "\"move_speed\"",
        "\"soft_margin_share\": 0.0625, \"move_speed\"",
        1,
    );
    let PolicyIn::V1(q) = read::policy(&with, "unused").expect("parses") else {
        panic!("version 1")
    };
    let e = q.check().expect_err("refused");
    assert!(
        e.starts_with("the policy states motion.soft_margin_share")
            && e.contains("motion.soft_travel_deg"),
        "{e}"
    );
    let json = serde_json::to_value(&*q).expect("json");
    assert!(json["motion"].get("soft_margin_share").is_none(), "{json}");

    let v0 = POLICY.replacen(
        "\"hold_band_noise_multiple\"",
        "\"soft_margin_share\": 0.0625, \"hold_band_noise_multiple\"",
        1,
    );
    let pol = read::policy(&v0, "policy").expect("policy");
    let u = upgrade(read::profile(NAMEPLATE).expect("nameplate"), Some(pol)).expect("upgrades");
    assert_eq!(u.policy.motion.soft_travel_deg, None);
    assert!(
        u.notes
            .iter()
            .any(|n| n.starts_with("the version 0 policy's soft_margin_share (0.0625) is not read")),
        "{:?}",
        u.notes
    );
    let r = resolve(&u.profile, &u.policy, None).expect("resolves");
    assert_eq!(r.policy.soft_travel_deg, None);
    assert_eq!(line(&r).value, Value::Number(180.0));
}

/// 0 deg is the servo's center of travel: the policy's
/// motion.center_offset_deg moves it, counterclockwise positive, and none
/// is the center. A version 1 policy that states the retired
/// angle_cw_stop_deg is refused in plain words naming center_offset_deg; a
/// version 0 one drops it with a note.
#[test]
fn the_center_offset_is_the_policys_and_the_clockwise_stop_angle_retires() {
    let (p, mut q) = v1();
    let line = |r: &osc_ident_inputs::Resolved| {
        r.report
            .fields
            .iter()
            .find(|l| l.field == "motion.center_offset_deg")
            .cloned()
            .expect("a line")
    };
    let r = resolve(&p, &q, None).expect("resolves");
    assert_eq!(r.policy.center_offset_deg, None);
    let l = line(&r);
    assert_eq!((&l.form, &l.value), (&Form::Stated, &Value::Number(0.0)));
    assert!(
        l.note
            .as_deref()
            .is_some_and(|n| n.starts_with("0 deg at the center of travel")),
        "{l:?}"
    );
    q.motion.center_offset_deg = Some(-12.5);
    let r = resolve(&p, &q, None).expect("resolves");
    assert_eq!(r.policy.center_offset_deg, Some(-12.5));
    assert_eq!(line(&r).value, Value::Number(-12.5));
    q.motion.center_offset_deg = Some(f64::NAN);
    let e = resolve(&p, &q, None).expect_err("refused");
    assert!(e.contains("motion.center_offset_deg is NaN"), "{e}");

    let with = RECORD_POLICY.replacen(
        "\"move_speed\"",
        "\"angle_cw_stop_deg\": 0.0, \"move_speed\"",
        1,
    );
    let PolicyIn::V1(q) = read::policy(&with, "unused").expect("parses") else {
        panic!("version 1")
    };
    assert_eq!(
        q.check().expect_err("refused"),
        "the policy states motion.angle_cw_stop_deg, which this tool no longer reads: 0 deg is the servo's center of travel; to move it, state motion.center_offset_deg, the degrees counterclockwise of the center"
    );
    let json = serde_json::to_value(&*q).expect("json");
    assert!(json["motion"].get("angle_cw_stop_deg").is_none(), "{json}");

    let v0 = POLICY.replacen(
        "\"hold_band_noise_multiple\"",
        "\"angle_cw_stop_deg\": 0.0, \"hold_band_noise_multiple\"",
        1,
    );
    let pol = read::policy(&v0, "policy").expect("policy");
    let u = upgrade(read::profile(NAMEPLATE).expect("nameplate"), Some(pol)).expect("upgrades");
    assert_eq!(u.policy.motion.center_offset_deg, None);
    assert!(
        u.notes
            .iter()
            .any(|n| n.starts_with("the version 0 policy's angle_cw_stop_deg (0) is not read")),
        "{:?}",
        u.notes
    );
    assert!(u.policy.check().is_ok());
}
