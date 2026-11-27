//! The profile and policy files: the record's at both versions, the
//! migration of version 0, the refusals, and what `resolve` gives.

use osc_ident_inputs::migrate::{PolicyIn, ProfileIn, slug};
use osc_ident_inputs::policy::{StallResponse, Switch};
use osc_ident_inputs::resolve::{Form, Value};
use osc_ident_inputs::tuning::{
    CurrentLimit, CurrentLoop, MoveSpeed, Observer, PotJump, SpeedCeiling, SpeedUp, StallRelease,
    StallSpeed, Yield,
};
use osc_ident_inputs::{BoardFacts, ServoProfile, TuningPolicy, read, resolve, upgrade, version};

const NAMEPLATE: &str = include_str!("../../testdata/mg90-d/nameplate.json");
const POLICY: &str = include_str!("../../testdata/mg90-d/policy.json");
const PROFILE: &str = include_str!("../../testdata/mg90-d/profile.json");
const RECORD_POLICY: &str = include_str!("../../testdata/mg90-d/policy-of-record.json");
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
    let u = upgrade(np, Some(pol)).expect("upgrades");
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
    assert_eq!(
        f["travel_deg"].key.as_deref(),
        Some("target.theta_at_stops_deg, the high less the low")
    );
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
    assert_eq!(q.motion.soft_margin_share, Some(0.0625));
    assert_eq!(q.motion.angle_cw_stop_deg, Some(0.0));
    assert_eq!(q.loops.current_loop, Some(CurrentLoop::Hz(1000.0)));
    assert_eq!(q.loops.observer, Some(Observer::Hz(15.0)));
    assert_eq!(q.loops.position_split, Some(7.104));
    assert_eq!(q.loops.hold_band_floor_counts, Some(4.0));
    assert_eq!(q.sensor.pot_jump, Some(PotJump::Counts(64.0)));
    assert_eq!(q.sensor.bad_samples, Some(4.0));
    assert_eq!(q.behavior.brush_term, Some(Switch::Off));
    assert_eq!(
        q.source.as_ref().and_then(|s| s.sha1.as_deref()),
        Some("b12f35ed8cb4b322d6c57b8678065cf674464283")
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
        assert_eq!(n.stall, nameplate.stall);
        assert_eq!(n.document, nameplate.document);
        assert_eq!(n.travel_deg, nameplate.travel_deg);
    }
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

/// A form whose rule this tool does not have yet is refused naming the
/// fixed form, so no field is left out unannounced.
#[test]
fn a_rule_form_this_tool_cannot_resolve_is_refused_naming_the_fixed_form() {
    let (p, q) = v1();
    type Set = fn(&mut TuningPolicy);
    let cases: [(&str, &str, Set); 6] = [
        ("protection.yield", "ma", |q| {
            q.protection.yield_ = Some(Yield::TorqueKgCm(None))
        }),
        ("protection.stall_release", "ma", |q| {
            q.protection.stall_release = Some(StallRelease::ShareOfYield(0.36))
        }),
        ("protection.stall_speed", "deg_s", |q| {
            q.protection.stall_speed = Some(StallSpeed::MultipleOfStillSpeed(4.0))
        }),
        ("loops.current_loop", "hz", |q| {
            q.loops.current_loop = Some(CurrentLoop::MultipleOfElectricalCorner(1.25))
        }),
        ("loops.observer", "hz", |q| {
            q.loops.observer = Some(Observer::MultipleOfMechanicalCorner(1.9))
        }),
        ("sensor.pot_jump", "counts", |q| {
            q.sensor.pot_jump = Some(PotJump::MultipleOfStep4(4.0))
        }),
    ];
    for (field, fixed, set) in cases {
        let mut q = q.clone();
        set(&mut q);
        let e = resolve(&p, &q, Some(&BOARD)).expect_err(field);
        assert!(
            e.contains(field) && e.contains(&format!("{{\"{fixed}\": ...}}")),
            "{e}"
        );
    }
}

/// Each fact the run needs, taken out of the profile, is refused naming
/// the field, the consequence first.
#[test]
fn each_required_profile_field_is_refused_with_its_message() {
    let (p, q) = v1();
    assert!(p.check().is_ok() && q.check().is_ok());
    type Take = fn(&mut ServoProfile);
    let cases: [(&str, &str, Take); 11] = [
        ("gear_class", "Without the gear material", |p| {
            p.gear_class = None
        }),
        (
            "gear_ratio",
            "Without the gear ratio the run cannot turn motor turns into horn degrees",
            |p| p.gear_ratio = Some(0.0),
        ),
        ("travel_deg", "A horn cannot turn more than one turn", |p| {
            p.travel_deg = Some(361.0)
        }),
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
    assert_eq!(r.policy.yield_current_ma, None);
}

#[test]
fn a_name_becomes_a_key() {
    assert_eq!(slug("mg90-d"), "mg90-d");
    assert_eq!(slug("  Vorpal MG90 (metal)! "), "vorpal-mg90-metal");
    assert_eq!(slug(&"a".repeat(50)).len(), 40);
}
