//! The set file against the notebooks' own: the set of record of mg90-d
//! (`testdata/mg90-d/constants.json`, a byte copy of the notebooks'
//! `mg90-d__dev-v006-2A-D.constants.json` at sha1 c94abef7) loads, hashes
//! and renders back byte for byte.

mod common;

use common::{descriptor, inputs, record, rules_set, table, text};
use osc_ident_inputs::{read, resolve, upgrade};
use osc_measure::build;
use osc_measure::set::{self, FromRules, SetFile, file_sha1, points_sha1};

#[test]
fn a_notebook_set_file_loads_and_round_trips() {
    let file = text("testdata/mg90-d/constants.json");
    assert_eq!(
        file_sha1(file.as_bytes()),
        "c94abef7c670b1a3c3d5732374e72d478953c931"
    );
    let set = record();
    assert_eq!(set.render(), file, "rendered back byte for byte");
    assert_eq!(
        set.hash(),
        "e8520bef6124c6631ba5386c6387f88edcffe5132ca4277bd058b14b28ac61e9"
    );
    assert_eq!(set.values().len(), 63);
    assert_eq!(set.table(), ("LIVE", points_sha1(&table().points).as_str()));

    // A value edited by hand, a plant and a protection field.
    for (from, to) in [
        ("\"value\": 3725,", "\"value\": 3726,"),
        ("\"value\": 1043,", "\"value\": 1044,"),
    ] {
        let edited = file.replacen(from, to, 1);
        assert_ne!(edited, file);
        let e = SetFile::parse(&edited).expect_err("an edited set is refused");
        assert!(e.0.contains("hashes are not those of its content"), "{e}");
    }
    let e = SetFile::parse(&file.replacen("\"format\": 1", "\"format\": 2", 1))
        .expect_err("another format");
    assert!(e.0.contains("format 2"), "{e}");
    // A protection marker that says another thing than the list.
    let edited = file.replacen(
        "\"protection\": {\"looser\": \"up\"}, \"q\": 0, \"rule\": \"cutoff.policy\"",
        "\"protection\": null, \"q\": 0, \"rule\": \"cutoff.policy\"",
        1,
    );
    assert_ne!(edited, file);
    assert!(SetFile::parse(&edited).is_err());

    // A set this crate writes round-trips the same way.
    let ours = rules_set(&inputs(), &table());
    let again = SetFile::parse(&ours.render()).expect("our set loads");
    assert_eq!(again, ours);
    assert_eq!(again.render(), ours.render());
}

#[test]
fn a_set_from_the_rules_is_the_set_of_record_but_for_the_held_field() {
    let ours = rules_set(&inputs(), &table());
    let rec = record();
    assert_eq!(ours.values().len(), 62);
    for (name, v) in rec.values() {
        match name {
            "fric_breakaway_counts" => assert_eq!(ours.value(name), None),
            _ => assert_eq!(ours.value(name), Some(v), "{name}"),
        }
    }
    assert_eq!(ours.table(), rec.table());
    assert_eq!(ours.slots(), rec.slots());
}

/// The record's servo profile and its own policy, every rule-bound value in
/// its fixed form, over the record's measured inputs give the set the
/// version 0 files give, byte for byte; so do the version 0 files read
/// through the migration. The overcurrent window comes from the board.
#[test]
fn the_record_inputs_as_profile_and_policy_bind_the_record_set() {
    let v0 = rules_set(&inputs(), &table()).render();
    let pairs = [
        (
            "testdata/mg90-d/profile.json",
            "testdata/mg90-d/policy-of-record.json",
        ),
        (
            "testdata/mg90-d/nameplate.json",
            "testdata/mg90-d/policy.json",
        ),
    ];
    for (profile, policy) in pairs {
        let mut i = inputs();
        let u = upgrade(
            read::profile(&text(profile)).expect("profile"),
            Some(read::policy(&text(policy), "policy").expect("policy")),
        )
        .expect("upgrades");
        let r = resolve(&u.profile, &u.policy, Some(&i.board.facts())).expect("resolves");
        assert_eq!(r.policy.oc_trip_samples, Some(8.0));
        i.nameplate = r.nameplate;
        i.policy = r.policy;
        assert_eq!(
            rules_set(&i, &table()).render(),
            v0,
            "{profile} with {policy}"
        );
    }
}

/// A field the rules stored at a bound goes into the set at that bound,
/// and the set's notes say what the rule gave.
#[test]
fn a_field_stored_at_a_bound_is_said_in_the_sets_notes() {
    let mut i = inputs();
    i.board.oc_trip_ma = 800.0;
    let t = table();
    let (file, notes) = set::from_rules(
        &build(&i),
        &FromRules {
            pair: "mg90-d__dev-v006",
            nameplate: &i.nameplate,
            counts_per_deg: i.measured.table.as_ref().map(|t| t.counts_per_deg),
            table: Some(&t),
            slots: None,
            files: &[],
        },
        &descriptor(),
    )
    .expect("a set");
    assert_eq!(file.value("current_limit_counts"), Some(&908.into()));
    assert_eq!(file.value("oc_trip_counts"), Some(&908.into()));
    let said: Vec<&String> = notes
        .iter()
        .filter(|n| n.starts_with("current_limit_counts: "))
        .collect();
    assert_eq!(said.len(), 1, "{notes:?}");
    assert!(said[0].ends_with("which fires before the limit folds: stored as 908"));
    SetFile::parse(&file.render()).expect("it loads");
}
