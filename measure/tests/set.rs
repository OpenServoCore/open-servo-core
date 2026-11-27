//! The set file against the notebooks' own: the set of record of mg90-d
//! (`testdata/mg90-d/constants.json`, a byte copy of the notebooks'
//! `mg90-d__dev-v006-2A-D.constants.json` at sha1 c30f6864) loads, hashes
//! and renders back byte for byte.

mod common;

use common::{inputs, record, rules_set, table, text};
use osc_measure::set::{SetFile, file_sha1, points_sha1};

#[test]
fn a_notebook_set_file_loads_and_round_trips() {
    let file = text("testdata/mg90-d/constants.json");
    assert_eq!(
        file_sha1(file.as_bytes()),
        "c30f686432c191421fbe6c93eaa5e9f8eed74a03"
    );
    let set = record();
    assert_eq!(set.render(), file, "rendered back byte for byte");
    assert_eq!(
        set.hash(),
        "362e018756172017f612be404de097dd772706b54d109445344b1e5324a82242"
    );
    assert_eq!(set.values().len(), 64);
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
fn a_set_from_the_rules_is_the_set_of_record_but_for_the_held_and_retired_fields() {
    let ours = rules_set(&inputs(), &table());
    let rec = record();
    assert_eq!(ours.values().len(), 62);
    for (name, v) in rec.values() {
        match name {
            "fric_breakaway_counts" | "v_undervolt_counts" => assert_eq!(ours.value(name), None),
            _ => assert_eq!(ours.value(name), Some(v), "{name}"),
        }
    }
    assert_eq!(ours.table(), rec.table());
    assert_eq!(ours.slots(), rec.slots());
}
