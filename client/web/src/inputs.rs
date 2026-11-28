//! The identification's inputs from a page, through osc-ident-inputs so the
//! web gives the CLI's messages and values: check a servo profile or a
//! tuning policy, read a file of any version as version 1, and preview what
//! a policy resolves to with a profile before any run. JS parses the JSON;
//! the values cross through serde-wasm-bindgen, so serde_json stays out of
//! the module.

use osc_ident_inputs::migrate::{ThermalV0, heat_of, slug};
use osc_ident_inputs::profile::FieldSource;
use osc_ident_inputs::read::{self, Probe};
use osc_ident_inputs::{
    BoardFacts, Policy, Report, ServoProfile, TuningPolicy, resolve, upgrade_with,
};
use serde::Serialize;
use serde::de::DeserializeOwned;
use tsify::{Ts, Tsify};
use wasm_bindgen::prelude::*;

/// A profile and a policy at version 1, whatever versions their files were
/// written at, and a line for each file read at an older version.
#[derive(Debug, Clone, Serialize, Tsify)]
#[tsify(missing_as_null, hashmap_as_object)]
#[serde(rename_all = "camelCase")]
pub struct Migrated {
    pub profile: ServoProfile,
    /// With no policy file, what a version 0 nameplate carried (the run
    /// voltage and the derates) and nothing else.
    pub policy: TuningPolicy,
    pub notes: Vec<String>,
}

/// A policy resolved with a profile before any run. The values a run binds
/// from what it measures are not here: the current limit is shown at a
/// preview resistance, and a rule form over what the run measures stays
/// in its form, its report line saying what the run sets it from.
#[derive(Debug, Clone, Serialize, Tsify)]
#[tsify(missing_as_null, hashmap_as_object)]
#[serde(rename_all = "camelCase")]
pub struct Preview {
    /// Each policy value as resolved: what a run's `inputs/resolved.json`
    /// holds.
    pub report: Report,
    /// The absolute values the rules read (deg/s, deg/s^2, mA, ms).
    pub values: Policy,
    /// The speed ceiling, deg/s, at the no-load rating the run voltage picks.
    pub speed_ceiling_deg_s: f64,
    pub speed_ceiling_at_v: f64,
    /// The current limit, A: the share of the stated stall row's volts over
    /// `r_ohm`. The run takes the identified R instead.
    pub current_limit_a: f64,
    pub current_limit_at_v: f64,
    pub r_ohm: f64,
}

/// A file's parsed value, read as whichever type its version names.
trait File {
    fn parse<T: DeserializeOwned>(&self) -> Result<T, String>;
}

impl File for JsValue {
    fn parse<T: DeserializeOwned>(&self) -> Result<T, String> {
        // a serde-wasm-bindgen error is a JS Error, which prints "Error: <why>"
        serde_wasm_bindgen::from_value(self.clone()).map_err(|e| {
            let e = e.to_string();
            e.strip_prefix("Error: ").map_or(e.clone(), String::from)
        })
    }
}

fn check<T: DeserializeOwned>(
    file: &impl File,
    check: fn(&T) -> Result<(), String>,
) -> Option<String> {
    file.parse().and_then(|v| check(&v)).err()
}

/// A version 0 `thermal.json` as read, with the file's name and sha1 for
/// the profile's provenance.
struct Thermal<'a, F> {
    file: &'a F,
    name: Option<String>,
    sha1: Option<String>,
}

fn migrate_files<F: File>(
    profile: &F,
    policy: Option<&F>,
    policy_key: Option<&str>,
    thermal: Option<Thermal<'_, F>>,
) -> Result<Migrated, String> {
    let p = read::profile_with(
        profile.parse::<Probe>()?,
        || profile.parse(),
        || profile.parse(),
    )?;
    let q = match policy {
        Some(f) => {
            let key = policy_key.map_or("policy".into(), slug);
            Some(read::policy_with(
                f.parse::<Probe>()?,
                &key,
                || f.parse(),
                || f.parse(),
            )?)
        }
        None => None,
    };
    let heat = match thermal {
        Some(t) => {
            let from = FieldSource {
                how: Some("file".into()),
                file: t.name,
                sha1: t.sha1,
                ..FieldSource::default()
            };
            Some((heat_of(&t.file.parse::<ThermalV0>()?)?, from))
        }
        None => None,
    };
    let u = upgrade_with(p, q, heat)?;
    Ok(Migrated {
        profile: u.profile,
        policy: u.policy,
        notes: u.notes,
    })
}

fn preview(
    profile: &ServoProfile,
    policy: &TuningPolicy,
    r_ohm: f64,
    board: Option<&BoardFacts>,
) -> Result<Preview, String> {
    if !(r_ohm.is_finite() && r_ohm > 0.0) {
        return Err(format!(
            "The current limit is the stated volts over the winding's resistance: the preview resistance {r_ohm} ohm is not over 0"
        ));
    }
    let r = resolve(profile, policy, board)?;
    let n = &r.nameplate;
    let (Some(speed), Some(stall), Some(ceiling), Some(share)) = (
        n.speed_row(),
        n.stall_row(),
        n.speed_ceiling_deg_s(),
        n.derate_other,
    ) else {
        unreachable!("resolve holds both rows and both derates")
    };
    Ok(Preview {
        report: r.report,
        values: r.policy,
        speed_ceiling_deg_s: ceiling,
        speed_ceiling_at_v: speed.at_v,
        current_limit_a: share * stall.at_v / r_ohm,
        current_limit_at_v: stall.at_v,
        r_ohm,
    })
}

/// Why the run cannot use `profile`, the consequence first; undefined when
/// it can. The rating rows are checked with a policy, by `resolvePreview`,
/// since the run voltage picks the row.
#[wasm_bindgen(js_name = validateProfile)]
pub fn validate_profile(profile: Ts<ServoProfile>) -> Option<String> {
    check(&JsValue::from(profile), ServoProfile::check)
}

/// Why the run cannot use `policy`, the consequence first; undefined when
/// it can.
#[wasm_bindgen(js_name = validatePolicy)]
pub fn validate_policy(policy: Ts<TuningPolicy>) -> Option<String> {
    check(&JsValue::from(policy), TuningPolicy::check)
}

/// `profile` and `policy` resolved as a run would resolve them, with the
/// current limit at `rOhm`; `board` gives the overcurrent window. Throws
/// the CLI's refusal.
#[wasm_bindgen(js_name = resolvePreview)]
pub fn resolve_preview(
    profile: Ts<ServoProfile>,
    policy: Ts<TuningPolicy>,
    #[wasm_bindgen(js_name = rOhm)] r_ohm: f64,
    board: Option<Ts<BoardFacts>>,
) -> Result<Ts<Preview>, JsError> {
    let parsed = |profile: JsValue, policy: JsValue, board: Option<JsValue>| {
        let board: Option<BoardFacts> = board.map(|b| b.parse()).transpose()?;
        preview(&profile.parse()?, &policy.parse()?, r_ohm, board.as_ref())
    };
    let p = parsed(profile.into(), policy.into(), board.map(JsValue::from))
        .map_err(|e| JsError::new(&e))?;
    Ok(p.into_ts()?)
}

/// Why the run cannot go while the profile states no heat model.
#[wasm_bindgen(js_name = noHeatModel)]
pub fn no_heat_model() -> String {
    osc_ident_inputs::no_heat_model()
}

/// A profile file's parsed JSON, and a policy's beside it, at version 1:
/// a version 0 nameplate's run voltage and derates move into the policy.
/// `policyKey` names a version 0 policy (its file's name). `thermal` is
/// the version 0 `thermal.json` beside a version 0 nameplate, read as the
/// profile's heat model, `thermalFile` and `thermalSha1` its provenance.
/// Throws the CLI's refusal, a newer version named.
#[wasm_bindgen]
pub fn migrate(
    #[wasm_bindgen(unchecked_param_type = "unknown")] profile: JsValue,
    #[wasm_bindgen(unchecked_param_type = "unknown")] policy: JsValue,
    #[wasm_bindgen(js_name = policyKey, unchecked_optional_param_type = "string | null")]
    policy_key: Option<String>,
    #[wasm_bindgen(unchecked_optional_param_type = "unknown")] thermal: JsValue,
    #[wasm_bindgen(js_name = thermalFile, unchecked_optional_param_type = "string")]
    thermal_file: Option<String>,
    #[wasm_bindgen(js_name = thermalSha1, unchecked_optional_param_type = "string")]
    thermal_sha1: Option<String>,
) -> Result<Ts<Migrated>, JsError> {
    let given = |v: &JsValue| !v.is_undefined() && !v.is_null();
    let policy = given(&policy).then_some(&policy);
    let thermal = given(&thermal).then_some(Thermal {
        file: &thermal,
        name: thermal_file,
        sha1: thermal_sha1,
    });
    let m = migrate_files(&profile, policy, policy_key.as_deref(), thermal)
        .map_err(|e| JsError::new(&e))?;
    Ok(m.into_ts()?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_ident_inputs::resolve::{Form, Value};

    const PROFILE: &str = include_str!("../../../measure/testdata/mg90-d/profile.json");
    const POLICY: &str = include_str!("../../../measure/testdata/mg90-d/policy-of-record.json");
    const NAMEPLATE: &str = include_str!("../../../measure/testdata/mg90-d/nameplate.json");
    const STOP_TO_STOP: &str =
        include_str!("../../../measure/testdata/mg90-d/nameplate-stop-to-stop.json");
    const POLICY_V0: &str = include_str!("../../../measure/testdata/mg90-d/policy.json");
    const DEFAULT: &str = include_str!("../../../measure/testdata/mg90-d/default-policy.json");
    const THERMAL: &str = include_str!("../../../measure/testdata/mg90-d/thermal.json");
    /// The board facts the bench servo publishes, as step 0 keeps them.
    const BOARD: &str = include_str!("../../../measure/testdata/mg90-d/board.json");
    /// The preview resistance the policy page shows: the record's 4.1767 ohm.
    const R_OHM: f64 = 4.18;

    impl File for serde_json::Value {
        fn parse<T: DeserializeOwned>(&self) -> Result<T, String> {
            T::deserialize(self).map_err(|e| e.to_string())
        }
    }

    fn json(text: &str) -> serde_json::Value {
        serde_json::from_str(text).expect("json")
    }

    fn record() -> (ServoProfile, TuningPolicy, BoardFacts) {
        (
            json(PROFILE).parse().expect("profile"),
            json(POLICY).parse().expect("policy"),
            json(BOARD)["registers"].parse().expect("board facts"),
        )
    }

    fn line<'a>(p: &'a Preview, field: &str) -> &'a osc_ident_inputs::resolve::Line {
        let found = p.report.fields.iter().find(|l| l.field == field);
        found.unwrap_or_else(|| panic!("{field}"))
    }

    #[test]
    fn the_records_profile_and_policy_pass() {
        assert_eq!(check(&json(PROFILE), ServoProfile::check), None);
        assert_eq!(check(&json(POLICY), TuningPolicy::check), None);
        let mut saved = json(POLICY);
        saved["revision"] = 3.into();
        assert_eq!(
            check(&saved, TuningPolicy::check),
            None,
            "the library's revision"
        );
    }

    /// Each fact the run needs, taken out, is refused naming the field,
    /// the consequence first: the profile's own by `validateProfile`, the
    /// rating rows by `resolvePreview`, the policy's by `validatePolicy`.
    #[test]
    fn each_required_field_refusal_returns_its_message() {
        type Edit = fn(&mut serde_json::Value);
        let profile: [(&str, &str, Edit); 11] = [
            ("gear_class", "Without the gear material", |v| {
                v["gear_class"] = serde_json::Value::Null
            }),
            (
                "gear_ratio",
                "Without the gear ratio the run cannot turn motor turns into horn degrees",
                |v| v["gear_ratio"] = 0.into(),
            ),
            (
                "rated_travel_deg",
                "The run sets the soft limits to the rated travel",
                |v| v["rated_travel_deg"] = 361.into(),
            ),
            (
                "rated_travel_deg",
                "The run sets the soft limits to the rated travel",
                |v| v["rated_travel_deg"] = serde_json::Value::Null,
            ),
            (
                "enter rated_travel_deg",
                "the profile states travel_deg, the stop-to-stop travel",
                |v| v["travel_deg"] = 184.33.into(),
            ),
            (
                "count_rises_ccw",
                "Without it the run cannot tell which stop is which",
                |v| v["count_rises_ccw"] = serde_json::Value::Null,
            ),
            (
                "brushes and rotor_segments",
                "Without both counts the run cannot measure the angle table",
                |v| v["brushes"] = serde_json::Value::Null,
            ),
            ("brushes and rotor_segments", "Without both counts", |v| {
                v["rotor_segments"] = 0.into()
            }),
            ("operating_v", "The run needs the maker's range", |v| {
                v["operating_v"] = serde_json::Value::Null
            }),
            ("key", "The key names", |v| v["key"] = "Vorpal MG90".into()),
            ("unknown field `brush`", "unknown field", |v| {
                v["brush"] = 2.into()
            }),
        ];
        for (field, consequence, edit) in profile {
            let mut v = json(PROFILE);
            edit(&mut v);
            let e = check(&v, ServoProfile::check).expect(field);
            assert!(e.starts_with(consequence) && e.contains(field), "{e}");
        }

        let (_, policy, board) = record();
        let rows: [(&str, &str, Edit); 3] = [
            (
                "no_load",
                "The speed ceiling that protects the brushes cannot be set",
                |v| v["no_load"] = serde_json::json!([]),
            ),
            (
                "no_load",
                "The speed ceiling",
                |v| {
                    v["no_load"] = serde_json::json!([{"at_v": 9.0, "seconds": 0.1, "per_deg": 60}])
                },
            ),
            ("stall", "The current limit cannot be set", |v| {
                v["stall"] = serde_json::json!([])
            }),
        ];
        for (field, consequence, edit) in rows {
            let mut v = json(PROFILE);
            edit(&mut v);
            let p: ServoProfile = v.parse().expect("profile");
            assert_eq!(p.check(), Ok(()), "the rows wait for the policy");
            let e = preview(&p, &policy, R_OHM, Some(&board)).expect_err(field);
            assert!(e.starts_with(consequence) && e.contains(field), "{e}");
        }

        let mut v = json(POLICY);
        v["motion"]["soft_margin_share"] = 0.0625.into();
        let e = check(&v, TuningPolicy::check).expect("retired");
        assert!(
            e.starts_with("the policy states motion.soft_margin_share")
                && e.contains("motion.soft_travel_deg"),
            "{e}"
        );
        let mut v = json(POLICY);
        v["motion"]["soft_travel_deg"] = 0.into();
        let e = check(&v, TuningPolicy::check).expect("no width");
        assert!(e.contains("motion.soft_travel_deg is 0"), "{e}");
        v["motion"]["soft_travel_deg"] = 170.into();
        assert_eq!(check(&v, TuningPolicy::check), None);

        let policy: [(&str, Edit); 4] = [
            ("protection.run_v", |v| {
                v["protection"]["run_v"] = serde_json::Value::Null
            }),
            ("protection.speed_ceiling", |v| {
                v["protection"]["speed_ceiling"] = serde_json::json!({"share_of_no_load": [4, 3]})
            }),
            ("protection.current_limit", |v| {
                v["protection"]["current_limit"] = serde_json::Value::Null
            }),
            ("protection.current_limit", |v| {
                v["protection"]["current_limit"] = serde_json::json!({"share_of_stall": 1.2})
            }),
        ];
        for (field, edit) in policy {
            let mut v = json(POLICY);
            edit(&mut v);
            let e = check(&v, TuningPolicy::check).expect(field);
            assert!(
                e.contains(&format!("the policy has no usable {field} (")),
                "{e}"
            );
        }
    }

    /// The version 0 pair reads as the record's version 1 values: the
    /// nameplate's run voltage and derates in the policy, its name the key.
    /// A nameplate alone (an import) gives its profile.
    #[test]
    fn a_version_0_file_migrates() {
        let m = migrate_files(
            &json(NAMEPLATE),
            Some(&json(POLICY_V0)),
            Some("policy"),
            None,
        )
        .expect("migrates");
        assert_eq!(m.profile.key, "mg90-d");
        assert_eq!(m.profile.gear_ratio, Some(308.0533));
        assert_eq!(m.policy.key, "policy");
        assert_eq!(m.policy.protection.run_v, Some(5.0));
        assert_eq!(m.notes.len(), 2, "{:?}", m.notes);
        assert!(m.notes[0].contains("version 0"), "{:?}", m.notes);
        let (_, _, board) = record();
        let p = preview(&m.profile, &m.policy, R_OHM, Some(&board)).expect("resolves");
        assert_eq!(p.speed_ceiling_deg_s, 400.0);
        assert_eq!(p.values.velocity_limit_deg_s, Some(200.0));

        let alone = migrate_files(&json(NAMEPLATE), None, None, None).expect("alone");
        let (record, _, _) = record();
        assert_eq!(alone.profile.no_load, record.no_load);
        assert_eq!(alone.profile.rated_travel_deg, record.rated_travel_deg);
        assert_eq!(alone.policy.protection.run_v, Some(5.0));
        assert_eq!(alone.notes.len(), 1);

        // a migrated profile written as JSON reads back as itself
        let text = serde_json::to_string(&alone.profile).expect("serializes");
        let back: ServoProfile = json(&text).parse().expect("reads back");
        assert_eq!(back, alone.profile);

        let v1 = migrate_files(&json(PROFILE), Some(&json(POLICY)), None, None).expect("version 1");
        assert!(v1.notes.is_empty());
        assert_eq!(v1.profile, record);

        // a version 0 travel_deg is the travel stop to stop: noted, never
        // carried, and the profile it gives is refused until the rating
        let m = migrate_files(&json(STOP_TO_STOP), None, None, None).expect("migrates");
        assert_eq!(m.profile.rated_travel_deg, None);
        assert!(
            m.notes.iter().any(|n| n.starts_with(
                "the version 0 nameplate's travel_deg (184.33) is the travel stop to stop"
            )),
            "{:?}",
            m.notes
        );
        let e = check(
            &json(&serde_json::to_string(&m.profile).expect("serializes")),
            ServoProfile::check,
        )
        .expect("no rating");
        assert!(e.contains("no usable rated_travel_deg"), "{e}");

        let newer = json(&PROFILE.replacen("osc.servo-profile/1", "osc.servo-profile/2", 1));
        let e = migrate_files(&newer, None, None, None).expect_err("newer");
        assert!(e.contains("version 2") && e.contains("up to 1"), "{e}");
        let e = migrate_files(&json(POLICY), None, None, None).expect_err("a policy is no profile");
        assert!(e.contains("not a osc.servo-profile file"), "{e}");
    }

    /// A version 0 nameplate with the thermal.json beside it imports to the
    /// record profile's heat model, the file named as its source; a file
    /// short of a value, or one beside a version 1 profile, is refused.
    #[test]
    fn a_version_0_thermal_json_becomes_the_profiles_heat_model() {
        let thermal = json(THERMAL);
        let beside = |t| Thermal {
            file: t,
            name: Some("thermal.json".into()),
            sha1: Some("97572d64".into()),
        };
        let m =
            migrate_files(&json(NAMEPLATE), None, None, Some(beside(&thermal))).expect("migrates");
        let (record, _, _) = record();
        assert_eq!(m.profile.thermal, record.thermal);
        let from = &m.profile.provenance.fields["thermal"];
        assert_eq!(
            (
                from.how.as_deref(),
                from.file.as_deref(),
                from.sha1.as_deref()
            ),
            (Some("file"), Some("thermal.json"), Some("97572d64"))
        );
        assert!(m.notes[0].contains("thermal.json"), "{:?}", m.notes);
        assert_eq!(
            check(
                &json(&serde_json::to_string(&m.profile).expect("serializes")),
                ServoProfile::check
            ),
            None
        );

        let mut short = json(THERMAL);
        short.as_object_mut().expect("object").remove("slow_tau_s");
        let e =
            migrate_files(&json(NAMEPLATE), None, None, Some(beside(&short))).expect_err("short");
        assert!(
            e.starts_with("The profile's heat model is all four values")
                && e.contains("slow_tau_s"),
            "{e}"
        );

        let e = migrate_files(
            &json(PROFILE),
            Some(&json(POLICY)),
            None,
            Some(beside(&thermal)),
        )
        .expect_err("beside version 1");
        assert!(
            e.starts_with("The profile is the heat model's one source"),
            "{e}"
        );

        assert!(no_heat_model().contains("the profile has no usable thermal"));
    }

    /// The record's profile and policy preview the record's values: the
    /// ceiling at two thirds of 600 deg/s, the move speed and acceleration,
    /// the current limit 0.8 x 4.8 V / 4.18 ohm, and the board's window.
    #[test]
    fn resolve_preview_on_the_record_gives_the_records_values() {
        let (profile, mut policy, board) = record();
        let p = preview(&profile, &policy, R_OHM, Some(&board)).expect("resolves");
        assert_eq!(p.speed_ceiling_deg_s, 400.0);
        assert_eq!(p.speed_ceiling_at_v, 4.8);
        assert_eq!(p.values.velocity_limit_deg_s, Some(200.0));
        assert_eq!(p.values.accel_limit_deg_s2, Some(4000.0));
        assert_eq!((p.current_limit_a * 100.0).round(), 92.0);
        assert_eq!(p.current_limit_at_v, 4.8);
        assert_eq!(p.report.profile.key, "vorpal-mg90");
        let oc = line(&p, "oc_trip");
        assert_eq!((&oc.form, &oc.value), (&Form::Board, &Value::Number(8.0)));
        assert!(
            preview(&profile, &policy, R_OHM, None)
                .expect("no board")
                .report
                .fields
                .iter()
                .all(|l| l.field != "oc_trip")
        );

        // the policy page's shares give the same values
        policy.motion.move_speed = Some(osc_ident_inputs::tuning::MoveSpeed::ShareOfCeiling(0.5));
        policy.motion.speed_up = Some(osc_ident_inputs::tuning::SpeedUp::S(0.05));
        let p = preview(&profile, &policy, R_OHM, Some(&board)).expect("resolves");
        assert_eq!(p.values.velocity_limit_deg_s, Some(200.0));
        assert_eq!(p.values.accel_limit_deg_s2, Some(4000.0));
        assert_eq!(line(&p, "motion.move_speed").form, Form::Share);

        for r in [0.0, -1.0, f64::NAN] {
            let e = preview(&profile, &policy, r, None).expect_err("no resistance");
            assert!(
                e.starts_with("The current limit is the stated volts"),
                "{e}"
            );
        }
    }

    /// The Default policy previews: its rule forms stay in their forms for
    /// the run, each line saying what the run sets it from, and on metal no
    /// yield torque is the cap.
    #[test]
    fn the_default_policy_previews_its_rule_forms() {
        let (profile, _, board) = record();
        let policy: TuningPolicy = json(DEFAULT).parse().expect("policy");
        assert_eq!(check(&json(DEFAULT), TuningPolicy::check), None);
        let p = preview(&profile, &policy, R_OHM, Some(&board)).expect("resolves");
        let y = line(&p, "protection.yield");
        assert_eq!(
            (&y.form, &y.value),
            (&Form::Share, &Value::Text("the cap".into()))
        );
        for f in [
            "protection.stall_release",
            "protection.stall_speed",
            "loops.current_loop",
            "loops.observer",
            "sensor.pot_jump",
        ] {
            let l = line(&p, f);
            assert_eq!(l.form, Form::Share, "{f}");
            assert!(
                l.note
                    .as_deref()
                    .is_some_and(|n| n.ends_with("set by the run")),
                "{f}"
            );
        }
        let v = serde_json::to_value(&p).expect("serializes");
        assert_eq!(
            v["values"]["yield"],
            serde_json::json!({"torque_kg_cm": null})
        );
        assert_eq!(
            v["values"]["current_loop"],
            serde_json::json!({"multiple_of_electrical_corner": 1.25})
        );
    }

    /// The wrapper keys are camelCase, the file types keep their file keys.
    #[test]
    fn a_preview_serializes_with_the_file_keys_inside() {
        let (profile, policy, board) = record();
        let p = preview(&profile, &policy, R_OHM, Some(&board)).expect("resolves");
        let v = serde_json::to_value(&p).expect("serializes");
        assert_eq!(v["speedCeilingDegS"], 400.0);
        assert_eq!(v["values"]["velocity_limit_deg_s"], 200.0);
        assert_eq!(v["report"]["policy"]["key"], "mg90-d-record");
        assert_eq!(v["report"]["fields"][0]["field"], "protection.run_v");
    }
}
