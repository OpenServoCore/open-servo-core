//! Version 0, the `nameplate.json` and `policy.json` written before the
//! schemas, read as a profile and a policy. The nameplate held three
//! choices that are the policy's (the run voltage and the two derates),
//! and the policy held the board's overcurrent window, which leaves it.

use std::collections::BTreeMap;

use serde::Deserialize;
use serde::de::IgnoredAny;

use crate::nameplate::Nameplate;
use crate::policy::Policy;
use crate::profile::{self, FieldSource, Model, Provenance, ServoProfile, StallRow};
use crate::tuning::{
    CurrentLimit, CurrentLoop, MoveSpeed, Observer, PositionError, PotJump, SpeedCeiling, SpeedUp,
    StallRelease, StallSpeed, TuningPolicy, Yield,
};

/// Whether every one of the six form values a version 0 policy states is
/// its fixed form.
fn all_fixed(p: &Policy) -> bool {
    !matches!(p.yield_, Some(Yield::TorqueKgCm(_)))
        && !matches!(p.stall_release, Some(StallRelease::ShareOfYield(_)))
        && !matches!(p.stall_speed, Some(StallSpeed::MultipleOfStillSpeed(_)))
        && !matches!(p.pot_jump, Some(PotJump::MultipleOfStep4(_)))
        && !matches!(
            p.current_loop,
            Some(CurrentLoop::MultipleOfElectricalCorner(_))
        )
        && !matches!(p.observer, Some(Observer::MultipleOfMechanicalCorner(_)))
}
use crate::why;

/// A version 0 file's `source` block: file references, by what they are
/// the source of.
pub type Source = BTreeMap<String, SourceEntry>;

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum SourceEntry {
    Text(String),
    File {
        file: String,
        #[serde(default)]
        key: Option<String>,
        #[serde(default)]
        sha1: Option<String>,
    },
    Other(IgnoredAny),
}

/// A profile file as read: version 0 with its `source` block, or 1.
#[derive(Clone, Debug)]
pub enum ProfileIn {
    V0 {
        nameplate: Nameplate,
        source: Source,
    },
    V1(ServoProfile),
}

/// A policy file as read: version 0 with its `source` block and the key it
/// takes (its file's name), or 1.
#[derive(Clone, Debug)]
pub enum PolicyIn {
    V0 {
        policy: Box<Policy>,
        source: Source,
        key: String,
    },
    V1(Box<TuningPolicy>),
}

/// What a version 1 pair is made of, and a line for each file read at an
/// older version.
#[derive(Clone, Debug)]
pub struct Upgraded {
    pub profile: ServoProfile,
    pub policy: TuningPolicy,
    pub notes: Vec<String>,
}

/// The nameplate's three choices that move to the policy.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Carried {
    run_v: Option<f64>,
    derate_speed: Option<[u32; 2]>,
    derate_other: Option<f64>,
}

/// `name` as a key: lowercase, every other character a `-`, at most 40.
pub fn slug(name: &str) -> String {
    let mut s = String::new();
    for c in name.chars() {
        let c = c.to_ascii_lowercase();
        if c.is_ascii_lowercase() || c.is_ascii_digit() {
            s.push(c);
        } else if !s.ends_with('-') {
            s.push('-');
        }
    }
    let s = s.trim_matches('-');
    s[..s.len().min(40)].trim_end_matches('-').to_string()
}

fn provenance(source: &Source) -> BTreeMap<String, FieldSource> {
    let mut fields = BTreeMap::new();
    let mut top = FieldSource::default();
    for (k, e) in source {
        match e {
            SourceEntry::Text(t) => match k.as_str() {
                "file" => top.file = Some(t.clone()),
                "key" => top.key = Some(t.clone()),
                "sha1" => top.sha1 = Some(t.clone()),
                _ => {
                    fields.insert(
                        k.clone(),
                        FieldSource {
                            note: Some(t.clone()),
                            ..FieldSource::default()
                        },
                    );
                }
            },
            SourceEntry::File { file, key, sha1 } => {
                fields.insert(
                    k.clone(),
                    FieldSource {
                        how: Some("file".into()),
                        file: Some(file.clone()),
                        key: key.clone(),
                        sha1: sha1.clone(),
                        ..FieldSource::default()
                    },
                );
            }
            SourceEntry::Other(_) => {}
        }
    }
    if top != FieldSource::default() {
        top.how = Some("file".into());
        fields.insert("source".into(), top);
    }
    fields
}

fn profile_of(np: &Nameplate, source: &Source) -> Result<(ServoProfile, Carried), String> {
    if np.retired_ripple_per_rev.is_some() {
        return Err(why::RETIRED_RIPPLE_PER_REV.into());
    }
    let profile = ServoProfile {
        schema: profile::SCHEMA.into(),
        key: slug(&np.name),
        revision: 0,
        model: Model {
            name: np.name.clone(),
            ..Model::default()
        },
        gear_class: np.gear_class,
        gear_ratio: np.gear_ratio,
        gear_stages: Vec::new(),
        brushes: np.brushes,
        rotor_segments: np.rotor_segments,
        travel_deg: np.travel_deg,
        count_rises_ccw: np.count_rises_ccw,
        operating_v: np.operating_v,
        no_load: np.no_load.clone(),
        stall: np
            .stall
            .iter()
            .map(|s| StallRow {
                at_v: s.at_v,
                torque_kg_cm: s.torque_kg_cm,
                current_a: s.current_a,
            })
            .collect(),
        output_stopper: None,
        thermal: None,
        provenance: Provenance {
            document: np.document.clone(),
            atlas: None,
            fields: provenance(source),
        },
    };
    let carried = Carried {
        run_v: np.run_v,
        derate_speed: np.derate_speed,
        derate_other: np.derate_other,
    };
    Ok((profile, carried))
}

fn policy_of(p: &Policy, source: &Source, key: &str) -> TuningPolicy {
    let mut t = TuningPolicy::empty(&slug(key));
    let q = &mut t.protection;
    q.yield_ = p.yield_;
    q.plastic_torque_limit_kg_cm = p.plastic_torque_limit_kg_cm;
    q.above_plastic_limit = p.above_plastic_limit;
    q.stall_release = p.stall_release;
    q.stall_response = p.stall_response;
    q.stall_time_ms = p.stall_time_ms;
    q.stall_speed = p.stall_speed;
    q.cutoff_c = p.cutoff_c;
    if p.pos_error_deg.is_some() || p.pos_error_time_ms.is_some() {
        q.position_error = Some(PositionError {
            deg: p.pos_error_deg,
            time_ms: p.pos_error_time_ms,
        });
    }
    q.duty_max_share = p.duty_max_share;
    q.collision_trip = p.collision_trip;
    let m = &mut t.motion;
    m.move_speed = p.velocity_limit_deg_s.map(MoveSpeed::DegS);
    m.speed_up = p.accel_limit_deg_s2.map(SpeedUp::DegS2);
    m.soft_margin_share = p.soft_margin_share;
    m.angle_cw_stop_deg = p.angle_cw_stop_deg;
    let l = &mut t.loops;
    l.current_loop = p.current_loop;
    l.observer = p.observer;
    l.velocity_em_ratio = p.velocity_em_ratio;
    l.velocity_zero_ratio = p.velocity_zero_ratio;
    l.velocity_shelf_margin_db = p.velocity_shelf_margin_db;
    l.position_split = p.position_split;
    l.hold_band_noise_multiple = p.hold_band_noise_multiple;
    l.hold_band_floor_counts = p.hold_band_floor_counts;
    t.sensor.pot_jump = p.pot_jump;
    t.sensor.bad_samples = p.sensor_bad_samples;
    t.behavior.openloop_zero_brake = p.openloop_zero_brake;
    t.behavior.brush_term = p.brush_term;
    let mut fields = provenance(source);
    t.source = fields.remove("source");
    t
}

/// The carried choices into a policy that does not state them; one it
/// states otherwise is refused, since it is unclear which was meant.
fn carry(c: Carried, t: &mut TuningPolicy) -> Result<(), String> {
    let p = &mut t.protection;
    let differ = |what: &str, v0: String, v1: String| {
        format!(
            "The run voltage and the derates are the policy's now, and the two files disagree: the version 0 nameplate states {what} {v0}, the policy {v1}; take it out of the nameplate"
        )
    };
    match (c.run_v, p.run_v) {
        (Some(a), Some(b)) if a != b => return Err(differ("run_v", a.to_string(), b.to_string())),
        (Some(a), None) => p.run_v = Some(a),
        _ => {}
    }
    match (c.derate_speed, p.speed_ceiling) {
        (Some(a), Some(SpeedCeiling::ShareOfNoLoad(b))) if a != b => {
            return Err(differ(
                "derate_speed",
                format!("{a:?}"),
                format!("speed_ceiling {b:?}"),
            ));
        }
        (Some(a), None) => p.speed_ceiling = Some(SpeedCeiling::ShareOfNoLoad(a)),
        _ => {}
    }
    match (c.derate_other, p.current_limit) {
        (Some(a), Some(CurrentLimit::ShareOfStall(b))) if a != b => {
            return Err(differ(
                "derate_other",
                a.to_string(),
                format!("current_limit {b}"),
            ));
        }
        (Some(a), None) => p.current_limit = Some(CurrentLimit::ShareOfStall(a)),
        _ => {}
    }
    Ok(())
}

/// A profile and a policy at version 1, whatever versions they were read
/// at. With no policy, one holding only what a version 0 nameplate carried:
/// the steps read the run voltage and the derates and nothing else of it.
pub fn upgrade(profile: ProfileIn, policy: Option<PolicyIn>) -> Result<Upgraded, String> {
    let mut notes = Vec::new();
    let (profile, carried) = match profile {
        ProfileIn::V0 { nameplate, source } => {
            let (p, c) = profile_of(&nameplate, &source)?;
            notes.push(format!(
                "the nameplate is version 0: read as the servo profile {}, its run_v, derate_speed and derate_other as the policy's",
                p.key
            ));
            (p, Some(c))
        }
        ProfileIn::V1(p) => (p, None),
    };
    let mut policy = match policy {
        Some(PolicyIn::V0 {
            policy,
            source,
            key,
        }) => {
            let t = policy_of(&policy, &source, &key);
            let board = match policy.oc_trip_samples {
                Some(n) => format!(", its oc_trip_samples {n} left to the board's oc_trip_us"),
                None => String::new(),
            };
            let stated = if all_fixed(&policy) {
                "with every value fixed"
            } else {
                "with each value in the form it states"
            };
            notes.push(format!(
                "the policy is version 0: read as the tuning policy {} {stated}{board}",
                t.key
            ));
            t
        }
        Some(PolicyIn::V1(t)) => *t,
        None => TuningPolicy::empty(&profile.key),
    };
    if let Some(c) = carried {
        carry(c, &mut policy)?;
    }
    Ok(Upgraded {
        profile,
        policy,
        notes,
    })
}
