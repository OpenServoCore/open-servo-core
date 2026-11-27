//! A profile and a tuning policy to the absolute values the rules and the
//! steps read, with a line for each value saying what it was stated as and
//! what it binds. The forms that need nothing measured resolve here; a
//! form whose rule this tool does not have yet is refused, naming the fixed
//! form to state instead, so a run never leaves a field out unannounced. A
//! value that cannot act is stored at the bound it is over, and its line
//! says so.

use serde::{Deserialize, Serialize};

use crate::nameplate::{Nameplate, Stall};
use crate::policy::Policy;
use crate::profile::{HeatModel, ServoProfile};
use crate::tuning::{
    CurrentLimit, CurrentLoop, MoveSpeed, Observer, PotJump, SpeedCeiling, SpeedUp, StallRelease,
    StallSpeed, TuningPolicy, Yield,
};

/// The board's facts a policy value resolves from, as the board file
/// names them.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
pub struct BoardFacts {
    pub tick_hz: f64,
    /// The overcurrent detector's window, us.
    #[serde(default)]
    pub oc_trip_us: Option<f64>,
}

#[derive(Clone, Debug)]
pub struct Resolved {
    pub nameplate: Nameplate,
    pub policy: Policy,
    /// The profile's heat model, where it states one.
    pub thermal: Option<HeatModel>,
    pub report: Report,
}

/// What `inputs/resolved.json` holds.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
pub struct Report {
    pub profile: Named,
    pub policy: Named,
    pub fields: Vec<Line>,
}

impl Report {
    /// A line for each value stored at a bound: `field: what it gave and
    /// where it is stored`.
    pub fn clamps(&self) -> Vec<String> {
        self.fields
            .iter()
            .filter_map(|l| Some(format!("{}: {}", l.field, l.clamp.as_ref()?)))
            .collect()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
pub struct Named {
    pub key: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub revision: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "lowercase")]
pub enum Form {
    /// A share or a time over another value the run knows.
    Share,
    /// An absolute value: it ties the policy to one servo.
    Fixed,
    /// A value that is the same on any servo.
    Stated,
    /// The board's.
    Board,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object, rename = "ResolvedValue")
)]
#[serde(untagged)]
pub enum Value {
    Number(f64),
    Ratio([u32; 2]),
    Text(String),
}

/// One policy value as resolved.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
pub struct Line {
    /// The policy's key.
    pub field: &'static str,
    pub form: Form,
    #[cfg_attr(feature = "tsify", tsify(type = "ResolvedValue"))]
    pub value: Value,
    pub unit: &'static str,
    /// What the rules read it as.
    pub binds: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    /// Where the value cannot act: what it gave, the bound it is over, and
    /// the value stored.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clamp: Option<String>,
}

fn not_yet(field: &str, rule: &str, fixed: &str) -> String {
    format!(
        "This tool does not set {field} by its rule yet ({rule}), so the run would leave it as the servo holds it: state it fixed, as {fixed}"
    )
}

struct Lines(Vec<Line>);

impl Lines {
    fn push(
        &mut self,
        field: &'static str,
        form: Form,
        value: Value,
        unit: &'static str,
        binds: &'static str,
    ) {
        self.0.push(Line {
            field,
            form,
            value,
            unit,
            binds,
            note: None,
            clamp: None,
        });
    }

    fn number(
        &mut self,
        field: &'static str,
        form: Form,
        v: Option<f64>,
        unit: &'static str,
        binds: &'static str,
    ) {
        if let Some(v) = v {
            self.push(field, form, Value::Number(v), unit, binds);
        }
    }

    fn text(&mut self, field: &'static str, name: Option<&'static str>, binds: &'static str) {
        if let Some(n) = name {
            self.push(field, Form::Stated, Value::Text(n.into()), "", binds);
        }
    }

    fn note(&mut self, note: String) {
        if let Some(l) = self.0.last_mut() {
            l.note = Some(note);
        }
    }

    fn clamp(&mut self, clamp: String) {
        if let Some(l) = self.0.last_mut() {
            l.clamp = Some(clamp);
        }
    }
}

/// The move speed `v`, deg/s, at most the speed ceiling: over it the
/// ceiling stops the move first.
fn under_ceiling(l: &mut Lines, v: f64, ceiling: f64) -> f64 {
    if v > ceiling {
        l.clamp(format!(
            "{v} deg/s is over the {ceiling} deg/s speed ceiling, where it cannot act: stored as {ceiling} deg/s"
        ));
        return ceiling;
    }
    v
}

/// The absolute inputs from `profile` and `policy`, each refused the
/// consequence first where the run cannot use it. `board` gives the
/// overcurrent window; without it the window is left as the servo holds it.
pub fn resolve(
    profile: &ServoProfile,
    policy: &TuningPolicy,
    board: Option<&BoardFacts>,
) -> Result<Resolved, String> {
    profile.check()?;
    policy.check()?;
    let q = &policy.protection;
    let (Some(SpeedCeiling::ShareOfNoLoad([num, den])), Some(CurrentLimit::ShareOfStall(other))) =
        (q.speed_ceiling, q.current_limit)
    else {
        unreachable!("TuningPolicy::check holds both")
    };
    let nameplate = Nameplate {
        name: profile.key.clone(),
        document: profile.provenance.document.clone(),
        gear_class: profile.gear_class,
        gear_ratio: profile.gear_ratio,
        count_rises_ccw: profile.count_rises_ccw,
        travel_deg: profile.travel_deg,
        operating_v: profile.operating_v,
        no_load: profile.no_load.clone(),
        stall: profile
            .stall
            .iter()
            .map(|s| Stall {
                at_v: s.at_v,
                current_a: s.current_a,
            })
            .collect(),
        brushes: profile.brushes,
        rotor_segments: profile.rotor_segments,
        retired_ripple_per_rev: None,
        run_v: q.run_v,
        derate_speed: Some([num, den]),
        derate_other: Some(other),
    };
    nameplate.check_user()?;
    let (Some(speed), Some(stall), Some(ceiling)) = (
        nameplate.speed_row(),
        nameplate.stall_row(),
        nameplate.speed_ceiling_deg_s(),
    ) else {
        unreachable!("Nameplate::check_user holds both rows and the derate")
    };

    let mut out = Policy::default();
    let mut l = Lines(Vec::new());
    l.number(
        "protection.run_v",
        Form::Stated,
        q.run_v,
        "V",
        "nameplate.run_v",
    );
    l.push(
        "protection.speed_ceiling",
        Form::Share,
        Value::Ratio([num, den]),
        "of the stated no-load speed",
        "nameplate.derate_speed",
    );
    l.note(format!("{ceiling} deg/s at the {} V rating", speed.at_v));
    l.push(
        "protection.current_limit",
        Form::Share,
        Value::Number(other),
        "of the stall current, the stated volts over R",
        "nameplate.derate_other",
    );
    let vr = format!(
        "of the {} V rating over the identified R, set by the run",
        stall.at_v
    );
    l.note(match stall.current_a {
        Some(a) => format!("{vr}; the {a} A the profile states there is reported beside it"),
        None => vr,
    });
    match q.yield_ {
        Some(Yield::Ma(ma)) => {
            out.yield_current_ma = Some(ma);
            l.number(
                "protection.yield",
                Form::Fixed,
                Some(ma),
                "mA",
                "policy.yield_current_ma",
            );
        }
        Some(Yield::TorqueKgCm(_)) => {
            return Err(not_yet(
                "protection.yield",
                "a max yield torque over the stated stall torque, times the stated volts over R",
                "{\"ma\": ...}",
            ));
        }
        None => {}
    }
    match q.stall_release {
        Some(StallRelease::Ma(ma)) => {
            out.stall_release_ma = Some(ma);
            l.number(
                "protection.stall_release",
                Form::Fixed,
                Some(ma),
                "mA",
                "policy.stall_release_ma",
            );
        }
        Some(StallRelease::ShareOfYield(_)) => {
            return Err(not_yet(
                "protection.stall_release",
                "a share of the yield current as the kernel runs it",
                "{\"ma\": ...}",
            ));
        }
        None => {}
    }
    out.stall_response = q.stall_response;
    l.text(
        "protection.stall_response",
        q.stall_response.map(|r| r.name()),
        "policy.stall_response",
    );
    out.stall_time_ms = q.stall_time_ms;
    l.number(
        "protection.stall_time_ms",
        Form::Stated,
        q.stall_time_ms,
        "ms",
        "policy.stall_time_ms",
    );
    match q.stall_speed {
        Some(StallSpeed::DegS(v)) => {
            out.stall_omega_max_deg_s = Some(v);
            l.number(
                "protection.stall_speed",
                Form::Fixed,
                Some(v),
                "deg/s",
                "policy.stall_omega_max_deg_s",
            );
        }
        Some(StallSpeed::MultipleOfStillSpeed(_)) => {
            return Err(not_yet(
                "protection.stall_speed",
                "a multiple of the largest observer speed over the check's still holds",
                "{\"deg_s\": ...}",
            ));
        }
        None => {}
    }
    out.cutoff_c = q.cutoff_c;
    l.number(
        "protection.cutoff_c",
        Form::Stated,
        q.cutoff_c,
        "C",
        "policy.cutoff_c",
    );
    if let Some(e) = q.position_error {
        out.pos_error_deg = e.deg;
        out.pos_error_time_ms = e.time_ms;
        l.number(
            "protection.position_error.deg",
            Form::Stated,
            e.deg,
            "deg",
            "policy.pos_error_deg",
        );
        l.number(
            "protection.position_error.time_ms",
            Form::Stated,
            e.time_ms,
            "ms",
            "policy.pos_error_time_ms",
        );
    }
    out.duty_max_share = q.duty_max_share;
    l.number(
        "protection.duty_max_share",
        Form::Stated,
        q.duty_max_share,
        "of the rail",
        "policy.duty_max_share",
    );
    out.collision_trip = q.collision_trip;
    l.text(
        "protection.collision_trip",
        q.collision_trip.map(|s| s.name()),
        "policy.collision_trip",
    );

    let m = &policy.motion;
    match m.move_speed {
        Some(MoveSpeed::ShareOfCeiling(s)) => {
            let v = s * ceiling;
            l.number(
                "motion.move_speed",
                Form::Share,
                Some(s),
                "of the speed ceiling",
                "policy.velocity_limit_deg_s",
            );
            l.note(format!("{v} deg/s"));
            out.velocity_limit_deg_s = Some(under_ceiling(&mut l, v, ceiling));
        }
        Some(MoveSpeed::DegS(v)) => {
            l.number(
                "motion.move_speed",
                Form::Fixed,
                Some(v),
                "deg/s",
                "policy.velocity_limit_deg_s",
            );
            out.velocity_limit_deg_s = Some(under_ceiling(&mut l, v, ceiling));
        }
        None => {}
    }
    match m.speed_up {
        Some(SpeedUp::S(t)) => {
            let Some(v) = out.velocity_limit_deg_s else {
                return Err("The speed-up time is the time to the move speed, and the policy states no motion.move_speed: state it, or the acceleration fixed as {\"deg_s2\": ...}".into());
            };
            if !(t.is_finite() && t > 0.0) {
                return Err(format!(
                    "A move cannot reach its speed in no time: the policy's motion.speed_up is {t} s; state a time over 0"
                ));
            }
            let a = v / t;
            out.accel_limit_deg_s2 = Some(a);
            l.number(
                "motion.speed_up",
                Form::Share,
                Some(t),
                "s to the move speed",
                "policy.accel_limit_deg_s2",
            );
            l.note(format!("{a} deg/s^2"));
        }
        Some(SpeedUp::DegS2(a)) => {
            out.accel_limit_deg_s2 = Some(a);
            l.number(
                "motion.speed_up",
                Form::Fixed,
                Some(a),
                "deg/s^2",
                "policy.accel_limit_deg_s2",
            );
        }
        None => {}
    }
    out.soft_margin_share = m.soft_margin_share;
    l.number(
        "motion.soft_margin_share",
        Form::Stated,
        m.soft_margin_share,
        "of the stops' span",
        "policy.soft_margin_share",
    );
    out.angle_cw_stop_deg = m.angle_cw_stop_deg;
    l.number(
        "motion.angle_cw_stop_deg",
        Form::Stated,
        m.angle_cw_stop_deg,
        "deg",
        "policy.angle_cw_stop_deg",
    );

    let p = &policy.loops;
    match p.current_loop {
        Some(CurrentLoop::Hz(hz)) => {
            out.current_loop_hz = Some(hz);
            l.number(
                "loops.current_loop",
                Form::Fixed,
                Some(hz),
                "Hz",
                "policy.current_loop_hz",
            );
        }
        Some(CurrentLoop::MultipleOfElectricalCorner(_)) => {
            return Err(not_yet(
                "loops.current_loop",
                "a multiple of the electrical corner R / (2 pi L)",
                "{\"hz\": ...}",
            ));
        }
        None => {}
    }
    match p.observer {
        Some(Observer::Hz(hz)) => {
            out.observer_hz = Some(hz);
            l.number(
                "loops.observer",
                Form::Fixed,
                Some(hz),
                "Hz",
                "policy.observer_hz",
            );
        }
        Some(Observer::MultipleOfMechanicalCorner(_)) => {
            return Err(not_yet(
                "loops.observer",
                "a multiple of the mechanical corner Ke^2 / (2 pi J R)",
                "{\"hz\": ...}",
            ));
        }
        None => {}
    }
    for (field, v, binds, slot) in [
        (
            "loops.velocity_em_ratio",
            p.velocity_em_ratio,
            "policy.velocity_em_ratio",
            &mut out.velocity_em_ratio,
        ),
        (
            "loops.velocity_zero_ratio",
            p.velocity_zero_ratio,
            "policy.velocity_zero_ratio",
            &mut out.velocity_zero_ratio,
        ),
        (
            "loops.position_split",
            p.position_split,
            "policy.position_split",
            &mut out.position_split,
        ),
        (
            "loops.hold_band_noise_multiple",
            p.hold_band_noise_multiple,
            "policy.hold_band_noise_multiple",
            &mut out.hold_band_noise_multiple,
        ),
    ] {
        *slot = v;
        l.number(field, Form::Stated, v, "", binds);
    }
    out.velocity_shelf_margin_db = p.velocity_shelf_margin_db;
    l.number(
        "loops.velocity_shelf_margin_db",
        Form::Stated,
        p.velocity_shelf_margin_db,
        "dB",
        "policy.velocity_shelf_margin_db",
    );
    out.hold_band_floor_counts = p.hold_band_floor_counts;
    l.number(
        "loops.hold_band_floor_counts",
        Form::Stated,
        p.hold_band_floor_counts,
        "raw counts",
        "policy.hold_band_floor_counts",
    );

    let s = &policy.sensor;
    match s.pot_jump {
        Some(PotJump::Counts(n)) => {
            out.sensor_delta_max_counts = Some(n);
            l.number(
                "sensor.pot_jump",
                Form::Fixed,
                Some(n),
                "raw counts",
                "policy.sensor_delta_max_counts",
            );
        }
        Some(PotJump::MultipleOfStep4(_)) => {
            return Err(not_yet(
                "sensor.pot_jump",
                "a multiple of the worst run of 4 successive pot steps while driving",
                "{\"counts\": ...}",
            ));
        }
        None => {}
    }
    out.sensor_bad_samples = s.bad_samples;
    l.number(
        "sensor.bad_samples",
        Form::Stated,
        s.bad_samples,
        "samples",
        "policy.sensor_bad_samples",
    );

    let b = &policy.behavior;
    out.openloop_zero_brake = b.openloop_zero_brake;
    l.text(
        "behavior.openloop_zero_brake",
        b.openloop_zero_brake.map(|s| s.name()),
        "policy.openloop_zero_brake",
    );
    out.brush_term = b.brush_term;
    l.text(
        "behavior.brush_term",
        b.brush_term.map(|s| s.name()),
        "policy.brush_term",
    );

    if let Some((us, hz)) = board.and_then(|b| b.oc_trip_us.map(|us| (us, b.tick_hz))) {
        let n = us * hz / 1e6;
        out.oc_trip_samples = Some(n);
        l.number(
            "oc_trip",
            Form::Board,
            Some(n),
            "samples",
            "policy.oc_trip_samples",
        );
        l.note(format!("the board's oc_trip_us {us} at {hz} Hz"));
    }

    Ok(Resolved {
        nameplate,
        policy: out,
        thermal: profile.thermal,
        report: Report {
            profile: Named {
                key: profile.key.clone(),
                revision: Some(profile.revision),
            },
            policy: Named {
                key: policy.key.clone(),
                revision: Some(policy.revision),
            },
            fields: l.0,
        },
    })
}
