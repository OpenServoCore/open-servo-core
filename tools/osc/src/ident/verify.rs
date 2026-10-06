//! `ident verify`'s recording, in the sweep's layout: `meta.json` (the
//! servo, its limits and both checks' plans, then their scores) and one CSV
//! per check of every telemetry read it scored, each row with the mode,
//! torque and goals it was read under. Saved whether the check passed,
//! failed or aborted, so a check re-scores offline.

use std::io::Write;

use anyhow::{Context, Result};
use osc_client::Id;
use osc_ident::burst::Capture;
use osc_ident::exp::verify::{
    VERIFY_CURRENT, VerifyCurrent, VerifyCurrentCfg, VerifyResult, VerifyVelocity,
    VerifyVelocityCfg,
};
use osc_ident::exp::{AbortReason, Cmd, Experiment, Guarded, Permitted, RigParams};
use osc_ident::frame::{TelBurst, TelemetrySnapshot};
use osc_ident::limits::ServoLimits;
use osc_ident::regs::control;
use serde_json::{Value, json};

use crate::rig::check_abort;
use crate::rig::csvio::{OutDir, SNAPSHOT_COLUMNS, snapshot_fields};
use crate::rig::limits::Stall;
use crate::rig::pump::Pump;
use crate::rig::servo::{Servo, guard};

pub(super) const META: &str = "meta.json";
pub(super) const CURRENT_CSV: &str = "verify_current.csv";
pub(super) const VELOCITY_CSV: &str = "verify_velocity.csv";

/// What the servo was told before a read: the context columns of a row.
#[derive(Copy, Clone, Default)]
struct Told {
    mode: i32,
    torque: i32,
    goal_duty: i32,
    goal_current: i32,
    goal_velocity: i32,
}

const TOLD_COLUMNS: &str = "mode,torque_enable,goal_duty,goal_current,goal_velocity";

/// An experiment with every snapshot it was handed kept beside what it had
/// told the servo by then.
struct Recorded<E> {
    exp: E,
    told: Told,
    rows: Vec<(Told, TelemetrySnapshot)>,
}

impl<E: Experiment> Recorded<E> {
    fn new(exp: E) -> Self {
        Self {
            exp,
            told: Told::default(),
            rows: Vec::new(),
        }
    }

    fn save(&self, out: &OutDir, name: &str) -> Result<()> {
        let mut w = out.file(name)?;
        writeln!(w, "{TOLD_COLUMNS},{SNAPSHOT_COLUMNS}")?;
        for (t, s) in &self.rows {
            writeln!(
                w,
                "{},{},{},{},{},{}",
                t.mode,
                t.torque,
                t.goal_duty,
                t.goal_current,
                t.goal_velocity,
                snapshot_fields(s.host_ms, s)
            )?;
        }
        w.flush()
            .with_context(|| format!("write {}", out.0.join(name).display()))
    }
}

impl<E: Experiment> Experiment for Recorded<E> {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        if let Some(s) = obs {
            self.rows.push((self.told, *s));
        }
        let cmd = self.exp.step(obs);
        if let Cmd::Write { reg, value } = &cmd {
            let t = &mut self.told;
            match *reg {
                control::MODE => t.mode = *value,
                control::TORQUE_ENABLE => t.torque = *value,
                control::GOAL_DUTY => t.goal_duty = *value,
                control::GOAL_CURRENT => t.goal_current = *value,
                control::GOAL_VELOCITY => t.goal_velocity = *value,
                _ => {}
            }
        }
        cmd
    }

    fn push_tel(&mut self, tel: &TelBurst) {
        self.exp.push_tel(tel);
    }

    fn push_burst(&mut self, cap: &Capture) {
        self.exp.push_burst(cap);
    }

    fn halted(&self) -> Option<AbortReason> {
        self.exp.halted()
    }
}

/// One check run to its end on `s` under `params`, its reads saved as
/// `name` before any abort or error is returned.
fn check<S: Servo, E: Experiment>(
    s: &mut S,
    out: &OutDir,
    name: &str,
    exp: E,
    params: RigParams,
) -> Result<(E, Option<AbortReason>)> {
    let mut g = Guarded::new(Recorded::new(exp), params);
    let ran = guard(s, |s| Pump::on(&mut *s, None).run(&mut g));
    let abort = g.abort();
    let rec = g.into_inner();
    rec.save(out, name)?;
    ran?;
    Ok((rec.exp, abort))
}

/// The two checks' plans as meta.json records them.
fn plans(cur: &VerifyCurrentCfg, vel: &VerifyVelocityCfg, params: &RigParams) -> Value {
    json!({
        "guard": params.pos_guard.map(|(lo, hi)| [lo, hi]),
        "i_abort_counts": params.i_abort,
        "current": {
            "seek_duty_q15": cur.seek_duty_q15,
            "steps_counts": cur.steps_counts,
            "dwell_polls": cur.dwell_polls,
            "poll_ms": cur.poll_ms,
            "rest_ms": cur.rest_ms,
            "tol": cur.tol,
        },
        "velocity": {
            "seek_duty_q15": vel.seek_duty_q15,
            "legs_cps": vel.legs_cps,
            "poll_ms": vel.poll_ms,
            "rest_ms": vel.rest_ms,
            "margin": vel.margin,
            "seek_margin": vel.seek_margin,
            "accel_trim_ms": vel.accel_trim_ms,
            "tol": vel.tol,
            "leg_cap_polls": vel.leg_cap_polls,
        },
    })
}

fn scores(v: &VerifyResult) -> Value {
    json!({
        "pass": v.pass,
        "current": v.current.as_ref().map(|c| c.steps.iter().map(|s| json!({
            "goal": s.goal,
            "mean_i": s.mean_i,
            "err_pct": s.err_pct,
            "settle_ms": s.settle_ms,
            "windows": s.windows,
            "pass": s.pass,
        })).collect::<Vec<_>>()),
        "velocity": v.velocity.as_ref().map(|l| l.legs.iter().map(|l| json!({
            "goal_cps": l.goal_cps,
            "meas_cps": l.meas_cps,
            "err_pct": l.err_pct,
            "r2": l.r2,
            "n": l.n,
            "pass": l.pass,
        })).collect::<Vec<_>>()),
    })
}

fn write_meta(out: &OutDir, meta: &Value) -> Result<()> {
    let path = out.0.join(META);
    std::fs::write(&path, serde_json::to_string_pretty(meta)?)
        .with_context(|| format!("write {}", path.display()))
}

/// Both checks on `s`, recorded into `out`: meta.json first, each check's
/// CSV as it ends, the scores into meta.json once both ran. `between` runs
/// after each check with the name of the one that ended.
pub(super) fn run<S: Servo>(
    s: &mut S,
    out: &OutDir,
    lim: &ServoLimits,
    params: RigParams,
    current: VerifyCurrentCfg,
    velocity: VerifyVelocityCfg,
    mut between: impl FnMut(&mut S, &'static str) -> Result<()>,
) -> Result<VerifyResult> {
    let id: Id = s.id();
    let stall = Stall::read(s.client(), id)?;
    let drive = crate::sweep::drive(lim, &stall, current.seek_duty_q15);
    let mut meta = Value::Object(crate::sweep::servo_meta(s.client(), id)?);
    meta["drive"] = Value::Object(drive);
    meta["verify"] = plans(&current, &velocity, &params);
    write_meta(out, &meta)?;

    // deliberate rail stall in Current mode: the directional endstop band
    // would zero i_ref at the soft wall, and the permit opens it, as for
    // resistance
    let e5 = Permitted::new(VerifyCurrent::new(current, &params));
    let (e5, abort) = check(s, out, CURRENT_CSV, e5, params.without_pos_guard())?;
    check_abort(VERIFY_CURRENT, abort)?;
    let cur = e5.into_inner().result();
    between(s, VERIFY_CURRENT)?;

    let e6 = VerifyVelocity::new(velocity, &params);
    let (e6, abort) = check(s, out, VELOCITY_CSV, e6, params)?;
    check_abort("verify velocity", abort)?;
    let vel = e6.result();
    between(s, "verify velocity")?;

    let v = VerifyResult::assemble(Some(cur), Some(vel));
    meta["result"] = scores(&v);
    write_meta(out, &meta)?;
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::Supply;
    use crate::rig::servo::bench::Bench;

    /// The bench MG90 on USB through both checks: the run's dir holds
    /// meta.json with the servo, the plans and the scores, and one CSV per
    /// check whose rows carry the goals they were read under - the current
    /// check's steps into the stops, the velocity check's legs.
    #[test]
    fn a_verify_run_leaves_its_recording() {
        let mut b = Bench::mg90(Supply::Usb);
        let id = b.id();
        // the table's rail is 2S, where verify current has no room
        let lim = ServoLimits {
            vbus: 1780,
            ..crate::rig::limits::read(&mut b.c, id).unwrap()
        };
        let plan = lim.stall_plan(lim.r_vpc().unwrap_or(7270.0 / 4096.0), None);
        let current = VerifyCurrentCfg::planned(&plan, lim.window_floor()).unwrap();
        let steps = current.steps_counts.clone();
        let velocity = VerifyVelocityCfg::planned(&plan);
        let params = RigParams::new(Some((532, 3526)), lim.abort_default())
            .with_floor(lim.window_floor_q15)
            .with_stops(lim.raw);
        let base = std::env::temp_dir().join(format!("osc-verify-{}", std::process::id()));
        let out = OutDir::create(&base).unwrap();
        let mut ended = Vec::new();
        let v = run(&mut b, &out, &lim, params, current, velocity, |b, done| {
            ended.push(done);
            b.servo.pos = 2029.0;
            Ok(())
        })
        .unwrap();
        assert_eq!(ended, [VERIFY_CURRENT, "verify velocity"]);

        let meta: Value =
            serde_json::from_str(&std::fs::read_to_string(out.0.join(META)).unwrap()).unwrap();
        assert_eq!(meta["verify"]["current"]["steps_counts"], json!(steps));
        assert_eq!(meta["verify"]["velocity"]["legs_cps"], json!([600, 1200]));
        assert_eq!(meta["result"]["pass"], json!(v.pass));
        assert!(meta["tick_hz"].is_u64() && meta["drive"]["current_limit_counts"] == 280);
        let n = v.current.as_ref().unwrap().steps.len();
        assert_eq!(meta["result"]["current"].as_array().unwrap().len(), n);

        let rows = |name: &str| -> Vec<Vec<String>> {
            std::fs::read_to_string(out.0.join(name))
                .unwrap()
                .lines()
                .map(|l| l.split(',').map(String::from).collect())
                .collect()
        };
        let col = |rows: &[Vec<String>], name: &str| -> Vec<i32> {
            let k = rows[0].iter().position(|c| c == name).unwrap();
            rows[1..].iter().map(|r| r[k].parse().unwrap()).collect()
        };
        let cur = rows(CURRENT_CSV);
        assert_eq!(
            cur[0].join(","),
            format!("{TOLD_COLUMNS},{SNAPSHOT_COLUMNS}")
        );
        let goals = col(&cur, "goal_current");
        for s in steps {
            assert!(
                goals.contains(&(s as i32)) && goals.contains(&-(s as i32)),
                "{s}"
            );
        }
        let vel = rows(VELOCITY_CSV);
        let goals = col(&vel, "goal_velocity");
        for g in [600, -600, 1200, -1200] {
            assert!(goals.contains(&g), "{g}");
        }
        assert!(col(&vel, "mode").contains(&2));
        let _ = std::fs::remove_dir_all(&base);
    }
}
