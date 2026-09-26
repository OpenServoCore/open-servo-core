//! A session procedure expanded against the envelope into the schedules one
//! capture records, with the block map session.py slices them back by.

use anyhow::{Context, Result, anyhow, bail};
use serde::Serialize;

use super::envelope::Envelope;
use super::procs::{Blocks, Procedure};
use crate::sweep::{Decay, Step};

/// Where one block landed in a recording's schedule.
#[derive(Serialize, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Block {
    pub(crate) name: String,
    pub(crate) first: usize,
    pub(crate) count: usize,
}

/// One recording of a capture.
#[derive(Debug)]
pub(crate) struct Plan {
    pub(crate) recording: String,
    pub(crate) decay: Decay,
    pub(crate) schedule: Vec<Step>,
    pub(crate) blocks: Vec<Block>,
}

/// Every recording of capture `n` (1-based), block order rotated by `n - 1`
/// when the procedure rotates.
pub(crate) fn expand(p: &Procedure, env: &Envelope, n: u32) -> Result<Vec<Plan>> {
    if n == 0 {
        bail!("captures count from 1");
    }
    p.recording
        .iter()
        .map(|r| {
            let rot = if p.rotate {
                (n as usize - 1) % r.blocks.len()
            } else {
                0
            };
            let mut schedule = Vec::new();
            let mut blocks = Vec::new();
            for name in r.blocks.iter().cycle().skip(rot).take(r.blocks.len()) {
                let steps = block_steps(&p.block, name, env)
                    .with_context(|| format!("recording {}", r.name))?;
                if steps.is_empty() {
                    bail!("block {name} expands to no steps");
                }
                blocks.push(Block {
                    name: name.clone(),
                    first: schedule.len(),
                    count: steps.len(),
                });
                schedule.extend(steps);
            }
            let mapped: usize = blocks.iter().map(|b| b.count).sum();
            if mapped != schedule.len() {
                bail!("block map covers {mapped} of {} steps", schedule.len());
            }
            Ok(Plan {
                recording: r.name.clone(),
                decay: r.decay,
                schedule,
                blocks,
            })
        })
        .collect()
}

fn block_steps(b: &Blocks, name: &str, env: &Envelope) -> Result<Vec<Step>> {
    Ok(match name {
        "grid" => b
            .grid
            .duties
            .iter()
            .map(|&d| {
                env.windows_ms
                    .get(&d)
                    .map(|&w| Step::Drive(d, Some(w)))
                    .ok_or_else(|| anyhow!("envelope has no window for grid duty {d}%"))
            })
            .collect::<Result<_>>()?,
        "coast" => {
            let c = &b.coast;
            coast_duties(&c.duties, env.coast.top_pct.max(env.coast.probe_pct))
                .into_iter()
                .flat_map(|d| {
                    [
                        Step::Drive(d, Some(c.drive_ms)),
                        Step::Coast(c.coast_ms),
                        Step::Drive(d, Some(c.drive_ms)),
                        Step::Brake(c.coast_ms),
                    ]
                })
                .collect()
        }
        "step" => b.step.steps.clone(),
        "reversal" => b.reversal.steps.clone(),
        "breakaway" => b
            .breakaway
            .duties
            .iter()
            .map(|&d| Step::Drive(d, Some(b.breakaway.window_ms)))
            .collect(),
        _ => bail!("no block {name:?}"),
    })
}

/// The coast block's duties up to `top`, with `top` itself as the top rung.
fn coast_duties(duties: &[u8], top: u8) -> Vec<u8> {
    let mut v: Vec<u8> = duties.iter().copied().filter(|&d| d <= top).collect();
    if !v.contains(&top) {
        v.push(top);
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::{envelope, pilot};

    /// The pinned mg90 envelope with windows for every grid duty.
    fn mg90() -> Envelope {
        let mut env = envelope::mg90();
        env.windows_ms = pilot::windows(env.v_ss.used(), env.limits.runway);
        env
    }

    fn default() -> Procedure {
        Procedure::parse(include_str!("session.toml")).unwrap()
    }

    fn names(p: &Plan) -> Vec<&str> {
        p.blocks.iter().map(|b| b.name.as_str()).collect()
    }

    #[test]
    fn block_order_rotates_by_capture() {
        let (p, env) = (default(), mg90());
        let order = |n| names(&expand(&p, &env, n).unwrap()[0]).join(" ");
        assert_eq!(order(1), "grid coast step reversal breakaway");
        assert_eq!(order(2), "coast step reversal breakaway grid");
        assert_eq!(order(3), "step reversal breakaway grid coast");
        assert_eq!(order(4), "reversal breakaway grid coast step");
        assert_eq!(order(5), "breakaway grid coast step reversal");
        assert_eq!(order(6), order(1));
        assert!(expand(&p, &env, 0).is_err());
    }

    #[test]
    fn rotation_off_keeps_the_listed_order() {
        let mut p = default();
        p.rotate = false;
        let plans = expand(&p, &mg90(), 3).unwrap();
        assert_eq!(names(&plans[0]), Blocks::NAMES);
    }

    #[test]
    fn block_map_tiles_the_schedule() {
        let (p, env) = (default(), mg90());
        for n in 1..=5 {
            for plan in expand(&p, &env, n).unwrap() {
                let mut next = 0;
                for b in &plan.blocks {
                    assert_eq!(b.first, next);
                    next += b.count;
                }
                assert_eq!(next, plan.schedule.len());
            }
        }
        let slow = &expand(&p, &env, 1).unwrap()[0];
        let counts: Vec<usize> = slow.blocks.iter().map(|b| b.count).collect();
        assert_eq!(counts, [20, 20, 8, 8, 18]);
    }

    #[test]
    fn coast_duties_derate_to_the_envelope_top() {
        let d = [20, 30, 40, 60, 80];
        assert_eq!(coast_duties(&d, 75), [20, 30, 40, 60, 75]);
        assert_eq!(coast_duties(&d, 80), d);
        assert_eq!(coast_duties(&d, 100), [20, 30, 40, 60, 80, 100]);

        let p = default();
        let mut env = mg90();
        let coast_top = |env: &Envelope| {
            let plan = &expand(&p, env, 1).unwrap()[0];
            let b = &plan.blocks[1];
            plan.schedule[b.first + b.count - 4]
        };
        env.coast.top_pct = 75;
        assert_eq!(coast_top(&env), Step::Drive(75, Some(80)));
        // under the probe's own duty (already run by the pilot) the probe wins
        env.coast.top_pct = 30;
        assert_eq!(coast_top(&env), Step::Drive(40, Some(80)));
    }

    #[test]
    fn a_grid_duty_without_a_window_is_named() {
        let mut env = mg90();
        env.windows_ms.remove(&35);
        let e = format!("{:#}", expand(&default(), &env, 1).unwrap_err());
        assert!(e.contains("no window for grid duty 35%"), "{e}");
    }

    /// session.py's contract: views are the block names plus `step:W`, which
    /// finds `30@W` followed by a coast exactly once in the slow schedule;
    /// `fast` reads the fast recording whole.
    #[test]
    fn plans_keep_the_session_py_contract() {
        let (p, env) = (default(), mg90());
        for n in 1..=5 {
            let plans = expand(&p, &env, n).unwrap();
            let [slow, fast] = plans.as_slice() else {
                panic!("{} recordings", plans.len());
            };
            assert_eq!((slow.recording.as_str(), slow.decay), ("slow", Decay::Slow));
            let mut views = names(slow);
            views.sort_unstable();
            assert_eq!(views, ["breakaway", "coast", "grid", "reversal", "step"]);
            let sched: Vec<String> = slow.schedule.iter().map(Step::to_string).collect();
            for w in [20, 40, 60, 80, 120] {
                let drive = format!("30@{w}");
                let hits = sched
                    .windows(2)
                    .filter(|p| p[0] == drive && p[1].starts_with("coast:"))
                    .count();
                assert_eq!(hits, 1, "capture {n}: {drive}");
            }

            assert_eq!((fast.recording.as_str(), fast.decay), ("fast", Decay::Fast));
            let grid: Vec<Step> = (1..=20u8)
                .map(|k| Step::Drive(k * 5, Some(env.windows_ms[&(k * 5)])))
                .collect();
            assert_eq!(fast.schedule, grid);
        }
    }
}
