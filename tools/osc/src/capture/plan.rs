//! A session procedure expanded against the envelope into the schedules one
//! capture records, with the block map session.py slices them back by. Only
//! what the pilot ran on the servo is planned: grid duties up to the highest
//! it kept, at the windows it measured under the recording's decay, coast
//! duties its coast ladder ran, and chains it ran both ways. What the pilot
//! found does not fit this servo is dropped, and the plan says so; what it
//! never ran, or a decay it has no pass under, sends the operator back to
//! it.

use anyhow::{Context, Result, anyhow, bail};
use serde::{Deserialize, Serialize};

use super::envelope::{Envelope, Grid};
use super::pilot::{chain_name, chains_of};
use super::procs::{Blocks, Procedure};
use crate::sweep::{Decay, Step};

/// Where one block landed in a recording's schedule.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq)]
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
    /// What the procedure asks and the envelope does not cover, dropped.
    pub(crate) dropped: Vec<String>,
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
            let pass = pass(env, r.decay).with_context(|| format!("recording {}", r.name))?;
            let rot = if p.rotate {
                (n as usize - 1) % r.blocks.len()
            } else {
                0
            };
            let mut schedule = Vec::new();
            let mut blocks = Vec::new();
            let mut dropped = Vec::new();
            for name in r.blocks.iter().cycle().skip(rot).take(r.blocks.len()) {
                let steps = block_steps(&p.block, name, env, pass, &mut dropped)
                    .with_context(|| format!("recording {}", r.name))?;
                if steps.is_empty() {
                    dropped.push(format!(
                        "{name}: nothing the pilot kept is left, so it is left out"
                    ));
                    continue;
                }
                blocks.push(Block {
                    name: name.clone(),
                    first: schedule.len(),
                    count: steps.len(),
                });
                schedule.extend(steps);
            }
            if schedule.is_empty() {
                bail!(
                    "recording {}: nothing the pilot kept is left to drive",
                    r.name
                );
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
                dropped,
            })
        })
        .collect()
}

/// The grid pass the pilot measured under `decay`.
fn pass(env: &Envelope, decay: Decay) -> Result<&Grid> {
    match decay {
        Decay::Slow => Ok(&env.grid),
        Decay::Fast => env.fast.as_ref().ok_or_else(|| {
            anyhow!(
                "it drives under fast decay and the envelope holds no pilot pass under it: run \
                 `osc capture pilot` again"
            )
        }),
    }
}

/// A block's steps against the envelope, what it drops noted in `dropped`.
fn block_steps(
    b: &Blocks,
    name: &str,
    env: &Envelope,
    pass: &Grid,
    dropped: &mut Vec<String>,
) -> Result<Vec<Step>> {
    Ok(match name {
        "grid" => {
            let over: Vec<String> = b
                .grid
                .duties
                .iter()
                .filter(|&&d| d > pass.top_pct)
                .map(|d| d.to_string())
                .collect();
            if !over.is_empty() {
                let why = pass
                    .refused
                    .as_ref()
                    .map_or(String::new(), |r| format!(": {}", r.why));
                dropped.push(format!(
                    "grid: {}% dropped, over {}%, the highest duty the pilot kept both ways{why}",
                    over.join(", "),
                    pass.top_pct
                ));
            }
            b.grid
                .duties
                .iter()
                .filter(|&&d| d <= pass.top_pct)
                .map(|&d| {
                    pass.rungs
                        .iter()
                        .find(|r| r.pct == d)
                        .map(|r| Step::Drive(d, Some(r.window_ms)))
                        .ok_or_else(|| anyhow!("envelope has no window for grid duty {d}%"))
                })
                .collect::<Result<_>>()?
        }
        "coast" => {
            let (c, e) = (&b.coast, &env.coast);
            if c.coast_ms != e.coast_ms {
                bail!(
                    "coast block coasts {} ms; the envelope's ladder coasted {} ms: rerun osc \
                     capture pilot",
                    c.coast_ms,
                    e.coast_ms
                );
            }
            let over: Vec<String> = c
                .duties
                .iter()
                .filter(|&&d| d > e.top_pct)
                .map(|d| d.to_string())
                .collect();
            if !over.is_empty() {
                dropped.push(format!(
                    "coast: {}% dropped, over {}%, the highest duty the pilot's coast ladder kept",
                    over.join(", "),
                    e.top_pct
                ));
            }
            coast_duties(&c.duties, e.top_pct)
                .into_iter()
                .map(|d| {
                    let r = e.ladder.iter().find(|r| r.pct == d).ok_or_else(|| {
                        anyhow!(
                            "coast duty {d}% never ran on the pilot's ladder: rerun osc capture \
                             pilot"
                        )
                    })?;
                    Ok([
                        Step::Drive(d, Some(r.drive_ms)),
                        Step::Coast(c.coast_ms),
                        Step::Drive(d, Some(r.drive_ms)),
                        Step::Brake(c.coast_ms),
                    ])
                })
                .collect::<Result<Vec<_>>>()?
                .into_iter()
                .flatten()
                .collect()
        }
        "step" => ran(env, "step", &b.step.steps, dropped)?,
        "reversal" => ran(env, "reversal", &b.reversal.steps, dropped)?,
        "ends" => ran(env, "ends", &b.ends.steps, dropped)?,
        "breakaway" => b
            .breakaway
            .duties
            .iter()
            .map(|&d| Step::Drive(d, Some(b.breakaway.window_ms)))
            .collect(),
        _ => bail!("no block {name:?}"),
    })
}

/// A chain block's steps, each chain one the pilot ran both ways; a chain
/// the pilot refused is dropped, one it never saw refuses the plan.
fn ran(
    env: &Envelope,
    block: &str,
    steps: &[Step],
    dropped: &mut Vec<String>,
) -> Result<Vec<Step>> {
    let mut kept = Vec::new();
    for chain in chains_of(steps) {
        let name = chain_name(&chain);
        if env.chain(block, &name).is_some() {
            kept.extend(chain);
            continue;
        }
        match env
            .refused_chains
            .iter()
            .find(|c| c.block == block && c.chain == name)
        {
            Some(r) => dropped.push(format!(
                "{block}: {name} dropped, the pilot refused it: {}",
                r.why
            )),
            None => bail!(
                "the {block} chain {name} never ran on the pilot: run `osc capture pilot` again"
            ),
        }
    }
    Ok(kept)
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
    use crate::capture::envelope::{self, ChainRefused};

    /// The bench servo's envelope as the pilot writes it on the test servo.
    fn mg90() -> Envelope {
        envelope::mg90()
    }

    fn default() -> Procedure {
        Procedure::parse(include_str!("session.toml")).unwrap()
    }

    fn names(p: &Plan) -> Vec<&str> {
        p.blocks.iter().map(|b| b.name.as_str()).collect()
    }

    fn err(p: &Procedure, env: &Envelope) -> String {
        format!("{:#}", expand(p, env, 1).unwrap_err())
    }

    #[test]
    fn block_order_rotates_by_capture() {
        let (p, env) = (default(), mg90());
        let order = |n| names(&expand(&p, &env, n).unwrap()[0]).join(" ");
        assert_eq!(order(1), "grid coast step reversal breakaway ends");
        assert_eq!(order(2), "coast step reversal breakaway ends grid");
        assert_eq!(order(3), "step reversal breakaway ends grid coast");
        assert_eq!(order(4), "reversal breakaway ends grid coast step");
        assert_eq!(order(5), "breakaway ends grid coast step reversal");
        assert_eq!(order(6), "ends grid coast step reversal breakaway");
        assert_eq!(order(7), order(1));
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
        for n in 1..=6 {
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
        assert_eq!(counts, [11, 12, 10, 4, 18, 2]);
    }

    /// The grid runs to the highest duty the pilot kept both ways, 55% on
    /// the bench servo, each rung at the window it measured; a lower top
    /// drops the duties over it.
    #[test]
    fn grid_duties_stop_at_the_envelope_top() {
        let (p, mut env) = (default(), mg90());
        let grid = |env: &Envelope| -> Vec<String> {
            let plan = &expand(&p, env, 1).unwrap()[0];
            let b = &plan.blocks[0];
            plan.schedule[b.first..b.first + b.count]
                .iter()
                .map(Step::to_string)
                .collect()
        };
        assert_eq!(
            grid(&env),
            [
                "5@1500", "10@1500", "15@1064", "20@735", "25@571", "30@471", "35@406", "40@361",
                "45@329", "50@305", "55@286"
            ]
        );
        env.grid.top_pct = 40;
        env.grid.rungs.retain(|r| r.pct <= 40);
        assert_eq!(grid(&env).last().unwrap(), "40@361");
        assert_eq!(grid(&env).len(), 8);
    }

    #[test]
    fn coast_duties_derate_to_the_envelope_top() {
        let d = [20, 30, 40, 60, 80];
        assert_eq!(coast_duties(&d, 75), [20, 30, 40, 60, 75]);
        assert_eq!(coast_duties(&d, 80), d);
        assert_eq!(coast_duties(&d, 100), [20, 30, 40, 60, 80, 100]);

        // each duty drives for its own drive_ms: to its goal and 20 ms on
        let p = default();
        let mut env = mg90();
        let coast = |env: &Envelope| {
            let plan = &expand(&p, env, 1).unwrap()[0];
            let b = &plan.blocks[1];
            plan.schedule[b.first..b.first + b.count].to_vec()
        };
        assert_eq!(
            coast(&env)
                .iter()
                .filter(|s| matches!(s, Step::Drive(..)))
                .map(Step::to_string)
                .collect::<Vec<_>>(),
            ["20@35", "20@35", "30@61", "30@61", "40@92", "40@92"]
        );
        env.coast.top_pct = 30;
        assert_eq!(coast(&env).len(), 8);

        // a duty the pilot's ladder never ran
        env.coast.top_pct = 60;
        assert!(err(&p, &env).contains("coast duty 60% never ran"));
        env.coast.top_pct = 40;
        env.coast.ladder.retain(|r| r.pct != 30);
        assert!(err(&p, &env).contains("coast duty 30% never ran"));
        let mut env = mg90();
        env.coast.coast_ms = 300;
        assert!(
            err(&p, &env).contains("coast block coasts 400 ms; the envelope's ladder coasted 300")
        );
    }

    #[test]
    fn a_grid_duty_without_a_window_is_named() {
        let mut env = mg90();
        env.grid.rungs.retain(|r| r.pct != 35);
        let e = err(&default(), &env);
        assert!(e.contains("no window for grid duty 35%"), "{e}");
    }

    /// Every step, reversal and ends chain must be one the pilot ran both
    /// ways: one it never ran sends the operator back to it; one it refused
    /// is dropped, and the plan says why. A 40% reversal is an operator's
    /// edit, and the pilot runs it once the procedure names it.
    #[test]
    fn a_chain_the_pilot_never_ran_is_refused() {
        let p = default();
        let mut env = mg90();
        env.chains.retain(|c| c.chain != "30@40,coast:400");
        assert_eq!(
            err(&p, &env),
            "recording slow: the step chain 30@40,coast:400 never ran on the pilot: run `osc \
             capture pilot` again"
        );
        env.refused_chains.push(ChainRefused {
            block: "step".into(),
            chain: "30@40,coast:400".into(),
            predicted: 3300,
            why: "its excursion does not fit the room to the soft limit".into(),
        });
        let slow = &expand(&p, &env, 1).unwrap()[0];
        assert_eq!(
            slow.dropped.last().unwrap(),
            "step: 30@40,coast:400 dropped, the pilot refused it: its excursion does not fit the \
             room to the soft limit"
        );
        let step = &slow.blocks[2];
        assert_eq!((step.name.as_str(), step.count), ("step", 8));

        let mut env = mg90();
        env.chains.retain(|c| c.block != "ends");
        assert!(err(&p, &env).contains("the ends chain 15@1277 never ran"));

        let edited = Procedure::parse(&include_str!("session.toml").replace(
            "steps = [\"20@60\", \"then:-20@60\", \"then:20@60\", \"brake:200\"]",
            "steps = [\"20@60\", \"then:-20@60\", \"then:20@60\", \"brake:200\", \"40@60\", \
             \"then:-40@60\", \"then:40@60\", \"brake:200\"]",
        ))
        .unwrap();
        assert!(expand(&edited, &mg90(), 1).is_ok(), "the pilot ran it");
        let mut env = mg90();
        env.chains.retain(|c| !c.chain.starts_with("40@60"));
        assert!(
            err(&edited, &env)
                .contains("the reversal chain 40@60,then:-40@60,then:40@60,brake:200 never ran")
        );
    }

    /// The bench servo's envelope while its pilot kept only 5 to 20%: the
    /// 25% reverse rung never held its goal, the coast ladder topped at 20%,
    /// and it refused a 40% reversal. The plan drops the grid and coast
    /// duties over 20% and says why, keeps every chain the pilot ran, and
    /// still plans every block.
    #[test]
    fn a_short_envelope_plans_what_it_covers() {
        let mut env = mg90();
        let why = "rev 25% did not hold its goal duty for 60 ms past the settle in 109 ms";
        for g in [&mut env.grid, env.fast.as_mut().unwrap()] {
            g.rungs.retain(|r| r.pct <= 20);
            g.top_pct = 20;
            g.refused = Some(envelope::LadderRefused {
                pct: 25,
                dir: envelope::Dir::Rev,
                why: why.into(),
            });
        }
        env.coast.ladder.retain(|r| r.pct <= 20);
        env.coast.top_pct = 20;
        env.chains.retain(|c| !c.chain.starts_with("40@60"));
        env.refused_chains.push(ChainRefused {
            block: "reversal".into(),
            chain: "40@60,then:-40@60,then:40@60,brake:200".into(),
            predicted: 1689,
            why: "its excursion does not fit the room to the soft limit".into(),
        });
        let plans = expand(&default(), &env, 1).unwrap();
        let [slow, fast] = plans.as_slice() else {
            panic!("{} recordings", plans.len());
        };
        assert_eq!(
            slow.dropped,
            [
                "grid: 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100% dropped, \
                 over 20%, the highest duty the pilot kept both ways: rev 25% did not hold its \
                 goal duty for 60 ms past the settle in 109 ms",
                "coast: 30, 40, 60, 80% dropped, over 20%, the highest duty the pilot's coast \
                 ladder kept"
            ]
        );
        assert_eq!(
            names(slow),
            ["grid", "coast", "step", "reversal", "breakaway", "ends"]
        );
        let counts: Vec<usize> = slow.blocks.iter().map(|b| b.count).collect();
        assert_eq!(counts, [4, 4, 10, 4, 18, 2]);
        let grid: Vec<String> = fast.schedule.iter().map(Step::to_string).collect();
        assert_eq!(grid, ["5@1500", "10@1500", "15@1064", "20@735"]);

        // a block with nothing left is left out, and the plan says so
        env.chains.retain(|c| c.block != "ends");
        for chain in ["15@1277", "20@897"] {
            env.refused_chains.push(ChainRefused {
                block: "ends".into(),
                chain: chain.into(),
                predicted: 3300,
                why: "it would stop in about 240 counts past the soft limit".into(),
            });
        }
        let slow = &expand(&default(), &env, 1).unwrap()[0];
        assert!(!names(slow).contains(&"ends"));
        assert_eq!(
            slow.dropped.last().unwrap(),
            "ends: nothing the pilot kept is left, so it is left out"
        );
    }

    /// The fast recording's grid is sized by the pilot's pass under fast
    /// decay: without one it is refused, with one its rungs stop at that
    /// pass's top, at that pass's windows.
    #[test]
    fn fast_recording_needs_its_own_pilot_pass() {
        let p = default();
        let mut env = mg90();
        env.fast = None;
        assert_eq!(
            err(&p, &env),
            "recording fast: it drives under fast decay and the envelope holds no pilot pass \
             under it: run `osc capture pilot` again"
        );

        let mut env = mg90();
        let fast = env.fast.as_mut().unwrap();
        fast.rungs.retain(|r| r.pct <= 45);
        fast.top_pct = 45;
        for r in &mut fast.rungs {
            r.window_ms -= 10;
        }
        let plans = expand(&p, &env, 1).unwrap();
        let grid: Vec<String> = plans[1].schedule.iter().map(Step::to_string).collect();
        assert_eq!(grid.len(), 9);
        assert_eq!(grid.last().unwrap(), "45@319");
        let slow = &plans[0];
        assert_eq!(
            slow.schedule[slow.blocks[0].count - 1].to_string(),
            "55@286"
        );
    }

    /// session.py's contract: views are the block names plus `step:W`, which
    /// finds `30@W` followed by a coast exactly once in the slow schedule;
    /// `fast` reads the fast recording whole.
    #[test]
    fn plans_keep_the_session_py_contract() {
        let (p, env) = (default(), mg90());
        for n in 1..=6 {
            let plans = expand(&p, &env, n).unwrap();
            let [slow, fast] = plans.as_slice() else {
                panic!("{} recordings", plans.len());
            };
            assert_eq!((slow.recording.as_str(), slow.decay), ("slow", Decay::Slow));
            let mut views = names(slow);
            views.sort_unstable();
            assert_eq!(
                views,
                ["breakaway", "coast", "ends", "grid", "reversal", "step"]
            );
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
            let grid: Vec<Step> = env
                .fast
                .as_ref()
                .unwrap()
                .rungs
                .iter()
                .map(|r| Step::Drive(r.pct, Some(r.window_ms)))
                .collect();
            assert_eq!(fast.schedule, grid);
        }
    }
}
