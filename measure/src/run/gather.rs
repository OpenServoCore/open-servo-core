//! `gather`: a run folder's step results to an inputs folder the rules
//! read. Touches no servo.

use std::path::Path;

use anyhow::{Context, Result, bail};

use super::files;
use super::store::{At, Store};
use crate::board;
use crate::gather::{
    self, CRUISE_RESULT, RunResults, SPINS_RESULT, StepResult, TABLE_RESULT, WINDOW_RESULT,
};
use crate::sink::Sink;
use crate::steps::{probe, rest, stops};

fn result(fs: &mut dyn Store, run: &Path, rel: &str) -> Result<Option<StepResult>> {
    let p = run.join(rel);
    if !fs.exists(&p) {
        return Ok(None);
    }
    let bytes = files::read(fs, &p)?;
    Ok(Some(
        StepResult::parse(&p.display().to_string(), &bytes).map_err(|e| anyhow::anyhow!("{e}"))?,
    ))
}

/// The inputs folder `out` from the run folder `run`: `measured.json`, the
/// table `pos_lut.json` (the table step's, else straight), the table
/// step's slots `wide-codes.json` where it ran, the profile's
/// cross-checks `cross-checks.json`, the profile (as `nameplate.json`,
/// with the `thermal.json` a version 0 nameplate took), the policy,
/// `resolved.json`, the policy's values as this run resolved them, and
/// step 0's `board.json`, which is there already when `out` is the run's
/// own `inputs/`. The lines it prints.
pub fn gather(
    fs: &mut dyn Store,
    run: &Path,
    nameplate: &Path,
    policy: &Path,
    out: &Path,
) -> Result<Vec<String>> {
    let own = run.join("inputs");
    let (_, board, board_bytes) = files::board_in(fs, &own)?;
    let loaded = files::inputs(fs, nameplate, Some(policy), Some(&board.facts()))?;
    let np = &loaded.resolved.nameplate;
    let np_bytes = files::read(fs, nameplate)?;
    let policy_bytes = files::read(fs, policy)?;
    let mut resolved = serde_json::to_value(&loaded.resolved.report)?;
    resolved["notes"] = loaded.notes.clone().into();
    let mut resolved = serde_json::to_vec_pretty(&resolved)?;
    resolved.push(b'\n');
    let results = RunResults {
        rest: result(fs, run, rest::RESULT)?,
        probe: result(fs, run, probe::RESULT)?,
        stops: result(fs, run, stops::RESULT)?,
        window: result(fs, run, WINDOW_RESULT)?,
        spins: result(fs, run, SPINS_RESULT)?,
        cruise: result(fs, run, CRUISE_RESULT)?,
        table: result(fs, run, TABLE_RESULT)?,
    };
    let g = gather::gather(&results, np).map_err(|e| anyhow::anyhow!("{e}"))?;
    let into_own = fs.exists(out) && same(fs, out, &own)?;
    if fs.exists(out) && !(into_own && only_board(fs, out)?) {
        bail!(
            "{} exists: an inputs folder is gathered into a new one",
            out.display()
        );
    }
    let mut sink = At {
        files: fs,
        root: out.to_path_buf(),
    };
    if !into_own {
        sink.write(board::FILE, &board_bytes)?;
    }
    sink.write("measured.json", gather::render(&g.measured).as_bytes())?;
    sink.write("pos_lut.json", gather::render(&g.table).as_bytes())?;
    sink.write(
        "cross-checks.json",
        gather::render(&g.cross_checks).as_bytes(),
    )?;
    sink.write("nameplate.json", &np_bytes)?;
    sink.write("policy.json", &policy_bytes)?;
    sink.write("resolved.json", &resolved)?;
    let mut wrote = vec![
        "measured.json",
        "pos_lut.json",
        "cross-checks.json",
        "nameplate.json",
        "policy.json",
        "resolved.json",
    ];
    if !into_own {
        wrote.push(board::FILE);
    }
    if let Some(w) = &g.wide_codes {
        sink.write("wide-codes.json", gather::render(w).as_bytes())?;
        wrote.push("wide-codes.json");
    }
    if let Some(b) = &loaded.thermal_json {
        sink.write("thermal.json", b)?;
        wrote.push("thermal.json");
    }
    let mut lines = loaded.notes;
    lines.extend(g.lines);
    lines.push(format!("wrote {}: {}", out.display(), wrote.join(", ")));
    Ok(lines)
}

fn same(fs: &dyn Store, a: &Path, b: &Path) -> Result<bool> {
    let canon = |p: &Path| {
        fs.canonical(p)
            .map_err(anyhow::Error::msg)
            .with_context(|| format!("{}", p.display()))
    };
    Ok(fs.exists(b) && canon(a)? == canon(b)?)
}

/// The run's own `inputs/` as step 0 left it: its board file and nothing
/// a gather writes.
fn only_board(fs: &dyn Store, dir: &Path) -> Result<bool> {
    for (name, _) in fs
        .entries(dir)
        .map_err(anyhow::Error::msg)
        .with_context(|| format!("{}", dir.display()))?
    {
        if name != board::FILE {
            return Ok(false);
        }
    }
    Ok(true)
}
