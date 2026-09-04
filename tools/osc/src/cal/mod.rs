//! `osc cal` -- the calibration orchestrator: find the mechanical end-stops,
//! confirm the real-world angle range with the operator, print the
//! count->unit report, then write the limits + drive polarity + angle
//! endpoints and persist with MGMT SAVE. Interactive by default; flags make
//! it headless. Gear ratio here is an operator input only - measuring it (a
//! TEL ripple sweep) is a later commit. The endstop state machine and the
//! kinematics/units math live in osc-ident; this wrapper owns USB, prompts,
//! and files.

use std::path::Path;
use std::sync::atomic::Ordering;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use dialoguer::{Confirm, Input};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::endstop::{Endstop, EndstopCfg, EndstopResult};
use osc_ident::exp::{Guarded, RigParams};
use osc_ident::kinematics::{self, KinematicsResult, angle_endpoints};
use osc_ident::regs::{calib, config, control};
use osc_ident::units::{self, SenseParams};

use crate::rig::csvio::{OutDir, SnapshotLog};
use crate::rig::pump::{self, Pump, read_snapshot, write_reg};
use crate::rig::snapshot::{self, read_u16};

/// Assumed total rated travel when the operator gives no phys angle: a 0..180
/// deg span. Only a default; every real servo overrides it.
const DEFAULT_TRAVEL_DEG: f64 = 180.0;

/// `osc cal` args: output dir plus the headless overrides. `--baud`/`--id`
/// come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir for the rollback snapshot + endstop log (default ./cal-out).
    #[arg(long)]
    out: Option<std::path::PathBuf>,
    /// Real angle (deg) at the min-count rail.
    #[arg(long)]
    phys_angle_min: Option<f64>,
    /// Real angle (deg) at the max-count rail.
    #[arg(long)]
    phys_angle_max: Option<f64>,
    /// App-facing working-range min (deg); defaults to the phys min.
    #[arg(long)]
    soft_angle_min: Option<f64>,
    /// App-facing working-range max (deg); defaults to the phys max.
    #[arg(long)]
    soft_angle_max: Option<f64>,
    /// Known gear ratio (motor rev per output rev); unset leaves it 0.
    #[arg(long)]
    gear_ratio: Option<f64>,
    /// Assume yes: never prompt. Required values must come from flags.
    #[arg(long)]
    yes: bool,
}

/// Entry from the top-level `osc cal` dispatch.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let mut c = crate::rig::connect(&baud)?;
    let id = Id::new(id);

    let sense = read_sense(&mut c, id)?;
    let ke_vpc_q = read_u16(&mut c, id, calib::KE_VPC_Q)?;
    if ke_vpc_q == 0 {
        eprintln!("warning: ke_vpc_q is 0 - run `osc ident` first for the torque report");
    }

    // Safety gate before any motion: both mechanical ends get driven.
    if !args.yes {
        println!("cal drives the servo into BOTH mechanical end-stops.");
        if !confirm("proceed", false)? {
            println!("declined, no writes");
            return Ok(());
        }
    }

    let out = OutDir::create(args.out.as_deref().unwrap_or(Path::new("./cal-out")))?;
    let r = run_endstop(&mut c, id, &out)?;
    let EndstopResult {
        pos_min_phys,
        pos_max_phys,
        drive_polarity,
        i_stall_counts,
    } = r;
    let span = pos_max_phys - pos_min_phys;
    println!(
        "rails: min {pos_min_phys} max {pos_max_phys} span {span} counts, polarity {}, stall {i_stall_counts} counts",
        if drive_polarity { "normal" } else { "reversed" },
    );

    recenter(&mut c, id, pos_min_phys, pos_max_phys, drive_polarity)?;

    let (phys_min, phys_max) = phys_angles(args)?;
    let (soft_min_deg, soft_max_deg) = soft_angles(args, phys_min, phys_max)?;
    let gear_ratio_centi = gear_ratio(args)?;

    // soft angle -> phys count via the linear phys map, clamped to the rails
    let soft_min_count =
        soft_to_count(soft_min_deg, phys_min, phys_max, pos_min_phys, pos_max_phys);
    let soft_max_count =
        soft_to_count(soft_max_deg, phys_min, phys_max, pos_min_phys, pos_max_phys);

    let (angle_min_cdeg, angle_max_cdeg) = angle_endpoints(phys_min, phys_max);
    let kin = KinematicsResult {
        angle_min_cdeg,
        angle_max_cdeg,
        gear_ratio_centi,
    };
    let factors = units::derive(&sense, &kin, pos_min_phys, pos_max_phys, ke_vpc_q);
    print!("{}", units::render(&factors));

    let dpc = kinematics::deg_per_count(angle_min_cdeg, angle_max_cdeg, pos_min_phys, pos_max_phys);
    println!("deg/count {dpc:.6}");
    println!(
        "phys angle {phys_min:.2}..{phys_max:.2} deg, soft angle {soft_min_deg:.2}..{soft_max_deg:.2} deg"
    );
    println!("soft counts {soft_min_count}..{soft_max_count}");

    if !args.yes && !confirm("write these values and MGMT SAVE", false)? {
        println!("declined before write");
        return Ok(());
    }

    // snapshot the calib + gain block before any write (rollback safety)
    snapshot::take_snapshot(&mut c, id, &out.0.join("snapshot.json"))?;

    write_reg(&mut c, id, config::POS_MIN_PHYS_COUNTS, pos_min_phys)?;
    write_reg(&mut c, id, config::POS_MAX_PHYS_COUNTS, pos_max_phys)?;
    write_reg(&mut c, id, config::POS_MIN_SOFT_COUNTS, soft_min_count)?;
    write_reg(&mut c, id, config::POS_MAX_SOFT_COUNTS, soft_max_count)?;
    write_reg(&mut c, id, config::DRIVE_POLARITY, drive_polarity as i32)?;
    write_reg(&mut c, id, calib::ANGLE_MIN_CDEG, angle_min_cdeg as i32)?;
    write_reg(&mut c, id, calib::ANGLE_MAX_CDEG, angle_max_cdeg as i32)?;
    write_reg(&mut c, id, calib::GEAR_RATIO_CENTI, gear_ratio_centi as i32)?;

    // SAVE needs torque off (protocol sec 9.4); park was already torque-off.
    write_reg(&mut c, id, control::TORQUE_ENABLE, 0)?;
    c.save(id).context("MGMT SAVE")?;
    println!("saved");
    Ok(())
}

fn read_sense(c: &mut Client<NusbPipe>, id: Id) -> Result<SenseParams> {
    Ok(SenseParams {
        shunt_r_mohm: read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz: read_u16(c, id, calib::TICK_HZ)?,
    })
}

fn run_endstop(c: &mut Client<NusbPipe>, id: Id, out: &OutDir) -> Result<EndstopResult> {
    println!("[endstop] seeking both rails (pos guard off)");
    // pos guard off: driving into the physical ends IS the method
    let params = RigParams::default().without_pos_guard();
    let mut log = SnapshotLog::create(out, "endstop_snapshots.csv")?;
    let mut exp = Guarded::new(Endstop::new(EndstopCfg::default(), &params), params);
    let ran = Pump {
        client: c,
        id,
        tel_port: None,
        tel_mask: 0,
        log: Some(&mut log),
    }
    .run(&mut exp, |_| {});
    // park safe whether the run finished, errored, or was ctrl-c'd
    let _ = write_reg(c, id, control::GOAL_DUTY, 0);
    let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    ran?;
    if let Some(reason) = exp.abort() {
        bail!("endstop aborted by the safety envelope: {reason:?}");
    }
    exp.into_inner()
        .result()
        .context("endstop did not reach both rails - no writes")
}

/// Drive to the rail midpoint so the servo does not rest on a hard stop.
/// Best-effort: uses the measured polarity to pick the duty sign toward the
/// midpoint, parks torque-off on arrival, and gives up quietly after the loop.
fn recenter(
    c: &mut Client<NusbPipe>,
    id: Id,
    pos_min: i32,
    pos_max: i32,
    drive_polarity: bool,
) -> Result<()> {
    const DUTY: i32 = 9000;
    let mid = (pos_min + pos_max) / 2;
    let margin = ((pos_max - pos_min) / 10).max(1);
    let lo = (mid - margin).clamp(0, u16::MAX as i32) as u16;
    let hi = (mid + margin).clamp(0, u16::MAX as i32) as u16;
    let park = |c: &mut Client<NusbPipe>| {
        let _ = write_reg(c, id, control::GOAL_DUTY, 0);
        let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    };
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    for _ in 0..200 {
        if pump::STOP.load(Ordering::SeqCst) {
            park(c);
            bail!("interrupted");
        }
        let pos = read_snapshot(c, id)?.pos;
        if (lo..=hi).contains(&pos) {
            park(c);
            return Ok(());
        }
        // duty sign toward the midpoint: polarity maps count direction to sign
        let toward_higher = (pos as i32) < mid;
        let duty = if toward_higher == drive_polarity {
            DUTY
        } else {
            -DUTY
        };
        write_reg(c, id, control::GOAL_DUTY, duty)?;
        std::thread::sleep(Duration::from_millis(25));
    }
    park(c);
    println!("[recenter] did not reach mid-travel (gear slip?), left parked");
    Ok(())
}

/// Phys angle at each rail: from flags under `--yes` (both required), else
/// prompted with the flags or 0..DEFAULT_TRAVEL_DEG as defaults.
fn phys_angles(args: &Args) -> Result<(f64, f64)> {
    if args.yes {
        let min = args
            .phys_angle_min
            .context("--yes needs --phys-angle-min")?;
        let max = args
            .phys_angle_max
            .context("--yes needs --phys-angle-max")?;
        return Ok((min, max));
    }
    let min = input_f64(
        "phys angle at min rail (deg)",
        args.phys_angle_min.unwrap_or(0.0),
    )?;
    let max = input_f64(
        "phys angle at max rail (deg)",
        args.phys_angle_max.unwrap_or(DEFAULT_TRAVEL_DEG),
    )?;
    Ok((min, max))
}

/// Soft (working-range) angle: flags under `--yes` (defaulting to phys), else
/// prompted with the flags or the phys angles as defaults.
fn soft_angles(args: &Args, phys_min: f64, phys_max: f64) -> Result<(f64, f64)> {
    if args.yes {
        return Ok((
            args.soft_angle_min.unwrap_or(phys_min),
            args.soft_angle_max.unwrap_or(phys_max),
        ));
    }
    let min = input_f64(
        "soft angle min (deg)",
        args.soft_angle_min.unwrap_or(phys_min),
    )?;
    let max = input_f64(
        "soft angle max (deg)",
        args.soft_angle_max.unwrap_or(phys_max),
    )?;
    Ok((min, max))
}

/// Gear ratio as centi (x100); 0 is the firmware "unset" sentinel.
fn gear_ratio(args: &Args) -> Result<u16> {
    let ratio = if args.yes {
        args.gear_ratio
    } else if let Some(g) = args.gear_ratio {
        Some(input_f64("gear ratio (motor rev per output rev)", g)?)
    } else if confirm("enter a known gear ratio", false)? {
        Some(input_f64("gear ratio (motor rev per output rev)", 1.0)?)
    } else {
        None
    };
    Ok(ratio_to_centi(ratio))
}

fn ratio_to_centi(ratio: Option<f64>) -> u16 {
    match ratio {
        Some(r) if r.is_finite() && r > 0.0 => {
            (r * 100.0).round().clamp(0.0, u16::MAX as f64) as u16
        }
        _ => 0,
    }
}

fn soft_to_count(angle: f64, phys_min: f64, phys_max: f64, pos_min: i32, pos_max: i32) -> i32 {
    let denom = phys_max - phys_min;
    let frac = if denom == 0.0 {
        0.0
    } else {
        (angle - phys_min) / denom
    };
    let count = pos_min as f64 + frac * (pos_max - pos_min) as f64;
    let (lo, hi) = (pos_min.min(pos_max), pos_min.max(pos_max));
    (count.round() as i32).clamp(lo, hi)
}

fn confirm(prompt: &str, default: bool) -> Result<bool> {
    Ok(Confirm::new()
        .with_prompt(prompt)
        .default(default)
        .interact()?)
}

fn input_f64(prompt: &str, default: f64) -> Result<f64> {
    Ok(Input::<f64>::new()
        .with_prompt(prompt)
        .default(default)
        .interact_text()?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soft_maps_and_clamps_within_rails() {
        // 0..180 deg over counts 100..4000
        assert_eq!(soft_to_count(0.0, 0.0, 180.0, 100, 4000), 100);
        assert_eq!(soft_to_count(180.0, 0.0, 180.0, 100, 4000), 4000);
        assert_eq!(soft_to_count(90.0, 0.0, 180.0, 100, 4000), 2050);
        // tighter-than-phys stays in band; out-of-range clamps to a rail
        assert_eq!(soft_to_count(-10.0, 0.0, 180.0, 100, 4000), 100);
        assert_eq!(soft_to_count(200.0, 0.0, 180.0, 100, 4000), 4000);
    }

    #[test]
    fn soft_degenerate_phys_span_is_min_rail() {
        assert_eq!(soft_to_count(50.0, 90.0, 90.0, 100, 4000), 100);
    }

    #[test]
    fn ratio_centi_encoding_and_sentinel() {
        assert_eq!(ratio_to_centi(Some(150.0)), 15000);
        assert_eq!(ratio_to_centi(Some(1.5)), 150);
        assert_eq!(ratio_to_centi(None), 0);
        assert_eq!(ratio_to_centi(Some(0.0)), 0);
        assert_eq!(ratio_to_centi(Some(-3.0)), 0);
        assert_eq!(ratio_to_centi(Some(f64::NAN)), 0);
        assert_eq!(ratio_to_centi(Some(1e9)), u16::MAX);
    }
}
