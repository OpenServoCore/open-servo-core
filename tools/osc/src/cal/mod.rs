//! `osc cal` -- the calibration orchestrator: find the mechanical end-stops,
//! confirm the real-world angle range with the operator, print the
//! count->unit report, then write the limits + drive polarity + angle
//! endpoints + gear + pot LUT and persist with MGMT SAVE. Interactive by
//! default; flags make it headless. With `--tel-port` the rail-to-rail seek
//! streams a TEL current+pos sweep: its commutation ripple gives a MEASURED
//! gear ratio (the gear prompt's default) and fills the pot linearization
//! LUT. Both are gear-2-dependent and degrade gracefully (identity LUT /
//! operator-input gear) when ripple SNR is low. The endstop state machine and
//! the kinematics/units/lut math live in osc-ident; this wrapper owns USB,
//! the TEL port, prompts, and files.

use std::path::Path;
use std::sync::atomic::Ordering;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use dialoguer::{Confirm, Input};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::endstop::{Endstop, EndstopCfg, EndstopResult};
use osc_ident::exp::{Cmd, Experiment, Guarded, RigParams};
use osc_ident::frame::{TelFrame, TelemetrySnapshot};
use osc_ident::kinematics::{self, KinematicsResult, angle_endpoints};
use osc_ident::lut::{self, PotLut};
use osc_ident::regs::{calib, config, control};
use osc_ident::ripple;
use osc_ident::units::{self, SenseParams};

use crate::rig::csvio::{OutDir, SnapshotLog};
use crate::rig::pump::{self, Pump, read_snapshot, write_reg};
use crate::rig::snapshot::{self, read_u16};

/// Commutation events per rotor rev for the brushed 3-slot motor: the ripple
/// tach and the LUT angle clock both count 6 per revolution.
const RIPPLE_PER_REV: f64 = 6.0;

/// Autocorr strength below this flags the ripple gear estimate as
/// low-confidence in the printout (still used - expected on a stripped gear).
const RIPPLE_CONF_MIN: f64 = 0.3;

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
    /// TEL stream serial device (the LinkE CDC); empty disables the ripple
    /// sweep (no measured gear, LUT left untouched).
    #[arg(long, default_value = "")]
    tel_port: String,
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
    // TEL frames arrive one per fast tick, so tick_hz is the sweep sample rate.
    let fs = sense.tick_hz as f64;
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

    let tel_port = (!args.tel_port.is_empty()).then(|| args.tel_port.clone());
    let out = OutDir::create(args.out.as_deref().unwrap_or(Path::new("./cal-out")))?;
    let (r, tel) = run_endstop(&mut c, id, &out, tel_port)?;
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

    // The clean full-travel traverse = the longest monotonic-pos run of the
    // captured frames; the pure lut/ripple fns then take it from here.
    let sweep = build_sweep(&tel);
    if let Some((pos, _)) = &sweep {
        println!(
            "[sweep] {} tel frames, monotonic run {} (fs {fs:.0} Hz from tick_hz)",
            tel.len(),
            pos.len()
        );
    }

    recenter(&mut c, id, pos_min_phys, pos_max_phys, drive_polarity)?;

    let (phys_min, phys_max) = phys_angles(args)?;
    let (soft_min_deg, soft_max_deg) = soft_angles(args, phys_min, phys_max)?;
    let (angle_min_cdeg, angle_max_cdeg) = angle_endpoints(phys_min, phys_max);
    let dpc = kinematics::deg_per_count(angle_min_cdeg, angle_max_cdeg, pos_min_phys, pos_max_phys);

    // measure-first: the ripple-derived gear ratio is the prompt's default,
    // the operator overrides. None (no sweep / low SNR / 0) keeps commit 6's
    // plain-input behavior.
    let measured = sweep.as_ref().and_then(|s| measure_gear(s, fs, dpc));
    if let Some((centi, strength)) = measured {
        let conf = if strength < RIPPLE_CONF_MIN {
            " (low confidence)"
        } else {
            ""
        };
        println!(
            "measured gear ratio {:.2}, ripple-derived{conf}",
            centi as f64 / 100.0
        );
    }
    let gear_ratio_centi = gear_ratio(args, measured.map(|(c, _)| c))?;

    // pot LUT from the same sweep; identity when the ripple SNR is too low to
    // clock true angle. No sweep at all -> None, LUT left untouched below.
    let lut = sweep.as_ref().map(|(pos, current)| {
        let phase = lut::cumulative_phase(current, fs, RIPPLE_PER_REV);
        let populated = phase.is_some();
        let l = match phase {
            Some(p) => lut::build(pos, &p, pos_min_phys as u16, pos_max_phys as u16),
            None => PotLut::identity(pos_min_phys as u16, pos_max_phys as u16),
        };
        (l, populated)
    });
    match &lut {
        Some((l, true)) => {
            let maxc = l.corr.iter().map(|&c| c.unsigned_abs()).max().unwrap_or(0);
            println!("pot LUT: populated, max |corr| {maxc} counts");
        }
        Some((_, false)) => println!("pot LUT: identity (ripple SNR too low)"),
        None => {}
    }

    // soft angle -> phys count via the linear phys map, clamped to the rails
    let soft_min_count =
        soft_to_count(soft_min_deg, phys_min, phys_max, pos_min_phys, pos_max_phys);
    let soft_max_count =
        soft_to_count(soft_max_deg, phys_min, phys_max, pos_min_phys, pos_max_phys);

    let kin = KinematicsResult {
        angle_min_cdeg,
        angle_max_cdeg,
        gear_ratio_centi,
    };
    let factors = units::derive(&sense, &kin, pos_min_phys, pos_max_phys, ke_vpc_q);
    print!("{}", units::render(&factors));

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

    if let Some((l, _)) = &lut {
        write_pot_lut(&mut c, id, l)?;
    }

    // SAVE needs torque off (protocol sec 9.4); park was already torque-off.
    write_reg(&mut c, id, control::TORQUE_ENABLE, 0)?;
    c.save(id).context("MGMT SAVE")?;
    println!("saved");
    Ok(())
}

/// Time-aligned (pos, current) sweep from the captured frames: the longest
/// monotonic-pos run is the clean constant-duty full-travel traverse. None
/// when no frames were captured or the run is too short to fit anything.
fn build_sweep(tel: &[TelFrame]) -> Option<(Vec<u16>, Vec<f64>)> {
    if tel.is_empty() {
        return None;
    }
    let samples: Vec<(u16, f64)> = tel
        .iter()
        .filter_map(|f| Some((f.pos?, f.current? as f64)))
        .collect();
    let (pos, current) = longest_monotonic_run(&samples);
    (pos.len() >= 2).then_some((pos, current))
}

/// Longest run of consecutive samples whose pos is monotonic (non-decreasing
/// OR non-increasing); equal steps keep both directions alive.
fn longest_monotonic_run(samples: &[(u16, f64)]) -> (Vec<u16>, Vec<f64>) {
    if samples.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let (mut best_lo, mut best_hi) = (0usize, 1usize);
    let (mut inc_start, mut dec_start) = (0usize, 0usize);
    for i in 1..samples.len() {
        let (prev, cur) = (samples[i - 1].0, samples[i].0);
        if cur < prev {
            inc_start = i;
        }
        if cur > prev {
            dec_start = i;
        }
        let inc_len = i + 1 - inc_start;
        let dec_len = i + 1 - dec_start;
        let (start, len) = if inc_len >= dec_len {
            (inc_start, inc_len)
        } else {
            (dec_start, dec_len)
        };
        if len > best_hi - best_lo {
            best_lo = start;
            best_hi = i + 1;
        }
    }
    let run = &samples[best_lo..best_hi];
    (
        run.iter().map(|s| s.0).collect(),
        run.iter().map(|s| s.1).collect(),
    )
}

/// Ripple-derived gear ratio (centi) + autocorr strength from a monotonic
/// sweep. None when the ripple SNR is too low to clock motor speed or the
/// result degenerates to the 0 sentinel.
fn measure_gear(sweep: &(Vec<u16>, Vec<f64>), fs: f64, dpc: f64) -> Option<(u16, f64)> {
    let (pos, current) = sweep;
    let n = pos.len();
    if n < 2 || fs <= 0.0 {
        return None;
    }
    let est = ripple::ripple_speed(current, fs, RIPPLE_PER_REV)?;
    let dt = (n - 1) as f64 / fs;
    // endpoint slope of pos vs time = output shaft speed in counts/s
    let pot_omega_cps = ((pos[n - 1] as f64 - pos[0] as f64) / dt).abs();
    let centi = kinematics::gear_ratio_centi(est.motor_rev_s, pot_omega_cps, dpc);
    (centi != 0).then_some((centi, est.strength))
}

/// Write the PotLutBlock: raw_min/raw_max as scalars, then the 55 corr knots
/// as one 110-byte blob (lut_corr is a single Bytes field, so a bulk write
/// beats 55 round-trips and matches the on-servo layout exactly).
fn write_pot_lut(c: &mut Client<NusbPipe>, id: Id, lut: &PotLut) -> Result<()> {
    write_reg(c, id, calib::POT_LUT_RAW_MIN, lut.raw_min as i32)?;
    write_reg(c, id, calib::POT_LUT_RAW_MAX, lut.raw_max as i32)?;
    let mut bytes = [0u8; 110];
    for (i, &k) in lut.corr.iter().enumerate() {
        bytes[2 * i..2 * i + 2].copy_from_slice(&k.to_le_bytes());
    }
    c.write(id, calib::POT_LUT_CORR.addr, &bytes)
        .context("write pot lut corr")?;
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

fn run_endstop(
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &OutDir,
    tel_port: Option<String>,
) -> Result<(EndstopResult, Vec<TelFrame>)> {
    println!("[endstop] seeking both rails (pos guard off)");
    // pos guard off: driving into the physical ends IS the method
    let params = RigParams::default().without_pos_guard();
    let mut log = SnapshotLog::create(out, "endstop_snapshots.csv")?;
    let mut exp = Guarded::new(Endstop::new(EndstopCfg::default(), &params), params);
    let mut tel: Vec<TelFrame> = Vec::new();
    // 0x1B = pos|current|duty|vdiff (same TEL mask as ident inertia)
    let tel_mask: u16 = 0x1B;
    let ran = if let Some(port) = tel_port {
        // Endstop itself never enables TEL; wrap it so the pump streams the
        // side-channel around the rail-to-rail seek (the second seek is a
        // constant-duty full-travel sweep - the LUT/gear source).
        let mut wrap = TelWrap {
            inner: &mut exp,
            phase: TelInject::MaskOn,
            mask: tel_mask,
        };
        Pump {
            client: c,
            id,
            tel_port: Some(port),
            tel_mask,
            log: Some(&mut log),
        }
        .run(&mut wrap, |frames| tel.extend_from_slice(frames))
    } else {
        Pump {
            client: c,
            id,
            tel_port: None,
            tel_mask: 0,
            log: Some(&mut log),
        }
        .run(&mut exp, |_| {})
    };
    // park safe whether the run finished, errored, or was ctrl-c'd
    let _ = write_reg(c, id, control::GOAL_DUTY, 0);
    let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    ran?;
    if let Some(reason) = exp.abort() {
        bail!("endstop aborted by the safety envelope: {reason:?}");
    }
    let r = exp
        .into_inner()
        .result()
        .context("endstop did not reach both rails - no writes")?;
    Ok((r, tel))
}

/// Injects the TEL enable/mask writes around an inner experiment so the pump
/// opens the side-channel for a run that would not enable TEL on its own
/// (Endstop). Prepends mask+enable, delegates the body, appends enable=0 +
/// mask=0 when the inner run reports Done.
enum TelInject {
    MaskOn,
    EnaOn,
    Body,
    EnaOff,
    MaskOff,
    Done,
}

struct TelWrap<'a, E> {
    inner: &'a mut E,
    phase: TelInject,
    mask: u16,
}

impl<E: Experiment> Experiment for TelWrap<'_, E> {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            TelInject::MaskOn => {
                self.phase = TelInject::EnaOn;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: self.mask as i32,
                }
            }
            TelInject::EnaOn => {
                self.phase = TelInject::Body;
                Cmd::Write {
                    reg: control::TEL_ENABLE,
                    value: 1,
                }
            }
            TelInject::Body => {
                let cmd = self.inner.step(obs);
                if matches!(cmd, Cmd::Done) {
                    self.phase = TelInject::EnaOff;
                    Cmd::Write {
                        reg: control::TEL_ENABLE,
                        value: 0,
                    }
                } else {
                    cmd
                }
            }
            TelInject::EnaOff => {
                self.phase = TelInject::MaskOff;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: 0,
                }
            }
            TelInject::MaskOff => {
                self.phase = TelInject::Done;
                Cmd::Done
            }
            TelInject::Done => Cmd::Done,
        }
    }
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

/// Gear ratio as centi (x100); 0 is the firmware "unset" sentinel. `measured`
/// (ripple-derived) is the measure-first default: under `--yes` it becomes the
/// value with no prompt; interactively it is the prompt's default the operator
/// can override. None keeps commit 6's plain-input behavior.
fn gear_ratio(args: &Args, measured: Option<u16>) -> Result<u16> {
    if args.yes {
        return Ok(measured.unwrap_or_else(|| ratio_to_centi(args.gear_ratio)));
    }
    let ratio = if let Some(m) = measured {
        Some(input_f64(
            "gear ratio (motor rev per output rev)",
            m as f64 / 100.0,
        )?)
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
    fn monotonic_run_picks_longest_clean_traverse() {
        // rising 5, a bump, then a longer falling run of 6 -> the fall wins
        let s: Vec<(u16, f64)> = [10u16, 20, 30, 40, 50, 45, 60, 55, 44, 33, 22, 11]
            .iter()
            .map(|&p| (p, p as f64))
            .collect();
        let (pos, cur) = longest_monotonic_run(&s);
        assert_eq!(pos, vec![60, 55, 44, 33, 22, 11]);
        assert_eq!(cur.len(), pos.len());
    }

    #[test]
    fn monotonic_run_allows_equal_steps() {
        let s: Vec<(u16, f64)> = [1u16, 1, 2, 2, 3, 3].iter().map(|&p| (p, 0.0)).collect();
        let (pos, _) = longest_monotonic_run(&s);
        assert_eq!(pos, vec![1, 1, 2, 2, 3, 3]);
    }

    #[test]
    fn monotonic_run_empty_and_single() {
        assert_eq!(longest_monotonic_run(&[]), (Vec::new(), Vec::new()));
        assert_eq!(longest_monotonic_run(&[(7, 1.0)]), (vec![7], vec![1.0]));
    }

    #[test]
    fn build_sweep_none_without_frames() {
        assert!(build_sweep(&[]).is_none());
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
