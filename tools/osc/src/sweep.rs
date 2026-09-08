//! `osc sweep` -- raw open-loop duty sweep for empirical plant capture:
//! per direction x per duty rung, seek to a start band by polling, then one
//! goal+arm COMMIT captures a TEL burst at full tick rate - no mid-rung
//! polling (the firmware soft-limit duty clamp, current limit, and faults
//! guard the silent window). A torque-off baseline burst runs first. Rows
//! land in sweep.csv, the run's constants in meta.json, both directly in
//! --out (no timestamp subdir).
//!
//! `coast:MS` / `brake:MS` schedule entries capture a zero-duty segment:
//! the preceding drive step feeds it without a settle (momentum carries
//! across the burst gap), and a brake segment runs with openloop_zero_brake
//! set (cleared again right after, so coast segments always see it clear).
//!
//! Duty sign is taken as-is (fwd = +duty): the seek's bang-bang polling
//! assumes normal drive polarity and bails the run if a reversed servo
//! walks away from the band.

use std::io::Write;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::frame::TelFrame;
use osc_ident::regs::{Reg, calib, control};

use crate::descriptor;
use crate::rig::pump::{self, STOP, exchange_tel_burst, read_snapshot, with_guard, write_reg};
use crate::rig::snapshot::read_u16;

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Dirs {
    Both,
    Fwd,
    Rev,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Decay {
    Slow,
    Fast,
}

impl Decay {
    fn as_str(self) -> &'static str {
        match self {
            Decay::Slow => "slow",
            Decay::Fast => "fast",
        }
    }
}

/// One schedule entry: a drive rung, or a zero-duty coast/brake segment.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Step {
    Drive(u8),
    Coast(u32),
    Brake(u32),
}

impl std::fmt::Display for Step {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Step::Drive(pct) => write!(f, "{pct}"),
            Step::Coast(ms) => write!(f, "coast:{ms}"),
            Step::Brake(ms) => write!(f, "brake:{ms}"),
        }
    }
}

fn parse_step(s: &str) -> Result<Step, String> {
    let ms = |v: &str| v.parse().map_err(|_| format!("bad ms {v:?}"));
    if let Some(v) = s.strip_prefix("coast:") {
        return Ok(Step::Coast(ms(v)?));
    }
    if let Some(v) = s.strip_prefix("brake:") {
        return Ok(Step::Brake(ms(v)?));
    }
    s.parse()
        .map(Step::Drive)
        .map_err(|_| format!("step is duty pct, coast:MS, or brake:MS, got {s:?}"))
}

/// `osc sweep` args. `--baud`/`--id` come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir; sweep.csv + meta.json land here directly.
    #[arg(long, default_value = "./sweep-out")]
    out: PathBuf,
    /// Duty grid, percent of full scale; `coast:MS` / `brake:MS` entries
    /// capture a zero-duty segment chained to the preceding step.
    #[arg(long, value_delimiter = ',', value_parser = parse_step,
          default_values_t = (1..=20u8).map(|k| Step::Drive(k * 5)))]
    duty_pct: Vec<Step>,
    #[arg(long, value_enum, default_value_t = Dirs::Both)]
    dirs: Dirs,
    /// openloop_decay written before the run.
    #[arg(long, value_enum, default_value_t = Decay::Slow)]
    decay: Decay,
    /// Capture per rung; samples = window_ms x 20 ticks.
    #[arg(long, default_value_t = 150)]
    window_ms: u32,
    /// Torque-off cool-down between rungs.
    #[arg(long, default_value_t = 500)]
    rest_ms: u32,
    /// Torque-off noise-floor capture before the grid.
    #[arg(long, default_value_t = 1000)]
    baseline_ms: u32,
    /// Seek drive, percent of full scale.
    #[arg(long, default_value_t = 28)]
    seek_duty_pct: u8,
    /// Seek envelope only - the captures themselves run unpolled.
    #[arg(long, default_value_t = 150)]
    guard_lo: u16,
    #[arg(long, default_value_t = 3950)]
    guard_hi: u16,
    /// TEL field mask (hex ok). Default = the raw six: pos, raw current,
    /// trough, applied duty, vmotor_a, vmotor_b - measurements only, kernel
    /// conclusions (vdiff/vbus/i_meas) stay out of raw captures.
    #[arg(long, default_value = "0x1cd")]
    tel_mask: String,
}

/// One rung's start band: fwd launches near the low guard edge, rev
/// mirrored, so the whole window fits before the far soft limit.
fn start_band(dir: i8, guard_lo: u16, guard_hi: u16) -> (u16, u16) {
    if dir > 0 {
        (guard_lo + 250, guard_lo + 550)
    } else {
        (guard_hi - 550, guard_hi - 250)
    }
}

fn pct_q15(pct: u8) -> i16 {
    (pct as i32 * 32767 / 100) as i16
}

fn samples_of_ms(ms: u32) -> u16 {
    ms.saturating_mul(20).min(u16::MAX as u32) as u16
}

fn check_stop() -> Result<()> {
    if STOP.load(std::sync::atomic::Ordering::SeqCst) {
        bail!("interrupted");
    }
    Ok(())
}

fn check_fault(c: &mut Client<NusbPipe>, id: Id) -> Result<u16> {
    let s = read_snapshot(c, id)?;
    if s.fault_flags != 0 {
        bail!(
            "servo faulted: flags {:#04x} code {}",
            s.fault_flags,
            s.fault_code
        );
    }
    Ok(s.pos)
}

/// Open-loop bang-bang seek into [lo, hi], 20 ms cadence, ctrl-c aware.
/// Bails on a fault or on the band distance growing (reversed polarity);
/// leaves duty 0 and torque ON (the rung drives next).
fn seek_band(c: &mut Client<NusbPipe>, id: Id, (lo, hi): (u16, u16), duty_q15: i16) -> Result<()> {
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    // Distance to the band, not envelope membership: a seek may legally
    // START at a rail (that is what it is for); reversed polarity shows as
    // the distance GROWING while driving.
    let dist = |pos: u16| (lo.saturating_sub(pos)) as u32 + (pos.saturating_sub(hi)) as u32;
    let mut best = u32::MAX;
    for _ in 0..500 {
        check_stop()?;
        let pos = check_fault(c, id)?;
        if (lo..=hi).contains(&pos) {
            write_reg(c, id, control::GOAL_DUTY, 0)?;
            return Ok(());
        }
        let d = dist(pos);
        best = best.min(d);
        if d > best + 200 {
            write_reg(c, id, control::GOAL_DUTY, 0)?;
            bail!("seek moving away from {lo}..{hi} at pos {pos} (reversed polarity?)");
        }
        let duty = if pos < lo { duty_q15 } else { -duty_q15 };
        write_reg(c, id, control::GOAL_DUTY, duty as i32)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    write_reg(c, id, control::GOAL_DUTY, 0)?;
    bail!("seek did not reach {lo}..{hi} in 10 s (gear slipping?)")
}

/// 3 PWM ticks at ARR 1200 (0.25% drive) - enough to dodge motor.rs's
/// ticks==0 coast mapping, small enough that the slow-decay brake dominates.
const BRAKE_DUTY_Q15: i32 = 82;

/// Dynamic brake after a rung: with openloop_decay Slow, a token duty holds
/// the idle H-bridge leg HIGH for the rest of each PWM period - the winding
/// shorts through the driver and momentum dies in a few hundred counts.
/// A plain duty-0 write is COAST (motor.rs maps zero ticks to both-low);
/// at battery volts the freewheel from a top rung crosses the remaining
/// runway and slams the physical stop. Retreat sign so the firmware
/// soft-limit clamp can never zero the brake near a wall. Leaves duty 0,
/// torque ON (the caller torques off).
fn brake_to_rest(c: &mut Client<NusbPipe>, id: Id, dir: i8) -> Result<()> {
    write_reg(c, id, control::GOAL_DUTY, -(dir as i32) * BRAKE_DUTY_Q15)?;
    let mut last = check_fault(c, id)?;
    for _ in 0..50 {
        check_stop()?;
        std::thread::sleep(Duration::from_millis(20));
        let pos = check_fault(c, id)?;
        if pos.abs_diff(last) < 4 {
            break;
        }
        last = pos;
    }
    write_reg(c, id, control::GOAL_DUTY, 0)?;
    Ok(())
}

fn rest(ms: u32) -> Result<()> {
    let mut left = ms;
    while left > 0 {
        check_stop()?;
        let slice = left.min(20);
        std::thread::sleep(Duration::from_millis(slice as u64));
        left -= slice;
    }
    Ok(())
}

fn write_rows(
    w: &mut impl Write,
    seg: u32,
    cmd_duty_q15: i16,
    dir: i8,
    frames: &[TelFrame],
) -> Result<()> {
    let opt = |v: Option<i32>| v.map(|v| v.to_string()).unwrap_or_default();
    for f in frames {
        writeln!(
            w,
            "{seg},{cmd_duty_q15},{dir},{},{},{},{},{},{},{},{},{},{},{}",
            f.tick,
            f.window_valid as u8,
            opt(f.pos.map(|v| v as i32)),
            opt(f.current.map(|v| v as i32)),
            opt(f.current_trough.map(|v| v as i32)),
            opt(f.duty_q15.map(|v| v as i32)),
            opt(f.vdiff.map(|v| v as i32)),
            opt(f.vbus.map(|v| v as i32)),
            opt(f.current_raw.map(|v| v as i32)),
            opt(f.vmotor_a.map(|v| v as i32)),
            opt(f.vmotor_b.map(|v| v as i32)),
        )?;
    }
    Ok(())
}

fn git_sha() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}

/// Entry from the top-level `osc sweep` dispatch.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let mask = crate::parse_u16(&args.tel_mask).context("--tel-mask")?;
    let mut c = crate::rig::connect(&baud)?;
    let id = Id::new(id);

    std::fs::create_dir_all(&args.out).with_context(|| format!("mkdir {}", args.out.display()))?;

    let identity = c.identity(id)?;
    let registry = descriptor::Registry::load()?;
    let (d, note) = registry.select(identity.model, identity.fw)?;
    if let Some(note) = note {
        println!("{note}");
    }
    let field_reg = |name: &str| -> Result<Reg> {
        let f = d.field(name)?;
        Ok(Reg {
            addr: f.addr,
            width: f.width as u8,
        })
    };
    let decay_reg = field_reg("openloop_decay")?;
    let zb_reg = field_reg("openloop_zero_brake")?;
    let tick_hz = read_u16(&mut c, id, calib::TICK_HZ)?;
    let vbus_counts = read_snapshot(&mut c, id)?.vbus_counts;
    let dirs: &[i8] = match args.dirs {
        Dirs::Both => &[1, -1],
        Dirs::Fwd => &[1],
        Dirs::Rev => &[-1],
    };
    let meta = serde_json::json!({
        "model": identity.model,
        "fw": identity.fw,
        "tick_hz": tick_hz,
        "vbus_counts": vbus_counts,
        "sense": {
            "shunt_r_mohm": read_u16(&mut c, id, calib::SHUNT_R_MOHM)?,
            "gain_milli": read_u16(&mut c, id, calib::GAIN_MILLI)?,
            "vmotor_div_top": read_u16(&mut c, id, calib::VMOTOR_DIV_TOP)?,
            "vmotor_div_bot": read_u16(&mut c, id, calib::VMOTOR_DIV_BOT)?,
            "vdd_mv": read_u16(&mut c, id, calib::VDD_MV)?,
        },
        "schedule": args.duty_pct.iter().map(Step::to_string).collect::<Vec<_>>(),
        "decay": args.decay.as_str(),
        "dirs": dirs,
        "window_ms": args.window_ms,
        "rest_ms": args.rest_ms,
        "baseline_ms": args.baseline_ms,
        "seek_duty_pct": args.seek_duty_pct,
        "guard": [args.guard_lo, args.guard_hi],
        "tel_mask": mask,
        "post_rung_brake": true,
        "git_sha": git_sha(),
    });
    let meta_path = args.out.join("meta.json");
    std::fs::write(&meta_path, serde_json::to_string_pretty(&meta)?)
        .with_context(|| format!("write {}", meta_path.display()))?;

    let csv_path = args.out.join("sweep.csv");
    let mut w = std::io::BufWriter::new(
        std::fs::File::create(&csv_path)
            .with_context(|| format!("create {}", csv_path.display()))?,
    );
    writeln!(
        w,
        "seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,duty_q15,vdiff,vbus,current_raw,vmotor_a,vmotor_b"
    )?;

    let seek_duty = pct_q15(args.seek_duty_pct);
    let r = with_guard(&mut c, id, |c| {
        write_reg(c, id, decay_reg, args.decay as i32)?;
        write_reg(c, id, zb_reg, 0)?;
        write_reg(c, id, control::TEL_MASK, mask as i32)?;

        // baseline: mid-travel, torque off, noise floor at full tick rate
        println!("[baseline] {} ms torque-off", args.baseline_ms);
        seek_band(c, id, (1750, 2350), seek_duty)?;
        write_reg(c, id, control::TORQUE_ENABLE, 0)?;
        let (frames, st) = exchange_tel_burst(c, id, samples_of_ms(args.baseline_ms), None, mask)?;
        println!(
            "  seg 0: {} frames, {} samples, {} seq holes, {} garble bytes",
            st.frames, st.samples, st.holes, st.garble
        );
        write_rows(&mut w, 0, 0, 0, &frames)?;

        let steps = &args.duty_pct;
        // A coast/brake segment chains to the step before it: no settle in
        // between, so the drive's momentum carries into the zero-duty burst.
        let feeds = |k: usize| matches!(steps.get(k + 1), Some(Step::Coast(_) | Step::Brake(_)));

        let mut seg = 1u32;
        for &dir in dirs {
            let mut live = false;
            for (k, &step) in steps.iter().enumerate() {
                check_stop()?;
                let (ms, duty) = match step {
                    Step::Drive(pct) => {
                        check_fault(c, id)?;
                        seek_band(
                            c,
                            id,
                            start_band(dir, args.guard_lo, args.guard_hi),
                            seek_duty,
                        )?;
                        write_reg(c, id, control::MODE, 0)?;
                        write_reg(c, id, control::TORQUE_ENABLE, 1)?;
                        (args.window_ms, dir as i32 * pct_q15(pct) as i32)
                    }
                    Step::Coast(ms) | Step::Brake(ms) => {
                        if !live {
                            check_fault(c, id)?;
                            write_reg(c, id, control::MODE, 0)?;
                            write_reg(c, id, control::TORQUE_ENABLE, 1)?;
                        }
                        (ms, 0)
                    }
                };
                if matches!(step, Step::Brake(_)) {
                    write_reg(c, id, zb_reg, 1)?;
                }
                let (frames, st) = exchange_tel_burst(
                    c,
                    id,
                    samples_of_ms(ms),
                    Some((control::GOAL_DUTY, duty)),
                    mask,
                )?;
                if matches!(step, Step::Brake(_)) {
                    write_reg(c, id, zb_reg, 0)?;
                }
                let what = match step {
                    Step::Drive(pct) => format!("duty {:+}%", dir as i32 * pct as i32),
                    Step::Coast(ms) => format!("coast {ms} ms"),
                    Step::Brake(ms) => format!("brake {ms} ms"),
                };
                println!(
                    "  seg {seg} ({what}): {} frames, {} samples, {} seq holes, {} garble bytes",
                    st.frames, st.samples, st.holes, st.garble
                );
                write_rows(&mut w, seg, duty as i16, dir, &frames)?;
                if feeds(k) {
                    live = true;
                } else {
                    brake_to_rest(c, id, dir)?;
                    write_reg(c, id, control::TORQUE_ENABLE, 0)?;
                    rest(args.rest_ms)?;
                    live = false;
                }
                seg += 1;
            }
        }
        Ok(())
    });
    // Belt for a run cut mid-brake-segment: the flag must end the sweep clear.
    let _ = write_reg(&mut c, id, zb_reg, 0);
    w.flush()?;
    r?;
    println!("sweep: {}", csv_path.display());
    println!("meta:  {}", meta_path.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duty_grid_maps_percent_to_q15() {
        assert_eq!(pct_q15(100), 32767);
        assert_eq!(pct_q15(50), 16383);
        assert_eq!(pct_q15(5), 1638);
        assert_eq!(pct_q15(28), 9174);
    }

    #[test]
    fn window_sizes_at_20_ticks_per_ms_and_caps_at_u16() {
        assert_eq!(samples_of_ms(150), 3000);
        assert_eq!(samples_of_ms(1000), 20000);
        assert_eq!(samples_of_ms(10_000), u16::MAX);
    }

    #[test]
    fn steps_parse_and_display_round_trip() {
        for (s, want) in [
            ("35", Step::Drive(35)),
            ("coast:200", Step::Coast(200)),
            ("brake:150", Step::Brake(150)),
        ] {
            assert_eq!(parse_step(s).unwrap(), want);
            assert_eq!(want.to_string(), s);
        }
        assert!(parse_step("coast").is_err());
        assert!(parse_step("brake:x").is_err());
        assert!(parse_step("slow").is_err());
    }

    #[test]
    fn start_bands_hug_the_guard_edges() {
        assert_eq!(start_band(1, 150, 3950), (400, 700));
        assert_eq!(start_band(-1, 150, 3950), (3400, 3700));
    }
}
