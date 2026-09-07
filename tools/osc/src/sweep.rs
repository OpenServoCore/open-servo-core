//! `osc sweep` -- raw open-loop duty sweep for empirical plant capture:
//! per direction x per duty rung, seek to a start band by polling, then one
//! goal+arm COMMIT captures a TEL burst at full tick rate - no mid-rung
//! polling (the firmware soft-limit duty clamp, current limit, and faults
//! guard the silent window). A torque-off baseline burst runs first. Rows
//! land in sweep.csv, the run's constants in meta.json, both directly in
//! --out (no timestamp subdir).
//!
//! Duty sign is taken as-is (fwd = +duty): the seek's bang-bang polling
//! assumes normal drive polarity, and the guard envelope bails the run if a
//! reversed servo walks away from the band.

use std::io::Write;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::frame::TelFrame;
use osc_ident::regs::{calib, control};

use crate::rig::pump::{self, STOP, exchange_tel_burst, read_snapshot, with_guard, write_reg};
use crate::rig::snapshot::read_u16;

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Dirs {
    Both,
    Fwd,
    Rev,
}

/// `osc sweep` args. `--baud`/`--id` come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir; sweep.csv + meta.json land here directly.
    #[arg(long, default_value = "./sweep-out")]
    out: PathBuf,
    /// Duty grid, percent of full scale.
    #[arg(long, value_delimiter = ',', default_values_t = (1..=20u8).map(|k| k * 5))]
    duty_pct: Vec<u8>,
    #[arg(long, value_enum, default_value_t = Dirs::Both)]
    dirs: Dirs,
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
    /// TEL field mask (hex ok). Default = the full mask; batching killed
    /// the old per-tick wire budget.
    #[arg(long, default_value = "0x3f")]
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
/// Bails on a fault or on leaving the guard envelope; leaves duty 0 and
/// torque ON (the rung drives next).
fn seek_band(
    c: &mut Client<NusbPipe>,
    id: Id,
    (lo, hi): (u16, u16),
    duty_q15: i16,
    guard: (u16, u16),
) -> Result<()> {
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    for _ in 0..500 {
        check_stop()?;
        let pos = check_fault(c, id)?;
        if !(guard.0..=guard.1).contains(&pos) {
            write_reg(c, id, control::GOAL_DUTY, 0)?;
            bail!("seek left the guard envelope at pos {pos} (reversed polarity?)");
        }
        if (lo..=hi).contains(&pos) {
            write_reg(c, id, control::GOAL_DUTY, 0)?;
            return Ok(());
        }
        let duty = if pos < lo { duty_q15 } else { -duty_q15 };
        write_reg(c, id, control::GOAL_DUTY, duty as i32)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    write_reg(c, id, control::GOAL_DUTY, 0)?;
    bail!("seek did not reach {lo}..{hi} in 10 s (gear slipping?)")
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
            "{seg},{cmd_duty_q15},{dir},{},{},{},{},{},{},{},{}",
            f.tick,
            f.window_valid as u8,
            opt(f.pos.map(|v| v as i32)),
            opt(f.current.map(|v| v as i32)),
            opt(f.current_trough.map(|v| v as i32)),
            opt(f.duty_q15.map(|v| v as i32)),
            opt(f.vdiff.map(|v| v as i32)),
            opt(f.vbus.map(|v| v as i32)),
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
        "duty_pct": args.duty_pct,
        "dirs": dirs,
        "window_ms": args.window_ms,
        "rest_ms": args.rest_ms,
        "baseline_ms": args.baseline_ms,
        "seek_duty_pct": args.seek_duty_pct,
        "guard": [args.guard_lo, args.guard_hi],
        "tel_mask": mask,
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
        "seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,duty_q15,vdiff,vbus"
    )?;

    let seek_duty = pct_q15(args.seek_duty_pct);
    let guard = (args.guard_lo, args.guard_hi);
    let r = with_guard(&mut c, id, |c| {
        write_reg(c, id, control::TEL_MASK, mask as i32)?;

        // baseline: mid-travel, torque off, noise floor at full tick rate
        println!("[baseline] {} ms torque-off", args.baseline_ms);
        seek_band(c, id, (1750, 2350), seek_duty, guard)?;
        write_reg(c, id, control::TORQUE_ENABLE, 0)?;
        let (frames, st) = exchange_tel_burst(c, id, samples_of_ms(args.baseline_ms), None, mask)?;
        println!(
            "  seg 0: {} frames, {} samples, {} seq holes, {} garble bytes",
            st.frames, st.samples, st.holes, st.garble
        );
        write_rows(&mut w, 0, 0, 0, &frames)?;

        let mut seg = 1u32;
        for &dir in dirs {
            for &pct in &args.duty_pct {
                check_stop()?;
                check_fault(c, id)?;
                let duty = dir as i32 * pct_q15(pct) as i32;
                seek_band(
                    c,
                    id,
                    start_band(dir, args.guard_lo, args.guard_hi),
                    seek_duty,
                    guard,
                )?;
                write_reg(c, id, control::MODE, 0)?;
                write_reg(c, id, control::TORQUE_ENABLE, 1)?;
                let (frames, st) = exchange_tel_burst(
                    c,
                    id,
                    samples_of_ms(args.window_ms),
                    Some((control::GOAL_DUTY, duty)),
                    mask,
                )?;
                println!(
                    "  seg {seg} (duty {:+}%): {} frames, {} samples, {} seq holes, {} garble bytes",
                    dir as i32 * pct as i32,
                    st.frames,
                    st.samples,
                    st.holes,
                    st.garble
                );
                write_rows(&mut w, seg, duty as i16, dir, &frames)?;
                write_reg(c, id, control::GOAL_DUTY, 0)?;
                write_reg(c, id, control::TORQUE_ENABLE, 0)?;
                rest(args.rest_ms)?;
                seg += 1;
            }
        }
        Ok(())
    });
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
    fn start_bands_hug_the_guard_edges() {
        assert_eq!(start_band(1, 150, 3950), (400, 700));
        assert_eq!(start_band(-1, 150, 3950), (3400, 3700));
    }
}
