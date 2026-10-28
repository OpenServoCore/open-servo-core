//! `osc burst` - raw high-rate shunt captures from rest, the instrument
//! under no fit: per duty x sign x repeat, torque on at zero duty in
//! OpenLoop, one burst armed at the step, torque off, a rest. Each capture
//! lands as `burst-K.csv` the moment it is read back, and the run's
//! constants as meta.json, both directly in --out.
//!
//! The rotor is never moved to a start point: the load is a resistor or a
//! locked rotor, so the run holds the stall permit (the bus dead-man on the
//! experiment image) the way `osc sweep --static-load` does. The only checks
//! are the instrument's own: every step stays under the 3.2 V a burst may
//! apply on the rail read before the run, and captures sit at least the
//! firmware's arm spacing apart.

use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_ident::burst::{CHANS_MAX, Capture, Chans, to_csv};
use osc_ident::limits::{BurstAllowance, ServoLimits};
use osc_ident::regs::{calib, control};
use osc_ident::units::{SenseParams, volts_per_count};

use crate::rig::Bus;
use crate::rig::pump::{self, Lease, STOP, capture_burst};
use crate::rig::servo::{Servo, Wire, guard};
use crate::rig::snapshot::{read_bool, read_u16};
use crate::sweep::{Dirs, pct_q15, servo_meta};

/// The firmware refuses an arm sooner than this after the one before.
const ARM_SPACING_MS: u32 = 100;

/// Torque on at zero duty this long before each arm, so the pre-arm reads
/// see the bridge the step leaves from.
const SETTLE_MS: u32 = 200;

/// `osc burst` args. `--baud`/`--id` come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir; burst-K.csv + meta.json land here directly.
    #[arg(long, default_value = "./burst-out")]
    out: PathBuf,
    /// Step duties, percent of full scale; each must apply at most 3.2 V
    /// on the rail.
    #[arg(long, value_delimiter = ',', required = true)]
    duty_pct: Vec<u8>,
    #[arg(long, value_enum, default_value_t = Dirs::Both)]
    dirs: Dirs,
    /// Captures per step duty and sign.
    #[arg(long, default_value_t = 3)]
    repeats: u32,
    /// Voltage channels behind the shunt: `driven`, `diff`, or a mask 0..=15
    /// (bit 0 vmotor_a, bit 1 vmotor_b, bit 2 vbus_raw, bit 3 interleave).
    #[arg(long, default_value = "diff", value_parser = parse_chans)]
    chans: Chans,
    /// Torque-off rest between captures; at least the firmware's 100 ms arm
    /// spacing.
    #[arg(long, default_value_t = 150)]
    rest_ms: u32,
}

fn parse_chans(s: &str) -> Result<Chans, String> {
    Chans::parse(s).ok_or_else(|| {
        format!(
            "expected driven, diff or a mask 0..={}, got {s:?}",
            CHANS_MAX
        )
    })
}

/// One arm: the signed step, q15.
pub(crate) fn plan(duty_pct: &[u8], dirs: Dirs, repeats: u32) -> Vec<i16> {
    let mut out = Vec::new();
    for pct in duty_pct {
        for &dir in dirs.signs() {
            for _ in 0..repeats {
                out.push(dir as i16 * pct_q15(*pct));
            }
        }
    }
    out
}

/// Refuse a run the instrument cannot take: a step over what a burst may
/// apply on a rail of `rail_mv`, or captures closer than the arm spacing.
pub(crate) fn check(duty_pct: &[u8], rest_ms: u32, rail_mv: f64) -> Result<()> {
    if rail_mv <= 0.0 {
        bail!("the servo reads no rail, so no burst duty can be sized against the 3.2 V cap");
    }
    let most = BurstAllowance::top(rail_mv);
    if let Some(hi) = duty_pct.iter().copied().max()
        && hi as f64 / 100.0 > most
    {
        bail!(
            "a step of {hi}% is over the {:.1}% a burst may drive on this {:.2} V rail: 3.2 V \
             applied, less 1% in case the rail rises before the arm",
            most * 100.0,
            rail_mv / 1000.0
        );
    }
    if rest_ms < ARM_SPACING_MS {
        bail!("--rest-ms {rest_ms} is under the firmware's {ARM_SPACING_MS} ms arm spacing");
    }
    Ok(())
}

/// The rail before the run, millivolts, from the limits' vmotor-tap counts.
fn rail_mv<P: Pipe>(c: &mut Client<P>, id: Id, lim: &ServoLimits) -> Result<f64> {
    let sense = SenseParams {
        shunt_r_mohm: 0,
        gain_milli: 0,
        vmotor_div_top: read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz: 0,
    };
    Ok(lim.vbus as f64 * volts_per_count(&sense) * 1000.0)
}

fn check_stop() -> Result<()> {
    if STOP.load(std::sync::atomic::Ordering::SeqCst) {
        bail!("interrupted");
    }
    Ok(())
}

fn rest<S: Servo>(s: &mut S, ms: u32) -> Result<()> {
    let mut left = ms;
    while left > 0 {
        check_stop()?;
        let slice = left.min(20);
        s.sleep(slice);
        left -= slice;
    }
    Ok(())
}

/// Every arm of `steps` in order, each capture handed to `on_cap` as it is
/// read back. Leaves the servo guarded and torqued off.
fn record<S: Servo>(
    s: &mut S,
    steps: &[i16],
    chans: Chans,
    rest_ms: u32,
    mut on_cap: impl FnMut(usize, &Capture) -> Result<()>,
) -> Result<()> {
    let id = s.id();
    let polarity = read_bool(s.client(), id, osc_ident::regs::config::DRIVE_POLARITY)?;
    let mut lease = Lease::new(true, s.experiment()?);
    guard(s, |s| {
        for (k, &step) in steps.iter().enumerate() {
            check_stop()?;
            let o = s.snapshot()?;
            if o.fault_flags != 0 {
                bail!(
                    "servo faulted: flags {:#04x} code {}",
                    o.fault_flags,
                    o.fault_code
                );
            }
            s.write(control::MODE, 0)?;
            s.write(control::GOAL_DUTY, 0)?;
            lease.torque_on(s)?;
            rest(s, SETTLE_MS)?;
            lease.keep(s)?;
            let mask = chans.for_step(step, polarity);
            let cap = capture_burst(s.client(), id, step, 0, mask, false)?;
            lease.write(s, control::TORQUE_ENABLE, 0)?;
            println!(
                "  burst {k}: step {step:+} q15, chans {} frame_len {}, step at {}, pos {}",
                cap.meta.chans, cap.meta.frame_len, cap.meta.step_index, cap.meta.pos
            );
            on_cap(k, &cap)?;
            rest(s, rest_ms)?;
        }
        Ok(())
    })
}

fn write_capture(dir: &Path, k: usize, cap: &Capture) -> Result<()> {
    let p = dir.join(format!("burst-{k}.csv"));
    let mut w = std::fs::File::create(&p).with_context(|| format!("create {}", p.display()))?;
    w.write_all(to_csv(cap).as_bytes())
        .with_context(|| format!("write {}", p.display()))
}

/// Entry from the top-level `osc burst` dispatch.
pub fn run(args: &Args, bus: Bus, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let id = Id::new(id);
    let mut c = crate::rig::connect(&bus, &[id])?;
    crate::state::check(&mut c, id)?;
    let lim = crate::rig::limits::read(&mut c, id)?;
    let rail_mv = rail_mv(&mut c, id, &lim)?;
    check(&args.duty_pct, args.rest_ms, rail_mv)?;
    let steps = plan(&args.duty_pct, args.dirs, args.repeats);

    std::fs::create_dir_all(&args.out).with_context(|| format!("mkdir {}", args.out.display()))?;
    let mut meta = servo_meta(&mut c, id)?;
    meta.insert("duty_pct".into(), args.duty_pct.clone().into());
    meta.insert("dirs".into(), args.dirs.signs().into());
    meta.insert("repeats".into(), args.repeats.into());
    meta.insert("chans".into(), format!("{:?}", args.chans).into());
    meta.insert("rest_ms".into(), args.rest_ms.into());
    meta.insert("settle_ms".into(), SETTLE_MS.into());
    meta.insert("rail_mv".into(), rail_mv.round().into());
    let meta_path = args.out.join("meta.json");
    std::fs::write(&meta_path, serde_json::to_string_pretty(&meta)?)
        .with_context(|| format!("write {}", meta_path.display()))?;

    println!(
        "[burst] {} captures from rest at {} on a {:.2} V rail",
        steps.len(),
        args.duty_pct
            .iter()
            .map(|p| format!("{p}%"))
            .collect::<Vec<_>>()
            .join(", "),
        rail_mv / 1000.0
    );
    let mut s = Wire::new(c, id);
    record(&mut s, &steps, args.chans, args.rest_ms, |k, cap| {
        write_capture(&args.out, k, cap)
    })?;
    println!("bursts: {}", args.out.display());
    println!("meta:   {}", meta_path.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plan_runs_each_duty_by_sign_then_repeat() {
        let p = plan(&[25, 40], Dirs::Both, 2);
        assert_eq!(p, [8191, 8191, -8191, -8191, 13106, 13106, -13106, -13106]);
        assert_eq!(plan(&[25], Dirs::Rev, 1), [-8191]);
    }

    /// 40% of 5 V is 2 V, under the cap; 40% of 8.4 V is 3.36 V, over it.
    #[test]
    fn a_step_over_the_volts_cap_is_refused() {
        check(&[25, 40], 150, 5000.0).unwrap();
        let e = check(&[25, 40], 150, 8400.0).unwrap_err().to_string();
        assert!(e.contains("40%"), "{e}");
        assert!(check(&[25], 150, 0.0).is_err());
    }

    #[test]
    fn a_rest_under_the_arm_spacing_is_refused() {
        assert!(check(&[25], 99, 5000.0).is_err());
        check(&[25], 100, 5000.0).unwrap();
    }

    #[test]
    fn chans_parse_as_the_ident_flag_does() {
        assert_eq!(parse_chans("diff").unwrap(), Chans::Diff);
        assert_eq!(parse_chans("3").unwrap(), Chans::Fixed(3));
        assert!(parse_chans("16").is_err());
    }
}
