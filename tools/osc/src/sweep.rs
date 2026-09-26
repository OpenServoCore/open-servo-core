//! `osc sweep` - raw open-loop duty sweep for empirical plant capture:
//! per direction x per duty rung, seek to a start band by polling, brake and
//! settle so the rung opens on a still shaft, then one
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
//! `then:PCT` is a drive step chained the same way (no seek before it): a
//! reversal pair like `20,then:-20` captures gear play with no reposition
//! between the two drives.
//!
//! Duty sign is taken as-is (fwd = +duty): the kernel applies
//! `drive_polarity` at the motor output, so +duty moves counts up on a
//! calibrated servo. The seek's distance-growing bail stays as the guard for
//! a servo that was never calibrated.
//!
//! `--stall` seeks the physical end stop instead of a start band and leaves
//! the shaft there, so the rungs push into it: a locked-output ladder for
//! resistance and inductance, with no back-EMF in the onset. It sets the
//! control table's `stall_permit` for the run and clears it after. Without
//! that the kernel zeroes the outbound duty at the wall and trips the stall
//! timer, and no soft-limit value avoids it - a stop can sit AT the position
//! rail, which is where both of this servo's are.

use std::io::Write;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::frame::TelFrame;
use osc_ident::regs::{Reg, calib, config, control};

use crate::descriptor;
use crate::rig::park::park;
use crate::rig::pump::{
    self, BurstStats, STOP, exchange_tel_burst, read_snapshot, with_guard, write_reg,
};
use crate::rig::snapshot::read_u16;

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub(crate) enum Dirs {
    Both,
    Fwd,
    Rev,
}

impl Dirs {
    pub(crate) fn signs(self) -> &'static [i8] {
        match self {
            Dirs::Both => &[1, -1],
            Dirs::Fwd => &[1],
            Dirs::Rev => &[-1],
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Decay {
    Slow,
    Fast,
}

impl Decay {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Decay::Slow => "slow",
            Decay::Fast => "fast",
        }
    }
}

/// One schedule entry: a drive rung, a chained drive, or a zero-duty
/// coast/brake segment. Drive and Then carry an optional per-step window in
/// ms, overriding `--window-ms`. A rung is one unpolled burst that nothing can
/// cut short, so its window is the ONLY thing bounding how far the shaft
/// travels: a slow rung needs a long one to cross the same span a fast rung
/// crosses in 150 ms, and a fast rung given the slow one's window drives into
/// the end stop.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Step {
    Drive(u8, Option<u32>),
    Then(i8, Option<u32>),
    Coast(u32),
    Brake(u32),
}

impl std::fmt::Display for Step {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Step::Drive(pct, None) => write!(f, "{pct}"),
            Step::Drive(pct, Some(ms)) => write!(f, "{pct}@{ms}"),
            Step::Then(pct, None) => write!(f, "then:{pct}"),
            Step::Then(pct, Some(ms)) => write!(f, "then:{pct}@{ms}"),
            Step::Coast(ms) => write!(f, "coast:{ms}"),
            Step::Brake(ms) => write!(f, "brake:{ms}"),
        }
    }
}

impl std::str::FromStr for Step {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_step(s)
    }
}

impl<'de> serde::Deserialize<'de> for Step {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        parse_step(&String::deserialize(d)?).map_err(serde::de::Error::custom)
    }
}

pub(crate) fn parse_step(s: &str) -> Result<Step, String> {
    let ms = |v: &str| v.parse().map_err(|_| format!("bad ms {v:?}"));
    if let Some(v) = s.strip_prefix("coast:") {
        return Ok(Step::Coast(ms(v)?));
    }
    if let Some(v) = s.strip_prefix("brake:") {
        return Ok(Step::Brake(ms(v)?));
    }
    // `PCT@MS` splits the window off first so both drive forms share it.
    let (head, window) = match s.split_once('@') {
        Some((h, w)) => {
            let w: u32 = ms(w)?;
            if w == 0 {
                return Err("step window must be nonzero".into());
            }
            (h, Some(w))
        }
        None => (s, None),
    };
    if let Some(v) = head.strip_prefix("then:") {
        let pct: i8 = v.parse().map_err(|_| format!("bad pct {v:?}"))?;
        if pct == 0 || pct.unsigned_abs() > 100 {
            return Err(format!(
                "then: pct must be nonzero within +/-100, got {pct}"
            ));
        }
        return Ok(Step::Then(pct, window));
    }
    head.parse()
        .map(|pct| Step::Drive(pct, window))
        .map_err(|_| {
            format!(
                "step is duty pct, then:PCT, coast:MS, or brake:MS, each optionally @MS, got {s:?}"
            )
        })
}

/// Whether step k feeds step k+1 with no settle between them: then/coast/
/// brake steps chain to their predecessor so momentum carries across the
/// burst gap.
fn feeds(steps: &[Step], k: usize) -> bool {
    matches!(
        steps.get(k + 1),
        Some(Step::Then(..) | Step::Coast(_) | Step::Brake(_))
    )
}

/// `osc sweep` args. `--baud`/`--id` come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir; sweep.csv + meta.json land here directly.
    #[arg(long, default_value = "./sweep-out")]
    out: PathBuf,
    /// Duty grid, percent of full scale; `coast:MS` / `brake:MS` entries
    /// capture a zero-duty segment chained to the preceding step, `then:PCT`
    /// a signed drive chained the same way (no seek).
    #[arg(long, value_delimiter = ',', value_parser = parse_step,
          default_values_t = (1..=20u8).map(|k| Step::Drive(k * 5, None)))]
    duty_pct: Vec<Step>,
    #[arg(long, value_enum, default_value_t = Dirs::Both)]
    dirs: Dirs,
    /// openloop_decay for the capture bursts; seeks and brakes stay slow.
    #[arg(long, value_enum, default_value_t = Decay::Slow)]
    decay: Decay,
    /// Capture per rung; samples = window_ms x 20 ticks.
    #[arg(long, default_value_t = 150)]
    window_ms: u32,
    /// Torque-off cool-down between rungs.
    #[arg(long, default_value_t = 500)]
    rest_ms: u32,
    /// Torque-off noise-floor capture before the grid; 0 skips it.
    #[arg(long, default_value_t = 1000)]
    baseline_ms: u32,
    /// Seek drive, percent of full scale.
    #[arg(long, default_value_t = 28)]
    seek_duty_pct: u8,
    /// Ceiling for the breakout escalation, percent of full scale. LEAVING a
    /// stop needs far more torque than holding against one, and the supply
    /// sets what a given duty buys: 40% frees this servo on 2S (~0.58 A) but
    /// the same current needs ~63% on USB's 4.5 V rail, so a 45% cap strands
    /// the shaft there. Raise it only as far as needed - breakout loads the
    /// teeth like a stall does, and this servo slips above ~0.3 A.
    #[arg(long, default_value_t = 45)]
    seek_cap_pct: u8,
    /// Seek the physical end stop and ladder against it instead of seeking a
    /// start band. Chain the rungs (`20,then:25,...`) to hold the gear train
    /// wound up across the whole ladder: released, it unwinds and the next
    /// onset measures the re-wind instead of the winding.
    #[arg(long)]
    stall: bool,
    /// The load is a resistor, not a motor: no shaft to seek, brake or
    /// settle, so every seek is skipped and the rungs drive directly.
    /// Implies the stall permit - the rungs draw current with no motion,
    /// which is exactly what the stall trip exists to catch.
    #[arg(long)]
    static_load: bool,
    /// Brake-and-hold after the seek, before the rung arms. Without it the
    /// rung opens on a shaft still coasting from the seek, and since a fwd
    /// rung's start band is at the LOW guard the coast is BACKWARD: measured
    /// -0.70 V of entry EMF, which lands in the intercept of any onset fit.
    /// Settled-window analysis does not care; onset R and L do.
    #[arg(long, default_value_t = 300)]
    settle_ms: u32,
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
    /// Attempts per rung before the sweep gives up. A rung is captured as one
    /// burst, so a dropped frame anywhere in it corrupts that rung alone -
    /// retrying just the rung costs seconds where discarding the recording
    /// costs a minute and a half of good rungs with it.
    #[arg(long, default_value_t = 3)]
    rung_tries: u32,
}

/// What one sweep run drives: `Args` minus the output dir, mask parsed.
pub(crate) struct Cfg {
    pub(crate) steps: Vec<Step>,
    pub(crate) dirs: Dirs,
    pub(crate) decay: Decay,
    pub(crate) window_ms: u32,
    pub(crate) rest_ms: u32,
    pub(crate) baseline_ms: u32,
    pub(crate) seek_duty_pct: u8,
    pub(crate) seek_cap_pct: u8,
    pub(crate) settle_ms: u32,
    pub(crate) stall: bool,
    pub(crate) static_load: bool,
    pub(crate) guard: (u16, u16),
    pub(crate) tel_mask: u16,
    pub(crate) rung_tries: u32,
}

impl TryFrom<&Args> for Cfg {
    type Error = anyhow::Error;

    fn try_from(a: &Args) -> Result<Self> {
        Ok(Self {
            steps: a.duty_pct.clone(),
            dirs: a.dirs,
            decay: a.decay,
            window_ms: a.window_ms,
            rest_ms: a.rest_ms,
            baseline_ms: a.baseline_ms,
            seek_duty_pct: a.seek_duty_pct,
            seek_cap_pct: a.seek_cap_pct,
            settle_ms: a.settle_ms,
            stall: a.stall,
            static_load: a.static_load,
            guard: (a.guard_lo, a.guard_hi),
            tel_mask: crate::parse_u16(&a.tel_mask).context("--tel-mask")?,
            rung_tries: a.rung_tries,
        })
    }
}

/// One committed segment: the baseline (seg 0) or one schedule step of one
/// direction, after its chain passed clean.
pub(crate) struct Segment {
    pub(crate) seg: u32,
    pub(crate) dir: i8,
    pub(crate) cmd_duty_q15: i16,
    pub(crate) frames: Vec<TelFrame>,
    pub(crate) stats: BurstStats,
}

/// A run that completed every chain, segments in commit order.
pub(crate) struct Recording {
    pub(crate) segments: Vec<Segment>,
}

/// One rung's start band: a narrow window CENTRED on the launch guard, so a
/// rung starts at `guard_lo` (fwd) or `guard_hi` (rev) give or take
/// `BAND_HALF`. Centring matters because the seek stops the instant it
/// enters the band and it does not always arrive from mid travel - a rung
/// that ends against a rail leaves the next seek approaching from the far
/// side, and a band offset to one side of the guard then starts the rung a
/// full band-width from where it was asked to.
pub(crate) const BAND_HALF: u16 = 75;

fn start_band(dir: i8, guard_lo: u16, guard_hi: u16) -> (u16, u16) {
    let c = if dir > 0 { guard_lo } else { guard_hi };
    (c.saturating_sub(BAND_HALF), c.saturating_add(BAND_HALF))
}

/// Why a rung is being replayed. Deliberately avoids the words the committed
/// segment line uses: callers scrape sweep stdout to decide a whole recording
/// failed (`captures/chain3/campaign.sh` greps `[1-9][0-9]* seq holes` and
/// `[1-9][0-9]* garble`), so a note about a rung we RECOVERED must not read as
/// a failed capture. Both halves matter - quoting only the counts still trips
/// the garble half via `holes=2 garble=0`. `retry_note_cannot_read_as_failure`
/// pins it.
fn retry_note(
    seg: u32,
    what: &str,
    holes: impl std::fmt::Display,
    garble: impl std::fmt::Display,
) -> String {
    format!("  seg {seg} ({what}) [h={holes} g={garble}]")
}

pub(crate) fn pct_q15(pct: u8) -> i16 {
    (pct as i32 * 32767 / 100) as i16
}

pub(crate) fn samples_of_ms(ms: u32) -> u16 {
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
fn seek_band(
    c: &mut Client<NusbPipe>,
    id: Id,
    (lo, hi): (u16, u16),
    duty_q15: i16,
    cap: i32,
) -> Result<()> {
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    // Distance to the band, not envelope membership: a seek may legally
    // START at a rail (that is what it is for); reversed polarity shows as
    // the distance GROWING while driving.
    let dist = |pos: u16| (lo.saturating_sub(pos)) as u32 + (pos.saturating_sub(hi)) as u32;
    let mut best = u32::MAX;
    // Leaving a stop costs more than arriving at one, and this seek starts
    // wherever the last rung parked - often hard against a rail. Escalate
    // when nothing moves: it drives TOWARD the middle, so there is nothing to
    // hit and the extra duty is free. The gentle duty belongs on the arrival.
    let mut mag = duty_q15 as i32;
    let mut still = 0;
    let mut last = u16::MAX;
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
        if pos.abs_diff(last) > STALL_EPS {
            still = 0;
        } else {
            still += 1;
            if still >= STALL_POLLS {
                still = 0;
                mag = (mag + SEEK_STEP_Q15).min(cap);
            }
        }
        last = pos;
        let duty = if pos < lo { mag } else { -mag };
        write_reg(c, id, control::GOAL_DUTY, duty)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    write_reg(c, id, control::GOAL_DUTY, 0)?;
    bail!("seek did not reach {lo}..{hi} in 10 s (gear slipping?)")
}

/// End-stop detect: `STALL_POLLS` consecutive reads moving no more than
/// `STALL_EPS` counts call it the mechanical rail. Mirrors osc-ident's
/// end-stop detector so the two agree on where the ends are.
const STALL_EPS: u16 = 3;
const STALL_POLLS: u32 = 8;
/// Breakout escalation when the shaft will not leave a stop: +5% of full
/// scale per stalled window, up to `--seek-cap-pct`.
const SEEK_STEP_Q15: i32 = 1638;
/// Counts of travel before a seek believes it has actually gone somewhere.
/// STALL_EPS is far too small for this: a few counts of ELASTIC WIND-UP at a
/// stop passes it, and a seek that mistakes wind-up for travel declares the
/// stop it is leaning on to be the one it was sent to find. Measured: a
/// reverse ladder ran eleven rungs against the FORWARD stop that way, then
/// broke free mid-ladder and traversed half the range.
const SEEK_TRAVEL_MIN: u16 = 100;

/// Drive into the physical end stop and leave the shaft pressed against it.
/// The rungs push the same way, so the train's backlash is taken up before
/// the first burst instead of during it.
///
/// A soft position limit is indistinguishable from a mechanical stop by
/// position alone - the kernel zeroes outbound open-loop duty when the
/// endstop band collapses, so the shaft stops either way. DUTY_APPLIED_Q15
/// tells them apart: it holds at a real stop and reads zero at a soft limit.
/// Without that check a run would ladder against a clamped duty and record a
/// grid of zero-current rungs.
fn seek_stop(c: &mut Client<NusbPipe>, id: Id, dir: i8, duty_q15: i16, cap: i32) -> Result<()> {
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    let mut duty = duty_q15 as i32;
    let start = check_fault(c, id)?;
    let mut last = start;
    let mut still = 0;
    // A stop only counts once the shaft has actually travelled. Standing
    // still is what BOTH "arrived" and "stuck against the far stop" look
    // like, and reading the second as the first runs the whole ladder at the
    // wrong end - measured, and it walked free mid-ladder once the duty rose.
    let mut moved = false;
    for _ in 0..500 {
        check_stop()?;
        write_reg(c, id, control::GOAL_DUTY, dir as i32 * duty)?;
        std::thread::sleep(Duration::from_millis(20));
        let pos = check_fault(c, id)?;
        if pos.abs_diff(start) >= SEEK_TRAVEL_MIN {
            moved = true;
        }
        // Judge PROGRESS over a window, not stillness poll to poll. Consecutive
        // -still counting is defeated by a couple of counts of ADC jitter at
        // the position rail: `still` resets, so the escalation below never
        // fires and the seek just drives at its opening duty until the loop
        // runs out. Net travel over the window is immune to that.
        still += 1;
        if still >= STALL_POLLS {
            still = 0;
            let progress = pos.abs_diff(last) >= SEEK_TRAVEL_MIN;
            last = pos;
            if !progress {
                if moved {
                    // Duty stays on: releasing lets the train unwind, and the
                    // burst would then re-wind it inside the measurement.
                    return Ok(());
                }
                // Leaving a stop costs more than holding against one: 20% of
                // full scale draws current without moving the shaft at all
                // where 40% frees it. Step up rather than guess a constant.
                duty += SEEK_STEP_Q15;
                if duty > cap {
                    write_reg(c, id, control::GOAL_DUTY, 0)?;
                    bail!(
                        "shaft never moved at pos {pos}, up to {duty} q15 - jammed, \
                         or stall_permit is not set and the kernel is zeroing the duty"
                    );
                }
            }
        }
    }
    write_reg(c, id, control::GOAL_DUTY, 0)?;
    bail!("no end stop in 10 s driving {dir:+} at {duty} q15")
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

pub(crate) const CSV_HEADER: &str = "seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,duty_q15,vdiff,vbus,current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw";

pub(crate) fn write_rows(w: &mut impl Write, s: &Segment) -> Result<()> {
    let opt = |v: Option<i32>| v.map(|v| v.to_string()).unwrap_or_default();
    let (seg, cmd_duty_q15, dir) = (s.seg, s.cmd_duty_q15, s.dir);
    for f in &s.frames {
        writeln!(
            w,
            "{seg},{cmd_duty_q15},{dir},{},{},{},{},{},{},{},{},{},{},{},{},{}",
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
            opt(f.vbus_raw.map(|v| v as i32)),
            opt(f.ntc_raw.map(|v| v as i32)),
        )?;
    }
    Ok(())
}

pub(crate) fn git_sha() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}

pub(crate) fn git_toplevel() -> Result<PathBuf> {
    std::process::Command::new("git")
        .args(["rev-parse", "--show-toplevel"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| PathBuf::from(s.trim()))
        .context("not inside a git checkout: pass --root")
}

/// The run's constants, written as meta.json before the first burst.
pub(crate) fn meta(c: &mut Client<NusbPipe>, id: Id, cfg: &Cfg) -> Result<serde_json::Value> {
    let identity = c.identity(id)?;
    let tick_hz = read_u16(c, id, calib::TICK_HZ)?;
    let vbus_counts = read_snapshot(c, id)?.vbus_counts;
    Ok(serde_json::json!({
        "model": identity.model,
        "fw": identity.fw,
        "tick_hz": tick_hz,
        "vbus_counts": vbus_counts,
        "sense": {
            "shunt_r_mohm": read_u16(c, id, calib::SHUNT_R_MOHM)?,
            "gain_milli": read_u16(c, id, calib::GAIN_MILLI)?,
            "vmotor_div_top": read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
            "vmotor_div_bot": read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
            "vdd_mv": read_u16(c, id, calib::VDD_MV)?,
        },
        "schedule": cfg.steps.iter().map(Step::to_string).collect::<Vec<_>>(),
        "decay": cfg.decay.as_str(),
        "dirs": cfg.dirs.signs(),
        "window_ms": cfg.window_ms,
        "rung_tries": cfg.rung_tries,
        "rest_ms": cfg.rest_ms,
        "baseline_ms": cfg.baseline_ms,
        "seek_duty_pct": cfg.seek_duty_pct,
        "seek_cap_pct": cfg.seek_cap_pct,
        "settle_ms": cfg.settle_ms,
        "stall": cfg.stall,
        "static_load": cfg.static_load,
        "guard": [cfg.guard.0, cfg.guard.1],
        "tel_mask": cfg.tel_mask,
        "post_rung_brake": true,
        "git_sha": git_sha(),
    }))
}

/// The descriptor-placed open-loop registers the run toggles.
struct Regs {
    decay: Reg,
    zero_brake: Reg,
}

fn resolve_regs(c: &mut Client<NusbPipe>, id: Id) -> Result<Regs> {
    let identity = c.identity(id)?;
    let registry = descriptor::load()?;
    let (d, note) = descriptor::select(&registry, identity.model, identity.fw)?;
    if let Some(note) = note {
        println!("{note}");
    }
    let field_reg = |name: &str| -> Result<Reg> {
        let f = descriptor::field(d, name)?;
        Ok(Reg {
            addr: f.addr,
            width: f.width as u8,
        })
    };
    Ok(Regs {
        decay: field_reg("openloop_decay")?,
        zero_brake: field_reg("openloop_zero_brake")?,
    })
}

/// Baseline, then every chain of every direction. Each committed segment
/// reaches `on_seg` as it lands, so a caller keeps what was captured before
/// a later chain gives up. Leaves the servo guarded and torqued off.
pub(crate) fn record(
    c: &mut Client<NusbPipe>,
    id: Id,
    cfg: &Cfg,
    mut on_seg: impl FnMut(&Segment) -> Result<()>,
) -> Result<Recording> {
    let regs = resolve_regs(c, id)?;
    let r = with_guard(c, id, |c| chains(c, id, cfg, &regs, &mut on_seg));
    // Belt for a run cut mid-burst: brake flag clear, decay back to slow,
    // guards back on. The permit is RAM-only so a power cycle clears it
    // anyway, but a servo left unguarded until someone reboots it is a trap.
    let _ = write_reg(c, id, regs.zero_brake, 0);
    let _ = write_reg(c, id, regs.decay, Decay::Slow as i32);
    let _ = write_reg(c, id, control::STALL_PERMIT, 0);
    r
}

fn chains(
    c: &mut Client<NusbPipe>,
    id: Id,
    cfg: &Cfg,
    regs: &Regs,
    on_seg: &mut impl FnMut(&Segment) -> Result<()>,
) -> Result<Recording> {
    let mask = cfg.tel_mask;
    let seek_duty = pct_q15(cfg.seek_duty_pct);
    let seek_cap = pct_q15(cfg.seek_cap_pct) as i32;
    let mut segments = Vec::new();
    let mut commit = |s: Segment| -> Result<()> {
        on_seg(&s)?;
        segments.push(s);
        Ok(())
    };

    write_reg(c, id, regs.decay, Decay::Slow as i32)?;
    write_reg(c, id, regs.zero_brake, 0)?;
    write_reg(c, id, control::TEL_MASK, mask as i32)?;
    if cfg.stall || cfg.static_load {
        write_reg(c, id, control::STALL_PERMIT, 1)?;
    }

    // baseline: mid-travel, torque off, noise floor at full tick rate
    if cfg.baseline_ms > 0 {
        println!("[baseline] {} ms torque-off", cfg.baseline_ms);
        if !cfg.static_load {
            seek_band(c, id, (1750, 2350), seek_duty, seek_cap)?;
        }
        write_reg(c, id, control::TORQUE_ENABLE, 0)?;
        let (frames, st) = exchange_tel_burst(c, id, samples_of_ms(cfg.baseline_ms), None, mask)?;
        println!(
            "  seg 0: {} frames, {} samples, {} seq holes, {} garble bytes",
            st.frames, st.samples, st.holes, st.garble
        );
        commit(Segment {
            seg: 0,
            dir: 0,
            cmd_duty_q15: 0,
            frames,
            stats: st,
        })?;
    }

    let steps = &cfg.steps;

    // Retry granularity is the CHAIN, not the step: then/coast/brake steps
    // inherit momentum from the drive they follow, so replaying one alone
    // would capture it from the wrong state. A chain starts wherever the
    // previous step does not feed this one.
    let starts: Vec<usize> = (0..steps.len())
        .filter(|&k| k == 0 || !feeds(steps, k - 1))
        .collect();

    let mut seg = 1u32;
    for &dir in cfg.dirs.signs() {
        for (ci, &chain0) in starts.iter().enumerate() {
            let chain_end = starts.get(ci + 1).copied().unwrap_or(steps.len());
            let seg0 = seg;
            let mut attempt = 0u32;
            let mut pending: Vec<(Segment, String)> = Vec::new();
            loop {
                attempt += 1;
                pending.clear();
                let mut dirty = None;
                let mut live = false;
                seg = seg0;
                for k in chain0..chain_end {
                    let step = steps[k];
                    check_stop()?;
                    // Drive is never chained, so it always arrives with live
                    // false: the seek is the only difference in its prep.
                    if let Step::Drive(..) = step {
                        check_fault(c, id)?;
                        if cfg.static_load {
                            // nothing to seek
                        } else if cfg.stall {
                            // Already stopped, and still pressed into the stop:
                            // nothing to brake and nothing to let settle.
                            seek_stop(c, id, dir, seek_duty, seek_cap)?;
                        } else {
                            seek_band(
                                c,
                                id,
                                start_band(dir, cfg.guard.0, cfg.guard.1),
                                seek_duty,
                                seek_cap,
                            )?;
                            // Kill the seek's momentum and let the shaft ring
                            // down. Sign is -dir, the mirror of the post-rung
                            // call: the seek parks NEAR its start-band wall, so
                            // the token brake duty has to point away from that
                            // one instead.
                            brake_to_rest(c, id, -dir)?;
                            rest(cfg.settle_ms)?;
                        }
                    } else if !live {
                        check_fault(c, id)?;
                    }
                    if !live {
                        write_reg(c, id, control::MODE, 0)?;
                        write_reg(c, id, control::TORQUE_ENABLE, 1)?;
                    }
                    let (ms, duty) = match step {
                        Step::Drive(pct, ms) => (
                            ms.unwrap_or(cfg.window_ms),
                            dir as i32 * pct_q15(pct) as i32,
                        ),
                        Step::Then(pct, ms) => (
                            ms.unwrap_or(cfg.window_ms),
                            dir as i32 * pct.signum() as i32 * pct_q15(pct.unsigned_abs()) as i32,
                        ),
                        Step::Coast(ms) | Step::Brake(ms) => (ms, 0),
                    };
                    if matches!(step, Step::Brake(_)) {
                        write_reg(c, id, regs.zero_brake, 1)?;
                    }
                    // Fast decay only inside the burst: the seek and the post-step
                    // brake need slow decay to move and to stop.
                    if cfg.decay == Decay::Fast {
                        write_reg(c, id, regs.decay, Decay::Fast as i32)?;
                    }
                    let (frames, st) = exchange_tel_burst(
                        c,
                        id,
                        samples_of_ms(ms),
                        Some((control::GOAL_DUTY, duty)),
                        mask,
                    )?;
                    if cfg.decay == Decay::Fast {
                        write_reg(c, id, regs.decay, Decay::Slow as i32)?;
                    }
                    if matches!(step, Step::Brake(_)) {
                        write_reg(c, id, regs.zero_brake, 0)?;
                    }
                    let what = match step {
                        Step::Drive(pct, _) => format!("duty {:+}%", dir as i32 * pct as i32),
                        Step::Then(pct, _) => format!("then {:+}%", dir as i32 * pct as i32),
                        Step::Coast(ms) => format!("coast {ms} ms"),
                        Step::Brake(ms) => format!("brake {ms} ms"),
                    };
                    if st.holes > 0 || st.garble > 0 {
                        dirty = Some(retry_note(seg, &what, st.holes, st.garble));
                    }
                    let s = Segment {
                        seg,
                        dir,
                        cmd_duty_q15: duty as i16,
                        frames,
                        stats: st,
                    };
                    pending.push((s, what));
                    if feeds(steps, k) {
                        live = true;
                    } else {
                        brake_to_rest(c, id, dir)?;
                        write_reg(c, id, control::TORQUE_ENABLE, 0)?;
                        rest(cfg.rest_ms)?;
                        live = false;
                    }
                    seg += 1;
                }
                match dirty {
                    None => break,
                    Some(why) if attempt < cfg.rung_tries => {
                        println!("  retry rung, attempt {attempt}:{}", why.trim_start());
                    }
                    Some(why) => bail!(
                        "rung failed {} attempts, giving up:{}",
                        cfg.rung_tries,
                        why.trim_start()
                    ),
                }
            }
            for (s, what) in pending.drain(..) {
                let st = &s.stats;
                println!(
                    "  seg {} ({what}): {} frames, {} samples, {} seq holes, {} garble bytes",
                    s.seg, st.frames, st.samples, st.holes, st.garble
                );
                commit(s)?;
            }
        }
    }
    Ok(Recording { segments })
}

/// Entry from the top-level `osc sweep` dispatch.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let cfg = Cfg::try_from(args)?;
    let mut c = crate::rig::connect(&baud)?;
    let id = Id::new(id);

    std::fs::create_dir_all(&args.out).with_context(|| format!("mkdir {}", args.out.display()))?;

    let meta = meta(&mut c, id, &cfg)?;
    let meta_path = args.out.join("meta.json");
    std::fs::write(&meta_path, serde_json::to_string_pretty(&meta)?)
        .with_context(|| format!("write {}", meta_path.display()))?;

    let csv_path = args.out.join("sweep.csv");
    let mut w = std::io::BufWriter::new(
        std::fs::File::create(&csv_path)
            .with_context(|| format!("create {}", csv_path.display()))?,
    );
    writeln!(w, "{CSV_HEADER}")?;

    // A resistor has no shaft to park.
    let center = if cfg.static_load {
        None
    } else {
        let lo = pump::read_i32(&mut c, id, config::POS_MIN_SOFT_COUNTS)?;
        let hi = pump::read_i32(&mut c, id, config::POS_MAX_SOFT_COUNTS)?;
        Some(((lo + hi) / 2).clamp(0, u16::MAX as i32) as u16)
    };

    let r = record(&mut c, id, &cfg, |s| write_rows(&mut w, s));
    let flushed = w.flush();
    let parked = center.map_or(Ok(()), |at| park(&mut c, id, at));
    flushed?;
    r?;
    parked?;
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
            ("35", Step::Drive(35, None)),
            ("35@800", Step::Drive(35, Some(800))),
            ("then:-20", Step::Then(-20, None)),
            ("then:20", Step::Then(20, None)),
            ("then:20@450", Step::Then(20, Some(450))),
            ("coast:200", Step::Coast(200)),
            ("brake:150", Step::Brake(150)),
        ] {
            assert_eq!(parse_step(s).unwrap(), want);
            assert_eq!(want.to_string(), s);
        }
        assert!(parse_step("coast").is_err());
        assert!(parse_step("brake:x").is_err());
        assert!(parse_step("then:0").is_err());
        assert!(parse_step("then:101").is_err());
        assert!(parse_step("then:-101").is_err());
        assert!(parse_step("then:x").is_err());
        assert!(parse_step("slow").is_err());
        assert!(parse_step("35@0").is_err());
        assert!(parse_step("35@x").is_err());
        assert!(parse_step("@800").is_err());
    }

    #[test]
    fn chained_steps_feed_from_their_predecessor() {
        let steps = [
            Step::Drive(20, None),
            Step::Then(-20, None),
            Step::Then(20, None),
            Step::Brake(200),
        ];
        let fed: Vec<bool> = (0..steps.len()).map(|k| feeds(&steps, k)).collect();
        assert_eq!(fed, [true, true, true, false]);
        assert!(!feeds(&[Step::Then(-20, None), Step::Drive(20, None)], 0));
    }

    #[test]
    fn retry_note_cannot_read_as_failure() {
        // the exact pattern captures/chain3/campaign.sh scrapes stdout with
        let bad = regex_lite(&retry_note(7, "duty +35%", 4, 9));
        assert!(!bad, "retry note reads as a failed capture");
        // and the committed line for a genuinely dirty rung still must match,
        // otherwise the outer gate would stop catching anything
        assert!(regex_lite(
            "  seg 7 (duty +35%): 200 frames, 4 seq holes, 0 garble bytes"
        ));
    }

    /// `[1-9][0-9]* seq holes` or `[1-9][0-9]* garble`, without a regex dep:
    /// a digit run ending just before the tail, whose first digit is not 0.
    fn regex_lite(line: &str) -> bool {
        [" seq holes", " garble"].iter().any(|tail| {
            line.match_indices(tail).any(|(i, _)| {
                let head = &line[..i];
                let digits = head.len() - head.trim_end_matches(|c: char| c.is_ascii_digit()).len();
                digits > 0 && !head[head.len() - digits..].starts_with('0')
            })
        })
    }

    #[test]
    fn cfg_carries_the_cli_defaults() {
        #[derive(clap::Parser)]
        struct Cli {
            #[command(flatten)]
            args: Args,
        }
        let cli = <Cli as clap::Parser>::try_parse_from(["sweep"]).unwrap();
        let cfg = Cfg::try_from(&cli.args).unwrap();
        let grid: Vec<Step> = (1..=20u8).map(|k| Step::Drive(k * 5, None)).collect();
        assert_eq!(cfg.steps, grid);
        assert_eq!(cfg.dirs, Dirs::Both);
        assert_eq!(cfg.decay, Decay::Slow);
        assert_eq!(cfg.window_ms, 150);
        assert_eq!(cfg.rest_ms, 500);
        assert_eq!(cfg.baseline_ms, 1000);
        assert_eq!((cfg.seek_duty_pct, cfg.seek_cap_pct), (28, 45));
        assert_eq!(cfg.settle_ms, 300);
        assert!(!cfg.stall && !cfg.static_load);
        assert_eq!(cfg.guard, (150, 3950));
        assert_eq!(cfg.tel_mask, 0x1cd);
        assert_eq!(cfg.rung_tries, 3);
    }

    #[test]
    fn schedule_strings_parse_through_the_cli_grammar() {
        let steps: Vec<Step> = ["20@450", "then:-20", "brake:150"]
            .iter()
            .map(|s| s.parse().unwrap())
            .collect();
        assert_eq!(
            steps,
            [
                Step::Drive(20, Some(450)),
                Step::Then(-20, None),
                Step::Brake(150)
            ]
        );
    }

    #[test]
    fn start_bands_hug_the_guard_edges() {
        // centred on the guard, so approach direction cannot shift the start
        assert_eq!(start_band(1, 400, 3700), (325, 475));
        assert_eq!(start_band(-1, 400, 3700), (3625, 3775));
    }
}
