//! Servo clock truth: reads the servo's clock against the adapter crystal
//! from the char pitch of its replies (bench `clock`), optionally around
//! CAL trains (protocol sec 9.3), plus the instrument's self-check on the
//! adapter's own echo. One JSON object per line on stdout.

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, bail};
use bench::cli::{Connect, SETTLE_MS, Target, parse_addr};
use bench::clock::{
    Reject, Reply, Summary, adapter_echo, adapter_offset_ppm, readings, servo_truth, train_pacing,
};
use bench::osc::{REBOOT_SETTLE_MS, build_cal, build_read, build_reboot};
use bench::run::xfer;
use bench::wire::Wire;
use clap::Parser;
use osc_protocol::table::TRIM_STEPS;
use osc_protocol::wire::ResultCode;

/// Echo self-check frame payload: long enough that one frame resolves the
/// adapter's divisor offsets to a few ppm at every catalog rate.
const ECHO_LEN: usize = 200;

#[derive(Parser, Debug)]
#[command(about = "Read a servo's clock against the adapter crystal (truth for the CAL trim).")]
struct Args {
    /// The servo's catalog rate (the host returns to it for every reading).
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    /// Replies per truth reading (0 = none).
    #[arg(long, default_value_t = 16)]
    replies: u32,
    /// Truth READ span length.
    #[arg(long, default_value_t = 240)]
    len: u16,
    /// Truth READ span address: decimal, 0x-hex, or a field name.
    #[arg(long, default_value = "0", value_parser = parse_addr)]
    addr: u16,
    /// One CAL train per iteration with this wire gap, us.
    #[arg(long)]
    gap_us: Option<u16>,
    /// Gaps per train (the train sends gaps + 1 bare breaks).
    #[arg(long, default_value_t = 8)]
    gaps: u8,
    /// Announced gap, us (default: the wire gap). Any other value is a
    /// lying train: it injects a known clock-offset reading.
    #[arg(long)]
    announce_us: Option<u16>,
    /// Host UART rate during each train or echo, raw bps (default: --baud).
    /// Off-catalog rates are the host detune.
    #[arg(long)]
    host_bps: Option<u32>,
    /// Iterations of [truth, train, trim read]; one more truth closes the run.
    #[arg(long, default_value_t = 1)]
    count: u32,
    /// Instrument self-check: time this many of the adapter's own frames
    /// (sent at --host-bps to --echo-id) instead of the servo.
    #[arg(long)]
    echo: Option<u32>,
    /// An id no servo on the bus holds, for the echo frames.
    #[arg(long, default_value_t = 200)]
    echo_id: u8,
    /// MGMT REBOOT the servo first: the trim loop restarts from the factory
    /// trim. It boots at its SAVED rate, which must be --baud.
    #[arg(long)]
    reboot: bool,
    /// Also print every frame: time, reading or reject reason, and its char
    /// start ticks.
    #[arg(long)]
    dump: bool,
}

fn main() -> Result<()> {
    let a = Args::parse();
    let baud = a.conn.baud;
    let host = a.host_bps.unwrap_or(baud);
    let mut w = a.conn.wire()?;

    if let Some(frames) = a.echo {
        w.set_baud(host)?;
        let replies = adapter_echo(&mut w, a.echo_id, baud, ECHO_LEN, frames)?;
        w.set_baud(baud)?;
        dump(a.dump, 0, &replies);
        println!(
            "{{\"kind\":\"echo\",\"baud\":{baud},\"host_bps\":{host},\"expect_ppm\":{:.3},{}}}",
            adapter_offset_ppm(host, baud),
            reading(&replies)
        );
        return Ok(());
    }

    if a.reboot {
        let ex = xfer(&mut w, &build_reboot(a.target.id), SETTLE_MS)?;
        if ex.status.result != Some(ResultCode::Ok) {
            bail!("reboot nacked: {:?}", ex.status.result);
        }
        sleep(Duration::from_millis(REBOOT_SETTLE_MS));
        println!("{{\"kind\":\"reboot\"}}");
    }

    for i in 0..=a.count {
        let trim = read_trim(&mut w, a.target.id)?;
        if a.replies > 0 {
            let replies = servo_truth(&mut w, a.target.id, baud, a.addr, a.len, a.replies)?;
            dump(a.dump, i, &replies);
            println!(
                "{{\"kind\":\"truth\",\"i\":{i},\"baud\":{baud},\"trim\":{trim},{}}}",
                reading(&replies)
            );
        }
        let Some(gap_us) = a.gap_us.filter(|_| i < a.count) else {
            continue;
        };
        let announce_us = a.announce_us.unwrap_or(gap_us);
        w.set_baud(host)?;
        w.train(
            &build_cal(announce_us, a.gaps),
            gap_us as u32,
            a.gaps as u32 + 1,
        )?;
        let stamps = w.drain_stamps()?;
        w.set_baud(baud)?;
        let pace = train_pacing(&stamps, a.gaps as usize + 1, gap_us as u32 * w.hz_per_us());
        // The decision applies in the servo main loop between frames.
        sleep(Duration::from_millis(SETTLE_MS));
        let after = read_trim(&mut w, a.target.id)?;
        println!(
            "{{\"kind\":\"train\",\"i\":{i},\"gap_us\":{gap_us},\"gaps\":{},\"announce_us\":{announce_us},\
             \"host_bps\":{host},\"pace_ppm\":{},\"pace_gap_err_max\":{},\"trim_before\":{trim},\"trim_after\":{after}}}",
            a.gaps,
            pace.as_ref()
                .map_or("null".into(), |p| format!("{:.1}", p.ppm)),
            pace.as_ref()
                .map_or("null".into(), |p| p.gap_err_max.to_string()),
        );
    }
    Ok(())
}

fn read_trim(w: &mut Wire, id: u8) -> Result<i8> {
    let ex = xfer(w, &build_read(id, TRIM_STEPS, 1), SETTLE_MS)?;
    match (ex.status.result, ex.status.payload.as_slice()) {
        (Some(ResultCode::Ok), [b]) => Ok(*b as i8),
        (r, p) => bail!("trim_steps read: {r:?} {p:?}"),
    }
}

/// JSON fields of one repeated reading.
fn reading(replies: &[Reply]) -> String {
    let (ppm, rejects) = readings(replies);
    match Summary::of(&ppm) {
        Some(s) => format!(
            "\"n\":{},\"rejects\":{rejects},\"ppm\":{:.2},\"sd\":{:.2},\"half95\":{:.2}",
            s.n,
            s.mean,
            s.sd,
            s.half95()
        ),
        None => format!(
            "\"n\":{},\"rejects\":{rejects},\"ppm\":null,\"sd\":null,\"half95\":null",
            ppm.len()
        ),
    }
}

/// One JSON line per frame of reading `i` when `on`.
fn dump(on: bool, i: u32, replies: &[Reply]) {
    if !on {
        return;
    }
    for (r, x) in replies.iter().enumerate() {
        let (ppm, arms, residual, reject) = match &x.fit {
            Ok(f) => (
                format!("{:.2}", f.ppm),
                f.arms.to_string(),
                format!("{:.3}", f.residual_max),
                "null".to_string(),
            ),
            Err(e) => (
                "null".into(),
                "null".into(),
                "null".into(),
                format!("\"{}\"", reject_text(e)),
            ),
        };
        let ticks: Vec<String> = x.ticks.iter().map(u32::to_string).collect();
        println!(
            "{{\"kind\":\"frame\",\"i\":{i},\"r\":{r},\"at_ms\":{:.3},\"ppm\":{ppm},\"arms\":{arms},\
             \"residual_max\":{residual},\"reject\":{reject},\"ticks\":[{}]}}",
            x.at.as_secs_f64() * 1e3,
            ticks.join(",")
        );
    }
}

fn reject_text(e: &Reject) -> String {
    match e {
        Reject::Exchange(m) => format!("exchange: {}", m.replace('"', "'")),
        Reject::Echo => "echo mismatch".into(),
        Reject::Short => "short".into(),
        Reject::Misdecode(bits) => format!("misdecode {bits:.2} bits"),
    }
}
