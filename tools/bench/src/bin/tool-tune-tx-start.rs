//! Measure the chip's `TX_START_ENTRY_TICKS` floor — the back-date the
//! `arm_tim2` path applies to CCR3 so the wire-bit lands at or after the
//! scheduled deadline. The const must be sized to the *minimum* observed
//! CC3-IRQ-entry latency; any larger value would let the wire-bit land
//! before the deadline (collides with prior slot's RDT).
//!
//! Requires the chip to be built with `--features tuning`; otherwise the
//! stamp surface (`TelemetryDxlTune`) stays zero.
//!
//! Workflow per batch:
//!   1. Clear `TelemetryDxlTune.tx_start_entry_min` (host writes 0).
//!   2. Drive N plain Pings (each Ping ⇒ one CC3 fire in the chip's
//!      reply path, which is the path the const back-dates).
//!   3. Read the field — saturating-min over the batch.
//!   4. Also collect pirate wire stamps `(first − req)` per shot to
//!      report wire-observed RDT vs configured RDT — the operator-side
//!      cross-check that says "K sized correctly ⟹ wire RDT ≈ configured".
//!
//! Final report: distribution of per-batch minima (min / median / p95 /
//! max) and the suggested const = `max(batch_min)` (the floor — every
//! batch saw at least this entry latency, so the next firmware build
//! sizing CCR3 to this value still satisfies `L_entry ≥ K`).

use std::time::Duration;

use anyhow::{Result, bail};
use bench::{
    BOOT_BAUD, RETURN_DELAY_2US_ADDR, Session, SessionArgs, TuneStamps, UNICAST_REPLY_US,
    build_ping, clear_tune, read_ct_u8, read_tune, round_from_stamps,
};
use clap::Parser;
use dxl_protocol::types::Id;

/// Current value of `TX_START_ENTRY_TICKS` in
/// `firmware/ch32/src/measurements.rs`. Reprinted next to the suggestion
/// so the operator can eyeball the delta without grepping the firmware.
const CURRENT_TX_START_ENTRY_TICKS: u16 = 46;

#[derive(Parser, Debug)]
#[command(about = "Sample the chip's TX_START_ENTRY_TICKS floor + wire RDT distribution.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Wire baud to drive at. The chip-side CC3 latency is in the HCLK
    /// domain so the floor is baud-independent in principle; the option
    /// lets you sanity-check across bauds.
    #[arg(short, long, default_value_t = BOOT_BAUD)]
    baud: u32,
    /// Independent batches; each batch starts with a tune-block clear and
    /// drives `--shots` Pings before reading the saturating-min back.
    #[arg(long, default_value_t = 10)]
    batches: u32,
    /// Pings per batch. Each Ping triggers one chip-side CC3 IRQ on the
    /// reply path, contributing one entry-latency sample to the field.
    #[arg(long, default_value_t = 200)]
    shots: u32,
    /// Inter-batch idle, milliseconds.
    #[arg(long, default_value_t = 10)]
    inter_batch_ms: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut session = Session::start(SessionArgs {
        port: args.port,
        target_baud: args.baud,
    })?;
    let id = Id::new(session.id);
    let ticks_per_us = session.pirate.hz_per_us()? as f64;
    let rdt_us = read_ct_u8(&mut session.pirate, id, RETURN_DELAY_2US_ADDR)? as f64 * 2.0;
    let byte_time_us = 10.0 * 1_000_000.0 / args.baud as f64;
    let expected_first_us = rdt_us + byte_time_us;

    println!(
        "pirate: {}   chip id: {}   baud: {}   rdt: {rdt_us:.1} µs   expected first-byte: \
         {expected_first_us:.1} µs ({rdt_us:.0} rdt + {byte_time_us:.2} byte_time)",
        session.pirate.port_path(),
        session.id,
        session.baud,
    );
    println!(
        "current TX_START_ENTRY_TICKS = {CURRENT_TX_START_ENTRY_TICKS}   sampling {} batches × \
         {} pings",
        args.batches, args.shots,
    );
    println!();

    println!(
        "  {:>5}  {:>10}  {:>11}  {:>11}  {:>11}  {:>6}",
        "batch", "tx_min_tk", "wire_med_us", "wire_p95_us", "wire_max_us", "rounds",
    );

    let mut batch_mins: Vec<u16> = Vec::with_capacity(args.batches as usize);
    let mut all_round_us: Vec<f64> = Vec::with_capacity((args.batches * args.shots) as usize);

    for b in 0..args.batches {
        clear_tune(&mut session.pirate, id)?;
        let _ = session.pirate.drain_stamps()?;

        let mut batch_rounds_us: Vec<f64> = Vec::with_capacity(args.shots as usize);
        for _ in 0..args.shots {
            let frame = build_ping(id)?;
            let _ = session.pirate.xfer(&frame, UNICAST_REPLY_US)?;
            let stamps = session.pirate.drain_stamps()?;
            if let Some(r) = round_from_stamps(&stamps) {
                let first_us = r.first.wrapping_sub(r.req) as f64 / ticks_per_us;
                batch_rounds_us.push(first_us);
            }
        }
        all_round_us.extend(batch_rounds_us.iter().copied());

        let TuneStamps {
            tx_start_entry_min, ..
        } = read_tune(&mut session.pirate, id)?;
        if tx_start_entry_min == 0 {
            bail!(
                "tune block returned zero for tx_start_entry_min after {} pings; chip not built \
                 with `--features tuning`?",
                args.shots
            );
        }
        batch_mins.push(tx_start_entry_min);

        let med = quantile_us(&batch_rounds_us, 0.50);
        let p95 = quantile_us(&batch_rounds_us, 0.95);
        let max = batch_rounds_us
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        println!(
            "  {b:>5}  {tx_start_entry_min:>10}  {med:>11.2}  {p95:>11.2}  {max:>11.2}  \
             {:>6}",
            batch_rounds_us.len(),
        );

        std::thread::sleep(Duration::from_millis(args.inter_batch_ms));
    }

    println!();
    print_summary(&batch_mins, &all_round_us, expected_first_us);
    Ok(())
}

fn print_summary(batch_mins: &[u16], wire_us: &[f64], expected_first_us: f64) {
    let mins_f: Vec<f64> = batch_mins.iter().map(|&v| v as f64).collect();
    let bm_min = mins_f.iter().copied().fold(f64::INFINITY, f64::min);
    let bm_med = quantile_us(&mins_f, 0.50);
    let bm_p95 = quantile_us(&mins_f, 0.95);
    let bm_max = mins_f.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    println!("=== batch-min tx_start_entry distribution (TIM2 ticks) ===");
    println!(
        "  min={bm_min:.0}   median={bm_med:.0}   p95={bm_p95:.0}   max={bm_max:.0}   n={}",
        batch_mins.len(),
    );

    // Wire RDT excess vs configured: (first − req) − expected_first_us.
    let excess_us: Vec<f64> = wire_us.iter().map(|us| us - expected_first_us).collect();
    let w_min = excess_us.iter().copied().fold(f64::INFINITY, f64::min);
    let w_med = quantile_us(&excess_us, 0.50);
    let w_p95 = quantile_us(&excess_us, 0.95);
    let w_max = excess_us.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    println!();
    println!("=== wire RDT excess vs configured first-byte (µs) ===");
    println!(
        "  min={w_min:+.2}   median={w_med:+.2}   p95={w_p95:+.2}   max={w_max:+.2}   n={}",
        wire_us.len(),
    );
    if w_min < 0.0 {
        println!(
            "  note: negative excess means a wire bit landed BEFORE the configured RDT — the \
             const is undersized (wire bit encroaches on prior slot's RDT)."
        );
    }

    let suggested = bm_max as u16;
    println!();
    println!("=== suggestion ===");
    println!(
        "  current TX_START_ENTRY_TICKS = {CURRENT_TX_START_ENTRY_TICKS}   measured floor = \
         {suggested}   delta = {:+}",
        suggested as i32 - CURRENT_TX_START_ENTRY_TICKS as i32,
    );
    println!(
        "  paste into firmware/ch32/src/measurements.rs:\n      pub const \
         TX_START_ENTRY_TICKS: u16 = {suggested};"
    );
}

/// Linear-interpolated quantile over `xs`. Returns NaN on empty input.
fn quantile_us(xs: &[f64], q: f64) -> f64 {
    if xs.is_empty() {
        return f64::NAN;
    }
    let mut sorted: Vec<f64> = xs.iter().copied().filter(|v| v.is_finite()).collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = sorted.len();
    if n == 0 {
        return f64::NAN;
    }
    let pos = q * (n as f64 - 1.0);
    let lo = pos.floor() as usize;
    let hi = pos.ceil() as usize;
    let frac = pos - lo as f64;
    sorted[lo] * (1.0 - frac) + sorted[hi] * frac
}
