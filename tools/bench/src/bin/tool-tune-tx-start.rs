//! Measure the chip's `TX_START_ENTRY_TICKS` floor — the back-date the
//! `arm_tim2` path applies to CCR3 so the wire-bit lands at or after the
//! scheduled deadline.
//!
//! Two signals captured per run, both required because they cover
//! complementary slices of the CC3→wire-bit path:
//!
//! - **Chip-side stamp** `TelemetryDxlTune.tx_start_entry_min`: captured
//!   at the very top of `on_tim2_cc3`. Covers PFIC entry + `qingke-rt`
//!   context save. Diagnostic only — `dma::enable` + DMA AHB arbitration
//!   + USART DR write happen AFTER the stamp and are NOT in the number.
//! - **Wire-side excess** `(first − req) − configured_first_byte_us`:
//!   pirate's stable-clock measurement of when the chip's first wire bit
//!   lands vs the deadline implied by `RDT`. This is the ground truth
//!   for the const — it covers the full path CC3-match → wire-bit. The
//!   smallest wire excess across all shots is the slack we can shrink
//!   into `TX_START_ENTRY_TICKS` while still keeping every wire bit at
//!   or after deadline.
//!
//! Suggestion math (all ticks HCLK = 48 MHz):
//! `K_recommended = K_current + min(wire_excess_hclk_ticks) - safety_margin`.
//! Pasting `K_recommended` makes the worst-case wire bit land
//! `safety_margin` ticks AFTER deadline; pasting a larger K would push
//! the worst-case wire bit BEFORE deadline (collides with prior slot's
//! RDT).
//!
//! Requires the chip to be built with `--features tuning`; otherwise the
//! chip-side stamp surface stays zero (wire excess still works).

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

/// V006 HCLK ticks per µs — TIM2 / chip-side `TX_START_ENTRY_TICKS` are
/// in this domain. Pirate ticks are at a different rate (18 MHz SysTick)
/// and must be converted to HCLK before adding to K.
const HCLK_TICKS_PER_US: f64 = 48.0;

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
    /// Safety margin to keep between worst-case wire bit and deadline,
    /// in HCLK ticks. The recommended const is `K_max_safe -
    /// safety_margin`. Default 3 ticks (~62 ns at 48 MHz) — small
    /// enough that K stays close to the optimum, large enough to
    /// absorb a sample we didn't observe.
    #[arg(long, default_value_t = 3)]
    safety_margin: u16,
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
    print_summary(
        &batch_mins,
        &all_round_us,
        expected_first_us,
        args.safety_margin,
    );
    Ok(())
}

fn print_summary(batch_mins: &[u16], wire_us: &[f64], expected_first_us: f64, safety_margin: u16) {
    let mins_f: Vec<f64> = batch_mins.iter().map(|&v| v as f64).collect();
    let bm_min = mins_f.iter().copied().fold(f64::INFINITY, f64::min);
    let bm_med = quantile_us(&mins_f, 0.50);
    let bm_p95 = quantile_us(&mins_f, 0.95);
    let bm_max = mins_f.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    println!("=== chip-side tx_start_entry_min distribution (TIM2 ticks, diagnostic) ===");
    println!(
        "  min={bm_min:.0}   median={bm_med:.0}   p95={bm_p95:.0}   max={bm_max:.0}   n={}",
        batch_mins.len(),
    );
    println!(
        "  covers PFIC entry + qingke-rt save only; dma::enable + DMA arb + USART DR write \
         happen AFTER the stamp."
    );

    // Wire RDT excess vs configured: (first − req) − expected_first_us.
    // This is the ground-truth signal: end-to-end CC3-match → wire-bit
    // latency relative to the deadline implied by RDT.
    let excess_us: Vec<f64> = wire_us.iter().map(|us| us - expected_first_us).collect();
    let w_min = excess_us.iter().copied().fold(f64::INFINITY, f64::min);
    let w_med = quantile_us(&excess_us, 0.50);
    let w_p95 = quantile_us(&excess_us, 0.95);
    let w_max = excess_us.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    println!();
    println!("=== wire excess (first-byte − configured deadline) — ground truth ===");
    println!(
        "  min={w_min:+.2} µs   median={w_med:+.2} µs   p95={w_p95:+.2} µs   max={w_max:+.2} µs   \
         n={}",
        wire_us.len(),
    );
    if w_min < 0.0 {
        println!(
            "  WARN: negative excess — chip's wire bit landed BEFORE the configured deadline. \
             TX_START_ENTRY_TICKS is too LARGE; reduce it by at least {} HCLK ticks before \
             reflashing.",
            (-w_min * HCLK_TICKS_PER_US).ceil() as i64,
        );
    }

    // Suggestion math, fully wire-derived. All ticks here are HCLK
    // (48 MHz, V006) — the chip's K and TIM2 domain are HCLK. Wire
    // excess is in µs; convert with HCLK_TICKS_PER_US, never with the
    // pirate's slower SysTick rate (those are separate domains).
    //
    //   K_max_safe = K_current + min(wire_excess_hclk_ticks)
    //   K_recommended = K_max_safe − safety_margin
    //
    // K LARGER → CCR3 earlier → wire bit earlier (risks landing before
    // deadline — collides with prior slot). K SMALLER → wire bit later
    // (safe; wastes a few HCLK ticks of slot time). `K_max_safe` is the
    // tightest legal K under the observed distribution.
    let min_excess_hclk = (w_min * HCLK_TICKS_PER_US).floor() as i32;
    let k_max_safe_signed = CURRENT_TX_START_ENTRY_TICKS as i32 + min_excess_hclk;
    let k_recommended_signed = k_max_safe_signed - safety_margin as i32;
    println!();
    println!("=== suggestion ===");
    println!(
        "  current K = {CURRENT_TX_START_ENTRY_TICKS}   min_wire_excess = {min_excess_hclk} HCLK \
         ticks   safety_margin = {safety_margin}"
    );
    println!(
        "  K_max_safe = current + min_excess = {k_max_safe_signed}   (largest K where worst-case \
         wire bit still ≥ deadline)"
    );
    if k_recommended_signed <= 0 || k_recommended_signed > u16::MAX as i32 {
        println!(
            "  ERROR: K_recommended = {k_recommended_signed} out of range. Re-run with \
             different --safety-margin."
        );
        return;
    }
    let k_recommended = k_recommended_signed as u16;
    let delta = k_recommended as i32 - CURRENT_TX_START_ENTRY_TICKS as i32;
    println!(
        "  K_recommended = K_max_safe − safety_margin = {k_recommended}   delta_from_current = \
         {delta:+}"
    );
    println!(
        "  paste into firmware/ch32/src/measurements.rs:\n      pub const \
         TX_START_ENTRY_TICKS: u16 = {k_recommended};"
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
