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
//! - **Wire-side excess** `(reply_first − req_end) − configured_first_byte_us`:
//!   pirate's stable-clock measurement of when the chip's first wire bit
//!   lands vs the deadline implied by `RDT`. This is the ground truth
//!   for the const — it covers the full path CC3-match → wire-bit. The
//!   0.1th-percentile (p0.1) wire excess is the robust slack we can shrink
//!   into `TX_START_ENTRY_TICKS` while still keeping 99.9% of wire bits at
//!   or after deadline. 99.9% (not 99%) because Fast Sync chains snoop
//!   each predecessor's reply — a single CRC-rejected frame collapses the
//!   chain tail, so a 1-in-1000 failure rate at scale is unacceptable.
//!   `min` is reported for diagnostic context but is a heavy-tailed
//!   extreme-order statistic — a single noise edge picked up by the
//!   pirate during the idle gap swings `min` by 1+ µs across runs while
//!   p0.1 stays within ~1 chip-HCLK tick at 20k samples (20-sample tail).
//!
//! Output (all ticks HCLK = 48 MHz): per-batch distribution plus an
//! across-batch summary with the recommended *delta* to apply to the
//! current `TX_START_ENTRY_TICKS` in `firmware/ch32/src/measurements.rs`.
//! Operator adds the delta to the current value. Delta sign convention:
//! positive → enlarge K (wire bit was landing too late); negative →
//! shrink K (wire bit was landing too early).
//!
//! Requires the chip to be built with `--features tuning`; otherwise the
//! chip-side stamp surface stays zero (wire excess still works).

use std::time::Duration;

use anyhow::{Result, bail};
use bench::{Bus, BusArgs, DEFAULT_IDLE_US, RETURN_DELAY_2US_ADDR, TuneStamps, build_ping};
use clap::Parser;
use dxl_protocol::types::Id;

/// V006 HCLK ticks per µs — TIM2 / chip-side `TX_START_ENTRY_TICKS` are
/// in this domain. Pirate ticks are at a different rate (144 MHz HCLK)
/// and must be converted to HCLK before adding to K.
const HCLK_TICKS_PER_US: f64 = 48.0;

/// Cliff-detection threshold for `p95 − median` wire-excess spread, in µs.
/// Healthy chip-side post-stamp latency produces a tight distribution
/// (observed spread ~0.3-0.5 µs at K below the floor); above the cliff
/// the upper tail jumps ~3 µs because CC3 scheduling falls into the
/// wrap-into-past / fire-immediately fallback path (one byte_time of
/// delay penalty at 3M baud). 1.5 µs sits 3-5× above healthy noise and
/// well below the smallest observed cliff jump, so it triggers cleanly.
const CLIFF_SPREAD_THRESHOLD_US: f64 = 1.5;

#[derive(Parser, Debug)]
#[command(about = "Sample the chip's TX_START_ENTRY_TICKS floor + wire RDT distribution.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Wire baud to drive at. Default 3M: worst-case slot-timing margin
    /// (byte_time = 3.33 µs) where TX_START_ENTRY_TICKS most needs to be
    /// pinned; chip-side CC3 latency is HCLK-domain so the floor is
    /// baud-independent in principle, but the wire-excess deep tail
    /// matters most here. Override to sanity-check across bauds.
    #[arg(short, long, default_value_t = 3_000_000)]
    baud: u32,
    /// Independent batches; each batch starts with a tune-block clear and
    /// drives `--shots` Pings before reading the saturating-min back.
    #[arg(long, default_value_t = 20)]
    batches: u32,
    /// Pings per batch. Each Ping triggers one chip-side CC3 IRQ on the
    /// reply path, contributing one entry-latency sample to the field.
    /// 1000 is large enough that every batch reliably samples the rare
    /// PFIC-entry low-tail (~0.5% of pings) — smaller batches expose a
    /// min-of-batch sampling artifact that masquerades as bimodal.
    #[arg(long, default_value_t = 1000)]
    shots: u32,
    /// Inter-batch idle, milliseconds.
    #[arg(long, default_value_t = 10)]
    inter_batch_ms: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut bus = Bus::start(BusArgs {
        port: args.port,
        target_baud: args.baud,
    })?;
    let id = Id::new(bus.id());
    let ticks_per_us = bus.hz_per_us()? as f64;
    let rdt_us = bus.read_ct_u8(id, RETURN_DELAY_2US_ADDR)? as f64 * 2.0;
    let byte_time_us = 10.0 * 1_000_000.0 / args.baud as f64;
    let expected_first_us = rdt_us + byte_time_us;

    println!(
        "pirate: {}   chip id: {}   baud: {}   rdt: {rdt_us:.1} µs   expected first-byte: \
         {expected_first_us:.1} µs ({rdt_us:.0} rdt + {byte_time_us:.2} byte_time)",
        bus.port_path(),
        bus.id(),
        bus.baud(),
    );
    println!(
        "sampling {} batches × \
         {} pings",
        args.batches, args.shots,
    );
    println!();

    println!(
        "  {:>5}  {:>10}  {:>11}  {:>11}  {:>11}  {:>11}  {:>6}",
        "batch", "tx_min_tk", "wire_med_us", "wire_p95_us", "wire_p99_us", "wire_max_us", "rounds",
    );

    let mut batch_mins: Vec<u16> = Vec::with_capacity(args.batches as usize);
    let mut all_round_us: Vec<f64> = Vec::with_capacity((args.batches * args.shots) as usize);

    for b in 0..args.batches {
        bus.clear_tune(id)?;

        let mut batch_rounds_us: Vec<f64> = Vec::with_capacity(args.shots as usize);
        for _ in 0..args.shots {
            let frame = build_ping(id)?;
            if let Some(t) = bus.xfer_round(&frame, DEFAULT_IDLE_US)? {
                let first_us = t.reply_first.wrapping_sub(t.req_end) as f64 / ticks_per_us;
                batch_rounds_us.push(first_us);
            }
        }
        all_round_us.extend(batch_rounds_us.iter().copied());

        let TuneStamps {
            tx_start_entry_min, ..
        } = bus.read_tune(id)?;
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
        let p99 = quantile_us(&batch_rounds_us, 0.99);
        let max = batch_rounds_us
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        println!(
            "  {b:>5}  {tx_start_entry_min:>10}  {med:>11.2}  {p95:>11.2}  {p99:>11.2}  {max:>11.2}  \
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

    println!("=== chip-side tx_start_entry_min distribution (TIM2 ticks, diagnostic) ===");
    println!(
        "  min={bm_min:.0}   median={bm_med:.0}   p95={bm_p95:.0}   max={bm_max:.0}   n={}",
        batch_mins.len(),
    );
    println!(
        "  covers PFIC entry + qingke-rt save only; dma::enable + DMA arb + USART DR write \
         happen AFTER the stamp."
    );

    let excess_us: Vec<f64> = wire_us.iter().map(|us| us - expected_first_us).collect();
    let w_min = excess_us.iter().copied().fold(f64::INFINITY, f64::min);
    let w_p0_1 = quantile_us(&excess_us, 0.001);
    let w_med = quantile_us(&excess_us, 0.50);
    let w_p95 = quantile_us(&excess_us, 0.95);
    let w_p99 = quantile_us(&excess_us, 0.99);
    let w_p99_9 = quantile_us(&excess_us, 0.999);
    let w_p99_99 = quantile_us(&excess_us, 0.9999);
    let w_max = excess_us.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    println!();
    println!("=== wire excess (first-byte − configured deadline) — ground truth ===");
    println!(
        "  min={w_min:+.2} µs   p0.1={w_p0_1:+.2} µs   median={w_med:+.2} µs   p95={w_p95:+.2} µs   max={w_max:+.2} µs   \
         n={}",
        wire_us.len(),
    );
    println!(
        "  upper tail:   p99={w_p99:+.2} µs   p99.9={w_p99_9:+.2} µs   p99.99={w_p99_99:+.2} µs",
    );
    let spread_us = w_p95 - w_med;
    let cliff_detected = spread_us > CLIFF_SPREAD_THRESHOLD_US;
    let p0_1_excess_hclk = (w_p0_1 * HCLK_TICKS_PER_US).floor() as i32;
    println!();
    if cliff_detected {
        println!("=== ⚠  K AT/PAST FLOOR — DO NOT BUMP ===");
        println!(
            "  p95 ({w_p95:+.2} µs) is {spread_us:.2} µs above median ({w_med:+.2} µs) — \
             upper tail has exploded."
        );
        println!(
            "  Healthy spread is ~0.3-0.5 µs; >{CLIFF_SPREAD_THRESHOLD_US:.1} µs means CC3 \
             scheduling is hitting wrap-into-past / fire-immediately fallback."
        );
        println!(
            "  At this regime the chip's USART/DMA setup races with CC3 IRQ — first byte \
             of the reply gets garbled on the wire, breaking snoop CRC."
        );
        println!(
            "  Roll TX_START_ENTRY_TICKS back -3 (or more for margin) and re-measure; ignore \
             any p0.1-based suggestion this run."
        );
        return;
    }
    println!("=== suggestion (99.9% wire-excess safety bound) ===");
    println!(
        "  shift TX_START_ENTRY_TICKS by {p0_1_excess_hclk:+} HCLK ticks (new = current{p0_1_excess_hclk:+})"
    );
    if p0_1_excess_hclk < 0 {
        println!(
            "  K is too LARGE — wire bit lands BEFORE deadline at the p0.1 tail; reduce K to push wire bit later."
        );
    } else if p0_1_excess_hclk > 0 {
        println!(
            "  K is too SMALL — even the p0.1 tail lands after deadline; enlarge K to tighten slot."
        );
    } else {
        println!("  K is at the p0.1 floor (0.1% of pings land at or just before deadline).");
    }
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
