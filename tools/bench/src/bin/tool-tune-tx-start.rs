//! Measure the chip's `TX_KICKOFF_FLOOR_TICKS` floor — the back-date the
//! `arm_tim2` path applies to CCR3 so the wire-bit lands slightly after
//! CC2 activates TX_EN.
//!
//! Ground-truth signal: **wire-side excess**
//! `(reply_first − req_end) − configured_first_byte_us` — the pirate's
//! stable-clock measurement of when the chip's first wire bit lands vs
//! the deadline implied by `RDT`. Covers the full path
//! CC3-match → wire-bit. The 0.1th-percentile (p0.1) wire excess is the
//! robust slack we can shrink into `TX_KICKOFF_FLOOR_TICKS` while still
//! keeping 99.9% of wire bits *after* CC2 (TX_EN active). 99.9% (not
//! 99%) because Fast Sync chains snoop each predecessor's reply — a
//! single first-byte drop collapses the chain tail, so a 1-in-1000
//! failure rate at scale is unacceptable. `min` is reported for
//! diagnostic context but is a heavy-tailed extreme-order statistic —
//! a single noise edge picked up by the pirate during the idle gap
//! swings `min` by 1+ µs across runs while p0.1 stays within ~1
//! chip-HCLK tick at 20k samples (20-sample tail).
//!
//! Complementary failure signal: **K-too-aggressive drop** — a Status
//! reply arriving with its leading `0xFF` missing (see
//! [`bench::DroppedLeadingFf`]). Chip's first bit-on-wire landed
//! *before* CC2 activated TX_EN, so the transceiver + pirate lost the
//! first byte in the direction-switch window. Any drop = K reduction
//! required, independent of the wire-excess distribution.
//!
//! Output (all ticks HCLK = 48 MHz): per-batch distribution plus an
//! across-batch summary with the recommended *delta* to apply to the
//! current `TX_KICKOFF_FLOOR_TICKS` in `firmware/ch32/src/measurements.rs`.
//! Operator adds the delta to the current value. Delta sign convention:
//! positive → enlarge K (wire bit was landing too late); negative →
//! shrink K (wire bit was landing too early).

use std::time::Duration;

use anyhow::Result;
use bench::{
    Bus, BusArgs, DroppedLeadingFf, RETURN_DELAY_2US_ADDR, build_ping, retry_on_drop,
};
use clap::Parser;
use dxl_protocol::types::Id;

/// Silence window per Ping xfer, µs. Tighter than the shared
/// `DEFAULT_IDLE_US` (10 ms) because a Ping reply is one short burst
/// (~11 bytes at any baud), reliably delivered in one USB-CDC poll —
/// no chained-reply or slow-slave gap to absorb here. 1 ms sits comfortably
/// above the USB-CDC bbatch RTT (~1 ms per empty poll) while cutting the
/// 20-batch × 1000-shot run from ~3 minutes to ~20 seconds.
const PING_REPLY_IDLE_US: u32 = 1_000;

/// V006 HCLK ticks per µs — TIM2 / chip-side `TX_KICKOFF_FLOOR_TICKS` are
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

/// Median wire-excess above which the chip is treated as using the
/// `software_fire` fallback path rather than `arm_tim2`'s hardware-OC
/// path. Empirical: at RDT below the arm_tim2 floor, `software_fire` adds
/// ~110-150 µs of DMA + USART setup latency on top of the configured
/// deadline; the hardware-OC path lands within ~1 µs. 50 µs sits well
/// above the arm_tim2 tail and well below the software_fire floor, so it
/// triggers cleanly.
const SOFTWARE_FIRE_EXCESS_THRESHOLD_US: f64 = 50.0;

#[derive(Parser, Debug)]
#[command(about = "Sample the chip's TX_KICKOFF_FLOOR_TICKS floor + wire RDT distribution.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Wire baud to drive at. Default 3M: worst-case slot-timing margin
    /// (byte_time = 3.33 µs) where TX_KICKOFF_FLOOR_TICKS most needs to be
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
    /// Optionally rewrite the chip's `RETURN_DELAY_2US` CT register before
    /// sampling. Value in µs, rounded to the nearest 2 µs step. Useful for
    /// A/B-testing whether the wire-excess tail is CPU-cost driven (extra
    /// slack should collapse the tail) or false-positive-anchor driven
    /// (extra slack won't help).
    #[arg(long)]
    rdt_us: Option<u16>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut drops: u32 = 0;
    let result = run(&args, &mut drops);
    let terminal_drop = result
        .as_ref()
        .err()
        .is_some_and(|e| e.downcast_ref::<DroppedLeadingFf>().is_some());
    if drops > 0 || terminal_drop {
        print_k_too_aggressive_advice(drops);
    }
    result
}

fn print_k_too_aggressive_advice(drops: u32) {
    eprintln!();
    eprintln!(
        "=== ⚠  TX_KICKOFF_FLOOR_TICKS TOO AGGRESSIVE — {drops} K-drop event(s) during run ==="
    );
    eprintln!(
        "  One or more chip Status replies dropped their leading 0xFF byte: the chip's first"
    );
    eprintln!(
        "  bit-on-wire landed BEFORE CC2 activated TX_EN, so the transceiver + pirate lost the"
    );
    eprintln!("  first byte in the direction-switch window. Reduce TX_KICKOFF_FLOOR_TICKS in");
    eprintln!(
        "  firmware/ch32/src/measurements.rs by ~5-10 HCLK ticks and re-flash before re-running."
    );
}

fn run(args: &Args, drops: &mut u32) -> Result<()> {
    let port = args.port.clone();
    let baud = args.baud;
    let mut bus = retry_on_drop(2, drops, || {
        Bus::start(BusArgs {
            port: port.clone(),
            target_baud: baud,
        })
    })?;
    let id = Id::new(bus.id());
    let ticks_per_us = bus.hz_per_us()? as f64;
    if let Some(us) = args.rdt_us {
        let steps = (us / 2).min(255) as u8;
        retry_on_drop(2, drops, || {
            bus.write_register(id, RETURN_DELAY_2US_ADDR, &[steps])
        })?;
    }
    let rdt_us =
        retry_on_drop(2, drops, || bus.read_ct_u8(id, RETURN_DELAY_2US_ADDR))? as f64 * 2.0;
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
        "  {:>5}  {:>11}  {:>11}  {:>11}  {:>11}  {:>6}",
        "batch", "wire_med_us", "wire_p95_us", "wire_p99_us", "wire_max_us", "rounds",
    );

    let mut all_round_us: Vec<f64> = Vec::with_capacity((args.batches * args.shots) as usize);
    retry_on_drop(2, drops, || bus.clear_counters(id))?;

    let mut batches_with_wire: u32 = 0;
    for b in 0..args.batches {
        let mut batch_rounds_us: Vec<f64> = Vec::with_capacity(args.shots as usize);
        for _ in 0..args.shots {
            let frame = build_ping(id)?;
            if let Some(t) = bus.xfer_round(&frame, PING_REPLY_IDLE_US)? {
                let first_us = t.reply_first.wrapping_sub(t.req_end) as f64 / ticks_per_us;
                batch_rounds_us.push(first_us);
            }
        }
        all_round_us.extend(batch_rounds_us.iter().copied());
        let has_wire = !batch_rounds_us.is_empty();
        if has_wire {
            batches_with_wire += 1;
        }

        print!("  {b:>5}  ");
        if has_wire {
            let med = quantile_us(&batch_rounds_us, 0.50);
            let p95 = quantile_us(&batch_rounds_us, 0.95);
            let p99 = quantile_us(&batch_rounds_us, 0.99);
            let max = batch_rounds_us
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);
            print!("{med:>11.2}  {p95:>11.2}  {p99:>11.2}  {max:>11.2}  ");
        } else {
            print!("{:>11}  {:>11}  {:>11}  {:>11}  ", "-", "-", "-", "-");
        }
        println!("{:>6}", batch_rounds_us.len());

        std::thread::sleep(Duration::from_millis(args.inter_batch_ms));
    }

    println!();

    if batches_with_wire == 0 {
        println!(
            "=== ⚠  chip silent — no wire replies in {} batches ===",
            args.batches,
        );
        println!(
            "  Possible causes: RDT ({rdt_us:.0} µs) below the chip's turnaround floor \
             (deadline math wraps into past + fire-immediately can't recover before host \
             echo drains), transport wedge, or servo powered off. Raise `--rdt-us` and retry."
        );
        return Ok(());
    }

    print_summary(&all_round_us, expected_first_us);
    Ok(())
}

fn print_summary(wire_us: &[f64], expected_first_us: f64) {
    if wire_us.is_empty() {
        return;
    }

    let excess_us: Vec<f64> = wire_us.iter().map(|us| us - expected_first_us).collect();
    let w_min = excess_us.iter().copied().fold(f64::INFINITY, f64::min);
    let w_p0_1 = quantile_us(&excess_us, 0.001);
    let w_med = quantile_us(&excess_us, 0.50);
    let w_p95 = quantile_us(&excess_us, 0.95);
    let w_p99 = quantile_us(&excess_us, 0.99);
    let w_p99_9 = quantile_us(&excess_us, 0.999);
    let w_p99_99 = quantile_us(&excess_us, 0.9999);
    let w_max = excess_us.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    if w_med > SOFTWARE_FIRE_EXCESS_THRESHOLD_US {
        println!("=== ⚠  chip replying via fire-immediately fallback ===");
        println!(
            "  wire_med excess is +{w_med:.1} µs — above the {SOFTWARE_FIRE_EXCESS_THRESHOLD_US:.0} µs threshold. \
             RDT is below the chip's arm_tim2 floor: the scheduler skipped CC3 arming and \
             started TX inline. K-tuning suggestion suppressed (would be garbage); raise \
             `--rdt-us` to exercise arm_tim2 and re-run."
        );
        println!();
    }

    println!("=== wire excess (first-byte − configured deadline) — ground truth ===");
    println!(
        "  min={w_min:+.2} µs   p0.1={w_p0_1:+.2} µs   median={w_med:+.2} µs   p95={w_p95:+.2} µs   max={w_max:+.2} µs   \
         n={}",
        wire_us.len(),
    );
    println!(
        "  upper tail:   p99={w_p99:+.2} µs   p99.9={w_p99_9:+.2} µs   p99.99={w_p99_99:+.2} µs",
    );
    if w_med > SOFTWARE_FIRE_EXCESS_THRESHOLD_US {
        return;
    }
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
            "  Roll TX_KICKOFF_FLOOR_TICKS back -3 (or more for margin) and re-measure; ignore \
             any p0.1-based suggestion this run."
        );
        return;
    }
    println!("=== suggestion (99.9% wire-excess safety bound) ===");
    println!(
        "  shift TX_KICKOFF_FLOOR_TICKS by {p0_1_excess_hclk:+} HCLK ticks (new = current{p0_1_excess_hclk:+})"
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
