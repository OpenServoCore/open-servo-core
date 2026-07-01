//! Pirate self-tune. Loopback-only — no servo on the bus.
//!
//! Three stages, run in order, halting at first failure:
//!
//! 1. **classifier** — fixed-density 16-byte payload via `SEND`, no
//!    inter-packet gap. Validates byte values, no `COUNT_UNDER`, inter-byte
//!    spacing within tolerance, and every stamp anchor matches a real IC
//!    ring entry.
//! 2. **stress** — slow-baud (115 200) random-burst soak. A seeded LCG
//!    generates 32 bursts of 1..=32 random bytes with random 0..=20 ms
//!    inter-burst gaps. Variable byte values mean variable falling-edge
//!    counts per byte, so the RX byte-ring index drifts independently
//!    of the IC edge-ring index — any pair-up bug shows up as a byte
//!    mismatch. Validates per-burst: byte count, byte values, strictly
//!    monotone stamp ticks, and `falling_total` delta == sum of expected
//!    edges (computed from the LSB-first UART frame of each byte).
//! 3. **tx-comp** — scheduled-`SEND at=` vs stamped wire edge, swept
//!    across baud. Residual = `stamp[0].tick − LAST?`. Regressed `a + b·brr`
//!    with inverse-variance weighting (weight ∝ 1/brr², since per-shot
//!    phase noise is U[0, brr] so median variance ∝ brr²); the slope is
//!    then stacked onto the pirate's current `(pipe, bit_q4)` (read via
//!    `COMP?`) to recommend new tunables:
//!    `pipe_new = pipe_cur + a`, `bit_q4_new = bit_q4_cur + 16·b`.
//!    Per-baud the bench reports `min/median/max/span` (raw spread of N
//!    shots), `span/brr` (normalized — pure UART DR→TSR phase noise is
//!    U[0, brr] so this should be ≈ 0.88 at the floor; >>1 means another
//!    jitter source is bleeding in), and `err/bit = median/brr` (a
//!    constant `err/bit` across rows means the model is off by one
//!    scalar — move `bit_q4`; a varying `err/bit` means the
//!    linear-in-brr model itself is wrong). Pass `--write` to push the
//!    recommendation back via `COMP`.

use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use bench::{BStamp, Bus, IcSnapshot, TxComp};
use clap::{Parser, ValueEnum};

#[derive(Copy, Clone, Debug, ValueEnum, PartialEq, Eq)]
enum Stage {
    Classifier,
    Stress,
    TxComp,
    All,
}

#[derive(Parser, Debug)]
#[command(
    about = "Pirate self-tune (loopback only): classifier + multi-wrap stress + TX_COMP cal."
)]
struct Args {
    /// Pirate USB-CDC path. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Bauds for classifier + tx-comp sweep.
    /// Default: 57600, 115200, 1M, 2M, 3M.
    #[arg(short, long, value_delimiter = ',')]
    bauds: Option<Vec<u32>>,
    /// Run only one stage. Default: all (halts at first failure).
    #[arg(long, value_enum, default_value_t = Stage::All)]
    stage: Stage,
    /// Inter-byte spacing tolerance (ticks) for the classifier.
    /// Default: bit_ticks / 4.
    #[arg(long)]
    tolerance: Option<u32>,
    /// Shots per baud for tx-comp regression. Each shot draws an
    /// independent φ ~ U[0, brr] from the USART DR→TSR bit-clock sync,
    /// so median SE shrinks ∝ 1/√n. 64 puts slow-baud median SE
    /// (~brr/16) inside ±150 ticks at brr=2500.
    #[arg(long, default_value_t = 64)]
    tx_comp_shots: u32,
    /// After stage 3, write the regression-recommended TX-comp values
    /// (`pipe`, `bit_q4`) back to the pirate via `COMP`. No-op if the
    /// recommendation matches what the pirate already has.
    #[arg(long)]
    write: bool,
    /// Stage 2 RNG seed (decimal or 0x-prefixed hex). Reproduce a failing
    /// run by re-running with the seed printed at stage start.
    #[arg(long, value_parser = parse_seed)]
    seed: Option<u64>,
}

fn parse_seed(s: &str) -> std::result::Result<u64, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse::<u64>().map_err(|e| e.to_string())
    }
}

/// Loopback payload (16 bytes). Varied edge density across the byte
/// space; no `FF FF FD 00` substring so no servo on the bus reacts.
const LOOPBACK_PAYLOAD: [u8; 16] = [
    0x55, 0xAA, 0x55, 0xAA, 0xF0, 0x0F, 0x33, 0xCC, 0x00, 0xFE, 0x01, 0x80, 0xA5, 0x5A, 0xC3, 0x3C,
];

/// Stress probe baud — slow enough that each burst fits well under the
/// 200 ms drain timeout while still exercising DMA HT/TC/IDLE many times
/// over the run.
const STRESS_BAUD: u32 = 115_200;

/// Number of random bursts in the stress soak.
const STRESS_BURSTS: usize = 32;

/// Per-burst byte-count range. Spans both sub-half-ring (no HT trip
/// within the burst) and a regime where back-to-back bursts cross HT/TC
/// boundaries on both the RX and IC rings.
const STRESS_MIN_BURST: u32 = 1;
const STRESS_MAX_BURST: u32 = 32;

/// Inter-burst host-side gap range (µs). 0 → back-to-back, upper end ≫
/// TIM2 wrap (~455 µs at 144 MHz) so HT/TC/IDLE fire at every relative
/// offset across the soak.
const STRESS_MAX_GAP_US: u32 = 20_000;

/// Default RNG seed (override with `--seed`). Fixed so CI runs are
/// reproducible; the actual seed used is printed at stage start.
const STRESS_DEFAULT_SEED: u64 = 0xC0DE_1234_DEAD_BEEF;

fn main() -> Result<()> {
    let args = Args::parse();
    let bauds: Vec<u32> = args
        .bauds
        .clone()
        .unwrap_or_else(|| vec![57_600, 115_200, 1_000_000, 2_000_000, 3_000_000]);

    let mut bus = Bus::start_pirate_only(args.port)?;
    println!("pirate: {}   (loopback-only)", bus.port_path());

    let run_all = args.stage == Stage::All;
    if run_all || args.stage == Stage::Classifier {
        stage_classifier(&mut bus, &bauds, args.tolerance)?;
    }
    if run_all || args.stage == Stage::Stress {
        stage_stress(&mut bus, args.seed.unwrap_or(STRESS_DEFAULT_SEED))?;
    }
    if run_all || args.stage == Stage::TxComp {
        stage_tx_comp(&mut bus, &bauds, args.tx_comp_shots, args.write)?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Stage 1: classifier
// ---------------------------------------------------------------------------

fn stage_classifier(bus: &mut Bus, bauds: &[u32], tol: Option<u32>) -> Result<()> {
    println!();
    println!("stage 1: classifier — 16-byte loopback echo (RX byte-spacing accuracy)");
    println!("  gates: stamp count, byte values, no PLL miss, inter-byte spacing ≤ tol,");
    println!("  every stamp anchored on a real IC edge.");
    println!();
    println!(
        "  {:>8}  {:>5}  {:>5}  {:>8}  result",
        "baud", "brr", "dev", "dev%bit"
    );
    let mut any_fail = false;
    for &baud in bauds {
        bus.pirate_set_baud(baud)?;
        match run_classifier_probe(bus, baud, tol) {
            Ok(line) => println!("  {baud:>8}  {line}  PASS"),
            Err(e) => {
                println!("  {baud:>8}  FAIL: {e}");
                any_fail = true;
            }
        }
    }
    if any_fail {
        bail!("stage 1 (classifier) failed");
    }
    Ok(())
}

fn run_classifier_probe(bus: &mut Bus, baud: u32, tol_arg: Option<u32>) -> Result<String> {
    bus.pirate_reset()?;
    std::thread::sleep(Duration::from_millis(20));
    while !bus.pirate_bbatch(255)?.is_empty() {}

    bus.pirate_send_now(&LOOPBACK_PAYLOAD)?;
    // Wait for the wire to finish + IDLE to assert (10 bit-times per byte +
    // one char-time idle gap) before draining; the walker fires once on
    // IDLE and at slow baud HT/TC won't trigger before then.
    let tx_us = LOOPBACK_PAYLOAD.len() as u32 * 10_000_000 / baud + 500;
    std::thread::sleep(Duration::from_micros(tx_us as u64));
    let stamps = drain_stamps_until(bus, LOOPBACK_PAYLOAD.len(), Duration::from_millis(200))?;
    let snap = bus.ic_snapshot()?;

    if stamps.len() != LOOPBACK_PAYLOAD.len() {
        bail!(
            "got {} stamps, expected {} (echo incomplete or unexpected noise)",
            stamps.len(),
            LOOPBACK_PAYLOAD.len()
        );
    }
    validate_byte_match(&stamps, &LOOPBACK_PAYLOAD)?;
    validate_no_count_under(&stamps)?;
    let byte_period = snap.bit_ticks.wrapping_mul(10);
    let tol = tol_arg.unwrap_or(snap.bit_ticks / 4);
    let max_dev = validate_spacing(&stamps, byte_period, tol, 0..stamps.len())?;
    validate_ic_correspondence(&stamps, &snap)?;

    // dev as % of one bit-time: 100% means the byte boundary slid a
    // full bit. PASS already gates dev ≤ tol (= brr/4 by default), so
    // values here are bounded by ~25% in the happy path.
    let _ = tol; // gating only; not shown
    let dev_pct_bit = max_dev as f64 / snap.bit_ticks as f64 * 100.0;
    Ok(format!(
        "{brr:>5}  {dev:>5}  {dev_pct_bit:>+7.2}%",
        brr = snap.bit_ticks,
        dev = max_dev,
    ))
}

// ---------------------------------------------------------------------------
// Stage 2: random-burst soak
// ---------------------------------------------------------------------------

fn stage_stress(bus: &mut Bus, seed: u64) -> Result<()> {
    println!();
    println!("stage 2: stress — random bursts (baud={STRESS_BAUD}, seed=0x{seed:016X})");
    println!("  gates per burst: byte count, byte values, strictly-monotone ticks,");
    println!("  falling_total delta == expected edges (computed from byte payload).");

    bus.pirate_set_baud(STRESS_BAUD)?;
    bus.pirate_reset()?;
    std::thread::sleep(Duration::from_millis(20));
    while !bus.pirate_bbatch(255)?.is_empty() {}

    let mut rng = seed;
    let mut any_fail = false;
    let mut total_bytes = 0usize;
    let mut total_edges = 0u32;
    let mut burst = Vec::with_capacity(STRESS_MAX_BURST as usize);

    for i in 0..STRESS_BURSTS {
        let len = lcg_in_range(&mut rng, STRESS_MIN_BURST, STRESS_MAX_BURST) as usize;
        burst.clear();
        burst.extend((0..len).map(|_| lcg_next(&mut rng) as u8));
        let gap_us = lcg_in_range(&mut rng, 0, STRESS_MAX_GAP_US);
        let expected_edges: u32 = burst.iter().map(|&b| falling_edges(b)).sum();
        match run_stress_burst(bus, &burst, expected_edges) {
            Ok(()) => {
                total_bytes += len;
                total_edges += expected_edges;
            }
            Err(e) => {
                println!(
                    "  burst {i}: len={len} gap_us={gap_us} edges={expected_edges}  FAIL: {e}"
                );
                any_fail = true;
                break;
            }
        }
        std::thread::sleep(Duration::from_micros(gap_us as u64));
    }
    println!(
        "  {STRESS_BURSTS} bursts, {total_bytes} bytes, {total_edges} edges  —  {}",
        if any_fail { "FAIL" } else { "PASS" }
    );
    if any_fail {
        bail!("stage 2 (stress) failed");
    }
    Ok(())
}

fn run_stress_burst(bus: &mut Bus, burst: &[u8], expected_edges: u32) -> Result<()> {
    // pirate_reset() doesn't clear FALLING_TOTAL or the IC ring (post-
    // trip diagnostic state). Snapshot the pre-burst count and gate on
    // the delta against the payload's exact expected edge count.
    let pre_total = bus.ic_snapshot()?.falling_total;
    bus.pirate_send_now(burst)?;
    let stamps = drain_stamps_until(bus, burst.len(), Duration::from_millis(200))?;
    let snap = bus.ic_snapshot()?;

    let verdict = validate_burst(burst, &stamps, &snap, pre_total, expected_edges);
    if let Err(e) = &verdict {
        dump_failure_context(burst, &stamps, &snap, pre_total, expected_edges, e);
    }
    verdict
}

fn validate_burst(
    burst: &[u8],
    stamps: &[BStamp],
    snap: &IcSnapshot,
    pre_total: u32,
    expected_edges: u32,
) -> Result<()> {
    if stamps.len() != burst.len() {
        bail!(
            "got {} stamps, expected {} (echo incomplete or unexpected tail)",
            stamps.len(),
            burst.len()
        );
    }
    validate_byte_match(stamps, burst)?;
    validate_no_count_under(stamps)?;
    for i in 0..stamps.len().saturating_sub(1) {
        let dt = stamps[i + 1].tick.wrapping_sub(stamps[i].tick);
        // Top half of u32 = wrapping_sub underflow = backwards stamp.
        if dt == 0 || dt > u32::MAX / 2 {
            bail!(
                "Δ stamps[{i}]→stamps[{}] = {dt} ticks (non-monotone or backwards)",
                i + 1
            );
        }
    }
    let ic_delta = snap.falling_total.wrapping_sub(pre_total);
    if ic_delta != expected_edges {
        bail!(
            "falling_total advanced by {ic_delta}, expected {expected_edges} \
             (mismatch ⇒ byte/edge ring desync or spurious IC capture)"
        );
    }
    Ok(())
}

/// Forensic dump for a failed burst. Prints the payload, every stamp with
/// its delta from the prev stamp and from the PLL-predicted tick (anchor
/// chain), and the surrounding IC ring entries. Stamp[0] is the cold-start
/// anchor; predicted[i] = stamps[0].tick + i·10·bit_ticks. Drift between
/// stamps[i].tick and predicted[i] reveals whether the PLL is snapping to
/// glitch edges (drift grows) vs missing a real edge (one-time miss).
fn dump_failure_context(
    burst: &[u8],
    stamps: &[BStamp],
    snap: &IcSnapshot,
    pre_total: u32,
    expected_edges: u32,
    err: &anyhow::Error,
) {
    let bit_ticks = snap.bit_ticks;
    let byte_period = bit_ticks.wrapping_mul(10);
    let ic_delta = snap.falling_total.wrapping_sub(pre_total);
    eprintln!();
    eprintln!("    --- forensic dump ---");
    eprintln!("    error: {err}");
    eprintln!(
        "    payload ({}B): {}",
        burst.len(),
        burst
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    eprintln!(
        "    bit_ticks={bit_ticks}, byte_period={byte_period}, cc_filter_delay={}",
        snap.cc_filter_delay
    );
    eprintln!(
        "    falling_total: pre={pre_total} post={} delta={ic_delta} expected={expected_edges}",
        snap.falling_total
    );
    eprintln!();
    eprintln!(
        "    {:>3}  {:>4}  {:>10}  {:>10}  {:>7}  {:>7}  flags  payload",
        "i", "byte", "tick", "predicted", "Δ_prev", "Δ_pred"
    );
    let base = stamps.first().map(|s| s.tick).unwrap_or(0);
    for (i, s) in stamps.iter().enumerate() {
        let predicted = base.wrapping_add(byte_period.wrapping_mul(i as u32));
        let delta_prev = if i == 0 {
            0i32
        } else {
            stamps[i].tick.wrapping_sub(stamps[i - 1].tick) as i32
        };
        let delta_pred = s.tick.wrapping_sub(predicted) as i32;
        let want = burst.get(i).copied().unwrap_or(0xFF);
        let match_mark = if s.byte == want { ' ' } else { '!' };
        eprintln!(
            "    {i:>3}  0x{:02X}  {:>10}  {:>10}  {:>+7}  {:>+7}  0x{:02X}    0x{want:02X}{match_mark}",
            s.byte, s.tick, predicted, delta_prev, delta_pred, s.flags
        );
    }
    eprintln!();
    eprintln!(
        "    IC ring window ({} entries shown, oldest first):",
        snap.entries.len()
    );
    let stamp_range = stamps
        .first()
        .zip(stamps.last())
        .map(|(f, l)| (f.tick, l.tick));
    for (i, &t) in snap.entries.iter().enumerate() {
        let in_burst = match stamp_range {
            Some((first, last)) => {
                let before =
                    t.wrapping_sub(first.wrapping_sub(byte_period)) <= byte_period.wrapping_mul(2);
                let inside = t.wrapping_sub(first) <= last.wrapping_sub(first);
                let after = last.wrapping_sub(t) > u32::MAX / 2
                    && t.wrapping_sub(last) <= byte_period.wrapping_mul(2);
                if inside {
                    '*'
                } else if before || after {
                    '~'
                } else {
                    ' '
                }
            }
            None => ' ',
        };
        eprintln!("    {in_burst} [{i:>3}] tick={t}");
    }
    eprintln!("    (* = inside stamp range, ~ = within 2 byte-periods of edge)");
    eprintln!();
}

/// Count 1→0 transitions in one LSB-first UART frame for `byte`:
/// `idle(1) → start(0) → b0 → b1 → … → b7 → stop(1)`. The start bit
/// always contributes one falling edge; data-bit edges depend on the
/// byte's bit pattern. The stop bit is high so it never adds a falling
/// edge regardless of `b7`.
fn falling_edges(byte: u8) -> u32 {
    let mut count = 1u32;
    let mut prev = 0u8;
    for i in 0..8 {
        let curr = (byte >> i) & 1;
        if prev == 1 && curr == 0 {
            count += 1;
        }
        prev = curr;
    }
    count
}

/// SplitMix64 — single-state deterministic generator. No external crate,
/// good enough for test-payload randomization.
fn lcg_next(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn lcg_in_range(state: &mut u64, lo: u32, hi: u32) -> u32 {
    let span = (hi - lo + 1) as u64;
    lo + (lcg_next(state) % span) as u32
}

// ---------------------------------------------------------------------------
// Stage 3: TX_COMP calibration
// ---------------------------------------------------------------------------

fn stage_tx_comp(bus: &mut Bus, bauds: &[u32], shots: u32, write: bool) -> Result<()> {
    println!();
    println!("stage 3: tx-comp (residual = stamp[0].tick − LAST?, single 0x55 byte)");
    let current = bus.pirate_comp()?;
    println!(
        "  current TX-comp:  pipe={} bit_q4={}",
        current.pipe, current.bit_q4
    );
    // bias%byte = systematic TX offset (= median residual) as % of one
    // byte-time. Tunable via (pipe, bit_q4); should be ~0 after tuning.
    // ±jit%byte = per-shot deviation from the median (half the peak-to-
    // peak span), again as % byte-time. Floor is ±0.5 bit-time ≈ ±5%
    // byte-time, from the U[0, brr] DR→TSR phase wait — physical, not
    // tunable. ±jit_ns is the same number in absolute nanoseconds.
    println!(
        "  {:>8}  {:>5}  {:>8}  {:>6}  {:>10}  {:>10}  {:>8}  result",
        "baud", "brr", "median", "span", "bias%byte", "±jit%byte", "±jit_ns"
    );
    let hz_per_us = bus.hz_per_us()?;
    let hclk_hz = hz_per_us * 1_000_000;
    let ns_per_tick = 1000.0 / hz_per_us as f64;
    // (brr, median, weight) for the weighted LS fit. Weight = 1/brr² is
    // the optimal inverse-variance weighting under the U[0, brr] phase
    // noise model: per-shot variance is brr²/12, so the variance of the
    // n-shot median scales the same way and 1/var = 12/brr² ∝ 1/brr².
    let mut points: Vec<(f64, f64, f64)> = Vec::with_capacity(bauds.len());
    let mut worst_bias_pct = 0.0_f64;
    let mut worst_half_jit_pct = 0.0_f64;
    let mut worst_half_jit_ns = 0.0_f64;
    let mut any_fail = false;
    for &baud in bauds {
        bus.pirate_set_baud(baud)?;
        let brr = hclk_hz / baud;
        let byte_ticks = (brr * 10) as f64;
        match collect_tx_comp_residuals(bus, shots) {
            Ok(residuals) => {
                let med = median_i64(&residuals);
                let lo = *residuals.iter().min().unwrap();
                let hi = *residuals.iter().max().unwrap();
                let span = hi - lo;
                let bias_pct = med as f64 / byte_ticks * 100.0;
                let half_jit_pct = (span as f64 / 2.0) / byte_ticks * 100.0;
                let half_jit_ns = (span as f64 / 2.0) * ns_per_tick;
                worst_bias_pct = worst_bias_pct.max(bias_pct.abs());
                worst_half_jit_pct = worst_half_jit_pct.max(half_jit_pct);
                worst_half_jit_ns = worst_half_jit_ns.max(half_jit_ns);
                println!(
                    "  {baud:>8}  {brr:>5}  {med:>+8}  {span:>6}  {bias_pct:>+9.3}%  ±{half_jit_pct:>8.3}%  ±{half_jit_ns:>6.0}  PASS"
                );
                let weight = 1.0 / (brr as f64).powi(2);
                points.push((brr as f64, med as f64, weight));
            }
            Err(e) => {
                println!("  {baud:>8}  {brr:>5}  FAIL: {e}");
                any_fail = true;
            }
        }
    }
    if any_fail {
        bail!("stage 3 (tx-comp) failed");
    }
    let Some((a, b)) = weighted_least_squares_fit(&points) else {
        bail!("stage 3: regression underdetermined (need ≥ 2 bauds)");
    };

    // Recommendation: stack the residual fit on top of what the pirate
    // already has — pipe_new = pipe_cur + a, bit_q4_new = bit_q4_cur + 16·b.
    // Clamp bit_q4 at 0; it's a u32 chip-side and a runaway model could
    // produce a (rounded) negative recommendation.
    let pipe_new = ((current.pipe as f64) + a).round();
    let bit_q4_new = ((current.bit_q4 as f64) + 16.0 * b).round().max(0.0);
    let pipe_new_u32 = pipe_new as u32;
    let bit_q4_new_u32 = bit_q4_new as u32;
    let d_pipe = pipe_new_u32 as i64 - current.pipe as i64;
    let d_bit = bit_q4_new_u32 as i64 - current.bit_q4 as i64;

    println!();
    println!("  fit:        residual = a + b·brr   (weighted LS, w ∝ 1/brr²)");
    println!("              a = {a:+.2} ticks");
    println!("              b = {b:+.4}");
    println!(
        "  recommend:  pipe={pipe_new_u32}  bit_q4={bit_q4_new_u32}    (Δ pipe={d_pipe:+}  Δ bit_q4={d_bit:+})"
    );

    let recommended = TxComp {
        pipe: pipe_new_u32,
        bit_q4: bit_q4_new_u32,
    };
    if recommended == current {
        println!("  already at recommended values; nothing to write.");
    } else if write {
        bus.pirate_set_comp(recommended)?;
        let verify = bus.pirate_comp()?;
        if verify != recommended {
            bail!("COMP write verify: wrote {recommended:?}, read back {verify:?}");
        }
        println!("  wrote COMP pipe={pipe_new_u32} bit_q4={bit_q4_new_u32}  →  OK");
    } else {
        println!("  rerun with --write to apply.");
    }
    println!();
    println!("  TX ruler accuracy:");
    println!(
        "    bias  ±{worst_bias_pct:.2}% byte-time   (systematic — tunable; should be ~0 after tuning)"
    );
    println!(
        "    jit   ±{worst_half_jit_pct:.2}% byte-time = ±{worst_half_jit_ns:.0} ns   (per-shot deviation from median)"
    );
    println!(
        "    floor: ±0.5 bit-time ≈ ±5% byte-time from U[0, brr] DR→TSR phase wait — physical, not tunable"
    );
    Ok(())
}

fn collect_tx_comp_residuals(bus: &mut Bus, shots: u32) -> Result<Vec<i64>> {
    let mut out = Vec::with_capacity(shots as usize);
    for shot in 0..shots {
        bus.pirate_reset()?;
        std::thread::sleep(Duration::from_millis(5));
        while !bus.pirate_bbatch(255)?.is_empty() {}

        // `send_now` routes through `schedule_send_at` with
        // at = now + TX_COMP + IMMEDIATE_SEND_MARGIN, which forces the
        // TIM4 OPM path (delta = 512, well inside the 16-bit scheduler
        // window) and stores LAST_SEND_TICK = at. `LAST?` then returns
        // the commanded wire-start tick, so residual = stamp − LAST?
        // directly measures (actual_pipeline − TX_COMP) + phase — i.e.
        // the value the send path actually consumes. (A naive 2 ms
        // future schedule would exceed TIM4 OPM's 16-bit range → fall
        // back to the immediate-DMA branch → TX_COMP never read →
        // regression diverges.)
        bus.pirate_send_now(&[0x55])?;

        let stamps = drain_stamps_until(bus, 1, Duration::from_millis(200))?;
        if stamps.is_empty() {
            bail!("shot {shot}: no stamp drained within 200 ms");
        }
        if stamps[0].flags != 0 {
            bail!(
                "shot {shot}: stamp[0].flags = 0x{:02X} (COUNT_UNDER)",
                stamps[0].flags
            );
        }
        if stamps[0].byte != 0x55 {
            bail!(
                "shot {shot}: stamp[0].byte = 0x{:02X}, expected 0x55",
                stamps[0].byte
            );
        }
        let commanded = bus.pirate_last_send_tick()?;
        let residual = stamps[0].tick.wrapping_sub(commanded) as i32 as i64;
        out.push(residual);
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Shared validators (carried over from tool-pirate-walker-validate)
// ---------------------------------------------------------------------------

fn validate_byte_match(stamps: &[BStamp], want: &[u8]) -> Result<()> {
    for (i, (s, &want)) in stamps.iter().zip(want.iter()).enumerate() {
        if s.byte != want {
            bail!("stamp[{i}].byte = 0x{:02X}, expected 0x{want:02X}", s.byte);
        }
    }
    Ok(())
}

fn validate_no_count_under(stamps: &[BStamp]) -> Result<()> {
    for (i, s) in stamps.iter().enumerate() {
        if s.flags != 0 {
            bail!(
                "stamp[{i}].flags = 0x{:02X} (COUNT_UNDER set — free-run, not a real edge)",
                s.flags
            );
        }
    }
    Ok(())
}

fn validate_spacing(
    stamps: &[BStamp],
    period: u32,
    tol: u32,
    range: std::ops::Range<usize>,
) -> Result<i64> {
    let mut max_dev: i64 = 0;
    for i in range.start..range.end.saturating_sub(1) {
        let dt = stamps[i + 1].tick.wrapping_sub(stamps[i].tick) as i64;
        let dev = (dt - period as i64).abs();
        if dev > max_dev {
            max_dev = dev;
        }
        if dev > tol as i64 {
            bail!(
                "Δ stamps[{i}]→stamps[{i_1}] = {dt} ticks, deviates {dev} > tol={tol} (period={period})",
                i_1 = i + 1
            );
        }
    }
    Ok(max_dev)
}

fn validate_ic_correspondence(stamps: &[BStamp], snap: &IcSnapshot) -> Result<()> {
    for (i, s) in stamps.iter().enumerate() {
        let anchor = s.tick.wrapping_add(snap.cc_filter_delay);
        let closest = snap
            .entries
            .iter()
            .map(|&t| (t as i64 - anchor as i64).abs())
            .min()
            .unwrap_or(i64::MAX);
        if closest > 2 {
            bail!(
                "stamp[{i}] anchor {anchor} has no matching IC entry (closest miss {closest} ticks)"
            );
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Drain helper
// ---------------------------------------------------------------------------

/// Poll `BBATCH` until at least `expected` stamps have accumulated, or
/// `timeout` elapses. The walker classifies incrementally (HT/TC + IDLE
/// driven on-chip), so an "empty" reply mid-frame is not an end-of-stream
/// signal — at slow baud, intermediate gaps between walker passes are
/// common. Always polls one more time after reaching `expected` to surface
/// unexpected tail bytes as a count mismatch in the validator.
fn drain_stamps_until(bus: &mut Bus, expected: usize, timeout: Duration) -> Result<Vec<BStamp>> {
    let mut stamps = Vec::new();
    let deadline = Instant::now() + timeout;
    while stamps.len() < expected && Instant::now() < deadline {
        let batch = bus.pirate_bbatch(255)?;
        if batch.is_empty() {
            std::thread::sleep(Duration::from_millis(2));
        } else {
            stamps.extend(batch);
        }
    }
    let tail = bus.pirate_bbatch(255)?;
    stamps.extend(tail);
    Ok(stamps)
}

// ---------------------------------------------------------------------------
// Numeric helpers
// ---------------------------------------------------------------------------

fn median_i64(xs: &[i64]) -> i64 {
    let mut v = xs.to_vec();
    v.sort_unstable();
    let n = v.len();
    if n == 0 {
        0
    } else if n.is_multiple_of(2) {
        (v[n / 2 - 1] + v[n / 2]) / 2
    } else {
        v[n / 2]
    }
}

/// Weighted least-squares fit `y = a + b·x` over `(x, y, weight)` triples.
/// Closed form: with `S_w = Σwᵢ`, `S_wx = Σwᵢxᵢ`, etc, the normal
/// equations give `b = (S_w·S_wxy − S_wx·S_wy) / (S_w·S_wxx − S_wx²)`
/// and `a = (S_wy − b·S_wx) / S_w`. Returns None if underdetermined
/// (< 2 points) or if the x-values are collinear under the weights.
fn weighted_least_squares_fit(points: &[(f64, f64, f64)]) -> Option<(f64, f64)> {
    if points.len() < 2 {
        return None;
    }
    let mut sw = 0.0;
    let mut swx = 0.0;
    let mut swy = 0.0;
    let mut swxx = 0.0;
    let mut swxy = 0.0;
    for &(x, y, w) in points {
        sw += w;
        swx += w * x;
        swy += w * y;
        swxx += w * x * x;
        swxy += w * x * y;
    }
    let den = sw * swxx - swx * swx;
    if den == 0.0 {
        return None;
    }
    let b = (sw * swxy - swx * swy) / den;
    let a = (swy - b * swx) / sw;
    Some((a, b))
}
