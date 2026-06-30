//! Pirate self-tune. Loopback-only — no servo on the bus.
//!
//! Three stages, run in order, halting at first failure:
//!
//! 1. **classifier** — fixed-density 16-byte payload via `MASTER`, no
//!    inter-packet gap. Validates byte values, no `COUNT_UNDER`, inter-byte
//!    spacing within tolerance, and every stamp anchor matches a real IC
//!    ring entry.
//! 2. **stress** — slow-baud (115 200) single-byte `MASTER` shots with
//!    host-side gaps that scatter DMA HT/TC/IDLE across the inter-byte
//!    window. Validates the RX edge ring and byte ring stay paired across
//!    walker boundaries: byte values, strictly-monotone stamp ticks, and
//!    1:1 IC ring correspondence. Host-sleep is non-deterministic by
//!    design — the test gates on pirate-attested invariants only.
//! 3. **fire-comp** — scheduled-`FIRE` vs stamped wire edge, swept across
//!    baud. Residual = `stamp[0].tick − LAST?`. Regressed `a + b·brr`:
//!    `a` reports the `FIRE_COMP_FLAT_TICKS` delta (firmware ships 96);
//!    `b` ≈ 0 confirms the brr-coefficient is correct.

use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use bench::{BStamp, Bus, IcSnapshot};
use clap::{Parser, ValueEnum};

#[derive(Copy, Clone, Debug, ValueEnum, PartialEq, Eq)]
enum Stage {
    Classifier,
    Stress,
    FireComp,
    All,
}

#[derive(Parser, Debug)]
#[command(
    about = "Pirate self-tune (loopback only): classifier + multi-wrap stress + FIRE_COMP cal."
)]
struct Args {
    /// Pirate USB-CDC path. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Bauds for classifier + fire-comp sweep.
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
    /// Shots per baud for fire-comp regression.
    #[arg(long, default_value_t = 16)]
    fire_comp_shots: u32,
}

/// Loopback payload (16 bytes). Varied edge density across the byte
/// space; no `FF FF FD 00` substring so no servo on the bus reacts.
const LOOPBACK_PAYLOAD: [u8; 16] = [
    0x55, 0xAA, 0x55, 0xAA, 0xF0, 0x0F, 0x33, 0xCC, 0x00, 0xFE, 0x01, 0x80, 0xA5, 0x5A, 0xC3, 0x3C,
];

/// Stress probe baud — slow enough that a single byte fits well inside one
/// TIM2 wrap (~455 µs at 144 MHz), so the inter-byte gap dominates.
const STRESS_BAUD: u32 = 115_200;

/// Stress shots per gap setting. Keep small enough that the IC ring still
/// holds every entry when we snapshot at the end (ring is sized for normal
/// bus traffic, not a forensic capture).
const STRESS_SHOTS: usize = 16;

/// Gap multiples (× TIM2 wrap) used for the stress sweep. Different
/// gap sizes land DMA HT/TC/IDLE at different positions relative to
/// each start-bit edge, scattering walker-boundary cases across the run.
const STRESS_GAP_WRAPS: &[u32] = &[1, 2, 5, 10];

/// Firmware-side `FIRE_COMP_FLAT_TICKS` we're calibrating. Stage 3's
/// regression intercept is reported as a delta from this baseline.
const FIRE_COMP_FLAT_TICKS_FW: u32 = 96;

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
        stage_stress(&mut bus)?;
    }
    if run_all || args.stage == Stage::FireComp {
        stage_fire_comp(&mut bus, &bauds, args.fire_comp_shots)?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Stage 1: classifier
// ---------------------------------------------------------------------------

fn stage_classifier(bus: &mut Bus, bauds: &[u32], tol: Option<u32>) -> Result<()> {
    println!();
    println!("stage 1: classifier — 16-byte loopback echo");
    println!("  gates: stamp count, byte values, no PLL miss, inter-byte spacing ≤ tol,");
    println!("  every stamp anchored on a real IC edge.");
    println!();
    println!(
        "  {:>8}  {:>5}  {:>5}  {:>5}  {:>6}  result",
        "baud", "brr", "dev", "tol", "margin"
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

    bus.pirate_master(&LOOPBACK_PAYLOAD)?;
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

    // (tol - dev) / tol * 100; PASS guarantees dev ≤ tol so margin ∈ [0, 100].
    let margin_pct = if tol == 0 {
        0
    } else {
        let tol_i = tol as i64;
        ((tol_i - max_dev).max(0) * 100 / tol_i) as i32
    };
    let margin = format!("{margin_pct}%");
    Ok(format!(
        "{brr:>5}  {dev:>5}  {tol:>5}  {margin:>6}",
        brr = snap.bit_ticks,
        dev = max_dev,
        tol = tol,
    ))
}

// ---------------------------------------------------------------------------
// Stage 2: multi-wrap stress
// ---------------------------------------------------------------------------

fn stage_stress(bus: &mut Bus) -> Result<()> {
    println!();
    println!("stage 2: stress (baud={STRESS_BAUD}, single 0xFF shots, host-side gaps)");
    println!("  gates: byte values, strictly-monotone stamp ticks, Δ sanity ceiling,");
    println!("  IC ring 1:1 matches stamps (host-sleep jitter is the randomizer, not a metric).");
    println!();
    println!(
        "  {:>3}  {:>9}  {:>5}  {:>9}  {:>9}  {:>8}  result",
        "×wr", "gap_us", "shots", "min_dt_us", "max_dt_us", "ic_match"
    );
    bus.pirate_set_baud(STRESS_BAUD)?;
    let hz_per_us = bus.hz_per_us()?;
    // TIM2 wraps every 2^16 ticks = 65536 / hz_per_us µs.
    let wrap_us = 65536u32.div_ceil(hz_per_us);
    let mut any_fail = false;
    for &gap_wraps in STRESS_GAP_WRAPS {
        let gap_us = wrap_us * gap_wraps;
        match run_stress_probe(bus, gap_us, hz_per_us) {
            Ok(line) => println!("  {gap_wraps:>3}  {gap_us:>9}  {line}  PASS"),
            Err(e) => {
                println!("  {gap_wraps:>3}  {gap_us:>9}  ----  FAIL: {e}");
                any_fail = true;
            }
        }
    }
    if any_fail {
        bail!("stage 2 (stress) failed");
    }
    Ok(())
}

fn run_stress_probe(bus: &mut Bus, gap_us: u32, hz_per_us: u32) -> Result<String> {
    bus.pirate_reset()?;
    std::thread::sleep(Duration::from_millis(20));
    while !bus.pirate_bbatch(255)?.is_empty() {}
    // pirate_reset() doesn't clear FALLING_TOTAL or the IC ring (the ring
    // is post-trip diagnostic state). Snapshot the pre-probe count and
    // gate on the delta instead — any spurious capture during the probe
    // window shows up as ic_delta > STRESS_SHOTS.
    let pre_total = bus.ic_snapshot()?.falling_total;

    // 0xFF: lone start-bit falling edge, then 9 high bits + idle. One IC
    // entry per shot — easy to count, easy to back-check.
    for _ in 0..STRESS_SHOTS {
        bus.pirate_master(&[0xFF])?;
        std::thread::sleep(Duration::from_micros(gap_us as u64));
    }
    let stamps = drain_stamps_until(bus, STRESS_SHOTS, Duration::from_millis(200))?;
    let snap = bus.ic_snapshot()?;

    if stamps.len() != STRESS_SHOTS {
        bail!("got {} stamps, expected {STRESS_SHOTS}", stamps.len());
    }
    for (i, s) in stamps.iter().enumerate() {
        if s.byte != 0xFF {
            bail!("stress[{i}] byte = 0x{:02X}, expected 0xFF", s.byte);
        }
    }
    validate_no_count_under(&stamps)?;
    validate_ic_correspondence(&stamps, &snap)?;
    let ic_delta = snap.falling_total.wrapping_sub(pre_total);
    if ic_delta as usize != STRESS_SHOTS {
        bail!(
            "falling_total advanced by {ic_delta}, expected {STRESS_SHOTS} \
             (extras = spurious IC captures during the probe window)"
        );
    }

    // Δ sanity ceiling: 100 ms ≈ 100 000 µs × hz_per_us. A backwards
    // tick (wrap-race miscount) shows up as `wrapping_sub` returning a
    // value near u32::MAX, far above this ceiling.
    let max_sane_delta = 100_000u32.saturating_mul(hz_per_us);
    let mut min_dt = u32::MAX;
    let mut max_dt = 0u32;
    for i in 0..stamps.len() - 1 {
        let dt = stamps[i + 1].tick.wrapping_sub(stamps[i].tick);
        if dt == 0 {
            bail!("Δ stamps[{i}]→stamps[{}] = 0 (duplicate tick)", i + 1);
        }
        if dt > max_sane_delta {
            bail!(
                "Δ stamps[{i}]→stamps[{}] = {dt} ticks > sane ceiling \
                 {max_sane_delta} (TIM3 stitch failure or backwards stamp)",
                i + 1
            );
        }
        min_dt = min_dt.min(dt);
        max_dt = max_dt.max(dt);
    }

    let ic_str = format!("{ic_delta}/{STRESS_SHOTS}");
    Ok(format!(
        "{n:>5}  {min:>9}  {max:>9}  {ic:>8}",
        n = stamps.len(),
        min = min_dt / hz_per_us,
        max = max_dt / hz_per_us,
        ic = ic_str,
    ))
}

// ---------------------------------------------------------------------------
// Stage 3: FIRE_COMP_FLAT calibration
// ---------------------------------------------------------------------------

fn stage_fire_comp(bus: &mut Bus, bauds: &[u32], shots: u32) -> Result<()> {
    println!();
    println!("stage 3: fire-comp (residual = stamp[0].tick − LAST?, single 0x55 byte)");
    println!(
        "  {:>8}  {:>5}  {:>5}  {:>10}  {:>11}  result",
        "baud", "brr", "shots", "median", "range"
    );
    let hz_per_us = bus.hz_per_us()?;
    let hclk_hz = hz_per_us * 1_000_000;
    let mut points: Vec<(f64, f64)> = Vec::with_capacity(bauds.len());
    let mut any_fail = false;
    for &baud in bauds {
        bus.pirate_set_baud(baud)?;
        let brr = hclk_hz / baud;
        match collect_fire_comp_residuals(bus, hz_per_us, shots) {
            Ok(residuals) => {
                let med = median_i64(&residuals);
                let lo = *residuals.iter().min().unwrap();
                let hi = *residuals.iter().max().unwrap();
                let range = format!("[{lo:+},{hi:+}]");
                println!("  {baud:>8}  {brr:>5}  {shots:>5}  {med:>+10}  {range:>11}  PASS");
                points.push((brr as f64, med as f64));
            }
            Err(e) => {
                println!("  {baud:>8}  {brr:>5}  ----  FAIL: {e}");
                any_fail = true;
            }
        }
    }
    if any_fail {
        bail!("stage 3 (fire-comp) failed");
    }
    if let Some((a, b)) = least_squares_fit(&points) {
        let measured_flat = (FIRE_COMP_FLAT_TICKS_FW as f64) + a;
        println!();
        println!("  fit:  residual = a + b·brr");
        println!(
            "        a = {a:+.2} ticks   →  measured FIRE_COMP_FLAT_TICKS ≈ {measured_flat:.1} (firmware has {FIRE_COMP_FLAT_TICKS_FW})"
        );
        println!("        b = {b:+.4}         (brr-slope residual; ideal 0)");
    }
    Ok(())
}

fn collect_fire_comp_residuals(bus: &mut Bus, hz_per_us: u32, shots: u32) -> Result<Vec<i64>> {
    let mut out = Vec::with_capacity(shots as usize);
    for shot in 0..shots {
        bus.pirate_reset()?;
        std::thread::sleep(Duration::from_millis(5));
        while !bus.pirate_bbatch(255)?.is_empty() {}

        // Fire 2 ms in the future — plenty of slack for the scheduler to
        // pick the TIM4 OPM path even at low baud where FIRE_COMP is fat.
        let now = bus.pirate_tick()?;
        let target = now.wrapping_add(2 * hz_per_us * 1_000);
        bus.pirate_fire(&[0x55], target)?;

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
        let commanded = bus.pirate_last_fired()?;
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

fn least_squares_fit(points: &[(f64, f64)]) -> Option<(f64, f64)> {
    if points.len() < 2 {
        return None;
    }
    let n = points.len() as f64;
    let mean_x = points.iter().map(|(x, _)| *x).sum::<f64>() / n;
    let mean_y = points.iter().map(|(_, y)| *y).sum::<f64>() / n;
    let mut num = 0.0;
    let mut den = 0.0;
    for &(x, y) in points {
        num += (x - mean_x) * (y - mean_y);
        den += (x - mean_x) * (x - mean_x);
    }
    if den == 0.0 {
        return None;
    }
    let b = num / den;
    let a = mean_y - b * mean_x;
    Some((a, b))
}
