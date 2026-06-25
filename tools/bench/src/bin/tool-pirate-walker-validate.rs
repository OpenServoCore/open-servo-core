//! Pirate walker accuracy self-test.
//!
//! Two probes per baud:
//!
//! 1. **Loopback**: a 16-byte fixed-edge-density payload driven through
//!    the pirate's PB10↔PB11 jumper. Validates classification on
//!    pirate-clocked TX with no inter-packet gap.
//! 2. **Unicast Ping** (default; skip with `--no-servo`): a real DXL Ping
//!    to the bus servo, validates classification across the TX-echo →
//!    servo-reply boundary (an inter-packet RDT gap + servo-clocked reply
//!    bytes).
//!
//! Pass criterion per TIMING.md §3.4:
//!
//! 1. Every TX byte produces exactly one stamp with the right value.
//! 2. No stamp carries `COUNT_UNDER` — every anchor is a real edge.
//! 3. Stamp spacing within an unbroken byte run equals `bit_ticks · 10`
//!    within tolerance (USART TX is the originator's locked clock).
//! 4. Each stamp's anchor (`stamp.tick + cc_filter_delay`) matches an
//!    entry in the IC ring snapshot within ±2 ticks — i.e. a real edge,
//!    not a free-run.

use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use bench::{BStamp, Bus, BusArgs, DEFAULT_IDLE_US, IcSnapshot, build_ping, lift_against};
use clap::Parser;
use dxl_protocol::types::Id;

#[derive(Parser, Debug)]
#[command(about = "Validate pirate walker accuracy on TX-echo + unicast-Ping replies.")]
struct Args {
    /// Pirate USB-CDC path. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Comma-separated bauds to validate at. Default: 1M, 2M, 3M, 115200.
    #[arg(short, long, value_delimiter = ',')]
    bauds: Option<Vec<u32>>,
    /// Inter-byte spacing tolerance in ticks. Default: `bit_ticks / 4`.
    #[arg(long)]
    tolerance: Option<u32>,
    /// Skip the servo Ping probe; loopback-only run.
    #[arg(long)]
    no_servo: bool,
}

/// Loopback payload (16 bytes). Varied edge density across the byte
/// space; no `FF FF FD 00` substring so no servo on the bus reacts.
const LOOPBACK_PAYLOAD: [u8; 16] = [
    0x55, 0xAA, 0x55, 0xAA, 0xF0, 0x0F, 0x33, 0xCC, 0x00, 0xFE, 0x01, 0x80, 0xA5, 0x5A, 0xC3, 0x3C,
];

fn main() -> Result<()> {
    let args = Args::parse();
    let bauds: Vec<u32> = args
        .bauds
        .unwrap_or_else(|| vec![1_000_000, 2_000_000, 3_000_000, 115_200]);

    if args.no_servo {
        run_loopback_only(args.port, &bauds, args.tolerance)
    } else {
        run_with_servo(args.port, &bauds, args.tolerance)
    }
}

fn print_header() {
    println!();
    println!(
        "  {:>14}  {:>8}  {:>5}  {:>9}  {:>4}  {:>4}  result",
        "probe", "baud", "bytes", "byte_per", "dev", "ic"
    );
}

fn print_row(label: &str, baud: u32, line: &str, result: &str) {
    println!("  {label:>14}  {baud:>8}  {line}  {result}");
}

fn run_loopback_only(port: Option<String>, bauds: &[u32], tol: Option<u32>) -> Result<()> {
    let mut bus = Bus::start_pirate_only(port)?;
    println!("pirate: {}   (loopback-only mode)", bus.port_path());
    print_header();
    let mut any_fail = false;
    for &baud in bauds {
        bus.pirate_set_baud(baud)?;
        match run_loopback(&mut bus, baud, tol) {
            Ok(line) => print_row("loopback", baud, &line, "PASS"),
            Err(e) => {
                print_row("loopback", baud, "----", &format!("FAIL: {e}"));
                any_fail = true;
            }
        }
    }
    if any_fail {
        bail!("walker-validate: at least one probe failed");
    }
    Ok(())
}

fn run_with_servo(port: Option<String>, bauds: &[u32], tol: Option<u32>) -> Result<()> {
    let mut bus = Bus::start(BusArgs {
        port,
        target_baud: bauds[0],
    })?;
    let chip_id = bus.id();
    println!(
        "pirate: {}   chip id: {}   (loopback + Ping mode)",
        bus.port_path(),
        chip_id
    );
    print_header();
    let mut any_fail = false;
    for &baud in bauds {
        if baud != bus.baud()
            && let Err(e) = bus.set_chip_baud(baud)
        {
            print_row("set_chip_baud", baud, "----", &format!("FAIL: {e}"));
            any_fail = true;
            continue;
        }
        match run_loopback(&mut bus, baud, tol) {
            Ok(line) => print_row("loopback", baud, &line, "PASS"),
            Err(e) => {
                print_row("loopback", baud, "----", &format!("FAIL: {e}"));
                any_fail = true;
            }
        }
        match run_servo_ping(&mut bus, Id::new(chip_id), tol) {
            Ok(line) => print_row("unicast Ping", baud, &line, "PASS"),
            Err(e) => {
                print_row("unicast Ping", baud, "----", &format!("FAIL: {e}"));
                any_fail = true;
            }
        }
    }
    if any_fail {
        bail!("walker-validate: at least one probe failed");
    }
    Ok(())
}

fn run_loopback(bus: &mut Bus, baud: u32, tolerance_arg: Option<u32>) -> Result<String> {
    bus.pirate_set_baud(baud)?;
    bus.pirate_reset()?;
    std::thread::sleep(Duration::from_millis(20));
    while !bus.pirate_bbatch(255)?.is_empty() {}

    bus.pirate_master(&LOOPBACK_PAYLOAD)?;
    std::thread::sleep(Duration::from_millis(20));

    let mut stamps = Vec::new();
    loop {
        let batch = bus.pirate_bbatch(255)?;
        if batch.is_empty() {
            break;
        }
        stamps.extend(batch);
    }
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
    let tol = tolerance_arg.unwrap_or(snap.bit_ticks / 4);
    let max_dev = validate_spacing(&stamps, byte_period, tol, 0..stamps.len())?;
    validate_ic_correspondence(&stamps, &snap)?;

    Ok(format!(
        "{n:>5}  {bp:>9}  {dev:>4}  {ic:>4}",
        n = stamps.len(),
        bp = byte_period,
        dev = max_dev,
        ic = snap.entries.len(),
    ))
}

fn run_servo_ping(bus: &mut Bus, id: Id, tolerance_arg: Option<u32>) -> Result<String> {
    bus.pirate_reset()?;
    std::thread::sleep(Duration::from_millis(5));
    while !bus.pirate_bbatch(255)?.is_empty() {}

    let req = build_ping(id)?;
    let cap = bus.xfer(&req, DEFAULT_IDLE_US)?;
    let snap = bus.ic_snapshot()?;

    let stamps = cap.stamps;
    if stamps.len() < req.len() {
        bail!(
            "got {} stamps, expected ≥ {} (echo incomplete)",
            stamps.len(),
            req.len()
        );
    }
    let timing = cap
        .timing
        .ok_or_else(|| anyhow!("no reply on Ping — chip silent or echo-only"))?;

    // Echo: must byte-match the request, byte_period spacing exact.
    validate_byte_match(&stamps[..req.len()], &req)?;
    validate_no_count_under(&stamps)?;
    let byte_period = snap.bit_ticks.wrapping_mul(10);
    let tol = tolerance_arg.unwrap_or(snap.bit_ticks / 4);
    let echo_dev = validate_spacing(&stamps, byte_period, tol, 0..req.len())?;

    // Reply: spacing at servo's clock — same tol absorbs ±0.4% HSI drift
    // (typically <8 ticks at 1 Mbaud).
    let reply_dev = validate_spacing(&stamps, byte_period, tol, req.len()..stamps.len())?;

    validate_ic_correspondence(&stamps, &snap)?;

    // Sanity: the gap between echo and reply should look like a real
    // inter-packet RDT (≥ 2 byte_periods) and surface as a > tol deviation
    // at the boundary.
    let gap = stamps[req.len()]
        .tick
        .wrapping_sub(stamps[req.len() - 1].tick) as i64;
    let max_dev = echo_dev.max(reply_dev);

    Ok(format!(
        "{n:>5}  {bp:>9}  {dev:>4}  {ic:>4}    (gap={gap}, reply_span={span})",
        n = stamps.len(),
        bp = byte_period,
        dev = max_dev,
        ic = snap.entries.len(),
        span = timing.reply_last.wrapping_sub(timing.reply_first),
    ))
}

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
    byte_period: u32,
    tol: u32,
    range: std::ops::Range<usize>,
) -> Result<i64> {
    let mut max_dev: i64 = 0;
    // Pairs strictly inside `range`: from range.start..range.end−1 to next.
    for i in range.start..range.end.saturating_sub(1) {
        let dt = stamps[i + 1].tick.wrapping_sub(stamps[i].tick) as i64;
        let dev = (dt - byte_period as i64).abs();
        if dev > max_dev {
            max_dev = dev;
        }
        if dev > tol as i64 {
            bail!(
                "inter-byte Δ stamps[{i}]→stamps[{i_1}] = {dt} ticks, deviates {dev} > tol={tol} (byte_period={byte_period})",
                i_1 = i + 1
            );
        }
    }
    Ok(max_dev)
}

fn validate_ic_correspondence(stamps: &[BStamp], snap: &IcSnapshot) -> Result<()> {
    let lifted: Vec<u32> = snap
        .entries
        .iter()
        .map(|&raw| lift_against(snap.ref_tick, raw))
        .collect();
    for (i, s) in stamps.iter().enumerate() {
        let anchor = s.tick.wrapping_add(snap.cc_filter_delay);
        let closest = lifted
            .iter()
            .map(|&t| (t as i64 - anchor as i64).abs())
            .min()
            .unwrap_or(i64::MAX);
        if closest > 2 {
            bail!(
                "stamp[{i}] anchor {} has no matching IC entry (closest miss {} ticks)",
                anchor,
                closest
            );
        }
    }
    Ok(())
}
