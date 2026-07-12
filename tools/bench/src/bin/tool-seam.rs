//! Host seam stationarity on silicon: the differential chain-pair tracker
//! (§9.3) reads drift as a shift from a baseline that only means anything
//! if the host's queuing seam between back-to-back silent instructions is
//! stationary. This tool measures that seam from the pirate's own TX echo
//! stamps — bursts of identical WRITE(NOREPLY) frames, break-to-break
//! spans inside each burst — and reports the distribution against the
//! servo's own pair gate (wire/16) and the per-pair ppm noise floor.

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, bail};
use bench::cli::{Connect, Target, print_conn};
use bench::osc::build_instruction;
use bench::pirate::BStamp;
use bench::run::{Stats, drain};
use clap::Parser;
use osc_protocol::wire::{Inst, Opcode};

/// goal_position: the hot-loop register the production writes target.
const GOAL_POSITION_ADDR: u16 = 0x0184;
const BREAK: u8 = 0x00;
const BITS_PER_BYTE: u32 = 10;
/// A burst's wire time is well under a millisecond at any supported baud.
const SETTLE_MS: u64 = 3;
/// The servo's pair-qualification gate: |err| ≤ wire/2^4 (drivers
/// `TRIM_GATE_SHIFT`). A seam past it disqualifies every pair.
const GATE_SHIFT: u32 = 4;

#[derive(Parser, Debug)]
#[command(about = "Measure the host seam distribution between burst frames from echo stamps.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    /// Frames per burst (spans per burst = frames − 1).
    #[arg(long, default_value_t = 16)]
    frames: u32,
    /// Bursts.
    #[arg(short, long, default_value_t = 64)]
    count: u32,
    /// Print a line per burst.
    #[arg(short, long)]
    verbose: bool,
}

/// Break ticks of `n` identical `frame` echoes in one burst capture, or an
/// error naming what broke (the tool measures the pirate, so a mismatched
/// echo is a finding, not noise to skip).
fn break_ticks(stamps: &[BStamp], frame: &[u8], n: usize) -> Result<Vec<u32>> {
    let stride = 1 + frame.len();
    let start = stamps
        .iter()
        .enumerate()
        .position(|(i, s)| {
            s.byte == BREAK
                && stamps.len() - i >= stride * n
                && stamps[i + 1..i + stride]
                    .iter()
                    .map(|s| s.byte)
                    .eq(frame.iter().copied())
        })
        .ok_or_else(|| anyhow::anyhow!("first frame echo not found in {} stamps", stamps.len()))?;
    let mut ticks = Vec::with_capacity(n);
    for k in 0..n {
        let s = &stamps[start + k * stride..start + (k + 1) * stride];
        if s[0].byte != BREAK || !s[1..].iter().map(|s| s.byte).eq(frame.iter().copied()) {
            bail!("frame {k} echo mismatch");
        }
        ticks.push(s[0].tick);
    }
    Ok(ticks)
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.frames < 2 {
        bail!("need at least 2 frames per burst for a span");
    }
    let mut client = args.conn.client()?;
    let hz_per_us = client.hz_per_us()?;
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / client.current_baud() as u64) as u32;
    print_conn(&client, args.target.id);
    println!("frames/burst {}  bursts {}", args.frames, args.count);

    // One silent write, repeated: every adjacent break pair brackets
    // exactly one CRC-verified silent instruction — the tracker's food.
    let mut payload = GOAL_POSITION_ADDR.to_le_bytes().to_vec();
    payload.extend_from_slice(&0i32.to_le_bytes());
    let frame = build_instruction(args.target.id, Opcode::Write, Inst::FLAG_NOREPLY, &payload);
    let burst: Vec<Vec<u8>> = (0..args.frames).map(|_| frame.clone()).collect();
    // The servo models the span as footprint·tpb: break ring byte + frame
    // bytes, 10 bits each (§9.3). The seam is the span's excess over it.
    let wire_ticks = (1 + frame.len() as u32) * BITS_PER_BYTE * bit_ticks;

    let mut seams_us: Vec<f64> = Vec::new();
    for i in 0..args.count {
        drain(&mut client)?;
        client.burst(&burst)?;
        sleep(Duration::from_millis(SETTLE_MS));
        let stamps = drain(&mut client)?;
        let ticks = break_ticks(&stamps, &frame, args.frames as usize)?;
        for pair in ticks.windows(2) {
            let seam = pair[1].wrapping_sub(pair[0]).wrapping_sub(wire_ticks);
            seams_us.push(seam as i32 as f64 / hz_per_us as f64);
        }
        if args.verbose {
            let last = seams_us.len();
            let n = args.frames as usize - 1;
            let burst_mean = seams_us[last - n..].iter().sum::<f64>() / n as f64;
            println!("burst {i:>4}  seam mean {burst_mean:8.2} us");
        }
    }

    let stats = Stats::from(&seams_us).expect("spans collected");
    let wire_us = wire_ticks as f64 / hz_per_us as f64;
    let span_us = wire_us + stats.mean;
    let gate_us = wire_us / (1 << GATE_SHIFT) as f64;
    let worst = seams_us
        .iter()
        .map(|s| s.abs())
        .fold(f64::NEG_INFINITY, f64::max);
    println!(
        "spans        {} (wire {wire_us:.2} us/frame)",
        seams_us.len()
    );
    println!("seam         min {:.2} us", stats.min);
    println!("             mean {:.2} us", stats.mean);
    println!("             max {:.2} us", stats.max);
    println!("             stddev {:.3} us", stats.stddev);
    println!(
        "noise        {:.0} ppm of span per pair (stddev/span)",
        stats.stddev / span_us * 1e6
    );
    if worst > gate_us {
        bail!("seam {worst:.2} us exceeds the servo pair gate wire/16 = {gate_us:.2} us");
    }
    println!("gate         worst |seam| {worst:.2} us <= wire/16 = {gate_us:.2} us");
    Ok(())
}
