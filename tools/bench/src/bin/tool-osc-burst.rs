//! Zero-gap bombardment on silicon: the production hot loop
//! `[GWRITE(HOLD), COMMIT, GREAD]` — or a plain
//! `[WRITE(NOREPLY) × N, READ]` — sent as ONE wire burst per cycle
//! (pirate `BURST`: break + frame back-to-back, sub-byte spacing).
//!
//! This is the silicon complement to `firmware/lib/integration/tests/hot_loop.rs`:
//! the sim proves the logical zero-gap contract with zero-cost handlers; this
//! tool exposes the ISR-latency window the sim cannot model (frame N's
//! end-deadline work vs frame N+1's break FE, same PFIC priority). A frame
//! silently missed by the framer shows up here as a STALE read-back value
//! (missed write/commit) or a missing reply (missed read) — plus whatever the
//! servo's crc/drop counters say via wlink.
//!
//! The cycle scenarios, loop, classification, and turnaround tally live in
//! [`bench::run`]; the asserting bench test (`tests/hardware/hot_loop.rs`) drives
//! the same [`hot_loop_cycle`]/[`plain_flood_cycle`] builders so the two can't
//! drift. The extra probe flags below stay tool-only forensics.

use std::time::Duration;

use anyhow::{Result, bail};
use bench::osc::{build_instruction, build_read, gread_uniform_payload, gwrite_uniform_payload};
use bench::pirate::{BStamp, Client, auto_detect_pirate};
use bench::run::{
    BurstCycle, CycleObservation, CycleOutcome, Stats, burst_measure_observed, hot_loop_cycle,
    plain_flood_cycle,
};
use clap::Parser;
use osc_protocol::wire::{Inst, Opcode};

/// goal_position (see tool-osc-write): the rule-heavy hot-loop register —
/// its soft-limit rules make commit the representative worst-case work.
const GOAL_POSITION_ADDR: u16 = 0x0184;
const BCAST: u8 = 0xFE;
/// goal_position width (i32), the GREAD/READ span these cycles read back.
const GOAL_LEN: u16 = 4;

#[derive(Parser, Debug)]
#[command(about = "Bombard a servo with zero-gap frame bursts and verify read-back.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Wire baud.
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    /// Servo id.
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    /// Cycles (one burst each).
    #[arg(short, long, default_value_t = 1000)]
    count: u32,
    /// Plain mode: `writes` NOREPLY writes + READ instead of the
    /// GWRITE/COMMIT/GREAD hot loop.
    #[arg(long, default_value_t = false)]
    plain: bool,
    /// NOREPLY writes per burst in --plain mode.
    #[arg(short, long, default_value_t = 8)]
    writes: u32,
    /// Print a line per failing cycle.
    #[arg(short, long)]
    verbose: bool,
    /// Hex-dump the stamp stream of every failing cycle.
    #[arg(long, default_value_t = false)]
    dump: bool,
    /// Send the cycle's frames as individual host-paced BRKSENDs instead of
    /// one zero-gap burst (control leg: isolates encoding from timing).
    #[arg(long, default_value_t = false)]
    paced: bool,
    /// Hot loop with a plain READ instead of GREAD (isolates GREAD).
    #[arg(long, default_value_t = false)]
    no_gread: bool,
    /// One GREAD per cycle, nothing else (reply presence only, no value check).
    #[arg(long, default_value_t = false)]
    gread_only: bool,
    /// GWRITE(HOLD) + plain READ, no COMMIT (reply presence only).
    #[arg(long, default_value_t = false)]
    no_commit: bool,
    /// COMMIT + plain READ, no GWRITE (reply presence only).
    #[arg(long, default_value_t = false)]
    commit_read: bool,
}

/// Build one cycle for `value`, dispatched by the mode flags. The default and
/// `--plain` paths delegate to the shared scenario builders (also driven by the
/// bench test); the rest are tool-only forensic isolations.
fn cycle_for(args: &Args, value: i32) -> BurstCycle {
    let addr = GOAL_POSITION_ADDR;
    if args.plain {
        plain_flood_cycle(args.id, addr, args.writes, value)
    } else if args.gread_only {
        let gread = build_instruction(
            BCAST,
            Opcode::Gread,
            0,
            &gread_uniform_payload(addr, GOAL_LEN, &[args.id]),
        );
        BurstCycle {
            frames: vec![gread.clone()],
            last: gread,
            expect: None,
        }
    } else if args.no_commit {
        let data = value.to_le_bytes();
        let gwrite = build_instruction(
            BCAST,
            Opcode::Gwrite,
            Inst::FLAG_HOLD,
            &gwrite_uniform_payload(addr, &[(args.id, &data)]),
        );
        let read = build_read(args.id, addr, GOAL_LEN);
        BurstCycle {
            frames: vec![gwrite, read.clone()],
            last: read,
            expect: None,
        }
    } else if args.commit_read {
        let commit = build_instruction(BCAST, Opcode::Commit, 0, &[]);
        let read = build_read(args.id, addr, GOAL_LEN);
        BurstCycle {
            frames: vec![commit, read.clone()],
            last: read,
            expect: None,
        }
    } else if args.no_gread {
        // The hot loop with its GREAD swapped for a plain READ (isolates GREAD).
        let mut c = hot_loop_cycle(args.id, addr, value);
        c.frames.pop();
        let read = build_read(args.id, addr, GOAL_LEN);
        c.frames.push(read.clone());
        c.last = read;
        c
    } else {
        hot_loop_cycle(args.id, addr, value)
    }
}

fn dump_stamps(c: u32, stamps: &[BStamp], bit_ticks: u32) {
    println!("--- cycle {c} stamps ({}):", stamps.len());
    let mut prev: Option<u32> = None;
    for st in stamps {
        let d = prev.map(|p| st.tick.wrapping_sub(p) as f64 / bit_ticks as f64);
        match d {
            Some(d) if d > 12.0 => println!("  {:02x} +{d:8.1}b", st.byte),
            Some(_) => print!(" {:02x}", st.byte),
            None => print!("  {:02x}", st.byte),
        }
        prev = Some(st.tick);
    }
    println!();
}

fn main() -> Result<()> {
    let args = Args::parse();
    let port = match &args.port {
        Some(p) => p.clone(),
        None => auto_detect_pirate()?,
    };
    let mut client = Client::open(&port, Duration::from_millis(500))?;
    client.set_baud(args.baud)?;
    client.reset()?;

    let report = burst_measure_observed(
        &mut client,
        args.count,
        3,
        args.paced,
        |v| cycle_for(&args, v),
        |obs: &CycleObservation| observe(&args, obs),
    )?;

    let mode = if args.plain {
        format!("plain (WRITE noreply x {} + READ)", args.writes.max(1))
    } else {
        "hot loop (GWRITE hold + COMMIT + GREAD)".to_string()
    };
    println!("port         {}", client.port_path());
    println!("baud         {}", args.baud);
    println!("id           {}", args.id);
    println!("mode         {mode}");
    println!(
        "cycles       {} ok, {} stale, {} no-reply, {} other",
        report.ok.len(),
        report.stale,
        report.no_reply,
        report.other,
    );
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }
    if report.failures() * 10 > args.count {
        bail!("failure rate over 10%");
    }
    Ok(())
}

/// Per-cycle forensic output, gated by `--verbose` / `--dump`.
fn observe(args: &Args, obs: &CycleObservation) {
    match obs.outcome {
        CycleOutcome::Ok { .. } => {}
        CycleOutcome::Stale { got, expected } => {
            if args.verbose {
                println!(
                    "cycle {:>5}  STALE read-back {got:02x?} (expected {expected:02x?})",
                    obs.index
                );
            }
        }
        CycleOutcome::ResultErr(r) => {
            if args.verbose {
                println!("cycle {:>5}  result {r:?}", obs.index);
            }
        }
        CycleOutcome::NoReply => {
            if args.verbose {
                println!("cycle {:>5}  no reply", obs.index);
            }
            if args.dump {
                dump_stamps(obs.index, obs.stamps, obs.bit_ticks);
            }
        }
        CycleOutcome::Other(e) => {
            if args.verbose {
                println!("cycle {:>5}  {e}", obs.index);
            }
            if args.dump {
                dump_stamps(obs.index, obs.stamps, obs.bit_ticks);
            }
        }
        CycleOutcome::SendErr(e) => {
            if args.verbose {
                println!("cycle {:>5}  send err: {e}", obs.index);
            }
        }
    }
}
