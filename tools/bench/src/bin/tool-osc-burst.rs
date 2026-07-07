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

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, bail};
use bench::osc::{ExchangeError, build_instruction, parse_exchange};
use bench::pirate::{BStamp, Client, auto_detect_pirate};
use bench::run::Stats;
use clap::Parser;
use osc_protocol::wire::{Inst, Opcode, ResultCode};

/// goal_position (see tool-osc-write): the rule-heavy hot-loop register —
/// its soft-limit rules make commit the representative worst-case work.
const GOAL_POSITION_ADDR: u16 = 0x0184;
const BCAST: u8 = 0xFE;

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

fn gwrite_uniform(addr: u16, id: u8, data: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&addr.to_le_bytes());
    p.push(data.len() as u8);
    p.push(id);
    p.extend_from_slice(data);
    p
}

fn gread_uniform(addr: u16, count: u16, ids: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&addr.to_le_bytes());
    p.extend_from_slice(&count.to_le_bytes());
    p.extend_from_slice(ids);
    p
}

/// One hot-loop cycle: GWRITE(HOLD) + COMMIT + GREAD, zero gap.
fn hot_loop_burst(id: u8, v: i32) -> (Vec<Vec<u8>>, Vec<u8>) {
    let gwrite = build_instruction(
        BCAST,
        Opcode::Gwrite,
        Inst::FLAG_HOLD,
        &gwrite_uniform(GOAL_POSITION_ADDR, id, &v.to_le_bytes()),
    );
    let commit = build_instruction(BCAST, Opcode::Commit, 0, &[]);
    let gread = build_instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform(GOAL_POSITION_ADDR, 4, &[id]),
    );
    (vec![gwrite, commit, gread.clone()], gread)
}

/// One plain cycle: WRITE(NOREPLY) × n with distinct values (last = `v`),
/// then READ, zero gap.
fn plain_burst(id: u8, n: u32, v: i32) -> (Vec<Vec<u8>>, Vec<u8>) {
    let a = GOAL_POSITION_ADDR.to_le_bytes();
    let mut frames = Vec::new();
    for k in 0..n {
        // Earlier writes count down toward `v` so every frame is distinct
        // and the final table value is `v` exactly.
        let val = v - (n - 1 - k) as i32;
        let d = val.to_le_bytes();
        let mut p = vec![a[0], a[1]];
        p.extend_from_slice(&d);
        frames.push(build_instruction(id, Opcode::Write, Inst::FLAG_NOREPLY, &p));
    }
    let read = build_instruction(id, Opcode::Read, 0, &[a[0], a[1], 4, 0]);
    frames.push(read.clone());
    (frames, read)
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
        if d.is_none_or(|d| d <= 12.0) {
            // stay on line
        }
        prev = Some(st.tick);
    }
    println!();
}

fn drain(client: &mut Client) -> Result<Vec<BStamp>> {
    let mut all = Vec::new();
    loop {
        let batch = client.bbatch(255)?;
        if batch.is_empty() {
            return Ok(all);
        }
        all.extend(batch);
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let port = match args.port {
        Some(p) => p,
        None => auto_detect_pirate()?,
    };
    let mut client = Client::open(&port, Duration::from_millis(500))?;
    client.set_baud(args.baud)?;
    client.reset()?;
    let hz_per_us = client.hz_per_us()?;
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / args.baud as u64) as u32;

    // Warmup exchange (first-post-reset flake), value distinct from cycle 0.
    let (frames, _) = hot_loop_burst(args.id, 0);
    let _ = client.burst(&frames);
    sleep(Duration::from_millis(5));
    drain(&mut client)?;

    let mut ok = Vec::new();
    let (mut stale, mut no_reply, mut other) = (0u32, 0u32, 0u32);
    for c in 0..args.count {
        // Values stay small positive (inside goal_position soft limits) and
        // never repeat between adjacent cycles, so a stale read-back is
        // unambiguous.
        let v = (c % 1000) as i32 + 1;
        let a = GOAL_POSITION_ADDR.to_le_bytes();
        let (frames, last, check) = if args.plain {
            let (f, l) = plain_burst(args.id, args.writes.max(1), v);
            (f, l, true)
        } else if args.gread_only {
            let g = build_instruction(
                BCAST,
                Opcode::Gread,
                0,
                &gread_uniform(GOAL_POSITION_ADDR, 4, &[args.id]),
            );
            (vec![g.clone()], g, false)
        } else if args.no_commit {
            let gwrite = build_instruction(
                BCAST,
                Opcode::Gwrite,
                Inst::FLAG_HOLD,
                &gwrite_uniform(GOAL_POSITION_ADDR, args.id, &v.to_le_bytes()),
            );
            let read = build_instruction(args.id, Opcode::Read, 0, &[a[0], a[1], 4, 0]);
            (vec![gwrite, read.clone()], read, false)
        } else if args.commit_read {
            let commit = build_instruction(BCAST, Opcode::Commit, 0, &[]);
            let read = build_instruction(args.id, Opcode::Read, 0, &[a[0], a[1], 4, 0]);
            (vec![commit, read.clone()], read, false)
        } else if args.no_gread {
            let (mut f, _) = hot_loop_burst(args.id, v);
            let read = build_instruction(args.id, Opcode::Read, 0, &[a[0], a[1], 4, 0]);
            f.pop();
            f.push(read.clone());
            (f, read, true)
        } else {
            let (f, l) = hot_loop_burst(args.id, v);
            (f, l, true)
        };
        let sent = if args.paced {
            frames.iter().try_for_each(|f| {
                sleep(Duration::from_millis(1));
                client.brksend(f)
            })
        } else {
            client.burst(&frames)
        };
        if let Err(e) = sent {
            other += 1;
            if args.verbose {
                println!("cycle {c:>5}  send err: {e}");
            }
            sleep(Duration::from_millis(2));
            continue;
        }
        sleep(Duration::from_millis(3));
        let stamps = drain(&mut client)?;
        match parse_exchange(&stamps, &last, bit_ticks) {
            Ok(ex) => {
                let expect = v.to_le_bytes();
                if ex.status.result != Some(ResultCode::Ok) {
                    other += 1;
                    if args.verbose {
                        println!("cycle {c:>5}  result {:?}", ex.status.result);
                    }
                } else if check && ex.status.payload != expect {
                    stale += 1;
                    if args.verbose {
                        println!(
                            "cycle {c:>5}  STALE read-back {:02x?} (expected {expect:02x?})",
                            ex.status.payload
                        );
                    }
                } else {
                    ok.push(ex.turnaround_ticks as f64 / hz_per_us as f64);
                }
            }
            Err(ExchangeError::NoReply) => {
                no_reply += 1;
                if args.verbose {
                    println!("cycle {c:>5}  no reply");
                }
                if args.dump {
                    dump_stamps(c, &stamps, bit_ticks);
                }
            }
            Err(e) => {
                other += 1;
                if args.verbose {
                    println!("cycle {c:>5}  {e}");
                }
                if args.dump {
                    dump_stamps(c, &stamps, bit_ticks);
                }
            }
        }
    }

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
        "cycles       {} ok, {stale} stale, {no_reply} no-reply, {other} other",
        ok.len()
    );
    if let Some(s) = Stats::from(&ok) {
        s.print();
    }
    if (stale + no_reply + other) * 10 > args.count {
        bail!("failure rate over 10%");
    }
    Ok(())
}
