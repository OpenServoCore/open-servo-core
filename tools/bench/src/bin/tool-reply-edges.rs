//! Reply-artifact forensics: run plain zero-gap cycles until the reply
//! parse fails (the ~2% "header failed validation" class at 1M), then
//! dump the pirate's stamp stream AND the raw IC falling-edge ring for
//! that exchange. The edges are the wire ground truth — if they match
//! the byte pattern a correct reply would produce, the servo transmitted
//! fine and the pirate's RX sampling merged the bytes; if they don't,
//! the servo garbled its TX.

use std::thread::sleep;
use std::time::Duration;

use anyhow::Result;
use bench::osc::{ExchangeError, build_instruction, parse_exchange};
use bench::pirate::{BStamp, Client, auto_detect_pirate};
use clap::Parser;
use osc_protocol::wire::{Inst, Opcode};

const GOAL_POSITION_ADDR: u16 = 0x0184;

#[derive(Parser, Debug)]
#[command(about = "Capture raw IC edges for a failing burst reply.")]
struct Args {
    #[arg(short, long)]
    port: Option<String>,
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    #[arg(short, long, default_value_t = 2000)]
    count: u32,
    /// NOREPLY writes per burst.
    #[arg(short, long, default_value_t = 8)]
    writes: u32,
}

fn plain_burst(id: u8, n: u32, v: i32) -> (Vec<Vec<u8>>, Vec<u8>) {
    let a = GOAL_POSITION_ADDR.to_le_bytes();
    let mut frames = Vec::new();
    for k in 0..n {
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

/// The whole cycle's stamp stream (all frames + reply), byte values with
/// per-byte deltas — shows whether every instruction byte reached the wire.
fn dump_all_stamps(stamps: &[BStamp], bit_ticks: u32) {
    println!("--- full stamp stream ({} bytes):", stamps.len());
    let mut prev: Option<u32> = None;
    let mut line = String::new();
    for s in stamps {
        let d = prev.map(|p| s.tick.wrapping_sub(p) as f64 / bit_ticks as f64);
        match d {
            Some(d) if d > 12.0 => {
                println!("  {line}");
                line = format!("(+{d:.1}b) {:02x}", s.byte);
            }
            _ => {
                line.push_str(&format!(" {:02x}", s.byte));
            }
        }
        prev = Some(s.tick);
    }
    println!("  {line}");
}

fn dump_failure(client: &mut Client, stamps: &[BStamp], bit_ticks: u32) -> Result<()> {
    let snap = client.ic_snapshot()?;
    println!("bit_ticks    {bit_ticks} (snap says {})", snap.bit_ticks);
    println!(
        "ic ring      falling_total={} walked={} rx_total={} byte_head={} entries={}",
        snap.falling_total,
        snap.walked,
        snap.rx_total,
        snap.byte_head,
        snap.entries.len()
    );

    // Stamps, absolute ticks, so they can be lined up with edges.
    println!("--- stamps tail (byte @ tick, delta in bits):");
    let mut prev: Option<u32> = None;
    let tail = stamps.len().saturating_sub(14);
    for s in &stamps[tail..] {
        let d = prev.map(|p| s.tick.wrapping_sub(p) as f64 / bit_ticks as f64);
        match d {
            Some(d) => println!("  {:02x} @ {:>10}  +{d:8.2}b", s.byte, s.tick),
            None => println!("  {:02x} @ {:>10}", s.byte, s.tick),
        }
        prev = Some(s.tick);
    }

    // Raw falling edges, absolute + delta-in-bits (tail only).
    println!("--- ic falling edges tail (tick, delta in bits):");
    let mut prev: Option<u32> = None;
    let etail = snap.entries.len().saturating_sub(24);
    for &t in &snap.entries[etail..] {
        match prev {
            Some(p) => {
                let d = t.wrapping_sub(p) as f64 / bit_ticks as f64;
                println!("  {t:>10}  +{d:8.2}b");
            }
            None => println!("  {t:>10}"),
        }
        prev = Some(t);
    }
    Ok(())
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

    // Warmup.
    let (frames, _) = plain_burst(args.id, args.writes.max(1), 0);
    let _ = client.burst(&frames);
    sleep(Duration::from_millis(5));
    drain(&mut client)?;

    let mut caught = 0u32;
    for c in 0..args.count {
        let v = (c % 1000) as i32 + 1;
        let (frames, last) = plain_burst(args.id, args.writes.max(1), v);
        client.burst(&frames)?;
        sleep(Duration::from_millis(3));
        let stamps = drain(&mut client)?;
        match parse_exchange(&stamps, &last, bit_ticks) {
            Ok(ex) if ex.status.payload != v.to_le_bytes() => {
                println!(
                    "=== STALE cycle {c}: read back {:02x?}, expected {:02x?}",
                    ex.status.payload,
                    v.to_le_bytes()
                );
                dump_all_stamps(&stamps, bit_ticks);
                dump_failure(&mut client, &stamps, bit_ticks)?;
                caught += 1;
                if caught >= 1 {
                    return Ok(());
                }
            }
            Ok(_) if c == 0 => {
                println!("=== GOOD cycle {c} baseline:");
                dump_failure(&mut client, &stamps, bit_ticks)?;
            }
            Ok(_) => {}
            Err(e @ (ExchangeError::BadHeader | ExchangeError::BadCrc { .. } | ExchangeError::NoReply)) => {
                println!("=== FAIL cycle {c}: {e}");
                dump_failure(&mut client, &stamps, bit_ticks)?;
                caught += 1;
                if caught >= 1 {
                    return Ok(());
                }
            }
            Err(e) => {
                println!("cycle {c}: {e} (not the target class, continuing)");
            }
        }
    }
    println!("{caught} failures in {} cycles", args.count);
    Ok(())
}
