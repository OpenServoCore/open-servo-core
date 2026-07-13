//! Fleet chain timing on silicon: one broadcast GREAD listing every servo,
//! the replies arriving as a snoop-sequenced status chain (protocol sec 6) -- or the full
//! production cycle (`GWRITE(HOLD) + COMMIT + GREAD chain`, --hot) -- with
//! per-slot chain-gap stats. The single-servo complement is `tool-burst`.

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, anyhow};
use bench::cli::{Connect, gate_fail_rate};
use bench::osc::{build_instruction, gread_uniform_payload, gwrite_uniform_payload};
use bench::pirate::{BStamp, Client};
use bench::run::{Stats, drain};
use clap::Parser;
use osc_protocol::crc::osc_crc;
use osc_protocol::frame::Header;
use osc_protocol::wire::{Inst, Opcode, ResultCode};

/// goal_position: the rule-heavy hot-loop register (see tool-burst).
const GOAL_POSITION_ADDR: u16 = 0x0184;
const GOAL_LEN: u16 = 4;
const BCAST: u8 = 0xFE;
const BREAK: u8 = 0x00;
const BITS_PER_BYTE: u32 = 10;
/// Chain wire time is well under a millisecond at any supported baud.
const SETTLE_MS: u64 = 3;

#[derive(Parser, Debug)]
#[command(about = "Broadcast GREAD chain across a fleet and time every slot.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    /// Chain slots, in reply order.
    #[arg(long, value_delimiter = ',', default_value = "1,2,3,4,5")]
    ids: Vec<u8>,
    /// Cycles (one chain each).
    #[arg(short, long, default_value_t = 500)]
    count: u32,
    /// Full production cycle: GWRITE(HOLD) + COMMIT ahead of the GREAD, one
    /// zero-gap burst; read-back must equal the cycle's written value.
    #[arg(long, default_value_t = false)]
    hot: bool,
    /// Print a line per failing cycle.
    #[arg(short, long)]
    verbose: bool,
}

/// One decoded chain slot with its wire timing.
struct Slot {
    id: u8,
    result: Option<ResultCode>,
    payload: Vec<u8>,
    break_tick: u32,
    end_tick: u32,
}

/// Decode the status chain that follows the GREAD echo. Mirrors
/// `osc::parse_exchange`, generalized to `want` successive frames.
fn parse_chain(
    stamps: &[BStamp],
    gread_wire: &[u8],
    want: usize,
    bit_ticks: u32,
) -> Result<(u32, Vec<Slot>)> {
    let echo = stamps
        .iter()
        .enumerate()
        .position(|(i, s)| {
            s.byte == BREAK
                && stamps.len() - i > gread_wire.len()
                && stamps[i + 1..=i + gread_wire.len()]
                    .iter()
                    .map(|s| s.byte)
                    .eq(gread_wire.iter().copied())
        })
        .ok_or_else(|| anyhow!("gread echo not found"))?;
    let echo_end = stamps[echo + gread_wire.len()]
        .tick
        .wrapping_add(BITS_PER_BYTE * bit_ticks);

    let mut slots = Vec::new();
    let mut i = echo + gread_wire.len() + 1;
    while slots.len() < want {
        let Some(off) = stamps[i..].iter().position(|s| s.byte == BREAK) else {
            break;
        };
        let b = i + off;
        let reply = &stamps[b..];
        if reply.len() < Header::SIZE {
            break;
        }
        let hdr_bytes = [reply[0].byte, reply[1].byte, reply[2].byte, reply[3].byte];
        let hdr = Header::from_bytes(&hdr_bytes);
        if hdr.validate().is_err() {
            i = b + 1;
            continue;
        }
        let footprint = hdr.frame_end();
        if reply.len() < footprint {
            break;
        }
        let frame: Vec<u8> = reply[..footprint].iter().map(|s| s.byte).collect();
        let clen = hdr.covered_len();
        if osc_crc(&frame[..clen]) != u16::from_le_bytes([frame[clen], frame[clen + 1]]) {
            i = b + 1;
            continue;
        }
        let p = hdr.payload_len() as usize;
        slots.push(Slot {
            id: hdr.id.as_byte(),
            result: hdr.inst.result(),
            payload: frame[4..4 + p].to_vec(),
            break_tick: reply[0].tick,
            end_tick: reply[footprint - 1]
                .tick
                .wrapping_add(BITS_PER_BYTE * bit_ticks),
        });
        i = b + footprint;
    }
    Ok((echo_end, slots))
}

fn cycle_frames(args: &Args, value: i32) -> (Vec<Vec<u8>>, Vec<u8>) {
    let gread = build_instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform_payload(GOAL_POSITION_ADDR, GOAL_LEN, &args.ids),
    );
    if args.hot {
        let data = value.to_le_bytes();
        let entries: Vec<(u8, &[u8])> = args.ids.iter().map(|&id| (id, &data[..])).collect();
        let gwrite = build_instruction(
            BCAST,
            Opcode::Gwrite,
            Inst::FLAG_HOLD,
            &gwrite_uniform_payload(GOAL_POSITION_ADDR, &entries),
        );
        let commit = build_instruction(BCAST, Opcode::Commit, 0, &[]);
        (vec![gwrite, commit, gread.clone()], gread)
    } else {
        (vec![gread.clone()], gread)
    }
}

/// One cycle's verdict: per-slot gaps come back only from fully-clean chains
/// so the stats never mix failure modes with timing.
fn run_cycle(
    client: &mut Client,
    args: &Args,
    value: i32,
    bit_ticks: u32,
) -> Result<(f64, Vec<f64>, f64)> {
    let (frames, gread) = cycle_frames(args, value);
    client.burst(&frames)?;
    sleep(Duration::from_millis(SETTLE_MS));
    let stamps = drain(client)?;

    // Timing needs anchored ticks: a COUNT_UNDER stamp has no boundary
    // capture behind it. BOUNDARY-flagged stamps are the anchors themselves
    // (every break stamp carries bit 1 since the boundary-capture pirate).
    if stamps.iter().any(|s| s.flags & BStamp::COUNT_UNDER != 0) {
        return Err(anyhow!("unanchored stamps (COUNT_UNDER)"));
    }
    let (echo_end, slots) = parse_chain(&stamps, &gread, args.ids.len(), bit_ticks)?;
    if slots.len() != args.ids.len() {
        return Err(anyhow!(
            "chain: {} of {} slots",
            slots.len(),
            args.ids.len()
        ));
    }
    for (slot, &id) in slots.iter().zip(&args.ids) {
        if slot.id != id {
            return Err(anyhow!("slot order: got id {}, want {}", slot.id, id));
        }
        if slot.result != Some(ResultCode::Ok) {
            return Err(anyhow!("id {}: result {:?}", slot.id, slot.result));
        }
        if args.hot && slot.payload != value.to_le_bytes() {
            return Err(anyhow!(
                "id {}: stale read-back {:02x?}",
                slot.id,
                slot.payload
            ));
        }
    }

    let hz = bit_ticks as f64 * args.conn.baud as f64 / 1_000_000.0;
    let slot0 = slots[0].break_tick.wrapping_sub(echo_end) as f64 / hz;
    let gaps = slots
        .windows(2)
        .map(|w| w[1].break_tick.wrapping_sub(w[0].end_tick) as f64 / hz)
        .collect();
    let total = slots.last().unwrap().end_tick.wrapping_sub(echo_end) as f64 / hz;
    Ok((slot0, gaps, total))
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.client()?;
    let hz_per_us = client.hz_per_us()?;
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / client.current_baud() as u64) as u32;

    // Warmup: prime the chip and flush stale stamps before measuring.
    let _ = run_cycle(&mut client, &args, 0, bit_ticks);

    let n_gaps = args.ids.len().saturating_sub(1);
    let mut slot0 = Vec::new();
    let mut gaps: Vec<Vec<f64>> = vec![Vec::new(); n_gaps];
    let mut totals = Vec::new();
    let mut fail = 0u32;
    for i in 0..args.count {
        let value = (i % 1000) as i32 + 1;
        match run_cycle(&mut client, &args, value, bit_ticks) {
            Ok((s0, g, total)) => {
                slot0.push(s0);
                for (k, v) in g.into_iter().enumerate() {
                    gaps[k].push(v);
                }
                totals.push(total);
            }
            Err(e) => {
                fail += 1;
                if args.verbose {
                    println!("cycle {i:>5}  FAIL: {e}");
                }
            }
        }
    }

    let mode = if args.hot {
        "hot (GWRITE hold + COMMIT + GREAD chain)"
    } else {
        "GREAD chain"
    };
    println!("baud         {}", args.conn.baud);
    println!("ids          {:?}", args.ids);
    println!("mode         {mode}");
    println!("cycles       {} ok, {fail} fail", slot0.len());
    if let Some(s) = Stats::from(&slot0) {
        println!(
            "slot0 turnaround  min {:.2}  mean {:.2}  max {:.2}  stddev {:.2} us",
            s.min, s.mean, s.max, s.stddev
        );
    }
    for (k, g) in gaps.iter().enumerate() {
        if let Some(s) = Stats::from(g) {
            println!(
                "gap slot{}->{}     min {:.2}  mean {:.2}  max {:.2}  stddev {:.2} us",
                k,
                k + 1,
                s.min,
                s.mean,
                s.max,
                s.stddev
            );
        }
    }
    if let Some(s) = Stats::from(&totals) {
        println!(
            "chain total       min {:.2}  mean {:.2}  max {:.2}  stddev {:.2} us",
            s.min, s.mean, s.max, s.stddev
        );
    }
    gate_fail_rate(fail, args.count)
}
