//! Sustained NOREPLY-write flood: pack as many WRITE(NOREPLY) frames as fit
//! into one instrument burst and send them back-to-back for `--secs`, with no
//! reads or per-cycle drain. Used to measure kernel-tick starvation under a
//! continuous write burst (read `telemetry.sample_tick` via wlink while this
//! runs). Not a turnaround tool -- it just floods the wire.

use std::time::{Duration, Instant};

use anyhow::Result;
use bench::cli::{Connect, Target};
use bench::osc::build_instruction;
use clap::Parser;
use osc_protocol::wire::{Inst, Opcode};

#[derive(Parser, Debug)]
#[command(about = "Flood a servo with continuous NOREPLY writes for N seconds.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    /// Control-table address to write.
    #[arg(short, long, default_value_t = 388)]
    addr: u16,
    /// Payload bytes per write (ramp-filled).
    #[arg(short = 'D', long, default_value_t = 4)]
    bytes: usize,
    /// Flood duration, seconds.
    #[arg(short, long, default_value_t = 10)]
    secs: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.wire()?;

    // One WRITE(NOREPLY) frame: addr + ramp payload.
    // Zero payload: valid for any writable span (0 is within every rule's
    // range), so heavy multi-register writes don't trip a validator.
    let a = args.addr.to_le_bytes();
    let mut payload = vec![a[0], a[1]];
    payload.extend(std::iter::repeat_n(0u8, args.bytes));
    let frame = build_instruction(args.target.id, Opcode::Write, Inst::FLAG_NOREPLY, &payload);

    // Pack as many frames as fit the 640-byte burst stream (1 length byte each).
    let per = (640 / (frame.len() + 1)).max(1);
    let frames: Vec<Vec<u8>> = std::iter::repeat_n(frame.clone(), per).collect();

    let deadline = Instant::now() + Duration::from_secs(args.secs);
    let mut bursts = 0u64;
    while Instant::now() < deadline {
        client.burst(&frames)?;
        bursts += 1;
        // Drop the capture so it never overflows -- we don't use the
        // stamps, this flood only generates wire traffic.
        client.reset()?;
    }
    let frames_sent = bursts * per as u64;
    println!(
        "flood done: {} bytes/write @ {:#06x}, {} frames/burst, {} bursts, {} frames in {}s",
        args.bytes, args.addr, per, bursts, frames_sent, args.secs
    );
    Ok(())
}
