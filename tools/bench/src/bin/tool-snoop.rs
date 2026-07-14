//! Passive wire snoop: park the adapter's edge capture on a bus and print
//! every decoded byte with timing. Debug glass for bus forensics -- shows
//! break boundaries, byte hex, and inter-byte gaps so any talker's TX
//! shape can be read byte-exact. Capture is passive by construction (the
//! instrument drains edges without driving), but note the adapter IS the
//! usual host -- as a third-party witness it only observes traffic it
//! didn't originate if some other host drives the bus.

use std::time::Duration;

use anyhow::Result;
use bench::cli::Connect;
use bench::edges::BStamp;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "Passively capture bus traffic through the adapter and dump it.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    /// Seconds to listen (the capture is poll-drained throughout).
    #[arg(short, long, default_value_t = 3)]
    seconds: u64,
    /// Gap (us) that starts a new burst line.
    #[arg(short, long, default_value_t = 300)]
    gap_us: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut wire = args.conn.wire()?;
    let hz_per_us = wire.hz_per_us();

    println!(
        "listening {} s at {} baud on the osc-adapter ...",
        args.seconds,
        wire.current_baud(),
    );
    let stamps = wire.collect_stamps(Duration::from_secs(args.seconds).as_millis() as u64)?;

    if stamps.is_empty() {
        println!("no bytes captured");
        return Ok(());
    }

    println!("{} bytes captured:", stamps.len());
    let mut prev_tick: Option<u32> = None;
    for s in &stamps {
        let gap = prev_tick.map(|p| s.tick.wrapping_sub(p) / hz_per_us);
        prev_tick = Some(s.tick);
        if gap.is_none_or(|g| g > args.gap_us) {
            println!();
            print!("burst: ");
        }
        if s.flags & BStamp::BOUNDARY != 0 {
            print!("|BRK|");
        }
        print!("{:02x} ", s.byte);
    }
    println!();
    println!("(|BRK| = law-derived break stamp ahead of that frame)");
    Ok(())
}
