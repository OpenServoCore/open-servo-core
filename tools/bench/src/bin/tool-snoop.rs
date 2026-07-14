//! Passive wire snoop: park the pirate on a bus someone else is driving and
//! print every captured byte with timing. Debug glass for third-party hosts
//! (e.g. the wch-linke adapter spike) -- shows break boundaries, byte hex,
//! and inter-byte gaps so a foreign host's TX shape can be read byte-exact.

use std::time::{Duration, Instant};

use anyhow::Result;
use bench::cli::Connect;
use bench::pirate::BStamp;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "Passively capture bus traffic through the pirate and dump it.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    /// Seconds to listen before draining.
    #[arg(short, long, default_value_t = 3)]
    seconds: u64,
    /// Gap (us) that starts a new burst line.
    #[arg(short, long, default_value_t = 300)]
    gap_us: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.client()?;
    let hz_per_us = client.hz_per_us()?;

    client.reset()?;
    println!(
        "listening {} s at {} baud on {} ...",
        args.seconds,
        client.current_baud(),
        client.port_path()
    );
    std::thread::sleep(Duration::from_secs(args.seconds));

    let mut stamps: Vec<BStamp> = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        match client.drain_byte()? {
            Some(s) => stamps.push(s),
            None => break,
        }
    }

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
    println!("(|BRK| = break-boundary capture on that byte's stamp)");
    Ok(())
}
