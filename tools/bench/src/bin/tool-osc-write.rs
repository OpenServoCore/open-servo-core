//! osc-native WRITE: measure TURNAROUND for the mutating path — the write is
//! staged through the LOW consumer at the covered checkpoint (decode +
//! validate overlap the frame's wire tail), and the verdict at the frame end
//! commits + sequences the ack break.

use std::time::Duration;

use anyhow::{Result, bail};
use bench::osc::build_write;
use bench::pirate::{Client, auto_detect_pirate};
use bench::run::{Stats, measure};
use clap::Parser;
use osc_protocol::wire::ResultCode;

/// Control-table address of `goal_position` (osc-core
/// `regions::control`, CONTROL_BASE_ADDR 0x180 + lifecycle offset 4; value
/// pinned here to keep the heavy core crate out of the bench build). The
/// hot-loop register: its ge/le soft-limit rules make it the representative
/// write-validation workload.
const GOAL_POSITION_ADDR: u16 = 0x0184;

#[derive(Parser, Debug)]
#[command(about = "WRITE a control-table span over osc-native and report TURNAROUND.")]
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
    /// Control-table address.
    #[arg(short, long, default_value_t = GOAL_POSITION_ADDR)]
    addr: u16,
    /// Payload as hex bytes (default: goal_position = 0).
    #[arg(short = 'D', long, default_value = "00000000")]
    data: String,
    /// Number of writes.
    #[arg(short, long, default_value_t = 50)]
    count: u32,
    /// Print a line per write.
    #[arg(short, long)]
    verbose: bool,
}

fn parse_hex(s: &str) -> Result<Vec<u8>> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !s.len().is_multiple_of(2) {
        bail!("hex payload needs an even digit count");
    }
    (0..s.len())
        .step_by(2)
        .map(|i| Ok(u8::from_str_radix(&s[i..i + 2], 16)?))
        .collect()
}

fn main() -> Result<()> {
    let args = Args::parse();
    let data = parse_hex(&args.data)?;
    let port = match args.port {
        Some(p) => p,
        None => auto_detect_pirate()?,
    };
    let mut client = Client::open(&port, Duration::from_millis(500))?;
    client.set_baud(args.baud)?;
    client.reset()?;

    let wire = build_write(args.id, args.addr, &data);
    // Settle: request + empty-ack wire time, plus servo latency and USB slack.
    let wire_bytes = wire.len() as u64 + 16;
    let settle_ms = wire_bytes * 10_000 / args.baud as u64 + 4;

    let report = measure(
        &mut client,
        &wire,
        args.count,
        settle_ms,
        args.verbose,
        |ex| {
            if ex.status.result != Some(ResultCode::Ok) {
                bail!("status result {:?}", ex.status.result);
            }
            Ok(())
        },
    )?;

    println!("port         {}", client.port_path());
    println!("baud         {}", args.baud);
    println!("id           {}", args.id);
    println!("write        {} bytes @ {:#06x}", data.len(), args.addr);
    println!("exchanges    {} ok, {} fail", report.ok.len(), report.fail);
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }

    if report.fail * 10 > args.count {
        std::process::exit(1);
    }
    Ok(())
}
