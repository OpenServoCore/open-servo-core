//! osc-native READ: measure TURNAROUND for large replies -- the streaming-TX
//! showcase (reply CRC + payload cost currently sits between frame end and
//! the status break).

use anyhow::{Result, bail};
use bench::cli::{Connect, Target, gate_fail_rate, print_conn};
use bench::osc::build_read;
use bench::run::{Stats, measure};
use clap::Parser;
use osc_protocol::wire::ResultCode;

#[derive(Parser, Debug)]
#[command(about = "READ a control-table span over osc-native and report TURNAROUND.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    /// Control-table address.
    #[arg(short, long, default_value_t = 0)]
    addr: u16,
    /// Bytes to read (single status frame: <= 252).
    #[arg(short, long, default_value_t = 240)]
    len: u16,
    /// Number of reads.
    #[arg(short, long, default_value_t = 50)]
    count: u32,
    /// Print a line per read.
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.wire()?;

    let wire = build_read(args.target.id, args.addr, args.len);
    // Settle: request + reply wire time at 10 bits/byte, plus servo latency
    // and USB slack.
    let wire_bytes = wire.len() as u64 + args.len as u64 + 16;
    let settle_ms = wire_bytes * 10_000 / args.conn.baud as u64 + 4;

    let want = args.len as usize;
    let verbose = args.verbose;
    let report = measure(
        &mut client,
        &wire,
        args.count,
        settle_ms,
        args.verbose,
        move |ex| {
            if ex.status.result != Some(ResultCode::Ok) {
                bail!("status result {:?}", ex.status.result);
            }
            if ex.status.payload.len() != want {
                bail!("payload {} bytes, wanted {want}", ex.status.payload.len());
            }
            if verbose {
                let hex: Vec<String> = ex
                    .status
                    .payload
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect();
                println!("payload      [{}]", hex.join(" "));
            }
            Ok(())
        },
    )?;

    print_conn(&client, args.target.id);
    println!("read         {} bytes @ {:#06x}", args.len, args.addr);
    println!("exchanges    {} ok, {} fail", report.ok.len(), report.fail);
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }
    gate_fail_rate(report.fail, args.count)
}
