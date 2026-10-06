//! osc-native WRITE: measure TURNAROUND for the mutating path -- the write is
//! staged through the LOW consumer at the covered checkpoint (decode +
//! validate overlap the frame's wire tail), and the verdict at the frame end
//! commits + sequences the ack break.

use anyhow::{Result, bail};
use bench::cli::{Connect, Target, gate_fail_rate, parse_addr, parse_hex, print_conn};
use bench::osc::build_write;
use bench::run::{Stats, measure};
use clap::Parser;
use osc_protocol::wire::ResultCode;

#[derive(Parser, Debug)]
#[command(about = "WRITE a control-table span over osc-native and report TURNAROUND.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    /// Control-table address: decimal, 0x-hex, or a field name. The default
    /// is the hot-loop register, whose ge/le soft-limit rules make it the
    /// representative write-validation workload.
    #[arg(short, long, default_value = "goal_position", value_parser = parse_addr)]
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

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.wire()?;

    let data = parse_hex(&args.data)?;
    let wire = build_write(args.target.id, args.addr, &data);
    // Settle: request + empty-ack wire time, plus servo latency and USB slack.
    let wire_bytes = wire.len() as u64 + 16;
    let settle_ms = wire_bytes * 10_000 / args.conn.baud as u64 + 4;
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

    print_conn(&client, args.target.id);
    println!("write        {} bytes @ {:#06x}", data.len(), args.addr);
    println!("exchanges    {} ok, {} fail", report.ok.len(), report.fail);
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }
    gate_fail_rate(report.fail, args.count)
}
