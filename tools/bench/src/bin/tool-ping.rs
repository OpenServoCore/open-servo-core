//! osc-native ping: measure TURNAROUND -- the project's success metric. Pings a
//! servo `count` times over the pirate and reports the turnaround distribution.

use anyhow::{Result, bail};
use bench::cli::{Connect, SETTLE_MS, Target, gate_fail_rate, print_conn};
use bench::osc::build_ping;
use bench::run::{Stats, measure};
use clap::Parser;
use osc_protocol::wire::ResultCode;

#[derive(Parser, Debug)]
#[command(about = "Ping a servo over osc-native and report TURNAROUND.")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    /// Number of pings.
    #[arg(short, long, default_value_t = 50)]
    count: u32,
    /// Print a line per ping.
    #[arg(short, long)]
    verbose: bool,
    /// Corrupt the wire CRC: every ping must draw SILENCE (protocol sec 5.3 layer 1 --
    /// a bad frame is never answered) and bump the servo's crc_fail counter.
    #[arg(long, default_value_t = false)]
    corrupt: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.wire()?;

    let mut ping = build_ping(args.target.id);
    if args.corrupt {
        let last = ping.len() - 1;
        ping[last] ^= 0xFF;
    }
    let report = measure(
        &mut client,
        &ping,
        args.count,
        SETTLE_MS,
        args.verbose,
        |ex| {
            if ex.status.result != Some(ResultCode::Ok) {
                bail!("status result {:?}", ex.status.result);
            }
            Ok(())
        },
    )?;

    print_conn(&client, args.target.id);
    println!("pings        {} ok, {} fail", report.ok.len(), report.fail);
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }
    gate_fail_rate(report.fail, args.count)
}
