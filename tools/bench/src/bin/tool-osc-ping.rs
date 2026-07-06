//! osc-native ping: measure TURNAROUND — the project's success metric. Pings a
//! servo `count` times over the pirate and reports the turnaround distribution.

use std::time::Duration;

use anyhow::{Result, bail};
use bench::osc::build_ping;
use bench::pirate::{Client, auto_detect_pirate};
use bench::run::{Stats, measure};
use clap::Parser;
use osc_protocol::wire::ResultCode;

#[derive(Parser, Debug)]
#[command(about = "Ping a servo over osc-native and report TURNAROUND.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Wire baud.
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
    /// Servo id to ping.
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    /// Number of pings.
    #[arg(short, long, default_value_t = 50)]
    count: u32,
    /// Print a line per ping.
    #[arg(short, long)]
    verbose: bool,
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

    let ping = build_ping(args.id);
    let report = measure(&mut client, &ping, args.count, 5, args.verbose, |ex| {
        if ex.status.result != Some(ResultCode::Ok) {
            bail!("status result {:?}", ex.status.result);
        }
        Ok(())
    })?;

    println!("port         {}", client.port_path());
    println!("baud         {}", args.baud);
    println!("id           {}", args.id);
    println!("pings        {} ok, {} fail", report.ok.len(), report.fail);
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }

    if report.fail * 10 > args.count {
        std::process::exit(1);
    }
    Ok(())
}
