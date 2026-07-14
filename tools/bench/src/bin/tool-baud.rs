//! osc-native baud switch: WRITE the servo's baud register, take the ack at
//! the OLD baud (the servo applies the change only after the ack drains,
//! protocol sec 4.2), then follow it and verify with a ping at the new rate.

use anyhow::{Result, anyhow, bail};
use bench::baud_index;
use bench::cli::{Connect, SETTLE_MS, Target};
use bench::osc::{build_ping, build_write};
use bench::run::measure;
use clap::Parser;
use osc_protocol::table::BAUD_RATE_IDX;
use osc_protocol::wire::ResultCode;

#[derive(Parser, Debug)]
#[command(about = "Switch a servo's baud and follow it (--baud = the current rate).")]
struct Args {
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    /// Target baud.
    #[arg(short, long, default_value_t = 3_000_000)]
    to: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.wire()?;
    let idx = baud_index(args.to).ok_or_else(|| anyhow!("unsupported baud {}", args.to))?;

    let write = build_write(args.target.id, BAUD_RATE_IDX, &[idx]);
    let report = measure(&mut client, &write, 1, SETTLE_MS, false, |ex| {
        if ex.status.result != Some(ResultCode::Ok) {
            bail!("write nacked: {:?}", ex.status.result);
        }
        Ok(())
    })?;
    if report.ok.is_empty() {
        bail!("no ack for the baud write at {} baud", args.conn.baud);
    }
    println!(
        "ack at {} baud ok; following to {}",
        args.conn.baud, args.to
    );

    client.set_baud(args.to)?;
    let ping = build_ping(args.target.id);
    let report = measure(&mut client, &ping, 3, SETTLE_MS, false, |ex| {
        if ex.status.result != Some(ResultCode::Ok) {
            bail!("status {:?}", ex.status.result);
        }
        Ok(())
    })?;
    if report.ok.is_empty() {
        bail!("servo did not answer at {} baud", args.to);
    }
    println!(
        "servo answering at {} baud ({} of 3 pings ok)",
        args.to,
        report.ok.len()
    );
    Ok(())
}
