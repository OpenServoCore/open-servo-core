//! osc-native baud switch: WRITE the servo's baud register, take the ack at
//! the OLD baud (the servo applies the change only after the ack drains,
//! §4.2), then follow it and verify with a ping at the new rate.

use std::time::Duration;

use anyhow::{Result, bail};
use bench::osc::{build_ping, build_write};
use bench::pirate::{Client, auto_detect_pirate};
use bench::run::measure;
use clap::Parser;
use osc_protocol::wire::ResultCode;

/// Control-table address of `baud_rate_idx` (osc-core
/// `regions::config::addr::comms::BAUD_RATE_IDX`; value pinned here to keep
/// the heavy core crate out of the bench build).
const BAUD_RATE_IDX_ADDR: u16 = 0x000D;

#[derive(Parser, Debug)]
#[command(about = "Switch a servo's baud and follow it.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Baud the servo currently listens at.
    #[arg(short, long, default_value_t = 1_000_000)]
    from: u32,
    /// Target baud.
    #[arg(short, long, default_value_t = 3_000_000)]
    to: u32,
    /// Servo id.
    #[arg(short, long, default_value_t = 1)]
    id: u8,
}

fn baud_index(bps: u32) -> Result<u8> {
    Ok(match bps {
        500_000 => 0,
        1_000_000 => 1,
        2_000_000 => 2,
        3_000_000 => 3,
        _ => bail!("unsupported baud {bps}"),
    })
}

fn main() -> Result<()> {
    let args = Args::parse();
    let port = match args.port {
        Some(p) => p,
        None => auto_detect_pirate()?,
    };
    let mut client = Client::open(&port, Duration::from_millis(500))?;
    client.set_baud(args.from)?;
    client.reset()?;

    let write = build_write(args.id, BAUD_RATE_IDX_ADDR, &[baud_index(args.to)?]);
    let report = measure(&mut client, &write, 1, 5, false, |ex| {
        if ex.status.result != Some(ResultCode::Ok) {
            bail!("write nacked: {:?}", ex.status.result);
        }
        Ok(())
    })?;
    if report.ok.is_empty() {
        bail!("no ack for the baud write at {} baud", args.from);
    }
    println!("ack at {} baud ok; following to {}", args.from, args.to);

    client.set_baud(args.to)?;
    client.reset()?;
    let ping = build_ping(args.id);
    let report = measure(&mut client, &ping, 3, 5, false, |ex| {
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
