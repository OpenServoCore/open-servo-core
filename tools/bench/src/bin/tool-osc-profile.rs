//! osc-native PROFILE read (§5.2): configure a profile slot over the wire,
//! then measure TURNAROUND for the gathered scattered-telemetry reply and
//! compare against the same bytes as a plain contiguous READ.

use std::time::Duration;

use anyhow::{Context, Result, bail};
use bench::osc::{build_profile_config, build_read, build_read_profile};
use bench::pirate::{Client, auto_detect_pirate};
use bench::run::{Stats, measure};
use clap::Parser;
use osc_protocol::wire::ResultCode;

#[derive(Parser, Debug)]
#[command(about = "Configure a profile slot, then measure PROFILE-read TURNAROUND.")]
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
    /// Profile slot to configure and read.
    #[arg(short, long, default_value_t = 0)]
    slot: u8,
    /// Spans as `addr:count` pairs. Default: scattered telemetry —
    /// position(4) + current(2) + vbus(2) from the converted block.
    #[arg(long, value_delimiter = ',', default_values_t =
        ["0x200:4".to_string(), "0x208:2".to_string(), "0x20C:2".to_string()])]
    spans: Vec<String>,
    /// Number of profile reads.
    #[arg(short, long, default_value_t = 50)]
    count: u32,
    /// Print a line per read.
    #[arg(short, long)]
    verbose: bool,
}

fn parse_span(s: &str) -> Result<(u16, u8)> {
    let (a, c) = s.split_once(':').context("span is `addr:count`")?;
    let addr = if let Some(hex) = a.strip_prefix("0x") {
        u16::from_str_radix(hex, 16)?
    } else {
        a.parse()?
    };
    let count: u8 = c.parse()?;
    if addr > 1023 || count == 0 || count > 63 {
        bail!("span {s}: addr caps at 1023, count at 1..=63");
    }
    Ok((addr, count))
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

    let spans: Vec<(u16, u8)> = args
        .spans
        .iter()
        .map(|s| parse_span(s))
        .collect::<Result<_>>()?;
    let total: usize = spans.iter().map(|&(_, c)| c as usize).sum();

    // Configure the slot (one ordinary WRITE, §5.2) and take the ack.
    let config = build_profile_config(args.id, args.slot, &spans);
    measure(&mut client, &config, 1, 5, false, |ex| {
        if ex.status.result != Some(ResultCode::Ok) {
            bail!("profile config nacked: {:?}", ex.status.result);
        }
        Ok(())
    })
    .context("profile slot config")?;

    let check = move |len: usize| {
        move |ex: &bench::osc::Exchange| {
            if ex.status.result != Some(ResultCode::Ok) {
                bail!("status result {:?}", ex.status.result);
            }
            if ex.status.payload.len() != len {
                bail!("payload {} bytes, wanted {len}", ex.status.payload.len());
            }
            Ok(())
        }
    };

    // Settle: request + reply wire time at 10 bits/byte, plus servo latency
    // and USB slack.
    let settle_ms = (24 + total as u64) * 10_000 / args.baud as u64 + 4;

    let wire = build_read_profile(args.id, args.slot);
    let profile = measure(
        &mut client,
        &wire,
        args.count,
        settle_ms,
        args.verbose,
        check(total),
    )?;

    // Reference: the same byte count as one contiguous plain READ (the spans'
    // first addr keeps the payload identical in size, not content).
    let wire = build_read(args.id, spans[0].0, total as u16);
    let plain = measure(
        &mut client,
        &wire,
        args.count,
        settle_ms,
        args.verbose,
        check(total),
    )?;

    println!("port         {}", client.port_path());
    println!("baud         {}", args.baud);
    println!("id           {}", args.id);
    println!(
        "profile      slot {} = {} spans, {} bytes",
        args.slot,
        spans.len(),
        total
    );
    println!(
        "exchanges    profile {} ok {} fail · plain {} ok {} fail",
        profile.ok.len(),
        profile.fail,
        plain.ok.len(),
        plain.fail
    );
    println!("-- profile read --");
    if let Some(s) = Stats::from(&profile.ok) {
        s.print();
    }
    println!("-- plain read ({total} B contiguous) --");
    if let Some(s) = Stats::from(&plain.ok) {
        s.print();
    }

    if (profile.fail + plain.fail) * 10 > args.count {
        std::process::exit(1);
    }
    Ok(())
}
