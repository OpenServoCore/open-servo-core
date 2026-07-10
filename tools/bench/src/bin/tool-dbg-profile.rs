//! Scratch forensics: soak profile reads, dump the raw stamp stream of any
//! exchange that fails to parse. Delete after the 2M truncation is root-caused
//! (task #7); run with `--baud 2000000` to chase it.

use anyhow::{Result, bail};
use bench::cli::{Connect, SETTLE_MS, Target};
use bench::osc::{build_profile_config, build_read_profile, parse_exchange};
use bench::run::capture;
use clap::Parser;
use osc_protocol::wire::ResultCode;

#[derive(Parser, Debug)]
struct Args {
    #[command(flatten)]
    conn: Connect,
    #[command(flatten)]
    target: Target,
    #[arg(short, long, default_value_t = 500)]
    count: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut client = args.conn.client()?;
    let id = args.target.id;

    let config = build_profile_config(id, 0, &[(0x200, 4), (0x208, 2), (0x20C, 2)]);
    let (stamps, bit_ticks) = capture(&mut client, &config, SETTLE_MS)?;
    let ex = parse_exchange(&stamps, &config, bit_ticks);
    match ex {
        Ok(ex) if ex.status.result == Some(ResultCode::Ok) => {}
        other => bail!("config exchange: {other:?}"),
    }

    let wire = build_read_profile(id, 0);
    let mut fails = 0;
    let mut prev_snap = client.ic_snapshot()?;
    for i in 0..args.count {
        let (stamps, bit_ticks) = capture(&mut client, &wire, 50)?;
        let snap = client.ic_snapshot()?;
        let d_rx = snap.rx_total.wrapping_sub(prev_snap.rx_total);
        let d_head = snap.byte_head.wrapping_sub(prev_snap.byte_head);
        let d_fall = snap.falling_total.wrapping_sub(prev_snap.falling_total);
        let d_walked = snap.walked.wrapping_sub(prev_snap.walked);
        if i < 3 {
            println!(
                "exchange {i}: Δrx {d_rx}  Δbyte_head {d_head}  Δfalling {d_fall}  Δwalked {d_walked}  stamps {}",
                stamps.len()
            );
        }
        prev_snap = snap;
        match parse_exchange(&stamps, &wire, bit_ticks) {
            Ok(ex) if i == 0 => {
                println!("== exchange 0 OK — {} stamps ==", stamps.len());
                let hex: Vec<String> = ex
                    .status
                    .payload
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect();
                println!("  payload [{}]", hex.join(" "));
                let mut prev = None;
                for s in &stamps {
                    let d = prev.map(|p: u32| s.tick.wrapping_sub(p)).unwrap_or(0);
                    println!(
                        "  byte {:02x}  flags {:02x}  +{d} ticks ({:.2} bits)",
                        s.byte,
                        s.flags,
                        d as f64 / bit_ticks as f64
                    );
                    prev = Some(s.tick);
                }
            }
            Ok(_) => {}
            Err(e) => {
                fails += 1;
                println!("== exchange {i} FAIL: {e} — {} stamps ==", stamps.len());
                println!(
                    "  Δrx {d_rx}  Δbyte_head {d_head}  Δfalling {d_fall}  Δwalked {d_walked}  (walked lag {})",
                    prev_snap.falling_total.wrapping_sub(prev_snap.walked)
                );
                let mut prev = None;
                for s in &stamps {
                    let d = prev.map(|p: u32| s.tick.wrapping_sub(p)).unwrap_or(0);
                    println!(
                        "  byte {:02x}  flags {:02x}  tick {:>10}  +{d} ticks ({:.2} bits)",
                        s.byte,
                        s.flags,
                        s.tick,
                        d as f64 / bit_ticks as f64
                    );
                    prev = Some(s.tick);
                }
            }
        }
    }
    println!("done: {fails}/{} failed", args.count);
    Ok(())
}
