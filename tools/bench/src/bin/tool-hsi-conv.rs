//! Bench the chip's autonomous HSI-trim convergence under plain Ping load.
//!
//! The chip owns its clock trim: it watches inter-byte timing on every
//! non-Status packet and nudges HSITRIM toward zero drift on its own (no
//! master CAL — see `firmware/lib/drivers/src/dxl/uart/clock.rs`). A boot
//! batch lands the bulk of the factory drift in the first reply; steady
//! batches squeeze the residual.
//!
//! This tool reboots the chip to its factory-default HSI, drives Pings, and
//! periodically probes the residual drift via the pirate's stable-clock
//! reply-span measurement. Exits nonzero if the trim doesn't settle inside
//! `1.5 * DRIFT_STEP_PPM` within `--max-pings`, so it doubles as a
//! regression check.

use std::time::Duration;

use anyhow::{Result, anyhow};
use bench::drift::DRIFT_STEP_PPM;
use bench::{BOOT_BAUD, Bus, BusArgs, DEFAULT_IDLE_US, build_ping};
use clap::Parser;
use dxl_protocol::types::Id;

#[derive(Parser, Debug)]
#[command(about = "Probe the chip's HSI-trim convergence curve.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    port: Option<String>,
    /// Baud to drive convergence at. Cold drift is always probed at the boot
    /// baud (1M); drift is baud-independent.
    #[arg(short, long, default_value_t = BOOT_BAUD)]
    baud: u32,
    /// Give up if the trim hasn't settled by this many Pings.
    #[arg(long, default_value_t = 2000)]
    max_pings: u32,
    /// Pings driven between drift probes.
    #[arg(long, default_value_t = 20)]
    probe_every: u32,
    /// Long-Read probes averaged per drift measurement.
    #[arg(long, default_value_t = 8)]
    probe_samples: u32,
    /// Consecutive in-bound probes required to declare convergence.
    #[arg(long, default_value_t = 3)]
    converge_streak: u32,
    /// Probe + print indefinitely; no convergence early-exit, `--max-pings`
    /// ignored. Ctrl-C to stop. Pair with `--probe-every 1` to watch
    /// per-ping drift evolve.
    #[arg(long)]
    forever: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let bound = 1.5 * DRIFT_STEP_PPM;

    let mut bus = Bus::start(BusArgs {
        port: args.port,
        target_baud: BOOT_BAUD,
    })?;
    let id = Id::new(bus.id());
    println!(
        "pirate: {}   chip id: {}   bound: ±{bound:.0} ppm",
        bus.port_path(),
        bus.id(),
    );

    println!("\n[reboot → factory-default HSI]");
    bus.reboot_chip()?;

    let cold = match bus.probe_drift_ppm(1) {
        Ok(v) => {
            println!("  cold drift   = {v:+.0} ppm");
            Some(v)
        }
        Err(e) => {
            println!("  cold drift   = n/a ({e})");
            None
        }
    };

    // First post-reboot instruction trips the boot-phase trim: a one-shot
    // full-envelope correction, distinct from the steady ±4-step nudges
    // that follow. Probe (at boot baud, before any switch) to isolate
    // what that single boot correction left.
    let _ = bus.xfer_reply(&build_ping(id)?, DEFAULT_IDLE_US)?;
    match bus.probe_drift_ppm(1) {
        Ok(v) => println!("  after 1 ping = {v:+.0} ppm"),
        Err(e) => println!("  after 1 ping = n/a ({e})"),
    }

    if args.baud != BOOT_BAUD {
        bus.set_chip_baud(args.baud)?;
    }

    println!("\n[converge @ {}]", args.baud);
    println!("  {:>6}  {:>10}", "pings", "drift_ppm");

    let mut pings: u32 = 0;
    let mut streak: u32 = 0;
    let mut converged_at: Option<u32> = None;

    loop {
        if !args.forever && pings >= args.max_pings {
            break;
        }
        for _ in 0..args.probe_every {
            let _ = bus.xfer_reply(&build_ping(id)?, DEFAULT_IDLE_US)?;
        }
        pings += args.probe_every;

        match bus.probe_drift_ppm(args.probe_samples) {
            Ok(drift) => {
                let flag = if drift.abs() <= bound { "ok" } else { "  " };
                println!("  {pings:>6}  {drift:>+10.0}  {flag}");
                if args.forever {
                    continue;
                }
                if drift.abs() <= bound {
                    streak += 1;
                    if streak >= args.converge_streak {
                        converged_at = Some(pings);
                        break;
                    }
                } else {
                    streak = 0;
                }
            }
            Err(e) => {
                println!("  {pings:>6}  {:>10}  ({e})", "n/a");
                streak = 0;
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    let cold_str = match cold {
        Some(v) => format!("{v:+.0} ppm"),
        None => "n/a".to_string(),
    };
    println!();
    if args.forever {
        println!("ran {pings} pings (cold {cold_str}); no verdict in --forever mode");
        Ok(())
    } else if let Some(at) = converged_at {
        println!("converged within ±{bound:.0} ppm after ~{at} pings (cold {cold_str})");
        Ok(())
    } else {
        Err(anyhow!(
            "did NOT converge within ±{bound:.0} ppm in {} pings (cold {cold_str})",
            args.max_pings
        ))
    }
}
