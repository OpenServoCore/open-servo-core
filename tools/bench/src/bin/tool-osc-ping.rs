//! osc-native ping: measure TURNAROUND — the project's success metric. Pings a
//! servo `count` times over the pirate and reports the turnaround distribution.

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, anyhow};
use bench::osc::{build_ping, parse_exchange};
use bench::pirate::{BStamp, Client, auto_detect_pirate};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(about = "Ping a servo over osc-native and report TURNAROUND.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID (0xC0DE/0xCAFE).
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

    let hz_per_us = client.hz_per_us()?;
    // Ticks per wire bit: full-rate ticks (hz_per_us per µs) divided by baud.
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / args.baud as u64) as u32;
    let ping = build_ping(args.id);

    let mut turnarounds_us: Vec<f64> = Vec::new();
    let mut fail = 0u32;
    for i in 0..args.count {
        // The first exchange after a reset is a known chip flake — retry once.
        let retries = if i == 0 { 1 } else { 0 };
        match ping_once(&mut client, &ping, bit_ticks, retries) {
            Ok(ticks) => {
                let us = ticks as f64 / hz_per_us as f64;
                turnarounds_us.push(us);
                if args.verbose {
                    println!("ping {i:>4}  turnaround {us:8.2} us");
                }
            }
            Err(e) => {
                fail += 1;
                if args.verbose {
                    println!("ping {i:>4}  FAIL: {e}");
                }
            }
        }
    }

    let ok = turnarounds_us.len();
    println!("port         {}", client.port_path());
    println!("baud         {}", args.baud);
    println!("id           {}", args.id);
    println!("pings        {} ok, {} fail", ok, fail);
    if let Some(s) = Stats::from(&turnarounds_us) {
        println!("turnaround   min {:.2} us", s.min);
        println!("             mean {:.2} us", s.mean);
        println!("             max {:.2} us", s.max);
        println!("             stddev {:.2} us", s.stddev);
    }

    if fail * 10 > args.count {
        std::process::exit(1);
    }
    Ok(())
}

/// One ping: drain stale stamps, transmit, wait for the exchange to settle,
/// collect stamps, parse. Retries the whole attempt `retries` more times.
fn ping_once(client: &mut Client, ping: &[u8], bit_ticks: u32, retries: u32) -> Result<u32> {
    let mut last_err = anyhow!("no attempt");
    for _ in 0..=retries {
        match ping_attempt(client, ping, bit_ticks) {
            Ok(ticks) => return Ok(ticks),
            Err(e) => last_err = e,
        }
    }
    Err(last_err)
}

fn ping_attempt(client: &mut Client, ping: &[u8], bit_ticks: u32) -> Result<u32> {
    drain(client)?;
    // "ERR busy" is a pirate-side transport nuisance (poll-fed TX losing a
    // TXE bound to walker/USB ISR bursts, ~2%): the send never launched, so
    // one retry measures nothing falsely. TODO: bump the pirate's TXE poll
    // bound next ISP flash and drop this.
    if let Err(e) = client.brksend(ping) {
        if !e.to_string().contains("busy") {
            return Err(e);
        }
        sleep(Duration::from_millis(2));
        client.brksend(ping)?;
    }
    sleep(Duration::from_millis(5));
    let stamps = drain(client)?;
    let ex = parse_exchange(&stamps, ping, bit_ticks).map_err(|e| anyhow!("{e}"))?;
    Ok(ex.turnaround_ticks)
}

fn drain(client: &mut Client) -> Result<Vec<BStamp>> {
    let mut all = Vec::new();
    loop {
        let batch = client.bbatch(255)?;
        if batch.is_empty() {
            return Ok(all);
        }
        all.extend(batch);
    }
}

struct Stats {
    min: f64,
    max: f64,
    mean: f64,
    stddev: f64,
}

impl Stats {
    fn from(xs: &[f64]) -> Option<Stats> {
        if xs.is_empty() {
            return None;
        }
        let n = xs.len() as f64;
        let mean = xs.iter().sum::<f64>() / n;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
        Some(Stats {
            min: xs.iter().cloned().fold(f64::INFINITY, f64::min),
            max: xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
            mean,
            stddev: var.sqrt(),
        })
    }
}
