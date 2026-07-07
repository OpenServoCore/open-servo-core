//! Shared measurement loop for the osc turnaround tools: send one
//! instruction repeatedly, parse each exchange from the stamp stream, and
//! report the TURNAROUND distribution (see [`crate::osc`] for the metric).

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, anyhow};

use crate::osc::{Exchange, parse_exchange};
use crate::pirate::{BStamp, Client};

pub struct Report {
    pub ok: Vec<f64>,
    pub fail: u32,
}

/// Run `count` exchanges of `wire` and collect turnaround in µs. `settle_ms`
/// must cover the reply's wire time plus servo latency. `check` validates the
/// decoded exchange (payload length, result code) before it counts.
pub fn measure(
    client: &mut Client,
    wire: &[u8],
    count: u32,
    settle_ms: u64,
    verbose: bool,
    check: impl Fn(&Exchange) -> Result<()>,
) -> Result<Report> {
    let hz_per_us = client.hz_per_us()?;
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / client.current_baud() as u64) as u32;

    let mut ok = Vec::new();
    let mut fail = 0u32;
    for i in 0..count {
        // The first exchange after a reset is a known chip flake — retry once.
        let retries = if i == 0 { 1 } else { 0 };
        let mut result = Err(anyhow!("no attempt"));
        for _ in 0..=retries {
            result = attempt(client, wire, bit_ticks, settle_ms, &check);
            if result.is_ok() {
                break;
            }
        }
        match result {
            Ok(ticks) => {
                let us = ticks as f64 / hz_per_us as f64;
                ok.push(us);
                if verbose {
                    println!("exchange {i:>4}  turnaround {us:8.2} us");
                }
            }
            Err(e) => {
                fail += 1;
                if verbose {
                    println!("exchange {i:>4}  FAIL: {e}");
                }
            }
        }
    }
    Ok(Report { ok, fail })
}

fn attempt(
    client: &mut Client,
    wire: &[u8],
    bit_ticks: u32,
    settle_ms: u64,
    check: &impl Fn(&Exchange) -> Result<()>,
) -> Result<u32> {
    drain(client)?;
    client.brksend(wire)?;
    sleep(Duration::from_millis(settle_ms));
    let stamps = drain(client)?;
    let ex = parse_exchange(&stamps, wire, bit_ticks).map_err(|e| anyhow!("{e}"))?;
    check(&ex)?;
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

pub struct Stats {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub stddev: f64,
}

impl Stats {
    pub fn from(xs: &[f64]) -> Option<Stats> {
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

    pub fn print(&self) {
        println!("turnaround   min {:.2} us", self.min);
        println!("             mean {:.2} us", self.mean);
        println!("             max {:.2} us", self.max);
        println!("             stddev {:.2} us", self.stddev);
    }
}
