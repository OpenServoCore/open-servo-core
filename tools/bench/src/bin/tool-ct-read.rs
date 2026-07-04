//! Raw control-table window dump — bench-side ground truth for region
//! address-map questions (e.g. where the telemetry link counters actually
//! sit vs where `LinkCounters` reads them).

use anyhow::Result;
use bench::{Bus, BusArgs};
use clap::Parser;
use dxl_protocol::Id;

#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = 1_000_000)]
    baud: u32,
    #[arg(long)]
    port: Option<String>,
    /// Start address (hex accepted with 0x prefix).
    #[arg(long, value_parser = parse_u16)]
    addr: u16,
    #[arg(long, default_value_t = 64)]
    len: u16,
}

fn parse_u16(s: &str) -> Result<u16, String> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x") {
        u16::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse()
            .map_err(|e: std::num::ParseIntError| e.to_string())
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut bus = Bus::start(BusArgs {
        port: args.port.clone(),
        target_baud: args.baud,
    })?;
    let id = bus.id();
    let data = bus.read_register(Id::new(id), args.addr, args.len)?;
    for (row, chunk) in data.chunks(16).enumerate() {
        let base = args.addr as usize + row * 16;
        let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
        println!("0x{base:04x}: {}", hex.join(" "));
    }
    Ok(())
}
