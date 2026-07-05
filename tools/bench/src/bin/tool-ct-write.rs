//! Raw control-table write — bench-side sibling of `tool-ct-read` for
//! poking registers during map/validation bring-up.

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
    /// Payload bytes, hex, space- or comma-separated (e.g. "01 02 ff").
    #[arg(long)]
    data: String,
    #[arg(long, default_value_t = 1)]
    repeat: u32,
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
    let payload: Vec<u8> = args
        .data
        .split([' ', ','])
        .filter(|t| !t.is_empty())
        .map(|t| u8::from_str_radix(t.trim_start_matches("0x"), 16))
        .collect::<Result<_, _>>()?;
    let mut bus = Bus::start(BusArgs {
        port: args.port.clone(),
        target_baud: args.baud,
    })?;
    let id = bus.id();
    for _ in 0..args.repeat {
        bus.write_register(Id::new(id), args.addr, &payload)?;
    }
    println!(
        "wrote {} B at 0x{:04x} x{} ok",
        payload.len(),
        args.addr,
        args.repeat
    );
    Ok(())
}
