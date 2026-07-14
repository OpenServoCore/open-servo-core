//! Shared CLI plumbing for the bench binaries: the connection args every
//! tool repeats, the bootstrap that turns them into a ready adapter
//! [`Wire`], and the common report pieces.

use anyhow::{Result, bail};
use clap::Args;

use crate::wire::Wire;

/// Standard per-exchange settle window (ms): covers the reply's wire time
/// plus servo latency at every supported baud, with margin.
pub const SETTLE_MS: u64 = 5;

/// Connection args shared by every wire-talking bench binary.
#[derive(Args, Debug)]
pub struct Connect {
    /// Wire baud.
    #[arg(short, long, default_value_t = crate::BOOT_BAUD)]
    pub baud: u32,
}

impl Connect {
    /// Claim the adapter (VID/PID autodetect) at the wire baud with
    /// capture state cleared.
    pub fn wire(&self) -> Result<Wire> {
        Wire::connect(self.baud)
    }
}

/// Servo target arg shared by the single-servo tools.
#[derive(Args, Debug)]
pub struct Target {
    /// Servo id.
    #[arg(short, long, default_value_t = 1)]
    pub id: u8,
}

/// Standard measurement report header: where, how fast, who.
pub fn print_conn(w: &Wire, id: u8) {
    println!("adapter      osc-adapter (usb)");
    println!("baud         {}", w.current_baud());
    println!("id           {id}");
}

/// The measurement tools' shared pass/fail verdict: error out when more
/// than 10% of `count` exchanges failed.
pub fn gate_fail_rate(failures: u32, count: u32) -> Result<()> {
    if failures * 10 > count {
        bail!("failure rate over 10% ({failures}/{count})");
    }
    Ok(())
}

/// Parse a hex byte string (whitespace tolerated) into bytes.
pub fn parse_hex(s: &str) -> Result<Vec<u8>> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !s.len().is_multiple_of(2) {
        bail!("hex payload needs an even digit count");
    }
    (0..s.len())
        .step_by(2)
        .map(|i| Ok(u8::from_str_radix(&s[i..i + 2], 16)?))
        .collect()
}

/// Parse a `u16`, accepting a `0x` hex prefix.
pub fn parse_u16(s: &str) -> Result<u16> {
    Ok(match s.strip_prefix("0x") {
        Some(hex) => u16::from_str_radix(hex, 16)?,
        None => s.parse()?,
    })
}

/// Parse one profile span given as `addr:count` (protocol sec 5.2 bounds).
pub fn parse_span(s: &str) -> Result<(u16, u8)> {
    let (a, c) = s
        .split_once(':')
        .ok_or_else(|| anyhow::anyhow!("span is `addr:count`"))?;
    let addr = parse_u16(a)?;
    let count: u8 = c.parse()?;
    if addr > 1023 || count == 0 || count > 63 {
        bail!("span {s}: addr caps at 1023, count at 1..=63");
    }
    Ok((addr, count))
}
