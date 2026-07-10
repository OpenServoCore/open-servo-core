//! Shared CLI plumbing for the bench binaries: the connection args every
//! tool repeats, the bootstrap that turns them into a ready pirate
//! [`Client`], and the common report pieces.

use std::time::Duration;

use anyhow::{Result, bail};
use clap::Args;

use crate::pirate::{Client, auto_detect_pirate};

/// Pirate USB-CDC open timeout.
pub const OPEN_TIMEOUT: Duration = Duration::from_millis(500);

/// Standard per-exchange settle window (ms): covers the reply's wire time
/// plus servo latency at every supported baud, with margin.
pub const SETTLE_MS: u64 = 5;

/// Open a pirate by explicit path or VID/PID autodetect.
pub fn open_pirate(port: Option<&str>) -> Result<Client> {
    let port = match port {
        Some(p) => p.to_string(),
        None => auto_detect_pirate()?,
    };
    Client::open(&port, OPEN_TIMEOUT)
}

/// Connection args shared by every wire-talking bench binary.
#[derive(Args, Debug)]
pub struct Connect {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID.
    #[arg(short, long)]
    pub port: Option<String>,
    /// Wire baud.
    #[arg(short, long, default_value_t = crate::BOOT_BAUD)]
    pub baud: u32,
}

impl Connect {
    /// Open the pirate at the wire baud with stale capture state cleared.
    pub fn client(&self) -> Result<Client> {
        let mut client = open_pirate(self.port.as_deref())?;
        client.set_baud(self.baud)?;
        client.reset()?;
        Ok(client)
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
pub fn print_conn(client: &Client, id: u8) {
    println!("port         {}", client.port_path());
    println!("baud         {}", client.current_baud());
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
