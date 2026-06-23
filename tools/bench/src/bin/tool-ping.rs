//! Discovery + sanity-check ping. Auto-detects the pirate, sweeps known
//! bauds to find the servo, sets it to `--baud`, then prints id / model /
//! firmware version for the responder.

use anyhow::{Result, anyhow};
use bench::{Session, SessionArgs, build_ping, decode_ping_status};
use clap::Parser;
use dxl_protocol::types::Id;

#[derive(Parser, Debug)]
#[command(about = "Discover + ping a servo via the pirate.")]
struct Args {
    /// Pirate USB-CDC device. Default: autodetect by VID/PID (0xC0DE/0xCAFE).
    #[arg(short, long)]
    port: Option<String>,
    /// Wire baud to leave the bus at. Servo is switched to this baud if it's
    /// currently somewhere else.
    #[arg(short, long, default_value_t = 1_000_000)]
    baud: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut session = Session::start(SessionArgs {
        port: args.port,
        target_baud: args.baud,
    })?;

    let frame = build_ping(Id::new(session.id))?;
    let reply = session
        .pirate
        .xfer(&frame, 2_000)?
        .ok_or_else(|| anyhow!("post-discovery ping timed out"))?;
    let info = decode_ping_status(&reply, Some(Id::new(session.id)))?;

    println!("port            {}", session.pirate.port_path());
    println!("baud            {}", session.baud);
    println!("id              {}", info.id);
    println!("model_number    0x{:04X}  ({})", info.model, info.model);
    println!("firmware_ver    0x{:02X}    ({})", info.fw_ver, info.fw_ver);
    println!("error           0x{:02X}", info.error);
    Ok(())
}
