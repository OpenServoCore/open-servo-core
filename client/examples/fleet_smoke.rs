//! Live-bench smoke for landing 1: connect over USB, sweep the fleet, chain
//! it, and run the first real ENUM walk. Read-only against the bus (pings,
//! reads, broadcast ENUM probes).

use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::{Error, Id};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut c = Client::connect(NusbPipe::open()?)?;
    let info = c.info();
    println!(
        "INFO: link v{}, {} ticks/us",
        info.version, info.ticks_per_us
    );

    let mut fleet = Vec::new();
    for raw in 1u8..=5 {
        let id = Id::new(raw);
        match c.ping(id) {
            Ok(p) => {
                println!(
                    "ping {raw}: model {:#06x} fw {} alert {}",
                    p.model, p.fw, p.alert
                );
                fleet.push(id);
            }
            Err(Error::Timeout { .. }) => println!("ping {raw}: absent"),
            Err(e) => return Err(e.into()),
        }
    }

    if fleet.len() > 1 {
        let chain = c.gread(&fleet, 392, 4)?; // GOAL_VELOCITY, V006 map
        println!(
            "chain: {} statuses, timeout_slot {:?}",
            chain.statuses.len(),
            chain.timeout_slot
        );
    }

    println!("discover: walking...");
    let uids = c.discover()?;
    for uid in &uids {
        println!("  uid {uid:?}");
    }
    println!(
        "discover: {} servos found vs {} pinged",
        uids.len(),
        fleet.len()
    );
    Ok(())
}
