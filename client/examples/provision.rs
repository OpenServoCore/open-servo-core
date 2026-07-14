//! Fleet provisioning over USB: the first-boot story (factory-fresh servos
//! all share the board-default id) and the common-register-block health view.
//!
//!   provision factory 1 2 3 4 5      wipe saved config, reboot to defaults
//!   provision discover               ENUM walk, print UIDs
//!   provision assign 1               walk, then assign ids base.. by UID order
//!   provision ping 1 2 3 4 5         liveness sweep
//!   provision status 1 2 3 4 5       decode both common-block fronts
//!   provision deadline 5 80          write response_deadline_us (marks dirty)
//!   provision clear 1 2 3 4 5        zero the telemetry counters (rw-clear)
//!   provision save 1 2 3 4 5         persist config, clears the dirty bit
//!   provision reboot 1 2 3 4 5       MGMT REBOOT
//!   provision cal 3 400 8            CAL trains; trim lands in `status`
//!   provision migrate 3 1 2 3 4 5    servo-first baud migration + reunion sweep
//!   provision chain 50 1 2 3 4 5     GREAD chain soak, reports non-clean runs

use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::{Error, Id};
use osc_protocol::table;
use osc_protocol::wire::BaudRate;

fn ids(args: &[String]) -> Result<Vec<Id>, Box<dyn std::error::Error>> {
    let out: Result<Vec<u8>, _> = args.iter().map(|a| a.parse::<u8>()).collect();
    Ok(out?.into_iter().map(Id::new).collect())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (cmd, rest) = args.split_first().ok_or("usage: provision <cmd> [args]")?;
    let mut c = Client::connect(NusbPipe::open()?)?;

    match cmd.as_str() {
        "factory" => {
            for id in ids(rest)? {
                c.factory(id)?;
                println!("id {}: factory ack (wiped, rebooting)", id.as_byte());
            }
        }
        "discover" => {
            for uid in c.discover()? {
                println!("uid {uid:?}");
            }
        }
        "assign" => {
            let base: u8 = rest.first().ok_or("assign <base_id>")?.parse()?;
            let mut uids = c.discover()?;
            uids.sort_by_key(|u| u.0);
            println!("{} servos found", uids.len());
            for (i, uid) in uids.iter().enumerate() {
                let id = Id::new(base + i as u8);
                c.assign(uid, id)?;
                println!("uid {uid:?} -> id {}", id.as_byte());
            }
            for (i, _) in uids.iter().enumerate() {
                let id = Id::new(base + i as u8);
                match c.ping(id) {
                    Ok(p) => println!("ping {}: model {:#06x} fw {}", id.as_byte(), p.model, p.fw),
                    Err(Error::Timeout { .. }) => println!("ping {}: ABSENT", id.as_byte()),
                    Err(e) => return Err(e.into()),
                }
            }
        }
        "ping" => {
            for id in ids(rest)? {
                match c.ping(id) {
                    Ok(_) => println!("ping {}: ok", id.as_byte()),
                    Err(Error::Timeout { .. }) => println!("ping {}: ABSENT", id.as_byte()),
                    Err(e) => return Err(e.into()),
                }
            }
        }
        "status" => {
            for id in ids(rest)? {
                let cfg = c.read(id, table::CONFIG_COMMON_START, 0x14)?;
                let tel = c.read(id, table::TELEMETRY_COMMON_START, 0x0C)?;
                println!(
                    "id {}: model {:#06x} fw {} hw {} cap {:#010x} | id {} baud_idx {} deadline {}us",
                    id.as_byte(),
                    u16::from_le_bytes([cfg[0], cfg[1]]),
                    cfg[2],
                    cfg[3],
                    u32::from_le_bytes([cfg[4], cfg[5], cfg[6], cfg[7]]),
                    cfg[0x10],
                    cfg[0x11],
                    u16::from_le_bytes([cfg[0x12], cfg[0x13]]),
                );
                println!(
                    "      fault {:#04x} status {:#04x} (dirty {}) trim {} | crc_fail {} framing_drop {}",
                    tel[0],
                    tel[1],
                    (tel[1] & table::STATUS_FLAG_CONFIG_DIRTY) != 0,
                    tel[2] as i8,
                    u32::from_le_bytes([tel[4], tel[5], tel[6], tel[7]]),
                    u32::from_le_bytes([tel[8], tel[9], tel[10], tel[11]]),
                );
            }
        }
        "deadline" => {
            let id = Id::new(rest.first().ok_or("deadline <id> <us>")?.parse()?);
            let us: u16 = rest.get(1).ok_or("deadline <id> <us>")?.parse()?;
            c.write(id, table::RESPONSE_DEADLINE_US, &us.to_le_bytes())?;
            println!("id {}: response_deadline_us = {us}", id.as_byte());
        }
        "clear" => {
            for id in ids(rest)? {
                c.write(id, table::CRC_FAIL_COUNT, &0u32.to_le_bytes())?;
                c.write(id, table::FRAMING_DROP_COUNT, &0u32.to_le_bytes())?;
                println!("id {}: counters cleared", id.as_byte());
            }
        }
        "save" => {
            for id in ids(rest)? {
                c.save(id)?;
                println!("id {}: save ack", id.as_byte());
            }
        }
        "reboot" => {
            for id in ids(rest)? {
                c.reboot(id)?;
                println!("id {}: reboot ack", id.as_byte());
            }
        }
        "migrate" => {
            let idx: u8 = rest.first().ok_or("migrate <baud_idx> <id...>")?.parse()?;
            let rate = BaudRate::from_idx(idx).ok_or("bad baud idx")?;
            let targets = ids(&rest[1..])?;
            // Servo-first (they ack at the old rate, apply deferred), then
            // the host follows and the sweep proves the reunion.
            for &id in &targets {
                c.write(id, table::BAUD_RATE_IDX, &[idx])?;
            }
            c.host_baud(rate)?;
            for &id in &targets {
                match c.ping(id) {
                    Ok(_) => println!("ping {} at {}: ok", id.as_byte(), rate.as_hz()),
                    Err(Error::Timeout { .. }) => println!("ping {}: ABSENT", id.as_byte()),
                    Err(e) => return Err(e.into()),
                }
            }
        }
        "chain" => {
            let n: u32 = rest.first().ok_or("chain <count> <id...>")?.parse()?;
            let targets = ids(&rest[1..])?;
            let mut bad = 0u32;
            for i in 0..n {
                let chain = c.gread(&targets, 392, 4)?; // GOAL_VELOCITY, V006 map
                let clean = chain.timeout_slot.is_none()
                    && chain.statuses.len() == targets.len()
                    && chain
                        .statuses
                        .iter()
                        .all(|s| s.result == Some(osc_protocol::wire::ResultCode::Ok));
                if !clean {
                    bad += 1;
                    println!(
                        "chain {i}: {} statuses, timeout_slot {:?}",
                        chain.statuses.len(),
                        chain.timeout_slot
                    );
                }
            }
            println!("chains: {}/{n} clean", n - bad);
        }
        "cal" => {
            let trains: u8 = rest
                .first()
                .ok_or("cal <trains> <gap_us> <gaps>")?
                .parse()?;
            let gap_us: u16 = rest.get(1).ok_or("cal <trains> <gap_us> <gaps>")?.parse()?;
            let gaps: u8 = rest.get(2).ok_or("cal <trains> <gap_us> <gaps>")?.parse()?;
            c.cal(trains, gap_us, gaps)?;
            println!("cal: {trains} trains of {gaps} x {gap_us}us sent");
        }
        other => return Err(format!("unknown command {other}").into()),
    }
    Ok(())
}
