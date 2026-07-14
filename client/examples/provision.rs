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
//!   provision cal 3 400 8 1 2 3 4 5  CAL trains + per-servo convergence trace
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
                let i = c.identity(id)?;
                let h = c.health(id)?;
                println!(
                    "id {}: model {:#06x} fw {} hw {} cap {:#010x}",
                    id.as_byte(),
                    i.model,
                    i.fw,
                    i.hw,
                    i.capabilities,
                );
                println!(
                    "      fault {:#04x} dirty {} trim {} | crc_fail {} framing_drop {}",
                    h.fault_flags,
                    h.config_dirty,
                    h.trim_steps,
                    h.crc_fail_count,
                    h.framing_drop_count,
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
                c.clear_counters(id)?;
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
            for (id, alive) in c.set_baud(&targets, rate)? {
                let verdict = if alive { "ok" } else { "ABSENT" };
                println!("ping {} at {}: {verdict}", id.as_byte(), rate.as_hz());
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
            let usage = "cal <trains> <gap_us> <gaps> <id...>";
            let trains: u8 = rest.first().ok_or(usage)?.parse()?;
            let gap_us: u16 = rest.get(1).ok_or(usage)?.parse()?;
            let gaps: u8 = rest.get(2).ok_or(usage)?.parse()?;
            let targets = ids(&rest[3..])?;
            for t in c.cal_verify(&targets, trains, gap_us, gaps)? {
                let verdict = if t.converged() { "converged" } else { "MOVING" };
                println!("id {}: trims {:?} {verdict}", t.id.as_byte(), t.trims);
            }
        }
        other => return Err(format!("unknown command {other}").into()),
    }
    Ok(())
}
