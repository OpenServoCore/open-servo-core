//! Fleet-mute characterization driver (transport open item): migrate toward
//! a target rate, then round-robin ping with timestamps until every servo
//! answers -- per-servo dead time, over N migration cycles.
//!
//!   mute_probe cycles <n> <id...>       1M -> 3M -> 1M cycles, dead times
//!   mute_probe order <n> <id...>        reversed write order each odd cycle
//!   mute_probe solo <n> <id>            single-servo migrate (no cross-baud)
//!   mute_probe rescue <id...>           mute the fleet, then rescue mid-mute

use std::time::{Duration, Instant};

use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::{BaudRate, Error, Id};
use osc_protocol::table;

/// Round-robin ping until every id answers; per-id dead time from `t0`.
fn wait_reunion(
    c: &mut Client<NusbPipe>,
    ids: &[Id],
    t0: Instant,
    cap: Duration,
) -> Result<Vec<(Id, Option<Duration>)>, Error> {
    let mut dead: Vec<(Id, Option<Duration>)> = ids.iter().map(|&id| (id, None)).collect();
    while dead.iter().any(|(_, d)| d.is_none()) && t0.elapsed() < cap {
        for slot in dead.iter_mut().filter(|(_, d)| d.is_none()) {
            match c.ping(slot.0) {
                Ok(_) => slot.1 = Some(t0.elapsed()),
                Err(Error::Timeout { .. }) => {}
                Err(e) => return Err(e),
            }
        }
    }
    Ok(dead)
}

fn report(tag: &str, dead: &[(Id, Option<Duration>)]) {
    for (id, d) in dead {
        match d {
            Some(d) => println!("  {tag} id {}: back after {:>9.3?}", id.as_byte(), d),
            None => println!("  {tag} id {}: NEVER (cap hit)", id.as_byte()),
        }
    }
}

/// Write baud registers in the given order, switch the host, stamp t0.
/// A timed-out baud write is INDETERMINATE (the servo may have applied it
/// with the ack lost -- silicon-observed); never retry, let the reunion
/// sweep at the new rate decide.
fn migrate(c: &mut Client<NusbPipe>, order: &[Id], rate: BaudRate) -> Result<Instant, Error> {
    for &id in order {
        match c.write(id, table::BAUD_RATE_IDX, &[rate.as_idx()]) {
            Ok(()) => {}
            Err(Error::Timeout { .. }) => {
                println!(
                    "  (baud ack LOST from id {} -- indeterminate)",
                    id.as_byte()
                );
            }
            Err(e) => return Err(e),
        }
    }
    c.host_baud(rate)?;
    Ok(Instant::now())
}

/// Post-reunion stability tail: sweep everyone for `span`, report relapses.
fn stability_tail(
    c: &mut Client<NusbPipe>,
    ids: &[Id],
    t0: Instant,
    span: Duration,
) -> Result<(), Error> {
    let start = Instant::now();
    while start.elapsed() < span {
        for &id in ids {
            match c.ping(id) {
                Ok(_) => {}
                Err(Error::Timeout { .. }) => {
                    println!("  RELAPSE id {} at +{:.3?}", id.as_byte(), t0.elapsed());
                }
                Err(e) => return Err(e),
            }
        }
    }
    Ok(())
}

const CAP: Duration = Duration::from_secs(10);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let usage = "mute_probe <cycles|order|solo|rescue> [n] <id...>";
    let mode = args.first().ok_or(usage)?.clone();
    let mut c = Client::connect(NusbPipe::open()?)?;

    let parse_ids = |from: usize| -> Result<Vec<Id>, Box<dyn std::error::Error>> {
        let out: Result<Vec<u8>, _> = args[from..].iter().map(|a| a.parse::<u8>()).collect();
        Ok(out?.into_iter().map(Id::new).collect())
    };

    match mode.as_str() {
        "cycles" | "order" => {
            let n: u32 = args.get(1).ok_or(usage)?.parse()?;
            let ids = parse_ids(2)?;
            for i in 0..n {
                let mut order = ids.clone();
                if mode == "order" && i % 2 == 1 {
                    order.reverse();
                }
                let t0 = migrate(&mut c, &order, BaudRate::B3000000)?;
                println!(
                    "cycle {i} -> 3M (write order {:?}):",
                    order.iter().map(|d| d.as_byte()).collect::<Vec<_>>()
                );
                report("3M", &wait_reunion(&mut c, &ids, t0, CAP)?);
                stability_tail(&mut c, &ids, t0, Duration::from_millis(200))?;
                let t0 = migrate(&mut c, &ids, BaudRate::B1000000)?;
                println!("cycle {i} -> 1M:");
                report("1M", &wait_reunion(&mut c, &ids, t0, CAP)?);
                stability_tail(&mut c, &ids, t0, Duration::from_millis(200))?;
            }
        }
        "solo" => {
            let n: u32 = args.get(1).ok_or(usage)?.parse()?;
            let ids = parse_ids(2)?;
            let [id] = ids.as_slice() else {
                return Err("solo takes exactly one id".into());
            };
            for i in 0..n {
                let t0 = migrate(&mut c, &[*id], BaudRate::B3000000)?;
                println!("solo {i} -> 3M:");
                report("3M", &wait_reunion(&mut c, &[*id], t0, CAP)?);
                let t0 = migrate(&mut c, &[*id], BaudRate::B1000000)?;
                println!("solo {i} -> 1M:");
                report("1M", &wait_reunion(&mut c, &[*id], t0, CAP)?);
            }
        }
        "rescue" => {
            let ids = parse_ids(1)?;
            let t0 = migrate(&mut c, &ids, BaudRate::B3000000)?;
            // One quick sweep to confirm the mute is on, then rescue into it.
            let mut muted = Vec::new();
            for &id in &ids {
                match c.ping(id) {
                    Ok(_) => {}
                    Err(Error::Timeout { .. }) => muted.push(id.as_byte()),
                    Err(e) => return Err(e.into()),
                }
            }
            println!("muted at +{:.3?}: {muted:?}", t0.elapsed());
            let t1 = Instant::now();
            let roster = c.rescue_sweep(&ids)?;
            println!("rescue at +{:.3?} -> roster {:?}", t0.elapsed(), roster);
            println!("rescue sweep took {:.3?}", t1.elapsed());
            // Leave the bench at 1M.
            for &id in &ids {
                c.write(id, table::BAUD_RATE_IDX, &[BaudRate::B1000000.as_idx()])?;
            }
            c.host_baud(BaudRate::B1000000)?;
            let t0 = Instant::now();
            report("1M", &wait_reunion(&mut c, &ids, t0, CAP)?);
        }
        _ => return Err(usage.into()),
    }
    Ok(())
}
