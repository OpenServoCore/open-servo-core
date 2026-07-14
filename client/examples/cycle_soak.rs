//! Cyclic hot-loop soak over USB: `Cycle::step` against the live fleet,
//! per-step echo check on the telemetry chain.
//!
//!   cycle_soak span <steps> <id...>
//!   cycle_soak profile <steps> <id...>   (configures profile slot 0 first)

use std::time::Instant;

use osc_client::blocking::Client;
use osc_client::cyclic::{Cycle, Group, Telemetry};
use osc_client::nusb::NusbPipe;
use osc_client::{Id, ResultCode};
use osc_protocol::table;

const GOAL_VELOCITY: u16 = 392; // V006 map
const PROFILE_SLOT0: u16 = 0x280; // protocol sec 5.2 pin

/// Span word encoding, protocol sec 5.2: `[addr:10][count:6]`.
const fn span_word(addr: u16, count: u16) -> u16 {
    (addr << 6) | count
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let usage = "cycle_soak <span|profile> <steps> <id...>";
    let mode = args.first().ok_or(usage)?.clone();
    let steps: u32 = args.get(1).ok_or(usage)?.parse()?;
    let ids: Vec<Id> = args[2..]
        .iter()
        .map(|a| a.parse::<u8>().map(Id::new))
        .collect::<Result<_, _>>()?;
    if ids.is_empty() {
        return Err(usage.into());
    }
    let mut c = Client::connect(NusbPipe::open()?)?;

    let telemetry = match mode.as_str() {
        "span" => Telemetry::Span {
            ids: ids.clone(),
            addr: GOAL_VELOCITY,
            count: 4,
        },
        "profile" => {
            let words = [
                span_word(GOAL_VELOCITY, 4),
                span_word(table::FAULT_FLAGS, 2),
            ];
            let mut spans = Vec::new();
            for w in words {
                spans.extend_from_slice(&w.to_le_bytes());
            }
            for &id in &ids {
                c.write(id, PROFILE_SLOT0, &spans)?;
            }
            Telemetry::Profile {
                ids: ids.clone(),
                slot: 0,
            }
        }
        _ => return Err(usage.into()),
    };
    let cycle = Cycle::new(
        vec![Group {
            addr: GOAL_VELOCITY,
            count: 4,
            ids: ids.clone(),
        }],
        telemetry,
    );

    let mut bad = 0u32;
    let start = Instant::now();
    for i in 0..steps {
        let vals: Vec<[u8; 4]> = (0..ids.len())
            .map(|k| (i.wrapping_mul(31).wrapping_add(k as u32)).to_le_bytes())
            .collect();
        let payloads: Vec<&[u8]> = vals.iter().map(|v| &v[..]).collect();
        let chain = c.step(&cycle, &payloads)?;
        let clean = chain.timeout_slot.is_none()
            && chain.statuses.len() == ids.len()
            && chain.statuses.iter().all(|s| {
                s.result == Some(ResultCode::Ok)
                    && s.payload.get(..4) == Some(&vals[s.slot as usize][..])
            });
        if !clean {
            bad += 1;
            println!(
                "step {i}: {} statuses, timeout {:?}",
                chain.statuses.len(),
                chain.timeout_slot
            );
        }
    }
    let dt = start.elapsed();
    println!(
        "{mode}: {}/{steps} clean in {:.2?} ({:.0} steps/s)",
        steps - bad,
        dt,
        steps as f64 / dt.as_secs_f64()
    );
    Ok(())
}
