//! Shared measurement loop for the osc turnaround tools: send one
//! instruction repeatedly, parse each exchange from the stamp stream, and
//! report the TURNAROUND distribution (see [`crate::osc`] for the metric).

use std::thread::sleep;
use std::time::Duration;

use anyhow::{Result, anyhow};
use osc_protocol::wire::{Inst, Opcode, ResultCode};

use crate::osc::{
    Exchange, ExchangeError, build_instruction, build_read, gread_uniform_payload,
    gwrite_uniform_payload, parse_exchange,
};
use crate::pirate::{BStamp, Client};

/// Broadcast id: group frames and COMMIT address every servo and draw no reply.
const BROADCAST: u8 = 0xFE;

pub struct Report {
    pub ok: Vec<f64>,
    pub fail: u32,
}

/// Run `count` exchanges of `wire` and collect turnaround in µs. `settle_ms`
/// must cover the reply's wire time plus servo latency. `check` validates the
/// decoded exchange (payload length, result code) before it counts.
pub fn measure(
    client: &mut Client,
    wire: &[u8],
    count: u32,
    settle_ms: u64,
    verbose: bool,
    check: impl Fn(&Exchange) -> Result<()>,
) -> Result<Report> {
    let hz_per_us = client.hz_per_us()?;
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / client.current_baud() as u64) as u32;

    let mut ok = Vec::new();
    let mut fail = 0u32;
    for i in 0..count {
        // The first exchange after a reset is a known chip flake — retry once.
        let retries = if i == 0 { 1 } else { 0 };
        let mut result = Err(anyhow!("no attempt"));
        for _ in 0..=retries {
            result = attempt(client, wire, bit_ticks, settle_ms, &check);
            if result.is_ok() {
                break;
            }
        }
        match result {
            Ok(ticks) => {
                let us = ticks as f64 / hz_per_us as f64;
                ok.push(us);
                if verbose {
                    println!("exchange {i:>4}  turnaround {us:8.2} us");
                }
            }
            Err(e) => {
                fail += 1;
                if verbose {
                    println!("exchange {i:>4}  FAIL: {e}");
                }
            }
        }
    }
    Ok(Report { ok, fail })
}

fn attempt(
    client: &mut Client,
    wire: &[u8],
    bit_ticks: u32,
    settle_ms: u64,
    check: &impl Fn(&Exchange) -> Result<()>,
) -> Result<u32> {
    let stamps = send_and_drain(client, wire, settle_ms)?;
    let ex = parse_exchange(&stamps, wire, bit_ticks).map_err(|e| anyhow!("{e}"))?;
    check(&ex)?;
    Ok(ex.turnaround_ticks)
}

/// Drain stale stamps, send `wire`, settle, and drain the exchange. The shared
/// capture step for [`measure`] and [`xfer`].
fn send_and_drain(client: &mut Client, wire: &[u8], settle_ms: u64) -> Result<Vec<BStamp>> {
    drain(client)?;
    client.brksend(wire)?;
    sleep(Duration::from_millis(settle_ms));
    drain(client)
}

/// One instruction→status exchange, decoded. The building block for the
/// hardware test suite; [`measure`] is the repeated form.
pub fn xfer(client: &mut Client, wire: &[u8], settle_ms: u64) -> Result<Exchange> {
    let (stamps, bit_ticks) = capture(client, wire, settle_ms)?;
    parse_exchange(&stamps, wire, bit_ticks).map_err(|e| anyhow!("{e}"))
}

/// Send `wire` and return the raw pirate stamps plus the `bit_ticks` needed to
/// parse them. Tests asserting silence inspect the [`crate::osc::ExchangeError`]
/// variant directly instead of going through [`xfer`].
pub fn capture(client: &mut Client, wire: &[u8], settle_ms: u64) -> Result<(Vec<BStamp>, u32)> {
    let hz_per_us = client.hz_per_us()?;
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / client.current_baud() as u64) as u32;
    let stamps = send_and_drain(client, wire, settle_ms)?;
    Ok((stamps, bit_ticks))
}

fn drain(client: &mut Client) -> Result<Vec<BStamp>> {
    let mut all = Vec::new();
    loop {
        let batch = client.bbatch(255)?;
        if batch.is_empty() {
            return Ok(all);
        }
        all.extend(batch);
    }
}

pub struct Stats {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub stddev: f64,
}

impl Stats {
    pub fn from(xs: &[f64]) -> Option<Stats> {
        if xs.is_empty() {
            return None;
        }
        let n = xs.len() as f64;
        let mean = xs.iter().sum::<f64>() / n;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
        Some(Stats {
            min: xs.iter().cloned().fold(f64::INFINITY, f64::min),
            max: xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
            mean,
            stddev: var.sqrt(),
        })
    }

    pub fn print(&self) {
        println!("turnaround   min {:.2} us", self.min);
        println!("             mean {:.2} us", self.mean);
        println!("             max {:.2} us", self.max);
        println!("             stddev {:.2} us", self.stddev);
    }
}

/// One zero-gap burst cycle: `frames` are sent back-to-back (or host-paced),
/// then the reply to `last` is parsed. `expect` present ⇒ its payload must match
/// (a stale read-back means a frame was silently missed by the framer); absent
/// ⇒ reply-presence only (no value check).
pub struct BurstCycle {
    pub frames: Vec<Vec<u8>>,
    pub last: Vec<u8>,
    pub expect: Option<Vec<u8>>,
}

/// The production hot loop for one servo (`osc-native-protocol.md` §5.2): a
/// broadcast `GWRITE(HOLD)` staging `value` at `addr`, a broadcast `COMMIT`, and
/// a `GREAD` reading it back — sent zero-gap. The GREAD reply must already carry
/// `value` (commit-before-read), so a stale read-back means a silently-dropped
/// GWRITE or COMMIT. Shared with `tool-osc-burst` so the tool and the asserting
/// bench test drive the identical cycle.
pub fn hot_loop_cycle(id: u8, addr: u16, value: i32) -> BurstCycle {
    let data = value.to_le_bytes();
    let gwrite = build_instruction(
        BROADCAST,
        Opcode::Gwrite,
        Inst::FLAG_HOLD,
        &gwrite_uniform_payload(addr, &[(id, &data)]),
    );
    let commit = build_instruction(BROADCAST, Opcode::Commit, 0, &[]);
    let gread = build_instruction(
        BROADCAST,
        Opcode::Gread,
        0,
        &gread_uniform_payload(addr, data.len() as u16, &[id]),
    );
    BurstCycle {
        frames: vec![gwrite, commit, gread.clone()],
        last: gread,
        expect: Some(data.to_vec()),
    }
}

/// A plain NOREPLY-write flood then a READ: `writes` back-to-back
/// `WRITE(NOREPLY)` to `addr` with distinct values (the last is `value`),
/// followed by a READ that must read `value` back. Exercises the silent-write
/// path with no per-write reply to pace the framer.
pub fn plain_flood_cycle(id: u8, addr: u16, writes: u32, value: i32) -> BurstCycle {
    let n = writes.max(1);
    let mut frames = Vec::with_capacity(n as usize + 1);
    for k in 0..n {
        // Count up to `value` so every frame is distinct and the final table
        // value is exactly `value`.
        let v = value - (n - 1 - k) as i32;
        let mut payload = addr.to_le_bytes().to_vec();
        payload.extend_from_slice(&v.to_le_bytes());
        frames.push(build_instruction(
            id,
            Opcode::Write,
            Inst::FLAG_NOREPLY,
            &payload,
        ));
    }
    let read = build_read(id, addr, value.to_le_bytes().len() as u16);
    frames.push(read.clone());
    BurstCycle {
        frames,
        last: read,
        expect: Some(value.to_le_bytes().to_vec()),
    }
}

/// The fate of one burst cycle. Richer than the [`BurstReport`] tally so a
/// forensic caller can log or dump per cycle; the tally folds it to counts.
#[derive(Debug)]
pub enum CycleOutcome {
    /// Reply OK and (if checked) the read-back value matched.
    Ok { turnaround_us: f64 },
    /// Reply OK but the read-back value was stale — a missed write/commit.
    Stale { got: Vec<u8>, expected: Vec<u8> },
    /// Reply parsed but carried a non-OK result code.
    ResultErr(Option<ResultCode>),
    /// No reply break followed the instruction echo.
    NoReply,
    /// The reply was present but malformed (truncated / bad header / bad CRC).
    Other(ExchangeError),
    /// The burst never reached the wire.
    SendErr(String),
}

/// Per-cycle view handed to a [`burst_measure_observed`] observer.
pub struct CycleObservation<'a> {
    pub index: u32,
    pub value: i32,
    pub stamps: &'a [BStamp],
    pub bit_ticks: u32,
    pub outcome: &'a CycleOutcome,
}

/// Tally over a burst run: `ok` holds the clean cycles' turnaround (µs); the
/// three counters are the silicon-only failure modes a zero-gap burst exposes
/// (the DES sim's zero-cost handlers cannot model them).
pub struct BurstReport {
    pub ok: Vec<f64>,
    pub stale: u32,
    pub no_reply: u32,
    pub other: u32,
}

impl BurstReport {
    pub fn failures(&self) -> u32 {
        self.stale + self.no_reply + self.other
    }

    pub fn cycles(&self) -> u32 {
        self.ok.len() as u32 + self.failures()
    }
}

/// Bombard the bus with `count` zero-gap bursts and tally the silicon failure
/// modes. `build(value)` produces each cycle's frames; see
/// [`burst_measure_observed`] for the value schedule and observer.
pub fn burst_measure(
    client: &mut Client,
    count: u32,
    settle_ms: u64,
    paced: bool,
    build: impl Fn(i32) -> BurstCycle,
) -> Result<BurstReport> {
    burst_measure_observed(client, count, settle_ms, paced, build, |_| {})
}

/// [`burst_measure`] with a per-cycle `observe` callback for verbose/dump.
///
/// The value schedule is owned here so the stale-detection invariant holds:
/// adjacent cycles carry distinct values (`(index % 1000) + 1`), and the warmup
/// uses a sentinel `0` that no cycle emits — so a stale read-back is never
/// masked by an equal-valued neighbour.
pub fn burst_measure_observed(
    client: &mut Client,
    count: u32,
    settle_ms: u64,
    paced: bool,
    build: impl Fn(i32) -> BurstCycle,
    mut observe: impl FnMut(&CycleObservation),
) -> Result<BurstReport> {
    let hz_per_us = client.hz_per_us()?;
    let bit_ticks = (hz_per_us as u64 * 1_000_000 / client.current_baud() as u64) as u32;

    // Warmup: prime the chip and flush any stale stamps before measuring.
    let warm = build(0);
    let _ = send_cycle(client, &warm, paced);
    sleep(Duration::from_millis(settle_ms));
    drain(client)?;

    let mut report = BurstReport {
        ok: Vec::new(),
        stale: 0,
        no_reply: 0,
        other: 0,
    };
    for index in 0..count {
        let value = (index % 1000) as i32 + 1;
        let cycle = build(value);
        let (stamps, outcome) = run_cycle(client, &cycle, paced, settle_ms, bit_ticks, hz_per_us)?;
        match &outcome {
            CycleOutcome::Ok { turnaround_us } => report.ok.push(*turnaround_us),
            CycleOutcome::Stale { .. } => report.stale += 1,
            CycleOutcome::NoReply => report.no_reply += 1,
            CycleOutcome::ResultErr(_) | CycleOutcome::Other(_) | CycleOutcome::SendErr(_) => {
                report.other += 1
            }
        }
        observe(&CycleObservation {
            index,
            value,
            stamps: &stamps,
            bit_ticks,
            outcome: &outcome,
        });
    }
    Ok(report)
}

fn send_cycle(client: &mut Client, cycle: &BurstCycle, paced: bool) -> Result<()> {
    if paced {
        cycle.frames.iter().try_for_each(|f| {
            sleep(Duration::from_millis(1));
            client.brksend(f)
        })
    } else {
        client.burst(&cycle.frames)
    }
}

fn run_cycle(
    client: &mut Client,
    cycle: &BurstCycle,
    paced: bool,
    settle_ms: u64,
    bit_ticks: u32,
    hz_per_us: u32,
) -> Result<(Vec<BStamp>, CycleOutcome)> {
    if let Err(e) = send_cycle(client, cycle, paced) {
        sleep(Duration::from_millis(2));
        return Ok((Vec::new(), CycleOutcome::SendErr(e.to_string())));
    }
    sleep(Duration::from_millis(settle_ms));
    let stamps = drain(client)?;
    let outcome = match parse_exchange(&stamps, &cycle.last, bit_ticks) {
        Ok(ex) if ex.status.result != Some(ResultCode::Ok) => {
            CycleOutcome::ResultErr(ex.status.result)
        }
        Ok(ex) => match &cycle.expect {
            Some(exp) if &ex.status.payload != exp => CycleOutcome::Stale {
                got: ex.status.payload,
                expected: exp.clone(),
            },
            _ => CycleOutcome::Ok {
                turnaround_us: ex.turnaround_ticks as f64 / hz_per_us as f64,
            },
        },
        Err(ExchangeError::NoReply) => CycleOutcome::NoReply,
        Err(e) => CycleOutcome::Other(e),
    };
    Ok((stamps, outcome))
}
