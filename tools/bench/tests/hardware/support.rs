//! Shared harness for the hardware suite. Config from env: `BENCH_PORT` (empty ⇒
//! autodetect the pirate), `BENCH_BAUD` (default the chip boot baud), `BENCH_ID`
//! (default 1). One shared pirate client per test binary, acquired lazily;
//! `#[serial]` on every test serialises access to it.
//!
//! State discipline: tests that mutate a persistent field restore it before
//! returning, so adjacent tests start from a clean table without a reboot.
#![allow(dead_code)]

use std::env;
use std::sync::{LazyLock, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::Result;
use bench::BOOT_BAUD;
use bench::osc::{Exchange, ExchangeError, StatusFrame, parse_exchange};
use bench::pirate::{Client, auto_detect_pirate};
use bench::run::{BurstCycle, BurstReport, Report, burst_measure, capture, measure, xfer};
use osc_protocol::wire::ResultCode;

/// Settle window per exchange (ms): covers the reply's wire time + servo latency
/// at every supported baud, with margin.
pub const SETTLE_MS: u64 = 5;

pub struct Bench {
    client: Client,
    id: u8,
}

static BENCH: LazyLock<Mutex<Bench>> = LazyLock::new(|| {
    let port = match env::var("BENCH_PORT") {
        Ok(p) if !p.is_empty() => p,
        _ => auto_detect_pirate().expect("BENCH_PORT unset and pirate autodetect failed"),
    };
    let baud = env::var("BENCH_BAUD")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(BOOT_BAUD);
    let id = env::var("BENCH_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    let mut client = Client::open(&port, Duration::from_millis(500)).expect("open pirate client");
    client.set_baud(baud).expect("set pirate baud");
    client.reset().expect("reset pirate");
    Mutex::new(Bench { client, id })
});

/// The shared bench. Recovers from mutex poison — a panicked test leaves the
/// pirate reusable (each exchange re-drains first), so one failure does not
/// cascade into every later test.
pub fn bench() -> MutexGuard<'static, Bench> {
    BENCH.lock().unwrap_or_else(|e| e.into_inner())
}

impl Bench {
    /// The servo id under test.
    pub fn id(&self) -> u8 {
        self.id
    }

    /// One instruction→status exchange, decoded.
    pub fn xfer(&mut self, wire: &[u8]) -> Result<Exchange> {
        xfer(&mut self.client, wire, SETTLE_MS)
    }

    /// Exchange and assert an OK status; returns the decoded status frame.
    pub fn status_ok(&mut self, wire: &[u8]) -> StatusFrame {
        let ex = self.xfer(wire).expect("exchange");
        assert_eq!(
            ex.status.result,
            Some(ResultCode::Ok),
            "expected OK status, got {:?}",
            ex.status
        );
        ex.status
    }

    /// Assert the instruction reached the wire (its echo is captured) but drew
    /// no status reply — the NOREPLY / broadcast / COMMIT silence contract (§5).
    pub fn expect_no_reply(&mut self, wire: &[u8]) {
        let (stamps, bit_ticks) = capture(&mut self.client, wire, SETTLE_MS).expect("capture");
        match parse_exchange(&stamps, wire, bit_ticks) {
            Err(ExchangeError::NoReply) => {}
            Err(ExchangeError::NoEcho) => panic!("instruction never reached the wire (no echo)"),
            Err(e) => panic!("expected silence, got parse error: {e}"),
            Ok(ex) => panic!("expected silence, got a reply: {:?}", ex.status),
        }
    }

    /// Repeated exchange for the turnaround metric (delegates to `run::measure`,
    /// gating each sample on an OK status).
    pub fn measure(&mut self, wire: &[u8], count: u32) -> Result<Report> {
        measure(&mut self.client, wire, count, SETTLE_MS, false, |ex| {
            if ex.status.result != Some(ResultCode::Ok) {
                anyhow::bail!("status {:?}", ex.status.result);
            }
            Ok(())
        })
    }

    /// Zero-gap burst run: `count` cycles of `build`, tallied by silicon failure
    /// mode (delegates to `run::burst_measure`).
    pub fn burst(&mut self, count: u32, build: impl Fn(i32) -> BurstCycle) -> BurstReport {
        burst_measure(&mut self.client, count, SETTLE_MS, false, build).expect("burst")
    }
}
