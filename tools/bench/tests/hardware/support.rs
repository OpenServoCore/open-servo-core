//! Shared harness for the hardware suite. Config from env: `BENCH_BAUD`
//! (default the chip boot baud), `BENCH_ID` (default 1); the osc-adapter is
//! claimed by USB VID/PID. One shared wire per test binary, acquired
//! lazily; `#[serial]` on every test serialises access to it.
//!
//! State discipline: tests that mutate a persistent field restore it before
//! returning, so adjacent tests start from a clean table without a reboot.
#![allow(dead_code)]

use std::env;
use std::sync::{LazyLock, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::Result;
pub use bench::cli::SETTLE_MS;
use bench::discover::{self, Found};
use bench::osc::{
    Exchange, ExchangeError, RESCUE_PULSE_US, StatusFrame, build_write, parse_exchange,
};
use bench::run::{BurstCycle, BurstReport, Report, burst_measure, capture, measure, xfer};
use bench::wire::Wire;
use bench::{BOOT_BAUD, RESCUE_BAUD};
use osc_protocol::wire::ResultCode;
use osc_servo_core::regions::config::addr::common::BAUD_RATE_IDX;

pub struct Bench {
    wire: Wire,
    id: u8,
}

static BENCH: LazyLock<Mutex<Bench>> = LazyLock::new(|| {
    let baud = env::var("BENCH_BAUD")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(BOOT_BAUD);
    let id = env::var("BENCH_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    let mut wire = Wire::connect(baud).expect("claim the osc-adapter");
    // The adapter's rails default OFF after its own power-cycle; asserting
    // the on-state is idempotent (absolute state, no glitch when already
    // on), and a freshly powered fleet needs its boot time.
    wire.client().set_rails(true, true).expect("rails on");
    std::thread::sleep(Duration::from_millis(300));
    Mutex::new(Bench { wire, id })
});

/// The shared bench. Recovers from mutex poison -- a panicked test leaves
/// the adapter reusable (each exchange resets capture first), so one
/// failure does not cascade into every later test.
pub fn bench() -> MutexGuard<'static, Bench> {
    BENCH.lock().unwrap_or_else(|e| e.into_inner())
}

impl Bench {
    /// The servo id under test.
    pub fn id(&self) -> u8 {
        self.id
    }

    /// Capture tick rate (ticks per us), for converting `turnaround_ticks`.
    pub fn hz_per_us(&mut self) -> u32 {
        self.wire.hz_per_us()
    }

    /// One instruction->status exchange, decoded.
    pub fn xfer(&mut self, wire: &[u8]) -> Result<Exchange> {
        xfer(&mut self.wire, wire, SETTLE_MS)
    }

    /// [`Self::xfer`] with a caller-chosen settle window (SAVE/FACTORY).
    pub fn xfer_within(&mut self, wire: &[u8], settle_ms: u64) -> Result<Exchange> {
        xfer(&mut self.wire, wire, settle_ms)
    }

    /// Rescue-pulse the bus (protocol sec 9.1) and follow it to the rescue baud.
    pub fn rescue_pulse(&mut self) {
        self.wire.pulse_low(RESCUE_PULSE_US).expect("rescue pulse");
        // Servo-side confirm completes ~100 us after the pulse ends.
        std::thread::sleep(Duration::from_millis(2));
        self.follow_baud(RESCUE_BAUD);
    }

    /// Move the host UART to `baud` WITHOUT telling the servo -- the
    /// harness side of losing (or finding) a servo whose rate the host
    /// doesn't know. Off-catalog rates allowed (the detune probe).
    pub fn follow_baud(&mut self, baud: u32) {
        self.wire.set_baud(baud).expect("set host baud");
    }

    /// protocol sec 9.2 prefix-tree walk at the current baud.
    /// The DUT's UID via the prefix walk -- fleet-safe at any baud: the
    /// walk descends collisions (discover's algorithm), so it finds the DUT
    /// whether the bus holds one servo or a chain.
    pub fn dut_uid(&mut self) -> [u8; 16] {
        let id = self.id();
        // A walk can only undercount (a parked fleet reads as silence) and
        // occasionally loses whole subtrees to a mid-walk mute -- retry
        // until the DUT shows (the client walk's roster-stability cousin).
        for _ in 0..4 {
            if let Some(f) = self.walk().into_iter().find(|f| f.id == id) {
                return f.uid;
            }
        }
        panic!("prefix walk did not find the DUT id {id} in 4 walks")
    }

    pub fn walk(&mut self) -> Vec<Found> {
        discover::walk(&mut self.wire).expect("prefix walk")
    }

    /// protocol sec 9.3 CAL train: broadcast `announce`, then `breaks` bare breaks
    /// `gap_us` apart, crystal-paced by the adapter. The announce and the
    /// wire gap are separate on purpose -- a mismatch is the trim test's
    /// clock-offset injector.
    pub fn cal_train(&mut self, announce: &[u8], gap_us: u32, breaks: u32) {
        self.wire
            .train(announce, gap_us, breaks)
            .expect("cal train");
    }

    /// Raw zero-gap burst, no reply parsing -- tracker food, not an
    /// exchange.
    pub fn burst_frames(&mut self, frames: &[Vec<u8>]) {
        self.wire.burst(frames).expect("burst frames");
    }

    /// Drop pending capture -- keeps a long food loop inside the ring
    /// contract without parsing anything.
    pub fn drain_stamps(&mut self) {
        self.wire.reset().expect("reset capture");
    }

    /// Exchange and assert an OK status; returns the decoded status frame.
    pub fn status_ok(&mut self, wire: &[u8]) -> StatusFrame {
        self.status_ok_within(wire, SETTLE_MS)
    }

    /// As [`Self::status_ok`] with a caller-chosen settle window -- SAVE and
    /// FACTORY ack only after the flash stall completes (protocol sec 9.4), well past the
    /// standard window.
    pub fn status_ok_within(&mut self, wire: &[u8], settle_ms: u64) -> StatusFrame {
        let ex = xfer(&mut self.wire, wire, settle_ms).expect("exchange");
        assert_eq!(
            ex.status.result,
            Some(ResultCode::Ok),
            "expected OK status, got {:?}",
            ex.status
        );
        ex.status
    }

    /// Assert the instruction reached the wire (its echo is captured) but drew
    /// no status reply -- the NOREPLY / broadcast / COMMIT silence contract (protocol sec 5).
    pub fn expect_no_reply(&mut self, wire: &[u8]) {
        let (stamps, bit_ticks) = capture(&mut self.wire, wire, SETTLE_MS).expect("capture");
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
        measure(&mut self.wire, wire, count, SETTLE_MS, false, |ex| {
            if ex.status.result != Some(ResultCode::Ok) {
                anyhow::bail!("status {:?}", ex.status.result);
            }
            Ok(())
        })
    }

    /// Zero-gap burst run: `count` cycles of `build`, tallied by silicon failure
    /// mode (delegates to `run::burst_measure`).
    pub fn burst(&mut self, count: u32, build: impl Fn(i32) -> BurstCycle) -> BurstReport {
        burst_measure(&mut self.wire, count, SETTLE_MS, false, build).expect("burst")
    }

    /// Switch the servo (and host) to `baud`: WRITE `baud_rate_idx`, take the
    /// ack at the OLD baud (the servo applies the change only once the ack has
    /// drained, protocol sec 4.2), then follow. A no-op if already there.
    pub fn switch_baud(&mut self, baud: u32) {
        if self.wire.current_baud() == baud {
            return;
        }
        let write = build_write(self.id, BAUD_RATE_IDX, &[baud_index(baud)]);
        let ex = xfer(&mut self.wire, &write, SETTLE_MS).expect("baud-write exchange");
        assert_eq!(
            ex.status.result,
            Some(ResultCode::Ok),
            "baud write to {baud} nacked: {:?}",
            ex.status
        );
        self.wire.set_baud(baud).expect("follow host baud");
    }

    /// [`burst`](Self::burst) at `baud`, restoring the boot baud afterwards so
    /// the next test starts clean. The report is returned before any assertion,
    /// so a failed budget never leaves the bus on the wrong baud.
    pub fn burst_at(
        &mut self,
        baud: u32,
        count: u32,
        build: impl Fn(i32) -> BurstCycle,
    ) -> BurstReport {
        self.switch_baud(baud);
        let report = self.burst(count, build);
        self.switch_baud(BOOT_BAUD);
        report
    }

    /// [`measure`](Self::measure) at `baud`, restoring the boot baud afterwards.
    pub fn measure_at(&mut self, baud: u32, wire: &[u8], count: u32) -> Result<Report> {
        self.switch_baud(baud);
        let report = self.measure(wire, count);
        self.switch_baud(BOOT_BAUD);
        report
    }
}

/// Index of `baud` in the supported set, for the `baud_rate_idx` register.
fn baud_index(baud: u32) -> u8 {
    bench::baud_index(baud).unwrap_or_else(|| panic!("unsupported baud {baud}"))
}
