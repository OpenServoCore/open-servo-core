//! Shared harness for the hardware suite. Config from env: `BENCH_BAUD`
//! (default the chip boot baud), `BENCH_ID` (default 1); the osc-adapter is
//! claimed by USB VID/PID. One shared wire per test binary, acquired
//! lazily; `#[serial]` on every test serialises access to it.
//!
//! State discipline: tests that mutate a persistent field restore it before
//! returning, so adjacent tests start from a clean table without a reboot.
#![allow(dead_code)]

use std::env;
use std::ops::RangeInclusive;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::sync::{LazyLock, Mutex, MutexGuard};
use std::thread::sleep;
use std::time::Duration;

use anyhow::{Context, Result};
pub use bench::cli::SETTLE_MS;
use bench::clock::{self, Summary};
use bench::discover::{self, Found};
use bench::osc::{
    Exchange, ExchangeError, RESCUE_PULSE_US, StatusFrame, build_read, build_write, parse_exchange,
};
use bench::run::{BurstCycle, BurstReport, Report, burst_measure, capture, measure, xfer};
use bench::wire::Wire;
use bench::{BOOT_BAUD, RESCUE_BAUD, SUPPORTED_BAUDS};
use osc_client::{DEFAULT_GUARD, StreamReply};
use osc_protocol::build;
use osc_protocol::tel::{CHUNKS_MAX, Chunks};
use osc_protocol::wire::{Id, Inst, Opcode, ResultCode};
use osc_servo_core::Mode;
use osc_servo_core::regions::config::addr::common::BAUD_RATE_IDX;
use osc_servo_core::regions::control::addr::lifecycle::{
    GOAL_DUTY, GOAL_POSITION, MODE, TORQUE_ENABLE,
};
use osc_servo_core::regions::control::addr::tel::{CHUNK0_ADDR, TEL_COUNT};
use osc_servo_core::regions::telemetry::addr::tick::POS;

const _: () = assert!(CHUNK0_ADDR + 4 * CHUNKS_MAX as u16 == TEL_COUNT);

/// The drive check's open-loop pulse: 12% duty for 200 ms from rest moves
/// a healthy servo hundreds of counts, and pulsed toward mid travel it
/// stays clear of both stops; under 20 counts is no motion.
pub const DRIVE_CHECK_DUTY_Q15: i16 = 3932;
const DRIVE_CHECK_PULSE: Duration = Duration::from_millis(200);
const DRIVE_CHECK_SETTLE: Duration = Duration::from_millis(300);
const DRIVE_CHECK_MIN_COUNTS: i32 = 20;

pub struct Bench {
    wire: Wire,
    id: u8,
    /// `pos_min_phys_counts..=pos_max_phys_counts`, read once: the goal rule
    /// rejects GOAL_POSITION outside them, and a calibration narrows them
    /// from the board default.
    goal_rails: RangeInclusive<i32>,
    /// The servo's configured baud, read once: what a reboot returns to
    /// and where every test leaves the bus. Rescue leaves the register
    /// untouched (protocol sec 9.1), so it reads true from a parked servo.
    home_baud: u32,
    /// Exchanges run through the engine rather than [`Wire::send_frame`].
    engine_frames: u64,
    /// The drive direction was checked this run (see
    /// [`Bench::require_forward_drive`]).
    drive_checked: bool,
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
    let goal_rails = bench::run::goal_rails(&mut wire, id).expect("goal rails");
    let [idx] = setup_read(&mut wire, id, BAUD_RATE_IDX);
    let home_baud = *SUPPORTED_BAUDS
        .get(usize::from(idx))
        .unwrap_or_else(|| panic!("baud_rate_idx {idx} out of range"));
    Mutex::new(Bench {
        wire,
        id,
        goal_rails,
        home_baud,
        engine_frames: 0,
        drive_checked: false,
    })
});

/// The shared bench. Recovers from mutex poison -- a panicked test leaves
/// the adapter reusable (each exchange resets capture first), so one
/// failure does not cascade into every later test.
pub fn bench() -> MutexGuard<'static, Bench> {
    BENCH.lock().unwrap_or_else(|e| e.into_inner())
}

impl Bench {
    /// Panics unless duty moves the position its own way (+duty counts
    /// up), once per run, before any test closes a loop: swapped motor leads or a wrong `drive_polarity`
    /// turn every closed loop into a runaway into a stop. One open-loop
    /// pulse from rest toward mid travel, torque off after; a reversed
    /// pulse is never repeated.
    pub fn require_forward_drive(&mut self) {
        if self.drive_checked {
            return;
        }
        let id = self.id;
        let from = self.pos();
        let sign = if from <= self.goal_mid() { 1 } else { -1 };
        let pulse = (sign as i16) * DRIVE_CHECK_DUTY_Q15;
        let to = catch_unwind(AssertUnwindSafe(|| {
            self.status_ok(&build_write(id, MODE, &[Mode::OpenLoop as u8]));
            self.status_ok(&build_write(id, GOAL_DUTY, &0i16.to_le_bytes()));
            self.status_ok(&build_write(id, TORQUE_ENABLE, &[1]));
            self.status_ok(&build_write(id, GOAL_DUTY, &pulse.to_le_bytes()));
            sleep(DRIVE_CHECK_PULSE);
            self.pos()
        }));
        let _ = self.xfer(&build_write(id, GOAL_DUTY, &0i16.to_le_bytes()));
        let _ = self.xfer(&build_write(id, TORQUE_ENABLE, &[0]));
        sleep(DRIVE_CHECK_SETTLE);
        let to = to.unwrap_or_else(|e| resume_unwind(e));
        let moved = sign * (to - from);
        assert!(
            moved > -DRIVE_CHECK_MIN_COUNTS,
            "drive reversed: duty {pulse} moved pos {from} -> {to}; motor leads swapped or \
             drive_polarity wrong (osc cal relearns it)"
        );
        assert!(
            moved >= DRIVE_CHECK_MIN_COUNTS,
            "drive did not move: duty {pulse} moved pos {from} -> {to}; rail, motor or stop?"
        );
        self.drive_checked = true;
    }

    fn pos(&mut self) -> i32 {
        let p = self.status_ok(&build_read(self.id, POS, 2)).payload;
        i32::from(u16::from_le_bytes([p[0], p[1]]))
    }

    /// The servo id under test.
    pub fn id(&self) -> u8 {
        self.id
    }

    /// The servo's configured baud (see the field).
    pub fn home_baud(&self) -> u32 {
        self.home_baud
    }

    /// A GOAL_POSITION value the goal rule accepts on any calibration.
    pub fn goal_mid(&self) -> i32 {
        (self.goal_rails.start() + self.goal_rails.end()) / 2
    }

    /// `goal` moved inside the range the goal rule accepts.
    pub fn goal_clamp(&self, goal: i32) -> i32 {
        goal.clamp(*self.goal_rails.start(), *self.goal_rails.end())
    }

    /// Instruction frames put on the wire since connect, engine exchanges
    /// included.
    pub fn frames_sent(&self) -> u64 {
        self.wire.frames_sent() + self.engine_frames
    }

    /// One TEL burst of `rows` of `chunks` as the stream soaks run it: the
    /// acked WRITE of the chunks and `tel_count` carries the stream and the
    /// engine collects its frames within `window`. Edge capture cannot keep
    /// pace with a stream and laps; it restarts clean after.
    pub fn tel_stream(
        &mut self,
        chunks: Chunks,
        rows: u16,
        window: Duration,
    ) -> Result<StreamReply> {
        let mut arm = [0u8; 4 * CHUNKS_MAX + 2];
        let (regs, count) = arm.split_at_mut(4 * CHUNKS_MAX);
        regs.copy_from_slice(&chunks.to_le_bytes());
        count.copy_from_slice(&rows.to_le_bytes());
        let mut p = [0u8; 2 + 4 * CHUNKS_MAX + 2];
        let n = build::write(&mut p, CHUNK0_ADDR, &arm).context("tel arm")?;
        let inst = Inst::instruction(Opcode::Write, 0);
        let c = self.wire.client();
        c.set_guard(window + DEFAULT_GUARD);
        let r = c.exchange_stream(Id::new(self.id), inst, &p[..n], window);
        c.set_guard(DEFAULT_GUARD);
        self.engine_frames += 1;
        self.wire.reset()?;
        Ok(r?)
    }

    /// [`bench::run::burst_goal_base`] on this servo's rails.
    pub fn burst_goal_base(&self, dip: i32) -> i32 {
        bench::run::burst_goal_base(&self.goal_rails, dip).expect("burst goal base")
    }

    /// Put GOAL_POSITION back after a burst. The boot goal (0) sits outside a
    /// calibrated servo's rails, where the rule refuses it; the burst's last
    /// value stands then.
    pub fn restore_goal(&mut self, orig: &[u8]) {
        let goal = i32::from_le_bytes(orig.try_into().expect("goal_position is 4 bytes"));
        if self.goal_rails.contains(&goal) {
            self.status_ok(&build_write(self.id, GOAL_POSITION, orig));
        }
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

    /// The servo's clock against the adapter crystal, ppm (+ = fast): the
    /// reply pitch of `replies` READs of `len` bytes at the current baud.
    pub fn servo_clock(&mut self, replies: u32, len: u16) -> Summary {
        let baud = self.wire.current_baud();
        let fits = clock::servo_truth(&mut self.wire, self.id, baud, 0, len, replies)
            .expect("clock truth capture");
        let (ppm, rejects) = clock::readings(&fits);
        Summary::of(&ppm).unwrap_or_else(|| {
            panic!("clock truth: {rejects} of {replies} replies gave no reading")
        })
    }

    /// Raw zero-gap burst, no reply parsing - traffic, not an exchange.
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

    /// [`burst`](Self::burst) at `baud`, restoring the home baud afterwards so
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
        self.switch_baud(self.home_baud);
        report
    }

    /// [`measure`](Self::measure) at `baud`, restoring the home baud afterwards.
    pub fn measure_at(&mut self, baud: u32, wire: &[u8], count: u32) -> Result<Report> {
        self.switch_baud(baud);
        let report = self.measure(wire, count);
        self.switch_baud(self.home_baud);
        report
    }
}

/// One register, read at setup before the shared bench exists.
fn setup_read<const N: usize>(wire: &mut Wire, id: u8, addr: u16) -> [u8; N] {
    let ex = xfer(wire, &build_read(id, addr, N as u16), SETTLE_MS).expect("setup read");
    assert_eq!(
        ex.status.result,
        Some(ResultCode::Ok),
        "setup read of {addr:#06x}: {:?}",
        ex.status
    );
    ex.status.payload[..]
        .try_into()
        .unwrap_or_else(|_| panic!("setup read of {addr:#06x}: {N}-byte payload"))
}

/// Index of `baud` in the supported set, for the `baud_rate_idx` register.
fn baud_index(baud: u32) -> u8 {
    bench::baud_index(baud).unwrap_or_else(|| panic!("unsupported baud {baud}"))
}
