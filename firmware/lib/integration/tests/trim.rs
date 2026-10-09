//! MGMT CAL break-pair ruler (`docs/osc-native-protocol.md` sec 9.3): the host
//! announces a break train, its crystal spaces the breaks, and each servo
//! measures the announced gap with its own clock -- break-FE entry stamps at
//! both ends of every gap, so entry latency cancels. Plain assertions on the
//! trim decisions and on the transport's health after trains.

use osc_integration::sim::{Sim, Source, instruction, status};
use osc_protocol::wire::{Inst, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;

mod support;

const ID: u8 = 5;
const BROADCAST: u8 = 0xFE;
const GAP_US: u64 = 400;
const GAPS: u8 = 8;
/// First ruler mark, us after the announce frame's start -- clear of the
/// frame itself at every operational baud.
const TRAIN_LEAD_US: u64 = 200;

fn cal_announce(gap_us: u16, gaps: u8) -> Vec<u8> {
    let [lo, hi] = gap_us.to_le_bytes();
    instruction(BROADCAST, Opcode::Mgmt, 0, &[0x06, lo, hi, gaps])
}

/// Announce at `t0`, then `marks` breaks on exact `GAP_US` spacing.
fn send_train(sim: &mut Sim, t0: u64, marks: u64) {
    sim.host_send_at(t0, &cal_announce(GAP_US as u16, GAPS));
    for k in 0..marks {
        sim.host_send_break_at(t0 + TRAIN_LEAD_US + k * GAP_US);
    }
}

fn train_end(t0: u64) -> u64 {
    t0 + TRAIN_LEAD_US + GAPS as u64 * GAP_US
}

/// The trio's real signature (+5.2k ppm, bench-measured): one train, one
/// decision, the nominal-seed acquire jump. Positive = slower.
#[test_log::test]
fn cal_train_draws_the_acquire_jump() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

#[test_log::test]
fn slow_clock_draws_the_symmetric_speedup() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, -5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(-2));
}

/// The ruler is us-denominated, so the measurement is baud-independent --
/// the same train at 3M reads the same skew.
#[test_log::test]
fn cal_at_3m_reads_the_same_skew() {
    let mut sim = Sim::new(BaudRate::B3000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

/// Inside half a nominal step the rounding IS the deadband: a well-trimmed
/// chip is never poked.
#[test_log::test]
fn near_nominal_clock_holds_still() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 900, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// A stray FE mid-gap (line noise) splits one gap into two sub-gate halves:
/// both rejected, two of the announced gaps spent, the survivors still carry
/// the decision -- a noisy train costs precision, never correctness.
#[test_log::test]
fn spurious_fe_mid_train_costs_gaps_not_the_train() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.inject_garble_at(TRAIN_LEAD_US + 3 * GAP_US + GAP_US / 2, 0xAA);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

/// Every gap outside the +/-6% gate (a host that can't keep the announced
/// spacing): fewer than half the gaps validate, and the train decides
/// NOTHING -- a mangled ruler yields no reading rather than a wrong one.
#[test_log::test]
fn mangled_train_decides_nothing() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    sim.host_send_at(0, &cal_announce(GAP_US as u16, GAPS));
    // Marks at alternating 150/650 us -- every gap far outside the gate.
    let mut t = TRAIN_LEAD_US;
    for k in 0..(GAPS as u64 + 1) {
        sim.host_send_break_at(t);
        t += if k % 2 == 0 { 150 } else { 650 };
    }
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// The train's break bytes are scan noise the framer's hunt clears silently:
/// the first instruction after a train answers clean, and neither counter
/// moved -- CAL is invisible to the link diagnostics.
#[test_log::test]
fn train_then_ping_answers_clean() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.host_send_at(train_end(0) + 500, &instruction(ID, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let replies: Vec<_> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "the ping's ack: {frames:#?}");
    let (inst, _) = status(replies[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let d = sim.servo_diag(s);
    assert_eq!(d.framing_drop_count, 0);
    assert_eq!(d.crc_fail_count, 0);
}

/// A train that dies mid-way trips the silence watchdog: no decision, and
/// the transport is back to answering within two announced gaps.
#[test_log::test]
fn dead_train_frees_the_transport() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID);
    sim.host_send_at(0, &cal_announce(GAP_US as u16, GAPS));
    sim.host_send_break_at(TRAIN_LEAD_US);
    sim.host_send_break_at(TRAIN_LEAD_US + GAP_US);
    sim.host_send_at(5_000, &instruction(ID, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let replies: Vec<_> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "the ping's ack: {frames:#?}");
    let (inst, _) = status(replies[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// CAL is broadcast-only (sec 9.3): a unicast CAL's ack would put our own break
/// on the wire exactly where the train starts, so it decodes Unsupported and
/// is answered as an instruction error.
#[test_log::test]
fn unicast_cal_is_refused() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID);
    let [lo, hi] = (GAP_US as u16).to_le_bytes();
    sim.host_send_at(0, &instruction(ID, Opcode::Mgmt, 0, &[0x06, lo, hi, GAPS]));
    let frames = sim.run();
    let replies: Vec<_> = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect();
    assert_eq!(replies.len(), 1, "the refusal: {frames:#?}");
    let (inst, _) = status(replies[0]);
    assert_eq!(inst.result(), Some(ResultCode::Instruction));
    assert_eq!(sim.poll_clock_trim(s), None);
}

// ---- between CALs ----------------------------------------------------------

const OTHER_ID: u8 = 6;
/// Silent hot-loop stand-in: WRITE|NOREPLY, 42-byte payload -> 48-byte
/// footprint, 480 us of wire at 1M; host seam = 20 us.
const PERIOD_US: u64 = 500;

/// `n` silent frames from `t0`, PERIOD_US apart.
fn send_silent(sim: &mut Sim, t0: u64, n: u64) -> u64 {
    let f = instruction(OTHER_ID, Opcode::Write, Inst::FLAG_NOREPLY, &[0u8; 42]);
    for k in 0..n {
        sim.host_send_at(t0 + k * PERIOD_US, &f);
    }
    t0 + n * PERIOD_US
}

/// The sim's Deadline provider's nominal step effect (`SimDeadline`).
const STEP_PPM: i32 = 2_500;

/// The chip adapter's side of the closed loop, which the sim does not own:
/// the main loop polls between frames and HSITRIM moves the oscillator by
/// the nominal effect per applied step (positive = slower), on top of the
/// thermal skew the scenario sets.
struct Oscillator {
    thermal_ppm: i32,
    total: i32,
}

impl Oscillator {
    fn new(sim: &mut Sim, id: u8, thermal_ppm: i32) -> (Self, usize) {
        let s = sim.add_servo_with(id, thermal_ppm, DEFAULT_RESPONSE_DEADLINE_US);
        (
            Self {
                thermal_ppm,
                total: 0,
            },
            s,
        )
    }

    fn apply(&self, sim: &mut Sim, s: usize) {
        let now = sim.now_us();
        sim.set_servo_skew_at(now, s, self.thermal_ppm - self.total * STEP_PPM);
    }

    fn poll(&mut self, sim: &mut Sim, s: usize) -> Option<i8> {
        let out = sim.poll_clock_trim(s);
        if let Some(total) = out {
            self.total = total as i32;
            self.apply(sim, s);
        }
        out
    }

    fn drift_to(&mut self, sim: &mut Sim, s: usize, thermal_ppm: i32) {
        self.thermal_ppm = thermal_ppm;
        self.apply(sim, s);
    }
}

/// One truthful train at `t0`, its decision applied; returns the quiet
/// instant after the train's hunt.
fn cal(sim: &mut Sim, osc: &mut Oscillator, s: usize, t0: u64) -> u64 {
    send_train(sim, t0, GAPS as u64 + 1);
    sim.run();
    osc.poll(sim, s);
    train_end(t0) + 5_000
}

/// Traffic never trims: thermal drift between CALs leaves the trim where
/// the last CAL put it however much silent food the servo hears and however
/// often the main loop polls, and only the host's next CAL follows it
/// (sec 9.3).
#[test_log::test]
fn traffic_never_trims_between_cals() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let (mut osc, s) = Oscillator::new(&mut sim, ID, 0);
    let mut t = 0;
    let mut decisions = Vec::new();
    for chunk in 0..8 {
        if chunk == 2 {
            osc.drift_to(&mut sim, s, 6_900);
        }
        t = send_silent(&mut sim, t, 130) + PERIOD_US;
        sim.run();
        decisions.push(osc.poll(&mut sim, s));
    }
    assert!(
        decisions.iter().all(Option::is_none),
        "silent food decides nothing: {decisions:?}"
    );
    assert_eq!(osc.total, 0);
}

/// The host's periodic CAL follows a thermal shift: two trains converge
/// the boot anchor, the oscillator then drifts by +6.9k ppm, and the next
/// trains re-converge - the first takes the steps, the second reads the
/// residual inside the deadband and identifies the step effect over them.
#[test_log::test]
fn cal_reconverges_after_a_thermal_shift() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let (mut osc, s) = Oscillator::new(&mut sim, ID, 5_200);
    let t = cal(&mut sim, &mut osc, s, 0);
    let t = cal(&mut sim, &mut osc, s, t);
    assert_eq!(osc.total, 2, "the trio's acquire jump");
    osc.drift_to(&mut sim, s, 5_200 + 6_900);
    let t = send_silent(&mut sim, t, 64);
    let t = cal(&mut sim, &mut osc, s, t);
    assert_eq!(osc.total, 5, "round(7100 / 2500) steps more");
    cal(&mut sim, &mut osc, s, t);
    assert_eq!(
        osc.total, 5,
        "converged: the residual is inside half a step"
    );
}
