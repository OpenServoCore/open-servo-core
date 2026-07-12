//! MGMT CAL break-pair ruler (`docs/osc-native-protocol.md` §9.3): the host
//! announces a break train, its crystal spaces the breaks, and each servo
//! measures the announced gap with its own clock — break-FE entry stamps at
//! both ends of every gap, so entry latency cancels. Plain assertions on the
//! trim decisions and on the transport's health after trains.

use osc_core::BaudRate;
use osc_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_integration::sim::{Sim, Source, instruction, status};
use osc_protocol::wire::{Opcode, ResultCode};

mod support;

const ID: u8 = 5;
const BROADCAST: u8 = 0xFE;
const GAP_US: u64 = 400;
const GAPS: u8 = 8;
/// First ruler mark, µs after the announce frame's start — clear of the
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

/// The trio's real signature (+5.2k ppm, bench 2026-07-11): one train, one
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

/// The ruler is µs-denominated, so the measurement is baud-independent —
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
/// the decision — a noisy train costs precision, never correctness.
#[test_log::test]
fn spurious_fe_mid_train_costs_gaps_not_the_train() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    send_train(&mut sim, 0, GAPS as u64 + 1);
    sim.inject_garble_at(TRAIN_LEAD_US + 3 * GAP_US + GAP_US / 2, 0xAA);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

/// Every gap outside the ±6% gate (a host that can't keep the announced
/// spacing): fewer than half the gaps validate, and the train decides
/// NOTHING — a mangled ruler yields no reading rather than a wrong one.
#[test_log::test]
fn mangled_train_decides_nothing() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    sim.host_send_at(0, &cal_announce(GAP_US as u16, GAPS));
    // Marks at alternating 150/650 µs — every gap far outside the gate.
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
/// moved — CAL is invisible to the link diagnostics.
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

/// CAL is broadcast-only (§9.3): a unicast CAL's ack would put our own break
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
