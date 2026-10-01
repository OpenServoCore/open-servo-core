//! The break wake against its own ringed 0x00 (transport sec 6): the chip's
//! TIM2 detector fires 9.25 bit-times into a break, ahead of the stop-bit
//! sample that rings the byte, so a wake can beat its byte into the ring; a
//! wake serviced late finds the byte already there. Every pin runs the wake
//! ahead of the byte, behind it, and alternating between the two.

use osc_integration::sim::{BreakWake, Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Inst, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_servo_core::regions::config::addr::common::MODEL_NUMBER;
use rstest::rstest;

const ID: u8 = 5;
const BCAST: u8 = 0xFE;

fn replies(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

fn result(f: &WireFrame) -> Option<ResultCode> {
    assert_valid(f);
    status(f).0.result()
}

/// Frames resolve from ring data and answer at every catalog baud, and no
/// transport counter moves.
#[rstest]
#[test_log::test]
fn frames_answer_at_every_baud(
    #[values(
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000
    )]
    rate: BaudRate,
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte, BreakWake::Alternating)] wake: BreakWake,
) {
    let mut sim = Sim::new(rate);
    sim.set_break_wake(wake);
    let s = sim.add_servo(ID);
    for k in 0..4 {
        let frame = if k % 2 == 0 {
            instruction(ID, Opcode::Ping, 0, &[])
        } else {
            instruction(ID, Opcode::Read, 0, &[0, 0, 4, 0])
        };
        sim.host_send(&frame);
        let frames = sim.run();
        let r = replies(&frames);
        assert_eq!(r.len(), 1, "exchange {k}: {frames:#?}");
        assert_eq!(result(r[0]), Some(ResultCode::Ok), "exchange {k}");
    }
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
}

/// A CAL train converges on the same decision whichever side of its byte
/// each ruler mark's wake lands: every mark is stamped at its wake, and the
/// slowest and fastest rates read the same skew.
#[rstest]
#[test_log::test]
fn cal_train_converges(
    #[values(BaudRate::B500000, BaudRate::B3000000)] rate: BaudRate,
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte, BreakWake::Alternating)] wake: BreakWake,
) {
    const GAP_US: u64 = 400;
    const GAPS: u8 = 8;
    let mut sim = Sim::new(rate);
    sim.set_break_wake(wake);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    let [lo, hi] = (GAP_US as u16).to_le_bytes();
    sim.host_send_at(
        0,
        &instruction(BCAST, Opcode::Mgmt, 0, &[0x06, lo, hi, GAPS]),
    );
    for k in 0..=GAPS as u64 {
        sim.host_send_break_at((k + 1) * GAP_US);
    }
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

/// The drift tracker still stamps: a baseline at the boot rate, then a
/// thermal drift of one trim step draws one step of correction.
#[rstest]
#[test_log::test]
fn drift_tracker_follows_thermal_drift(
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte, BreakWake::Alternating)] wake: BreakWake,
) {
    const PERIOD_US: u64 = 500;
    let silent = instruction(ID + 1, Opcode::Write, Inst::FLAG_NOREPLY, &[0u8; 42]);
    let send = |sim: &mut Sim, t0: u64, n: u64| -> u64 {
        for k in 0..n {
            sim.host_send_at(t0 + k * PERIOD_US, &silent);
        }
        t0 + n * PERIOD_US
    };
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.set_break_wake(wake);
    let s = sim.add_servo_with(ID, 0, DEFAULT_RESPONSE_DEADLINE_US);
    let t = send(&mut sim, 0, 180);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None, "no drift, no decision");
    sim.set_servo_skew_at(t, s, 2_600);
    send(&mut sim, t + PERIOD_US, 140);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(1));
}

const CHAIN_DEADLINE_US: u16 = 60;

fn gread_models(ids: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&MODEL_NUMBER.to_le_bytes());
    p.extend_from_slice(&2u16.to_le_bytes());
    p.extend_from_slice(ids);
    instruction(BCAST, Opcode::Gread, 0, &p)
}

/// A status chain answers in list order (each predecessor's break suspends
/// the next slot's reclaim window while its frame plays out), and a missing
/// predecessor is reclaimed one window after its slot, flagged.
#[rstest]
#[test_log::test]
fn chain_answers_and_reclaims(
    #[values(
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000
    )]
    rate: BaudRate,
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte, BreakWake::Alternating)] wake: BreakWake,
) {
    let mut sim = Sim::new(rate);
    sim.set_break_wake(wake);
    for id in [1, 2, 3] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    sim.host_send(&gread_models(&[1, 2, 3]));
    let frames = sim.run();
    let r = replies(&frames);
    assert_eq!(r.len(), 3, "{frames:#?}");
    for (k, f) in r.iter().enumerate() {
        assert_eq!(f.bytes[1], k as u8 + 1, "list order");
        assert_eq!(result(f), Some(ResultCode::Ok));
    }

    // Servo 4 is absent: servo 5 reclaims its slot no earlier than servo 3's
    // end + reply gap + the window, and within a few byte-times of it.
    sim.add_servo_with(5, 0, CHAIN_DEADLINE_US);
    sim.host_send(&gread_models(&[3, 4, 5]));
    let frames = sim.run();
    let r = replies(&frames);
    assert_eq!(r.len(), 2, "{frames:#?}");
    assert_eq!(result(r[0]), Some(ResultCode::Ok));
    assert_eq!(result(r[1]), Some(ResultCode::PredecessorSilent));
    let byte = 48 * 10_000_000 / rate.as_hz() as u64;
    let floor =
        r[0].end + osc_servo_drivers::bus::REPLY_GAP_US as u64 * 48 + CHAIN_DEADLINE_US as u64 * 48;
    assert!(r[1].at >= floor, "reclaim early: {} < {floor}", r[1].at);
    assert!(
        r[1].at <= floor + 8 * byte,
        "reclaim late: {} vs {floor}",
        r[1].at
    );
}
