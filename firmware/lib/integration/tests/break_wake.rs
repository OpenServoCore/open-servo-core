//! The break wake against its own ringed 0x00 (transport sec 6): the chip's
//! TIM2 detector fires 9.5 bit-times into a break, ahead of the stop-bit
//! sample that rings the byte, so a wake can beat its byte into the ring; a
//! wake serviced late finds the byte already there. Every pin runs the wake
//! ahead of the byte, behind it, and alternating between the two.

use osc_integration::sim::{BreakWake, Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Opcode, ResultCode};
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
/// each ruler mark's wake lands: every mark is its detector's stamp, and
/// the slowest and fastest rates read the same skew.
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

/// PFIC HIGH entries a bystander pays per foreign exchange, pinned: the
/// host reads servo 6 and servo 6 answers. Neither frame schedules a
/// milestone at servo 5: one header deadline each (plus the re-inspection
/// when the wake leads its byte), the request resolves at the reply's
/// break, and the reply at its starve horizon on the quiet bus behind it.
#[rstest]
#[test_log::test]
fn bystander_entries_per_foreign_exchange_are_pinned(
    #[values(BaudRate::B500000, BaudRate::B1000000, BaudRate::B3000000)] rate: BaudRate,
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte)] wake: BreakWake,
) {
    const N: u64 = 20;
    let reinspect = u64::from(wake == BreakWake::BeforeByte);
    let mut sim = Sim::new(rate);
    sim.set_break_wake(wake);
    let s = sim.add_servo(ID);
    sim.add_servo(ID + 1);
    for _ in 0..N {
        sim.host_send(&instruction(ID + 1, Opcode::Read, 0, &[0, 0, 32, 0]));
        let frames = sim.run();
        let r = replies(&frames);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].bytes[1], ID + 1);
    }
    let e = sim.entries(s);
    assert_eq!(e.compare, N * (3 + 2 * reinspect), "deadline wakes");
    assert_eq!(e.break_wake, N * 2, "break wakes");
    assert_eq!(e.tx_done, 0, "TX arm completions");
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
}

/// PFIC HIGH entries per two-slot GREAD (servos 6 then 5, a 32-byte span),
/// pinned per role. Every servo pays the GREAD's header, covered and end
/// deadlines. Slot 1 adds the predecessor status's header and end only (the
/// snoop consumes nothing ahead of the end) and its trigger; slot 0 adds
/// its trigger and the header of slot 1's status, which nothing waits on,
/// then that status's starve horizon; the bystander pays a header per
/// status and the last one's horizon. The re-inspection adds one per
/// ringed break when the wake leads its byte.
#[rstest]
#[test_log::test]
fn chain_entries_per_gread_are_pinned(
    #[values(BaudRate::B500000, BaudRate::B1000000, BaudRate::B3000000)] rate: BaudRate,
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte)] wake: BreakWake,
) {
    const N: u64 = 20;
    let reinspect = u64::from(wake == BreakWake::BeforeByte);
    let mut p = Vec::new();
    p.extend_from_slice(&0u16.to_le_bytes());
    p.extend_from_slice(&32u16.to_le_bytes());
    p.extend_from_slice(&[ID + 1, ID]);
    let gread = instruction(BCAST, Opcode::Gread, 0, &p);
    let mut sim = Sim::new(rate);
    sim.set_break_wake(wake);
    let slot1 = sim.add_servo_with(ID, 0, CHAIN_DEADLINE_US);
    let slot0 = sim.add_servo_with(ID + 1, 0, CHAIN_DEADLINE_US);
    let bystander = sim.add_servo_with(ID + 2, 0, CHAIN_DEADLINE_US);
    for _ in 0..N {
        sim.host_send(&gread);
        let frames = sim.run();
        let r = replies(&frames);
        assert_eq!(r.len(), 2);
        for f in r {
            assert_eq!(result(f), Some(ResultCode::Ok));
        }
    }
    for (s, deadlines, breaks) in [(slot1, 6, 2), (slot0, 6, 2), (bystander, 6, 3)] {
        let e = sim.entries(s);
        assert_eq!(e.compare, N * (deadlines + breaks * reinspect), "servo {s}");
        assert_eq!(e.break_wake, N * breaks, "servo {s}");
    }
}

/// PFIC HIGH entries per exchange, pinned (a ping, then a 32-byte read):
/// the break-after-byte budget is one break wake, two TX arm completions
/// and three deadline wakes for a ping (header, frame end, trigger; a read
/// adds its covered checkpoint); a wake ahead of its byte adds exactly the
/// one re-inspection deadline. Any other count is a change in HIGH load on
/// the motor kernel's time.
#[rstest]
#[test_log::test]
fn high_entries_per_exchange_are_pinned(
    #[values(BaudRate::B500000, BaudRate::B1000000, BaudRate::B3000000)] rate: BaudRate,
    #[values(BreakWake::BeforeByte, BreakWake::AfterByte)] wake: BreakWake,
) {
    const N: u64 = 20;
    let reinspect = u64::from(wake == BreakWake::BeforeByte);
    for (frame, deadlines) in [
        (instruction(ID, Opcode::Ping, 0, &[]), 3),
        (instruction(ID, Opcode::Read, 0, &[0, 0, 32, 0]), 4),
    ] {
        let mut sim = Sim::new(rate);
        sim.set_break_wake(wake);
        let s = sim.add_servo(ID);
        for _ in 0..N {
            sim.host_send(&frame);
            assert_eq!(replies(&sim.run()).len(), 1);
        }
        let e = sim.entries(s);
        assert_eq!(e.compare, N * (deadlines + reinspect), "deadline wakes");
        assert_eq!(e.break_wake, N, "break wakes");
        assert_eq!(e.tx_done, N * 2, "TX arm completions");
    }
}
