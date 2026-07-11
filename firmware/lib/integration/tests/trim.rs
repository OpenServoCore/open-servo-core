//! Passive clock trim (`docs/osc-native-protocol.md` §9.3): a servo measures
//! its own clock against the host's instruction byte cadence — the host is
//! the crystal reference — and converges its oscillator trim without any
//! protocol machinery. The frames here address ANOTHER id: a servo converges
//! just by hearing the host talk, which is what trims a whole fleet from its
//! ordinary boot traffic.

use osc_core::BaudRate;
use osc_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_integration::sim::{Sim, instruction};
use osc_protocol::wire::Opcode;

mod support;

const ID: u8 = 5;
const OTHER_ID: u8 = 6;

/// Queue `n` fat host writes addressed to an absent id: every frame is a
/// CRC-verified instruction (dispatched to nobody, no replies), long enough
/// to clear the framer's cadence-sample floor, spaced clear of each other.
fn queue_fat_frames(sim: &mut Sim, n: u64) {
    let frame = instruction(OTHER_ID, Opcode::Write, 0, &[0u8; 200]);
    for k in 0..n {
        sim.host_send_at(k * 4_000, &frame);
    }
}

/// The trio's real signature (+5.2k ppm, bench 2026-07-11): the first full
/// window jumps straight to the nominal-seed correction, positive = slower.
#[test_log::test]
fn fast_clock_draws_a_multi_step_slowdown_from_the_first_window() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    queue_fat_frames(&mut sim, 12);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(2));
}

#[test_log::test]
fn slow_clock_draws_the_symmetric_speedup() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, -5_200, DEFAULT_RESPONSE_DEADLINE_US);
    queue_fat_frames(&mut sim, 12);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), Some(-2));
}

/// Inside half a nominal step the rounding IS the deadband: a well-trimmed
/// chip is never poked, whatever the traffic volume.
#[test_log::test]
fn near_nominal_clock_holds_still() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 900, DEFAULT_RESPONSE_DEADLINE_US);
    queue_fat_frames(&mut sim, 12);
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
}

/// No measured window, no decision — and short frames (a ping-only wire)
/// never fill one: cadence samples gate on span, so trim activity follows
/// exactly the traffic class (fat group frames) whose snoop margin needs it.
#[test_log::test]
fn short_frames_fill_no_window() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo_with(ID, 5_200, DEFAULT_RESPONSE_DEADLINE_US);
    let ping = instruction(OTHER_ID, Opcode::Ping, 0, &[]);
    for k in 0..200 {
        sim.host_send_at(k * 500, &ping);
    }
    sim.run();
    assert_eq!(sim.poll_clock_trim(s), None);
}
