//! End-to-end smoke: prove the whole loop (host frame → wire → real framer →
//! real dispatch → real TX engine → recorded reply) before the test-suite
//! chunks build on this facade.

use osc_core::BaudRate;
use osc_protocol::wire::{Opcode, ResultCode};

use super::servo::{DEFAULT_FIRMWARE, DEFAULT_MODEL};
use super::{Sim, core::TICKS_PER_US};
use super::{Source, assert_valid, instruction, status};

const SERVO_ID: u8 = 5;
const BYTE_TIME_US: u64 = 10; // 1 byte-time at 1 M baud

#[test]
fn ping_round_trip() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(SERVO_ID);

    sim.host_send(&instruction(SERVO_ID, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    assert_eq!(frames.len(), 2, "host PING + servo reply: {frames:#?}");
    let inst = frames
        .iter()
        .find(|f| f.from == Source::Host)
        .expect("host frame");
    let reply = frames
        .iter()
        .find(|f| matches!(f.from, Source::Servo(_)))
        .expect("servo reply");

    // A well-formed status from id 5 carrying model(2) + fw(1), padded (p=3 odd).
    assert_eq!(reply.from, Source::Servo(SERVO_ID));
    assert_valid(reply);
    let (rinst, payload) = status(reply);
    assert!(rinst.is_status());
    assert_eq!(rinst.result(), Some(ResultCode::Ok));
    assert!(rinst.pad(), "3-byte payload is odd → padded");
    let m = DEFAULT_MODEL.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], DEFAULT_FIRMWARE]);

    // Reply lead: >= T_turn (2 byte-times) and < 200 µs after the instruction.
    let lead = reply.at - inst.end;
    assert!(
        lead >= 2 * BYTE_TIME_US * TICKS_PER_US,
        "reply must lead by >= T_turn, got {lead} ticks"
    );
    assert!(
        lead < 200 * TICKS_PER_US,
        "reply must land within 200 µs, got {lead} ticks"
    );
}
