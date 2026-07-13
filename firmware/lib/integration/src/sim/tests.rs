//! End-to-end smoke: prove the whole loop (host frame -> wire -> real framer ->
//! real dispatch -> real TX engine -> recorded reply) before the test-suite
//! chunks build on this facade.

use osc_protocol::wire::{Opcode, ResultCode};
use osc_servo_core::BaudRate;

use super::servo::{DEFAULT_FIRMWARE, DEFAULT_MODEL};
use super::{HandlerCost, Sim, core::TICKS_PER_US};
use super::{Source, assert_valid, instruction, status};

const SERVO_ID: u8 = 5;

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

    // A well-formed status from id 5 carrying model(2) + fw(1), p=3 (no pad).
    assert_eq!(reply.from, Source::Servo(SERVO_ID));
    assert_valid(reply);
    let (rinst, payload) = status(reply);
    assert!(rinst.is_status());
    assert_eq!(rinst.result(), Some(ResultCode::Ok));
    let m = DEFAULT_MODEL.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], DEFAULT_FIRMWARE]);

    // Reply lead: >= reply gap (fixed us, sec 7) and < 200 us after the instruction.
    let lead = reply.at - inst.end;
    assert!(
        lead >= osc_servo_drivers::bus::REPLY_GAP_US as u64 * TICKS_PER_US,
        "reply must lead by >= reply gap, got {lead} ticks"
    );
    assert!(
        lead < 200 * TICKS_PER_US,
        "reply must land within 200 us, got {lead} ticks"
    );
}

/// A costly `on_break` body defers the framer deadlines past the whole frame:
/// every byte is already ringed (DMA is CPU-independent) when the pended wake
/// finally lands, so the exchange completes correctly -- just late. Occupancy
/// stretches reply latency; it must never lose an isolated frame.
#[test]
fn handler_cost_defers_delivery_without_loss() {
    const BUSY_US: u64 = 150;
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(SERVO_ID);
    sim.set_handler_cost(
        s,
        HandlerCost {
            on_break_us: BUSY_US as u32,
            ..Default::default()
        },
    );

    sim.host_send(&instruction(SERVO_ID, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    assert_eq!(frames.len(), 2, "host PING + late servo reply: {frames:#?}");
    let inst = frames
        .iter()
        .find(|f| f.from == Source::Host)
        .expect("host frame");
    let reply = frames
        .iter()
        .find(|f| matches!(f.from, Source::Servo(_)))
        .expect("servo reply");
    assert_valid(reply);
    let (rinst, _) = status(reply);
    assert_eq!(rinst.result(), Some(ResultCode::Ok));

    // The header deadline pended behind the 150 us on_break body, so the
    // reply cannot lead by less than the body's tail past the break.
    let lead = reply.at - inst.end;
    assert!(
        lead > (BUSY_US - 60) * TICKS_PER_US,
        "reply should be deferred by the busy body, lead {lead} ticks"
    );

    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
}

/// Break wakes landing while a body runs pend as ONE flag, not a queue:
/// three wire breaks against a 500 us body deliver exactly two `on_break`
/// invocations (the live one, then one coalesced pend) -- the silicon
/// behavior behind the zero-gap frame loss, unchanged by the LBD wake
/// (the flag is still one bit).
#[test]
fn pended_breaks_coalesce_like_pfic() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(SERVO_ID);
    sim.set_handler_cost(
        s,
        HandlerCost {
            on_break_us: 500,
            ..Default::default()
        },
    );

    sim.inject_break_at(1_000);
    sim.inject_break_at(1_100);
    sim.inject_break_at(1_200);
    sim.run();

    assert_eq!(
        sim.delivered_breaks(s),
        2,
        "three wire breaks against a busy body must coalesce to two deliveries"
    );
}
