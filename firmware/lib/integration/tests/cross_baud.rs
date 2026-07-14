//! Cross-baud garble and the parked resolver (the fleet-mute RCA,
//! `docs/osc-native-protocol.md` sec 3.4 / sec 8): garble that spells a
//! plausible frame prefix parks the resolver on a phantom footprint, frames
//! that follow FEED the phantom instead of dispatching, and recovery is the
//! starve giveup -- which needs one horizon of NO ring progress, exactly
//! what back-to-back traffic denies. Silicon-measured as 8-19 ms of fleet
//! mute during unpaced toward-faster baud migrations (survivor = the
//! last-written servo); here the mechanism is pinned exactly.
//!
//! These scenarios run on the SCRIPTED host with overlapping schedules: the
//! link-mode client rig cannot hold a park across commands (every exchange
//! runs the queue dry, which is unbounded quiet), so client-level suites
//! only see the paced choreographies -- by design, those are always clean.

use osc_integration::sim::{Sim, Source, instruction};
use osc_protocol::wire::{BaudRate, Opcode, STARVE_HORIZON_BYTE_TIMES};

const ID5: u8 = 5;

fn servo_frames_after(frames: &[osc_integration::sim::WireFrame], at: u64) -> Vec<(u64, Vec<u8>)> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)) && f.at >= at)
        .map(|f| (f.at, f.bytes.clone()))
        .collect()
}

#[test]
fn a_parked_resolver_eats_the_next_frame_and_replies_late() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);
    sim.run();
    let t = sim.now_us() + 50;
    // Superposition residue: a plausible prefix that never completes. The
    // next frame's own break byte lands as the header's 4th byte, so the
    // phantom claims LEN 0xFA -- a 253-byte footprint.
    sim.inject_break_at(t);
    sim.inject_garble_at(t + 15, 0x05);
    sim.inject_garble_at(t + 30, 0xFA);
    // The ping lands inside the park, well under the giveup horizon.
    sim.host_send_at(t + 100, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    // The ping was eaten: no reply within anything resembling a turnaround.
    // The starve giveup (64 byte-times of no progress = 640 us at 1M) then
    // sacrificed the phantom, the hunt recovered the eaten ping from the
    // ring, and the reply left LATE -- past any host response window, which
    // is how an eaten frame reads as a timeout AND as unexpected wire bytes.
    let replies = servo_frames_after(&frames, 0);
    assert_eq!(replies.len(), 1, "the eaten ping still dispatches once");
    let ping_end_us = frames
        .iter()
        .find(|f| matches!(f.from, Source::Host))
        .expect("the ping was recorded")
        .end
        / 48; // ticks -> us
    let reply_at_us = replies[0].0 / 48;
    let horizon_us = STARVE_HORIZON_BYTE_TIMES as u64 * 10; // byte-times at 1M
    assert!(
        reply_at_us >= ping_end_us + horizon_us,
        "the reply must wait out the giveup horizon: reply at {reply_at_us} us, \
         ping ended {ping_end_us} us, horizon {horizon_us} us"
    );
    // The sacrificed phantom is the frame-drop the fleet's counters showed.
    assert_eq!(sim.servo_diag(s).framing_drop_count, 1);

    // With the bus quiet again, the next exchange is a normal turnaround.
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let replies = servo_frames_after(&frames, 0);
    assert_eq!(replies.len(), 1, "clean turnaround after quiet");
}

#[test]
fn migration_garble_storms_the_switched_receiver() {
    // End-to-end through the waveform model: a real baud write moves the
    // servo to 3M (deferred apply at ack drain), then the host's next 1M
    // frame reaches it as spurious breaks + multiplied characters -- never
    // a resolvable frame, so the servo cannot answer traffic at the old
    // rate. This is the migration window every already-switched servo sits
    // in while its slower siblings are still being written.
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);
    sim.run();
    // WRITE baud_rate_idx = 3 (table 0x011): acks at 1M, applies after.
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        0,
        &[0x11, 0x00, BaudRate::B3000000.as_idx()],
    ));
    let frames = sim.run();
    assert_eq!(
        servo_frames_after(&frames, 0).len(),
        1,
        "the ack left at 1M"
    );

    // The servo now samples the wire at 3M; a 1M ping is a garble storm.
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert_eq!(
        servo_frames_after(&frames, 0).len(),
        0,
        "cross-rate traffic must never resolve into a dispatch"
    );
    let _ = s;
}
