//! Cross-baud garble and the parked resolver (the fleet-mute RCA,
//! `docs/osc-native-protocol.md` sec 3.4 / sec 8): garble that spells a
//! plausible frame prefix parks the resolver on a phantom footprint until
//! the next frame's break, whose hardware ring position ends it (a frame
//! never straddles a break). Data alone would recover only by the starve
//! giveup, one horizon of NO ring progress, exactly what back-to-back
//! traffic denies: silicon-measured as 8-19 ms of fleet mute during
//! unpaced toward-faster baud migrations.
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

/// The reply's lead from the ping's wire end, ticks, and the one reply.
fn reply_lead(frames: &[osc_integration::sim::WireFrame]) -> u64 {
    let replies = servo_frames_after(frames, 0);
    assert_eq!(replies.len(), 1, "the ping dispatches once");
    let ping_end = frames
        .iter()
        .find(|f| matches!(f.from, Source::Host))
        .expect("the ping was recorded")
        .end;
    replies[0].0 - ping_end
}

#[test]
fn a_phantom_dies_at_the_next_frames_break() {
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
    // The ping lands inside the claim, well under the giveup horizon.
    sim.host_send_at(t + 100, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let lead = reply_lead(&frames);
    // The ping's break lies inside the phantom's claim: a frame never
    // straddles a break, so the phantom is the frame-drop the fleet's
    // counters showed, sacrificed at that break instead of a horizon later.
    assert_eq!(sim.servo_diag(s).framing_drop_count, 1);

    // The same ping on a quiet bus: the eaten one answered as fast.
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let clean = reply_lead(&frames);
    let byte = 10 * 48; // one byte-time at 1M, ticks
    assert!(
        lead <= clean + byte,
        "the ping behind the phantom led by {} us, a clean one by {} us",
        lead / 48,
        clean / 48
    );
    assert!(
        lead < STARVE_HORIZON_BYTE_TIMES as u64 * byte,
        "no giveup horizon"
    );
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
