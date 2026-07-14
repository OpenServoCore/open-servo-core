//! Framing faults and recovery (`docs/osc-native-protocol.md` sec 3.2, sec 3.3,
//! sec 9.1). These document the break-framed convergence story: a corrupted
//! frame is lost to its CRC, and the next break re-anchors cleanly -- anchor
//! parity is irrelevant (sec 3.2 self-aligning feed). Plain assertions on the
//! observed diagnostics counters.

use osc_integration::sim::{Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Inst, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::control::addr::lifecycle::{
    GOAL_POSITION, GOAL_VELOCITY, TORQUE_ENABLE,
};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{byte_ticks, matrix, sim};

const ID5: u8 = 5;

fn servo_frames(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

fn sole_reply(frames: &[WireFrame]) -> &WireFrame {
    let replies = servo_frames(frames);
    assert_eq!(replies.len(), 1, "expected one servo reply: {frames:#?}");
    assert_valid(replies[0]);
    replies[0]
}

/// A WRITE of a 4-byte goal_velocity -- payload 6 B (even, no pad), footprint
/// 12 B: a comfortably long frame to time a mid-flight garble against.
fn write_gv(id: u8, val: i32) -> Vec<u8> {
    let a = GOAL_VELOCITY.to_le_bytes();
    let v = val.to_le_bytes();
    instruction(id, Opcode::Write, 0, &[a[0], a[1], v[0], v[1], v[2], v[3]])
}

#[test]
fn midframe_garble_costs_one_frame() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // Frame A: one stray ring byte injected inside its wire window. The
    // stray byte fills A's span one byte early, so A's CRC covers a
    // shifted window and fails -- A dies by DATA (the fault contract: the
    // garble's FE is only a wake). No ack, not applied.
    sim.host_send_at(0, &write_gv(ID5, 0x0A0A0A0A));
    sim.inject_garble_at(50, 0xAA);

    // Frame B: the stray byte advanced the ring by one, so B anchors at ODD
    // parity -- irrelevant (sec 3.2 self-aligning feed): B validates, applies,
    // and acks. Convergence costs exactly the one garbled frame.
    sim.host_send_at(1000, &write_gv(ID5, 0x0B0B0B0B));

    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames)); // the sole reply is B's ack
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        0x0B0B0B0B,
        "B applied despite the odd anchor"
    );
    // A dies exactly once at layer 1 -- by CRC or by starve; the contract
    // is the sum, as elsewhere in this suite.
    let d = sim.servo_diag(s);
    assert_eq!(d.framing_drop_count + d.crc_fail_count, 1);
}

#[apply(matrix)]
fn lone_garble_costs_nothing(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // A lone garble byte on an idle bus advances the ring by one -> the next
    // frame anchors at odd parity -- irrelevant (sec 3.2): answered normally.
    sim.inject_garble_at(10, 0xAA);
    sim.host_send_at(100, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let d = sim.servo_diag(s);
    assert_eq!(d.framing_drop_count, 0);
    assert_eq!(d.crc_fail_count, 0);
}

#[apply(matrix)]
fn truncated_frame_starves_then_recovers(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // A sealed WRITE cut to 6 ring bytes (break + ID,LEN,INST,addr0,addr1): the
    // header parses and computes a frame end that never arrives -> the framer's
    // end-recheck plateau exhausts and the frame is dropped (sec 4.1). 6 B is even,
    // so ring parity is preserved (no rearm needed).
    let full = write_gv(ID5, 0x11223344);
    sim.host_send_at(0, &full[..6]);
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());
    assert_eq!(sim.servo_diag(s).framing_drop_count, 1);

    // A complete ping afterwards is answered.
    sim.host_send_at(1000, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[apply(matrix)]
fn break_preempts_partial_frame(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // A truncated frame immediately followed by a complete ping (back-to-back):
    // the ping's fresh break preempts the in-flight partial frame (sec 3.3) ->
    // one framing_drop, and the ping re-anchors and is answered.
    let full = write_gv(ID5, 0x11223344);
    sim.host_send(&full[..6]);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    // The partial dies exactly once at layer 1. The resolver classifies it
    // through the CRC gate (the ping's bytes fill the partial's footprint
    // window and the mixed span fails) rather than by FE position -- either
    // counter is one honest event; the contract is the sum.
    let d = sim.servo_diag(s);
    assert_eq!(d.framing_drop_count + d.crc_fail_count, 1);
}

#[apply(matrix)]
fn corrupt_crc_tail_cancels_front_loaded_read(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // A READ whose covered span is intact but whose trailing CRC is corrupted:
    // the read is front-loaded (dispatched + reply staged) at covered-complete,
    // then the wire-CRC check at the frame end fails -> the staged reply is
    // dropped and counted, and the read-only op never touched the table.
    let mut frame = instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]);
    let last = frame.len() - 1;
    frame[last] ^= 0xFF; // corrupt CRC-hi only; the covered span stays valid
    sim.host_send(&frame);
    let frames = sim.run();

    assert!(
        servo_frames(&frames).is_empty(),
        "a bad-CRC read gets no reply: {frames:#?}"
    );
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 1, "the wire-CRC check failed");
    assert_eq!(d.framing_drop_count, 0);

    // The next read is answered -- the pending frame was cleanly dropped.
    sim.host_send_at(1000, &instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[test]
fn break_after_covered_cancels_front_loaded_read() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A READ is front-loaded at covered-complete (~86 us, two byte-times
    // before the packet-end estimate) and CRC-verified at its end (~106 us).
    // A fresh break dropped into that window preempts the frame and must
    // cancel the staged reply -- no phantom read reply.
    sim.host_send_at(0, &instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    sim.inject_break_at(95); // a stray break dropped into the read's window

    let frames = sim.run();
    assert!(
        servo_frames(&frames).is_empty(),
        "the cancelled read never replies: {frames:#?}"
    );
    // The injected mid-frame break shifts every byte after it, so the
    // resolver sees a corrupt frame (CRC gate), not a positional preempt --
    // one honest layer-1 count either way.
    let d = sim.servo_diag(s);
    assert!(
        d.framing_drop_count + d.crc_fail_count >= 1,
        "the preempted read is counted"
    );

    // The bus heals (sec 3.2) and answers a following read within the heal bound.
    let mut answered = false;
    for k in 0..3u64 {
        sim.host_send_at(
            200 + k * 200,
            &instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]),
        );
        let frames = sim.run();
        if let [r] = servo_frames(&frames)[..] {
            assert_valid(r);
            assert_eq!(status(r).0.result(), Some(ResultCode::Ok));
            answered = true;
            break;
        }
    }
    assert!(answered, "a following read is answered after the break");
}

#[apply(matrix)]
fn crc_fail_write_reverts_and_keeps_held_entry(baud_idx: u8) {
    // sec 5.3 L1 across frames: a bad-CRC pending write must revert without
    // disturbing entries HELD by an earlier frame. A HOLD stages torque_enable;
    // a corrupt plain write of goal_velocity is staged on top then reverted;
    // COMMIT must land only the held torque_enable.
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);
    let te = TORQUE_ENABLE.to_le_bytes();
    let gv = GOAL_VELOCITY.to_le_bytes();

    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        Inst::FLAG_HOLD,
        &[te[0], te[1], 1],
    ));
    sim.run();
    assert!(!sim.servo_table(s, |t| t.control.lifecycle.torque_enable));

    // Corrupt a data byte (index 6, past the 2-byte addr) so the covered span
    // fails CRC while still decoding as our goal_velocity write.
    let mut bad = instruction(ID5, Opcode::Write, 0, &[gv[0], gv[1], 9, 9, 9, 9]);
    bad[6] ^= 0xFF;
    sim.host_send(&bad);
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());
    assert_eq!(sim.servo_diag(s).crc_fail_count, 1);
    assert_eq!(sim.servo_table(s, |t| t.control.lifecycle.goal_velocity), 0);

    // COMMIT applies only the held torque_enable; the reverted write is gone.
    sim.host_send(&instruction(ID5, Opcode::Commit, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(sim.servo_table(s, |t| t.control.lifecycle.torque_enable));
    assert_eq!(sim.servo_table(s, |t| t.control.lifecycle.goal_velocity), 0);
}

#[test]
fn break_preempts_pending_write_then_recovers() {
    // A WRITE preempted by a fresh break before its CRC verdict must leave the
    // table clean; the dangling staged write is reclaimed by the dispatcher
    // auto-revert, so a following write applies.
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    sim.host_send_at(0, &write_gv(ID5, 0x0A0A0A0A));
    sim.inject_break_at(115); // a stray break dropped mid-frame
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        0,
        "a preempted write must not mutate the table"
    );

    // Heal (sec 3.2) and confirm a following write applies + acks.
    let mut applied = false;
    for k in 0..3u64 {
        sim.host_send_at(300 + k * 300, &write_gv(ID5, 0x0C0C0C0C));
        let frames = sim.run();
        if let [r] = servo_frames(&frames)[..] {
            assert_valid(r);
            assert_eq!(status(r).0.result(), Some(ResultCode::Ok));
            applied = true;
            break;
        }
    }
    assert!(applied, "a following write is answered after the break");
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        0x0C0C0C0C
    );
}

#[apply(matrix)]
fn latched_refires_mid_frame_never_kill_the_trusted_stream(baud_idx: u8) {
    // The fault contract's regression pin (bench: hot GWRITE+COMMIT+GREAD
    // chains fell 95% -> 80%): spurious break wakes -- coalesced or lagged
    // services; on the FE-era wake, latched-flag re-fires after NOREPLY
    // frames -- land mid-frame while trusted traffic streams. Those wakes
    // carry no position and no time; two of them during one frame's flight
    // must cost NOTHING. (The deleted wire-fault fence recorded service-time
    // cursors and killed the live frame here.)
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // A NOREPLY write (leaves the flag latched on real silicon), then a
    // burst of acked writes with re-fires landing inside each wire window.
    let a = GOAL_VELOCITY.to_le_bytes();
    sim.host_send_at(
        0,
        &instruction(
            ID5,
            Opcode::Write,
            Inst::FLAG_NOREPLY,
            &[a[0], a[1], 1, 0, 0, 0],
        ),
    );
    let bt_us = byte_ticks(baud_idx) / 48; // sim byte-time in us
    let mut acks = 0;
    for k in 0..4u64 {
        let at = 1000 + k * 5000;
        sim.host_send_at(at, &write_gv(ID5, 0x0C0C0C0C + k as i32));
        // Two re-fires inside the frame's wire window: one early (header
        // region), one late (interior) -- the old fence needed exactly two
        // progressing services to plant mid-frame and kill.
        sim.inject_wake_refire_at(at + 2 * bt_us, s);
        sim.inject_wake_refire_at(at + 6 * bt_us, s);
        let frames = sim.run();
        acks += servo_frames(&frames).len();
    }
    assert_eq!(acks, 4, "every acked write answered despite re-fires");
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        0x0C0C0C0C + 3,
        "last write applied"
    );
    let d = sim.servo_diag(s);
    assert_eq!(
        d.framing_drop_count + d.crc_fail_count,
        0,
        "re-fires cost nothing on a trusted stream"
    );
}

#[test]
fn foreign_baud_garbage_recovers_within_the_data_bound() {
    // Foreign-baud recovery, specified to the fault contract (osc-native
    // sec 3.4): a probe at a foreign baud rings FE-dense garble, and a
    // phantom header inside it (garbage LEN) claims interior. Instructions
    // following at the servo's own baud feed that interior -- and since an
    // FE carries no position, NOTHING may kill the phantom by fault evidence
    // (a fence built on service-time cursors killed live frames under burst
    // load). The guarantee is data-bounded recovery instead: the phantom
    // dies by footprint-fill CRC or by the starve horizon (64 byte-times of
    // ring silence), the hunt then resolves the swallowed instruction from
    // ring data, and -- the property that must stay dead -- the lateness
    // NEVER becomes a steady state: every subsequent exchange is answered
    // off its own frame.
    const TICKS_PER_US: u64 = 48;
    let mut sim = Sim::new(BaudRate::B2000000);
    sim.add_servo(ID5);

    sim.set_host_baud(BaudRate::B1000000);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    sim.set_host_baud(BaudRate::B2000000);
    sim.host_send_at(100, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    let ping = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Host))
        .nth(1)
        .expect("own-baud ping recorded");
    let reply = frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .find(|f| f.at > ping.end)
        .expect("ping never answered -- swallowed as phantom interior");
    assert_valid(reply);
    assert_eq!(status(reply).0.result(), Some(ResultCode::Ok));
    // Recovery bound: each 0x00-plausible junk anchor in the garble costs
    // at most one starve horizon (64 byte-times = 320 us at 2M) on a quiet
    // wire. The garble yields a couple such anchors; 3 horizons + a frame
    // of slack is the contract's practical ceiling here.
    let lead = reply.at - ping.end;
    assert!(
        lead < 1000 * TICKS_PER_US,
        "ping answered {} us after its end -- recovery unbounded",
        lead / TICKS_PER_US
    );

    // The exchanges after it stay clean and prompt -- no one-late residue
    // (the indefinite cascade the hunt-flip fixed; CRC rejection of the
    // phantom flips the hunt on, and the hunt converges).
    for k in 0..2 {
        sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
        let frames = sim.run();
        let (inst, _) = status(sole_reply(&frames));
        assert_eq!(inst.result(), Some(ResultCode::Ok), "follow-up {k}");
    }
}

#[test]
fn rescue_pulse_drops_to_500k() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A >=300 us dominant low is a rescue command (sec 9.1): volatile switch to 0.5M.
    sim.hold_line_low_at(0, 400);
    sim.run();

    // Volatile: the config register still reads the operational baud.
    assert_eq!(
        sim.servo_table(s, |t| t.config.comms.baud_rate_idx),
        BaudRate::B1000000.as_idx()
    );

    // A ping at the operational 1M baud now mismatches the 0.5M servo -> no reply.
    sim.host_send_at(600, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());

    // Talk at the rescue rate. The mismatched traffic left unparseable garble
    // in the ring (byte count at a wrong-rate receiver is arbitrary), so sec 3.2
    // allows one frame for the parity self-heal: the first valid ping may be
    // spent on drop + rearm, the second must answer.
    sim.set_host_baud(BaudRate::B500000);
    let mut reply = None;
    for attempt in 0..2u64 {
        sim.host_send_at(
            1200 + attempt * 600,
            &instruction(ID5, Opcode::Ping, 0, &[]),
        );
        let frames = sim.run();
        if let [r] = servo_frames(&frames)[..] {
            assert_valid(r);
            reply = Some(status(r).0);
            break;
        }
    }
    let inst = reply.expect("rescue ping answered within the sec 3.2 heal bound");
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[test]
fn zero_payload_write_does_not_trip_rescue() {
    // sec 9.1 phantom-rescue regression net: a long WRITE of zeros keeps the
    // line dominant for most of its wire time (a zero byte is low for 9 of
    // its 10 bit-times) -- bench-caught on the FE-era confirm, which sampled
    // mid-payload and dropped the rate mid-instruction. The veto now lives
    // in the chip's main-loop sampler (ring-frozen window: every completed
    // char moves NDTR); at this model level the pin holds that ordinary
    // traffic never produces a rescue declaration.
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    let a = GOAL_POSITION.to_le_bytes();
    let w = instruction(ID5, Opcode::Write, 0, &[a[0], a[1], 0, 0, 0, 0, 0, 0, 0, 0]);
    sim.host_send_at(0, &w);
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));

    // Still at the operational rate: a following 1M ping is answered clean.
    sim.host_send_at(1_000, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(sim.servo_diag(s).framing_drop_count, 0);
}

#[apply(matrix)]
fn short_break_is_not_rescue(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // Ordinary frames, each led by a normal break (risen by ISR entry, sec 9.1) --
    // no false rescue, diagnostics stay clean across several exchanges.
    for k in 0..4 {
        sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
        let frames = sim.run();
        let (inst, _) = status(sole_reply(&frames));
        assert_eq!(inst.result(), Some(ResultCode::Ok), "frame {k}");
    }

    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
    // No false rescue: the operational baud is unchanged.
    assert_eq!(
        sim.servo_table(s, |t| t.config.comms.baud_rate_idx),
        baud_idx
    );
}

/// The wrong-baud wedge, replayed: wrong-baud garble marination alone once
/// made a fleet servo permanently deaf under the FE-era storm throttle (the
/// mute's restore chain died through a stolen poll link). The surviving
/// design (errors never interrupt; the wake is LBD-only -- transport sec 7)
/// has no mute and nothing to wedge: faster-baud garble rings silently (zero
/// wakes, sec 3.4), slower-baud garble wakes into junk that dies by data, and
/// after one starve horizon of silence the servo answers at its configured
/// rate.
#[test]
fn wrong_baud_marination_never_deafens() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let _s = sim.add_servo(ID5);

    // Phase 1: 3M traffic at the 1M servo -- sub-10-bit lows, invisible.
    sim.set_host_baud(BaudRate::B3000000);
    for _ in 0..25 {
        sim.host_send(&write_gv(ID5, 0x0A0A0A0A));
    }
    sim.run();

    // Phase 2: 0.5M traffic -- every foreign break a genuine >=10-bit low:
    // the servo wakes into garbled junk, anchors it, and data kills it.
    sim.set_host_baud(BaudRate::B500000);
    for _ in 0..25 {
        sim.host_send(&write_gv(ID5, 0x0A0A0A0A));
    }
    sim.run();

    // One starve horizon of bus silence (sec 3.4 host pacing), then a ping at
    // the configured rate must answer.
    sim.set_host_baud(BaudRate::B1000000);
    let settle = sim.now_us() + 1_000;
    sim.host_send_at(settle, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

/// The rescue half of the wedge: the wedged fleet was deaf-to-rescue because
/// both rescue paths sat behind the muted wake. With the wake unmutable, a
/// rescue pulse into a freshly marinated bus must run the sec 9.1 confirm chain
/// and the servo must answer at the 0.5M rescue rate.
#[test]
fn rescue_reaches_a_marinated_servo() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let _s = sim.add_servo(ID5);

    sim.set_host_baud(BaudRate::B3000000);
    for _ in 0..25 {
        sim.host_send(&write_gv(ID5, 0x0A0A0A0A));
    }
    sim.run();
    sim.set_host_baud(BaudRate::B500000);
    for _ in 0..25 {
        sim.host_send(&write_gv(ID5, 0x0A0A0A0A));
    }
    sim.run();

    let at = sim.now_us() + 1_000;
    sim.hold_line_low_at(at, 400);
    sim.run();

    sim.set_host_baud(BaudRate::B500000);
    sim.host_send_at(at + 2_000, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}
