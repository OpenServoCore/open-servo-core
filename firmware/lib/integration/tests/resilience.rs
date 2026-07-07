//! Framing faults and recovery (`docs/osc-native-protocol.md` §3.2, §3.3,
//! §9.1). These document the break-framed convergence story: a single stray
//! ring byte flips anchor parity, and the framer heals at the next frame
//! boundary. Plain assertions on the observed diagnostics counters.

use osc_core::BaudRate;
use osc_core::regions::control::addr::lifecycle::{GOAL_POSITION, GOAL_VELOCITY};
use osc_integration::sim::{Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Opcode, ResultCode};

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

/// A WRITE of a 4-byte goal_velocity — payload 6 B (even, no pad), footprint
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

    // Frame A: one stray ring byte injected inside its wire window. The break
    // anchored A at even parity and its header parsed, but the extra byte
    // shifts every byte after it, so the CRC-covered span reads misaligned →
    // CRC fail. No ack, not applied.
    sim.host_send_at(0, &write_gv(ID5, 0x0A0A0A0A));
    sim.inject_garble_at(50, 0xAA);

    // Frame B: the stray byte advanced the ring by one, so the next break's
    // ring byte lands at ODD parity → §3.2 layer-1 fault → framing_drop +
    // boundary rearm (ring cursor reset to 0). No ack, not applied.
    sim.host_send_at(1000, &write_gv(ID5, 0x0B0B0B0B));

    // Frame C: the rearm re-aligned the ring, so C anchors even → clean parse,
    // ack, applied. Convergence in exactly two lost frames (§3.2).
    sim.host_send_at(2000, &write_gv(ID5, 0x0C0C0C0C));

    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames)); // the sole reply is C's ack
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_velocity),
        0x0C0C0C0C,
        "only frame C applied"
    );
    let d = sim.servo_diag(s);
    assert_eq!(
        d.crc_fail_count, 1,
        "A: even anchor, misaligned span → CRC fail"
    );
    assert_eq!(
        d.framing_drop_count, 1,
        "B: odd anchor from the parity flip → dropped + rearm"
    );
}

#[test]
fn lone_garble_self_heals_in_one_frame() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A lone garble byte on an idle bus advances the ring by one → odd parity.
    sim.inject_garble_at(10, 0xAA);

    // Ping #1 anchors odd → dropped at its computed end + boundary rearm; silent.
    sim.host_send_at(100, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());
    let d = sim.servo_diag(s);
    assert_eq!(d.framing_drop_count, 1);
    assert_eq!(d.crc_fail_count, 0);

    // Ping #2 anchors even again → answered. The bus self-healed in one frame.
    sim.host_send_at(1000, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[test]
fn truncated_frame_starves_then_recovers() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A sealed WRITE cut to 6 ring bytes (break + ID,LEN,INST,addr0,addr1): the
    // header parses and computes a frame end that never arrives → the framer's
    // end-recheck plateau exhausts and the frame is dropped (§4.1). 6 B is even,
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

#[test]
fn break_preempts_partial_frame() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A truncated frame immediately followed by a complete ping (back-to-back):
    // the ping's fresh break preempts the in-flight partial frame (§3.3) →
    // one framing_drop, and the ping re-anchors and is answered.
    let full = write_gv(ID5, 0x11223344);
    sim.host_send(&full[..6]);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let d = sim.servo_diag(s);
    assert_eq!(
        d.framing_drop_count, 1,
        "the fresh break preempts the partial"
    );
    assert_eq!(d.crc_fail_count, 0);
}

#[test]
fn corrupt_crc_tail_cancels_front_loaded_read() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A READ whose covered span is intact but whose trailing CRC is corrupted:
    // the read is front-loaded (dispatched + reply staged) at covered-complete,
    // then the wire-CRC check at the frame end fails → the staged reply is
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

    // The next read is answered — speculation was cleanly cancelled.
    sim.host_send_at(1000, &instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[test]
fn break_after_covered_cancels_front_loaded_read() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A READ is front-loaded at covered-complete (~86 µs, two byte-times
    // before the packet-end estimate) and CRC-verified at its end (~106 µs).
    // A fresh break dropped into that window preempts the frame and must
    // cancel the staged reply — no phantom read reply.
    sim.host_send_at(0, &instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    sim.inject_garble_at(95, 0x00); // 0x00 FE byte = a break

    let frames = sim.run();
    assert!(
        servo_frames(&frames).is_empty(),
        "the cancelled read never replies: {frames:#?}"
    );
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0, "a break-cancel is not a CRC fail");
    assert!(
        d.framing_drop_count >= 1,
        "the read was preempted by the break"
    );

    // The bus heals (§3.2) and answers a following read within the heal bound.
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

#[test]
fn rescue_pulse_drops_to_500k() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // A ≥300 µs dominant low is a rescue command (§9.1): volatile switch to 0.5M.
    sim.hold_line_low_at(0, 400);
    sim.run();

    // Volatile: the config register still reads the operational baud.
    assert_eq!(
        sim.servo_table(s, |t| t.config.comms.baud_rate_idx),
        BaudRate::B1000000
    );

    // A ping at the operational 1M baud now mismatches the 0.5M servo → no reply.
    sim.host_send_at(600, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());

    // Talk at the rescue rate. The mismatched traffic left unparseable garble
    // in the ring (byte count at a wrong-rate receiver is arbitrary), so §3.2
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
    let inst = reply.expect("rescue ping answered within the §3.2 heal bound");
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[test]
fn zero_payload_write_does_not_trip_rescue() {
    // §9.1 confirm vs in-flight traffic: a long WRITE of zeros keeps the
    // line dominant well past the 100 µs confirm at 1M (a zero byte is low
    // for 9 of its 10 bit-times) — bench-caught: the confirm sampled
    // mid-payload and dropped the rate mid-instruction. Ring-cursor progress
    // since the arm must veto the confirm; only a byte-less hold is a pulse.
    // 8 zero bytes (goal_position + goal_velocity) put a zero data byte
    // squarely under the +100 µs sample.
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

#[test]
fn short_break_is_not_rescue() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // Ordinary frames, each led by a normal break (risen by ISR entry, §9.1) —
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
    assert_eq!(
        sim.servo_table(s, |t| t.config.comms.baud_rate_idx),
        BaudRate::B1000000
    );
}
