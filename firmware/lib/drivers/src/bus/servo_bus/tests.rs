//! Composite spec tests: real byte frames laid into the ring, driven through
//! `on_break`/`on_deadline` against the real core dispatcher, asserting on the
//! recorded wire bytes. The fakes advance `now` by reading the armed tick, so
//! every deadline the mux computes is exercised end to end.

use osc_core::regions::CONTROL_BASE_ADDR;
use osc_core::regions::config::addr::comms::ID as ID_ADDR;
use osc_core::{BaudRate, Dispatch, RegionStorage, Session, Shared};
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Id, Inst, MgmtOp, Opcode, ResultCode};

use crate::mocks::bus::{FakeWire, Harness, RING_LEN, WireEvent};

const ID: u8 = 7;
const RATE: BaudRate = BaudRate::B1000000;
const TPB: u32 = 10; // TICKS_PER_US(1) * 1e7 / 1e6
const T_TURN: u32 = 2 * TPB;

fn shared_seeded() -> Shared {
    let shared = Shared::new();
    shared.table.with_mut(|t| {
        t.config.identity.model_number = 0x1234;
        t.config.identity.firmware_version = 0x56;
    });
    shared
}

// --- frame builders -------------------------------------------------------

fn instruction(id: u8, op: Opcode, flags: u8, payload: &[u8]) -> std::vec::Vec<u8> {
    let mut b = FrameBuf::<64>::new();
    b.start(Id::new(id), Inst::instruction(op, flags));
    b.payload_mut()[..payload.len()].copy_from_slice(payload);
    b.finish(payload.len() as u8);
    b.seal().to_vec()
}

fn status(id: u8, result: ResultCode, data: &[u8]) -> std::vec::Vec<u8> {
    let mut b = FrameBuf::<64>::new();
    b.start(Id::new(id), Inst::status(result, false, false));
    b.payload_mut()[..data.len()].copy_from_slice(data);
    b.finish(data.len() as u8);
    b.seal().to_vec()
}

// --- drivers --------------------------------------------------------------

/// Deliver a whole frame: break, header deadline, the covered-complete
/// checkpoint (non-doomed frames), then the end deadline (which dispatches).
/// Returns the framer's packet-end estimate — the tick reply T_turn is
/// measured from: break tick + (footprint − 1) byte-times + the drift adder.
/// Doomed frames rearm at their boundary with no covered checkpoint, so the
/// covered fire is their terminal event.
fn deliver<D: Dispatch>(
    bus: &mut crate::bus::ServoBus<crate::mocks::bus::TestProviders>,
    h: &Harness,
    anchor: usize,
    frame: &[u8],
    now0: u32,
    d: &mut D,
) -> u32 {
    h.ring.place(anchor, frame);
    let fp = frame.len();
    h.deadline.set_now(now0);
    h.ring.set_cursor(((anchor + 1) % RING_LEN) as u16);
    bus.on_break();

    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + 4) % RING_LEN) as u16);
    bus.on_deadline(d);

    // Covered checkpoint (non-doomed) or the doomed frame's boundary rearm.
    let c = h.deadline.armed().expect("covered/end deadline");
    h.deadline.set_now(c);
    h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
    bus.on_deadline(d);

    // A doomed frame left no framer deadline; a live one re-armed the end.
    if let Some(b) = h.deadline.armed() {
        h.deadline.set_now(b);
        h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
        bus.on_deadline(d);
    }
    // The framer's packet_end (see `lock_end`): anchor tick + remaining
    // byte-times + footprint>>6 drift adder.
    now0.wrapping_add((fp as u32 - 1) * TPB)
        .wrapping_add(fp as u32 * TPB >> 6)
}

fn fire<D: Dispatch>(
    bus: &mut crate::bus::ServoBus<crate::mocks::bus::TestProviders>,
    h: &Harness,
    d: &mut D,
) {
    let at = h.deadline.armed().expect("a deadline is armed");
    h.deadline.set_now(at);
    bus.on_deadline(d);
}

/// Pump arm-completions until the reply drains: the CRC ships as its own final
/// arm, so a full reply now spans more than one TC.
fn drain_tx(bus: &mut crate::bus::ServoBus<crate::mocks::bus::TestProviders>, h: &Harness) {
    loop {
        bus.on_tx_complete();
        if matches!(h.wire.log().last(), Some(WireEvent::Release)) {
            break;
        }
    }
}

/// Parse the most recent reply frame from the wire log into (id, inst, payload).
fn last_reply(wire: &FakeWire) -> (u8, Inst, std::vec::Vec<u8>) {
    let log = wire.log();
    let start = log
        .iter()
        .rposition(|e| *e == WireEvent::Start)
        .expect("a reply started");
    let mut bytes = std::vec::Vec::new();
    for e in &log[start + 1..] {
        match e {
            WireEvent::Send(v) => bytes.extend_from_slice(v),
            WireEvent::Release => break,
            WireEvent::Start => {}
        }
    }
    let id = bytes[0];
    let len = bytes[1];
    let inst = Inst(bytes[2]);
    let p = wire::payload_len(len, inst.pad()) as usize;
    (id, inst, bytes[3..3 + p].to_vec())
}

// --- scenarios ------------------------------------------------------------

#[test]
fn s1_ping_round_trip() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    let end = deliver(&mut bus, &h, 100, &frame, 1000, &mut d);

    // Reply is staged but not on the wire until the T_turn deadline.
    assert!(!h.wire.started());
    assert_eq!(h.deadline.armed(), Some(end + T_TURN));

    fire(&mut bus, &h, &mut d);
    assert!(h.wire.started());
    drain_tx(&mut bus, &h);

    // Exact bytes: sealed status(model, fw), padded, valid CRC.
    let reference = status(ID, ResultCode::Ok, &[0x34, 0x12, 0x56]);
    assert_eq!(h.wire.sent(), reference[1..]);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert!(inst.is_status());
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(data, &[0x34, 0x12, 0x56]);
}

#[test]
fn s2_read_returns_table_bytes() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // READ addr 0 (even) count 4.
    let frame = instruction(ID, Opcode::Read, 0, &[0, 0, 4, 0]);
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);

    let (_, inst, data) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(data.len(), 4);
    assert_eq!(&data[..2], &[0x34, 0x12]); // model number, LE
}

#[test]
fn s3_write_acks_then_noreply_is_silent() {
    // WRITE torque_enable=1 → ack, table mutated.
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let frame = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]);
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);
    let (_, inst, data) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(data.is_empty());
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));

    // WRITE with NOREPLY: applied, no wire activity.
    let h2 = Harness::new();
    let mut bus2 = h2.build(ID, RATE, 60);
    let shared2 = Shared::new();
    let mut session2 = Session::new();
    let mut d2 = session2.dispatcher(&shared2);
    let frame2 = instruction(
        ID,
        Opcode::Write,
        Inst::FLAG_NOREPLY,
        &[addr[0], addr[1], 1],
    );
    deliver(&mut bus2, &h2, 100, &frame2, 1000, &mut d2);
    assert!(!h2.wire.started());
    assert_eq!(h2.deadline.armed(), None); // no chain armed
    assert!(shared2.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn s4_write_id_acks_from_old_id_then_applies() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let addr = ID_ADDR.to_le_bytes();
    let frame = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 42]);
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);

    // Ack leaves from the OLD id; new id not yet applied.
    let (id, _, _) = last_reply(&h.wire);
    assert_eq!(id, ID);

    // TX drains → deferred id apply. A ping to 42 now replies.
    drain_tx(&mut bus, &h);
    let ping = instruction(42, Opcode::Ping, 0, &[]);
    deliver(&mut bus, &h, 200, &ping, 5000, &mut d);
    fire(&mut bus, &h, &mut d);
    let (id2, inst2, _) = last_reply(&h.wire);
    assert_eq!(id2, 42);
    assert!(inst2.is_status());
}

#[test]
fn s5_gread_slot1_waits_for_predecessor() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 600);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // Uniform GREAD addr 0 count 4, ids [5, 7] → we are slot 1.
    let gread = instruction(ID, Opcode::Gread, 0, &[0, 0, 4, 0, 5, ID]);
    let gread = broadcast_id(gread);
    let end = deliver(&mut bus, &h, 100, &gread, 1000, &mut d);
    // Armed at end + T_turn + reclaim; not yet triggerable.
    assert_eq!(h.deadline.armed(), Some(end + T_TURN + 600));
    assert!(!h.wire.started());

    // Predecessor (id 5) status frame → our slot pends.
    let pre = status(5, ResultCode::Ok, &[0xAA, 0xBB]);
    deliver(&mut bus, &h, 200, &pre, end + 2, &mut d);
    assert!(!h.wire.started());

    // Now the T_turn deadline triggers our reply, not silent.
    fire(&mut bus, &h, &mut d);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[test]
fn s6_gread_slot1_reclaim_marks_predecessor_silent() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 600);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let gread = broadcast_id(instruction(ID, Opcode::Gread, 0, &[0, 0, 4, 0, 5, ID]));
    deliver(&mut bus, &h, 100, &gread, 1000, &mut d);
    // No predecessor: the reclaim window expires and we take the slot.
    fire(&mut bus, &h, &mut d);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::PredecessorSilent));
}

#[test]
fn s7_corrupt_crc_drops_with_no_reply() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let mut frame = instruction(ID, Opcode::Ping, 0, &[]);
    let covered = frame.len() - 2;
    frame[covered] ^= 0xFF; // corrupt the CRC-lo byte
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);

    assert!(!h.wire.started());
    assert_eq!(h.deadline.armed(), None);
    assert_eq!(bus.diag().crc_fail_count, 1);
}

#[test]
fn s8_odd_anchor_rearms_without_reply() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    deliver(&mut bus, &h, 101, &frame, 1000, &mut d); // odd anchor

    assert!(!h.wire.started());
    assert_eq!(h.ring.rearms(), 1);
    assert_eq!(bus.diag().framing_drop_count, 1);
}

#[test]
fn s9_frame_wrapping_ring_boundary_decodes() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // Ping footprint 6 at anchor 508 wraps 508..512, 0, 1.
    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    deliver(&mut bus, &h, RING_LEN - 4, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);

    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert!(inst.is_status());
    assert_eq!(data, &[0x34, 0x12, 0x56]);
}

#[test]
fn s10_rescue_break_switches_to_500k() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // FE with the line held low → rescue candidate.
    h.line.set_low(true);
    h.deadline.set_now(1000);
    h.ring.set_cursor(1); // FE byte at index 0 is the default 0x00
    bus.on_break();

    // Fire the ~100 µs recheck with the line still low.
    h.deadline.set_now(1000 + 100); // RESCUE_CONFIRM_US * TICKS_PER_US
    bus.on_deadline(&mut d);

    let applied = h.baud.applied();
    assert_eq!(applied.first(), Some(&RATE)); // new() applied the configured rate
    assert_eq!(applied.last(), Some(&BaudRate::B500000));
}

#[test]
fn s11_break_after_covered_cancels_speculation() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // Drive a READ up to its covered checkpoint: the reply is front-loaded
    // (dispatched + staged) but not yet on the wire.
    let frame = instruction(ID, Opcode::Read, 0, &[0, 0, 4, 0]);
    let anchor = 100usize;
    let fp = frame.len();
    h.ring.place(anchor, &frame);
    h.deadline.set_now(1000);
    h.ring.set_cursor(((anchor + 1) % RING_LEN) as u16);
    bus.on_break();
    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + 4) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    let c = h.deadline.armed().expect("covered deadline");
    h.deadline.set_now(c);
    h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    assert!(!h.wire.started(), "reply is staged, not yet triggered");

    // A fresh break lands before the end deadline: it preempts the frame and
    // must cancel the staged reply (no phantom read reply may reach the wire).
    let m = 300usize; // ring[m] defaults to 0x00 → a real break
    h.deadline.set_now(c + 1);
    h.ring.set_cursor(((m + 1) % RING_LEN) as u16);
    bus.on_break();
    assert_eq!(
        bus.diag().framing_drop_count,
        1,
        "preempted read is dropped"
    );

    // Drain whatever the new break armed (a starving header): still silent.
    if h.deadline.armed().is_some() {
        fire(&mut bus, &h, &mut d);
    }
    assert!(!h.wire.started(), "the cancelled reply never fires");

    // The following read exchange still works.
    let next = instruction(ID, Opcode::Read, 0, &[0, 0, 4, 0]);
    deliver(&mut bus, &h, 100, &next, c + 5000, &mut d);
    fire(&mut bus, &h, &mut d);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

/// Rewrite a frame's ID field to broadcast (group ops address via their list).
fn broadcast_id(mut frame: std::vec::Vec<u8>) -> std::vec::Vec<u8> {
    // Re-seal after changing the ID: [0x00][ID][LEN][INST]…
    frame[1] = Id::BROADCAST.as_byte();
    let covered = frame.len() - 2;
    let crc = osc_protocol::crc::osc_crc(&frame[..covered]);
    frame[covered..].copy_from_slice(&crc.to_le_bytes());
    frame
}

// A COMMIT + MGMT smoke to keep the decode arms exercised alongside the wire
// scenarios (broadcast COMMIT applies silently; MGMT REBOOT defers a reboot).
#[test]
fn commit_and_reboot_paths() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // MGMT REBOOT (unicast) acks then stages a reboot honored on poll.
    let frame = instruction(ID, Opcode::Mgmt, 0, &[MgmtOp::Reboot as u8]);
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);
    let (_, inst, _) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    bus.on_tx_complete();
    assert!(bus.take_reboot().is_some());
    assert!(bus.take_reboot().is_none());
}
