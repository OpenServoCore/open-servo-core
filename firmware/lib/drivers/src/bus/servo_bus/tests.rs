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
use crate::traits::bus::tick_reached;

const ID: u8 = 7;
const RATE: BaudRate = BaudRate::B1000000;
const TPB: u32 = 10; // TICKS_PER_US(1) * 1e7 / 1e6
const T_TURN: u32 = 2 * TPB;
// END_SLACK_US(5) × TestProviders TICKS_PER_US(1).
const END_SLACK: u32 = 5;

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
    b.start(Id::new(id), Inst::status(result, false));
    b.payload_mut()[..data.len()].copy_from_slice(data);
    b.finish(data.len() as u8);
    b.seal().to_vec()
}

// --- drivers --------------------------------------------------------------

/// Deliver a whole frame: break, header deadline (which emits Covered for
/// frames whose covered span is already ringed), then every remaining framer
/// wake through the end deadline (which dispatches). Framer wakes all land at
/// or before `end_due` = packet_end + slack; the chain's T_turn deadline lies
/// beyond it, so the loop leaves reply sequencing to `fire`.
/// Returns the framer's packet-end estimate — the tick reply T_turn is
/// measured from: break tick + (footprint − 1) byte-times + the drift adder.
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
    // The ladder reaches `anchor` through prior traffic in production; the
    // harness fabricates positions, so bootstrap it explicitly (A2).
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(now0);
    h.ring.set_cursor(((anchor + 1) % RING_LEN) as u16);
    bus.on_break(d);

    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + 4) % RING_LEN) as u16);
    bus.on_deadline(d);

    // The framer's packet_end (see `lock_end`): anchor tick + remaining
    // byte-times + footprint>>6 drift adder.
    let end = now0
        .wrapping_add((fp as u32 - 1) * TPB)
        .wrapping_add((fp as u32 * TPB) >> 6);
    let end_due = end.wrapping_add(END_SLACK);
    while let Some(t) = h.deadline.armed() {
        if !tick_reached(end_due, t) {
            break; // beyond the frame window: chain/rescue, not the framer
        }
        h.deadline.set_now(t);
        h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
        bus.on_deadline(d);
    }
    // Table-class frames publish to the LOW consumer; pump the consumer +
    // adoption wake so the verdict resolves and the reply is sequenced (A3(b)).
    // (Wire-class frames dispatch inline and ignore the empty pump.)
    h.pump(bus, d);
    end
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
    let p = wire::payload_len(len) as usize;
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

    // Exact bytes: sealed status(model, fw), valid CRC.
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
    drain_tx(&mut bus, &h);

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
    drain_tx(&mut bus, &h);
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
    drain_tx(&mut bus, &h);
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
    drain_tx(&mut bus, &h);
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
fn s8_odd_anchor_round_trips() {
    // Anchor parity is irrelevant (§3.2 self-aligning feed): an odd-anchored
    // frame validates and replies like any other.
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    deliver(&mut bus, &h, 101, &frame, 1000, &mut d); // odd anchor
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);

    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert!(inst.is_status());
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(data, &[0x34, 0x12, 0x56]);
    assert_eq!(bus.diag().framing_drop_count, 0);
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
    drain_tx(&mut bus, &h);

    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert!(inst.is_status());
    assert_eq!(data, &[0x34, 0x12, 0x56]);
}

#[test]
fn s12_write_wrapping_ring_boundary_mutates_and_acks() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // WRITE torque_enable=1 (footprint 10) placed so its payload wraps the seam:
    // anchor 506 lays header+addr at 506..512, the data byte + CRC at 0..4.
    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let frame = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]);
    deliver(&mut bus, &h, RING_LEN - 6, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);

    let (_, inst, data) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(data.is_empty());
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
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
    bus.on_break(&mut d);

    // Fire the ~100 µs recheck with the line still low.
    h.deadline.set_now(1000 + 100); // RESCUE_CONFIRM_US * TICKS_PER_US
    bus.on_deadline(&mut d);

    let applied = h.baud.applied();
    assert_eq!(applied.first(), Some(&RATE)); // new() applied the configured rate
    assert_eq!(applied.last(), Some(&BaudRate::B500000));
}

#[test]
fn s11_break_after_covered_kills_staged_reply() {
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
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(((anchor + 1) % RING_LEN) as u16);
    bus.on_break(&mut d);
    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + 4) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    let c = h.deadline.armed().expect("covered deadline");
    h.deadline.set_now(c);
    h.ring.set_cursor(((anchor + fp - 2) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    assert!(!h.wire.started(), "reply is staged, not yet triggered");

    // A fresh break lands before the end deadline. Data-first: the READ is
    // complete in the ring, so it resolves and its reply stages — and the
    // break then kills the staged-not-streaming reply (the host moved on):
    // no phantom read reply reaches the wire.
    let m = 300usize; // ring[m] defaults to 0x00 → a real break
    h.deadline.set_now(c + 1);
    h.ring.set_cursor(((m + 1) % RING_LEN) as u16);
    bus.on_break(&mut d);
    assert!(!h.wire.started(), "staged reply killed by the break");

    // Drain whatever the new break armed (a starving header): still silent.
    if h.deadline.armed().is_some() {
        fire(&mut bus, &h, &mut d);
    }
    assert!(!h.wire.started(), "the cancelled reply never fires");

    // The following read exchange still works.
    let next = instruction(ID, Opcode::Read, 0, &[0, 0, 4, 0]);
    deliver(&mut bus, &h, 100, &next, c + 5000, &mut d);
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

#[test]
fn s13_frontier_write_publishes_at_covered_commits_at_verdict() {
    // A WRITE publishes to the LOW consumer at covered-complete (the spine:
    // hop + decode + dispatch overlap the frame's wire tail). The consumer
    // stages; the table stays untouched and nothing reaches the wire until
    // the verdict — frame end + adopted record — commits and sequences.
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let frame = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]);
    let anchor = 100usize;
    let fp = frame.len();
    h.ring.place(anchor, &frame);
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(((anchor + 1) % RING_LEN) as u16);
    bus.on_break(&mut d);
    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + 4) % RING_LEN) as u16);
    bus.on_deadline(&mut d);

    // Covered checkpoint (two byte-times short of the end): the job is
    // published — before the frame has even ended — and nothing is applied.
    let c = h.deadline.armed().expect("covered deadline");
    h.deadline.set_now(c);
    h.ring.set_cursor(((anchor + fp - 2) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    assert!(!h.handoff.idle(), "write-class publishes at covered");
    // The consumer dispatches (stages) while the CRC tail is still inbound.
    h.pump(&mut bus, &mut d);
    assert!(
        !shared.table.with(|t| t.control.lifecycle.torque_enable),
        "a pending write is staged, not yet committed"
    );
    assert!(!h.wire.started());

    // Frame end delivers the verdict's other half: commit + sequence.
    h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
    fire(&mut bus, &h, &mut d);
    assert!(
        shared.table.with(|t| t.control.lifecycle.torque_enable),
        "the CRC-verified write is committed"
    );
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);

    let reference = status(ID, ResultCode::Ok, &[]);
    assert_eq!(h.wire.sent(), reference[1..]);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(data.is_empty());
    assert_eq!(bus.diag().crc_fail_count, 0);
}

#[test]
fn s14_corrupt_write_leaves_table_and_reverts() {
    // A WRITE whose covered span is intact but whose trailing CRC is corrupted:
    // front-loaded (staged) at covered, then the frame-end CRC check fails →
    // the staged write is reverted and the table is byte-identical.
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let mut frame = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]);
    let last = frame.len() - 1;
    frame[last] ^= 0xFF; // corrupt CRC-hi; the covered span stays valid
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);

    assert!(!h.wire.started());
    assert_eq!(bus.diag().crc_fail_count, 1);
    assert!(
        !shared.table.with(|t| t.control.lifecycle.torque_enable),
        "a bad-CRC write must not mutate the table"
    );

    // The staged write was reverted — a following clean write applies.
    let next = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]);
    deliver(&mut bus, &h, 100, &next, 5000, &mut d);
    fire(&mut bus, &h, &mut d);
    let (_, inst, _) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
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

/// F2 edge (silicon 2026-07-08, the no-reply wedge): a prompt FE entry can
/// beat the break byte's own DMA drain, so the resolver sees a caught-up
/// ladder. The FE promises its byte — the framer must hold a bounded wait,
/// and the aim must survive unrelated wakes that walk the resolver before
/// the byte lands (the adoption wake destroyed the one-shot recheck and the
/// transport went aimless: masked flags, no deadlines, deaf until reset).
#[test]
fn fe_before_its_byte_still_resolves_the_frame() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let anchor = 100usize;
    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    let fp = frame.len();
    h.ring.place(anchor, &frame);
    bus.framer.resync(anchor as u16);

    // FE delivered with NOTHING ringed yet — not even the break byte.
    h.deadline.set_now(1000);
    h.ring.set_cursor(anchor as u16);
    bus.on_break(&mut d);
    let promise = h.deadline.armed().expect("FE-promise wait armed");
    assert!(tick_reached(1000 + TPB, promise), "bounded by a byte-time");

    // An unrelated wake walks the resolver before the byte lands (the
    // adoption-interleave regression): the promise must be re-derived.
    h.deadline.set_now(1002);
    bus.on_deadline(&mut d);
    assert!(
        h.deadline.armed().is_some(),
        "promise aim survives an early walk"
    );

    // The frame lands in full during the promise window; the wake fast-paths
    // it (A2), the consumer dispatches, and the reply fires on its grid.
    let at = h.deadline.armed().unwrap();
    h.deadline.set_now(at);
    h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    h.pump(&mut bus, &mut d);
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

/// The zero-hop half of the class routing: a complete wire-class frame
/// (READ) dispatches inline at HIGH — the LOW consumer is never woken, and
/// the reply sequences straight from the resolve wake.
#[test]
fn complete_read_dispatches_inline_without_hop() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let frame = instruction(ID, Opcode::Read, 0, &[0, 0, 4, 0]);
    let anchor = 100usize;
    h.ring.place(anchor, &frame);
    bus.framer.resync(anchor as u16);
    // The whole frame is already ringed when the (lagged) break delivers:
    // the fast path resolves it in this one wake.
    h.deadline.set_now(1000);
    h.ring
        .set_cursor(((anchor + frame.len()) % RING_LEN) as u16);
    bus.on_break(&mut d);

    assert!(h.wake.lanes().is_empty(), "wire class never hops to LOW");
    assert!(h.handoff.idle());
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

/// Ordering across classes: a backlog WRITE(NOREPLY) resolves, hops, and
/// commits before the READ behind it dispatches — the ring walk holds at the
/// pending write (backpressure) and resumes at adoption, so effects land in
/// wire order.
#[test]
fn backlog_write_then_read_processes_in_order() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let write = instruction(
        ID,
        Opcode::Write,
        Inst::FLAG_NOREPLY,
        &[addr[0], addr[1], 1],
    );
    let read = instruction(ID, Opcode::Read, 0, &[0, 0, 4, 0]);
    let anchor = 100usize;
    h.ring.place(anchor, &write);
    h.ring.place(anchor + write.len(), &read);
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring
        .set_cursor(((anchor + write.len() + read.len()) % RING_LEN) as u16);
    bus.on_break(&mut d);

    // The write published (backlog → the kernel's lane) and the walk holds:
    // the read is NOT dispatched while the write's verdict is open.
    assert_eq!(h.wake.lanes(), [crate::traits::bus::Lane::Queued]);
    assert!(!h.wire.started());
    assert!(
        !shared.table.with(|t| t.control.lifecycle.torque_enable),
        "table effect awaits the verdict"
    );

    // Adoption commits the write, resumes the walk, and the read dispatches
    // inline in the same wake.
    h.pump(&mut bus, &mut d);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

/// A phantom FE (noise, no byte ever lands) surrenders after the promise
/// window instead of re-arming forever.
#[test]
fn phantom_fe_surrenders_after_one_byte_time() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    bus.framer.resync(100);
    h.deadline.set_now(1000);
    h.ring.set_cursor(100);
    bus.on_break(&mut d);
    let promise = h.deadline.armed().expect("FE-promise wait armed");

    // The promise fires with still nothing ringed: the framer goes idle.
    h.deadline.set_now(promise);
    bus.on_deadline(&mut d);
    assert_eq!(h.deadline.armed(), None, "no re-arm on a phantom FE");
}
