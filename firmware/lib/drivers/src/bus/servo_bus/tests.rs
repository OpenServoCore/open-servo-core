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
// REPLY_GAP_US × TestProviders TICKS_PER_US(1).
const REPLY_GAP: u32 = 12;

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

/// Deliver a whole frame: break, then the header deadline with the covered
/// span already ringed (Covered emits and dispatch runs from that wake),
/// then the end deadline (the verdict). Timing is ring-cadence (A2 extended
/// to the clock): every aim projects `now + missing byte-times` from live
/// ring state, so the wakes land deterministically for any footprint.
/// Returns the frozen packet-end estimate the reply's reply gap is measured
/// from: the covered wake + the 2-byte CRC tail at wire pace.
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

    // Deadline A: the header locks, and with the covered span ringed the
    // covered checkpoint fires from this same wake — the dispatch point.
    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + fp - 2) % RING_LEN) as u16);
    bus.on_deadline(d);

    // The frozen ring-cadence estimate (missing = the 2 CRC bytes; the
    // drift adder is zero at these spans on the test clock).
    let end = a.wrapping_add(2 * TPB);

    // Deadline B (end_due): the whole frame is in — verdict + sequencing.
    let b = h.deadline.armed().expect("deadline B");
    h.deadline.set_now(b);
    h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
    bus.on_deadline(d);
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

    // Reply is staged but not on the wire until the reply gap deadline.
    assert!(!h.wire.started());
    assert_eq!(h.deadline.armed(), Some(end + REPLY_GAP));

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
    // Armed at end + reply gap + reclaim; not yet triggerable.
    assert_eq!(h.deadline.armed(), Some(end + REPLY_GAP + 600));
    assert!(!h.wire.started());

    // Predecessor (id 5) status frame → our slot pends.
    let pre = status(5, ResultCode::Ok, &[0xAA, 0xBB]);
    deliver(&mut bus, &h, 200, &pre, end + 2, &mut d);
    assert!(!h.wire.started());

    // Now the reply gap deadline triggers our reply, not silent.
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

    // Fire the ~100 µs recheck with the line still low: a passing first
    // sample re-arms one byte-time out instead of switching.
    h.deadline.set_now(1000 + 100); // RESCUE_CONFIRM_US * TICKS_PER_US
    bus.on_deadline(&mut d);
    assert_eq!(
        h.baud.applied().last(),
        Some(&RATE),
        "one sample never switches"
    );

    // The second sample, still low, still no ring progress → rescue. Pump the
    // mux: framer starvation rechecks interleave with the reconfirm slot.
    for _ in 0..4 {
        if h.baud.applied().last() == Some(&BaudRate::B500000) {
            break;
        }
        fire(&mut bus, &h, &mut d);
    }

    let applied = h.baud.applied();
    assert_eq!(applied.first(), Some(&RATE)); // new() applied the configured rate
    assert_eq!(applied.last(), Some(&BaudRate::B500000));
}

/// The phantom-rescue alias (bench 2026-07-09 at 1M): a host TX stall
/// freezes the cursor across the +100 µs confirm while the sample lands
/// inside a data byte's low bits. The byte in flight completes before the
/// second sample — its ring progress must veto the switch.
#[test]
fn s10b_rescue_reconfirm_vetoes_data_alias() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    h.line.set_low(true);
    h.deadline.set_now(1000);
    h.ring.set_cursor(1);
    bus.on_break(&mut d);

    // First sample: line low (a data byte's start/zero bits), cursor frozen.
    h.deadline.set_now(1000 + 100);
    bus.on_deadline(&mut d);
    assert_eq!(h.baud.applied().last(), Some(&RATE));

    // The aliased byte completes before the second sample: cursor moves.
    // Pump past the interleaved framer wakes so the reconfirm slot fires.
    h.ring.set_cursor(2);
    for _ in 0..4 {
        fire(&mut bus, &h, &mut d);
    }

    assert_eq!(
        h.baud.applied().last(),
        Some(&RATE),
        "data alias must not rescue"
    );
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
fn s13_frontier_write_stages_at_covered_commits_at_verdict() {
    // A WRITE dispatches inline at covered-complete (the spine: the CRC feed
    // chews the covered span under the dispatch body). The write stages; the
    // table stays untouched and nothing reaches the wire until the verdict at
    // frame end commits and sequences.
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
    // The write dispatched inline at covered — staged while the CRC tail is
    // still inbound; the table stays untouched until the verdict.
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

    // MGMT REBOOT (unicast) acks then stages a reboot honored on poll —
    // withheld while the ack drains (a reset mid-frame truncates the ack on
    // the wire; bench 2026-07-10), taken once the TX released.
    let frame = instruction(ID, Opcode::Mgmt, 0, &[MgmtOp::Reboot as u8]);
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);
    let (_, inst, _) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(bus.take_reboot().is_none(), "withheld while the ack drains");
    drain_tx(&mut bus, &h);
    assert!(bus.take_reboot().is_some());
    assert!(bus.take_reboot().is_none());
}

/// A complete frame (READ) dispatches inline at HIGH and its reply sequences
/// straight from the resolve wake — no deferral to a later verdict.
#[test]
fn complete_read_dispatches_inline() {
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

    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

/// Ordering across a backlog: a WRITE(NOREPLY) resolves and commits before the
/// READ behind it dispatches — drive_framer resolves each complete frame and
/// runs its verdict fully before resolving the next, so effects land in wire
/// order.
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

    // Both complete frames resolve in this one wake: the NOREPLY write commits
    // inline, then the read dispatches behind it — effects in wire order. The
    // read's reply is staged (not yet fired), its chain armed.
    assert!(
        shared.table.with(|t| t.control.lifecycle.torque_enable),
        "the backlog write committed before the read dispatched"
    );
    assert!(!h.wire.started());
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

/// A wire-fault wake whose evidence hasn't ringed yet (the drain a beat
/// behind ISR entry — or the FE consumed by a garble-latched flag's SR-DR
/// pair, so this wake is the LAST one the frame gets) must leave a framer
/// recheck armed: one byte-time later the ring tells the truth and the
/// frame resolves by data. Without it the frame sat complete-but-unresolved
/// until unrelated traffic (the post-garble one-instruction-late residue,
/// bench 2026-07-10).
#[test]
fn break_service_before_drain_rechecks_and_answers() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let anchor = 100usize;
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    // FE serviced with the cursor still AT the anchor: nothing new ringed.
    h.ring.set_cursor(anchor as u16);
    bus.on_break(&mut d);
    let recheck = h.deadline.armed().expect("recheck armed on empty evidence");
    assert_eq!(recheck, 1000 + TPB);

    // By the recheck the whole ping has ringed (no further FE ever fires).
    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    h.ring.place(anchor, &frame);
    h.ring
        .set_cursor(((anchor + frame.len()) % RING_LEN) as u16);
    h.deadline.set_now(recheck);
    bus.on_deadline(&mut d);

    // Resolved from ring data alone: the reply stages and fires.
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

/// §6 A4 storm throttle: a zero-progress fault service mutes the fault wake
/// (the latched flag would otherwise re-enter the vector continuously), the
/// deadline slot polls the ring at FAULT_MUTE_POLL_BYTE_TIMES while quiet,
/// and ring progress restores the wake and resolves in the same wake.
#[test]
fn zero_progress_fault_mutes_wake_until_ring_progress() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);
    let poll = super::FAULT_MUTE_POLL_BYTE_TIMES * TPB;

    let anchor = 100usize;
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(anchor as u16);
    assert!(h.line.fault_wake());
    bus.on_break(&mut d);
    // Recorded, not yet muted: the recheck delivers the verdict — muting at
    // the fault service misfires on the routine mid-burst latch.
    assert!(h.line.fault_wake(), "fault service records, recheck mutes");
    assert_eq!(h.deadline.armed(), Some(1000 + TPB));

    // Recheck finds nothing: the wire is provably quiet — mute and fall
    // back to the poll cadence.
    h.deadline.set_now(1000 + TPB);
    bus.on_deadline(&mut d);
    assert!(!h.line.fault_wake(), "quiet recheck mutes");
    assert_eq!(h.deadline.armed(), Some(1000 + TPB + poll));

    // Quiet poll: still muted, next poll one cadence out.
    h.deadline.set_now(1000 + TPB + poll);
    bus.on_deadline(&mut d);
    assert!(!h.line.fault_wake());
    assert_eq!(h.deadline.armed(), Some(1000 + TPB + 2 * poll));

    // A whole ping rings before the next poll (its break FE never woke us —
    // the wake is muted): the poll's progress check restores the wake and
    // the same wake resolves the frame from ring data.
    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    h.ring.place(anchor, &frame);
    h.ring
        .set_cursor(((anchor + frame.len()) % RING_LEN) as u16);
    h.deadline.set_now(1000 + TPB + 2 * poll);
    bus.on_deadline(&mut d);
    assert!(h.line.fault_wake(), "ring progress restores the wake");

    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

/// §6 A4 burst corner (bench 2026-07-11: hot chains fell 95%→83% when the
/// fault service muted directly): a zero-progress fault service whose
/// evidence arrives within the byte-time must NEVER drop the wake — the
/// recheck sees the progress, resolves by data, and live breaks keep
/// waking the driver.
#[test]
fn mid_burst_latched_fault_never_mutes() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let anchor = 100usize;
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(anchor as u16);
    // Lagged-FE re-entry: zero progress at service time.
    bus.on_break(&mut d);
    assert!(h.line.fault_wake());

    // The next frame's bytes land before the recheck (zero-gap burst).
    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    h.ring.place(anchor, &frame);
    h.ring
        .set_cursor(((anchor + frame.len()) % RING_LEN) as u16);
    h.deadline.set_now(1000 + TPB);
    bus.on_deadline(&mut d);
    assert!(
        h.line.fault_wake(),
        "progress at the recheck keeps the wake"
    );

    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

/// §6 A4 × §9.1: a rescue pulse landing while the wake is muted delivers no
/// FE wake, so the poll's line-low mirror must arm the confirm — and the
/// sub-100 µs polls must not push a pending confirm out (the `rescue_at`
/// gate). The confirm chain then applies the 0.5 M rescue rate under the
/// mute, and the poll keeps running at the new byte-time.
#[test]
fn rescue_pulse_while_muted_confirms_via_poll() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);
    let poll = super::FAULT_MUTE_POLL_BYTE_TIMES * TPB;

    let anchor = 100usize;
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(anchor as u16);
    bus.on_break(&mut d);
    h.deadline.set_now(1000 + TPB);
    bus.on_deadline(&mut d);
    assert!(!h.line.fault_wake());
    let poll1 = 1000 + TPB + poll;

    // Pulse start: the FE latches silently; the poll sees the low line and
    // arms the +100 µs confirm.
    h.line.set_low(true);
    h.deadline.set_now(poll1);
    bus.on_deadline(&mut d);
    let confirm = poll1 + 100; // RESCUE_CONFIRM_US × test TICKS_PER_US(1)
    assert_eq!(h.deadline.armed(), Some(poll1 + poll), "poll fires first");

    // The next poll must NOT re-arm (and so push out) the pending confirm.
    h.deadline.set_now(poll1 + poll);
    bus.on_deadline(&mut d);
    assert_eq!(h.deadline.armed(), Some(confirm));

    // Confirm sample 1 (cursor frozen, line low) → reconfirm a byte-time on.
    h.deadline.set_now(confirm);
    bus.on_deadline(&mut d);
    let reconfirm = h.deadline.armed().expect("reconfirm armed");
    h.deadline.set_now(reconfirm);
    bus.on_deadline(&mut d);
    assert_eq!(h.baud.applied().last(), Some(&BaudRate::B500000));
    assert!(!h.line.fault_wake(), "rescue applies under the mute");

    // Pulse released; host bytes at the rescue rate ring. The poll — now on
    // 0.5 M byte-times — sees progress and restores the wake.
    h.line.set_low(false);
    h.ring.place(anchor, &[0x00]);
    h.ring.set_cursor((anchor as u16) + 1);
    let next = h.deadline.armed().expect("poll re-armed post-rescue");
    h.deadline.set_now(next);
    bus.on_deadline(&mut d);
    assert!(h.line.fault_wake());
}
