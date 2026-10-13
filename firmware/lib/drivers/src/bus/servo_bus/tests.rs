//! Composite spec tests: real byte frames laid into the ring, driven through
//! `on_break`/`on_deadline` against the real core dispatcher, asserting on the
//! recorded wire bytes. The fakes advance `now` by reading the armed tick, so
//! every deadline the mux computes is exercised end to end.

use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Id, Inst, MgmtOp, Opcode, ResultCode};
use osc_servo_core::regions::CONTROL_BASE_ADDR;
use osc_servo_core::regions::config::addr::common::ID as ID_ADDR;
use osc_servo_core::{BaudRate, Dispatch, RegionStorage, Session, Shared};

use crate::bus::RESCUE_LOW_US;
use crate::mocks::bus::{FakeWire, Harness, RING_LEN, WireEvent};

const ID: u8 = 7;
const RATE: BaudRate = BaudRate::B1000000;
const TPB: u32 = 10; // TICKS_PER_US(1) * 1e7 / 1e6
// REPLY_GAP_US x TestProviders TICKS_PER_US(1).
const REPLY_GAP: u32 = 12;

fn shared_seeded() -> Shared {
    let shared = Shared::new();
    shared.table.with_mut(|t| {
        t.config.common.model_number = 0x1234;
        t.config.common.firmware_version = 0x7856;
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
/// then the end deadline (the verdict). Timing is ring-cadence (time derived
/// from the stream): every aim projects `now + missing byte-times` from live
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
    // The ladder reaches `anchor` through prior traffic in production; the
    // harness fabricates positions, so bootstrap it explicitly.
    bus.framer.resync(anchor as u16);
    deliver_next(bus, h, anchor, frame, now0, d)
}

/// [`deliver`] for a frame the ladder reaches on its own: the one right
/// behind the previous frame's footprint.
fn deliver_next<D: Dispatch>(
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
    bus.on_break(d);

    // Deadline A: the header locks, and with the covered span ringed the
    // covered checkpoint fires from this same wake -- the dispatch point.
    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + fp - 2) % RING_LEN) as u16);
    bus.on_deadline(d);

    // The frozen ring-cadence estimate (missing = the 2 CRC bytes; the
    // drift adder is zero at these spans on the test clock).
    let end = a.wrapping_add(2 * TPB);

    // Deadline B (end_due): the whole frame is in -- verdict + sequencing.
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
            _ => {}
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
    let reference = status(ID, ResultCode::Ok, &[0x34, 0x12, 0x56, 0x78]);
    assert_eq!(h.wire.sent(), reference[1..]);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert!(inst.is_status());
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(data, &[0x34, 0x12, 0x56, 0x78]);
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
    // WRITE torque_enable=1 -> ack, table mutated.
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

    // TX drains -> deferred id apply. A ping to 42 now replies.
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

    // Uniform GREAD addr 0 count 4, ids [5, 7] -> we are slot 1.
    let gread = instruction(ID, Opcode::Gread, 0, &[0, 0, 4, 0, 5, ID]);
    let gread = broadcast_id(gread);
    let end = deliver(&mut bus, &h, 100, &gread, 1000, &mut d);
    // Armed at end + reply gap + reclaim; not yet triggerable.
    assert_eq!(h.deadline.armed(), Some(end + REPLY_GAP + 600));
    assert!(!h.wire.started());

    // Predecessor (id 5) status frame, the GREAD's 4 bytes -> our slot pends.
    let pre = status(5, ResultCode::Ok, &[0xAA, 0xBB, 0xCC, 0xDD]);
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
    // Anchor parity is irrelevant (sec 3.2 self-aligning feed): an odd-anchored
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
    assert_eq!(data, &[0x34, 0x12, 0x56, 0x78]);
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
    assert_eq!(data, &[0x34, 0x12, 0x56, 0x78]);
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

/// Main-loop line samples every `step` ticks over `[from, to]`, all low.
fn sample_low(
    bus: &mut crate::bus::ServoBus<crate::mocks::bus::TestProviders>,
    h: &Harness,
    from: u32,
    to: u32,
    step: usize,
) {
    for t in (from..=to).step_by(step) {
        h.deadline.set_now(t);
        bus.sample_rescue(true);
    }
}

fn rescued(h: &Harness) -> bool {
    h.baud.applied().last() == Some(&BaudRate::B500000)
}

/// sec 9.1 rescue is the main-loop line sampler's declaration (the break
/// detector wakes once per span, so no wake can measure a pulse's length):
/// low samples spanning `RESCUE_LOW_US` with the ring frozen switch to the
/// rescue rate and resync at the provably-still cursor.
#[test]
fn s10_rescue_break_switches_to_500k_and_resyncs() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);

    // The pulse's ringed 0x00 at index 0 restarts the window, which opens
    // at the next frozen sample.
    h.ring.set_cursor(1);
    sample_low(&mut bus, &h, 950, 950, 1);
    sample_low(&mut bus, &h, 1000, 1000 + RESCUE_LOW_US - 1, 50);
    assert!(!rescued(&h), "declared short of the window");
    sample_low(&mut bus, &h, 1000 + RESCUE_LOW_US, 1000 + RESCUE_LOW_US, 1);

    let applied = h.baud.applied();
    assert_eq!(applied.first(), Some(&RATE)); // new() applied the configured rate
    assert!(rescued(&h));
    // Every transport slot cleared: nothing armed until the host talks.
    assert_eq!(h.deadline.armed(), None);
}

/// sec 9.1: traffic moves the ring, so a line that reads low at every sample
/// while bytes keep arriving is never a pulse.
#[test]
fn s10_ring_progress_restarts_the_rescue_window() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);

    for k in 0..20u32 {
        h.ring.set_cursor(k as u16);
        h.deadline.set_now(1000 + k * 50);
        bus.sample_rescue(true);
    }
    assert!(!rescued(&h));
}

/// sec 9.1 own TX: HDSEL keeps the servo's own bytes out of the ring, so
/// samples landing on their low bits find the ring frozen. A sample taken
/// while the reply streams restarts the window, so a TX release between
/// two low samples leaves no window to inherit; a real hold after the TX
/// still declares a full window later.
#[test]
fn s10_own_tx_never_reads_as_a_rescue_pulse() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);
    fire(&mut bus, &h, &mut d);
    assert!(h.wire.started());

    let t0 = 5_000;
    sample_low(&mut bus, &h, t0, t0 + 4 * RESCUE_LOW_US, 50);
    assert!(!rescued(&h), "own TX read as a pulse");

    drain_tx(&mut bus, &h);
    let t1 = t0 + 4 * RESCUE_LOW_US + 1;
    sample_low(&mut bus, &h, t1, t1 + RESCUE_LOW_US - 1, 50);
    assert!(!rescued(&h), "the window outlived the TX release");
    sample_low(&mut bus, &h, t1 + RESCUE_LOW_US, t1 + RESCUE_LOW_US, 1);
    assert!(rescued(&h));
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
    // complete in the ring, so it resolves and its reply stages -- and the
    // break then kills the staged-not-streaming reply (the host moved on):
    // no phantom read reply reaches the wire.
    let m = 300usize; // ring[m] defaults to 0x00 -> a real break
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

/// sec 9.2 exemption to s11's wire-safety kill: a broadcast-ENUM reply's job is
/// to collide -- a peer matcher's leading break (its 0x00 ringing right
/// behind our frame, exactly the s11 kill evidence) must NOT kill the staged
/// reply, or the wire carries one clean frame, the walk records a unique
/// match, and the laggard's whole subtree goes invisible.
#[test]
fn enum_reply_survives_a_peer_matchers_break() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    // Broadcast ENUM, empty prefix: every servo matches (the tree root).
    // Verdict-first, so the reply stages at the frame end with its trigger
    // armed one reply gap out.
    let frame = instruction(
        Id::BROADCAST.as_byte(),
        Opcode::Mgmt,
        0,
        &[MgmtOp::Enum as u8, 0],
    );
    let end = deliver(&mut bus, &h, 100, &frame, 1000, &mut d);
    assert!(!h.wire.started(), "reply staged, trigger one reply gap out");

    // The peer matcher's break lands inside the reply gap.
    let m = 300usize; // ring[m] defaults to 0x00 -> a real break
    h.deadline.set_now(end + 2);
    h.ring.set_cursor(((m + 1) % RING_LEN) as u16);
    bus.on_break(&mut d);

    // The staged reply survives and fires -- colliding energy on a real
    // wire, the signal the walk descends on. The trigger sits up to
    // SLOTS-1 byte-times out (sec 9.2 slot draw), behind the peer break's
    // starving-header rechecks -- drain generously.
    let mut guard = 0;
    while !h.wire.started() && h.deadline.armed().is_some() && guard < 64 {
        fire(&mut bus, &h, &mut d);
        guard += 1;
    }
    assert!(
        h.wire.started(),
        "the ENUM reply must not yield to the break"
    );
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(data.len(), 16, "the full UID ships");
}

/// sec 9.2 slot delay: an ENUM reply's trigger rides reply gap + a slot draw --
/// `(folded uid CRC ^ tick) & (SLOTS-1)` byte-times -- so cycle-identical
/// twin matchers can't answer in unison (a sub-bit-aligned superposition of
/// near-equal frames reads back as ONE clean frame). The default
/// all-zero UID folds to key 0, so the draw here is the tick's low bits.
/// Plain replies keep the bare grid (s1 pins that side).
#[test]
fn enum_reply_draws_its_slot_delay() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let frame = instruction(
        Id::BROADCAST.as_byte(),
        Opcode::Mgmt,
        0,
        &[MgmtOp::Enum as u8, 0],
    );
    let anchor = 100usize;
    let fp = frame.len();
    h.ring.place(anchor, &frame);
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(((anchor + 1) % RING_LEN) as u16);
    bus.on_break(&mut d);

    let a = h.deadline.armed().expect("deadline A");
    h.deadline.set_now(a);
    h.ring.set_cursor(((anchor + fp - 2) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    let end = a.wrapping_add(2 * TPB);

    // Verdict-first: dispatch + sequencing run at deadline B, so the slot
    // draw reads the clock at exactly `b`.
    let b = h.deadline.armed().expect("deadline B");
    h.deadline.set_now(b);
    h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
    bus.on_deadline(&mut d);

    let draw = (b as u8 & (wire::ENUM_REPLY_SLOTS - 1)) as u32;
    assert_eq!(
        h.deadline.armed(),
        Some(end + REPLY_GAP + draw * TPB),
        "trigger = reply gap + slot draw"
    );

    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (_, inst, data) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(data.len(), 16);
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
    // published -- before the frame has even ended -- and nothing is applied.
    let c = h.deadline.armed().expect("covered deadline");
    h.deadline.set_now(c);
    h.ring.set_cursor(((anchor + fp - 2) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    // The write dispatched inline at covered -- staged while the CRC tail is
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

/// `Dispatch` wrapper that records the wire state at every commit.
struct CommitWatch<'a, D> {
    inner: D,
    wire: &'a FakeWire,
    commits: u32,
    wire_started_at_commit: bool,
}

impl<D: Dispatch> Dispatch for CommitWatch<'_, D> {
    fn dispatch<R: osc_servo_core::Reply>(
        &mut self,
        req: osc_servo_core::Request<'_>,
        ctx: osc_servo_core::RequestCtx,
        reply: &mut R,
    ) -> osc_servo_core::Dispatched {
        self.inner.dispatch(req, ctx, reply)
    }

    fn commit<R: osc_servo_core::Reply>(&mut self, reply: &mut R) {
        self.commits += 1;
        self.wire_started_at_commit = self.wire.started();
        self.inner.commit(reply);
    }

    fn revert(&mut self) {
        self.inner.revert();
    }
}

/// The verdict commits a CRC-verified write before it sequences the ack, so
/// the status never leaves ahead of the effect it acknowledges. The verdict
/// wake lands a dispatch body past the frame end - beyond the reply gap, so
/// sequencing triggers inline (the silicon case: the covered dispatch
/// overruns the gap).
#[test]
fn acked_write_commits_before_its_reply_triggers() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = CommitWatch {
        inner: session.dispatcher(&shared),
        wire: &h.wire,
        commits: 0,
        wire_started_at_commit: false,
    };

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
    h.ring.set_cursor(((anchor + fp - 2) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    assert_eq!(d.commits, 0, "staged, not committed, at covered");

    let b = h.deadline.armed().expect("deadline B");
    h.deadline.set_now(b.wrapping_add(REPLY_GAP + 30));
    h.ring.set_cursor(((anchor + fp) % RING_LEN) as u16);
    bus.on_deadline(&mut d);
    assert_eq!(d.commits, 1);
    assert!(
        !d.wire_started_at_commit,
        "the commit precedes the status break"
    );
    assert!(h.wire.started(), "the late verdict triggers inline");
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
    drain_tx(&mut bus, &h);
    let (_, inst, data) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(data.is_empty());
}

/// An ack the host abandons is killed after its write committed. Every path
/// that drops a staged reply (stale-reply kill, verdict-first supersede,
/// `drop_staged`, rescue) runs after the verdict body, so none
/// can lose a CRC-verified write; this pins a following break and the
/// rescue declaration.
#[test]
fn killed_ack_keeps_its_verified_write() {
    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let write = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]);
    for rescue in [false, true] {
        let h = Harness::new();
        let mut bus = h.build(ID, RATE, 60);
        let shared = Shared::new();
        let mut session = Session::new();
        let mut d = session.dispatcher(&shared);

        let end = deliver(&mut bus, &h, 100, &write, 1000, &mut d);
        assert_eq!(h.deadline.armed(), Some(end + REPLY_GAP), "ack staged");
        assert!(!h.wire.started());

        if rescue {
            sample_low(&mut bus, &h, end + 1, end + 2 + RESCUE_LOW_US, 1);
            assert!(rescued(&h));
        } else {
            let m = 300usize;
            h.deadline.set_now(end + 1);
            h.ring.set_cursor(((m + 1) % RING_LEN) as u16);
            bus.on_break(&mut d);
            if h.deadline.armed().is_some() {
                fire(&mut bus, &h, &mut d);
            }
        }
        assert!(!h.wire.started(), "the ack is killed, rescue {rescue}");
        assert!(
            shared.table.with(|t| t.control.lifecycle.torque_enable),
            "the verified write survives the kill, rescue {rescue}"
        );
    }
}

#[test]
fn s14_corrupt_write_leaves_table_and_reverts() {
    // A WRITE whose covered span is intact but whose trailing CRC is corrupted:
    // front-loaded (staged) at covered, then the frame-end CRC check fails ->
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

    // The staged write was reverted -- a following clean write applies.
    let next = instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]);
    deliver(&mut bus, &h, 100, &next, 5000, &mut d);
    fire(&mut bus, &h, &mut d);
    let (_, inst, _) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

/// `Dispatch` wrapper that counts dispatches.
struct DispatchCount<D> {
    inner: D,
    dispatches: u32,
}

impl<D: Dispatch> Dispatch for DispatchCount<D> {
    fn dispatch<R: osc_servo_core::Reply>(
        &mut self,
        req: osc_servo_core::Request<'_>,
        ctx: osc_servo_core::RequestCtx,
        reply: &mut R,
    ) -> osc_servo_core::Dispatched {
        self.dispatches += 1;
        self.inner.dispatch(req, ctx, reply)
    }

    fn commit<R: osc_servo_core::Reply>(&mut self, reply: &mut R) {
        self.inner.commit(reply);
    }

    fn revert(&mut self) {
        self.inner.revert();
    }
}

/// An id no servo on the test bus holds.
const FOREIGN: u8 = 0x63;

/// A READ for another servo is dropped at its header: no dispatch, no
/// reply, nothing left armed, and the CRC gate still moves the ladder.
#[test]
fn foreign_read_costs_no_dispatch_and_no_reply() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = DispatchCount {
        inner: session.dispatcher(&shared),
        dispatches: 0,
    };

    let frame = instruction(FOREIGN, Opcode::Read, 0, &[0, 0, 4, 0]);
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);

    assert_eq!(d.dispatches, 0);
    assert!(!h.wire.started());
    assert_eq!(h.deadline.armed(), None);
    assert!(bus.framer.caught_up((100 + frame.len()) as u16));
    assert_eq!(bus.diag().crc_fail_count, 0);
}

/// The frame behind a foreign one resolves: at the derived anchor when the
/// foreign frame verified, and through the hunt (one byte in) when it
/// failed its CRC. The foreign frame carries no 0x00 past its break, so the
/// hunt finds no junk candidate inside it.
#[test]
fn foreign_frame_moves_the_ladder_on_good_and_bad_crc() {
    for corrupt in [false, true] {
        let h = Harness::new();
        let mut bus = h.build(ID, RATE, 60);
        let shared = shared_seeded();
        let mut session = Session::new();
        let mut d = DispatchCount {
            inner: session.dispatcher(&shared),
            dispatches: 0,
        };

        let mut foreign = instruction(FOREIGN, Opcode::Write, 0, &[0x10, 0x01, 0x05]);
        if corrupt {
            let last = foreign.len() - 1;
            foreign[last] ^= 0xFF;
        }
        assert!(!foreign[1..].contains(&0));
        let end = deliver(&mut bus, &h, 100, &foreign, 1000, &mut d);
        assert_eq!(bus.diag().crc_fail_count, u32::from(corrupt));

        let ping = instruction(ID, Opcode::Ping, 0, &[]);
        deliver_next(&mut bus, &h, 100 + foreign.len(), &ping, end + 500, &mut d);
        fire(&mut bus, &h, &mut d);
        drain_tx(&mut bus, &h);
        let (id, inst, _) = last_reply(&h.wire);
        assert_eq!(id, ID, "corrupt={corrupt}");
        assert_eq!(inst.result(), Some(ResultCode::Ok), "corrupt={corrupt}");
        assert_eq!(d.dispatches, 1, "corrupt={corrupt}");
        assert_eq!(bus.diag().crc_fail_count, u32::from(corrupt));
        assert_eq!(bus.diag().framing_drop_count, 0);
    }
}

/// Broadcast is not foreign: a broadcast WRITE dispatches and applies,
/// silently (sec 5).
#[test]
fn broadcast_write_still_dispatches() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = DispatchCount {
        inner: session.dispatcher(&shared),
        dispatches: 0,
    };

    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let frame = broadcast_id(instruction(ID, Opcode::Write, 0, &[addr[0], addr[1], 1]));
    deliver(&mut bus, &h, 100, &frame, 1000, &mut d);

    assert_eq!(d.dispatches, 1);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
    assert!(!h.wire.started());
}

/// A status nobody waits on gets its CRC verdict: a LEN garbled long would
/// otherwise swallow the frame behind it. Here the claimed footprint runs
/// four bytes into a ping for us; the verdict fails, is counted, and the
/// hunt finds the ping's break, which still dispatches and answers.
#[test]
fn garbled_status_len_is_caught_and_the_frame_behind_it_answers() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = DispatchCount {
        inner: session.dispatcher(&shared),
        dispatches: 0,
    };

    let mut st = status(5, ResultCode::Ok, &[0x11, 0x22, 0x33, 0x44]);
    st[2] += 4;
    assert!(!st[1..].contains(&0));
    let ping = instruction(ID, Opcode::Ping, 0, &[]);
    let (a, b) = (100, 100 + st.len());
    h.ring.place(a, &st);
    h.ring.place(b, &ping);
    bus.framer.resync(a as u16);

    h.deadline.set_now(1000);
    h.ring.set_cursor((a + 1) as u16);
    bus.on_break(&mut d);
    h.ring.set_cursor(b as u16);
    fire(&mut bus, &h, &mut d);
    let horizon = h.deadline.armed().expect("starve horizon");
    h.deadline.set_now(horizon - 100);
    h.ring.set_cursor((b + 1) as u16);
    bus.on_break(&mut d);
    h.ring.set_cursor((b + ping.len()) as u16);
    fire(&mut bus, &h, &mut d);
    assert_eq!(bus.diag().crc_fail_count, 1);
    assert_eq!(d.dispatches, 1);

    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(bus.diag().framing_drop_count, 0);
}

/// A slot waiting on a uniform GREAD knows every predecessor's status
/// size: the span it read, or an empty error status. A predecessor whose
/// LEN says anything else is dropped before it counts (a short one would
/// time the slot into the predecessor's own tail), so the slot answers at
/// its reclaim, flagged; an error status still counts.
#[test]
fn waiting_slot_drops_a_status_len_its_gread_rules_out() {
    for (pre, admitted) in [
        (status(5, ResultCode::Ok, &[0xAA, 0xBB]), false),
        (status(5, ResultCode::Range, &[]), true),
    ] {
        let h = Harness::new();
        let mut bus = h.build(ID, RATE, 600);
        let shared = shared_seeded();
        let mut session = Session::new();
        let mut d = session.dispatcher(&shared);

        // Uniform GREAD addr 0 count 4, ids [5, 7] -> we are slot 1.
        let gread = broadcast_id(instruction(ID, Opcode::Gread, 0, &[0, 0, 4, 0, 5, ID]));
        let end = deliver(&mut bus, &h, 100, &gread, 1000, &mut d);
        deliver(&mut bus, &h, 200, &pre, end + 2, &mut d);
        assert_eq!(
            bus.diag().framing_drop_count,
            u32::from(!admitted),
            "admitted={admitted}"
        );
        assert!(!h.wire.started());

        fire(&mut bus, &h, &mut d);
        drain_tx(&mut bus, &h);
        let (id, inst, _) = last_reply(&h.wire);
        assert_eq!(id, ID);
        let want = if admitted {
            ResultCode::Ok
        } else {
            ResultCode::PredecessorSilent
        };
        assert_eq!(inst.result(), Some(want), "admitted={admitted}");
    }
}

/// Own TX holds the one CRC engine from trigger to release, so a status
/// that resolves while the reply streams (a peer talking over it) waits
/// through every wake meanwhile: the release resumes the ladder, the
/// verdict runs, and the reply's own CRC comes out whole.
#[test]
fn status_verdict_waits_for_own_tx_release() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let ping = instruction(ID, Opcode::Ping, 0, &[]);
    deliver(&mut bus, &h, 100, &ping, 1000, &mut d);
    fire(&mut bus, &h, &mut d);
    assert!(h.wire.started());

    let mut st = status(5, ResultCode::Ok, &[0x11, 0x22]);
    let last = st.len() - 1;
    st[last] ^= 0xFF;
    let a = 100 + ping.len();
    h.ring.place(a, &st);
    h.ring.set_cursor((a + 1) as u16);
    bus.on_break(&mut d);
    h.ring.set_cursor((a + st.len()) as u16);
    bus.on_break(&mut d);
    assert_eq!(bus.diag().crc_fail_count, 0, "held while the reply streams");
    assert_eq!(h.deadline.armed(), None, "only the release resumes it");

    drain_tx(&mut bus, &h);
    let replies = tx_frames(&h.wire, 0);
    assert_eq!(replies.len(), 1);
    assert_frame_crc(&replies[0]);

    fire(&mut bus, &h, &mut d);
    assert_eq!(bus.diag().crc_fail_count, 1, "checked after the release");
    assert!(bus.framer.caught_up((a + st.len()) as u16));
}

/// Rewrite a frame's ID field to broadcast (group ops address via their list).
fn broadcast_id(mut frame: std::vec::Vec<u8>) -> std::vec::Vec<u8> {
    // Re-seal after changing the ID: [0x00][ID][LEN][INST]...
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

    // MGMT REBOOT (unicast) acks then stages a reboot honored on poll --
    // withheld while the ack drains (a reset mid-frame truncates the ack on
    // the wire; bench-caught), taken once the TX released.
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

/// A complete frame (READ) dispatches inline at the bus level and its reply sequences
/// straight from the resolve wake -- no deferral to a later verdict.
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

    // The newest ringed byte is the CRC tail, not a break's 0x00, so the
    // wake's re-inspection is armed ahead of the reply trigger.
    fire(&mut bus, &h, &mut d);
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

/// Ordering across a backlog: a WRITE(NOREPLY) resolves and commits before the
/// READ behind it dispatches -- drive_framer resolves each complete frame and
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
    // inline, then the read dispatches behind it -- effects in wire order. The
    // read's reply is staged (not yet fired), its chain armed.
    assert!(
        shared.table.with(|t| t.control.lifecycle.torque_enable),
        "the backlog write committed before the read dispatched"
    );
    assert!(!h.wire.started());
    // Re-inspection first (the newest ringed byte is the read's CRC tail),
    // then the reply trigger.
    fire(&mut bus, &h, &mut d);
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, data) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(&data[..2], &[0x34, 0x12]);
}

/// A wire-fault wake whose evidence hasn't ringed yet (the drain a beat
/// behind ISR entry -- or the FE consumed by a garble-latched flag's SR-DR
/// pair, so this wake is the LAST one the frame gets) must leave a framer
/// recheck armed: one byte-time later the ring tells the truth and the
/// frame resolves by data. Without it the frame sat complete-but-unresolved
/// until unrelated traffic (the post-garble one-instruction-late residue,
/// bench-caught).
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

/// sec 3.4 spurious wake (a coalesced or lagged break service -- the FE-era
/// level-pend re-fires produced the same shape): a wake with zero ring
/// progress arms only the one-byte-time recheck, and evidence landing
/// within that byte-time resolves by data -- live traffic never pays for
/// the spurious entry (bench: an eager verdict here cost hot chains
/// 95% -> 83%).
#[test]
fn spurious_wake_arms_recheck_and_resolves_by_data() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let anchor = 100usize;
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(anchor as u16);
    // Spurious entry: zero progress at service time -- the only trace is
    // the one-byte-time recheck.
    bus.on_break(&mut d);
    assert_eq!(h.deadline.armed(), Some(1000 + TPB));

    // The next frame's bytes land before the recheck (zero-gap burst): the
    // recheck resolves the frame from ring data.
    let frame = instruction(ID, Opcode::Ping, 0, &[]);
    h.ring.place(anchor, &frame);
    h.ring
        .set_cursor(((anchor + frame.len()) % RING_LEN) as u16);
    h.deadline.set_now(1000 + TPB);
    bus.on_deadline(&mut d);

    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let (id, inst, _) = last_reply(&h.wire);
    assert_eq!(id, ID);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

/// The quiet twin: a spurious wake on a genuinely idle wire costs exactly
/// one empty recheck -- after it, no deadline is armed at all (the next
/// break wakes the driver; there is no wake to mute and no poll to keep
/// alive -- errors never interrupt, the break detector is the only wake,
/// transport sec 7).
#[test]
fn spurious_wake_on_quiet_wire_costs_one_recheck() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = shared_seeded();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let anchor = 100usize;
    bus.framer.resync(anchor as u16);
    h.deadline.set_now(1000);
    h.ring.set_cursor(anchor as u16);
    bus.on_break(&mut d);
    assert_eq!(h.deadline.armed(), Some(1000 + TPB));

    h.deadline.set_now(1000 + TPB);
    bus.on_deadline(&mut d);
    assert_eq!(h.deadline.armed(), None, "idle again: nothing to poll");
}

/// Every frame of a back-to-back silent burst resolves clean however late
/// its break's wake is served (position from the stream, transport sec
/// 5.1): each wake lands 2.5 byte-times behind its break byte, with the ID
/// and LEN bytes already ringed behind it, and the ladder still walks onto
/// the break and through every whole frame.
#[test]
fn lagged_break_wakes_resolve_every_frame() {
    const F: usize = 48;
    const LAG: u32 = 25;
    const SEAM: u32 = 4;
    const BREAKS: u32 = 257;
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = Shared::new();
    let mut session = Session::new();
    let mut d = session.dispatcher(&shared);

    let addr = CONTROL_BASE_ADDR.to_le_bytes();
    let mut payload = std::vec![addr[0], addr[1]];
    payload.extend_from_slice(&[0u8; F - 8]);
    let frame = instruction(ID, Opcode::Write, Inst::FLAG_NOREPLY, &payload);
    assert_eq!(frame.len(), F);

    // Bytes of the frame whose break byte rang at `t` that have landed by `at`.
    let landed = |at: u32, t: u32| {
        if at < t {
            0
        } else {
            ((at - t) / TPB + 1).min(F as u32) as usize
        }
    };
    let mut anchor = 0usize;
    let mut t = 1000u32;
    bus.framer.resync(0);
    for _ in 0..BREAKS {
        h.ring.place(anchor, &frame);
        let wake = t + LAG;
        // Pended deadlines go first (SysTick arbitrates ahead of TIM2).
        while let Some(at) = h.deadline.armed().filter(|&at| at <= wake) {
            h.deadline.set_now(at);
            h.ring
                .set_cursor(((anchor + landed(at, t)) % RING_LEN) as u16);
            bus.on_deadline(&mut d);
        }
        h.deadline.set_now(wake);
        h.ring
            .set_cursor(((anchor + landed(wake, t)) % RING_LEN) as u16);
        bus.on_break(&mut d);
        let next = t + F as u32 * TPB + SEAM;
        while let Some(at) = h.deadline.armed().filter(|&at| at < next) {
            h.deadline.set_now(at);
            h.ring
                .set_cursor(((anchor + landed(at, t)) % RING_LEN) as u16);
            bus.on_deadline(&mut d);
        }
        anchor = (anchor + F) % RING_LEN;
        t = next;
    }
    assert_eq!(bus.diag().crc_fail_count, 0);
    assert_eq!(bus.diag().framing_drop_count, 0);
}

// --- TEL burst ------------------------------------------------------------

use crate::mocks::bus::{FakeCrc, TestProviders};
use crate::tel::{TelChannel, TelFeed};
use osc_protocol::crc::osc_crc;
use osc_servo_core::ControlTableCell;
use osc_servo_core::regions::control::addr::tel::{CHUNK0_ADDR, TEL_COUNT};
use osc_servo_core::regions::telemetry::addr::tick::POS;
use osc_servo_core::tel::{Chunk, Chunks, FLAG_LAST, TelStream as _};

type Feed = TelFeed<TestProviders>;

/// TEL streams at this rate only.
const TEL_RATE: BaudRate = BaudRate::B3000000;

/// A seeded table for a TEL scenario: 'static, as the chip's, since the
/// rows copy from it.
fn tel_shared() -> &'static Shared {
    std::boxed::Box::leak(std::boxed::Box::new(shared_seeded()))
}

/// The kernel half over the harness's wire and ring and `table`, the bus
/// half attached.
fn tel_split(
    h: &Harness,
    bus: &mut crate::bus::ServoBus<TestProviders>,
    table: &'static ControlTableCell,
) -> Feed {
    let ch: &'static TelChannel = std::boxed::Box::leak(std::boxed::Box::new(TelChannel::new()));
    let (feed, drain) =
        ch.split::<TestProviders>(h.wire.clone(), FakeCrc::new(), h.ring.clone(), table);
    bus.attach_tel(drain);
    feed
}

/// One register write over the wire at `anchor`, its ack drained; returns
/// the anchor right behind it.
#[allow(clippy::too_many_arguments)]
fn write_at<D: Dispatch>(
    bus: &mut crate::bus::ServoBus<TestProviders>,
    h: &Harness,
    d: &mut D,
    flags: u8,
    addr: u16,
    data: &[u8],
    anchor: usize,
    now: u32,
) -> usize {
    let mut p = addr.to_le_bytes().to_vec();
    p.extend_from_slice(data);
    let frame = instruction(ID, Opcode::Write, flags, &p);
    deliver(bus, h, anchor, &frame, now, d);
    if flags & Inst::FLAG_NOREPLY == 0 {
        fire(bus, h, d);
        drain_tx(bus, h);
    }
    anchor + frame.len()
}

/// The arm in one WRITE: the chunks, then the count.
fn arm_bytes(c: Chunks, count: u16) -> std::vec::Vec<u8> {
    let mut b = c.to_le_bytes().to_vec();
    b.extend_from_slice(&count.to_le_bytes());
    b
}

const POS_ROW: Chunks = Chunks::one(POS, 2);

fn tel_count(shared: &Shared) -> u16 {
    shared.table.with(|t| t.control.tel.tel_count)
}

/// The kernel's tick: the tick block's `pos`, the row, then the tail; a
/// break on the wire ends before the next tick.
fn tick(feed: &mut Feed, h: &Harness, shared: &Shared, pos: u16) {
    if h.wire.breaking() {
        h.wire.finish();
    }
    shared.table.with_mut(|t| {
        t.telemetry.tick.pos = pos;
        t.telemetry.tick.sample_tick = 0x0100 + (pos as u32 & 0xFF);
    });
    feed.on_tick();
    feed.on_tick_end();
}

/// Tick the burst through, each image read whole once its wire time has
/// passed (bytes x 10 bit-times at 3M, in 50 us ticks); the images the
/// wire carried.
fn stream_all(
    feed: &mut Feed,
    h: &Harness,
    shared: &Shared,
    ticks: u16,
) -> std::vec::Vec<std::vec::Vec<u8>> {
    const BYTES_PER_TICK: usize = 15;
    let from = h.wire.streamed().len();
    let mut left: Option<usize> = None;
    for i in 0..ticks {
        tick(feed, h, shared, 0x2000 + i);
        if left.is_none() {
            left = h.wire.streaming().map(|len| len.div_ceil(BYTES_PER_TICK));
        }
        if let Some(l) = left.as_mut() {
            if *l == 0 {
                h.wire.finish();
                left = None;
            } else {
                *l -= 1;
            }
        }
        if feed.idle() {
            break;
        }
    }
    h.wire.streamed()[from..].to_vec()
}

/// Complete reply frames (Start..Release) recorded at or after `from`.
fn tx_frames(wire: &FakeWire, from: usize) -> std::vec::Vec<std::vec::Vec<u8>> {
    let log = wire.log();
    let mut out = std::vec::Vec::new();
    let mut cur: Option<std::vec::Vec<u8>> = None;
    for e in &log[from..] {
        match e {
            WireEvent::Start => cur = Some(std::vec::Vec::new()),
            WireEvent::Send(v) => {
                if let Some(c) = cur.as_mut() {
                    c.extend_from_slice(v);
                }
            }
            WireEvent::Release => {
                if let Some(c) = cur.take() {
                    out.push(c);
                }
            }
            _ => {}
        }
    }
    out
}

/// The frame's CRC verifies over its covered span (with the 0x00 align
/// prefix the DMA skips).
fn assert_frame_crc(frame: &[u8]) {
    let mut covered = std::vec![0u8];
    covered.extend_from_slice(&frame[..frame.len() - 2]);
    let want = u16::from_le_bytes([frame[frame.len() - 2], frame[frame.len() - 1]]);
    assert_eq!(osc_crc(&covered), want, "stream frame CRC");
}

/// The lane passes at the arming ack's release, not before; the burst's
/// frames come from the kernel's images, and the line stays the burst's
/// until its LAST frame has left the wire.
#[test]
fn tel_lane_passes_at_the_ack_release() {
    let h = Harness::new();
    let mut bus = h.build(ID, TEL_RATE, 60);
    let shared = tel_shared();
    let mut feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    let a = CHUNK0_ADDR.to_le_bytes();
    let mut p = std::vec![a[0], a[1]];
    p.extend(arm_bytes(POS_ROW, 20));
    let arm = instruction(ID, Opcode::Write, 0, &p);
    deliver(&mut bus, &h, 100, &arm, 20_000, &mut d);
    fire(&mut bus, &h, &mut d);
    assert!(
        bus.tx.streaming() && !bus.tx.lent(),
        "the ack holds the wire"
    );
    assert!(!feed.active(), "the lane waits for the ack");
    drain_tx(&mut bus, &h);
    assert!(feed.active() && bus.tx.lent(), "the ack's release lends");
    assert_eq!(last_reply(&h.wire).1.result(), Some(ResultCode::Ok));

    let frames = stream_all(&mut feed, &h, shared, 64);
    assert_eq!(frames.len(), 2, "16 rows, then the 4-row LAST frame");
    let f1 = &frames[0];
    assert_eq!(f1[..3], [ID, wire::len_for(4 + 16 * 2), 0xA4]);
    assert_eq!(f1[3..7], [0x00, 0x00, 0x00, 0x01], "seq 0, row 0's tick");
    assert_eq!(f1[7..9], [0x00, 0x20], "first row's pos");
    assert_frame_crc(f1);
    let f2 = &frames[1];
    assert_eq!(f2[1], wire::len_for(4 + 4 * 2));
    assert_eq!(f2[3..5], [0x01, FLAG_LAST]);
    assert_frame_crc(f2);
    assert!(bus.tx.lent(), "taken back only in the next bus body");
    assert_eq!(tel_count(shared), 20, "armed until the bus takes it back");
}

/// sec 5.6: the servo owns the line from the ack to LAST. A host frame
/// mid-burst is ignored - no reply, the burst runs on - and the first
/// instruction after LAST is served, the ignored one skipped, with
/// `tel_count` already 0 from the lane's return.
#[test]
fn tel_count_reads_zero_after_last() {
    let h = Harness::new();
    let mut bus = h.build(ID, TEL_RATE, 60);
    let shared = tel_shared();
    let mut feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    let at = write_at(
        &mut bus,
        &h,
        &mut d,
        0,
        CHUNK0_ADDR,
        &arm_bytes(POS_ROW, 32),
        100,
        1000,
    );
    assert!(bus.tx.lent());
    for i in 0..4 {
        tick(&mut feed, &h, shared, i);
    }
    let replies = h.wire.log().len();

    let ping = instruction(ID, Opcode::Ping, 0, &[]);
    h.ring.place(at, &ping);
    h.ring.set_cursor(((at + ping.len()) % RING_LEN) as u16);
    h.deadline.set_now(40_000);
    bus.on_break(&mut d);
    assert_eq!(h.deadline.armed(), None, "nothing scheduled for it");
    let frames = stream_all(&mut feed, &h, shared, 64);
    assert_eq!(frames.len(), 2, "the burst runs on");
    assert!(
        !h.wire.log()[replies..].contains(&WireEvent::Start),
        "the mid-burst ping is never answered"
    );
    assert_eq!(tel_count(shared), 32, "LAST is out, the lane not yet back");

    let next = at + ping.len();
    let read = instruction(
        ID,
        Opcode::Read,
        0,
        &[TEL_COUNT as u8, (TEL_COUNT >> 8) as u8, 2, 0],
    );
    deliver_next(&mut bus, &h, next, &read, 90_000, &mut d);
    fire(&mut bus, &h, &mut d);
    drain_tx(&mut bus, &h);
    let starts = h.wire.log()[replies..]
        .iter()
        .filter(|e| **e == WireEvent::Start)
        .count();
    assert_eq!(starts, 1, "one reply, to the read after LAST");
    let (_, inst, payload) = last_reply(&h.wire);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(payload, [0, 0], "tel_count reads 0 once the burst is over");
    assert!(
        !feed.active() && !bus.tx.lent(),
        "the lane is the bus's again"
    );
}

/// No ack to wait for: a NOREPLY arm lends at once.
#[test]
fn tel_arm_with_no_ack_lends_at_the_arm() {
    let h = Harness::new();
    let mut bus = h.build(ID, TEL_RATE, 60);
    let shared = tel_shared();
    let mut feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    write_at(
        &mut bus,
        &h,
        &mut d,
        Inst::FLAG_NOREPLY,
        CHUNK0_ADDR,
        &arm_bytes(POS_ROW, 5),
        100,
        20_000,
    );
    assert!(feed.active() && bus.tx.lent());
    let frames = stream_all(&mut feed, &h, shared, 32);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0][3..5], [0, FLAG_LAST]);
    assert_frame_crc(&frames[0]);
}

/// An ack killed before it streams (the host moved on) takes its burst
/// with it, and `tel_count` reads 0.
#[test]
fn tel_count_reads_zero_after_a_dropped_lend() {
    let h = Harness::new();
    let mut bus = h.build(ID, TEL_RATE, 60);
    let shared = tel_shared();
    let feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    let a = CHUNK0_ADDR.to_le_bytes();
    let mut p = std::vec![a[0], a[1]];
    p.extend(arm_bytes(POS_ROW, 20));
    let arm = instruction(ID, Opcode::Write, 0, &p);
    let end = deliver(&mut bus, &h, 100, &arm, 20_000, &mut d);
    assert!(bus.tx.staged());
    assert_eq!(tel_count(shared), 20);
    let m = 400usize;
    h.deadline.set_now(end + 1);
    h.ring.set_cursor(((m + 1) % RING_LEN) as u16);
    bus.on_break(&mut d);
    if h.deadline.armed().is_some() {
        fire(&mut bus, &h, &mut d);
    }
    assert!(!bus.tx.busy() && !feed.active(), "no lane without its ack");
    assert_eq!(tel_count(shared), 0);
}

/// TEL streams at 3M only: below it an arm acks and never lends, and
/// `tel_count` reads 0 behind the ack.
#[test]
fn tel_count_reads_zero_after_an_arm_below_three_megabaud() {
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = tel_shared();
    let feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    write_at(
        &mut bus,
        &h,
        &mut d,
        0,
        CHUNK0_ADDR,
        &arm_bytes(POS_ROW, 32),
        100,
        1000,
    );
    assert_eq!(last_reply(&h.wire).1.result(), Some(ResultCode::Ok));
    assert!(!feed.active(), "below 3M an arm never lends");
    assert_eq!(tel_count(shared), 0);
}

/// Chunks that are no row (over 12 B) arm as a disarm: the write acks,
/// nothing lends, `tel_count` reads 0; a zero count never lends either.
#[test]
fn tel_count_reads_zero_after_an_arm_that_is_no_row() {
    let h = Harness::new();
    let mut bus = h.build(ID, TEL_RATE, 60);
    let shared = tel_shared();
    let feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    let wide = Chunks::two(Chunk::new(POS, 8), Chunk::new(POS, 8));
    let at = write_at(
        &mut bus,
        &h,
        &mut d,
        0,
        CHUNK0_ADDR,
        &arm_bytes(wide, 32),
        100,
        1000,
    );
    assert_eq!(last_reply(&h.wire).1.result(), Some(ResultCode::Ok));
    assert!(!feed.active(), "a row over 12 B never lends");
    assert_eq!(tel_count(shared), 0);

    let at = write_at(
        &mut bus,
        &h,
        &mut d,
        0,
        CHUNK0_ADDR,
        &arm_bytes(POS_ROW, 0),
        at,
        40_000,
    );
    assert!(!feed.active(), "count 0 never lends");
    write_at(
        &mut bus,
        &h,
        &mut d,
        0,
        TEL_COUNT,
        &32u16.to_le_bytes(),
        at,
        80_000,
    );
    assert!(feed.active());
}

/// The chunks carry no table rules, so the 10 B arm block validates as
/// a bare count: an odd chunk acks Ok, never lends, and `tel_count`
/// reads 0 (protocol sec 5.6).
#[test]
fn tel_count_reads_zero_after_an_arm_with_an_odd_chunk() {
    let h = Harness::new();
    let mut bus = h.build(ID, TEL_RATE, 60);
    let shared = tel_shared();
    let feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    let odd = Chunks::one(POS + 1, 2);
    write_at(
        &mut bus,
        &h,
        &mut d,
        0,
        CHUNK0_ADDR,
        &arm_bytes(odd, 32),
        100,
        1000,
    );
    assert_eq!(last_reply(&h.wire).1.result(), Some(ResultCode::Ok));
    assert!(!feed.active(), "an odd chunk never lends");
    assert_eq!(tel_count(shared), 0);
}

/// The count is capped at one second of ticks: a longer write is a rule
/// reject and lends nothing.
#[test]
fn tel_count_over_the_cap_rejects() {
    use osc_servo_core::tel::BURST_ROWS_MAX;
    let h = Harness::new();
    let mut bus = h.build(ID, RATE, 60);
    let shared = tel_shared();
    let feed = tel_split(&h, &mut bus, &shared.table);
    let mut session = Session::new();
    let mut d = session.dispatcher(shared);

    let arm = arm_bytes(POS_ROW, BURST_ROWS_MAX + 1);
    write_at(&mut bus, &h, &mut d, 0, CHUNK0_ADDR, &arm, 100, 1000);
    assert_eq!(last_reply(&h.wire).1.result(), Some(ResultCode::Validation));
    assert!(!feed.active());
}
