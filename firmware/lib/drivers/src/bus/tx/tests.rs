//! TX engine spec tests: staged layouts driven to completion against fake
//! wire/CRC, asserting the streamed bytes match a software-sealed reference.

use super::*;
use crate::bus::snapshot::{self, Beat, Buf};
use crate::mocks::bus::aligned_copy;
use osc_protocol::crc::{osc_crc, osc_crc_continue};
use osc_protocol::wire::Id;
use std::boxed::Box;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec;
use std::vec::Vec;

struct FakeCrc {
    state: u16,
    /// Fixed like the chip's static buffer: snapshot addresses stay put.
    /// Starts dirty, so a byte the frame needs and nobody writes shows.
    snap: Box<Buf>,
    copies: usize,
    /// Every channel arm of the copies, in order.
    arms: Vec<(Beat, u16)>,
}

impl FakeCrc {
    fn new() -> Self {
        FakeCrc {
            state: 0,
            snap: Box::new(Buf([0xA5; snapshot::LEN])),
            copies: 0,
            arms: Vec::new(),
        }
    }
}

impl CrcEngine for FakeCrc {
    fn reset(&mut self) {
        self.state = 0;
    }
    fn feed(&mut self, span: &[u8]) {
        assert_eq!(span.len() % 2, 0, "CRC feed must be even-length (F12)");
        assert_eq!(
            span.as_ptr().addr() % 2,
            0,
            "CRC feed must be even-addressed (F12)"
        );
        self.state = osc_crc_continue(self.state, span);
    }
    unsafe fn snapshot(&mut self, spans: &[&[u8]]) -> *mut u8 {
        self.copies += 1;
        let arms = &mut self.arms;
        // SAFETY: the caller's total.
        unsafe {
            snapshot::gather(&mut *self.snap, spans, |s, d, beat, beats| {
                arms.push((beat, beats));
                aligned_copy(s, d, beat, beats);
            })
        }
    }
    unsafe fn copy(&mut self, _dst: *mut u8, _src: *const u8, _units: u16) {}
    fn result(&mut self) -> Option<u16> {
        Some(self.state)
    }
}

#[derive(Debug, PartialEq)]
enum Event {
    Start,
    Send(Vec<u8>),
    Release,
}

#[derive(Clone, Default)]
struct FakeWire(Rc<RefCell<Vec<Event>>>);

impl TxWire for FakeWire {
    fn start_frame(&mut self) {
        self.0.borrow_mut().push(Event::Start);
    }
    fn send(&mut self, span: &[u8]) {
        self.0.borrow_mut().push(Event::Send(span.to_vec()));
    }
    fn release(&mut self) {
        self.0.borrow_mut().push(Event::Release);
    }
    fn idle(&self) -> bool {
        true
    }
    fn break_start(&mut self) {}
    fn stream(&mut self, _image: &[u8]) {}
}

fn engine() -> (TxEngine<FakeWire>, Rc<RefCell<Vec<Event>>>) {
    let wire = FakeWire::default();
    let log = wire.0.clone();
    (TxEngine::new(wire), log)
}

/// Software-sealed frame for the same reply, including the 0x00 prefix.
/// Built in its own full-size buffer - REPLY_BUF only fits the copy
/// path, and the reference is a whole linearized frame.
fn reference(id: u8, result: ResultCode, alert: bool, data: &[u8]) -> Vec<u8> {
    let mut b = FrameBuf::<{ wire::MAX_PAYLOAD as usize + 8 }>::new();
    b.start(Id::new(id), Inst::status(result, alert));
    b.payload_mut()[..data.len()].copy_from_slice(data);
    b.finish(data.len() as u8);
    b.seal().to_vec()
}

/// Drive a staged frame to completion, returning the arm outcomes. The one
/// CRC engine spans trigger and every arm boundary, as on the chip.
/// Drive a staged frame to completion with the SAME engine handle used at
/// stage (the snapshot lives in it, as on the chip).
fn run(eng: &mut TxEngine<FakeWire>, crc: &mut FakeCrc, over: Option<ResultCode>) -> Vec<TxOut> {
    eng.trigger(crc, over);
    let mut outs = Vec::new();
    loop {
        let out = eng.on_arm_complete(crc);
        outs.push(out);
        if out == TxOut::Released {
            return outs;
        }
    }
}

fn sends(log: &[Event]) -> Vec<Vec<u8>> {
    log.iter()
        .filter_map(|e| match e {
            Event::Send(v) => Some(v.clone()),
            _ => None,
        })
        .collect()
}

fn wire_bytes(log: &[Event]) -> Vec<u8> {
    sends(log).concat()
}

#[repr(align(4))]
struct Aligned<const N: usize>([u8; N]);

#[test]
fn empty_ack_matches_sealed_reference() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    eng.stage(&mut crc, 7, ResultCode::Ok, false, &[]).unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let reference = reference(7, ResultCode::Ok, false, &[]);
    assert_eq!(log[0], Event::Start);
    let s = sends(&log);
    // Header arm, then the CRC as its own final 2-byte arm, patched to
    // the same value the single-shot software seal produces.
    assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![3, 2]);
    assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
    assert_eq!(wire_bytes(&log), reference[1..]);
    assert_eq!(*log.last().unwrap(), Event::Release);
    // Empty payload: status bit set, no flags.
    assert_eq!(wire_bytes(&log)[2], 0x80);
}

#[test]
fn small_copy_reply_matches_sealed_reference() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    // 2 bytes <= SMALL_COPY_MAX -> the copy path, even payload.
    let data = [0xDE, 0xAD];
    eng.stage(&mut crc, 1, ResultCode::Ok, true, &data).unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let reference = reference(1, ResultCode::Ok, true, &data);
    let s = sends(&log);
    // Header + payload, then the CRC tail as its own final arm.
    assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![5, 2]);
    assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
    assert_eq!(wire_bytes(&log), reference[1..]);
    let inst = Inst(wire_bytes(&log)[2]);
    assert!(inst.is_status());
    assert!(inst.alert());
}

#[test]
fn small_odd_zero_copy_streams_whole_payload() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    // 3 even-addressed bytes: above SMALL_COPY_MAX -> snapshot; the odd
    // last byte is folded into the CRC in software (sec 3.2), not moved into
    // a tail arm.
    let data = Aligned([0xDE, 0xAD, 0xBE, 0]);
    eng.stage(&mut crc, 1, ResultCode::Ok, true, &data.0[..3])
        .unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let reference = reference(1, ResultCode::Ok, true, &data.0[..3]);
    let s = sends(&log);
    // Header + whole payload, CRC.
    assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![6, 2]);
    assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
    assert_eq!(wire_bytes(&log), reference[1..]);
    let inst = Inst(wire_bytes(&log)[2]);
    assert!(inst.is_status());
    assert!(inst.alert());
}

#[test]
fn odd_pointer_streams_zero_copy() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    // Odd-addressed source above SMALL_COPY_MAX: snapshotted like any
    // other span - the chip CRC provider stages odd pointers through its
    // copy channel (sec 5); the engine is parity-blind.
    let backing = Aligned([0u8, 0xDE, 0xAD, 0xBE]);
    let data = &backing.0[1..4];
    eng.stage(&mut crc, 1, ResultCode::Ok, true, data).unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let reference = reference(1, ResultCode::Ok, true, data);
    let s = sends(&log);
    // Header + payload, CRC.
    assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![6, 2]);
    assert_eq!(wire_bytes(&log), reference[1..]);
}

#[test]
fn snapshot_reply_streams_in_two_arms() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
    eng.stage(&mut crc, 9, ResultCode::Ok, false, &data.0)
        .unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let s = sends(&log);
    // ID..payload in one arm from the snapshot, then the CRC arm.
    assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![43, 2]);
    let reference = reference(9, ResultCode::Ok, false, &data.0);
    assert_eq!(crc.snap.0[..44], reference[..44]);
    assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
    assert_eq!(wire_bytes(&log), reference[1..]);
    // CRC over the covered span matches the reference flavor (the leading
    // alignment byte is a no-op, sec 3.2).
    let covered_len = wire::covered_len(wire::len_for(40));
    let crc = u16::from_le_bytes([s[1][0], s[1][1]]);
    assert_eq!(osc_crc(&reference[..covered_len]), crc);
}

#[test]
fn odd_snapshot_reply_folds_its_last_byte() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    let data = Aligned(core::array::from_fn::<u8, 41, _>(|i| !(i as u8)));
    eng.stage(&mut crc, 5, ResultCode::Ok, false, &data.0)
        .unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let s = sends(&log);
    // Odd payload streams whole; its last byte reaches the CRC via the
    // software fold at patch time (sec 3.2), so the wire CRC still matches
    // the byte-wise reference.
    assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![44, 2]);
    let reference = reference(5, ResultCode::Ok, false, &data.0);
    assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
    assert_eq!(wire_bytes(&log), reference[1..]);
}

#[test]
fn gather_matches_sealed_reference_of_concat() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    // Three spans, odd interior lengths (no parity constraint, sec 5.2): the
    // wire must carry exactly the concatenation.
    let a = Aligned([0x10, 0x11, 0x12, 0]);
    let b = Aligned([0x20, 0]);
    let c = Aligned([0x30, 0x31, 0x32, 0x33]);
    let spans: [&[u8]; 3] = [&a.0[..3], &b.0[..1], &c.0];
    let concat: Vec<u8> = spans.concat();
    eng.stage_gather(&mut crc, 3, ResultCode::Ok, false, &spans)
        .unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let reference = reference(3, ResultCode::Ok, false, &concat);
    let s = sends(&log);
    // One contiguous snapshot arm, CRC - same shape as a plain read (the
    // gather is invisible to the wire).
    assert_eq!(
        s.iter().map(Vec::len).collect::<Vec<_>>(),
        vec![3 + concat.len(), 2]
    );
    assert_eq!(wire_bytes(&log), reference[1..]);
    assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
}

#[test]
fn gather_tiny_total_takes_copy_path() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    // Two 1-byte spans: total 2 <= SMALL_COPY_MAX -- buffer copy, no
    // snapshot involved.
    let spans: [&[u8]; 2] = [&[0xAA], &[0xBB]];
    eng.stage_gather(&mut crc, 4, ResultCode::Ok, false, &spans)
        .unwrap();
    run(&mut eng, &mut crc, None);
    let log = log.borrow();
    let reference = reference(4, ResultCode::Ok, false, &[0xAA, 0xBB]);
    assert_eq!(wire_bytes(&log), reference[1..]);
    assert_eq!(crc.copies, 0);
}

#[test]
fn copy_path_stages_the_source_bytes() {
    let src = Aligned([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
    // The spans, and the snapshot copies they take: 0 is the staging buffer.
    let cases: [(&[&[u8]], usize); 8] = [
        // Even pair: the one halfword load.
        (&[&src.0[..2]], 0),
        // Odd pair, gathered bytes, one byte, empty: byte loads.
        (&[&src.0[1..3]], 0),
        (&[&src.0[..1], &src.0[3..4]], 0),
        (&[&src.0[2..3]], 0),
        (&[&[]], 0),
        // A 4-aligned word: the one word load.
        (&[&src.0[4..8]], 0),
        // A word at 2 mod 4, or gathered: the snapshot.
        (&[&src.0[2..6]], 1),
        (&[&src.0[..2], &src.0[4..6]], 1),
    ];
    for (spans, copies) in cases {
        let (mut eng, log) = engine();
        let mut crc = FakeCrc::new();
        eng.stage_gather(&mut crc, 2, ResultCode::Ok, false, spans)
            .unwrap();
        run(&mut eng, &mut crc, None);
        let log = log.borrow();
        let concat = spans.concat();
        assert_eq!(sends(&log)[0][3..], concat, "{spans:02x?}");
        assert_eq!(
            wire_bytes(&log),
            reference(2, ResultCode::Ok, false, &concat)[1..],
            "{spans:02x?}"
        );
        assert_eq!(crc.copies, copies, "{spans:02x?}");
    }
}

/// A snapshot reply at each of the four frame shifts, odd and even
/// lengths, with and without the alert flag and the INST override: ID, LEN,
/// INST, payload and CRC match the sealed reference byte for byte.
#[test]
fn every_shift_streams_the_reference_frame() {
    let src = Aligned(core::array::from_fn::<u8, 264, _>(|i| {
        (i as u8).wrapping_mul(7) ^ 0x3C
    }));
    let overs = [None, Some(ResultCode::PredecessorSilent)];
    for a in 0..4 {
        for len in (3..=13).chain([40, 41, 251, 252]) {
            for alert in [false, true] {
                for over in overs {
                    let data = &src.0[a..a + len];
                    let (mut eng, log) = engine();
                    let mut crc = FakeCrc::new();
                    eng.stage(&mut crc, 0x2B, ResultCode::Ok, alert, data)
                        .unwrap();
                    run(&mut eng, &mut crc, over);
                    let log = log.borrow();
                    let case = (a, len, alert, over);
                    let reference = reference(0x2B, over.unwrap_or(ResultCode::Ok), alert, data);
                    assert_eq!(wire_bytes(&log), reference[1..], "{case:?}");
                    if (a, len) == (0, 4) {
                        assert_eq!(crc.copies, 0, "{case:?}");
                        continue;
                    }
                    assert_eq!(crc.copies, 1, "{case:?}");
                    // The frame sits at the source's alignment, a zero
                    // under an odd base for the feed.
                    assert_eq!(crc.snap.0[a..a + 4 + len], reference[..4 + len], "{case:?}");
                    if a % 2 == 1 {
                        assert_eq!(crc.snap.0[a - 1], 0, "{case:?}");
                    }
                    assert!(
                        crc.arms.iter().all(|&(b, _)| b == Beat::Word),
                        "{case:?}: {:?}",
                        crc.arms
                    );
                }
            }
        }
    }
}

/// Each row of the gathered plan (`snapshot::plan`) in a profile read: the
/// streamed frame matches the sealed reference of the concatenation, and
/// each span took the arm its row names.
#[test]
fn every_gather_row_streams_the_reference_frame() {
    let src = Aligned(core::array::from_fn::<u8, 128, _>(|i| !(i as u8) ^ 0x11));
    let s = &src.0;
    type Row<'a> = (&'a [&'a [u8]], &'a [(Beat, u16)]);
    let cases: [Row; 7] = [
        // A later span at its own alignment: word beats, CPU ends.
        (
            &[&s[0..12], &s[64..76]],
            &[(Beat::Word, 3), (Beat::Word, 3)],
        ),
        (&[&s[1..4], &s[64..75]], &[(Beat::Word, 2)]),
        // Up to CPU_SPAN_MAX, at any alignment: the CPU alone.
        (&[&s[0..3], &s[64..68], &s[70..72], &s[73..74]], &[]),
        (&[&s[0..1], &s[64..71], &s[66..72], &s[4..8]], &[]),
        // Mismatched, longer, same parity: halfword beats, CPU bytes.
        (&[&s[0..2], &s[64..73]], &[(Beat::Half, 4)]),
        (&[&s[1..3], &s[65..74]], &[(Beat::Half, 4)]),
        // Opposite parity: byte beats.
        (&[&s[0..1], &s[64..73]], &[(Beat::Byte, 9)]),
    ];
    for (spans, arms) in cases {
        let (mut eng, log) = engine();
        let mut crc = FakeCrc::new();
        eng.stage_gather(&mut crc, 6, ResultCode::Ok, true, spans)
            .unwrap();
        run(&mut eng, &mut crc, None);
        let log = log.borrow();
        let concat = spans.concat();
        assert_eq!(
            wire_bytes(&log),
            reference(6, ResultCode::Ok, true, &concat)[1..],
            "{spans:02x?}"
        );
        assert_eq!(crc.arms, arms, "{spans:02x?}");
    }
}

/// CH6 is the TEL lane's while the engine is lent (sec 5.6), and the
/// snapshot arms it with whole-register writes and no critical section:
/// a lent engine must refuse a read before touching the copy channel.
#[test]
fn a_lent_engine_stages_no_snapshot() {
    let (mut eng, _log) = engine();
    let mut crc = FakeCrc::new();
    eng.lend();
    assert!(eng.lent());
    let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
    assert_eq!(
        eng.stage(&mut crc, 1, ResultCode::Ok, false, &data.0),
        Err(SendError::Busy)
    );
    assert_eq!((crc.copies, crc.arms.len()), (0, 0));
}

#[test]
fn gather_overflow_rejected() {
    let (mut eng, _log) = engine();
    let mut crc = FakeCrc::new();
    let big = [0u8; 130];
    let spans: [&[u8]; 2] = [&big, &big];
    assert_eq!(
        eng.stage_gather(&mut crc, 1, ResultCode::Ok, false, &spans),
        Err(SendError::Overflow)
    );
    assert!(!eng.busy());
}

#[test]
fn override_rewrites_inst_and_crc_still_validates() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
    eng.stage(&mut crc, 9, ResultCode::Ok, false, &data.0)
        .unwrap();
    run(&mut eng, &mut crc, Some(ResultCode::PredecessorSilent));
    let log = log.borrow();
    let wire_out = wire_bytes(&log);
    let inst = Inst(wire_out[2]);
    assert_eq!(inst.result(), Some(ResultCode::PredecessorSilent));
    let covered_len = wire::covered_len(wire::len_for(40));
    let crc = u16::from_le_bytes([wire_out[covered_len - 1], wire_out[covered_len]]);
    assert_eq!(osc_crc(&wire_out[..covered_len - 1]), crc);
}

#[test]
fn double_stage_is_busy_and_overflow_rejected() {
    let (mut eng, _log) = engine();
    let mut crc = FakeCrc::new();
    eng.stage(&mut crc, 1, ResultCode::Ok, false, &[]).unwrap();
    assert!(eng.busy());
    assert_eq!(
        eng.stage(&mut crc, 2, ResultCode::Ok, false, &[]),
        Err(SendError::Busy)
    );
    let big = [0u8; 253];
    let (mut eng2, _log2) = engine();
    assert_eq!(
        eng2.stage(&mut crc, 1, ResultCode::Ok, false, &big),
        Err(SendError::Overflow)
    );
    assert!(!eng2.busy());
}

#[test]
fn abort_releases_and_unblocks_stage() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    eng.stage(&mut crc, 1, ResultCode::Ok, false, &[]).unwrap();
    let _ = eng.abort();
    assert_eq!(*log.borrow(), vec![Event::Release]);
    assert!(!eng.busy());
    eng.stage(&mut crc, 2, ResultCode::Ok, false, &[]).unwrap();
    // Aborting mid-stream also releases.
    eng.trigger(&mut crc, None);
    let _ = eng.abort();
    assert_eq!(*log.borrow().last().unwrap(), Event::Release);
    assert!(!eng.busy());
}

#[test]
fn arm_sequencing_ends_in_released_exactly_once() {
    let (mut eng, _log) = engine();
    let mut crc = FakeCrc::new();
    let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
    eng.stage(&mut crc, 9, ResultCode::Ok, false, &data.0)
        .unwrap();
    let outs = run(&mut eng, &mut crc, None);
    assert_eq!(outs, vec![TxOut::Armed, TxOut::Released]);
    assert!(!eng.busy());
}

#[test]
fn inst_override_lands_in_the_snapshot() {
    let (mut eng, log) = engine();
    let mut crc = FakeCrc::new();
    let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
    eng.stage(&mut crc, 9, ResultCode::Ok, false, &data.0)
        .unwrap();
    run(&mut eng, &mut crc, Some(ResultCode::PredecessorSilent));
    let inst = Inst::status(ResultCode::PredecessorSilent, false).0;
    assert_eq!(crc.snap.0[3], inst);
    let log = log.borrow();
    let reference = reference(9, ResultCode::PredecessorSilent, false, &data.0);
    assert_eq!(wire_bytes(&log), reference[1..]);
}
