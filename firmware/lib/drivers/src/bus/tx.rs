//! Status-reply TX engine (`docs/osc-native-protocol.md` §4.2): stage a frame
//! layout, then trigger — break + up to four DMA arms, the last always the
//! 2-byte CRC. The wire starts before the CRC is known; the hardware engine
//! chews the covered span in parallel and the value is patched into that final
//! arm at the boundary before it (the engine outruns the wire 8:1, F6, and DMA
//! fetches just-in-time, so the patch beats the read).

use crate::traits::bus::{CrcEngine, TxWire};
use osc_core::traits::SendError;
use osc_protocol::crc::osc_crc_continue;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Id, Inst, ResultCode};

/// Staging buffer size. Payloads stream from the engine's snapshot (§4.2), so
/// the buffer holds only the header and CRC tail plus the ≤
/// [`SMALL_COPY_MAX`] copy path.
pub const REPLY_BUF: usize = 16;

/// Payloads at or below this are copied into the staging buffer. Kept minimal:
/// the copy costs ~0.3 µs/byte of turnaround (bench-measured at 3M), so
/// anything the DMA can stream in place should stream. The floor exists
/// because an odd payload donates its last byte to the tail arm — at p = 1
/// that would leave a zero-length DMA arm.
const SMALL_COPY_MAX: usize = 2;

/// Outcome of an arm-completion event.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TxOut {
    Armed,
    Released,
}

/// One DMA arm (or CRC feed) as a descriptor — resolved to a slice only at
/// send time, so the engine never holds self-referential borrows.
#[derive(Copy, Clone)]
enum Arm {
    // len is u16: the odd-pointer copy path can arm up to footprint-1 = 257
    // bytes (a 251-byte payload from an odd table address). off is u16 too:
    // the CRC tail of that same maximal frame sits at offset 256.
    Buf { off: u16, len: u16 },
    Ext { ptr: *const u8, len: u16 },
}

const NO_ARM: Arm = Arm::Buf { off: 0, len: 0 };

enum State {
    Idle,
    /// `slot_key` is present on collision-tolerant replies: broadcast-ENUM
    /// replies' §9.2 job is to collide, so the composite's break-wake kill
    /// skips them, and the key (folded UID CRC) draws the reply's slot
    /// delay — cycle-identical twin matchers otherwise answer in unison,
    /// and a sub-bit-aligned superposition of near-equal frames reads back
    /// as one clean frame, hiding the loser's subtree from the walk.
    Staged {
        slot_key: Option<u8>,
    },
    Streaming {
        next: u8,
    },
}

pub struct TxEngine<W: TxWire> {
    wire: W,
    buf: FrameBuf<REPLY_BUF>,
    // The CRC tail is always the final arm (index `n_feeds`); the `n_feeds`
    // arms before it map 1:1 to feed spans, fed one per arm boundary.
    arms: [Arm; 4],
    n_arms: u8,
    feeds: [Arm; 3],
    n_feeds: u8,
    /// Buffer offset of the CRC tail slot (zeroed at stage as the placeholder).
    crc_off: u16,
    state: State,
    result: ResultCode,
    /// The covered byte the even-bulk feeds leave un-fed when the span is odd
    /// (§3.2): read and folded into the engine result at patch time (the
    /// pointer targets engine-stable storage — buffer or snapshot).
    tail: Option<*const u8>,
    alert: bool,
    crc_misses: u32,
}

impl<W: TxWire> TxEngine<W> {
    pub fn new(wire: W) -> Self {
        Self {
            wire,
            buf: FrameBuf::new(),
            arms: [NO_ARM; 4],
            n_arms: 0,
            feeds: [NO_ARM; 3],
            n_feeds: 0,
            crc_off: 0,
            state: State::Idle,
            result: ResultCode::Ok,
            tail: None,
            alert: false,
            crc_misses: 0,
        }
    }

    pub fn busy(&self) -> bool {
        !matches!(self.state, State::Idle)
    }

    /// A frame is staged but not yet triggered — safe to abort (a fresh
    /// instruction supersedes it). Streaming frames must not be aborted.
    pub fn staged(&self) -> bool {
        matches!(self.state, State::Staged { .. })
    }

    /// The staged reply is exempt from the break-wake kill (§9.2 ENUM).
    pub fn collision_tolerant(&self) -> bool {
        self.slot_key().is_some()
    }

    /// A collision-tolerant staged reply's slot-delay key (§9.2), else None.
    pub fn slot_key(&self) -> Option<u8> {
        match self.state {
            State::Staged { slot_key } => slot_key,
            _ => None,
        }
    }

    /// Mark the staged reply collision-tolerant with its slot-delay key
    /// (§9.2: an ENUM reply's job is to collide with peer matchers, offset
    /// by its slot). No-op unless a frame is staged.
    pub fn mark_collision_tolerant(&mut self, key: u8) {
        if let State::Staged { slot_key } = &mut self.state {
            *slot_key = Some(key);
        }
    }

    /// Arms are on the wire — the servo owns the line until the final TC.
    pub fn streaming(&self) -> bool {
        matches!(self.state, State::Streaming { .. })
    }

    /// Build the frame layout for a status reply; touches no wire state
    /// (enable-when-ready is [`trigger`](Self::trigger), §4.2).
    pub fn stage<C: CrcEngine>(
        &mut self,
        crc: &mut C,
        id: u8,
        result: ResultCode,
        alert: bool,
        data: &[u8],
    ) -> Result<(), SendError> {
        self.stage_gather(crc, id, result, alert, &[data])
    }

    /// Gathered form of [`stage`](Self::stage): the payload is `spans`
    /// concatenated in order (§5.2 profile reads; a plain reply is the
    /// one-span case). Payload totals above [`SMALL_COPY_MAX`] are
    /// snapshotted through the CRC engine's stable buffer at cumulative
    /// offsets — wire and CRC both stream the one contiguous snapshot, so a
    /// scattered read costs the same single copy as a plain read (§4.2).
    pub fn stage_gather<C: CrcEngine>(
        &mut self,
        crc: &mut C,
        id: u8,
        result: ResultCode,
        alert: bool,
        spans: &[&[u8]],
    ) -> Result<(), SendError> {
        if self.busy() {
            return Err(SendError::Busy);
        }
        let total: usize = spans.iter().map(|s| s.len()).sum();
        if total > wire::MAX_PAYLOAD as usize {
            return Err(SendError::Overflow);
        }
        let p = total as u8;
        let inst = Inst::status(result, alert);
        // Small payloads are cheaper to copy than to arm. An odd covered span
        // feeds its even bulk and leaves the last byte for the software fold
        // at patch (§3.2); odd POINTERS are the CRC provider's concern (it
        // stages them through its copy channel, §5).
        if total <= SMALL_COPY_MAX {
            self.buf.start(Id::new(id), inst);
            let pay = self.buf.payload_mut();
            let mut at = 0;
            for s in spans {
                pay[at..at + s.len()].copy_from_slice(s);
                at += s.len();
            }
            self.buf.finish(p);
            let len = wire::len_for(p);
            let cov = wire::covered_len(len);
            let bulk = cov & !1;
            // Header + payload, then the 2 CRC bytes as their own arm.
            self.arms[0] = Arm::Buf {
                off: 1,
                len: (cov - 1) as u16,
            };
            self.arms[1] = Arm::Buf {
                off: cov as u16,
                len: 2,
            };
            self.n_arms = 2;
            self.feeds[0] = Arm::Buf {
                off: 0,
                len: bulk as u16,
            };
            self.n_feeds = 1;
            self.tail = if cov & 1 == 1 {
                Some(&raw const self.buf.bytes()[cov - 1])
            } else {
                None
            };
            self.crc_off = cov as u16;
        } else {
            // Snapshot reads (§4.2): the payload is copied ONCE into the
            // engine's stable snapshot buffer — each span at its cumulative
            // offset — and both the wire arms and the CRC feeds stream the
            // snapshot: the CRC provably covers the transmitted bytes, and
            // the reply carries an atomic point-in-time image (`stage` runs
            // kernel-exclusive; the provider orders the copies ahead of both
            // consumers).
            let b = self.buf.bytes_mut();
            b[0] = wire::ALIGN_BYTE;
            b[1] = id;
            b[2] = wire::len_for(p);
            b[3] = inst.0;
            let mut ptr: *const u8 = core::ptr::null();
            let mut off: u16 = 0;
            for s in spans {
                if s.is_empty() {
                    continue;
                }
                let dst = crc.snapshot(off, s);
                if off == 0 {
                    ptr = dst;
                }
                off += s.len() as u16;
            }
            self.arms[0] = Arm::Buf { off: 1, len: 3 };
            self.arms[1] = Arm::Ext { ptr, len: p as u16 };
            // The buffer's bytes 4..6 double as the CRC tail arm.
            self.arms[2] = Arm::Buf { off: 4, len: 2 };
            self.n_arms = 3;
            self.feeds[0] = Arm::Buf { off: 0, len: 4 };
            self.feeds[1] = Arm::Ext {
                ptr,
                len: (p & !1) as u16,
            };
            self.n_feeds = 2;
            // The fold byte is read at patch time, not here: the snapshot is
            // best-effort asynchronous and may still be streaming — by the
            // patch boundary the copy has long completed (§4.2).
            self.tail = if p & 1 == 1 {
                Some(unsafe { ptr.add(p as usize - 1) })
            } else {
                None
            };
            self.crc_off = 4;
        }
        // Placeholder CRC: what ships if the patch window is missed.
        let off = self.crc_off as usize;
        self.buf.bytes_mut()[off..off + 2].copy_from_slice(&[0, 0]);
        self.result = result;
        self.alert = alert;
        self.state = State::Staged { slot_key: None };
        Ok(())
    }

    /// Finalize and start: optional result override (chain reclaim's
    /// predecessor-silent, §6) rewrites INST, then break + first arm with the
    /// first CRC feed armed behind it. The CRC value lands later, at the
    /// boundary before its own trailing arm ([`on_arm_complete`]) — the wire
    /// starts before the CRC is known so the engine chews in parallel (§4.2).
    pub fn trigger<C: CrcEngine>(&mut self, crc: &mut C, over: Option<ResultCode>) {
        if !matches!(self.state, State::Staged { .. }) {
            debug_assert!(false, "trigger without a staged frame");
            return;
        }
        self.buf.bytes_mut()[3] = Inst::status(over.unwrap_or(self.result), self.alert).0;
        crc.reset();
        self.wire.start_frame();
        self.wire.send(resolve(&self.buf, self.arms[0]));
        crc.feed(resolve(&self.buf, self.feeds[0]));
        self.state = State::Streaming { next: 1 };
    }

    /// Per-arm DMA TC. Feeds the next CRC span and streams the next arm; before
    /// the final CRC arm, patches the computed CRC into the buffer; after the
    /// last arm, releases the wire (caller then applies deferred config).
    pub fn on_arm_complete<C: CrcEngine>(&mut self, crc: &mut C) -> TxOut {
        match self.state {
            State::Streaming { next } if next < self.n_arms => {
                let i = next as usize;
                if i < self.n_feeds as usize {
                    // Arm the next feed span. A full arm's wire-time has elapsed
                    // since the previous feed, so the chip drain-spin is a no-op.
                    crc.feed(resolve(&self.buf, self.feeds[i]));
                } else {
                    // Next arm is the CRC tail: land the value before the DMA
                    // reaches it.
                    self.patch_crc(crc);
                }
                self.wire.send(resolve(&self.buf, self.arms[i]));
                self.state = State::Streaming { next: next + 1 };
                TxOut::Armed
            }
            State::Streaming { .. } => {
                self.wire.release();
                self.state = State::Idle;
                TxOut::Released
            }
            // Spurious TC: the wire is already released, don't touch it.
            _ => {
                debug_assert!(false, "arm completion while idle");
                TxOut::Released
            }
        }
    }

    /// Poll the CRC and patch the trailing 2 bytes. Called at the boundary
    /// before the CRC arm: the DMA is physically reading the buffer as we
    /// write here — by design. The CRC arm is last (>= header's worth of head
    /// start) and the engine runs ~8x wire speed (F6), so the patch wins.
    fn patch_crc<C: CrcEngine>(&mut self, crc: &mut C) {
        // Every feed was armed at least one arm's wire-time ago, so `result()`
        // is Some on the first poll in any healthy exchange; the fixed bound
        // only guards a sick engine (placeholder zeros ship, host retries).
        let mut budget = super::SPIN_PER_BYTE;
        let value = loop {
            if let Some(v) = crc.result() {
                break Some(v);
            }
            if budget == 0 {
                break None;
            }
            budget -= 1;
            core::hint::spin_loop();
        };
        match value {
            Some(v) => {
                // Fold the un-fed trailing covered byte, if the span was odd
                // (§3.2) — the only software CRC on the servo. SAFETY: the
                // pointer targets the frame buffer or the engine's snapshot,
                // both stable for this exchange; any snapshot copy completed
                // arms ago (transfer ordering).
                let v = match self.tail {
                    Some(b) => osc_crc_continue(v, &[unsafe { *b }]),
                    None => v,
                };
                let off = self.crc_off as usize;
                self.buf.bytes_mut()[off..off + 2].copy_from_slice(&v.to_le_bytes());
            }
            None => self.crc_misses = self.crc_misses.wrapping_add(1),
        }
    }

    /// Drop anything staged or streaming and release the wire.
    pub fn abort(&mut self) {
        if self.busy() {
            self.wire.release();
            self.state = State::Idle;
        }
    }

    /// CRC-engine patch-window misses (placeholder CRC shipped), monotonic.
    pub fn crc_misses(&self) -> u32 {
        self.crc_misses
    }
}

fn resolve(buf: &FrameBuf<REPLY_BUF>, arm: Arm) -> &[u8] {
    match arm {
        Arm::Buf { off, len } => &buf.bytes()[off as usize..off as usize + len as usize],
        // SAFETY: `Ext` descriptors exist only between `stage` and the final
        // arm completion (or `abort`), and `stage`'s contract requires the
        // referent to outlive the transmission (static control-table storage).
        Arm::Ext { ptr, len } => unsafe { core::slice::from_raw_parts(ptr, len as usize) },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::crc::{osc_crc, osc_crc_continue};
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::vec;
    use std::vec::Vec;

    struct FakeCrc {
        state: u16,
        snap: Vec<u8>,
    }

    impl FakeCrc {
        fn new() -> Self {
            FakeCrc {
                state: 0,
                snap: Vec::new(),
            }
        }
    }

    impl CrcEngine for FakeCrc {
        fn reset(&mut self) {
            self.state = 0;
        }
        fn feed(&mut self, span: &[u8]) {
            assert_eq!(span.len() % 2, 0, "CRC feed must be even-length (F12)");
            // Odd ADDRESSES are legal: the chip provider stages them (§5).
            self.state = osc_crc_continue(self.state, span);
        }
        fn snapshot(&mut self, off: u16, src: &[u8]) -> *const u8 {
            let off = off as usize;
            if self.snap.len() < off + src.len() {
                self.snap.resize(off + src.len(), 0);
            }
            self.snap[off..off + src.len()].copy_from_slice(src);
            unsafe { self.snap.as_ptr().add(off) }
        }
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
    }

    fn engine() -> (TxEngine<FakeWire>, Rc<RefCell<Vec<Event>>>) {
        let wire = FakeWire::default();
        let log = wire.0.clone();
        (TxEngine::new(wire), log)
    }

    /// Software-sealed frame for the same reply, including the 0x00 prefix.
    /// Built in its own full-size buffer — REPLY_BUF only fits streamed
    /// layouts, and the reference is a whole linearized frame.
    fn reference(id: u8, result: ResultCode, alert: bool, data: &[u8]) -> Vec<u8> {
        let mut b = FrameBuf::<64>::new();
        b.start(Id::new(id), Inst::status(result, alert));
        b.payload_mut()[..data.len()].copy_from_slice(data);
        b.finish(data.len() as u8);
        b.seal().to_vec()
    }

    /// Drive a staged frame to completion, returning the arm outcomes. The one
    /// CRC engine spans trigger and every arm boundary, as on the chip.
    /// Drive a staged frame to completion with the SAME engine handle used at
    /// stage (the snapshot lives in it, as on the chip).
    fn run(
        eng: &mut TxEngine<FakeWire>,
        crc: &mut FakeCrc,
        over: Option<ResultCode>,
    ) -> Vec<TxOut> {
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

    #[repr(align(2))]
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
        // 2 bytes ≤ SMALL_COPY_MAX → the copy path, even payload.
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
        // 3 even-addressed bytes: above SMALL_COPY_MAX → zero-copy; the odd
        // last byte is folded into the CRC in software (§3.2), not moved into
        // a tail arm.
        let data = Aligned([0xDE, 0xAD, 0xBE, 0]);
        eng.stage(&mut crc, 1, ResultCode::Ok, true, &data.0[..3])
            .unwrap();
        run(&mut eng, &mut crc, None);
        let log = log.borrow();
        let reference = reference(1, ResultCode::Ok, true, &data.0[..3]);
        let s = sends(&log);
        // Header, whole streamed payload, CRC.
        assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![3, 3, 2]);
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
        // Odd-addressed source above SMALL_COPY_MAX: streamed zero-copy like
        // any other span — the chip CRC provider stages odd pointers through
        // its copy channel (§5); the engine is parity-blind.
        let backing = Aligned([0u8, 0xDE, 0xAD, 0xBE]);
        let data = &backing.0[1..4];
        eng.stage(&mut crc, 1, ResultCode::Ok, true, data).unwrap();
        run(&mut eng, &mut crc, None);
        let log = log.borrow();
        let reference = reference(1, ResultCode::Ok, true, data);
        let s = sends(&log);
        // Header, streamed payload, CRC.
        assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![3, 3, 2]);
        assert_eq!(wire_bytes(&log), reference[1..]);
    }

    #[test]
    fn zero_copy_even_streams_three_arms() {
        let (mut eng, log) = engine();
        let mut crc = FakeCrc::new();
        let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
        eng.stage(&mut crc, 9, ResultCode::Ok, false, &data.0)
            .unwrap();
        run(&mut eng, &mut crc, None);
        let log = log.borrow();
        let s = sends(&log);
        // Header, Ext payload, then the CRC as its own final 2-byte arm.
        assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![3, 40, 2]);
        let reference = reference(9, ResultCode::Ok, false, &data.0);
        assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
        assert_eq!(wire_bytes(&log), reference[1..]);
        // CRC over the covered span matches the reference flavor (the leading
        // alignment byte is a no-op, §3.2).
        let covered_len = wire::covered_len(wire::len_for(40));
        let crc = u16::from_le_bytes([s[2][0], s[2][1]]);
        assert_eq!(osc_crc(&reference[..covered_len]), crc);
    }

    #[test]
    fn zero_copy_odd_streams_whole_payload() {
        let (mut eng, log) = engine();
        let mut crc = FakeCrc::new();
        let data = Aligned(core::array::from_fn::<u8, 41, _>(|i| !(i as u8)));
        eng.stage(&mut crc, 5, ResultCode::Ok, false, &data.0)
            .unwrap();
        run(&mut eng, &mut crc, None);
        let log = log.borrow();
        let s = sends(&log);
        // Odd payload streams whole; its last byte reaches the CRC via the
        // software fold at patch time (§3.2), so the wire CRC still matches
        // the byte-wise reference.
        assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![3, 41, 2]);
        let reference = reference(5, ResultCode::Ok, false, &data.0);
        assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
        assert_eq!(wire_bytes(&log), reference[1..]);
    }

    #[test]
    fn gather_matches_sealed_reference_of_concat() {
        let (mut eng, log) = engine();
        let mut crc = FakeCrc::new();
        // Three spans, odd interior lengths (no parity constraint, §5.2): the
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
        // Header, one contiguous snapshot arm, CRC — same shape as a plain
        // zero-copy read (the gather is invisible to the wire).
        assert_eq!(
            s.iter().map(Vec::len).collect::<Vec<_>>(),
            vec![3, concat.len(), 2]
        );
        assert_eq!(wire_bytes(&log), reference[1..]);
        assert_eq!(*s.last().unwrap(), reference[reference.len() - 2..]);
    }

    #[test]
    fn gather_tiny_total_takes_copy_path() {
        let (mut eng, log) = engine();
        let mut crc = FakeCrc::new();
        // Two 1-byte spans: total 2 <= SMALL_COPY_MAX — buffer copy, no
        // snapshot involved.
        let spans: [&[u8]; 2] = [&[0xAA], &[0xBB]];
        eng.stage_gather(&mut crc, 4, ResultCode::Ok, false, &spans)
            .unwrap();
        run(&mut eng, &mut crc, None);
        let log = log.borrow();
        let reference = reference(4, ResultCode::Ok, false, &[0xAA, 0xBB]);
        assert_eq!(wire_bytes(&log), reference[1..]);
        assert!(crc.snap.is_empty());
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
        eng.abort();
        assert_eq!(*log.borrow(), vec![Event::Release]);
        assert!(!eng.busy());
        eng.stage(&mut crc, 2, ResultCode::Ok, false, &[]).unwrap();
        // Aborting mid-stream also releases.
        eng.trigger(&mut crc, None);
        eng.abort();
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
        assert_eq!(outs, vec![TxOut::Armed, TxOut::Armed, TxOut::Released]);
        assert!(!eng.busy());
    }
}
