//! Status-reply TX engine (`docs/osc-native-protocol.md` §4.2): stage a frame
//! layout, then trigger — break + up to three DMA arms, with the CRC computed
//! by the hardware engine and patched into the trailing bytes while earlier
//! bytes are already on the wire (fire-first append-later: the engine outruns
//! the wire 8:1, F6, and DMA fetches just-in-time).

use crate::traits::bus::{CrcEngine, TxWire};
use osc_core::traits::SendError;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Id, Inst, ResultCode};

/// Staging buffer size: the largest legal frame footprint (§3.1), 258.
pub const REPLY_BUF: usize = super::FRAME_MAX;

/// Payloads at or below this are copied into the staging buffer; a second
/// and third DMA arm cost more than a small memcpy.
const SMALL_COPY_MAX: usize = 32;

/// Copy-path capacity: `FrameBuf::payload_mut` reserves pad + CRC space, so
/// a copied payload caps one byte short of `MAX_PAYLOAD`. Only odd-addressed
/// maximal payloads hit this; the dispatcher never produces them.
const COPY_PAYLOAD_MAX: usize = REPLY_BUF - 7;

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
    // bytes (a 251-byte payload from an odd table address).
    Buf { off: u8, len: u16 },
    Ext { ptr: *const u8, len: u16 },
}

const NO_ARM: Arm = Arm::Buf { off: 0, len: 0 };

enum State {
    Idle,
    Staged,
    Streaming { next: u8 },
}

pub struct TxEngine<W: TxWire> {
    wire: W,
    buf: FrameBuf<REPLY_BUF>,
    arms: [Arm; 3],
    n_arms: u8,
    feeds: [Arm; 3],
    n_feeds: u8,
    /// Buffer offset of the CRC tail slot (zeroed at stage as the placeholder).
    crc_off: u16,
    covered: u16,
    state: State,
    result: ResultCode,
    pad: bool,
    alert: bool,
    crc_misses: u32,
}

impl<W: TxWire> TxEngine<W> {
    pub fn new(wire: W) -> Self {
        Self {
            wire,
            buf: FrameBuf::new(),
            arms: [NO_ARM; 3],
            n_arms: 0,
            feeds: [NO_ARM; 3],
            n_feeds: 0,
            crc_off: 0,
            covered: 0,
            state: State::Idle,
            result: ResultCode::Ok,
            pad: false,
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
        matches!(self.state, State::Staged)
    }

    /// Arms are on the wire — the servo owns the line until the final TC.
    pub fn streaming(&self) -> bool {
        matches!(self.state, State::Streaming { .. })
    }

    /// Build the frame layout for a status reply; touches no wire state
    /// (enable-when-ready is [`trigger`](Self::trigger), §4.2).
    ///
    /// Contract: slices above [`SMALL_COPY_MAX`] are streamed in place and
    /// must reference storage that outlives the transmission — the dispatcher
    /// only produces such slices from the static control table.
    pub fn stage(
        &mut self,
        id: u8,
        result: ResultCode,
        alert: bool,
        data: &[u8],
    ) -> Result<(), SendError> {
        if self.busy() {
            return Err(SendError::Busy);
        }
        if data.len() > wire::MAX_PAYLOAD as usize {
            return Err(SendError::Overflow);
        }
        let p = data.len() as u8;
        let pad = wire::needs_pad(p);
        let inst = Inst::status(result, pad, alert);
        // Odd pointers can't feed the CRC DMA (F12); small payloads are
        // cheaper to copy than to arm.
        if data.len() <= SMALL_COPY_MAX || data.as_ptr() as usize & 1 == 1 {
            if data.len() > COPY_PAYLOAD_MAX {
                return Err(SendError::Overflow);
            }
            self.buf.start(Id::new(id), inst);
            self.buf.payload_mut()[..data.len()].copy_from_slice(data);
            self.buf.finish(p);
            let len = wire::len_for(p);
            let cov = wire::covered_len(len);
            self.arms[0] = Arm::Buf {
                off: 1,
                len: (wire::footprint(len) - 1) as u16,
            };
            self.n_arms = 1;
            self.feeds[0] = Arm::Buf {
                off: 0,
                len: cov as u16,
            };
            self.n_feeds = 1;
            self.crc_off = cov as u16;
        } else {
            // Zero-copy (§4.2): header + tail in the buffer, payload streamed
            // from its home. Every CRC feed is even-length and even-addressed
            // by construction (F12): the 4-byte covered prefix is 2 whole
            // halfwords in the align(2) buffer, and an odd payload donates
            // its last byte to the tail so it halfword-pairs with the PAD.
            let b = self.buf.bytes_mut();
            b[0] = wire::CRC_PREFIX;
            b[1] = id;
            b[2] = wire::len_for(p);
            b[3] = inst.0;
            let ptr = data.as_ptr();
            if pad {
                b[4] = data[data.len() - 1];
                b[5] = 0;
                self.arms = [
                    Arm::Buf { off: 1, len: 3 },
                    Arm::Ext {
                        ptr,
                        len: (p - 1) as u16,
                    },
                    Arm::Buf { off: 4, len: 4 },
                ];
                self.n_arms = 3;
                self.feeds = [
                    Arm::Buf { off: 0, len: 4 },
                    Arm::Ext {
                        ptr,
                        len: (p - 1) as u16,
                    },
                    Arm::Buf { off: 4, len: 2 },
                ];
                self.n_feeds = 3;
                self.crc_off = 6;
            } else {
                self.arms = [
                    Arm::Buf { off: 1, len: 3 },
                    Arm::Ext { ptr, len: p as u16 },
                    Arm::Buf { off: 4, len: 2 },
                ];
                self.n_arms = 3;
                self.feeds[0] = Arm::Buf { off: 0, len: 4 };
                self.feeds[1] = Arm::Ext { ptr, len: p as u16 };
                self.n_feeds = 2;
                self.crc_off = 4;
            }
        }
        // Placeholder CRC: what ships if the patch window is missed.
        let off = self.crc_off as usize;
        self.buf.bytes_mut()[off..off + 2].copy_from_slice(&[0, 0]);
        self.covered = wire::covered_len(wire::len_for(p)) as u16;
        self.result = result;
        self.pad = pad;
        self.alert = alert;
        self.state = State::Staged;
        Ok(())
    }

    /// Finalize and start: optional result override (chain reclaim's
    /// predecessor-silent, §6) rewrites INST, then CRC-feed + patch, then
    /// break + first arm.
    pub fn trigger<C: CrcEngine>(&mut self, crc: &mut C, over: Option<ResultCode>) {
        if !matches!(self.state, State::Staged) {
            debug_assert!(false, "trigger without a staged frame");
            return;
        }
        self.buf.bytes_mut()[3] = Inst::status(over.unwrap_or(self.result), self.pad, self.alert).0;
        crc.reset();
        for i in 0..self.n_feeds as usize {
            let span = resolve(&self.buf, self.feeds[i]);
            crc.feed(span);
        }
        let mut budget = super::SPIN_PER_BYTE * self.covered as u32;
        let crc = loop {
            if let Some(v) = crc.result() {
                break Some(v);
            }
            if budget == 0 {
                break None;
            }
            budget -= 1;
            core::hint::spin_loop();
        };
        match crc {
            Some(v) => {
                let off = self.crc_off as usize;
                self.buf.bytes_mut()[off..off + 2].copy_from_slice(&v.to_le_bytes());
            }
            // Ship the placeholder zeros: the host sees a CRC-fail frame and
            // retries; never wedge the wire on a sick CRC engine.
            None => self.crc_misses = self.crc_misses.wrapping_add(1),
        }
        self.wire.start_frame();
        self.wire.send(resolve(&self.buf, self.arms[0]));
        self.state = State::Streaming { next: 1 };
    }

    /// Per-arm DMA TC. Streams the next arm, or releases the wire after the
    /// last one (caller then applies deferred config).
    pub fn on_arm_complete(&mut self) -> TxOut {
        match self.state {
            State::Streaming { next } if next < self.n_arms => {
                self.wire.send(resolve(&self.buf, self.arms[next as usize]));
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

    struct FakeCrc(u16);

    impl CrcEngine for FakeCrc {
        fn reset(&mut self) {
            self.0 = 0;
        }
        fn feed(&mut self, span: &[u8]) {
            assert_eq!(span.len() % 2, 0, "CRC feed must be even-length (F12)");
            assert_eq!(
                span.as_ptr() as usize % 2,
                0,
                "CRC feed must be even-addressed (F12)"
            );
            self.0 = osc_crc_continue(self.0, span);
        }
        fn result(&mut self) -> Option<u16> {
            Some(self.0)
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
    fn reference(id: u8, result: ResultCode, alert: bool, data: &[u8]) -> Vec<u8> {
        let mut b = FrameBuf::<REPLY_BUF>::new();
        b.start(Id::new(id), Inst::status(result, false, alert));
        b.payload_mut()[..data.len()].copy_from_slice(data);
        b.finish(data.len() as u8);
        b.seal().to_vec()
    }

    /// Drive a staged frame to completion, returning the arm outcomes.
    fn run(eng: &mut TxEngine<FakeWire>, over: Option<ResultCode>) -> Vec<TxOut> {
        eng.trigger(&mut FakeCrc(0), over);
        let mut outs = Vec::new();
        loop {
            let out = eng.on_arm_complete();
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
        eng.stage(7, ResultCode::Ok, false, &[]).unwrap();
        run(&mut eng, None);
        let log = log.borrow();
        let reference = reference(7, ResultCode::Ok, false, &[]);
        assert_eq!(log[0], Event::Start);
        assert_eq!(sends(&log).len(), 1);
        assert_eq!(wire_bytes(&log), reference[1..]);
        assert_eq!(*log.last().unwrap(), Event::Release);
        // No pad on an empty payload: status bit set, PAD clear.
        assert_eq!(wire_bytes(&log)[2], 0x80);
    }

    #[test]
    fn small_padded_reply_matches_sealed_reference() {
        let (mut eng, log) = engine();
        let data = [0xDE, 0xAD, 0xBE];
        eng.stage(1, ResultCode::Ok, true, &data).unwrap();
        run(&mut eng, None);
        let log = log.borrow();
        let reference = reference(1, ResultCode::Ok, true, &data);
        assert_eq!(sends(&log).len(), 1);
        assert_eq!(wire_bytes(&log), reference[1..]);
        let inst = Inst(wire_bytes(&log)[2]);
        assert!(inst.is_status());
        assert!(inst.pad());
        assert!(inst.alert());
    }

    #[test]
    fn zero_copy_even_streams_three_arms() {
        let (mut eng, log) = engine();
        let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
        eng.stage(9, ResultCode::Ok, false, &data.0).unwrap();
        run(&mut eng, None);
        let log = log.borrow();
        let s = sends(&log);
        assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![3, 40, 2]);
        let reference = reference(9, ResultCode::Ok, false, &data.0);
        assert_eq!(wire_bytes(&log), reference[1..]);
        // CRC over prefix + covered span matches the reference flavor.
        let covered_len = wire::covered_len(wire::len_for(40));
        let crc = u16::from_le_bytes([s[2][0], s[2][1]]);
        assert_eq!(osc_crc(&reference[..covered_len]), crc);
    }

    #[test]
    fn zero_copy_odd_moves_last_byte_into_tail() {
        let (mut eng, log) = engine();
        let data = Aligned(core::array::from_fn::<u8, 41, _>(|i| !(i as u8)));
        eng.stage(5, ResultCode::Ok, false, &data.0).unwrap();
        run(&mut eng, None);
        let log = log.borrow();
        let s = sends(&log);
        assert_eq!(s.iter().map(Vec::len).collect::<Vec<_>>(), vec![3, 40, 4]);
        let reference = reference(5, ResultCode::Ok, false, &data.0);
        assert_eq!(s[2][0], data.0[40]);
        assert_eq!(s[2][1], 0x00);
        assert_eq!(wire_bytes(&log), reference[1..]);
    }

    #[test]
    fn override_rewrites_inst_and_crc_still_validates() {
        let (mut eng, log) = engine();
        let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
        eng.stage(9, ResultCode::Ok, false, &data.0).unwrap();
        run(&mut eng, Some(ResultCode::PredecessorSilent));
        let log = log.borrow();
        let wire_out = wire_bytes(&log);
        let inst = Inst(wire_out[2]);
        assert_eq!(inst.result(), Some(ResultCode::PredecessorSilent));
        let mut covered = vec![wire::CRC_PREFIX];
        let covered_len = wire::covered_len(wire::len_for(40));
        covered.extend_from_slice(&wire_out[..covered_len - 1]);
        let crc = u16::from_le_bytes([wire_out[covered_len - 1], wire_out[covered_len]]);
        assert_eq!(osc_crc(&covered), crc);
    }

    #[test]
    fn double_stage_is_busy_and_overflow_rejected() {
        let (mut eng, _log) = engine();
        eng.stage(1, ResultCode::Ok, false, &[]).unwrap();
        assert!(eng.busy());
        assert_eq!(
            eng.stage(2, ResultCode::Ok, false, &[]),
            Err(SendError::Busy)
        );
        let big = [0u8; 253];
        let (mut eng2, _log2) = engine();
        assert_eq!(
            eng2.stage(1, ResultCode::Ok, false, &big),
            Err(SendError::Overflow)
        );
        assert!(!eng2.busy());
    }

    #[test]
    fn abort_releases_and_unblocks_stage() {
        let (mut eng, log) = engine();
        eng.stage(1, ResultCode::Ok, false, &[]).unwrap();
        eng.abort();
        assert_eq!(*log.borrow(), vec![Event::Release]);
        assert!(!eng.busy());
        eng.stage(2, ResultCode::Ok, false, &[]).unwrap();
        // Aborting mid-stream also releases.
        eng.trigger(&mut FakeCrc(0), None);
        eng.abort();
        assert_eq!(*log.borrow().last().unwrap(), Event::Release);
        assert!(!eng.busy());
    }

    #[test]
    fn arm_sequencing_ends_in_released_exactly_once() {
        let (mut eng, _log) = engine();
        let data = Aligned(core::array::from_fn::<u8, 40, _>(|i| i as u8));
        eng.stage(9, ResultCode::Ok, false, &data.0).unwrap();
        let outs = run(&mut eng, None);
        assert_eq!(outs, vec![TxOut::Armed, TxOut::Armed, TxOut::Released]);
        assert!(!eng.busy());
    }
}
