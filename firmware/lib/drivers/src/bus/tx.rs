//! Status-reply TX engine (`docs/osc-native-protocol.md` sec 4.2): stage a frame
//! layout, then trigger - break + two DMA arms, ID..payload then the 2-byte
//! CRC. The wire starts before the CRC is known; the hardware engine chews
//! the covered span in parallel and the value is patched into the CRC arm at
//! the boundary before it (the engine outruns the wire 8:1, F6, and DMA
//! fetches just-in-time, so the patch beats the read).

use crate::traits::bus::{CrcEngine, TxWire};
use osc_protocol::crc::osc_crc_byte;
use osc_protocol::frame::Header;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Inst, ResultCode};
use osc_servo_core::traits::SendError;

/// Payloads at or below this are copied into the staging buffer. Kept minimal:
/// the copy costs ~0.3 us/byte of turnaround (bench-measured at 3M), so
/// anything the DMA can stream in place should stream.
const SMALL_COPY_MAX: usize = 2;

const CRC_LEN: usize = core::mem::size_of::<u16>();

/// The CRC tail's slot in the staging buffer, behind the copy path's payload.
const CRC_AT: usize = Header::SIZE + SMALL_COPY_MAX;

/// Staging buffer size: header, the copy path's payload and the CRC tail.
pub const REPLY_BUF: usize = CRC_AT + CRC_LEN;

const INST_AT: usize = Header::SIZE - 1;

const CRC_ARM: Arm = Arm::Buf {
    off: CRC_AT as u16,
    len: CRC_LEN as u16,
};

/// Outcome of an arm-completion event.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TxOut {
    Armed,
    Released,
}

/// One DMA arm (or CRC feed) as a descriptor -- resolved to a slice only at
/// send time, so the engine never holds self-referential borrows.
#[derive(Copy, Clone)]
enum Arm {
    Buf { off: u16, len: u16 },
    Ext { ptr: *const u8, len: u16 },
}

const NO_ARM: Arm = Arm::Buf { off: 0, len: 0 };

enum State {
    Idle,
    /// `slot_key` is present on collision-tolerant replies: broadcast-ENUM
    /// replies' sec 9.2 job is to collide, so the composite's break-wake kill
    /// skips them, and the key (folded UID CRC) draws the reply's slot
    /// delay -- cycle-identical twin matchers otherwise answer in unison,
    /// and a sub-bit-aligned superposition of near-equal frames reads back
    /// as one clean frame, hiding the loser's subtree from the walk.
    Staged {
        slot_key: Option<u8>,
    },
    Streaming {
        crc_armed: bool,
    },
}

pub struct TxEngine<W: TxWire> {
    wire: W,
    buf: FrameBuf<REPLY_BUF>,
    /// ID..payload: the first arm.
    body: Arm,
    /// The covered span's even bulk, fed at trigger.
    feed: Arm,
    state: State,
    /// The covered byte the even-bulk feed leaves un-fed when the span is odd
    /// (sec 3.2): read and folded into the engine result at patch time (the
    /// pointer targets engine-stable storage -- buffer or snapshot).
    tail: Option<*const u8>,
    alert: bool,
    crc_misses: u32,
}

impl<W: TxWire> TxEngine<W> {
    pub fn new(wire: W) -> Self {
        Self {
            wire,
            buf: FrameBuf::new(),
            body: NO_ARM,
            feed: NO_ARM,
            state: State::Idle,
            tail: None,
            alert: false,
            crc_misses: 0,
        }
    }

    pub fn busy(&self) -> bool {
        !matches!(self.state, State::Idle)
    }

    /// A frame is staged but not yet triggered -- safe to abort (a fresh
    /// instruction supersedes it). Streaming frames must not be aborted.
    pub fn staged(&self) -> bool {
        matches!(self.state, State::Staged { .. })
    }

    /// The staged reply is exempt from the break-wake kill (sec 9.2 ENUM).
    pub fn collision_tolerant(&self) -> bool {
        self.slot_key().is_some()
    }

    /// A collision-tolerant staged reply's slot-delay key (sec 9.2), else None.
    pub fn slot_key(&self) -> Option<u8> {
        match self.state {
            State::Staged { slot_key } => slot_key,
            _ => None,
        }
    }

    /// Mark the staged reply collision-tolerant with its slot-delay key
    /// (sec 9.2: an ENUM reply's job is to collide with peer matchers, offset
    /// by its slot). No-op unless a frame is staged.
    pub fn mark_collision_tolerant(&mut self, key: u8) {
        if let State::Staged { slot_key } = &mut self.state {
            *slot_key = Some(key);
        }
    }

    /// Arms are on the wire -- the servo owns the line until the final TC.
    pub fn streaming(&self) -> bool {
        matches!(self.state, State::Streaming { .. })
    }

    /// Build the frame layout for a status reply; touches no wire state
    /// (enable-when-ready is [`trigger`](Self::trigger), sec 4.2).
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
    /// concatenated in order (sec 5.2 profile reads; a plain reply is the
    /// one-span case). Payload totals above [`SMALL_COPY_MAX`] are
    /// snapshotted through the CRC engine's stable buffer behind the header,
    /// at cumulative offsets - wire and CRC both stream the one contiguous
    /// snapshot, so a scattered read costs the same single copy as a plain
    /// read (sec 4.2).
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
        let cov = wire::covered_len(wire::len_for(p));
        let b = self.buf.bytes_mut();
        b[0] = wire::ALIGN_BYTE;
        b[1] = id;
        b[2] = wire::len_for(p);
        b[INST_AT] = Inst::status(result, alert).0;
        // An odd covered span feeds its even bulk and leaves the last byte
        // for the software fold at patch (sec 3.2); odd POINTERS are the CRC
        // provider's concern (it stages them through its copy channel, sec 5).
        let base: *const u8 = if total <= SMALL_COPY_MAX {
            let pay = self.buf.payload_mut();
            let mut at = 0;
            for s in spans {
                pay[at..at + s.len()].copy_from_slice(s);
                at += s.len();
            }
            self.body = Arm::Buf {
                off: 1,
                len: (cov - 1) as u16,
            };
            self.feed = Arm::Buf {
                off: 0,
                len: (cov & !1) as u16,
            };
            self.buf.bytes().as_ptr()
        } else {
            // Snapshot reads (sec 4.2): header and payload are copied ONCE
            // into the engine's stable snapshot buffer, header first (a later
            // copy waits out the one in flight), and both the wire arm and the
            // CRC feed stream the snapshot: the CRC provably covers the
            // transmitted bytes, and the reply carries an atomic point-in-time
            // image (`stage` runs kernel-exclusive; the provider orders the
            // copies ahead of both consumers).
            let base = crc.snapshot(0, &self.buf.bytes()[..Header::SIZE]);
            let mut off = Header::SIZE as u16;
            for s in spans {
                if s.is_empty() {
                    continue;
                }
                crc.snapshot(off, s);
                off += s.len() as u16;
            }
            self.body = Arm::Ext {
                ptr: base.wrapping_add(1),
                len: (cov - 1) as u16,
            };
            self.feed = Arm::Ext {
                ptr: base,
                len: (cov & !1) as u16,
            };
            base
        };
        // The fold byte is read at patch time, not here: the snapshot is
        // best-effort asynchronous and may still be streaming - by the
        // patch boundary the copy has long completed (sec 4.2).
        self.tail = (cov & 1 == 1).then(|| base.wrapping_add(cov - 1));
        // Placeholder CRC: what ships if the patch window is missed.
        self.buf.bytes_mut()[CRC_AT..].fill(0);
        self.alert = alert;
        self.state = State::Staged { slot_key: None };
        Ok(())
    }

    /// Finalize and start: optional result override (chain reclaim's
    /// predecessor-silent, sec 6) rewrites INST, then break + the body arm
    /// with the CRC feed behind it. The CRC value lands later, at the
    /// boundary before the CRC arm ([`on_arm_complete`]) - the wire starts
    /// before the CRC is known so the engine chews in parallel (sec 4.2).
    pub fn trigger<C: CrcEngine>(&mut self, crc: &mut C, over: Option<ResultCode>) {
        if !matches!(self.state, State::Staged { .. }) {
            debug_assert!(false, "trigger without a staged frame");
            return;
        }
        if let Some(r) = over {
            self.buf.bytes_mut()[INST_AT] = Inst::status(r, self.alert).0;
            // A snapshot reply streams its INST from the snapshot; a copy-path
            // reply ignores this byte.
            crc.snapshot(INST_AT as u16, &self.buf.bytes()[INST_AT..Header::SIZE]);
        }
        crc.reset();
        self.wire.start_frame();
        self.wire.send(resolve(&self.buf, self.body));
        crc.feed(resolve(&self.buf, self.feed));
        self.state = State::Streaming { crc_armed: false };
    }

    /// Per-arm DMA TC. After the body arm, patches the computed CRC into the
    /// buffer and streams the CRC arm; after the CRC arm, releases the wire
    /// (caller then applies deferred config).
    pub fn on_arm_complete<C: CrcEngine>(&mut self, crc: &mut C) -> TxOut {
        match self.state {
            State::Streaming { crc_armed: false } => {
                self.patch_crc(crc);
                self.wire.send(resolve(&self.buf, CRC_ARM));
                self.state = State::Streaming { crc_armed: true };
                TxOut::Armed
            }
            State::Streaming { crc_armed: true } => {
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
    /// write here -- by design. The CRC arm is last (>= header's worth of head
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
                // (sec 3.2) -- the only software CRC on the servo. SAFETY: the
                // pointer targets the frame buffer or the engine's snapshot,
                // both stable for this exchange; any snapshot copy completed
                // arms ago (transfer ordering).
                let v = match self.tail {
                    Some(b) => osc_crc_byte(v, unsafe { *b }),
                    None => v,
                };
                self.buf.bytes_mut()[CRC_AT..].copy_from_slice(&v.to_le_bytes());
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
mod tests;
