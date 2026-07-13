//! Status-reply TX engine (`docs/osc-native-protocol.md` sec 4.2): stage a frame
//! layout, then trigger -- break + up to four DMA arms, the last always the
//! 2-byte CRC. The wire starts before the CRC is known; the hardware engine
//! chews the covered span in parallel and the value is patched into that final
//! arm at the boundary before it (the engine outruns the wire 8:1, F6, and DMA
//! fetches just-in-time, so the patch beats the read).

use crate::traits::bus::{CrcEngine, TxWire};
use osc_protocol::crc::osc_crc_continue;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Id, Inst, ResultCode};
use osc_servo_core::traits::SendError;

/// Staging buffer size. Payloads stream from the engine's snapshot (sec 4.2), so
/// the buffer holds only the header and CRC tail plus the <=
/// [`SMALL_COPY_MAX`] copy path.
pub const REPLY_BUF: usize = 16;

/// Payloads at or below this are copied into the staging buffer. Kept minimal:
/// the copy costs ~0.3 us/byte of turnaround (bench-measured at 3M), so
/// anything the DMA can stream in place should stream. The floor exists
/// because an odd payload donates its last byte to the tail arm -- at p = 1
/// that would leave a zero-length DMA arm.
const SMALL_COPY_MAX: usize = 2;

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
    /// replies' sec 9.2 job is to collide, so the composite's break-wake kill
    /// skips them, and the key (folded UID CRC) draws the reply's slot
    /// delay -- cycle-identical twin matchers otherwise answer in unison,
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
    /// snapshotted through the CRC engine's stable buffer at cumulative
    /// offsets -- wire and CRC both stream the one contiguous snapshot, so a
    /// scattered read costs the same single copy as a plain read (sec 4.2).
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
        // at patch (sec 3.2); odd POINTERS are the CRC provider's concern (it
        // stages them through its copy channel, sec 5).
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
            // Snapshot reads (sec 4.2): the payload is copied ONCE into the
            // engine's stable snapshot buffer -- each span at its cumulative
            // offset -- and both the wire arms and the CRC feeds stream the
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
            // best-effort asynchronous and may still be streaming -- by the
            // patch boundary the copy has long completed (sec 4.2).
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
    /// predecessor-silent, sec 6) rewrites INST, then break + first arm with the
    /// first CRC feed armed behind it. The CRC value lands later, at the
    /// boundary before its own trailing arm ([`on_arm_complete`]) -- the wire
    /// starts before the CRC is known so the engine chews in parallel (sec 4.2).
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
mod tests;
