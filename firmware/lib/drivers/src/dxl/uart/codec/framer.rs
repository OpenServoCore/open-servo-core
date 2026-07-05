//! Frame-driven RX classifier — replaces the streaming `Parser` on the
//! receive path. Where the parser emitted a running event stream the sink
//! stepped through byte-by-byte, the framer classifies one whole frame at
//! the head of the RX ring (`probe` over its first 8 bytes) and then, for
//! frames this servo must decode, copies the whole frame into a
//! driver-owned scratch — fused with the per-byte CRC fold — so
//! [`decode_instruction`] runs once over a contiguous, wrap-free slice.
//!
//! The framer holds no ring bytes across polls: each poll copies the
//! newly-published prefix into `scratch` and advances the ring read cursor
//! as it copies (see [`CodecRx::poll`](super::codec_rx::CodecRx::poll)).
//! Foreign / Status / ChainStatus / oversize frames are never copied — the
//! caller's [`SkipFsm`](super::skip::SkipFsm) consumes them straight off the
//! ring.

use dxl_protocol::CrcUmts;
use dxl_protocol::frame::{FrameKind, Probe, RawFrame, probe};
use dxl_protocol::types::packet::{Instruction, decode_instruction};
use dxl_protocol::wire::{BROADCAST_ID, CRC_BYTES, REQUEST_HEADER_BYTES};

/// Largest instruction frame this servo copies into `scratch` for one-shot
/// decode. A frame whose wire `total` exceeds this is byte-skipped instead
/// (it can't be one we participate in at the default control-RW sizing).
/// Sized above the worst-case own instruction: a whole-table Write plus its
/// sync header and CRC, with margin.
pub(crate) const HELD_FRAME_MAX: usize = 192;

/// Framer lifecycle. `Hunt` re-probes the ring head each poll; `Copy` holds
/// the copy-in-progress bookkeeping for one own/broadcast instruction frame
/// spanning one or more polls.
enum Mode {
    Hunt,
    Copy(CopyState),
}

/// One in-flight copy of an own/broadcast instruction frame.
struct CopyState {
    /// Full wire size of the frame (`probe`'s `total`).
    total: usize,
    /// Bytes copied into `scratch` so far.
    copied: usize,
    id: u8,
    instruction: u8,
}

/// Classification of the frame at the ring head, as the caller must act on
/// it. `Copy` frames are folded into `scratch`; everything else is skipped
/// raw off the ring under `id`.
pub(super) enum FrameClass {
    /// Not enough bytes to classify yet — wait for more wire.
    NeedMore,
    /// The leading `skip` bytes cannot start a frame — drop and re-probe.
    Junk { skip: usize },
    /// An own/broadcast instruction frame small enough to decode: begin
    /// copying `total` bytes into `scratch`.
    Copy {
        total: usize,
        id: u8,
        instruction: u8,
    },
    /// A frame to byte-skip whole: foreign instruction, Status, ChainStatus,
    /// or an oversize instruction. `is_instruction` gates drift sampling.
    Skip {
        total: usize,
        id: u8,
        is_instruction: bool,
    },
}

/// Result of a completed frame copy.
pub(super) enum FrameOutcome<'a> {
    /// CRC matched — the decoded instruction borrows `scratch`.
    Good {
        instr: Instruction<'a>,
        broadcast: bool,
    },
    /// CRC mismatch or a malformed body — silent drop, advance past the
    /// header, and re-probe.
    Bad,
}

pub(super) struct Framer<CRC: CrcUmts> {
    scratch: [u8; HELD_FRAME_MAX],
    /// Running CRC over the copied bytes `[0, total-2)`; compared against the
    /// trailing wire CRC at [`Self::finish`].
    crc: CRC,
    mode: Mode,
}

impl<CRC: CrcUmts> Framer<CRC> {
    pub(super) fn new() -> Self {
        Self {
            scratch: [0; HELD_FRAME_MAX],
            crc: CRC::new(),
            mode: Mode::Hunt,
        }
    }

    /// A frame copy is in progress across polls.
    pub(super) fn is_copying(&self) -> bool {
        matches!(self.mode, Mode::Copy(_))
    }

    /// Classify the frame at the ring head from its first 8 bytes. `front` /
    /// `back` are the ring's contiguous slices (pre-/post-wrap); the header
    /// may straddle the wrap, so probe a stack copy of up to 8 bytes.
    /// `our_id` decides own vs foreign for plain instructions.
    pub(super) fn classify(&self, front: &[u8], back: &[u8], our_id: u8) -> FrameClass {
        let mut head = [0u8; REQUEST_HEADER_BYTES];
        let n = fill(&mut head, front, back);
        match probe(&head[..n]) {
            Probe::NeedMore => FrameClass::NeedMore,
            Probe::Junk { skip } => FrameClass::Junk { skip },
            Probe::Frame {
                total,
                id,
                instruction,
                kind,
            } => {
                let is_instruction = kind == FrameKind::Instruction;
                let own = kind == FrameKind::Instruction
                    && (id == our_id || id == BROADCAST_ID)
                    && total <= HELD_FRAME_MAX;
                if own {
                    FrameClass::Copy {
                        total,
                        id,
                        instruction,
                    }
                } else {
                    FrameClass::Skip {
                        total,
                        id,
                        is_instruction,
                    }
                }
            }
        }
    }

    /// Enter `Copy` for a frame of `total` wire bytes and reset the CRC fold.
    pub(super) fn begin(&mut self, total: usize, id: u8, instruction: u8) {
        self.crc.reset();
        self.mode = Mode::Copy(CopyState {
            total,
            copied: 0,
            id,
            instruction,
        });
    }

    /// Copy the available ring bytes (`front` then `back`) into `scratch`,
    /// folding each byte at wire index `< total-2` into the CRC. Returns the
    /// number of bytes consumed (the caller advances the ring by that much).
    /// Stops at `total`; the trailing 2 CRC bytes are copied but not folded.
    pub(super) fn absorb(&mut self, front: &[u8], back: &[u8]) -> usize {
        let Framer { scratch, crc, mode } = self;
        let Mode::Copy(cs) = mode else {
            return 0;
        };
        let mut consumed = 0;
        let fold_end = cs.total.saturating_sub(CRC_BYTES);
        for slice in [front, back] {
            for &b in slice {
                if cs.copied >= cs.total {
                    return consumed;
                }
                scratch[cs.copied] = b;
                if cs.copied < fold_end {
                    crc.update_byte(b);
                }
                cs.copied += 1;
                consumed += 1;
            }
        }
        consumed
    }

    /// Whether the in-flight copy has all `total` bytes.
    pub(super) fn copy_complete(&self) -> bool {
        matches!(&self.mode, Mode::Copy(cs) if cs.copied >= cs.total)
    }

    /// Finish a completed copy: compare the folded CRC to the trailing wire
    /// bytes and decode. Returns [`FrameOutcome::Bad`] on mismatch or a
    /// malformed body. Leaves the framer in `Hunt`-ready state via
    /// [`Self::reset`] (caller invokes after consuming the outcome).
    pub(super) fn finish(&self) -> FrameOutcome<'_> {
        let Mode::Copy(cs) = &self.mode else {
            return FrameOutcome::Bad;
        };
        let total = cs.total;
        let received = u16::from_le_bytes([
            self.scratch[total - CRC_BYTES],
            self.scratch[total - CRC_BYTES + 1],
        ]);
        if self.crc.finalize() != received {
            return FrameOutcome::Bad;
        }
        let body = &self.scratch[REQUEST_HEADER_BYTES..total - CRC_BYTES];
        let frame = RawFrame {
            kind: FrameKind::Instruction,
            id: cs.id,
            instruction: cs.instruction,
            body,
        };
        match decode_instruction(&frame) {
            Ok(instr) => FrameOutcome::Good {
                instr,
                broadcast: cs.id == BROADCAST_ID,
            },
            Err(_) => FrameOutcome::Bad,
        }
    }

    /// Return to `Hunt` after a completed / dropped frame.
    pub(super) fn reset(&mut self) {
        self.mode = Mode::Hunt;
    }
}

/// Copy up to `dst.len()` bytes from the two contiguous ring slices into
/// `dst`; returns how many landed.
fn fill(dst: &mut [u8], front: &[u8], back: &[u8]) -> usize {
    let mut n = 0;
    for slice in [front, back] {
        for &b in slice {
            if n >= dst.len() {
                return n;
            }
            dst[n] = b;
            n += 1;
        }
    }
    n
}
