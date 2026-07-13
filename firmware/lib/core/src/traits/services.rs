use osc_protocol::FrameBytes;
use osc_protocol::wire::{MgmtOp, ResultCode, UID_LEN};

use crate::{BaudRate, BootMode};

/// Reply-side runtime failure. Overflow is a sizing bug (reply exceeds the
/// bus's staging buffer); Busy means a previous reply is still draining.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SendError {
    Busy,
    Overflow,
}

/// One encoded status reply, data-centric per driver-pattern sec 7.4: the bus
/// derives all wire placement (frame layout, chain slot timing, PAD) from
/// its cached request state; core hands only result + payload.
/// `alert` mirrors the device-level alarm state (osc-native sec 5.3 layer 3).
pub struct Status<'a> {
    pub result: ResultCode,
    pub alert: bool,
    pub data: &'a [u8],
}

/// A decoded instruction addressed to this servo; addressing and group-op
/// slot resolution already done by the bus. Core sees no wire bytes.
pub enum Request<'a> {
    Ping,
    Read {
        addr: u16,
        count: u16,
    },
    /// READ/GREAD with the PROFILE flag: reply with the named slot's spans
    /// concatenated (sec 5.2).
    ReadProfile {
        slot: u8,
    },
    Write {
        addr: u16,
        data: FrameBytes<'a>,
        hold: bool,
    },
    Commit,
    /// MGMT ENUM (sec 9.2): reply with the full UID iff ours begins with the
    /// prefix; silent otherwise.
    Enumerate {
        prefix_len: u8,
        prefix: [u8; UID_LEN],
    },
    /// MGMT ASSIGN (sec 9.2): the servo whose UID matches takes `new_id`
    /// immediately and acks from it.
    Assign {
        uid: [u8; UID_LEN],
        new_id: u8,
    },
    /// MGMT CAL (sec 9.3, broadcast-only): `gaps + 1` bare breaks spaced
    /// `gap_us` apart follow this frame -- the host-crystal ruler the bus
    /// measures its clock against.
    Calibrate {
        gap_us: u16,
        gaps: u8,
    },
    Mgmt {
        op: MgmtOp,
        args: FrameBytes<'a>,
    },
    /// Valid frame the bus cannot resolve (unknown opcode/flag combination).
    /// Dispatch answers `ResultCode::Instruction` (sec 5.3 layer 2).
    Unsupported,
}

/// Bus-derived context core may not derive itself.
pub struct RequestCtx {
    /// The wire contract permits a status for this request (false for
    /// NOREPLY-flagged instructions and non-acking broadcasts).
    pub may_reply: bool,
}

/// Max spans per gathered status -- mirrors the profile region's slot width
/// (`regions::profile::SPANS_PER_SLOT`).
pub const GATHER_MAX: usize = 8;

/// Dispatcher-facing reply surface.
pub trait Reply {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError>;
    /// Scattered status (sec 5.2): the payload is `spans` concatenated in order --
    /// the bus copies them once into its snapshot and streams one frame.
    /// `spans` holds at most [`GATHER_MAX`] entries.
    fn send_status_gather(
        &mut self,
        result: ResultCode,
        alert: bool,
        spans: &[&[u8]],
    ) -> Result<(), SendError>;
    /// Deferred ID change -- applies after the in-flight TX completes.
    fn stage_id(&mut self, id: u8);
    /// Immediate ID change -- a status staged after this call already carries
    /// the new id (the ASSIGN ack leaves from it, sec 9.2).
    fn set_id(&mut self, id: u8);
    /// Deferred baud change -- applied at TX complete so the ack leaves at the old rate.
    fn stage_baud(&mut self, baud: BaudRate);
    /// Immediate: the bus caches this for chain-reclaim timing (sec 6).
    fn set_response_deadline(&mut self, us: u16);
    /// Deferred reboot, honored after any in-flight TX drains.
    fn stage_reboot(&mut self, mode: BootMode);
    /// MGMT CAL accepted (sec 9.3): the bus measures the announced break train
    /// against its own clock and trims the oscillator from it.
    fn begin_clock_cal(&mut self, gap_us: u16, gaps: u8);
}

/// Outcome of a [`Dispatch::dispatch`]: whether a table effect was staged.
pub enum Dispatched {
    /// No table effect -- nothing awaits a verdict in the dispatcher (any
    /// staged reply is the bus's wire effect to gate).
    Done,
    /// A table effect is staged; the caller owes the verdict --
    /// [`Dispatch::commit`] on CRC pass, [`Dispatch::revert`] on fail.
    Pending,
}

/// Single-shot typed dispatch: the bus hands one fully-decoded request plus
/// its ctx. `R` is generic per call so the hot path avoids `dyn Reply`.
///
/// Dispatch-before-verdict is the spine: `dispatch` runs before the frame's
/// CRC verdict and STAGES effects -- the reply through `R` (the wire effect,
/// sent or dropped by the bus) and writes into the staging buffer (the table
/// effect, gated by [`Self::commit`]/[`Self::revert`]). COMMIT and MGMT
/// cannot stage their effects; the bus routes them verdict-first (CRC checked
/// before dispatch), and `dispatch` applies them directly on that contract.
pub trait Dispatch {
    fn dispatch<R: Reply>(
        &mut self,
        req: Request<'_>,
        ctx: RequestCtx,
        reply: &mut R,
    ) -> Dispatched;

    /// Verdict pass: promote the staged table effect -- a plain write applies
    /// into the live table and fires its hooks; a held write keeps its
    /// entries for a later COMMIT.
    fn commit<R: Reply>(&mut self, reply: &mut R);

    /// Verdict fail (or frame died): discard the staged table effect.
    fn revert(&mut self);
}
