use osc_protocol::FrameBytes;
use osc_protocol::wire::{MgmtOp, ResultCode};

use crate::{BaudRate, BootMode};

/// Reply-side runtime failure. Overflow is a sizing bug (reply exceeds the
/// bus's staging buffer); Busy means a previous reply is still draining.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SendError {
    Busy,
    Overflow,
}

/// One encoded status reply, data-centric per driver-pattern §7.4: the bus
/// derives all wire placement (frame layout, chain slot timing, PAD) from
/// its cached request state; core hands only result + payload.
/// `alert` mirrors the device-level alarm state (osc-native §5.3 layer 3).
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
    Write {
        addr: u16,
        data: FrameBytes<'a>,
        hold: bool,
    },
    Commit,
    Mgmt {
        op: MgmtOp,
        args: FrameBytes<'a>,
    },
    /// Valid frame the bus cannot resolve (unknown opcode/flag combination).
    /// Dispatch answers `ResultCode::Instruction` (§5.3 layer 2).
    Unsupported,
}

/// Bus-derived context core may not derive itself.
pub struct RequestCtx {
    /// The wire contract permits a status for this request (false for
    /// NOREPLY-flagged instructions and non-acking broadcasts).
    pub may_reply: bool,
}

/// Dispatcher-facing reply surface.
pub trait Reply {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), SendError>;
    /// Deferred ID change — applies after the in-flight TX completes.
    fn stage_id(&mut self, id: u8);
    /// Deferred baud change — applied at TX complete so the ack leaves at the old rate.
    fn stage_baud(&mut self, baud: BaudRate);
    /// Immediate: the bus caches this for chain-reclaim timing (§6).
    fn set_response_deadline(&mut self, us: u16);
    /// Deferred reboot, honored after any in-flight TX drains.
    fn stage_reboot(&mut self, mode: BootMode);
}

/// Outcome of a [`Dispatch::dispatch`]: whether a table effect was staged.
pub enum Dispatched {
    /// No table effect — nothing awaits a verdict in the dispatcher (any
    /// staged reply is the bus's wire effect to gate).
    Done,
    /// A table effect is staged; the caller owes the verdict —
    /// [`Dispatch::commit`] on CRC pass, [`Dispatch::revert`] on fail.
    Pending,
}

/// Single-shot typed dispatch: the bus hands one fully-decoded request plus
/// its ctx. `R` is generic per call so the hot path avoids `dyn Reply`.
///
/// Dispatch-before-verdict is the spine: `dispatch` runs before the frame's
/// CRC verdict and STAGES effects — the reply through `R` (the wire effect,
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

    /// Verdict pass: promote the staged table effect — a plain write applies
    /// into the live table and fires its hooks; a held write keeps its
    /// entries for a later COMMIT.
    fn commit<R: Reply>(&mut self, reply: &mut R);

    /// Verdict fail (or frame died): discard the staged table effect.
    fn revert(&mut self);
}
