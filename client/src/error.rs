//! Three-layer errors mirroring protocol sec 5.3, plus the carrier:
//! the pipe died / the link refused or desynced / the frame layer went
//! silent / the servo answered a non-OK result. ALERT is a flag on replies
//! (device layer, orthogonal), never an error.

use std::fmt;

use osc_protocol::wire::ResultCode;

use crate::pipe::PipeError;
use crate::pos_lut::LutError;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    Pipe(PipeError),
    Link(LinkError),
    /// No reply within the engine's window; `slot` is the silent GREAD list
    /// position (0 for unicast).
    Timeout {
        slot: u8,
    },
    /// Instruction-level rejection from the servo.
    Servo(ResultCode),
    /// The descriptor cannot name what the call needs (a field, the stamp
    /// recipe): the servo was never asked.
    Descriptor(String),
    /// A position table write that did not go live.
    Lut(LutError),
    /// The node does not publish a register the call needs: its firmware
    /// predates it.
    Unpublished(&'static str),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkError {
    /// The adapter refused the submit.
    Rejected(RejectReason),
    /// A record the session cannot reconcile (wrong seq, SEQ_NONE, decode
    /// failure mid-command): an adapter-invariant breach on a reliable pipe.
    /// The session is poisoned; reconnect.
    Desync(String),
    /// A record that does not parse at all.
    Malformed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectReason {
    BadId,
    BadInst,
    BadPayload,
    TooLong,
    Busy,
    Malformed,
    Other(u8),
}

impl RejectReason {
    pub fn from_byte(b: u8) -> Self {
        use osc_host::link::record as rec;
        match b {
            rec::REASON_BAD_ID => RejectReason::BadId,
            rec::REASON_BAD_INST => RejectReason::BadInst,
            rec::REASON_BAD_PAYLOAD => RejectReason::BadPayload,
            rec::REASON_TOO_LONG => RejectReason::TooLong,
            rec::REASON_BUSY => RejectReason::Busy,
            rec::REASON_MALFORMED => RejectReason::Malformed,
            other => RejectReason::Other(other),
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Pipe(e) => write!(f, "{e}"),
            Error::Link(LinkError::Rejected(r)) => write!(f, "adapter rejected: {r:?}"),
            Error::Link(LinkError::Desync(m)) => write!(f, "link desync: {m}"),
            Error::Link(LinkError::Malformed) => write!(f, "malformed record"),
            Error::Timeout { slot } => write!(f, "no reply (slot {slot})"),
            Error::Servo(code) => write!(f, "servo answered {code:?}"),
            Error::Descriptor(m) => write!(f, "descriptor: {m}"),
            Error::Lut(e) => write!(f, "{e}"),
            Error::Unpublished(r) => write!(
                f,
                "the servo does not publish {r}: its firmware predates it, reflash"
            ),
        }
    }
}

impl From<PipeError> for Error {
    fn from(e: PipeError) -> Self {
        Error::Pipe(e)
    }
}

impl From<LinkError> for Error {
    fn from(e: LinkError) -> Self {
        Error::Link(e)
    }
}

impl std::error::Error for Error {}
