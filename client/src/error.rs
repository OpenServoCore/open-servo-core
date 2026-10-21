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
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkError {
    /// The adapter answered ERROR (link sec 3.2).
    Rejected(RejectReason),
    /// A record the session cannot reconcile (wrong seq or kind, a record
    /// for nothing outstanding): an adapter-invariant breach on a reliable
    /// pipe. The session is poisoned; reconnect.
    Desync(String),
    /// A record that does not parse at all.
    Malformed,
}

/// An ERROR code (link sec 3.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectReason {
    Malformed,
    UnknownKind,
    Unsupported,
    WrongMode,
    Busy,
    BadRate,
    BadId,
    BadInst,
    BadPayload,
    TooLong,
    /// A request's `len` broke the limits; the adapter ignores the pipe
    /// until a new session opens.
    Framing,
    Other(u8),
}

impl RejectReason {
    pub fn from_byte(b: u8) -> Self {
        use osc_host::link::record as rec;
        match b {
            rec::ERR_MALFORMED => RejectReason::Malformed,
            rec::ERR_UNKNOWN_KIND => RejectReason::UnknownKind,
            rec::ERR_UNSUPPORTED => RejectReason::Unsupported,
            rec::ERR_WRONG_MODE => RejectReason::WrongMode,
            rec::ERR_BUSY => RejectReason::Busy,
            rec::ERR_BAD_RATE => RejectReason::BadRate,
            rec::ERR_BAD_ID => RejectReason::BadId,
            rec::ERR_BAD_INST => RejectReason::BadInst,
            rec::ERR_BAD_PAYLOAD => RejectReason::BadPayload,
            rec::ERR_TOO_LONG => RejectReason::TooLong,
            rec::ERR_FRAMING => RejectReason::Framing,
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
