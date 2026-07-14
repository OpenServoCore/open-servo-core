//! The transport seam: a reliable ordered byte pipe to one adapter (the
//! link contract in `osc_host::link::record`). Carrier chunking carries no
//! meaning -- records reassemble in the session.

use std::fmt;

pub trait Pipe {
    /// Deliver bytes in order, whole.
    fn send(&mut self, bytes: &[u8]) -> impl Future<Output = Result<(), PipeError>>;
    /// The next delivered chunk (never empty). Pends until bytes arrive.
    fn recv(&mut self) -> impl Future<Output = Result<Vec<u8>, PipeError>>;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PipeError {
    /// Transport failure (USB error, device gone).
    Io(String),
    /// The pipe produced nothing within the client's guard window -- a dead
    /// or wedged adapter, not a protocol timeout (the engine owns those).
    Stalled,
}

impl fmt::Display for PipeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PipeError::Io(m) => write!(f, "pipe: {m}"),
            PipeError::Stalled => write!(f, "pipe stalled past the guard window"),
        }
    }
}

impl std::error::Error for PipeError {}
