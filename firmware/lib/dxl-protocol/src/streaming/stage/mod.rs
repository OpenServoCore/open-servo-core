//! Per-phase parser stages. Parent [`Parser`](super::Parser) orchestrates
//! transitions between them.

mod header;
mod sync;

pub(crate) use header::HeaderStage;
pub(crate) use sync::SyncStage;
