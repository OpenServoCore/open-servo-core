//! Per-phase parser stages. Parent [`Parser`](super::Parser) orchestrates
//! transitions between them.

mod crc;
mod header;
mod payload;
mod slots;
mod sync;

pub(crate) use crc::CrcStage;
pub(crate) use header::HeaderStage;
pub(crate) use payload::{PayloadKind, PayloadStage};
pub(crate) use slots::SlotsStage;
pub(crate) use sync::SyncStage;
