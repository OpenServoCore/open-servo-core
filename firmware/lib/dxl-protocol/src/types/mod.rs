//! DXL 2.0 wire-protocol value types: addressing primitives, instruction
//! byte, status error byte, and the slave-originated reply shapes consumed
//! by the encoders.

mod id;
mod instruction;
mod reply;
mod status;

pub use id::Id;
pub use instruction::{BulkReadEntry, Instruction};
pub use reply::{PingStatus, Slot, SlotPosition, Status};
pub use status::{ErrorCode, StatusError};
