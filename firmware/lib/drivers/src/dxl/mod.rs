//! DXL-family drivers. One sibling module per wire — today only
//! [`uart`]; a future CAN-FD variant would land as a sibling `can`
//! module without rearranging the parent.

pub mod uart;
