//! Overlay types. Every `#[repr(C)]` struct mirrors the on-the-wire
//! (unstuffed) layout at alignment 1, so a consumer can cast a `&[u8]`
//! accumulator into any of them at any offset.

mod entries;
mod header;
pub mod instruction;
mod slot;
mod status;

pub use entries::{
    BulkReadEntry, BulkWriteEntries, BulkWriteEntry, SyncWriteEntries, SyncWriteEntry,
};
pub use header::{Header, Id, U16Le};
pub use instruction::{
    ActionPacket, BulkReadHeader, BulkReadPacket, BulkWriteHeader, BulkWritePacket,
    FactoryResetPacket, FastBulkReadHeader, FastBulkReadPacket, FastSyncReadHeader,
    FastSyncReadPacket, Instruction, InstructionByte, PingPacket, RawHeader, RawPacket, ReadPacket,
    RebootPacket, SyncReadHeader, SyncReadPacket, SyncWriteHeader, SyncWritePacket, WriteHeader,
    WritePacket,
};
pub use slot::{BulkSlotInfo, FastSlotInfo, SlotPosition, SyncSlotInfo};
pub use status::{
    ErrorCode, FastBulkReadStatus, FastBulkSlotIter, FastSyncReadStatus, FastSyncSlotIter,
    PingStatus, RequestKind, Slot, Status, StatusError, StatusHeader, StatusPacket,
};

#[derive(Copy, Clone, Debug)]
pub enum Packet<'a> {
    Ping(&'a PingPacket),
    Read(&'a ReadPacket),
    Write(WritePacket<'a>),
    RegWrite(WritePacket<'a>),
    Action(&'a ActionPacket),
    Reboot(&'a RebootPacket),
    FactoryReset(&'a FactoryResetPacket),
    Status(StatusPacket<'a>),
    SyncRead(SyncReadPacket<'a>),
    SyncWrite(SyncWritePacket<'a>),
    BulkRead(BulkReadPacket<'a>),
    BulkWrite(BulkWritePacket<'a>),
    FastSyncRead(FastSyncReadPacket<'a>),
    FastBulkRead(FastBulkReadPacket<'a>),
    Raw(RawPacket<'a>),
}

/// Host-originated frames. Mirror of [`Packet`] minus `Status`, used at
/// service boundaries that explicitly only handle requests (the DXL bus
/// dispatcher surfaces these via `poll()`; observed peer Status frames feed
/// the driver-internal drift signal and stay out of this surface).
#[derive(Copy, Clone, Debug)]
pub enum InstructionPacket<'a> {
    Ping(&'a PingPacket),
    Read(&'a ReadPacket),
    Write(WritePacket<'a>),
    RegWrite(WritePacket<'a>),
    Action(&'a ActionPacket),
    Reboot(&'a RebootPacket),
    FactoryReset(&'a FactoryResetPacket),
    SyncRead(SyncReadPacket<'a>),
    SyncWrite(SyncWritePacket<'a>),
    BulkRead(BulkReadPacket<'a>),
    BulkWrite(BulkWritePacket<'a>),
    FastSyncRead(FastSyncReadPacket<'a>),
    FastBulkRead(FastBulkReadPacket<'a>),
    Raw(RawPacket<'a>),
}

impl<'a> InstructionPacket<'a> {
    /// Target ID byte. Lets the bus driver decide "is this addressed to us
    /// or BROADCAST" without re-walking every variant at the call site.
    pub fn id(&self) -> Id {
        match self {
            InstructionPacket::Ping(p) => p.header.id,
            InstructionPacket::Read(p) => p.header.id,
            InstructionPacket::Write(p) => p.header.header.id,
            InstructionPacket::RegWrite(p) => p.header.header.id,
            InstructionPacket::Action(p) => p.header.id,
            InstructionPacket::Reboot(p) => p.header.id,
            InstructionPacket::FactoryReset(p) => p.header.id,
            InstructionPacket::SyncRead(p) => p.header.header.id,
            InstructionPacket::SyncWrite(p) => p.header.header.id,
            InstructionPacket::BulkRead(p) => p.header.header.id,
            InstructionPacket::BulkWrite(p) => p.header.header.id,
            InstructionPacket::FastSyncRead(p) => p.header.header.id,
            InstructionPacket::FastBulkRead(p) => p.header.header.id,
            InstructionPacket::Raw(p) => p.header.header.id,
        }
    }
}

impl<'a> Packet<'a> {
    /// Project a decoded frame onto the host-originated subset. Returns
    /// `None` for `Status` so the caller's match never needs an
    /// unreachable arm.
    pub fn into_instruction_packet(self) -> Option<InstructionPacket<'a>> {
        match self {
            Packet::Ping(p) => Some(InstructionPacket::Ping(p)),
            Packet::Read(p) => Some(InstructionPacket::Read(p)),
            Packet::Write(p) => Some(InstructionPacket::Write(p)),
            Packet::RegWrite(p) => Some(InstructionPacket::RegWrite(p)),
            Packet::Action(p) => Some(InstructionPacket::Action(p)),
            Packet::Reboot(p) => Some(InstructionPacket::Reboot(p)),
            Packet::FactoryReset(p) => Some(InstructionPacket::FactoryReset(p)),
            Packet::Status(_) => None,
            Packet::SyncRead(p) => Some(InstructionPacket::SyncRead(p)),
            Packet::SyncWrite(p) => Some(InstructionPacket::SyncWrite(p)),
            Packet::BulkRead(p) => Some(InstructionPacket::BulkRead(p)),
            Packet::BulkWrite(p) => Some(InstructionPacket::BulkWrite(p)),
            Packet::FastSyncRead(p) => Some(InstructionPacket::FastSyncRead(p)),
            Packet::FastBulkRead(p) => Some(InstructionPacket::FastBulkRead(p)),
            Packet::Raw(p) => Some(InstructionPacket::Raw(p)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand-roll a Status overlay over a buffer that mirrors the on-wire
    /// shape; lets the projection test cover [`Packet::Status`] without a
    /// wire-byte producer in this module.
    fn status_packet(buf: &mut [u8]) -> Packet<'_> {
        buf[..4].copy_from_slice(&[0xFF, 0xFF, 0xFD, 0x00]);
        buf[4] = 0x05;
        buf[5] = 0x04;
        buf[6] = 0x00;
        buf[7] = Instruction::Status.as_u8();
        buf[8] = 0x00;
        // SAFETY: buf is at least 9 bytes; StatusHeader is #[repr(C)] align
        // 1 and the bytes we wrote match its field order exactly.
        let header: &StatusHeader = unsafe { &*(buf.as_ptr() as *const StatusHeader) };
        Packet::Status(StatusPacket {
            header,
            params: &buf[core::mem::size_of::<StatusHeader>()..],
        })
    }

    #[test]
    fn status_projects_to_none() {
        let mut buf = [0u8; 9];
        let pkt = status_packet(&mut buf);
        assert!(matches!(pkt, Packet::Status(_)));
        assert!(pkt.into_instruction_packet().is_none());
    }

    #[test]
    fn ping_projects_to_instruction_ping() {
        let mut buf = [0u8; 8];
        buf[..4].copy_from_slice(&[0xFF, 0xFF, 0xFD, 0x00]);
        buf[4] = 0x01;
        buf[5] = 0x03;
        buf[6] = 0x00;
        buf[7] = Instruction::Ping.as_u8();
        // SAFETY: PingPacket is #[repr(C)] align 1 covering the 8 bytes we wrote.
        let p: &PingPacket = unsafe { &*(buf.as_ptr() as *const PingPacket) };
        let pkt = Packet::Ping(p);
        match pkt.into_instruction_packet() {
            Some(InstructionPacket::Ping(p)) => assert_eq!(p.header.id, Id::new(0x01)),
            other => panic!("expected Ping, got {other:?}"),
        }
    }
}
