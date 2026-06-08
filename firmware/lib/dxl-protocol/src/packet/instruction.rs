//! Instruction byte + per-instruction request packet overlays.

#![allow(dead_code)]

use super::entries::{BulkReadEntry, BulkWriteEntries, SyncWriteEntries};
use super::header::{Header, U16Le};

/// DXL 2.0 instruction byte. `Ext(b)` carries any non-standard byte --
/// pattern-match on it to dispatch chip-specific extensions.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Instruction {
    Ping,
    Read,
    Write,
    RegWrite,
    Action,
    FactoryReset,
    Reboot,
    Clear,
    ControlTableBackup,
    Status,
    SyncRead,
    SyncWrite,
    FastSyncRead,
    BulkRead,
    BulkWrite,
    FastBulkRead,
    Ext(u8),
}

impl Instruction {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Ping => 0x01,
            Self::Read => 0x02,
            Self::Write => 0x03,
            Self::RegWrite => 0x04,
            Self::Action => 0x05,
            Self::FactoryReset => 0x06,
            Self::Reboot => 0x08,
            Self::Clear => 0x10,
            Self::ControlTableBackup => 0x20,
            Self::Status => 0x55,
            Self::SyncRead => 0x82,
            Self::SyncWrite => 0x83,
            Self::FastSyncRead => 0x8A,
            Self::BulkRead => 0x92,
            Self::BulkWrite => 0x93,
            Self::FastBulkRead => 0x9A,
            Self::Ext(b) => b,
        }
    }

    pub const fn from_u8(b: u8) -> Self {
        match b {
            0x01 => Self::Ping,
            0x02 => Self::Read,
            0x03 => Self::Write,
            0x04 => Self::RegWrite,
            0x05 => Self::Action,
            0x06 => Self::FactoryReset,
            0x08 => Self::Reboot,
            0x10 => Self::Clear,
            0x20 => Self::ControlTableBackup,
            0x55 => Self::Status,
            0x82 => Self::SyncRead,
            0x83 => Self::SyncWrite,
            0x8A => Self::FastSyncRead,
            0x92 => Self::BulkRead,
            0x93 => Self::BulkWrite,
            0x9A => Self::FastBulkRead,
            _ => Self::Ext(b),
        }
    }
}

/// 1-byte wire overlay for the instruction field. Round-trips any byte;
/// non-standard bytes surface as [`Instruction::Ext`] via [`kind`](Self::kind).
#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct InstructionByte(pub u8);

impl InstructionByte {
    pub const fn from_byte(b: u8) -> Self {
        Self(b)
    }
    pub const fn from_instruction(i: Instruction) -> Self {
        Self(i.as_u8())
    }
    pub const fn as_byte(self) -> u8 {
        self.0
    }
    pub const fn kind(self) -> Instruction {
        Instruction::from_u8(self.0)
    }
}

// ----- fixed-shape request overlays -----

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PingPacket {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ReadPacket {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct WriteHeader {
    pub header: Header,
    pub addr: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ActionPacket {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct RebootPacket {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FactoryResetPacket {
    pub header: Header,
    pub mode: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct SyncReadHeader {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct SyncWriteHeader {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkReadHeader {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkWriteHeader {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadHeader {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadHeader {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct RawHeader {
    pub header: Header,
}

// ----- variable-length request wrappers -----

#[derive(Copy, Clone, Debug)]
pub struct WritePacket<'a> {
    pub header: &'a WriteHeader,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncReadPacket<'a> {
    pub header: &'a SyncReadHeader,
    pub ids: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadPacket<'a> {
    pub header: &'a FastSyncReadHeader,
    pub ids: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct BulkReadPacket<'a> {
    pub header: &'a BulkReadHeader,
    pub entries: &'a [BulkReadEntry],
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadPacket<'a> {
    pub header: &'a FastBulkReadHeader,
    pub entries: &'a [BulkReadEntry],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWritePacket<'a> {
    pub header: &'a SyncWriteHeader,
    pub body: &'a [u8],
}

impl<'a> SyncWritePacket<'a> {
    pub fn entries(&self) -> SyncWriteEntries<'a> {
        SyncWriteEntries::new(self.body, self.header.length.get() as usize)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWritePacket<'a> {
    pub header: &'a BulkWriteHeader,
    pub body: &'a [u8],
}

impl<'a> BulkWritePacket<'a> {
    pub fn entries(&self) -> BulkWriteEntries<'a> {
        BulkWriteEntries::new(self.body)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RawPacket<'a> {
    pub header: &'a RawHeader,
    pub params: &'a [u8],
}

#[cfg(test)]
mod tests {
    use super::super::header::Id;
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    #[test]
    fn alignment_is_one_everywhere() {
        assert_eq!(align_of::<PingPacket>(), 1);
        assert_eq!(align_of::<ReadPacket>(), 1);
        assert_eq!(align_of::<WriteHeader>(), 1);
        assert_eq!(align_of::<ActionPacket>(), 1);
        assert_eq!(align_of::<RebootPacket>(), 1);
        assert_eq!(align_of::<FactoryResetPacket>(), 1);
        assert_eq!(align_of::<SyncReadHeader>(), 1);
        assert_eq!(align_of::<SyncWriteHeader>(), 1);
        assert_eq!(align_of::<BulkReadHeader>(), 1);
        assert_eq!(align_of::<BulkWriteHeader>(), 1);
        assert_eq!(align_of::<FastSyncReadHeader>(), 1);
        assert_eq!(align_of::<FastBulkReadHeader>(), 1);
        assert_eq!(align_of::<RawHeader>(), 1);
    }

    #[test]
    fn sizes_match_wire() {
        assert_eq!(size_of::<PingPacket>(), 8);
        assert_eq!(size_of::<ReadPacket>(), 12);
        assert_eq!(size_of::<WriteHeader>(), 10);
        assert_eq!(size_of::<ActionPacket>(), 8);
        assert_eq!(size_of::<RebootPacket>(), 8);
        assert_eq!(size_of::<FactoryResetPacket>(), 9);
        assert_eq!(size_of::<SyncReadHeader>(), 12);
        assert_eq!(size_of::<SyncWriteHeader>(), 12);
        assert_eq!(size_of::<BulkReadHeader>(), 8);
        assert_eq!(size_of::<BulkWriteHeader>(), 8);
        assert_eq!(size_of::<FastSyncReadHeader>(), 12);
        assert_eq!(size_of::<FastBulkReadHeader>(), 8);
        assert_eq!(size_of::<RawHeader>(), 8);
    }

    #[test]
    fn read_packet_offsets() {
        assert_eq!(offset_of!(ReadPacket, header), 0);
        assert_eq!(offset_of!(ReadPacket, addr), 8);
        assert_eq!(offset_of!(ReadPacket, length), 10);
    }

    #[test]
    fn write_header_offsets() {
        assert_eq!(offset_of!(WriteHeader, header), 0);
        assert_eq!(offset_of!(WriteHeader, addr), 8);
    }

    #[test]
    fn factory_reset_offsets() {
        assert_eq!(offset_of!(FactoryResetPacket, header), 0);
        assert_eq!(offset_of!(FactoryResetPacket, mode), 8);
    }

    #[test]
    fn sync_read_header_offsets() {
        assert_eq!(offset_of!(SyncReadHeader, header), 0);
        assert_eq!(offset_of!(SyncReadHeader, addr), 8);
        assert_eq!(offset_of!(SyncReadHeader, length), 10);
    }

    #[test]
    fn sync_write_header_offsets() {
        assert_eq!(offset_of!(SyncWriteHeader, header), 0);
        assert_eq!(offset_of!(SyncWriteHeader, addr), 8);
        assert_eq!(offset_of!(SyncWriteHeader, length), 10);
    }

    #[test]
    fn fast_sync_read_header_offsets() {
        assert_eq!(offset_of!(FastSyncReadHeader, header), 0);
        assert_eq!(offset_of!(FastSyncReadHeader, addr), 8);
        assert_eq!(offset_of!(FastSyncReadHeader, length), 10);
    }

    #[test]
    fn read_packet_overlay() {
        let bytes = [
            0xFF,
            0xFF,
            0xFD,
            0x00, // sync
            0x07, // id
            0x07,
            0x00,                      // len = 7
            Instruction::Read.as_u8(), // instruction
            0x84,
            0x00, // addr = 0x0084
            0x04,
            0x00, // length = 4
        ];
        let p: &ReadPacket = unsafe { &*(bytes.as_ptr() as *const ReadPacket) };
        assert_eq!(p.header.sync, [0xFF, 0xFF, 0xFD, 0x00]);
        assert_eq!(p.header.id, Id::new(0x07));
        assert_eq!(p.header.len.get(), 7);
        assert_eq!(p.header.instruction.kind(), Instruction::Read);
        assert_eq!(p.addr.get(), 0x0084);
        assert_eq!(p.length.get(), 4);
    }

    fn make_sync_write(buf: &[u8]) -> SyncWritePacket<'_> {
        let header: &SyncWriteHeader = unsafe { &*(buf.as_ptr() as *const SyncWriteHeader) };
        SyncWritePacket {
            header,
            body: &buf[size_of::<SyncWriteHeader>()..],
        }
    }

    #[test]
    fn sync_write_entries_walks_full_entries() {
        let buf: [u8; 12 + 8] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x0C,
            0x00,
            Instruction::SyncWrite.as_u8(),
            0x50,
            0x00,
            0x03,
            0x00,
            1,
            10,
            11,
            12, //
            2,
            20,
            21,
            22,
        ];
        let p = make_sync_write(&buf);
        let mut it = p.entries();
        let e0 = it.next().unwrap();
        assert_eq!(e0.id, Id::new(1));
        assert_eq!(e0.data, &[10, 11, 12]);
        let e1 = it.next().unwrap();
        assert_eq!(e1.id, Id::new(2));
        assert_eq!(e1.data, &[20, 21, 22]);
        assert!(it.next().is_none());
    }

    #[test]
    fn sync_write_entries_drops_trailing_partial() {
        let buf: [u8; 12 + 6] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x0C,
            0x00,
            Instruction::SyncWrite.as_u8(),
            0x50,
            0x00,
            0x03,
            0x00,
            1,
            10,
            11,
            12, //
            2,
            20, // truncated
        ];
        let p = make_sync_write(&buf);
        assert_eq!(p.entries().count(), 1);
    }

    fn make_bulk_write(buf: &[u8]) -> BulkWritePacket<'_> {
        let header: &BulkWriteHeader = unsafe { &*(buf.as_ptr() as *const BulkWriteHeader) };
        BulkWritePacket {
            header,
            body: &buf[size_of::<BulkWriteHeader>()..],
        }
    }

    #[test]
    fn bulk_write_entries_walks_varying_lengths() {
        let buf: [u8; 8 + 17] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x11,
            0x00,
            Instruction::BulkWrite.as_u8(), //
            1,
            0x50,
            0x00,
            0x03,
            0x00,
            0xAA,
            0xBB,
            0xCC, //
            2,
            0x54,
            0x02,
            0x04,
            0x00,
            0x10,
            0x20,
            0x30,
            0x40,
        ];
        let p = make_bulk_write(&buf);
        let mut it = p.entries();
        let e0 = it.next().unwrap();
        assert_eq!(e0.id, Id::new(1));
        assert_eq!(e0.addr, 0x0050);
        assert_eq!(e0.data, &[0xAA, 0xBB, 0xCC]);
        let e1 = it.next().unwrap();
        assert_eq!(e1.id, Id::new(2));
        assert_eq!(e1.addr, 0x0254);
        assert_eq!(e1.data, &[0x10, 0x20, 0x30, 0x40]);
        assert!(it.next().is_none());
    }

    #[test]
    fn bulk_write_entries_drops_truncated_data() {
        let buf: [u8; 8 + 7] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x07,
            0x00,
            Instruction::BulkWrite.as_u8(), //
            3,
            0x00,
            0x01,
            0x04,
            0x00,
            0xAA,
            0xBB, // claims 4, has 2
        ];
        let p = make_bulk_write(&buf);
        assert!(p.entries().next().is_none());
    }
}
