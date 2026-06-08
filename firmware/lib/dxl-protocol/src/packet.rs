//! Streaming-parser overlay types. Each `#[repr(C)]` struct mirrors the
//! on-the-wire (unstuffed) byte layout exactly, with alignment 1 — so the
//! decoder can cast a `&[u8]` accumulator into any of these structs at any
//! offset.

#![allow(dead_code)]

/// Byte-aligned little-endian u16. Two `u8` fields keep alignment at 1 so
/// every overlay struct that contains one stays alignment-1 and casts
/// soundly from any byte offset.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct U16Le {
    pub lo: u8,
    pub hi: u8,
}

impl U16Le {
    #[inline]
    pub const fn from_u16(v: u16) -> Self {
        let [lo, hi] = v.to_le_bytes();
        Self { lo, hi }
    }

    #[inline]
    pub const fn get(&self) -> u16 {
        u16::from_le_bytes([self.lo, self.hi])
    }

    #[inline]
    pub fn set(&mut self, v: u16) {
        let [lo, hi] = v.to_le_bytes();
        self.lo = lo;
        self.hi = hi;
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Header {
    pub sync: [u8; 4],
    pub id: u8,
    pub len: U16Le,
    pub instruction: u8,
}

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
pub struct StatusHeader {
    pub header: Header,
    pub error: u8,
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

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkReadEntry {
    pub id: u8,
    pub addr: U16Le,
    pub length: U16Le,
}

// ───── variable-length wrappers ─────

#[derive(Copy, Clone, Debug)]
pub struct WritePacket<'a> {
    pub header: &'a WriteHeader,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct StatusPacket<'a> {
    pub header: &'a StatusHeader,
    pub params: &'a [u8],
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
        SyncWriteEntries {
            cursor: self.body,
            data_len: self.header.length.get() as usize,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWriteEntry<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWriteEntries<'a> {
    cursor: &'a [u8],
    data_len: usize,
}

impl<'a> Iterator for SyncWriteEntries<'a> {
    type Item = SyncWriteEntry<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        let stride = 1 + self.data_len;
        if self.cursor.len() < stride {
            return None;
        }
        let (head, rest) = self.cursor.split_at(stride);
        self.cursor = rest;
        Some(SyncWriteEntry {
            id: head[0],
            data: &head[1..],
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWritePacket<'a> {
    pub header: &'a BulkWriteHeader,
    pub body: &'a [u8],
}

impl<'a> BulkWritePacket<'a> {
    pub fn entries(&self) -> BulkWriteEntries<'a> {
        BulkWriteEntries { cursor: self.body }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWriteEntry<'a> {
    pub id: u8,
    pub addr: u16,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWriteEntries<'a> {
    cursor: &'a [u8],
}

impl<'a> Iterator for BulkWriteEntries<'a> {
    type Item = BulkWriteEntry<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.cursor.len() < 5 {
            return None;
        }
        let id = self.cursor[0];
        let addr = u16::from_le_bytes([self.cursor[1], self.cursor[2]]);
        let length = u16::from_le_bytes([self.cursor[3], self.cursor[4]]) as usize;
        let total = 5 + length;
        if self.cursor.len() < total {
            return None;
        }
        let (head, rest) = self.cursor.split_at(total);
        self.cursor = rest;
        Some(BulkWriteEntry {
            id,
            addr,
            data: &head[5..],
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RawPacket<'a> {
    pub header: &'a RawHeader,
    pub params: &'a [u8],
}

// ───── status payload overlays ─────

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PingStatus {
    pub model: U16Le,
    pub fw_version: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ReadStatus<'a> {
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadStatus<'a> {
    pub error: u8,
    pub payload: &'a [u8],
}

impl<'a> FastSyncReadStatus<'a> {
    pub fn slots(&self, slot_length: u16) -> FastSyncSlotIter<'a> {
        FastSyncSlotIter {
            cursor: self.payload,
            slot_length: slot_length as usize,
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadStatus<'a> {
    pub error: u8,
    pub payload: &'a [u8],
}

impl<'a> FastBulkReadStatus<'a> {
    pub fn slots<L: IntoIterator<Item = u16>>(
        &self,
        lengths: L,
    ) -> FastBulkSlotIter<'a, L::IntoIter> {
        FastBulkSlotIter {
            cursor: self.payload,
            lengths: lengths.into_iter(),
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Slot<'a> {
    pub id: u8,
    pub error: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncSlotIter<'a> {
    cursor: &'a [u8],
    slot_length: usize,
    slot0_error: u8,
    slot_index: usize,
}

impl<'a> Iterator for FastSyncSlotIter<'a> {
    type Item = Slot<'a>;
    fn next(&mut self) -> Option<Slot<'a>> {
        next_slot(
            &mut self.cursor,
            self.slot_length,
            self.slot0_error,
            &mut self.slot_index,
        )
    }
}

pub struct FastBulkSlotIter<'a, L: Iterator<Item = u16>> {
    cursor: &'a [u8],
    lengths: L,
    slot0_error: u8,
    slot_index: usize,
}

impl<'a, L: Iterator<Item = u16>> Iterator for FastBulkSlotIter<'a, L> {
    type Item = Slot<'a>;
    fn next(&mut self) -> Option<Slot<'a>> {
        let len = self.lengths.next()? as usize;
        next_slot(
            &mut self.cursor,
            len,
            self.slot0_error,
            &mut self.slot_index,
        )
    }
}

fn next_slot<'a>(
    cursor: &mut &'a [u8],
    slot_length: usize,
    slot0_error: u8,
    slot_index: &mut usize,
) -> Option<Slot<'a>> {
    let (id, error) = if *slot_index == 0 {
        if cursor.is_empty() {
            return None;
        }
        let (id, rest) = cursor.split_first()?;
        *cursor = rest;
        (*id, slot0_error)
    } else {
        if cursor.len() < 2 {
            return None;
        }
        let error = cursor[0];
        let id = cursor[1];
        *cursor = &cursor[2..];
        (id, error)
    };
    if cursor.len() < slot_length {
        return None;
    }
    let (data, rest) = cursor.split_at(slot_length);
    *cursor = rest;
    *slot_index += 1;
    Some(Slot { id, error, data })
}

// ───── status-context dispatch ─────

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RequestKind {
    Ping,
    Read,
    Write,
    RegWrite,
    Action,
    Reboot,
    FactoryReset,
    SyncRead,
    SyncWrite,
    BulkRead,
    BulkWrite,
    FastSyncRead,
    FastBulkRead,
}

#[derive(Copy, Clone, Debug)]
pub enum Status<'a> {
    Empty,
    Ping(&'a PingStatus),
    Read(ReadStatus<'a>),
    FastSyncRead(FastSyncReadStatus<'a>),
    FastBulkRead(FastBulkReadStatus<'a>),
}

impl<'a> StatusPacket<'a> {
    #[inline]
    pub fn error(&self) -> u8 {
        self.header.error
    }

    pub fn interpret(self, req: RequestKind) -> Status<'a> {
        match req {
            RequestKind::Write
            | RequestKind::RegWrite
            | RequestKind::Action
            | RequestKind::Reboot
            | RequestKind::FactoryReset
            | RequestKind::SyncWrite
            | RequestKind::BulkWrite => Status::Empty,

            RequestKind::Ping => {
                if self.params.len() < core::mem::size_of::<PingStatus>() {
                    Status::Empty
                } else {
                    // SAFETY: PingStatus is repr(C), align 1, all u8/U16Le
                    // fields. Length checked above; `self.params` is
                    // initialized for its full length.
                    let p = unsafe { &*(self.params.as_ptr() as *const PingStatus) };
                    Status::Ping(p)
                }
            }

            RequestKind::Read | RequestKind::SyncRead | RequestKind::BulkRead => {
                Status::Read(ReadStatus { data: self.params })
            }

            RequestKind::FastSyncRead => Status::FastSyncRead(FastSyncReadStatus {
                error: self.header.error,
                payload: self.params,
            }),

            RequestKind::FastBulkRead => Status::FastBulkRead(FastBulkReadStatus {
                error: self.header.error,
                payload: self.params,
            }),
        }
    }
}

// ───── top-level decode enum ─────

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instruction::*;
    use core::mem::{align_of, offset_of, size_of};

    // ─── alignment: every overlay struct must be align 1 so the decoder
    //     can cast its `[MaybeUninit<u8>; M]` accumulator at any offset. ───

    #[test]
    fn alignment_is_one_everywhere() {
        assert_eq!(align_of::<U16Le>(), 1);
        assert_eq!(align_of::<Header>(), 1);
        assert_eq!(align_of::<PingPacket>(), 1);
        assert_eq!(align_of::<ReadPacket>(), 1);
        assert_eq!(align_of::<WriteHeader>(), 1);
        assert_eq!(align_of::<ActionPacket>(), 1);
        assert_eq!(align_of::<RebootPacket>(), 1);
        assert_eq!(align_of::<FactoryResetPacket>(), 1);
        assert_eq!(align_of::<StatusHeader>(), 1);
        assert_eq!(align_of::<SyncReadHeader>(), 1);
        assert_eq!(align_of::<SyncWriteHeader>(), 1);
        assert_eq!(align_of::<BulkReadHeader>(), 1);
        assert_eq!(align_of::<BulkWriteHeader>(), 1);
        assert_eq!(align_of::<FastSyncReadHeader>(), 1);
        assert_eq!(align_of::<FastBulkReadHeader>(), 1);
        assert_eq!(align_of::<RawHeader>(), 1);
        assert_eq!(align_of::<BulkReadEntry>(), 1);
        assert_eq!(align_of::<PingStatus>(), 1);
    }

    // ─── sizes match the on-the-wire (unstuffed) byte layout. ───

    #[test]
    fn sizes_match_wire() {
        assert_eq!(size_of::<U16Le>(), 2);
        assert_eq!(size_of::<Header>(), 8);
        assert_eq!(size_of::<PingPacket>(), 8);
        assert_eq!(size_of::<ReadPacket>(), 12);
        assert_eq!(size_of::<WriteHeader>(), 10);
        assert_eq!(size_of::<ActionPacket>(), 8);
        assert_eq!(size_of::<RebootPacket>(), 8);
        assert_eq!(size_of::<FactoryResetPacket>(), 9);
        assert_eq!(size_of::<StatusHeader>(), 9);
        assert_eq!(size_of::<SyncReadHeader>(), 12);
        assert_eq!(size_of::<SyncWriteHeader>(), 12);
        assert_eq!(size_of::<BulkReadHeader>(), 8);
        assert_eq!(size_of::<BulkWriteHeader>(), 8);
        assert_eq!(size_of::<FastSyncReadHeader>(), 12);
        assert_eq!(size_of::<FastBulkReadHeader>(), 8);
        assert_eq!(size_of::<RawHeader>(), 8);
        assert_eq!(size_of::<BulkReadEntry>(), 5);
        assert_eq!(size_of::<PingStatus>(), 3);
    }

    // ─── field offsets match the on-the-wire byte positions. ───

    #[test]
    fn header_offsets() {
        assert_eq!(offset_of!(Header, sync), 0);
        assert_eq!(offset_of!(Header, id), 4);
        assert_eq!(offset_of!(Header, len), 5);
        assert_eq!(offset_of!(Header, instruction), 7);
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
    fn status_header_offsets() {
        assert_eq!(offset_of!(StatusHeader, header), 0);
        assert_eq!(offset_of!(StatusHeader, error), 8);
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
    fn bulk_read_entry_offsets() {
        assert_eq!(offset_of!(BulkReadEntry, id), 0);
        assert_eq!(offset_of!(BulkReadEntry, addr), 1);
        assert_eq!(offset_of!(BulkReadEntry, length), 3);
    }

    #[test]
    fn ping_status_offsets() {
        assert_eq!(offset_of!(PingStatus, model), 0);
        assert_eq!(offset_of!(PingStatus, fw_version), 2);
    }

    // ─── U16Le round-trip. ───

    #[test]
    fn u16le_round_trip() {
        let v = U16Le::from_u16(0x1234);
        assert_eq!(v.lo, 0x34);
        assert_eq!(v.hi, 0x12);
        assert_eq!(v.get(), 0x1234);
    }

    // ─── overlay-onto-bytes round-trip: cast a hand-built byte array into
    //     each overlay struct and confirm the fields match. ───

    #[test]
    fn read_packet_overlay() {
        let bytes = [
            0xFF, 0xFF, 0xFD, 0x00, // sync
            0x07, // id
            0x07, 0x00,       // len = 7
            INSTR_READ, // instruction
            0x84, 0x00, // addr = 0x0084
            0x04, 0x00, // length = 4
        ];
        let p: &ReadPacket = unsafe { &*(bytes.as_ptr() as *const ReadPacket) };
        assert_eq!(p.header.sync, [0xFF, 0xFF, 0xFD, 0x00]);
        assert_eq!(p.header.id, 0x07);
        assert_eq!(p.header.len.get(), 7);
        assert_eq!(p.header.instruction, INSTR_READ);
        assert_eq!(p.addr.get(), 0x0084);
        assert_eq!(p.length.get(), 4);
    }

    #[test]
    fn bulk_read_entries_overlay() {
        let body: [u8; 10] = [
            0x01, 0x10, 0x00, 0x04, 0x00, // id=1 addr=0x0010 len=4
            0x02, 0x20, 0x00, 0x08, 0x00, // id=2 addr=0x0020 len=8
        ];
        let entries: &[BulkReadEntry] =
            unsafe { core::slice::from_raw_parts(body.as_ptr() as *const BulkReadEntry, 2) };
        assert_eq!(entries[0].id, 1);
        assert_eq!(entries[0].addr.get(), 0x0010);
        assert_eq!(entries[0].length.get(), 4);
        assert_eq!(entries[1].id, 2);
        assert_eq!(entries[1].addr.get(), 0x0020);
        assert_eq!(entries[1].length.get(), 8);
    }

    // ─── variable-stride iterators. ───

    fn make_sync_write(buf: &[u8]) -> SyncWritePacket<'_> {
        let header: &SyncWriteHeader = unsafe { &*(buf.as_ptr() as *const SyncWriteHeader) };
        SyncWritePacket {
            header,
            body: &buf[size_of::<SyncWriteHeader>()..],
        }
    }

    #[test]
    fn sync_write_entries_walks_full_entries() {
        // Header: sync + id + len(=12) + instr + addr(=0x50) + length(=3).
        // Body: two stride-4 entries [id, d0, d1, d2].
        let buf: [u8; 12 + 8] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x0C,
            0x00,
            INSTR_SYNC_WRITE,
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
        assert_eq!(e0.id, 1);
        assert_eq!(e0.data, &[10, 11, 12]);
        let e1 = it.next().unwrap();
        assert_eq!(e1.id, 2);
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
            INSTR_SYNC_WRITE,
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

    fn make_bulk_write<'a>(buf: &'a [u8]) -> BulkWritePacket<'a> {
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
            INSTR_BULK_WRITE, //
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
        assert_eq!(e0.id, 1);
        assert_eq!(e0.addr, 0x0050);
        assert_eq!(e0.data, &[0xAA, 0xBB, 0xCC]);
        let e1 = it.next().unwrap();
        assert_eq!(e1.id, 2);
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
            INSTR_BULK_WRITE, //
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

    // ─── Fast Sync/Bulk Read status slot iteration. ───

    #[test]
    fn fast_sync_status_slots_walk_uniform_length() {
        // 3 slots, length=4. payload = id0 + data0 + err1 + id1 + data1 + err2 + id2 + data2
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, //
            0x02, 20, 0xB0, 0xB1, 0xB2, 0xB3, //
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3,
        ];
        let s = FastSyncReadStatus {
            error: 0x01,
            payload: &payload,
        };
        let collected: heapless::Vec<(u8, u8, &[u8]), 4> =
            s.slots(4).map(|sl| (sl.id, sl.error, sl.data)).collect();
        assert_eq!(collected.len(), 3);
        assert_eq!(collected[0], (10, 0x01, &[0xA0, 0xA1, 0xA2, 0xA3][..]));
        assert_eq!(collected[1], (20, 0x02, &[0xB0, 0xB1, 0xB2, 0xB3][..]));
        assert_eq!(collected[2], (30, 0x00, &[0xC0, 0xC1, 0xC2, 0xC3][..]));
    }

    #[test]
    fn fast_sync_status_slots_stop_on_truncation() {
        let payload = [10, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 20, 0xB0]; // slot 1 missing 3 bytes
        let s = FastSyncReadStatus {
            error: 0x00,
            payload: &payload,
        };
        assert_eq!(s.slots(4).count(), 1);
    }

    #[test]
    fn fast_bulk_status_slots_varying_lengths() {
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, //
            0x02, 20, 0xB0, 0xB1, //
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,
        ];
        let s = FastBulkReadStatus {
            error: 0x05,
            payload: &payload,
        };
        let collected: heapless::Vec<(u8, u8, usize), 4> = s
            .slots([4u16, 2, 6].iter().copied())
            .map(|sl| (sl.id, sl.error, sl.data.len()))
            .collect();
        assert_eq!(collected.len(), 3);
        assert_eq!(collected[0], (10, 0x05, 4));
        assert_eq!(collected[1], (20, 0x02, 2));
        assert_eq!(collected[2], (30, 0x00, 6));
    }

    // ─── interpret() dispatches every RequestKind to the right Status variant. ───

    fn make_status<'a>(buf: &'a [u8]) -> StatusPacket<'a> {
        let header: &StatusHeader = unsafe { &*(buf.as_ptr() as *const StatusHeader) };
        StatusPacket {
            header,
            params: &buf[size_of::<StatusHeader>()..],
        }
    }

    #[test]
    fn interpret_ping_decodes_overlay() {
        let buf: [u8; 9 + 3] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x06,
            0x00,
            INSTR_STATUS,
            0x00, //
            0xFC,
            0x03,
            0x2A, // model=1020, fw=0x2A
        ];
        let s = make_status(&buf);
        assert_eq!(s.error(), 0x00);
        match s.interpret(RequestKind::Ping) {
            Status::Ping(p) => {
                assert_eq!(p.model.get(), 1020);
                assert_eq!(p.fw_version, 0x2A);
            }
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn interpret_ping_short_payload_falls_back_to_empty() {
        let buf: [u8; 9 + 1] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x04,
            0x00,
            INSTR_STATUS,
            0x00,
            0xFC,
        ];
        let s = make_status(&buf);
        assert!(matches!(s.interpret(RequestKind::Ping), Status::Empty));
    }

    #[test]
    fn interpret_read_wraps_payload_slice() {
        let buf: [u8; 9 + 4] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x07,
            0x00,
            INSTR_STATUS,
            0x00, //
            0x01,
            0x02,
            0x03,
            0x04,
        ];
        let s = make_status(&buf);
        for req in [
            RequestKind::Read,
            RequestKind::SyncRead,
            RequestKind::BulkRead,
        ] {
            match s.interpret(req) {
                Status::Read(r) => assert_eq!(r.data, &[0x01, 0x02, 0x03, 0x04]),
                other => panic!("{req:?} expected Read, got {other:?}"),
            }
        }
    }

    #[test]
    fn interpret_write_family_is_empty() {
        let buf: [u8; 9] = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, INSTR_STATUS, 0x00];
        let s = make_status(&buf);
        for req in [
            RequestKind::Write,
            RequestKind::RegWrite,
            RequestKind::Action,
            RequestKind::Reboot,
            RequestKind::FactoryReset,
            RequestKind::SyncWrite,
            RequestKind::BulkWrite,
        ] {
            assert!(
                matches!(s.interpret(req), Status::Empty),
                "{req:?} should produce Status::Empty"
            );
        }
    }

    #[test]
    fn interpret_fast_sync_carries_error_and_payload() {
        let buf: [u8; 9 + 5] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x08,
            0x00,
            INSTR_STATUS,
            0x07, //
            10,
            0xAA,
            0xBB,
            0xCC,
            0xDD,
        ];
        let s = make_status(&buf);
        match s.interpret(RequestKind::FastSyncRead) {
            Status::FastSyncRead(f) => {
                assert_eq!(f.error, 0x07);
                assert_eq!(f.payload, &[10, 0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected FastSyncRead, got {other:?}"),
        }
    }

    #[test]
    fn interpret_fast_bulk_carries_error_and_payload() {
        let buf: [u8; 9 + 5] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x08,
            0x00,
            INSTR_STATUS,
            0x07, //
            10,
            0xAA,
            0xBB,
            0xCC,
            0xDD,
        ];
        let s = make_status(&buf);
        match s.interpret(RequestKind::FastBulkRead) {
            Status::FastBulkRead(f) => {
                assert_eq!(f.error, 0x07);
                assert_eq!(f.payload, &[10, 0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected FastBulkRead, got {other:?}"),
        }
    }
}
