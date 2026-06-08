//! Wire-position math for multi-slot Sync/Bulk Read requests: given an `id`,
//! find which slot in the wire layout it occupies and how many response bytes
//! precede it in the coalesced reply. Math is identical whether the caller
//! is the addressed slave (composing its own slot) or a master/sniffer
//! (decoding a captured response).

#![allow(dead_code)]

use crate::constants::{
    CRC_BYTES, FAST_RESPONSE_SLOT0_BYTES, FAST_RESPONSE_SLOT_BYTES, RESPONSE_HEADER_BYTES,
};

use super::entries::{BulkWriteEntry, SyncWriteEntry};
use super::instruction::{
    BulkReadPacket, BulkWritePacket, FastBulkReadPacket, FastSyncReadPacket, SyncReadPacket,
    SyncWritePacket,
};

/// Position of our slot in the coalesced Fast Status response. Variants that
/// emit the response header carry the DXL `Length` field for the whole
/// multi-slot frame; `Last` carries the chain CRC.
///
/// Chain producers (slaves emitting `Last` in a multi-slave coalesced reply)
/// don't know the running CRC at frame-build time — it depends on the wire
/// bytes of every prior slave. They emit a placeholder value here and let
/// the chip's fire-time ISR overwrite the trailing two bytes with the real
/// CRC. Callers that *do* know the CRC at build time (single-slot tests,
/// replay tools, sniffers) pass the real value.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SlotPosition {
    /// Single-slot response — emits full header and locally-computed CRC.
    Only { packet_length: u16 },
    /// First of N — emits header + body, no CRC (successors continue).
    First { packet_length: u16 },
    /// Body only.
    Middle,
    /// Body + caller-supplied CRC bytes.
    Last { crc: u16 },
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SyncSlotInfo {
    pub index: usize,
    pub bytes_before: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BulkSlotInfo {
    pub index: usize,
    pub address: u16,
    pub length: u16,
    pub bytes_before: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FastSlotInfo {
    pub our_slot: usize,
    pub n_slots: usize,
    pub address: u16,
    pub length: u16,
    pub packet_length: u16,
    pub bytes_before: u32,
}

impl FastSlotInfo {
    pub const fn position(&self) -> SlotPosition {
        match (self.our_slot, self.n_slots) {
            (0, 1) => SlotPosition::Only {
                packet_length: self.packet_length,
            },
            (0, _) => SlotPosition::First {
                packet_length: self.packet_length,
            },
            (k, n) if k + 1 == n => SlotPosition::Last { crc: 0 },
            _ => SlotPosition::Middle,
        }
    }
}

impl<'a> SyncReadPacket<'a> {
    pub fn find_slot(&self, id: u8) -> Option<SyncSlotInfo> {
        let index = self.ids.iter().position(|b| *b == id)?;
        let length = self.header.length.get();
        let per_slot = (RESPONSE_HEADER_BYTES as u32)
            .saturating_add(length as u32)
            .saturating_add(CRC_BYTES as u32);
        Some(SyncSlotInfo {
            index,
            bytes_before: per_slot.saturating_mul(index as u32),
        })
    }
}

impl<'a> BulkReadPacket<'a> {
    pub fn find_slot(&self, id: u8) -> Option<BulkSlotInfo> {
        let mut bytes_before = 0u32;
        for (index, entry) in self.entries.iter().enumerate() {
            let length = entry.length.get();
            if entry.id == id {
                return Some(BulkSlotInfo {
                    index,
                    address: entry.addr.get(),
                    length,
                    bytes_before,
                });
            }
            bytes_before = bytes_before
                .saturating_add(RESPONSE_HEADER_BYTES as u32)
                .saturating_add(length as u32)
                .saturating_add(CRC_BYTES as u32);
        }
        None
    }
}

impl<'a> FastSyncReadPacket<'a> {
    /// `max_slots` bounds how many ids the walk inspects; callers with a
    /// fixed slot budget pass it in so a malformed request can't unbalance
    /// downstream loops. Returns `None` if our id isn't in the list or the
    /// list exceeds `max_slots`.
    pub fn find_slot(&self, id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let length = self.header.length.get();
        let mut our_slot = None;
        let mut n_slots = 0usize;
        for (i, &slot_id) in self.ids.iter().enumerate() {
            if i >= max_slots {
                return None;
            }
            if slot_id == id && our_slot.is_none() {
                our_slot = Some(i);
            }
            n_slots = i + 1;
        }
        let our_slot = our_slot?;
        let payload = length as u32;
        let bytes_before = if our_slot == 0 {
            0
        } else {
            FAST_RESPONSE_SLOT0_BYTES as u32
                + payload
                + (our_slot as u32 - 1) * (FAST_RESPONSE_SLOT_BYTES as u32 + payload)
        };
        let packet_length = 3u16 + (n_slots as u16) * (2 + length);
        Some(FastSlotInfo {
            our_slot,
            n_slots,
            address: self.header.addr.get(),
            length,
            packet_length,
            bytes_before,
        })
    }
}

impl<'a> FastBulkReadPacket<'a> {
    /// `max_slots` bounds how many entries the walk inspects.
    pub fn find_slot(&self, id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let mut found: Option<(usize, u16, u16, u32)> = None;
        let mut n_slots = 0usize;
        let mut total_payload = 0u32;
        let mut bytes_before = 0u32;
        for (i, entry) in self.entries.iter().enumerate() {
            if i >= max_slots {
                return None;
            }
            let entry_len = entry.length.get();
            if entry.id == id && found.is_none() {
                found = Some((i, entry.addr.get(), entry_len, bytes_before));
            }
            let prefix = if i == 0 {
                FAST_RESPONSE_SLOT0_BYTES as u32
            } else {
                FAST_RESPONSE_SLOT_BYTES as u32
            };
            bytes_before = bytes_before
                .saturating_add(prefix)
                .saturating_add(entry_len as u32);
            total_payload = total_payload.saturating_add(entry_len as u32);
            n_slots = i + 1;
        }
        let (our_slot, address, length, bytes_before) = found?;
        let packet_length = (3u32 + (n_slots as u32) * 2 + total_payload) as u16;
        Some(FastSlotInfo {
            our_slot,
            n_slots,
            address,
            length,
            packet_length,
            bytes_before,
        })
    }
}

impl<'a> SyncWritePacket<'a> {
    /// Returns the typed entry for `id`, or `None` if not present.
    pub fn find_entry(&self, id: u8) -> Option<SyncWriteEntry<'a>> {
        self.entries().find(|e| e.id == id)
    }
}

impl<'a> BulkWritePacket<'a> {
    /// Returns the typed entry for `id`, or `None` if not present.
    pub fn find_entry(&self, id: u8) -> Option<BulkWriteEntry<'a>> {
        self.entries().find(|e| e.id == id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::entries::BulkReadEntry;
    use crate::packet::instruction::{
        BulkReadHeader, FastBulkReadHeader, FastSyncReadHeader, Instruction, SyncReadHeader,
        SyncWriteHeader,
    };
    use core::mem::size_of;

    fn make_sync_read(buf: &[u8]) -> SyncReadPacket<'_> {
        let header: &SyncReadHeader = unsafe { &*(buf.as_ptr() as *const SyncReadHeader) };
        SyncReadPacket {
            header,
            ids: &buf[size_of::<SyncReadHeader>()..],
        }
    }

    fn make_bulk_read(buf: &[u8], n_entries: usize) -> BulkReadPacket<'_> {
        let header: &BulkReadHeader = unsafe { &*(buf.as_ptr() as *const BulkReadHeader) };
        let entries: &[BulkReadEntry] = unsafe {
            core::slice::from_raw_parts(
                buf.as_ptr().add(size_of::<BulkReadHeader>()) as *const BulkReadEntry,
                n_entries,
            )
        };
        BulkReadPacket { header, entries }
    }

    fn make_fast_sync_read(buf: &[u8]) -> FastSyncReadPacket<'_> {
        let header: &FastSyncReadHeader = unsafe { &*(buf.as_ptr() as *const FastSyncReadHeader) };
        FastSyncReadPacket {
            header,
            ids: &buf[size_of::<FastSyncReadHeader>()..],
        }
    }

    fn make_fast_bulk_read(buf: &[u8], n_entries: usize) -> FastBulkReadPacket<'_> {
        let header: &FastBulkReadHeader = unsafe { &*(buf.as_ptr() as *const FastBulkReadHeader) };
        let entries: &[BulkReadEntry] = unsafe {
            core::slice::from_raw_parts(
                buf.as_ptr().add(size_of::<FastBulkReadHeader>()) as *const BulkReadEntry,
                n_entries,
            )
        };
        FastBulkReadPacket { header, entries }
    }

    fn make_sync_write(buf: &[u8]) -> SyncWritePacket<'_> {
        let header: &SyncWriteHeader = unsafe { &*(buf.as_ptr() as *const SyncWriteHeader) };
        SyncWritePacket {
            header,
            body: &buf[size_of::<SyncWriteHeader>()..],
        }
    }

    fn make_bulk_write(buf: &[u8]) -> BulkWritePacket<'_> {
        let header: &crate::packet::instruction::BulkWriteHeader =
            unsafe { &*(buf.as_ptr() as *const crate::packet::instruction::BulkWriteHeader) };
        BulkWritePacket {
            header,
            body: &buf[size_of::<crate::packet::instruction::BulkWriteHeader>()..],
        }
    }

    #[test]
    fn sync_read_find_slot_locates_id_and_offset() {
        let buf: [u8; 12 + 3] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, Instruction::SyncRead.as_u8(), //
            0x50, 0x00, 0x04, 0x00, //
            1, 2, 3,
        ];
        let p = make_sync_read(&buf);
        assert_eq!(
            p.find_slot(1).unwrap(),
            SyncSlotInfo {
                index: 0,
                bytes_before: 0
            }
        );
        // per-slot = header(9) + length(4) + crc(2) = 15
        assert_eq!(
            p.find_slot(2).unwrap(),
            SyncSlotInfo {
                index: 1,
                bytes_before: 15
            }
        );
        assert_eq!(
            p.find_slot(3).unwrap(),
            SyncSlotInfo {
                index: 2,
                bytes_before: 30
            }
        );
        assert!(p.find_slot(9).is_none());
    }

    #[test]
    fn bulk_read_find_slot_carries_per_entry_addr_and_length() {
        let buf: [u8; 8 + 10] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0D, 0x00, Instruction::BulkRead.as_u8(), //
            1, 0x10, 0x00, 0x04, 0x00, // id=1 addr=0x0010 len=4
            2, 0x20, 0x00, 0x08, 0x00, // id=2 addr=0x0020 len=8
        ];
        let p = make_bulk_read(&buf, 2);
        let i0 = p.find_slot(1).unwrap();
        assert_eq!(i0.index, 0);
        assert_eq!(i0.address, 0x0010);
        assert_eq!(i0.length, 4);
        assert_eq!(i0.bytes_before, 0);
        let i1 = p.find_slot(2).unwrap();
        assert_eq!(i1.index, 1);
        assert_eq!(i1.address, 0x0020);
        assert_eq!(i1.length, 8);
        // header(9) + 4 + crc(2) = 15
        assert_eq!(i1.bytes_before, 15);
        assert!(p.find_slot(9).is_none());
    }

    #[test]
    fn fast_sync_find_slot_populates_packet_length_and_offset() {
        let buf: [u8; 12 + 3] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, Instruction::FastSyncRead.as_u8(), //
            0x10, 0x00, 0x02, 0x00, //
            9, 7, 0,
        ];
        let p = make_fast_sync_read(&buf);
        let info = p.find_slot(0, 32).unwrap();
        assert_eq!(info.our_slot, 2);
        assert_eq!(info.n_slots, 3);
        assert_eq!(info.address, 0x10);
        assert_eq!(info.length, 2);
        // LEN = 3 + 3*(2+2) = 15
        assert_eq!(info.packet_length, 15);
        assert!(make_fast_sync_read(&buf).find_slot(0, 2).is_none());
    }

    #[test]
    fn fast_bulk_find_slot_handles_varying_lengths() {
        let buf: [u8; 8 + 15] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x12, 0x00, Instruction::FastBulkRead.as_u8(), //
            1, 0, 0, 4, 0, //
            2, 0, 0, 8, 0, //
            3, 0, 0, 2, 0,
        ];
        let p = make_fast_bulk_read(&buf, 3);
        assert_eq!(p.find_slot(1, 32).unwrap().bytes_before, 0);
        // FAST_RESPONSE_SLOT0_BYTES(10) + 4 = 14
        assert_eq!(p.find_slot(2, 32).unwrap().bytes_before, 14);
        // 14 + FAST_RESPONSE_SLOT_BYTES(2) + 8 = 24
        assert_eq!(p.find_slot(3, 32).unwrap().bytes_before, 24);
        // LEN = 3 + 3*2 + (4+8+2) = 23
        assert_eq!(p.find_slot(1, 32).unwrap().packet_length, 23);
    }

    #[test]
    fn fast_slot_position_classifies_correctly() {
        let mk = |our_slot, n_slots| FastSlotInfo {
            our_slot,
            n_slots,
            address: 0,
            length: 0,
            packet_length: 7,
            bytes_before: 0,
        };
        assert_eq!(mk(0, 1).position(), SlotPosition::Only { packet_length: 7 });
        assert_eq!(
            mk(0, 3).position(),
            SlotPosition::First { packet_length: 7 }
        );
        assert_eq!(mk(1, 3).position(), SlotPosition::Middle);
        assert_eq!(mk(2, 3).position(), SlotPosition::Last { crc: 0 });
    }

    #[test]
    fn sync_write_find_entry_returns_target() {
        let buf: [u8; 12 + 8] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0C, 0x00, Instruction::SyncWrite.as_u8(), //
            0x50, 0x00, 0x03, 0x00, //
            1, 10, 11, 12, //
            2, 20, 21, 22,
        ];
        let p = make_sync_write(&buf);
        let e = p.find_entry(2).unwrap();
        assert_eq!(e.id, 2);
        assert_eq!(e.data, &[20, 21, 22]);
        assert!(p.find_entry(9).is_none());
    }

    #[test]
    fn bulk_write_find_entry_returns_target_with_addr() {
        let buf: [u8; 8 + 17] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, Instruction::BulkWrite.as_u8(), //
            1, 0x50, 0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC, //
            2, 0x54, 0x02, 0x04, 0x00, 0x10, 0x20, 0x30, 0x40,
        ];
        let p = make_bulk_write(&buf);
        let e = p.find_entry(2).unwrap();
        assert_eq!(e.addr, 0x0254);
        assert_eq!(e.data, &[0x10, 0x20, 0x30, 0x40]);
        assert!(p.find_entry(9).is_none());
    }
}
