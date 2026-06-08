//! Entry types and iterators for variable-length request bodies.

#![allow(dead_code)]

use super::header::{Id, U16Le};

/// Fixed-stride entry in BulkRead / FastBulkRead request bodies.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkReadEntry {
    pub id: Id,
    pub addr: U16Le,
    pub length: U16Le,
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWriteEntry<'a> {
    pub id: Id,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWriteEntries<'a> {
    cursor: &'a [u8],
    data_len: usize,
}

impl<'a> SyncWriteEntries<'a> {
    pub(super) fn new(cursor: &'a [u8], data_len: usize) -> Self {
        Self { cursor, data_len }
    }
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
            id: Id::new(head[0]),
            data: &head[1..],
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWriteEntry<'a> {
    pub id: Id,
    pub addr: u16,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWriteEntries<'a> {
    cursor: &'a [u8],
}

impl<'a> BulkWriteEntries<'a> {
    pub(super) fn new(cursor: &'a [u8]) -> Self {
        Self { cursor }
    }
}

impl<'a> Iterator for BulkWriteEntries<'a> {
    type Item = BulkWriteEntry<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.cursor.len() < 5 {
            return None;
        }
        let id = Id::new(self.cursor[0]);
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

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    #[test]
    fn bulk_read_entry_layout() {
        assert_eq!(align_of::<BulkReadEntry>(), 1);
        assert_eq!(size_of::<BulkReadEntry>(), 5);
        assert_eq!(offset_of!(BulkReadEntry, id), 0);
        assert_eq!(offset_of!(BulkReadEntry, addr), 1);
        assert_eq!(offset_of!(BulkReadEntry, length), 3);
    }

    #[test]
    fn bulk_read_entries_overlay() {
        let body: [u8; 10] = [
            0x01, 0x10, 0x00, 0x04, 0x00, // id=1 addr=0x0010 len=4
            0x02, 0x20, 0x00, 0x08, 0x00, // id=2 addr=0x0020 len=8
        ];
        let entries: &[BulkReadEntry] =
            unsafe { core::slice::from_raw_parts(body.as_ptr() as *const BulkReadEntry, 2) };
        assert_eq!(entries[0].id, Id::new(1));
        assert_eq!(entries[0].addr.get(), 0x0010);
        assert_eq!(entries[0].length.get(), 4);
        assert_eq!(entries[1].id, Id::new(2));
        assert_eq!(entries[1].addr.get(), 0x0020);
        assert_eq!(entries[1].length.get(), 8);
    }
}
