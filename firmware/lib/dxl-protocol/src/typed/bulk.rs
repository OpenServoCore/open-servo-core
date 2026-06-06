//! Bulk/Sync Read and Write request-body helpers: per-slave entries inside
//! `BulkReadPacket::body` / `FastBulkReadPacket::body` /
//! `SyncWritePacket::body` / `BulkWritePacket::body`, plus the `find_slot`
//! lookups slaves use to locate their slot in the upcoming response train (for
//! reads) or to extract their payload (for writes).

use crate::wire::{ByteIter, Bytes, CRC_BYTES, RESPONSE_HEADER_BYTES};

use super::packet::{BulkReadPacket, BulkWritePacket, SyncReadPacket, SyncWritePacket};

/// One slave's entry in a Bulk Read / Fast Bulk Read request body: `id`,
/// register `address`, and read `length`. Five wire bytes total
/// (`id + addr_le16 + len_le16`).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BulkEntry {
    pub id: u8,
    pub address: u16,
    pub length: u16,
}

/// `BulkReadPacket::find_slot` result. `bytes_before` is the cumulative wire
/// length of all Status frames before this slot's reply in the response train
/// (`Σ over preceding slots (RESPONSE_HEADER_BYTES + slot.length + CRC_BYTES)`).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BulkSlotInfo {
    pub index: usize,
    pub id: u8,
    pub address: u16,
    pub length: u16,
    pub bytes_before: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SyncSlotInfo {
    pub index: usize,
    /// `index × (RESPONSE_HEADER_BYTES + length + CRC_BYTES)`.
    pub bytes_before: u32,
}

/// Walks a body of 5-byte `id + addr_le16 + len_le16` entries, dropping any
/// trailing partial tuple. The same wire shape appears in both
/// [`BulkReadPacket`] requests and [`FastBulkReadPacket`](super::packet::FastBulkReadPacket)
/// requests; both packets' `slots()` return this iterator.
#[derive(Clone)]
pub struct BulkReadSlotIter<'a> {
    inner: ByteIter<'a>,
}

impl<'a> BulkReadSlotIter<'a> {
    pub(crate) fn new(body: Bytes<'a>) -> Self {
        Self { inner: body.iter() }
    }
}

impl<'a> Iterator for BulkReadSlotIter<'a> {
    type Item = BulkEntry;

    fn next(&mut self) -> Option<Self::Item> {
        let id = self.inner.next()?;
        let a_lo = self.inner.next()?;
        let a_hi = self.inner.next()?;
        let l_lo = self.inner.next()?;
        let l_hi = self.inner.next()?;
        Some(BulkEntry {
            id,
            address: u16::from_le_bytes([a_lo, a_hi]),
            length: u16::from_le_bytes([l_lo, l_hi]),
        })
    }
}

impl<'a> BulkReadPacket<'a> {
    /// Decoded `(id, address, length)` entries; trailing partial tuples dropped.
    pub fn slots(&self) -> BulkReadSlotIter<'a> {
        BulkReadSlotIter::new(self.body)
    }

    /// Walk entries once; for the first matching `id`, return its position,
    /// fields, and the cumulative response-stream offset before it.
    pub fn find_slot(&self, id: u8) -> Option<BulkSlotInfo> {
        let mut bytes_before = 0u32;
        for (index, entry) in self.slots().enumerate() {
            if entry.id == id {
                return Some(BulkSlotInfo {
                    index,
                    id: entry.id,
                    address: entry.address,
                    length: entry.length,
                    bytes_before,
                });
            }
            bytes_before = bytes_before
                .saturating_add(RESPONSE_HEADER_BYTES as u32)
                .saturating_add(entry.length as u32)
                .saturating_add(CRC_BYTES as u32);
        }
        None
    }
}

impl<'a> SyncReadPacket<'a> {
    /// `index` and the cumulative response-stream offset for `id`'s reply slot.
    pub fn find_slot(&self, id: u8) -> Option<SyncSlotInfo> {
        let index = self.ids.iter().position(|b| b == id)?;
        let per_slot = (RESPONSE_HEADER_BYTES as u32)
            .saturating_add(self.length as u32)
            .saturating_add(CRC_BYTES as u32);
        let bytes_before = per_slot.saturating_mul(index as u32);
        Some(SyncSlotInfo {
            index,
            bytes_before,
        })
    }
}

/// Per-slot header inside a Sync Write body — just the addressed `id`. Data
/// bytes follow on the wire at fixed stride `length` (from the packet header),
/// extracted via [`SyncWritePacket::find_slot_data`]; the iterator skips them.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SyncWriteEntry {
    pub id: u8,
}

/// Per-slot header inside a Bulk Write body — `id` plus the slot's own
/// `address` and `length` (which differ across slots). The `length`-byte data
/// payload follows on the wire and is extracted via
/// [`BulkWritePacket::find_slot_data`]; the iterator skips it.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BulkWriteEntry {
    pub id: u8,
    pub address: u16,
    pub length: u16,
}

/// Walks a Sync Write body yielding the addressed ID for each `1 + length`
/// stride; data bytes are skipped logically (via the unstuffing `ByteIter`).
/// Trailing partial chunks are dropped silently.
#[derive(Clone)]
pub struct SyncWriteSlotIter<'a> {
    iter: ByteIter<'a>,
    length: u16,
}

impl<'a> SyncWriteSlotIter<'a> {
    pub(crate) fn new(body: Bytes<'a>, length: u16) -> Self {
        Self {
            iter: body.iter(),
            length,
        }
    }
}

impl<'a> Iterator for SyncWriteSlotIter<'a> {
    type Item = SyncWriteEntry;

    fn next(&mut self) -> Option<Self::Item> {
        let id = self.iter.next()?;
        for _ in 0..self.length {
            self.iter.next()?;
        }
        Some(SyncWriteEntry { id })
    }
}

/// Walks a Bulk Write body yielding `(id, address, length)` headers; the
/// `length`-byte data payload after each header is skipped logically (via the
/// unstuffing `ByteIter`). Trailing partial tuples are dropped silently.
#[derive(Clone)]
pub struct BulkWriteSlotIter<'a> {
    iter: ByteIter<'a>,
}

impl<'a> BulkWriteSlotIter<'a> {
    pub(crate) fn new(body: Bytes<'a>) -> Self {
        Self { iter: body.iter() }
    }
}

impl<'a> Iterator for BulkWriteSlotIter<'a> {
    type Item = BulkWriteEntry;

    fn next(&mut self) -> Option<Self::Item> {
        let id = self.iter.next()?;
        let a_lo = self.iter.next()?;
        let a_hi = self.iter.next()?;
        let l_lo = self.iter.next()?;
        let l_hi = self.iter.next()?;
        let length = u16::from_le_bytes([l_lo, l_hi]);
        for _ in 0..length {
            self.iter.next()?;
        }
        Some(BulkWriteEntry {
            id,
            address: u16::from_le_bytes([a_lo, a_hi]),
            length,
        })
    }
}

impl<'a> SyncWritePacket<'a> {
    /// Decoded `(id)` headers; trailing partial chunks dropped.
    pub fn slots(&self) -> SyncWriteSlotIter<'a> {
        SyncWriteSlotIter::new(self.body, self.length)
    }

    /// Walk the body looking for `id`; on match, copy its `length`-byte
    /// unstuffed payload into `buf` and return the number of bytes written.
    /// Returns `None` if `id` isn't present, if a slot's data is truncated, or
    /// if `buf` is too small.
    pub fn find_slot_data(&self, id: u8, buf: &mut [u8]) -> Option<usize> {
        let want = self.length as usize;
        if buf.len() < want {
            return None;
        }
        let mut iter = self.body.iter();
        loop {
            let slot_id = iter.next()?;
            if slot_id == id {
                for slot in buf.iter_mut().take(want) {
                    *slot = iter.next()?;
                }
                return Some(want);
            }
            for _ in 0..want {
                iter.next()?;
            }
        }
    }
}

impl<'a> BulkWritePacket<'a> {
    /// Decoded `(id, address, length)` headers; trailing partial tuples
    /// dropped.
    pub fn slots(&self) -> BulkWriteSlotIter<'a> {
        BulkWriteSlotIter::new(self.body)
    }

    /// Walk the body looking for `id`; on match, copy its `length`-byte
    /// unstuffed payload into `buf` and return the matched header plus the
    /// byte count written. Returns `None` if `id` isn't present, if a slot's
    /// data is truncated, or if `buf` is too small for that slot's length.
    pub fn find_slot_data(&self, id: u8, buf: &mut [u8]) -> Option<(BulkWriteEntry, usize)> {
        let mut iter = self.body.iter();
        loop {
            let slot_id = iter.next()?;
            let a_lo = iter.next()?;
            let a_hi = iter.next()?;
            let l_lo = iter.next()?;
            let l_hi = iter.next()?;
            let length = u16::from_le_bytes([l_lo, l_hi]);
            let want = length as usize;
            if slot_id == id {
                if buf.len() < want {
                    return None;
                }
                for slot in buf.iter_mut().take(want) {
                    *slot = iter.next()?;
                }
                return Some((
                    BulkWriteEntry {
                        id: slot_id,
                        address: u16::from_le_bytes([a_lo, a_hi]),
                        length,
                    },
                    want,
                ));
            }
            for _ in 0..want {
                iter.next()?;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bulk(body: &[u8]) -> BulkReadPacket<'_> {
        BulkReadPacket {
            body: Bytes::unstuffed(body),
        }
    }

    fn sync(address: u16, length: u16, ids: &[u8]) -> SyncReadPacket<'_> {
        SyncReadPacket {
            address,
            length,
            ids: Bytes::unstuffed(ids),
        }
    }

    #[test]
    fn bulk_slots_iterates_complete_triples_only() {
        let body = [1, 0x10, 0, 4, 0, 2, 0x20, 0, 8, 0, 9, 9, 9];
        let v: heapless::Vec<_, 8> = bulk(&body).slots().collect();
        assert_eq!(v.len(), 2);
        assert_eq!(
            v[0],
            BulkEntry {
                id: 1,
                address: 0x10,
                length: 4
            }
        );
        assert_eq!(
            v[1],
            BulkEntry {
                id: 2,
                address: 0x20,
                length: 8
            }
        );
    }

    #[test]
    fn bulk_find_slot_returns_info_with_bytes_before() {
        let body = [9, 0xFE, 0xFE, 4, 0, 0, 0x12, 0, 2, 0];
        let info = bulk(&body).find_slot(0).expect("id present");
        assert_eq!(info.index, 1);
        assert_eq!(info.id, 0);
        assert_eq!(info.address, 0x12);
        assert_eq!(info.length, 2);
        // RESPONSE_HEADER_BYTES(9) + 4 + CRC_BYTES(2) = 15
        assert_eq!(info.bytes_before, 15);
    }

    #[test]
    fn bulk_find_slot_first_slot_has_zero_bytes_before() {
        let body = [5, 0, 0, 4, 0, 7, 0, 0, 4, 0];
        let info = bulk(&body).find_slot(5).expect("id present");
        assert_eq!(info.index, 0);
        assert_eq!(info.bytes_before, 0);
    }

    #[test]
    fn bulk_find_slot_missing_id_returns_none() {
        let body = [1, 0, 0, 2, 0, 2, 0, 0, 2, 0];
        assert!(bulk(&body).find_slot(9).is_none());
    }

    #[test]
    fn bulk_find_slot_sums_varied_response_lengths() {
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0, 3, 0, 0, 2, 0];
        let p = bulk(&body);
        assert_eq!(p.find_slot(1).unwrap().bytes_before, 0);
        // RESPONSE_HEADER_BYTES(9) + 4 + CRC_BYTES(2) = 15
        assert_eq!(p.find_slot(2).unwrap().bytes_before, 15);
        // 15 + 9 + 8 + 2 = 34
        assert_eq!(p.find_slot(3).unwrap().bytes_before, 34);
    }

    #[test]
    fn sync_find_slot_returns_index_and_bytes_before() {
        let p = sync(0, 4, &[10, 20, 30]);
        let info = p.find_slot(20).expect("id present");
        assert_eq!(info.index, 1);
        // 1 × (RESPONSE_HEADER_BYTES(9) + 4 + CRC_BYTES(2)) = 15
        assert_eq!(info.bytes_before, 15);
    }

    #[test]
    fn sync_find_slot_first_slot_has_zero_bytes_before() {
        let info = sync(0, 4, &[10, 20, 30]).find_slot(10).expect("id present");
        assert_eq!(info.index, 0);
        assert_eq!(info.bytes_before, 0);
    }

    #[test]
    fn sync_find_slot_missing_id_returns_none() {
        assert!(sync(0, 4, &[1, 2, 3]).find_slot(9).is_none());
    }

    fn sync_write(address: u16, length: u16, body: &[u8]) -> SyncWritePacket<'_> {
        SyncWritePacket {
            address,
            length,
            body: Bytes::unstuffed(body),
        }
    }

    fn bulk_write(body: &[u8]) -> BulkWritePacket<'_> {
        BulkWritePacket {
            body: Bytes::unstuffed(body),
        }
    }

    #[test]
    fn sync_write_slots_iterates_complete_chunks_only() {
        // length=3 → stride 4: id 1, id 2, then a stray byte (not enough for
        // another full slot) — dropped.
        let body = [1, 10, 11, 12, 2, 20, 21, 22, 9];
        let p = sync_write(0x0050, 3, &body);
        let v: heapless::Vec<_, 4> = p.slots().collect();
        assert_eq!(v.len(), 2);
        assert_eq!(v[0].id, 1);
        assert_eq!(v[1].id, 2);
    }

    #[test]
    fn sync_write_slots_truncated_payload_dropped() {
        // length=4 but second entry has only 3 data bytes — drop it.
        let body = [5, 1, 2, 3, 4, 6, 1, 2, 3];
        let p = sync_write(0x0050, 4, &body);
        let v: heapless::Vec<_, 4> = p.slots().collect();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].id, 5);
    }

    #[test]
    fn sync_write_slots_zero_length_yields_every_byte_as_id() {
        let body = [1, 2, 3];
        let p = sync_write(0x0050, 0, &body);
        let v: heapless::Vec<_, 4> = p.slots().collect();
        assert_eq!(v.len(), 3);
        assert_eq!(v[0].id, 1);
        assert_eq!(v[2].id, 3);
    }

    #[test]
    fn sync_write_find_slot_data_returns_first_match() {
        let body = [1, 10, 11, 2, 20, 21, 1, 99, 99];
        let p = sync_write(0x0050, 2, &body);
        let mut buf = [0u8; 4];
        let n = p.find_slot_data(1, &mut buf).expect("id present");
        assert_eq!(n, 2);
        assert_eq!(&buf[..n], &[10, 11]);
        assert!(p.find_slot_data(7, &mut buf).is_none());
    }

    #[test]
    fn sync_write_find_slot_data_buffer_too_small_returns_none() {
        let body = [1, 10, 11, 12];
        let p = sync_write(0x0050, 3, &body);
        let mut buf = [0u8; 2];
        assert!(p.find_slot_data(1, &mut buf).is_none());
    }

    #[test]
    fn bulk_write_slots_iterates_complete_tuples_only() {
        // Entry A: id=1, addr=0x0050, len=3
        // Entry B: id=2, addr=0x0254, len=4
        let body = [
            1, 0x50, 0x00, 3, 0, 0xAA, 0xBB, 0xCC, //
            2, 0x54, 0x02, 4, 0, 0x10, 0x20, 0x30, 0x40,
        ];
        let p = bulk_write(&body);
        let v: heapless::Vec<_, 4> = p.slots().collect();
        assert_eq!(v.len(), 2);
        assert_eq!(
            v[0],
            BulkWriteEntry {
                id: 1,
                address: 0x0050,
                length: 3
            }
        );
        assert_eq!(
            v[1],
            BulkWriteEntry {
                id: 2,
                address: 0x0254,
                length: 4
            }
        );
    }

    #[test]
    fn bulk_write_slots_truncated_header_dropped() {
        // Trailing entry has id + addr + partial length — drop it.
        let body = [1, 0x10, 0x00, 2, 0, 0xDE, 0xAD, 9, 0x20, 0x00, 4];
        let p = bulk_write(&body);
        let v: heapless::Vec<_, 4> = p.slots().collect();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].id, 1);
    }

    #[test]
    fn bulk_write_slots_truncated_data_dropped() {
        // Entry claims length=4 but only 2 data bytes remain.
        let body = [3, 0x00, 0x01, 4, 0, 0xAA, 0xBB];
        let p = bulk_write(&body);
        assert!(p.slots().next().is_none());
    }

    #[test]
    fn bulk_write_find_slot_data_returns_first_match() {
        let body = [
            1, 0x50, 0x00, 2, 0, 0xAA, 0xBB, //
            2, 0x54, 0x02, 4, 0, 0x11, 0x22, 0x33, 0x44, //
            1, 0x00, 0x00, 1, 0, 0x99,
        ];
        let p = bulk_write(&body);
        let mut buf = [0u8; 8];
        let (hdr, n) = p.find_slot_data(2, &mut buf).expect("id present");
        assert_eq!(hdr.address, 0x0254);
        assert_eq!(hdr.length, 4);
        assert_eq!(n, 4);
        assert_eq!(&buf[..n], &[0x11, 0x22, 0x33, 0x44]);
        let (first, n) = p.find_slot_data(1, &mut buf).expect("id present");
        assert_eq!(first.address, 0x0050);
        assert_eq!(first.length, 2);
        assert_eq!(&buf[..n], &[0xAA, 0xBB]);
        assert!(p.find_slot_data(9, &mut buf).is_none());
    }

    #[test]
    fn bulk_write_find_slot_data_buffer_too_small_returns_none() {
        let body = [1, 0x50, 0x00, 4, 0, 0xAA, 0xBB, 0xCC, 0xDD];
        let p = bulk_write(&body);
        let mut buf = [0u8; 2];
        assert!(p.find_slot_data(1, &mut buf).is_none());
    }
}
