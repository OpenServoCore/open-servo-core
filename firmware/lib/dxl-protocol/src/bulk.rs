use crate::bytes::ByteIter;
use crate::packet::{BulkReadPacket, CRC_BYTES, RESPONSE_HEADER_BYTES, SyncReadPacket};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BulkSlot {
    pub id: u8,
    pub address: u16,
    pub length: u16,
}

#[derive(Clone)]
pub struct BulkReadSlotIter<'a> {
    inner: ByteIter<'a>,
}

impl<'a> Iterator for BulkReadSlotIter<'a> {
    type Item = BulkSlot;

    fn next(&mut self) -> Option<Self::Item> {
        let id = self.inner.next()?;
        let a_lo = self.inner.next()?;
        let a_hi = self.inner.next()?;
        let l_lo = self.inner.next()?;
        let l_hi = self.inner.next()?;
        Some(BulkSlot {
            id,
            address: u16::from_le_bytes([a_lo, a_hi]),
            length: u16::from_le_bytes([l_lo, l_hi]),
        })
    }
}

impl<'a> BulkReadPacket<'a> {
    /// Decoded `(id, address, length)` slots; trailing partial tuples dropped.
    pub fn slots(&self) -> BulkReadSlotIter<'a> {
        BulkReadSlotIter {
            inner: self.body.iter(),
        }
    }

    /// `(slot_index, slot)` for `id`, or `None`.
    pub fn find_slot(&self, id: u8) -> Option<(usize, BulkSlot)> {
        self.slots().enumerate().find(|(_, s)| s.id == id)
    }

    /// Response-stream bytes emitted before `id`'s frame:
    /// `Σ over preceding slots (RESPONSE_HEADER_BYTES + slot.length + CRC_BYTES)`.
    /// `None` if `id` is not in the request.
    pub fn bytes_before(&self, id: u8) -> Option<u32> {
        let mut bytes = 0u32;
        for s in self.slots() {
            if s.id == id {
                return Some(bytes);
            }
            bytes = bytes
                .saturating_add(RESPONSE_HEADER_BYTES as u32)
                .saturating_add(s.length as u32)
                .saturating_add(CRC_BYTES as u32);
        }
        None
    }
}

impl<'a> SyncReadPacket<'a> {
    /// Index of `id` in the id list, or `None`.
    pub fn slot_index(&self, id: u8) -> Option<usize> {
        self.ids.iter().position(|b| b == id)
    }

    /// Total ids in the request (unstuffed).
    pub fn slot_count(&self) -> usize {
        self.ids.unstuffed_len()
    }

    /// Response-stream bytes emitted before `id`'s frame:
    /// `slot_index × (RESPONSE_HEADER_BYTES + length + CRC_BYTES)`.
    /// `None` if `id` is not in the request.
    pub fn bytes_before(&self, id: u8) -> Option<u32> {
        let slot = self.slot_index(id)?;
        let per_slot = (RESPONSE_HEADER_BYTES as u32)
            .saturating_add(self.length as u32)
            .saturating_add(CRC_BYTES as u32);
        Some(per_slot.saturating_mul(slot as u32))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bytes::Bytes;

    fn bulk(body: &[u8]) -> BulkReadPacket<'_> {
        BulkReadPacket {
            body: Bytes::Raw(body),
        }
    }

    fn sync(address: u16, length: u16, ids: &[u8]) -> SyncReadPacket<'_> {
        SyncReadPacket {
            address,
            length,
            ids: Bytes::Raw(ids),
        }
    }

    #[test]
    fn bulk_slots_iterates_complete_triples_only() {
        let body = [1, 0x10, 0, 4, 0, 2, 0x20, 0, 8, 0, 9, 9, 9];
        let v: heapless::Vec<_, 8> = bulk(&body).slots().collect();
        assert_eq!(v.len(), 2);
        assert_eq!(
            v[0],
            BulkSlot {
                id: 1,
                address: 0x10,
                length: 4
            }
        );
        assert_eq!(
            v[1],
            BulkSlot {
                id: 2,
                address: 0x20,
                length: 8
            }
        );
    }

    #[test]
    fn bulk_find_slot_returns_index_and_slot() {
        let body = [9, 0xFE, 0xFE, 4, 0, 0, 0x12, 0, 2, 0];
        let (idx, slot) = bulk(&body).find_slot(0).expect("our id present");
        assert_eq!(idx, 1);
        assert_eq!(
            slot,
            BulkSlot {
                id: 0,
                address: 0x12,
                length: 2
            }
        );
    }

    #[test]
    fn bulk_find_slot_missing_id_returns_none() {
        let body = [1, 0, 0, 2, 0, 2, 0, 0, 2, 0];
        assert!(bulk(&body).find_slot(9).is_none());
    }

    #[test]
    fn bulk_bytes_before_sums_response_frames() {
        // slot 0 id=1 len=4, slot 1 id=2 len=8, slot 2 id=3 len=2.
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0, 3, 0, 0, 2, 0];
        let p = bulk(&body);
        assert_eq!(p.bytes_before(1), Some(0));
        // RESPONSE_HEADER_BYTES(9) + 4 + CRC_BYTES(2) = 15
        assert_eq!(p.bytes_before(2), Some(15));
        // 15 + 9 + 8 + 2 = 34
        assert_eq!(p.bytes_before(3), Some(34));
    }

    #[test]
    fn bulk_bytes_before_missing_id_returns_none() {
        let body = [1, 0, 0, 4, 0];
        assert!(bulk(&body).bytes_before(9).is_none());
    }

    #[test]
    fn sync_slot_index_finds_id() {
        assert_eq!(sync(0, 2, &[9, 7, 0]).slot_index(7), Some(1));
    }

    #[test]
    fn sync_slot_index_missing_returns_none() {
        assert!(sync(0, 1, &[1, 2, 3]).slot_index(9).is_none());
    }

    #[test]
    fn sync_slot_count_counts_ids() {
        assert_eq!(sync(0, 1, &[1, 2, 3]).slot_count(), 3);
    }

    #[test]
    fn sync_bytes_before_uniform_payload() {
        let p = sync(0, 4, &[10, 20, 30]);
        assert_eq!(p.bytes_before(10), Some(0));
        // slot 1: 1 × (RESPONSE_HEADER_BYTES(9) + 4 + CRC_BYTES(2)) = 15
        assert_eq!(p.bytes_before(20), Some(15));
        // slot 2: 2 × 15 = 30
        assert_eq!(p.bytes_before(30), Some(30));
    }

    #[test]
    fn sync_bytes_before_missing_id_returns_none() {
        assert!(sync(0, 4, &[1, 2, 3]).bytes_before(9).is_none());
    }
}
