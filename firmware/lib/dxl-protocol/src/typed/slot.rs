use crate::wire::{ByteIter, CRC_BYTES, RESPONSE_HEADER_BYTES};

use super::packet::{BulkReadPacket, SyncReadPacket};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BulkSlot {
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

    /// Walk slots once; for the first slot matching `id`, return its position,
    /// fields, and the cumulative response-stream offset before it.
    pub fn find_slot(&self, id: u8) -> Option<BulkSlotInfo> {
        let mut bytes_before = 0u32;
        for (index, slot) in self.slots().enumerate() {
            if slot.id == id {
                return Some(BulkSlotInfo {
                    index,
                    id: slot.id,
                    address: slot.address,
                    length: slot.length,
                    bytes_before,
                });
            }
            bytes_before = bytes_before
                .saturating_add(RESPONSE_HEADER_BYTES as u32)
                .saturating_add(slot.length as u32)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::Bytes;

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
    fn bulk_find_slot_returns_info_with_bytes_before() {
        // slot 0 id=9 len=4, slot 1 id=0 len=2.
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
        // slot 0 id=1 len=4, slot 1 id=2 len=8, slot 2 id=3 len=2.
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
}
