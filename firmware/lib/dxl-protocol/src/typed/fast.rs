use crate::wire::{FAST_RESPONSE_SLOT_BYTES, FAST_RESPONSE_SLOT0_BYTES};

use super::bulk::BulkReadSlotIter;
use super::packet::{FastBulkReadPacket, FastSyncReadPacket};

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

/// Which Fast Read request kind a `FastReadPacket` represents — lets the
/// dispatcher pick the right typed status from a generic `P: FastReadPacket`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FastReadVariant {
    Sync,
    Bulk,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FastSlotInfo {
    pub our_slot: usize,
    pub n_slots: usize,
    pub address: u16,
    pub length: u16,
    /// DXL `Length` field for the coalesced Status frame; only used by
    /// `Only` / `First` slots that emit the header.
    pub packet_length: u16,
    /// Cumulative response bytes emitted before our slot's payload in the
    /// coalesced Fast Status response.
    pub bytes_before: u32,
}

/// Shared shape so the dispatcher can drive Fast Sync and Fast Bulk Reads
/// through the same code path.
pub trait FastReadPacket {
    const VARIANT: FastReadVariant;
    /// `None` when `id` isn't present or slot count exceeds `max_slots`.
    fn find_slot(&self, id: u8, max_slots: usize) -> Option<FastSlotInfo>;
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

impl<'a> FastReadPacket for FastSyncReadPacket<'a> {
    const VARIANT: FastReadVariant = FastReadVariant::Sync;

    fn find_slot(&self, id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let mut our_slot = None;
        let mut n_slots = 0usize;
        for (i, slot_id) in self.ids.iter().enumerate() {
            if i >= max_slots {
                return None;
            }
            if slot_id == id && our_slot.is_none() {
                our_slot = Some(i);
            }
            n_slots = i + 1;
        }
        let our_slot = our_slot?;
        let payload = self.length as u32;
        let bytes_before = if our_slot == 0 {
            0
        } else {
            FAST_RESPONSE_SLOT0_BYTES as u32
                + payload
                + (our_slot as u32 - 1) * (FAST_RESPONSE_SLOT_BYTES as u32 + payload)
        };
        // LEN covers instr(1) + N×(ERR+ID+payload) + crc(2).
        let packet_length = 3u16 + (n_slots as u16) * (2 + self.length);
        Some(FastSlotInfo {
            our_slot,
            n_slots,
            address: self.address,
            length: self.length,
            packet_length,
            bytes_before,
        })
    }
}

impl<'a> FastReadPacket for FastBulkReadPacket<'a> {
    const VARIANT: FastReadVariant = FastReadVariant::Bulk;

    fn find_slot(&self, id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let mut found: Option<(usize, u16, u16, u32)> = None;
        let mut n_slots = 0usize;
        let mut total_payload = 0u32;
        let mut bytes_before = 0u32;
        for (i, entry) in self.slots().enumerate() {
            if i >= max_slots {
                return None;
            }
            if entry.id == id && found.is_none() {
                found = Some((i, entry.address, entry.length, bytes_before));
            }
            let prefix = if i == 0 {
                FAST_RESPONSE_SLOT0_BYTES as u32
            } else {
                FAST_RESPONSE_SLOT_BYTES as u32
            };
            bytes_before = bytes_before
                .saturating_add(prefix)
                .saturating_add(entry.length as u32);
            total_payload = total_payload.saturating_add(entry.length as u32);
            n_slots = i + 1;
        }
        let (our_slot, address, length, bytes_before) = found?;
        // LEN covers instr(1) + N×(ERR+ID) + Σ payload_i + crc(2).
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

impl<'a> FastBulkReadPacket<'a> {
    /// Decoded `(id, address, length)` entries from the body — same wire shape
    /// as [`BulkReadPacket::slots`](crate::BulkReadPacket::slots); trailing
    /// partial tuples dropped.
    pub fn slots(&self) -> BulkReadSlotIter<'a> {
        BulkReadSlotIter::new(self.body)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BulkEntry;
    use crate::wire::Bytes;

    fn sync(address: u16, length: u16, ids: &[u8]) -> FastSyncReadPacket<'_> {
        FastSyncReadPacket {
            address,
            length,
            ids: Bytes::unstuffed(ids),
        }
    }

    fn bulk(body: &[u8]) -> FastBulkReadPacket<'_> {
        FastBulkReadPacket {
            body: Bytes::unstuffed(body),
        }
    }

    #[test]
    fn position_classifies_slot_pattern() {
        let only = FastSlotInfo {
            our_slot: 0,
            n_slots: 1,
            address: 0,
            length: 0,
            packet_length: 7,
            bytes_before: 0,
        };
        assert_eq!(only.position(), SlotPosition::Only { packet_length: 7 });
        let first = FastSlotInfo {
            our_slot: 0,
            n_slots: 3,
            ..only
        };
        assert_eq!(first.position(), SlotPosition::First { packet_length: 7 });
        let middle = FastSlotInfo {
            our_slot: 1,
            n_slots: 3,
            ..only
        };
        assert_eq!(middle.position(), SlotPosition::Middle);
        let last = FastSlotInfo {
            our_slot: 2,
            n_slots: 3,
            ..only
        };
        assert_eq!(last.position(), SlotPosition::Last { crc: 0 });
    }

    #[test]
    fn sync_find_slot_returns_slot_and_packet_length() {
        let p = sync(0x10, 2, &[9, 7, 0]);
        let info = p.find_slot(0, 32).expect("our id present");
        assert_eq!(info.our_slot, 2);
        assert_eq!(info.n_slots, 3);
        assert_eq!(info.address, 0x10);
        assert_eq!(info.length, 2);
        // LEN = 3 + 3*(2+2) = 15
        assert_eq!(info.packet_length, 15);
    }

    #[test]
    fn sync_find_slot_missing_id_returns_none() {
        assert!(sync(0, 1, &[1, 2, 3]).find_slot(9, 32).is_none());
    }

    #[test]
    fn sync_find_slot_respects_max_slots_bound() {
        let ids = [1, 2, 3, 4, 5];
        assert!(sync(0, 1, &ids).find_slot(5, 4).is_none());
        assert!(sync(0, 1, &ids).find_slot(5, 5).is_some());
    }

    #[test]
    fn sync_find_slot_populates_bytes_before_uniform_payload() {
        let p = sync(0, 4, &[1, 2, 3]);
        assert_eq!(p.find_slot(1, 32).unwrap().bytes_before, 0);
        // slot 1: FAST_RESPONSE_SLOT0_BYTES(10) + 4 = 14
        assert_eq!(p.find_slot(2, 32).unwrap().bytes_before, 14);
        // slot 2: 14 + FAST_RESPONSE_SLOT_BYTES(2) + 4 = 20
        assert_eq!(p.find_slot(3, 32).unwrap().bytes_before, 20);
    }

    #[test]
    fn bulk_slots_iterates_complete_triples_only() {
        // Two complete tuples, then 3 stray bytes that get dropped.
        let body = [1, 0x10, 0, 4, 0, 2, 0x20, 0, 8, 0, 9, 9, 9];
        let v: heapless::Vec<_, 8> = bulk(&body).slots().collect();
        assert_eq!(
            &v[..],
            &[
                BulkEntry {
                    id: 1,
                    address: 0x10,
                    length: 4
                },
                BulkEntry {
                    id: 2,
                    address: 0x20,
                    length: 8
                },
            ]
        );
    }

    #[test]
    fn bulk_find_slot_returns_per_tuple_address_and_length() {
        let body = [9, 0xFE, 0xFE, 4, 0, 0, 0x12, 0, 2, 0];
        let info = bulk(&body).find_slot(0, 32).expect("our id present");
        assert_eq!(info.our_slot, 1);
        assert_eq!(info.n_slots, 2);
        assert_eq!(info.address, 0x12);
        assert_eq!(info.length, 2);
        // LEN = 3 + 2*2 + (4+2) = 13
        assert_eq!(info.packet_length, 13);
    }

    #[test]
    fn bulk_find_slot_missing_id_returns_none() {
        let body = [1, 0, 0, 2, 0, 2, 0, 0, 2, 0];
        assert!(bulk(&body).find_slot(9, 32).is_none());
    }

    #[test]
    fn bulk_find_slot_ignores_trailing_partial_tuple() {
        let body = [1, 0, 0, 4, 0, 9, 9, 9];
        let info = bulk(&body).find_slot(1, 32).expect("our id present");
        assert_eq!(info.n_slots, 1);
    }

    #[test]
    fn bulk_find_slot_populates_bytes_before_for_varied_lengths() {
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0, 3, 0, 0, 2, 0];
        let p = bulk(&body);
        assert_eq!(p.find_slot(1, 32).unwrap().bytes_before, 0);
        // FAST_RESPONSE_SLOT0_BYTES(10) + 4 = 14
        assert_eq!(p.find_slot(2, 32).unwrap().bytes_before, 14);
        // 14 + FAST_RESPONSE_SLOT_BYTES(2) + 8 = 24
        assert_eq!(p.find_slot(3, 32).unwrap().bytes_before, 24);
    }

    #[test]
    fn bulk_find_slot_bytes_before_matches_sync_when_uniform() {
        let bulk_body = [1, 0, 0, 4, 0, 2, 0, 0, 4, 0, 3, 0, 0, 4, 0];
        let b = bulk(&bulk_body);
        let s = sync(0, 4, &[1, 2, 3]);
        for id in [1u8, 2, 3] {
            assert_eq!(
                b.find_slot(id, 32).unwrap().bytes_before,
                s.find_slot(id, 32).unwrap().bytes_before
            );
        }
    }
}
