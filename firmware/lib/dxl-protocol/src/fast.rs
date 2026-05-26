use crate::Instruction;
use crate::buf::WriteBuf;
use crate::bytes::ByteIter;
use crate::crc::crc16;
use crate::packet::{BROADCAST_ID, FastBulkReadPacket, FastSyncReadPacket, HEADER};
use crate::writer::WriteError;

/// Slot 0 emits coalesced-header(8) + ERR + ID before its payload.
pub const FAST_SLOT0_PREFIX: u32 = 10;
/// Slots k≥1 emit ERR + ID before their payload.
pub const FAST_SLOT_PREFIX: u32 = 2;

pub struct FastSlotBody<'a> {
    pub error: u8,
    pub id: u8,
    pub data: &'a [u8],
}

pub enum FastSlot<'a> {
    First {
        packet_length: u16,
        body: FastSlotBody<'a>,
    },
    Middle(FastSlotBody<'a>),
    Last(FastSlotBody<'a>),
    Only {
        packet_length: u16,
        body: FastSlotBody<'a>,
    },
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FastSlotPosition {
    Only,
    First,
    Middle,
    Last,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FastSlotInfo {
    pub our_slot: usize,
    pub n_slots: usize,
    pub address: u16,
    pub length: u16,
    /// DXL `Length` field for the coalesced Status frame.
    pub packet_length: u16,
}

/// Shared shape so the dispatcher can drive Fast Sync and Fast Bulk Reads
/// through the same code path.
pub trait FastReadPacket {
    /// `None` when our id isn't present or slot count exceeds `max_slots`.
    fn find_slot(&self, our_id: u8, max_slots: usize) -> Option<FastSlotInfo>;
    /// Cumulative response bytes emitted before `slot`'s payload. `slot == 0` → 0.
    fn bytes_before(&self, slot: usize) -> u32;
}

impl FastSlotInfo {
    pub const fn position(&self) -> FastSlotPosition {
        match (self.our_slot, self.n_slots) {
            (0, 1) => FastSlotPosition::Only,
            (0, _) => FastSlotPosition::First,
            (k, n) if k + 1 == n => FastSlotPosition::Last,
            _ => FastSlotPosition::Middle,
        }
    }
}

impl<'a> FastReadPacket for FastSyncReadPacket<'a> {
    fn find_slot(&self, our_id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let mut our_slot = None;
        let mut n_slots = 0usize;
        for (i, id) in self.ids.iter().enumerate() {
            if i >= max_slots {
                return None;
            }
            if id == our_id && our_slot.is_none() {
                our_slot = Some(i);
            }
            n_slots = i + 1;
        }
        let our_slot = our_slot?;
        // LEN covers instr(1) + N×(ERR+ID+payload) + crc(2).
        let packet_length = 3u16 + (n_slots as u16) * (2 + self.length);
        Some(FastSlotInfo {
            our_slot,
            n_slots,
            address: self.address,
            length: self.length,
            packet_length,
        })
    }

    fn bytes_before(&self, slot: usize) -> u32 {
        if slot == 0 {
            return 0;
        }
        let payload = self.length as u32;
        FAST_SLOT0_PREFIX + payload + (slot as u32 - 1) * (FAST_SLOT_PREFIX + payload)
    }
}

impl<'a> FastReadPacket for FastBulkReadPacket<'a> {
    fn find_slot(&self, our_id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let mut found = None;
        let mut n_slots = 0usize;
        let mut total_payload = 0u32;
        for (i, (id, address, length)) in self.tuples().enumerate() {
            if i >= max_slots {
                return None;
            }
            if id == our_id && found.is_none() {
                found = Some((i, address, length));
            }
            total_payload = total_payload.saturating_add(length as u32);
            n_slots = i + 1;
        }
        let (our_slot, address, length) = found?;
        // LEN covers instr(1) + N×(ERR+ID) + Σ payload_i + crc(2).
        let packet_length = (3u32 + (n_slots as u32) * 2 + total_payload) as u16;
        Some(FastSlotInfo {
            our_slot,
            n_slots,
            address,
            length,
            packet_length,
        })
    }

    fn bytes_before(&self, slot: usize) -> u32 {
        if slot == 0 {
            return 0;
        }
        let mut bytes = 0u32;
        for (i, (_, _, length)) in self.tuples().take(slot).enumerate() {
            let prefix = if i == 0 {
                FAST_SLOT0_PREFIX
            } else {
                FAST_SLOT_PREFIX
            };
            bytes = bytes.saturating_add(prefix).saturating_add(length as u32);
        }
        bytes
    }
}

impl<'a> FastBulkReadPacket<'a> {
    /// `(id, address, length)` triples from the body; trailing partials dropped.
    pub fn tuples(&self) -> FastBulkTupleIter<'a> {
        FastBulkTupleIter {
            inner: self.body.iter(),
        }
    }
}

#[derive(Clone)]
pub struct FastBulkTupleIter<'a> {
    inner: ByteIter<'a>,
}

impl<'a> Iterator for FastBulkTupleIter<'a> {
    type Item = (u8, u16, u16);

    fn next(&mut self) -> Option<Self::Item> {
        let id = self.inner.next()?;
        let a_lo = self.inner.next()?;
        let a_hi = self.inner.next()?;
        let l_lo = self.inner.next()?;
        let l_hi = self.inner.next()?;
        Some((
            id,
            u16::from_le_bytes([a_lo, a_hi]),
            u16::from_le_bytes([l_lo, l_hi]),
        ))
    }
}

/// CRC slot placeholder. Fire ISR overwrites in-flight during the DMA
/// pre-fetch race; on race loss these bytes appear on the wire and the
/// master sees a CRC mismatch (intentional — surfaces the failure).
const CRC_PLACEHOLDER: [u8; 2] = [0xAA, 0xBB];

pub fn write_fast_slot<W: WriteBuf>(out: &mut W, slot: &FastSlot<'_>) -> Result<(), WriteError> {
    let start = out.len();
    match write_fast_slot_inner(out, slot) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn write_fast_slot_inner<W: WriteBuf>(out: &mut W, slot: &FastSlot<'_>) -> Result<(), WriteError> {
    match slot {
        FastSlot::First {
            packet_length,
            body,
        } => {
            write_header(out, *packet_length)?;
            write_body(out, body)?;
        }
        FastSlot::Middle(body) => {
            write_body(out, body)?;
        }
        FastSlot::Last(body) => {
            write_body(out, body)?;
            reserve_crc(out)?;
        }
        FastSlot::Only {
            packet_length,
            body,
        } => {
            let frame_start = out.len();
            write_header(out, *packet_length)?;
            write_body(out, body)?;
            // No predecessors on the bus → CRC is purely local; no snoop and
            // no fire-time patch required (unlike Last). Compute it here so
            // the wire bytes are correct the instant we hit the DMA.
            let crc = crc16(&out.as_slice()[frame_start..]);
            out.push(crc as u8)?;
            out.push((crc >> 8) as u8)?;
        }
    }
    Ok(())
}

fn write_header<W: WriteBuf>(out: &mut W, length: u16) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(BROADCAST_ID)?;
    let lb = length.to_le_bytes();
    out.push(lb[0])?;
    out.push(lb[1])?;
    out.push(Instruction::Status.as_u8())?;
    Ok(())
}

fn write_body<W: WriteBuf>(out: &mut W, body: &FastSlotBody<'_>) -> Result<(), WriteError> {
    out.push(body.error)?;
    out.push(body.id)?;
    for &b in body.data {
        out.push(b)?;
    }
    Ok(())
}

fn reserve_crc<W: WriteBuf>(out: &mut W) -> Result<(), WriteError> {
    out.push(CRC_PLACEHOLDER[0])?;
    out.push(CRC_PLACEHOLDER[1])?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bytes::Bytes;

    fn sync(address: u16, length: u16, ids: &[u8]) -> FastSyncReadPacket<'_> {
        FastSyncReadPacket {
            address,
            length,
            ids: Bytes::Raw(ids),
        }
    }

    fn bulk(body: &[u8]) -> FastBulkReadPacket<'_> {
        FastBulkReadPacket {
            body: Bytes::Raw(body),
        }
    }

    #[test]
    fn position_classifies_slot_pattern() {
        let only = FastSlotInfo {
            our_slot: 0,
            n_slots: 1,
            address: 0,
            length: 0,
            packet_length: 0,
        };
        assert_eq!(only.position(), FastSlotPosition::Only);
        let first = FastSlotInfo {
            our_slot: 0,
            n_slots: 3,
            ..only
        };
        assert_eq!(first.position(), FastSlotPosition::First);
        let middle = FastSlotInfo {
            our_slot: 1,
            n_slots: 3,
            ..only
        };
        assert_eq!(middle.position(), FastSlotPosition::Middle);
        let last = FastSlotInfo {
            our_slot: 2,
            n_slots: 3,
            ..only
        };
        assert_eq!(last.position(), FastSlotPosition::Last);
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
    fn sync_bytes_before_uniform_payload() {
        let p = sync(0, 4, &[1, 2, 3]);
        assert_eq!(p.bytes_before(0), 0);
        // slot 1: FAST_SLOT0_PREFIX(10) + 4 = 14
        assert_eq!(p.bytes_before(1), 14);
        // slot 2: 14 + FAST_SLOT_PREFIX(2) + 4 = 20
        assert_eq!(p.bytes_before(2), 20);
    }

    #[test]
    fn bulk_tuples_iterates_complete_triples_only() {
        // Two complete tuples, then 3 stray bytes that get dropped.
        let body = [1, 0x10, 0, 4, 0, 2, 0x20, 0, 8, 0, 9, 9, 9];
        let v: heapless::Vec<_, 8> = bulk(&body).tuples().collect();
        assert_eq!(&v[..], &[(1, 0x10, 4), (2, 0x20, 8)]);
    }

    #[test]
    fn bulk_find_slot_returns_per_tuple_address_and_length() {
        // slot 0: id=9 addr=0xFEFE len=4 (bogus, but our_slot resolves separately)
        // slot 1: id=0 addr=0x12 len=2
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
    fn bulk_bytes_before_sums_varied_lengths() {
        // slot 0: len=4, slot 1: len=8, slot 2: len=2.
        let body = [1, 0, 0, 4, 0, 2, 0, 0, 8, 0, 3, 0, 0, 2, 0];
        let p = bulk(&body);
        assert_eq!(p.bytes_before(0), 0);
        // FAST_SLOT0_PREFIX(10) + 4 = 14
        assert_eq!(p.bytes_before(1), 14);
        // 14 + FAST_SLOT_PREFIX(2) + 8 = 24
        assert_eq!(p.bytes_before(2), 24);
    }

    #[test]
    fn bulk_bytes_before_matches_sync_when_uniform() {
        // Uniform 4-byte payloads across 3 slots — bulk must agree with sync.
        let bulk_body = [1, 0, 0, 4, 0, 2, 0, 0, 4, 0, 3, 0, 0, 4, 0];
        let b = bulk(&bulk_body);
        let s = sync(0, 4, &[1, 2, 3]);
        for k in 0..3 {
            assert_eq!(b.bytes_before(k), s.bytes_before(k));
        }
    }
}
