use crate::wire::{
    BROADCAST_ID, ByteIter, CrcUmts, FAST_RESPONSE_SLOT_BYTES, FAST_RESPONSE_SLOT0_BYTES, HEADER,
    WriteBuf, WriteError,
};

use super::instruction::Instruction;
use super::packet::{FastBulkReadPacket, FastSyncReadPacket};
use super::status_error::StatusError;

/// Position of our slot in the coalesced Fast Status chain. Variants that
/// emit the chain header carry the chain's DXL `Length` field; the rest don't
/// need it.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FastPosition {
    /// Single-slot chain — emits full header and locally-computed CRC.
    Only { packet_length: u16 },
    /// First of N — emits header + body, no CRC (successors continue the chain).
    First { packet_length: u16 },
    /// Body only.
    Middle,
    /// Body + CRC placeholder (chip patches the placeholder at fire time).
    Last,
}

/// Which Fast Read request kind a `FastReadPacket` represents — lets the
/// dispatcher pick `Reply::FastSyncRead` vs `FastBulkRead` from a
/// generic `P: FastReadPacket`.
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
    /// coalesced Fast Status chain.
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
    pub const fn position(&self) -> FastPosition {
        match (self.our_slot, self.n_slots) {
            (0, 1) => FastPosition::Only {
                packet_length: self.packet_length,
            },
            (0, _) => FastPosition::First {
                packet_length: self.packet_length,
            },
            (k, n) if k + 1 == n => FastPosition::Last,
            _ => FastPosition::Middle,
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
        for (i, (slot_id, address, length)) in self.tuples().enumerate() {
            if i >= max_slots {
                return None;
            }
            if slot_id == id && found.is_none() {
                found = Some((i, address, length, bytes_before));
            }
            let prefix = if i == 0 {
                FAST_RESPONSE_SLOT0_BYTES as u32
            } else {
                FAST_RESPONSE_SLOT_BYTES as u32
            };
            bytes_before = bytes_before
                .saturating_add(prefix)
                .saturating_add(length as u32);
            total_payload = total_payload.saturating_add(length as u32);
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

/// Serialize one Fast Status chain slot. The payload is NOT byte-stuffed —
/// Fast Read decoding is positional (slot indices), not trigger-driven, so
/// stuffing would break deterministic chain layout.
pub(crate) fn write_fast<W: WriteBuf, I: Iterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    position: FastPosition,
    id: u8,
    error: StatusError,
    payload: &mut I,
) -> Result<(), WriteError> {
    let start = out.len();
    match write_fast_inner::<W, _, CRC>(out, position, id, error, payload) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn write_fast_inner<W: WriteBuf, I: Iterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    position: FastPosition,
    id: u8,
    error: StatusError,
    payload: &mut I,
) -> Result<(), WriteError> {
    match position {
        FastPosition::Only { packet_length } => {
            let frame_start = out.len();
            write_fast_header(out, packet_length)?;
            write_fast_body(out, id, error, payload)?;
            // No predecessors → CRC is purely local; compute over the bytes
            // we just emitted and append.
            let crc = CRC::accumulate(0, &out.as_slice()[frame_start..]);
            out.push(crc as u8)?;
            out.push((crc >> 8) as u8)?;
        }
        FastPosition::First { packet_length } => {
            write_fast_header(out, packet_length)?;
            write_fast_body(out, id, error, payload)?;
        }
        FastPosition::Middle => {
            write_fast_body(out, id, error, payload)?;
        }
        FastPosition::Last => {
            write_fast_body(out, id, error, payload)?;
            out.push(CRC_PLACEHOLDER[0])?;
            out.push(CRC_PLACEHOLDER[1])?;
        }
    }
    Ok(())
}

fn write_fast_header<W: WriteBuf>(out: &mut W, length: u16) -> Result<(), WriteError> {
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

fn write_fast_body<W: WriteBuf, I: Iterator<Item = u8>>(
    out: &mut W,
    id: u8,
    error: StatusError,
    payload: &mut I,
) -> Result<(), WriteError> {
    out.push(error.as_u8())?;
    out.push(id)?;
    for b in payload.by_ref() {
        out.push(b)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::Bytes;

    fn sync(address: u16, length: u16, ids: &[u8]) -> FastSyncReadPacket<'_> {
        FastSyncReadPacket {
            address,
            length,
            ids: Bytes::raw(ids),
        }
    }

    fn bulk(body: &[u8]) -> FastBulkReadPacket<'_> {
        FastBulkReadPacket {
            body: Bytes::raw(body),
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
        assert_eq!(only.position(), FastPosition::Only { packet_length: 7 });
        let first = FastSlotInfo {
            our_slot: 0,
            n_slots: 3,
            ..only
        };
        assert_eq!(first.position(), FastPosition::First { packet_length: 7 });
        let middle = FastSlotInfo {
            our_slot: 1,
            n_slots: 3,
            ..only
        };
        assert_eq!(middle.position(), FastPosition::Middle);
        let last = FastSlotInfo {
            our_slot: 2,
            n_slots: 3,
            ..only
        };
        assert_eq!(last.position(), FastPosition::Last);
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
    fn bulk_find_slot_populates_bytes_before_for_varied_lengths() {
        // slot 0 id=1 len=4, slot 1 id=2 len=8, slot 2 id=3 len=2.
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
        // Uniform 4-byte payloads across 3 slots — bulk must agree with sync.
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
