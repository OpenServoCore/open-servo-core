use crate::wire::Bytes;

use super::ext::{NoStatusExt, StatusExt};
use super::packet::RawStatus;
use super::status_error::StatusError;

#[derive(Copy, Clone, Debug)]
pub struct PingStatus {
    pub id: u8,
    pub model: u16,
    pub firmware: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ReadStatus<'a> {
    pub id: u8,
    pub data: Bytes<'a>,
}

#[derive(Copy, Clone, Debug)]
pub struct SyncReadStatus<'a> {
    pub id: u8,
    pub data: Bytes<'a>,
}

#[derive(Copy, Clone, Debug)]
pub struct BulkReadStatus<'a> {
    pub id: u8,
    pub data: Bytes<'a>,
}

#[derive(Copy, Clone, Debug)]
pub struct WriteStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct RegWriteStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ActionStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct RebootStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ErrorStatus {
    pub id: u8,
    pub error: StatusError,
}

/// Wire-faithful piece of a Fast Sync / Fast Bulk Read response: one slave's
/// `(id, error, data)` block. The same shape appears in two contexts:
///
/// - **Slave TX** (single slot): the chip emits one `Slot` via
///   [`write_slot`](crate::write_slot) tagged with its
///   [`SlotPosition`](crate::SlotPosition) in the coalesced response.
/// - **Master RX** (multi-slot): [`FastSyncReadStatus::slots`] and
///   [`FastBulkReadStatus::slots`] walk the parsed coalesced frame and yield a
///   `Slot` per slave.
///
/// `error` is the raw wire byte — slaves pass [`StatusError::as_u8`]; masters
/// can interpret via [`StatusError::from_u8`]. Wire-faithful (no enum coercion
/// at decode means unknown vendor error codes don't crash the iterator).
///
/// `data` is the payload `Bytes` — unstuffed by spec (Fast Read decoding is
/// positional, not trigger-driven).
#[derive(Copy, Clone, Debug)]
pub struct Slot<'a> {
    pub id: u8,
    pub error: u8,
    pub data: Bytes<'a>,
}

/// Master-side typed view of a coalesced Fast Sync Read response.
///
/// On the wire, every slave's slot is `length` data bytes wide (uniform —
/// taken from the master's `FastSyncReadPacket::length`). Call [`Self::slots`]
/// with that length to walk the response one slot at a time.
#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadStatus<'a> {
    /// Slot 0's error byte — the response frame's error field is per spec
    /// the first slot's error. Subsequent slots carry their own error byte
    /// inline in `payload`.
    pub error: u8,
    /// Raw payload bytes: `slot0_id + slot0_data || slot1_err + slot1_id +
    /// slot1_data || ...`. Fast Read payloads are not stuffed; if `payload`
    /// came in as `Bytes::Stuffed`, [`Self::slots`] re-tags it as `Unstuffed`
    /// before walking.
    pub payload: Bytes<'a>,
}

impl<'a> FastSyncReadStatus<'a> {
    pub(crate) fn from_raw(raw: RawStatus<'a>) -> Self {
        Self {
            error: raw.error,
            payload: raw.params,
        }
    }

    /// Iterate per-slave `Slot { id, error, data }` blocks. `slot_length` is
    /// the per-slave data width from the original `FastSyncReadPacket::length`.
    /// Returns `None` when the response payload runs out (truncated frame, or
    /// just past the last slot).
    pub fn slots(&self, slot_length: u16) -> FastSyncSlotIter<'a> {
        FastSyncSlotIter {
            cursor: self.payload.as_unstuffed(),
            slot_length: slot_length as usize,
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

/// Master-side typed view of a coalesced Fast Bulk Read response. Per-slot
/// data widths vary; [`Self::slots`] takes an iterator of `length` values
/// (typically derived from the original `FastBulkReadPacket::slots()`).
#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadStatus<'a> {
    pub error: u8,
    pub payload: Bytes<'a>,
}

impl<'a> FastBulkReadStatus<'a> {
    pub(crate) fn from_raw(raw: RawStatus<'a>) -> Self {
        Self {
            error: raw.error,
            payload: raw.params,
        }
    }

    /// Iterate per-slave `Slot { id, error, data }` blocks, taking each slot's
    /// data width from `lengths`. Stops when either the payload runs out or
    /// `lengths` does.
    pub fn slots<L: IntoIterator<Item = u16>>(
        &self,
        lengths: L,
    ) -> FastBulkSlotIter<'a, L::IntoIter> {
        FastBulkSlotIter {
            cursor: self.payload.as_unstuffed(),
            lengths: lengths.into_iter(),
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

/// Per-slot iterator for [`FastSyncReadStatus::slots`].
#[derive(Copy, Clone, Debug)]
pub struct FastSyncSlotIter<'a> {
    cursor: Bytes<'a>,
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

/// Per-slot iterator for [`FastBulkReadStatus::slots`].
pub struct FastBulkSlotIter<'a, L: Iterator<Item = u16>> {
    cursor: Bytes<'a>,
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
    cursor: &mut Bytes<'a>,
    slot_length: usize,
    slot0_error: u8,
    slot_index: &mut usize,
) -> Option<Slot<'a>> {
    let (id, error) = if *slot_index == 0 {
        // Slot 0's error byte was hoisted to the frame's error field; payload
        // starts with the slot's id.
        let (id, rest) = cursor.split_first()?;
        *cursor = rest;
        (id, slot0_error)
    } else {
        let (error, rest) = cursor.split_first()?;
        *cursor = rest;
        let (id, rest) = cursor.split_first()?;
        *cursor = rest;
        (id, error)
    };
    let (data, rest) = cursor.split_at(slot_length)?;
    *cursor = rest;
    *slot_index += 1;
    Some(Slot { id, error, data })
}

/// Typed view of a Status-instruction frame. One variant per logical response
/// shape; pairs with the request instruction the master sent so the master can
/// typed-decode the raw status payload, and the slave can typed-construct
/// replies (single-slot variants only — Fast slots emit via
/// [`write_slot`](crate::write_slot)).
///
/// `Read` / `SyncRead` / `BulkRead` share wire bytes — separate variants are
/// kept for caller-side intent. `FastSyncRead` / `FastBulkRead` are
/// **master-side only**: a coalesced multi-slot response, decoded via
/// [`decode_status`](crate::decode_status), and walked via the variant's
/// `slots()` iterator. Slaves never construct these — they emit `Slot` pieces.
///
/// The optional `S` parameter plugs in a vendor status extension (see
/// [`StatusExt`]); pure-DXL callers leave it at the [`NoStatusExt`] default,
/// which makes [`Status::Ext`] statically uninhabited.
#[derive(Copy, Clone, Debug)]
pub enum Status<'a, S: StatusExt = NoStatusExt> {
    Ping(PingStatus),
    Read(ReadStatus<'a>),
    SyncRead(SyncReadStatus<'a>),
    BulkRead(BulkReadStatus<'a>),
    Write(WriteStatus),
    RegWrite(RegWriteStatus),
    Action(ActionStatus),
    Reboot(RebootStatus),
    Error(ErrorStatus),
    FastSyncRead(FastSyncReadStatus<'a>),
    FastBulkRead(FastBulkReadStatus<'a>),
    Ext(S::Variant<'a>),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::BROADCAST_ID;

    fn raw(error: u8, params: &[u8]) -> RawStatus<'_> {
        RawStatus {
            id: BROADCAST_ID,
            error,
            params: Bytes::unstuffed(params),
        }
    }

    #[test]
    fn sync_slots_walks_uniform_length_payload() {
        // 3 slots, length=4 each. Slot 0 error in raw.error; slots 1+ carry
        // their own error byte before id.
        // payload = id0 + data0(4) + err1 + id1 + data1(4) + err2 + id2 + data2(4)
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, // slot 0
            0x02, 20, 0xB0, 0xB1, 0xB2, 0xB3, // slot 1
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3, // slot 2
        ];
        let s = FastSyncReadStatus::from_raw(raw(0x01, &payload));
        let slots: heapless::Vec<(u8, u8, [u8; 4]), 8> = s
            .slots(4)
            .map(|slot| {
                let mut buf = [0u8; 4];
                slot.data.copy_to_slice(&mut buf).unwrap();
                (slot.id, slot.error, buf)
            })
            .collect();
        assert_eq!(slots.len(), 3);
        assert_eq!(slots[0], (10, 0x01, [0xA0, 0xA1, 0xA2, 0xA3]));
        assert_eq!(slots[1], (20, 0x02, [0xB0, 0xB1, 0xB2, 0xB3]));
        assert_eq!(slots[2], (30, 0x00, [0xC0, 0xC1, 0xC2, 0xC3]));
    }

    #[test]
    fn sync_slots_stops_on_truncated_payload() {
        // Slot 1 declared but missing its data bytes.
        let payload = [10, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 20, 0xB0];
        let s = FastSyncReadStatus::from_raw(raw(0x00, &payload));
        let count = s.slots(4).count();
        assert_eq!(count, 1, "second slot is truncated; iter should stop");
    }

    #[test]
    fn sync_slots_handles_zero_length_data() {
        // Two slots with length=0; each contributes just (err, id) bytes.
        let payload = [10, 0x00, 20];
        let s = FastSyncReadStatus::from_raw(raw(0x00, &payload));
        let slots: heapless::Vec<(u8, u8), 4> = s.slots(0).map(|s| (s.id, s.error)).collect();
        assert_eq!(&slots[..], &[(10, 0x00), (20, 0x00)]);
    }

    #[test]
    fn bulk_slots_walks_varying_length_payload() {
        // Slot 0: id=10, len=4. Slot 1: id=20, len=2. Slot 2: id=30, len=6.
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, // slot 0 (4-byte data)
            0x02, 20, 0xB0, 0xB1, // slot 1 (2-byte data)
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, // slot 2 (6-byte data)
        ];
        let s = FastBulkReadStatus::from_raw(raw(0x05, &payload));
        let lengths = [4u16, 2, 6];
        let mut collected: heapless::Vec<(u8, u8, u8), 8> = heapless::Vec::new();
        for slot in s.slots(lengths.iter().copied()) {
            let first = slot.data.iter().next().unwrap();
            collected.push((slot.id, slot.error, first)).unwrap();
        }
        assert_eq!(collected.len(), 3);
        assert_eq!(collected[0], (10, 0x05, 0xA0));
        assert_eq!(collected[1], (20, 0x02, 0xB0));
        assert_eq!(collected[2], (30, 0x00, 0xC0));
    }

    #[test]
    fn slot_data_can_straddle_ring_boundary() {
        // Construct a payload where slot 0's data crosses the head/tail cut.
        let buf = [10, 0xA0, 0xA1, 0xA2, 0xA3];
        let split_at = 3;
        let (head, tail) = buf.split_at(split_at);
        let payload = Bytes::unstuffed_split(head, tail);
        let s = FastSyncReadStatus {
            error: 0x00,
            payload,
        };
        let mut iter = s.slots(4);
        let slot = iter.next().unwrap();
        assert_eq!(slot.id, 10);
        let collected: heapless::Vec<u8, 8> = slot.data.iter().collect();
        assert_eq!(&collected[..], &[0xA0, 0xA1, 0xA2, 0xA3]);
        assert!(iter.next().is_none());
    }
}
