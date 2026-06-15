//! Per-slot walks for SyncRead/SyncWrite/BulkRead/BulkWrite and their Fast
//! variants. Emits `SyncSlot`/`BulkSlot` per slot header, interleaved with
//! `WriteDataChunk` for data-carrying variants.

use crate::crc::CrcUmts;
use crate::streaming::event::{InstructionPayload, PayloadEvent};
use crate::types::Id;

#[cfg(test)]
extern crate alloc;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum SlotPattern {
    /// 1-byte slot (id); no data. SyncRead, FastSyncRead.
    SyncRead,
    /// 1-byte slot (id) + shared-`length` data. SyncWrite.
    SyncWrite { length: u16 },
    /// 5-byte slot (id, addr_le, len_le); no data. BulkRead, FastBulkRead.
    BulkRead,
    /// 5-byte slot (id, addr_le, len_le) + per-slot data. BulkWrite.
    BulkWrite,
}

impl SlotPattern {
    fn header_bytes(self) -> u8 {
        match self {
            Self::SyncRead | Self::SyncWrite { .. } => 1,
            Self::BulkRead | Self::BulkWrite => 5,
        }
    }
}

#[derive(Copy, Clone)]
enum SlotState {
    Header { buf: [u8; 5], cursor: u8 },
    Data { remaining: u16 },
}

impl SlotState {
    fn fresh_header() -> Self {
        Self::Header {
            buf: [0; 5],
            cursor: 0,
        }
    }
}

pub(crate) struct SlotsStage {
    pattern: SlotPattern,
    body_remaining: u16,
    slot_index: u8,
    state: SlotState,
}

impl SlotsStage {
    pub(crate) fn new(pattern: SlotPattern, body_len: u16) -> Self {
        Self {
            pattern,
            body_remaining: body_len,
            slot_index: 0,
            state: SlotState::fresh_header(),
        }
    }

    pub(crate) fn body_remaining(&self) -> u16 {
        self.body_remaining
    }

    /// `(consumed, event)`; consumed bytes are CRC-folded. One event per
    /// call: slot demarcation on header completion, or `WriteDataChunk`
    /// mid-data.
    pub(crate) fn feed<CRC: CrcUmts>(
        &mut self,
        slice: &[u8],
        offset: u16,
        crc: &mut CRC,
    ) -> (u16, Option<PayloadEvent>) {
        let avail = core::cmp::min(slice.len(), self.body_remaining as usize);
        if avail == 0 {
            return (0, None);
        }
        match self.state {
            SlotState::Header {
                mut buf,
                mut cursor,
            } => {
                let needed = self.pattern.header_bytes();
                let b = slice[0];
                crc.update(&[b]);
                buf[cursor as usize] = b;
                cursor += 1;
                self.body_remaining -= 1;
                if cursor < needed {
                    self.state = SlotState::Header { buf, cursor };
                    return (1, None);
                }
                let (ev, data_len) = build_slot_event(self.pattern, self.slot_index, &buf);
                self.slot_index = self.slot_index.wrapping_add(1);
                self.state = if data_len > 0 {
                    SlotState::Data {
                        remaining: data_len,
                    }
                } else {
                    SlotState::fresh_header()
                };
                (1, Some(ev))
            }
            SlotState::Data { remaining } => {
                let take = core::cmp::min(remaining as usize, avail) as u16;
                crc.update(&slice[..take as usize]);
                self.body_remaining -= take;
                let new_remaining = remaining - take;
                self.state = if new_remaining > 0 {
                    SlotState::Data {
                        remaining: new_remaining,
                    }
                } else {
                    SlotState::fresh_header()
                };
                let ev = PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                    offset,
                    length: take,
                });
                (take, Some(ev))
            }
        }
    }
}

fn build_slot_event(pattern: SlotPattern, index: u8, buf: &[u8; 5]) -> (PayloadEvent, u16) {
    let id = Id::new(buf[0]);
    match pattern {
        SlotPattern::SyncRead => (
            PayloadEvent::Instruction(InstructionPayload::SyncSlot { id, index }),
            0,
        ),
        SlotPattern::SyncWrite { length } => (
            PayloadEvent::Instruction(InstructionPayload::SyncSlot { id, index }),
            length,
        ),
        SlotPattern::BulkRead => {
            let address = u16::from_le_bytes([buf[1], buf[2]]);
            let length = u16::from_le_bytes([buf[3], buf[4]]);
            (
                PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                    id,
                    index,
                    address,
                    length,
                }),
                0,
            )
        }
        SlotPattern::BulkWrite => {
            let address = u16::from_le_bytes([buf[1], buf[2]]);
            let length = u16::from_le_bytes([buf[3], buf[4]]);
            (
                PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                    id,
                    index,
                    address,
                    length,
                }),
                length,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::SoftwareCrcUmts;

    type Crc = SoftwareCrcUmts;

    fn slot_events<CRC: CrcUmts>(
        s: &mut SlotsStage,
        slice: &[u8],
        crc: &mut CRC,
    ) -> alloc::vec::Vec<(u16, PayloadEvent)> {
        let mut out = alloc::vec::Vec::new();
        let mut off = 0u16;
        while (off as usize) < slice.len() && s.body_remaining() > 0 {
            let (n, ev) = s.feed(&slice[off as usize..], off, crc);
            if let Some(ev) = ev {
                out.push((off, ev));
            }
            if n == 0 {
                break;
            }
            off += n;
        }
        out
    }

    #[test]
    fn sync_read_single_slot_emits_one_event() {
        let mut s = SlotsStage::new(SlotPattern::SyncRead, 1);
        let mut c = Crc::new();
        let evs = slot_events(&mut s, &[0x42], &mut c);
        assert_eq!(s.body_remaining(), 0);
        assert_eq!(evs.len(), 1);
        assert!(matches!(
            evs[0].1,
            PayloadEvent::Instruction(InstructionPayload::SyncSlot { id, index: 0 })
                if id == Id::new(0x42)
        ));
    }

    #[test]
    fn sync_read_multi_slot_walks_index() {
        let mut s = SlotsStage::new(SlotPattern::SyncRead, 3);
        let mut c = Crc::new();
        let evs = slot_events(&mut s, &[0x11, 0x22, 0x33], &mut c);
        assert_eq!(evs.len(), 3);
        for (i, want_id) in [0x11, 0x22, 0x33].iter().enumerate() {
            match evs[i].1 {
                PayloadEvent::Instruction(InstructionPayload::SyncSlot { id, index }) => {
                    assert_eq!(id, Id::new(*want_id));
                    assert_eq!(index as usize, i);
                }
                other => panic!("expected SyncSlot, got {other:?}"),
            }
        }
    }

    #[test]
    fn bulk_read_decodes_addr_and_length_le() {
        // id=0x07 addr=0x0084 len=0x0010
        let body = [0x07, 0x84, 0x00, 0x10, 0x00];
        let mut s = SlotsStage::new(SlotPattern::BulkRead, body.len() as u16);
        let mut c = Crc::new();
        let evs = slot_events(&mut s, &body, &mut c);
        assert_eq!(evs.len(), 1);
        match evs[0].1 {
            PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                id,
                index,
                address,
                length,
            }) => {
                assert_eq!(id, Id::new(0x07));
                assert_eq!(index, 0);
                assert_eq!(address, 0x0084);
                assert_eq!(length, 0x0010);
            }
            other => panic!("expected BulkSlot, got {other:?}"),
        }
    }

    #[test]
    fn sync_write_interleaves_slot_and_data() {
        // 2 slots, shared length=4: id1 + 4B data + id2 + 4B data.
        let body = [0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0x02, 0xEE, 0xFF, 0x10, 0x20];
        let mut s = SlotsStage::new(SlotPattern::SyncWrite { length: 4 }, body.len() as u16);
        let mut c = Crc::new();
        let evs = slot_events(&mut s, &body, &mut c);
        assert_eq!(evs.len(), 4);
        assert!(matches!(
            evs[0].1,
            PayloadEvent::Instruction(InstructionPayload::SyncSlot { id, index: 0 })
                if id == Id::new(0x01)
        ));
        assert!(matches!(
            evs[1].1,
            PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                offset: 1,
                length: 4
            })
        ));
        assert!(matches!(
            evs[2].1,
            PayloadEvent::Instruction(InstructionPayload::SyncSlot { id, index: 1 })
                if id == Id::new(0x02)
        ));
        assert!(matches!(
            evs[3].1,
            PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                offset: 6,
                length: 4
            })
        ));
    }

    #[test]
    fn bulk_write_uses_per_slot_data_length() {
        // slot 0: id=1 addr=0x0080 len=2 data=AABB; slot 1: id=2 addr=0x0090 len=1 data=CC
        let body = [
            0x01, 0x80, 0x00, 0x02, 0x00, 0xAA, 0xBB, 0x02, 0x90, 0x00, 0x01, 0x00, 0xCC,
        ];
        let mut s = SlotsStage::new(SlotPattern::BulkWrite, body.len() as u16);
        let mut c = Crc::new();
        let evs = slot_events(&mut s, &body, &mut c);
        assert_eq!(evs.len(), 4);
        assert!(matches!(
            evs[0].1,
            PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                id,
                index: 0,
                address: 0x0080,
                length: 2,
            }) if id == Id::new(0x01)
        ));
        assert!(matches!(
            evs[1].1,
            PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                offset: 5,
                length: 2
            })
        ));
        assert!(matches!(
            evs[2].1,
            PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                id,
                index: 1,
                address: 0x0090,
                length: 1,
            }) if id == Id::new(0x02)
        ));
        assert!(matches!(
            evs[3].1,
            PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                offset: 12,
                length: 1
            })
        ));
    }

    #[test]
    fn chunked_feed_resumes_mid_data() {
        // SyncWrite length=4, 1 slot; split id+2B then 2B.
        let mut s = SlotsStage::new(SlotPattern::SyncWrite { length: 4 }, 5);
        let mut c = Crc::new();
        let (n0, ev0) = s.feed(&[0x07, 0x11, 0x22], 0, &mut c);
        assert_eq!(n0, 1);
        assert!(matches!(
            ev0,
            Some(PayloadEvent::Instruction(InstructionPayload::SyncSlot {
                id,
                index: 0
            })) if id == Id::new(0x07)
        ));
        let (n1, ev1) = s.feed(&[0x11, 0x22], 1, &mut c);
        assert_eq!(n1, 2);
        assert!(matches!(
            ev1,
            Some(PayloadEvent::Instruction(
                InstructionPayload::WriteDataChunk {
                    offset: 1,
                    length: 2
                }
            ))
        ));
        let (n2, ev2) = s.feed(&[0x33, 0x44], 0, &mut c);
        assert_eq!(n2, 2);
        assert!(matches!(
            ev2,
            Some(PayloadEvent::Instruction(
                InstructionPayload::WriteDataChunk {
                    offset: 0,
                    length: 2
                }
            ))
        ));
        assert_eq!(s.body_remaining(), 0);
    }

    #[test]
    fn chunked_feed_resumes_mid_bulk_header() {
        // 5-byte BulkRead entry, byte-at-a-time.
        let mut s = SlotsStage::new(SlotPattern::BulkRead, 5);
        let mut c = Crc::new();
        let (n0, ev0) = s.feed(&[0x07, 0x84], 0, &mut c);
        assert_eq!(n0, 1);
        assert!(ev0.is_none());
        let (n1, ev1) = s.feed(&[0x84], 0, &mut c);
        assert_eq!(n1, 1);
        assert!(ev1.is_none());
        let (n2, ev2) = s.feed(&[0x00, 0x04, 0x00], 0, &mut c);
        assert_eq!(n2, 1);
        assert!(ev2.is_none());
        let (n3, ev3) = s.feed(&[0x04, 0x00], 0, &mut c);
        assert_eq!(n3, 1);
        assert!(ev3.is_none());
        let (n4, ev4) = s.feed(&[0x00], 0, &mut c);
        assert_eq!(n4, 1);
        match ev4 {
            Some(PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                id,
                address,
                length,
                ..
            })) => {
                assert_eq!(id, Id::new(0x07));
                assert_eq!(address, 0x0084);
                assert_eq!(length, 0x0004);
            }
            other => panic!("expected BulkSlot, got {other:?}"),
        }
        assert_eq!(s.body_remaining(), 0);
    }

    #[test]
    fn empty_slice_is_no_op() {
        let mut s = SlotsStage::new(SlotPattern::SyncRead, 3);
        let mut c = Crc::new();
        let (n, ev) = s.feed(&[], 0, &mut c);
        assert_eq!(n, 0);
        assert!(ev.is_none());
        assert_eq!(s.body_remaining(), 3);
    }

    #[test]
    fn crc_folds_every_consumed_byte() {
        let body = [0x01, 0xAA, 0xBB, 0x02, 0xCC, 0xDD];
        let mut s = SlotsStage::new(SlotPattern::SyncWrite { length: 2 }, body.len() as u16);
        let mut c = Crc::new();
        let _ = slot_events(&mut s, &body, &mut c);

        let mut expected = Crc::new();
        expected.update(&body);
        assert_eq!(c.finalize(), expected.finalize());
    }
}
