//! Payload streaming: opaque chunk emission over body bytes.

use crate::crc::CrcUmts;
use crate::streaming::event::{InstructionPayload, PayloadEvent, StatusPayload};

#[derive(Copy, Clone)]
pub(crate) enum PayloadKind {
    Instruction,
    Status,
}

pub(crate) struct PayloadStage {
    kind: PayloadKind,
    remaining: u16,
}

impl PayloadStage {
    pub(crate) fn new(kind: PayloadKind, body_len: u16) -> Self {
        Self {
            kind,
            remaining: body_len,
        }
    }

    pub(crate) fn remaining(&self) -> u16 {
        self.remaining
    }

    /// Returns `(consumed, event)`; consumed bytes are CRC-folded.
    pub(crate) fn feed<CRC: CrcUmts>(
        &mut self,
        slice: &[u8],
        offset: u16,
        crc: &mut CRC,
    ) -> (u16, Option<PayloadEvent>) {
        let take = core::cmp::min(self.remaining as usize, slice.len());
        if take == 0 {
            return (0, None);
        }
        crc.update(&slice[..take]);
        self.remaining -= take as u16;
        let length = take as u16;
        let ev = match self.kind {
            PayloadKind::Instruction => {
                PayloadEvent::Instruction(InstructionPayload::WriteDataChunk { offset, length })
            }
            PayloadKind::Status => {
                PayloadEvent::Status(StatusPayload::ReadDataChunk { offset, length })
            }
        };
        (length, Some(ev))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;

    type Crc = SoftwareCrcUmts;

    #[test]
    fn full_body_in_one_call_emits_chunk_and_drains() {
        let mut s = PayloadStage::new(PayloadKind::Instruction, 4);
        let mut c = Crc::new();
        let (n, ev) = s.feed(&[1, 2, 3, 4], 7, &mut c);
        assert_eq!(n, 4);
        assert_eq!(s.remaining(), 0);
        assert!(matches!(
            ev,
            Some(PayloadEvent::Instruction(
                InstructionPayload::WriteDataChunk {
                    offset: 7,
                    length: 4
                }
            ))
        ));
    }

    #[test]
    fn slice_truncated_mid_body_emits_partial_chunk() {
        let mut s = PayloadStage::new(PayloadKind::Status, 6);
        let mut c = Crc::new();
        let (n, ev) = s.feed(&[0xAA, 0xBB], 0, &mut c);
        assert_eq!(n, 2);
        assert_eq!(s.remaining(), 4);
        assert!(matches!(
            ev,
            Some(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset: 0,
                length: 2
            }))
        ));
    }

    #[test]
    fn resumes_across_feed_calls() {
        let mut s = PayloadStage::new(PayloadKind::Instruction, 5);
        let mut c = Crc::new();
        let (n1, _) = s.feed(&[1, 2], 0, &mut c);
        assert_eq!((n1, s.remaining()), (2, 3));
        let (n2, _) = s.feed(&[3, 4, 5, 6, 7], 4, &mut c);
        assert_eq!((n2, s.remaining()), (3, 0));
    }

    #[test]
    fn zero_remaining_is_no_op() {
        let mut s = PayloadStage::new(PayloadKind::Status, 0);
        let mut c = Crc::new();
        let (n, ev) = s.feed(&[1, 2, 3], 0, &mut c);
        assert_eq!(n, 0);
        assert!(ev.is_none());
    }

    #[test]
    fn empty_slice_is_no_op() {
        let mut s = PayloadStage::new(PayloadKind::Instruction, 4);
        let mut c = Crc::new();
        let (n, ev) = s.feed(&[], 0, &mut c);
        assert_eq!(n, 0);
        assert!(ev.is_none());
        assert_eq!(s.remaining(), 4);
    }

    #[test]
    fn crc_folds_consumed_bytes_only() {
        let mut s = PayloadStage::new(PayloadKind::Instruction, 3);
        let mut c = Crc::new();
        let _ = s.feed(&[1, 2, 3, 99, 100], 0, &mut c);

        let mut expected = Crc::new();
        expected.update(&[1, 2, 3]);
        assert_eq!(c.finalize(), expected.finalize());
    }
}
