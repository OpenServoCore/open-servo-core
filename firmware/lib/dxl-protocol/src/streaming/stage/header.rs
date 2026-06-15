//! Header decode stage: id, length, instruction, optional error byte, and
//! per-variant param bytes.

use crate::crc::CrcUmts;
use crate::packet::{Id, Instruction, StatusError};

use crate::streaming::event::{HeaderEvent, InstructionHeader, StatusHeader};

pub(crate) struct HeaderStage {
    cursor: u8,
    id: Id,
    length: u16,
    instr: u8,
    error: u8,
    params: [u8; 4],
}

impl HeaderStage {
    pub(crate) fn new() -> Self {
        Self {
            cursor: 0,
            id: Id::new(0),
            length: 0,
            instr: 0,
            error: 0,
            params: [0; 4],
        }
    }

    /// Cursor: 0=id, 1..=2=length_le16, 3=instr, then error (Status) or up
    /// to 4 params.
    pub(crate) fn feed<CRC: CrcUmts>(&mut self, b: u8, crc: &mut CRC) -> Option<HeaderEvent> {
        let c = self.cursor as usize;
        crc.update(&[b]);
        match c {
            0 => self.id = Id::new(b),
            1 => self.length = b as u16,
            2 => self.length |= (b as u16) << 8,
            3 => self.instr = b,
            _ => {
                let i = c - 4;
                if self.instr == Instruction::Status.as_u8() {
                    self.error = b;
                } else if i < self.params.len() {
                    self.params[i] = b;
                }
            }
        }
        self.cursor += 1;
        if c >= 3 && self.cursor as usize == self.total_bytes() {
            return Some(self.emit());
        }
        None
    }

    fn total_bytes(&self) -> usize {
        let kind = Instruction::from_u8(self.instr);
        if matches!(kind, Instruction::Status) {
            5
        } else {
            4 + kind.fixed_param_bytes()
        }
    }

    fn emit(&self) -> HeaderEvent {
        let kind = Instruction::from_u8(self.instr);
        if matches!(kind, Instruction::Status) {
            HeaderEvent::Status(StatusHeader {
                id: self.id,
                error: StatusError::from_byte(self.error),
                length: self.length.saturating_sub(4),
            })
        } else {
            HeaderEvent::Instruction(InstructionHeader::from_wire(
                kind,
                self.id,
                self.length,
                &self.params,
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;

    type Crc = SoftwareCrcUmts;

    fn feed_all(h: &mut HeaderStage, crc: &mut Crc, bytes: &[u8]) -> Option<HeaderEvent> {
        let mut last = None;
        for &b in bytes {
            last = h.feed(b, crc);
        }
        last
    }

    #[test]
    fn ping_decodes() {
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [0x07, 0x03, 0x00, Instruction::Ping.as_u8()];
        match feed_all(&mut h, &mut c, &bytes) {
            Some(HeaderEvent::Instruction(InstructionHeader::Ping { id })) => {
                assert_eq!(id, Id::new(0x07));
            }
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn factory_reset_carries_mode() {
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [0x05, 0x04, 0x00, Instruction::FactoryReset.as_u8(), 0xFF];
        match feed_all(&mut h, &mut c, &bytes) {
            Some(HeaderEvent::Instruction(InstructionHeader::FactoryReset { id, mode })) => {
                assert_eq!(id, Id::new(0x05));
                assert_eq!(mode, 0xFF);
            }
            other => panic!("expected FactoryReset, got {other:?}"),
        }
    }

    #[test]
    fn read_carries_addr_and_length() {
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [
            0x07,
            0x07,
            0x00,
            Instruction::Read.as_u8(),
            0x84,
            0x00,
            0x04,
            0x00,
        ];
        match feed_all(&mut h, &mut c, &bytes) {
            Some(HeaderEvent::Instruction(InstructionHeader::Read {
                id,
                address,
                length,
            })) => {
                assert_eq!(id, Id::new(0x07));
                assert_eq!(address, 0x0084);
                assert_eq!(length, 4);
            }
            other => panic!("expected Read, got {other:?}"),
        }
    }

    #[test]
    fn write_derives_body_length_from_wire() {
        // wire_len = 1(instr) + 2(addr) + 4(data) + 2(crc) = 9
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [0x03, 0x09, 0x00, Instruction::Write.as_u8(), 0x50, 0x00];
        match feed_all(&mut h, &mut c, &bytes) {
            Some(HeaderEvent::Instruction(InstructionHeader::Write {
                id,
                address,
                length,
            })) => {
                assert_eq!(id, Id::new(0x03));
                assert_eq!(address, 0x0050);
                assert_eq!(length, 4);
            }
            other => panic!("expected Write, got {other:?}"),
        }
    }

    #[test]
    fn sync_write_uses_param_length() {
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [
            0xFE,
            0x0F,
            0x00,
            Instruction::SyncWrite.as_u8(),
            0x50,
            0x00,
            0x03,
            0x00,
        ];
        match feed_all(&mut h, &mut c, &bytes) {
            Some(HeaderEvent::Instruction(InstructionHeader::SyncWrite {
                id,
                address,
                length,
            })) => {
                assert_eq!(id, Id::new(0xFE));
                assert_eq!(address, 0x0050);
                assert_eq!(length, 3);
            }
            other => panic!("expected SyncWrite, got {other:?}"),
        }
    }

    #[test]
    fn status_carries_error_and_body_length() {
        // wire_len = 1(instr) + 1(error) + 4(body) + 2(crc) = 8 -> body=4
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [0x01, 0x08, 0x00, Instruction::Status.as_u8(), 0x07];
        match feed_all(&mut h, &mut c, &bytes) {
            Some(HeaderEvent::Status(s)) => {
                assert_eq!(s.id, Id::new(0x01));
                assert_eq!(s.error, StatusError::from_byte(0x07));
                assert_eq!(s.length, 4);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn raw_for_unknown_instruction() {
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [0x01, 0x05, 0x00, 0x99];
        match feed_all(&mut h, &mut c, &bytes) {
            Some(HeaderEvent::Instruction(InstructionHeader::Raw { id, instr, length })) => {
                assert_eq!(id, Id::new(0x01));
                assert_eq!(instr, 0x99);
                assert_eq!(length, 2);
            }
            other => panic!("expected Raw, got {other:?}"),
        }
    }

    #[test]
    fn crc_updates_with_every_header_byte() {
        let (mut h, mut c) = (HeaderStage::new(), Crc::new());
        let bytes = [
            0x07,
            0x07,
            0x00,
            Instruction::Read.as_u8(),
            0x84,
            0x00,
            0x04,
            0x00,
        ];
        let _ = feed_all(&mut h, &mut c, &bytes);

        let mut expected = Crc::new();
        expected.update(&bytes);
        assert_eq!(c.finalize(), expected.finalize());
    }
}
