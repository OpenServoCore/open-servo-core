//! Streaming parser orchestrator and [`EventStream`] iterator.

use crate::crc::CrcUmts;

use super::Event;
use super::event::HeaderEvent;
use super::stage::{CrcStage, HeaderStage, PayloadKind, PayloadStage, SyncStage};

enum Phase {
    Sync(SyncStage),
    Header(HeaderStage),
    Payload(PayloadStage),
    Crc(CrcStage),
    Done,
}

pub struct Parser<CRC: CrcUmts> {
    phase: Phase,
    crc: CRC,
}

impl<CRC: CrcUmts> Parser<CRC> {
    pub fn new() -> Self {
        Self {
            phase: Phase::Sync(SyncStage::new()),
            crc: CRC::new(),
        }
    }

    pub fn reset(&mut self) {
        self.phase = Phase::Sync(SyncStage::new());
        self.crc.reset();
    }

    pub fn feed<'p>(&'p mut self, bytes: &'p [u8]) -> EventStream<'p, CRC> {
        EventStream {
            parser: self,
            bytes,
            offset: 0,
        }
    }

    fn step(&mut self, b: u8) -> Option<Event> {
        match &mut self.phase {
            Phase::Sync(s) => {
                s.feed(b, &mut self.crc)?;
                self.phase = Phase::Header(HeaderStage::new());
                Some(Event::Sync)
            }
            Phase::Header(s) => {
                let ev = s.feed(b, &mut self.crc)?;
                let body_len = s.body_len();
                let kind = match ev {
                    HeaderEvent::Status(_) => PayloadKind::Status,
                    HeaderEvent::Instruction(_) => PayloadKind::Instruction,
                };
                self.phase = if body_len > 0 {
                    Phase::Payload(PayloadStage::new(kind, body_len))
                } else {
                    Phase::Crc(CrcStage::new())
                };
                Some(Event::Header(ev))
            }
            Phase::Crc(s) => {
                let result = s.feed(b, &self.crc)?;
                self.reset();
                Some(Event::Crc(result))
            }
            Phase::Payload(_) | Phase::Done => None,
        }
    }
}

impl<CRC: CrcUmts> Default for Parser<CRC> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct EventStream<'p, CRC: CrcUmts> {
    parser: &'p mut Parser<CRC>,
    bytes: &'p [u8],
    offset: usize,
}

impl<'p, CRC: CrcUmts> EventStream<'p, CRC> {
    pub fn consumed(&self) -> usize {
        self.offset
    }
}

impl<'p, CRC: CrcUmts> Iterator for EventStream<'p, CRC> {
    type Item = Event;

    fn next(&mut self) -> Option<Event> {
        while self.offset < self.bytes.len() {
            if matches!(self.parser.phase, Phase::Done) {
                return None;
            }
            if let Phase::Payload(ref mut s) = self.parser.phase {
                let chunk_start = self.offset as u16;
                let (consumed, ev) = s.feed(
                    &self.bytes[self.offset..],
                    chunk_start,
                    &mut self.parser.crc,
                );
                self.offset += consumed as usize;
                if s.remaining() == 0 {
                    self.parser.phase = Phase::Crc(CrcStage::new());
                }
                if let Some(ev) = ev {
                    return Some(Event::Payload(ev));
                }
                continue;
            }
            let b = self.bytes[self.offset];
            self.offset += 1;
            if let Some(ev) = self.parser.step(b) {
                return Some(ev);
            }
        }
        None
    }
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;
    use crate::packet::Instruction;
    use crate::streaming::event::{
        CrcResult, HeaderEvent, InstructionHeader, InstructionPayload, PayloadEvent,
    };
    use alloc::vec;
    use alloc::vec::Vec;

    type Crc = SoftwareCrcUmts;

    fn in_sync<C: CrcUmts>(p: &Parser<C>) -> bool {
        matches!(p.phase, Phase::Sync(_))
    }
    fn in_header<C: CrcUmts>(p: &Parser<C>) -> bool {
        matches!(p.phase, Phase::Header(_))
    }
    fn in_payload<C: CrcUmts>(p: &Parser<C>) -> bool {
        matches!(p.phase, Phase::Payload(_))
    }
    fn in_crc<C: CrcUmts>(p: &Parser<C>) -> bool {
        matches!(p.phase, Phase::Crc(_))
    }

    /// Wire-correct Write packet with trailing CRC.
    fn write_packet(id: u8, addr: u16, body: &[u8]) -> Vec<u8> {
        let wire_len = (1 + 2 + body.len() + 2) as u16;
        let mut buf = vec![0xFF, 0xFF, 0xFD, 0x00, id];
        buf.extend_from_slice(&wire_len.to_le_bytes());
        buf.push(Instruction::Write.as_u8());
        buf.extend_from_slice(&addr.to_le_bytes());
        buf.extend_from_slice(body);
        let mut c = Crc::new();
        c.update(&buf);
        let crc = c.finalize();
        buf.push(crc as u8);
        buf.push((crc >> 8) as u8);
        buf
    }

    fn ping_packet(id: u8) -> Vec<u8> {
        let mut buf = vec![
            0xFF,
            0xFF,
            0xFD,
            0x00,
            id,
            0x03,
            0x00,
            Instruction::Ping.as_u8(),
        ];
        let mut c = Crc::new();
        c.update(&buf);
        let crc = c.finalize();
        buf.push(crc as u8);
        buf.push((crc >> 8) as u8);
        buf
    }

    #[test]
    fn empty_input_emits_nothing() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[]);
        assert!(events.next().is_none());
        assert_eq!(events.consumed(), 0);
    }

    #[test]
    fn sync_completion_advances_to_header() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[0xFF, 0xFF, 0xFD, 0x00]);
        assert!(matches!(events.next(), Some(Event::Sync)));
        assert!(events.next().is_none());
        assert!(in_header(&p));
    }

    #[test]
    fn good_write_packet_emits_sync_header_chunk_and_good_crc() {
        let bytes = write_packet(0x03, 0x0050, &[0xAA, 0xBB, 0xCC, 0xDD]);
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&bytes);

        assert!(matches!(events.next(), Some(Event::Sync)));
        assert!(matches!(
            events.next(),
            Some(Event::Header(HeaderEvent::Instruction(
                InstructionHeader::Write { length: 4, .. }
            )))
        ));
        match events.next() {
            Some(Event::Payload(PayloadEvent::Instruction(
                InstructionPayload::WriteDataChunk { offset, length },
            ))) => {
                assert_eq!(length, 4);
                assert_eq!(
                    &bytes[offset as usize..(offset + length) as usize],
                    &[0xAA, 0xBB, 0xCC, 0xDD]
                );
            }
            other => panic!("expected WriteDataChunk, got {other:?}"),
        }
        assert_eq!(events.next(), Some(Event::Crc(CrcResult::Good)));
        assert_eq!(events.consumed(), bytes.len());
    }

    #[test]
    fn bad_crc_emits_bad_verdict() {
        let mut bytes = write_packet(0x03, 0x0050, &[1, 2, 3, 4]);
        let last = bytes.len() - 1;
        bytes[last] ^= 0xFF;
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&bytes);
        let verdict = events.find(|e| matches!(e, Event::Crc(_)));
        assert_eq!(verdict, Some(Event::Crc(CrcResult::Bad)));
    }

    #[test]
    fn zero_body_packet_skips_payload_phase() {
        let bytes = ping_packet(0x07);
        let mut p: Parser<Crc> = Parser::new();
        let events: Vec<Event> = p.feed(&bytes).collect();
        assert!(!events.iter().any(|e| matches!(e, Event::Payload(_))));
        assert_eq!(events.last(), Some(&Event::Crc(CrcResult::Good)));
    }

    #[test]
    fn chunked_feed_splits_payload_across_calls() {
        let bytes = write_packet(0x03, 0x0050, &[1, 2, 3, 4, 5, 6]);
        let mut p: Parser<Crc> = Parser::new();

        let mid = bytes.len() - 4;
        let mut got = Vec::new();
        got.extend(p.feed(&bytes[..mid]));
        got.extend(p.feed(&bytes[mid..]));

        let chunks: Vec<(u16, u16)> =
            got.iter()
                .filter_map(|e| match e {
                    Event::Payload(PayloadEvent::Instruction(
                        InstructionPayload::WriteDataChunk { offset, length },
                    )) => Some((*offset, *length)),
                    _ => None,
                })
                .collect();
        assert_eq!(chunks.len(), 2);
        assert_eq!(chunks.iter().map(|(_, l)| *l).sum::<u16>(), 6);
        assert_eq!(got.last(), Some(&Event::Crc(CrcResult::Good)));
    }

    #[test]
    fn crc_verdict_returns_parser_to_sync() {
        let bytes = ping_packet(0x05);
        let mut p: Parser<Crc> = Parser::new();
        let _ = p.feed(&bytes).count();
        assert!(in_sync(&p));
    }

    #[test]
    fn back_to_back_packets_stream_through() {
        let mut bytes = ping_packet(0x01);
        bytes.extend(ping_packet(0x02));
        let mut p: Parser<Crc> = Parser::new();
        let verdicts: Vec<Event> = p
            .feed(&bytes)
            .filter(|e| matches!(e, Event::Crc(_)))
            .collect();
        assert_eq!(
            verdicts,
            vec![Event::Crc(CrcResult::Good), Event::Crc(CrcResult::Good)]
        );
    }

    #[test]
    fn reset_returns_to_sync_phase() {
        let mut p: Parser<Crc> = Parser::new();
        let _ = p.feed(&[0xFF, 0xFF, 0xFD, 0x00]).count();
        assert!(in_header(&p));
        p.reset();
        assert!(in_sync(&p));
    }

    #[test]
    fn intermediate_phase_helpers_work() {
        let bytes = write_packet(0x03, 0x0050, &[1, 2]);
        let mut p: Parser<Crc> = Parser::new();
        let header_end = 4 + 7; // sync + (id + len + instr + 2 addr)
        let _ = p.feed(&bytes[..header_end]).count();
        assert!(in_payload(&p));
        let body_end = header_end + 2;
        let _ = p.feed(&bytes[header_end..body_end]).count();
        assert!(in_crc(&p));
    }
}
