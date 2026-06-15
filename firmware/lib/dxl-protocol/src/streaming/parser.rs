//! Streaming parser orchestrator and [`EventStream`] iterator.

use crate::crc::CrcUmts;

use super::Event;
use super::stage::{HeaderStage, SyncStage};

enum Phase {
    Sync(SyncStage),
    Header(HeaderStage),
    Payload,
    Crc,
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
                self.phase = Phase::Payload;
                Some(Event::Header(ev))
            }
            Phase::Payload | Phase::Crc | Phase::Done => None,
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
            if matches!(self.parser.phase, Phase::Payload | Phase::Crc | Phase::Done) {
                return None;
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
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;
    use crate::packet::Instruction;
    use crate::streaming::event::HeaderEvent;

    type Crc = SoftwareCrcUmts;

    fn in_sync<C: CrcUmts>(p: &Parser<C>) -> bool {
        matches!(p.phase, Phase::Sync(_))
    }
    fn in_header<C: CrcUmts>(p: &Parser<C>) -> bool {
        matches!(p.phase, Phase::Header(_))
    }
    fn in_payload<C: CrcUmts>(p: &Parser<C>) -> bool {
        matches!(p.phase, Phase::Payload)
    }

    #[test]
    fn empty_input_emits_nothing() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[]);
        assert!(events.next().is_none());
        assert_eq!(events.consumed(), 0);
    }

    #[test]
    fn sync_completion_emits_event_and_advances_to_header() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[0xFF, 0xFF, 0xFD, 0x00]);
        assert!(matches!(events.next(), Some(Event::Sync)));
        assert_eq!(events.consumed(), 4);
        assert!(events.next().is_none());
        assert!(in_header(&p));
    }

    #[test]
    fn header_completion_emits_event_and_advances_to_payload() {
        let bytes = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x07,
            0x03,
            0x00,
            Instruction::Ping.as_u8(),
        ];
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&bytes);
        assert!(matches!(events.next(), Some(Event::Sync)));
        assert!(matches!(
            events.next(),
            Some(Event::Header(HeaderEvent::Instruction(_)))
        ));
        assert_eq!(events.consumed(), bytes.len());
        assert!(in_payload(&p));
    }

    #[test]
    fn chunked_input_resumes_across_feed_calls() {
        let mut p: Parser<Crc> = Parser::new();
        {
            let mut e1 = p.feed(&[0xFF, 0xFF]);
            assert!(e1.next().is_none());
            assert_eq!(e1.consumed(), 2);
        }
        let mut e2 = p.feed(&[0xFD, 0x00]);
        assert!(matches!(e2.next(), Some(Event::Sync)));
        assert_eq!(e2.consumed(), 2);
    }

    #[test]
    fn payload_phase_emits_no_events() {
        let bytes = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x07,
            0x03,
            0x00,
            Instruction::Ping.as_u8(),
        ];
        let mut p: Parser<Crc> = Parser::new();
        let _ = p.feed(&bytes).count();
        assert!(in_payload(&p));

        let mut events = p.feed(&[0xAA, 0xBB, 0xCC]);
        assert!(events.next().is_none());
        assert_eq!(events.consumed(), 0);
    }

    #[test]
    fn reset_returns_to_sync_phase() {
        let mut p: Parser<Crc> = Parser::new();
        let _ = p.feed(&[0xFF, 0xFF, 0xFD, 0x00]).count();
        assert!(in_header(&p));
        p.reset();
        assert!(in_sync(&p));
    }
}
