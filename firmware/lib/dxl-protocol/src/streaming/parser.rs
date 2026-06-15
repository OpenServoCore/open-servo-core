//! Streaming parser scalar state and [`EventStream`] iterator.

use crate::constants::HEADER;
use crate::crc::CrcUmts;
use crate::packet::Id;

use super::Event;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum State {
    Sync,
    Header,
    Payload,
    Crc,
    Done,
}

pub struct Parser<CRC: CrcUmts> {
    state: State,
    sync_matched: u8,
    cursor: u16,
    header_id: Id,
    header_length: u16,
    header_instr: u8,
    header_error: u8,
    crc: CRC,
}

impl<CRC: CrcUmts> Parser<CRC> {
    pub fn new() -> Self {
        Self {
            state: State::Sync,
            sync_matched: 0,
            cursor: 0,
            header_id: Id::new(0),
            header_length: 0,
            header_instr: 0,
            header_error: 0,
            crc: CRC::new(),
        }
    }

    pub fn reset(&mut self) {
        self.state = State::Sync;
        self.sync_matched = 0;
        self.cursor = 0;
        self.header_id = Id::new(0);
        self.header_length = 0;
        self.header_instr = 0;
        self.header_error = 0;
        self.crc.reset();
    }

    pub fn feed<'p>(&'p mut self, bytes: &'p [u8]) -> EventStream<'p, CRC> {
        EventStream {
            parser: self,
            bytes,
            offset: 0,
        }
    }

    // KMP backoff for FF FF FD 00 (f[1]=0, f[2]=1, f[3]=0) -- locks
    // `FF FF FF FD 00` onto the embedded header at offset 1.
    fn step_sync(&mut self, b: u8) -> Option<Event> {
        let m = self.sync_matched as usize;
        if b == HEADER[m] {
            let new_m = m + 1;
            if new_m == HEADER.len() {
                self.crc.update(&HEADER);
                self.sync_matched = 0;
                self.state = State::Header;
                return Some(Event::Sync);
            }
            self.sync_matched = new_m as u8;
        } else {
            match m {
                2 if b == HEADER[0] => {}
                3 if b == HEADER[0] => self.sync_matched = 1,
                _ => self.sync_matched = if b == HEADER[0] { 1 } else { 0 },
            }
        }
        None
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
    /// Bytes consumed from the input slice so far.
    pub fn consumed(&self) -> usize {
        self.offset
    }
}

impl<'p, CRC: CrcUmts> Iterator for EventStream<'p, CRC> {
    type Item = Event;

    fn next(&mut self) -> Option<Event> {
        while self.offset < self.bytes.len() {
            match self.parser.state {
                State::Sync => {
                    let b = self.bytes[self.offset];
                    self.offset += 1;
                    if let Some(ev) = self.parser.step_sync(b) {
                        return Some(ev);
                    }
                }
                _ => return None,
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;

    type Crc = SoftwareCrcUmts;

    #[test]
    fn empty_input_emits_nothing() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[]);
        assert!(events.next().is_none());
        assert_eq!(events.consumed(), 0);
    }

    #[test]
    fn sync_emitted_on_exact_preamble() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[0xFF, 0xFF, 0xFD, 0x00]);
        assert!(matches!(events.next(), Some(Event::Sync)));
        assert_eq!(events.consumed(), 4);
        assert!(events.next().is_none());
    }

    #[test]
    fn sync_emitted_after_noise() {
        let mut p: Parser<Crc> = Parser::new();
        let bytes = [0x00, 0x12, 0xFF, 0x34, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0x00];
        let mut events = p.feed(&bytes);
        assert!(matches!(events.next(), Some(Event::Sync)));
        assert_eq!(events.consumed(), bytes.len());
    }

    #[test]
    fn sync_finds_embedded_after_extra_ff() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[0xFF, 0xFF, 0xFF, 0xFD, 0x00]);
        assert!(matches!(events.next(), Some(Event::Sync)));
        assert_eq!(events.consumed(), 5);
    }

    #[test]
    fn sync_falls_back_on_ff_after_fd() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0x00]);
        assert!(matches!(events.next(), Some(Event::Sync)));
        assert_eq!(events.consumed(), 7);
    }

    #[test]
    fn sync_partial_then_more() {
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
    fn sync_resets_with_reset() {
        let mut p: Parser<Crc> = Parser::new();
        let _ = p.feed(&[0xFF, 0xFF]).next();
        p.reset();
        let mut events = p.feed(&[0xFD, 0x00]);
        assert!(events.next().is_none());
    }

    #[test]
    fn sync_transitions_to_header_and_seeds_crc() {
        let mut p: Parser<Crc> = Parser::new();
        let _ = p.feed(&[0xFF, 0xFF, 0xFD, 0x00]).next();
        assert!(matches!(p.state, State::Header));
        assert_eq!(p.sync_matched, 0);

        let mut expected = Crc::new();
        expected.update(&HEADER);
        assert_eq!(p.crc.finalize(), expected.finalize());
    }

    #[test]
    fn reset_clears_state() {
        let mut p: Parser<Crc> = Parser::new();
        let _ = p.feed(&[0xFF, 0xFF]);
        p.reset();
        assert!(matches!(p.state, State::Sync));
        assert_eq!(p.cursor, 0);
    }
}
