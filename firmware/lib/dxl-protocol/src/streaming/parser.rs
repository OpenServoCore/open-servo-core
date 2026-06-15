//! Streaming parser scalar state and [`EventStream`] iterator.
//!
//! The skeleton emits nothing; commits 3-7 fill in sync, header decode,
//! payload streaming, slot demarcation, CRC verdict, and resync.

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
    fn skeleton_emits_nothing_on_any_input() {
        let mut p: Parser<Crc> = Parser::new();
        let mut events = p.feed(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01]);
        assert!(events.next().is_none());
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
