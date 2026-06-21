//! Shared helpers for unit tests inside the crate.

extern crate alloc;

use alloc::vec::Vec as AVec;

use crate::crc::SoftwareCrcUmts;
use crate::streaming::{
    Event, HeaderEvent, InstructionHeader, InstructionPayload, Parser, PayloadEvent, StatusHeader,
    StatusPayload,
};
use crate::types::Id;

pub type Crc = SoftwareCrcUmts;

pub fn parse(wire: &[u8]) -> AVec<Event> {
    let mut p: Parser<Crc> = Parser::new();
    p.feed(wire).collect()
}

pub fn instruction_header(events: &[Event]) -> InstructionHeader {
    events
        .iter()
        .find_map(|e| match e {
            Event::Header(HeaderEvent::Instruction(h)) => Some(*h),
            _ => None,
        })
        .expect("expected instruction header event")
}

pub fn status_header(events: &[Event]) -> StatusHeader {
    events
        .iter()
        .find_map(|e| match e {
            Event::Header(HeaderEvent::Status(h)) => Some(*h),
            _ => None,
        })
        .expect("expected status header event")
}

pub fn assert_crc_good(events: &[Event]) {
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Crc(crate::streaming::CrcResult::Good))),
        "no Crc(Good) verdict in events: {events:?}"
    );
}

pub fn write_chunks(events: &[Event]) -> AVec<(u16, u16)> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                offset,
                length,
            })) => Some((*offset, *length)),
            _ => None,
        })
        .collect()
}

pub fn read_chunks(events: &[Event]) -> AVec<(u16, u16)> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) => Some((*offset, *length)),
            _ => None,
        })
        .collect()
}

/// Concatenated `ReadDataChunk` bytes from `wire`. Assumes unstuffed input
/// (callers using stuffed payloads should pull `read_chunks` and unstuff).
pub fn read_data(wire: &[u8], events: &[Event]) -> AVec<u8> {
    let mut out = AVec::new();
    for (offset, length) in read_chunks(events) {
        out.extend_from_slice(&wire[offset as usize..(offset + length) as usize]);
    }
    out
}

pub fn sync_slots(events: &[Event]) -> AVec<(Id, u8)> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Instruction(InstructionPayload::SyncSlot {
                id,
                index,
            })) => Some((*id, *index)),
            _ => None,
        })
        .collect()
}

pub fn bulk_slots(events: &[Event]) -> AVec<(Id, u8, u16, u16)> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                id,
                index,
                address,
                length,
            })) => Some((*id, *index, *address, *length)),
            _ => None,
        })
        .collect()
}
