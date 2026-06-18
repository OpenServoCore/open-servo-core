//! Test helpers for rendering raw wire bytes and decoding Status replies.
//!
//! Integration tests snapshot the raw byte stream with [`format_hex`] (per
//! `feedback_snapshot_vs_assertion`) and assert the decoded shape with
//! [`parse_status`] returning a `Status<'_>`.

use core::fmt::Write;

use dxl_protocol::crc::SoftwareCrcUmts;
use dxl_protocol::streaming::{
    Event, HeaderEvent, Parser, PayloadEvent, StatusHeader, StatusPayload,
};
use dxl_protocol::types::{BulkReadEntry, Id, Instruction, PingStatus, Slot, Status, StatusError};

pub fn format_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 3);
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            out.push(' ');
        }
        write!(out, "{b:02x}").unwrap();
    }
    out
}

/// Decodes a single Status reply on the wire into a typed [`Status<'a>`].
///
/// The caller passes the instruction the host originally sent (`req`) because
/// a DXL Status reply carries no instruction byte — its body is opaque to the
/// parser, and the master must know what was sent to interpret what came back.
pub fn parse_status(req: Instruction, wire: &[u8]) -> Status<'_> {
    let mut iter = parse_status_stream(req, wire).into_iter();
    let first = iter.next().expect("parse_status: no Status reply in wire");
    assert!(
        iter.next().is_none(),
        "parse_status: more than one Status reply on wire; use parse_status_stream",
    );
    first
}

/// Decodes every consecutive Status reply on the wire (back-to-back broadcast
/// replies, multi-servo Sync/Bulk responses, etc.) in the order they appear.
pub fn parse_status_stream(req: Instruction, wire: &[u8]) -> Vec<Status<'_>> {
    let mut parser: Parser<SoftwareCrcUmts> = Parser::new();
    let events: Vec<Event> = parser.feed(wire).collect();

    let mut replies = Vec::new();
    let mut pending_header: Option<StatusHeader> = None;
    let mut pending_body: Option<&[u8]> = None;
    for ev in &events {
        match ev {
            Event::Header(HeaderEvent::Status(h)) => pending_header = Some(*h),
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) => {
                pending_body = Some(&wire[*offset as usize..(*offset + *length) as usize]);
            }
            Event::Crc => {
                if let Some(header) = pending_header.take() {
                    replies.push(decode_one(req, header, pending_body.take()));
                }
            }
            _ => {}
        }
    }
    replies
}

fn decode_one<'a>(req: Instruction, header: StatusHeader, body: Option<&'a [u8]>) -> Status<'a> {
    match req {
        Instruction::Ping => {
            let body = body.expect("parse_status: Ping reply missing payload");
            assert_eq!(body.len(), 3, "Ping reply body must be 3 bytes");
            Status::Ping {
                id: header.id,
                error: header.error,
                status: PingStatus {
                    model: u16::from_le_bytes([body[0], body[1]]),
                    fw_version: body[2],
                },
            }
        }
        Instruction::Read | Instruction::SyncRead | Instruction::BulkRead => Status::Read {
            id: header.id,
            error: header.error,
            data: body.unwrap_or(&[]),
        },
        Instruction::Write
        | Instruction::RegWrite
        | Instruction::Action
        | Instruction::SyncWrite
        | Instruction::BulkWrite
        | Instruction::Reboot
        | Instruction::FactoryReset
        | Instruction::Clear
        | Instruction::ControlTableBackup
        | Instruction::Ext(_) => Status::Empty {
            id: header.id,
            error: header.error,
        },
        other => panic!("parse_status: instruction {other:?} not yet supported"),
    }
}

/// Decode a Fast Sync Read coalesced reply into one [`Slot`] per chain slot.
///
/// The streaming parser collapses a Fast chain into one `Status` header + one
/// or more `ReadDataChunk` events because the wire IS one Status packet
/// (broadcast id, single CRC). To recover per-slot views, this walker uses
/// the host-known `length` (shared across slots in Sync Read) to slice the
/// concatenated body: slot 0 hoists its `error` from the header (the wire's
/// status-error byte position IS slot 0's error per [`SlotEncoder`]); slot
/// k>0 reads its own `[err, id, data...]` from body bytes.
///
/// Returns an empty Vec if no chain landed (no header, malformed CRC).
///
/// [`SlotEncoder`]: dxl_protocol::SlotEncoder
pub fn parse_fast_sync_chain(wire: &[u8], length: u16) -> Vec<Slot<'_>> {
    let (header, body) = match fast_chain_header_and_body(wire) {
        Some(parts) => parts,
        None => return Vec::new(),
    };
    walk_fast_slots(header, body, |_| length as usize)
}

/// Decode a Fast Bulk Read coalesced reply. Per-slot length comes from the
/// host-side `entries` list (one per slot, in chain order). See
/// [`parse_fast_sync_chain`] for the wire-walk model.
pub fn parse_fast_bulk_chain<'a>(wire: &'a [u8], entries: &[BulkReadEntry]) -> Vec<Slot<'a>> {
    let (header, body) = match fast_chain_header_and_body(wire) {
        Some(parts) => parts,
        None => return Vec::new(),
    };
    walk_fast_slots(header, body, |idx| entries[idx].length as usize)
}

fn fast_chain_header_and_body(wire: &[u8]) -> Option<(StatusHeader, &[u8])> {
    let mut parser: Parser<SoftwareCrcUmts> = Parser::new();
    let events: Vec<Event> = parser.feed(wire).collect();

    let mut header = None;
    let mut body_start = None;
    let mut body_end = None;
    for ev in &events {
        match ev {
            Event::Header(HeaderEvent::Status(h)) => header = Some(*h),
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) => {
                let lo = *offset as usize;
                let hi = lo + *length as usize;
                body_start = Some(body_start.unwrap_or(lo));
                body_end = Some(hi);
            }
            _ => {}
        }
    }
    match (header, body_start, body_end) {
        (Some(h), Some(lo), Some(hi)) => Some((h, &wire[lo..hi])),
        _ => None,
    }
}

fn walk_fast_slots<'a>(
    header: StatusHeader,
    body: &'a [u8],
    slot_data_len: impl Fn(usize) -> usize,
) -> Vec<Slot<'a>> {
    let mut slots = Vec::new();
    let mut cursor = 0usize;
    let mut idx = 0usize;
    while cursor < body.len() {
        let data_len = slot_data_len(idx);
        let (error, id, after_id) = if idx == 0 {
            // Slot 0: error hoisted into Status header; body starts at id.
            (header.error, body[cursor], cursor + 1)
        } else {
            // Slot k>0: [error, id, data...] inline.
            (
                StatusError::from_byte(body[cursor]),
                body[cursor + 1],
                cursor + 2,
            )
        };
        let data_end = after_id + data_len;
        if data_end > body.len() {
            break;
        }
        slots.push(Slot {
            id: Id::new(id),
            error,
            data: &body[after_id..data_end],
        });
        cursor = data_end;
        idx += 1;
    }
    slots
}
