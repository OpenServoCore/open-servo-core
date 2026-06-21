//! Test helpers for rendering raw wire bytes and decoding Status replies.
//!
//! Integration tests snapshot the raw byte stream with [`format_hex`] (per
//! `feedback_snapshot_vs_assertion`) and assert the decoded shape with
//! [`parse_status`] returning a `Status<'_>`.

use core::fmt::Write;

use dxl_protocol::crc::SoftwareCrcUmts;
use dxl_protocol::streaming::{
    CrcResult, Event, HeaderEvent, Parser, PayloadEvent, StatusHeader, StatusPayload,
};
use dxl_protocol::types::{BulkReadEntry, Id, Instruction, PingStatus, Slot, Status, StatusError};

/// CRC outcome observed at the end of a Fast Status decode.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FastStatusCrc {
    /// Parser emitted `Event::Crc(CrcResult::Good)` — a spec-compliant
    /// host accepts the Status packet.
    Good,
    /// Parser emitted `Event::Crc(CrcResult::Bad)` — a spec-compliant
    /// host rejects the whole packet. The decoded slots still reflect
    /// the bytes actually on the wire, so tests can assert packet shape
    /// independently of host acceptance.
    Bad,
    /// Stream ended without a terminal CRC event of either flavour
    /// (e.g. truncated mid-frame).
    Truncated,
}

/// Decoded Fast Sync/Bulk Status reply: the per-slot shape on the wire
/// plus the CRC verdict a real host would reach. Returned by
/// [`parse_fast_sync_status`] and [`parse_fast_bulk_status`].
#[derive(Debug, PartialEq, Eq)]
pub struct FastStatus<'a> {
    pub slots: Vec<Slot<'a>>,
    pub crc: FastStatusCrc,
}

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
            Event::Crc(CrcResult::Good) => {
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

/// Decode a Fast Sync Read coalesced reply into per-slot views plus the
/// CRC verdict.
///
/// The streaming parser collapses a Fast Sync/Bulk reply into one
/// `Status` header + one or more `ReadDataChunk` events because the wire
/// IS one Status packet (broadcast id, single CRC). To recover per-slot
/// views, this walker uses the host-known per-slot length to slice the
/// concatenated body: slot 0 hoists its `error` from the header (the
/// wire's status-error byte position IS slot 0's error per
/// [`SlotEncoder`]); slot k>0 reads its own `[err, id, data...]` from
/// body bytes. Slot length is looked up by **wire id** — a silenced
/// middle slot would otherwise misalign a position-indexed walk against
/// heterogeneous Bulk entries.
///
/// Slots are decoded from whatever body bytes were observed, regardless
/// of the trailing CRC outcome — so tests can assert the actual wire
/// shape under failure scenarios (silent predecessor, dead servo) while
/// [`FastStatus::crc`] records the verdict a real host would reach.
///
/// [`SlotEncoder`]: dxl_protocol::SlotEncoder
pub fn parse_fast_sync_status(wire: &[u8], length: u16) -> FastStatus<'_> {
    decode_fast_status(wire, |_| Some(length as usize))
}

/// Decode a Fast Bulk Read coalesced reply. Per-slot length comes from
/// the host-side `entries` list, looked up by wire id. Returns `None`
/// for an unexpected id — the walker treats that as a truncation
/// boundary. See [`parse_fast_sync_status`] for the wire-walk model.
pub fn parse_fast_bulk_status<'a>(wire: &'a [u8], entries: &[BulkReadEntry]) -> FastStatus<'a> {
    decode_fast_status(wire, |id| {
        entries
            .iter()
            .find(|e| e.id.as_byte() == id)
            .map(|e| e.length as usize)
    })
}

fn decode_fast_status<'a>(
    wire: &'a [u8],
    slot_data_len: impl Fn(u8) -> Option<usize>,
) -> FastStatus<'a> {
    let mut parser: Parser<SoftwareCrcUmts> = Parser::new();
    let events: Vec<Event> = parser.feed(wire).collect();

    let mut header = None;
    let mut body_start = None;
    let mut body_end = None;
    let mut crc = FastStatusCrc::Truncated;
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
            Event::Crc(CrcResult::Good) => crc = FastStatusCrc::Good,
            Event::Crc(CrcResult::Bad) => crc = FastStatusCrc::Bad,
            _ => {}
        }
    }
    let slots = match (header, body_start, body_end) {
        (Some(h), Some(lo), Some(hi)) => walk_fast_slots(h, &wire[lo..hi], slot_data_len),
        _ => Vec::new(),
    };
    FastStatus { slots, crc }
}

fn walk_fast_slots<'a>(
    header: StatusHeader,
    body: &'a [u8],
    slot_data_len: impl Fn(u8) -> Option<usize>,
) -> Vec<Slot<'a>> {
    let mut slots = Vec::new();
    let mut cursor = 0usize;
    let mut first = true;
    while cursor < body.len() {
        let (error, id, after_id) = if first {
            // Slot 0: error hoisted into Status header; body starts at id.
            (header.error, body[cursor], cursor + 1)
        } else {
            // Slot k>0: [error, id, data...] inline.
            if cursor + 1 >= body.len() {
                break;
            }
            (
                StatusError::from_byte(body[cursor]),
                body[cursor + 1],
                cursor + 2,
            )
        };
        let Some(data_len) = slot_data_len(id) else {
            break;
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
        first = false;
    }
    slots
}
