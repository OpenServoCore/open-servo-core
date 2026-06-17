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
use dxl_protocol::types::{Instruction, PingStatus, Status};

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
        | Instruction::ControlTableBackup => Status::Empty {
            id: header.id,
            error: header.error,
        },
        other => panic!("parse_status: instruction {other:?} not yet supported"),
    }
}
