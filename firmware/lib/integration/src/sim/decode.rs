//! Test helpers for rendering raw wire bytes and decoding Status replies.
//!
//! Integration tests snapshot the raw byte stream with [`format_hex`] (per
//! `feedback_snapshot_vs_assertion`) and assert the decoded shape with
//! [`parse_status`] returning a `Status<'_>`.

use core::fmt::Write;

use dxl_protocol::crc::SoftwareCrcUmts;
use dxl_protocol::streaming::{Event, HeaderEvent, Parser, PayloadEvent, StatusPayload};
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
    let mut parser: Parser<SoftwareCrcUmts> = Parser::new();
    let events: Vec<Event> = parser.feed(wire).collect();

    let header = events
        .iter()
        .find_map(|e| match e {
            Event::Header(HeaderEvent::Status(h)) => Some(*h),
            _ => None,
        })
        .expect("parse_status: no Status header in wire");

    let body = events.iter().find_map(|e| match e {
        Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk { offset, length })) => {
            Some(&wire[*offset as usize..(*offset + *length) as usize])
        }
        _ => None,
    });

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
