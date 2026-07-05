//! Test helpers for rendering raw wire bytes and decoding Status replies.
//!
//! Integration tests snapshot the raw byte stream with [`format_hex`] (per
//! `feedback_snapshot_vs_assertion`) and assert the decoded shape with
//! [`parse_status`] returning a `Status<'_>`.

use core::fmt::Write;

use dxl_protocol::crc::SoftwareCrcUmts;
use dxl_protocol::crc16_umts_continue;
use dxl_protocol::frame::{FrameKind, Probe, parse, probe};
use dxl_protocol::types::packet::decode_status;
use dxl_protocol::types::{BulkReadEntry, Id, Instruction, PingStatus, Slot, Status, StatusError};
use dxl_protocol::wire::CRC_BYTES;

/// CRC outcome observed at the end of a Fast Status decode.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FastStatusCrc {
    /// The packet's cumulative CRC matched — a spec-compliant host accepts
    /// the coalesced Status packet.
    Good,
    /// The packet's trailing cumulative CRC did not match — a spec-compliant
    /// host rejects the whole packet. The decoded slots still reflect the
    /// bytes actually on the wire, so tests can assert packet shape
    /// independently of host acceptance.
    Bad,
    /// The frame ended before its declared `Length` (truncated mid-frame),
    /// so no terminal CRC could be checked.
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
/// decoder, and the master must know what was sent to interpret what came back.
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
    let mut replies = Vec::new();
    let mut cursor = 0;
    while cursor < wire.len() {
        let rest = &wire[cursor..];
        let Ok((frame, n)) = parse::<SoftwareCrcUmts>(rest) else {
            break;
        };
        if frame.kind != FrameKind::Status {
            break;
        }
        let Ok(reply) = decode_status(&frame) else {
            break;
        };
        // Post-error param region as a raw wire slice (unstuffed in practice —
        // the register payloads these tests exercise carry no stuffing).
        let body: &[u8] = &frame.body[1..];
        replies.push(decode_one(req, reply.id, reply.error, body));
        cursor += n;
    }
    replies
}

fn decode_one(req: Instruction, id: Id, error: StatusError, body: &[u8]) -> Status<'_> {
    match req {
        Instruction::Ping => {
            assert_eq!(body.len(), 3, "Ping reply body must be 3 bytes");
            Status::Ping {
                id,
                error,
                status: PingStatus {
                    model: u16::from_le_bytes([body[0], body[1]]),
                    fw_version: body[2],
                },
            }
        }
        Instruction::Read | Instruction::SyncRead | Instruction::BulkRead => Status::Read {
            id,
            error,
            data: body,
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
        | Instruction::Ext(_) => Status::Empty { id, error },
        other => panic!("parse_status: instruction {other:?} not yet supported"),
    }
}

/// Decode a Fast Sync Read coalesced reply into per-slot views plus the
/// CRC verdict.
///
/// The wire IS one coalesced Status packet (broadcast id, one trailing
/// cumulative CRC). To recover per-slot views this walker uses the host-known
/// per-slot length to slice the concatenated body: slot 0 hoists its `error`
/// from the packet header (the wire's status-error byte position IS slot 0's
/// error per the official FAST layout); slot k>0 reads its own
/// `[err, id, data...]` from body bytes. Slot length is looked up by **wire
/// id** — a silenced middle slot would otherwise misalign a position-indexed
/// walk against heterogeneous Bulk entries.
///
/// Slots are decoded from whatever body bytes were observed, regardless
/// of the trailing CRC outcome — so tests can assert the actual wire
/// shape under failure scenarios (silent predecessor, dead servo) while
/// [`FastStatus::crc`] records the verdict a real host would reach.
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
    // `probe` classifies the head from its first 8 bytes and reports `total`
    // (the declared frame size) even when the wire holds fewer bytes — so a
    // truncated chain still yields its body prefix here.
    let total = match probe(wire) {
        Probe::Frame {
            total,
            kind: FrameKind::ChainStatus,
            ..
        } => total,
        _ => {
            return FastStatus {
                slots: Vec::new(),
                crc: FastStatusCrc::Truncated,
            };
        }
    };

    // Slot 0's error sits at wire[8]; its block (starting at the id byte) at
    // wire[9]. A frame too short to carry even that is a bare truncation.
    const BODY_START: usize = 9;
    if wire.len() < BODY_START {
        return FastStatus {
            slots: Vec::new(),
            crc: FastStatusCrc::Truncated,
        };
    }
    let slot0_error = StatusError::from_byte(wire[8]);

    let (body, crc) = if wire.len() >= total {
        // Complete frame: the trailing pair is the packet's cumulative CRC
        // (the last block's checkpoint). Verify it over everything before it.
        let crc_pos = total - CRC_BYTES;
        let got = u16::from_le_bytes([wire[crc_pos], wire[crc_pos + 1]]);
        let crc = if crc16_umts_continue(0, &wire[..crc_pos]) == got {
            FastStatusCrc::Good
        } else {
            FastStatusCrc::Bad
        };
        (&wire[BODY_START..crc_pos], crc)
    } else {
        (&wire[BODY_START..], FastStatusCrc::Truncated)
    };

    let slots = walk_fast_slots(slot0_error, body, slot_data_len);
    FastStatus { slots, crc }
}

fn walk_fast_slots<'a>(
    slot0_error: StatusError,
    body: &'a [u8],
    slot_data_len: impl Fn(u8) -> Option<usize>,
) -> Vec<Slot<'a>> {
    let mut slots = Vec::new();
    let mut cursor = 0usize;
    let mut first = true;
    while cursor < body.len() {
        let (error, id, after_id) = if first {
            // Slot 0: error hoisted into the packet header; body starts at id.
            (slot0_error, body[cursor], cursor + 1)
        } else {
            // Slot k>0: [error, id, data, crc...] inline.
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
        // Every block ends with its cumulative chain CRC (official layout).
        // The FINAL block's CRC doubles as the packet CRC excluded from
        // `body`, so allow the walk to end exactly at `data_end` there.
        if data_end > body.len() {
            break;
        }
        slots.push(Slot {
            id: Id::new(id),
            error,
            data: &body[after_id..data_end],
        });
        cursor = data_end + CRC_BYTES;
        first = false;
    }
    slots
}
