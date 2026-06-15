//! Reference vector encoding and parsing through the streaming Parser
//! and the codec encoder API.

use dxl_protocol::streaming::{
    Event, HeaderEvent, InstructionHeader, InstructionPayload, Parser, PayloadEvent,
};
use dxl_protocol::types::{ErrorCode, Id, Slot, StatusError};
use dxl_protocol::{
    CrcUmts, Instruction, InstructionEncoder, SlotEncoder, SoftwareCrcUmts, StatusEncoder,
    WriteError,
};
use heapless::Vec as HVec;

type Crc = SoftwareCrcUmts;

fn crc_oneshot(seed: u16, bytes: &[u8]) -> u16 {
    let mut c = SoftwareCrcUmts::new_with_state(seed);
    c.update(bytes);
    c.finalize()
}

fn encode<F>(f: F) -> HVec<u8, 256>
where
    F: FnOnce(&mut InstructionEncoder<'_, HVec<u8, 256>, Crc>) -> Result<(), WriteError>,
{
    let mut out: HVec<u8, 256> = HVec::new();
    {
        let mut w = InstructionEncoder::<_, Crc>::new(&mut out);
        f(&mut w).expect("encode failed");
    }
    out
}

fn parse_events(wire: &[u8]) -> Vec<Event> {
    let mut p: Parser<Crc> = Parser::new();
    p.feed(wire).collect()
}

fn instruction_header(events: &[Event]) -> InstructionHeader {
    events
        .iter()
        .find_map(|e| match e {
            Event::Header(HeaderEvent::Instruction(h)) => Some(*h),
            _ => None,
        })
        .expect("expected instruction header event")
}

fn status_header(events: &[Event]) -> dxl_protocol::streaming::StatusHeader {
    events
        .iter()
        .find_map(|e| match e {
            Event::Header(HeaderEvent::Status(h)) => Some(*h),
            _ => None,
        })
        .expect("expected status header event")
}

/// Strip DXL 2.0 stuffing from `wire[start..end]`. Pre-warm the 3-byte
/// sliding window with the 3 wire bytes immediately before `start` — the
/// encoder's trigger straddles the address→body boundary when addr ends
/// in `FF FF` and the first body byte is `FD`, so the unstuff state at
/// chunk start depends on what came before.
fn unstuff_wire(wire: &[u8], start: usize, end: usize) -> Vec<u8> {
    let mut last3 = [0u8; 3];
    if start >= 3 {
        last3.copy_from_slice(&wire[start - 3..start]);
    }
    let mut out = Vec::with_capacity(end - start);
    for &b in &wire[start..end] {
        if last3 == [0xFF, 0xFF, 0xFD] && b == 0xFD {
            last3 = [last3[1], last3[2], b];
            continue;
        }
        out.push(b);
        last3 = [last3[1], last3[2], b];
    }
    out
}

fn write_data_unstuffed(wire: &[u8], events: &[Event]) -> Vec<u8> {
    let chunks: Vec<(u16, u16)> = events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                offset,
                length,
            })) => Some((*offset, *length)),
            _ => None,
        })
        .collect();
    if chunks.is_empty() {
        return Vec::new();
    }
    let start = chunks[0].0 as usize;
    let end = chunks.last().map(|(o, l)| (o + l) as usize).unwrap();
    unstuff_wire(wire, start, end)
}

fn status_data_unstuffed(wire: &[u8], events: &[Event]) -> Vec<u8> {
    let chunks: Vec<(u16, u16)> = events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Status(
                dxl_protocol::streaming::StatusPayload::ReadDataChunk { offset, length },
            )) => Some((*offset, *length)),
            _ => None,
        })
        .collect();
    if chunks.is_empty() {
        return Vec::new();
    }
    let start = chunks[0].0 as usize;
    let end = chunks.last().map(|(o, l)| (o + l) as usize).unwrap();
    unstuff_wire(wire, start, end)
}

fn slot(id: u8, error: StatusError, data: &[u8]) -> Slot<'_> {
    Slot {
        id: Id::new(id),
        error,
        data,
    }
}

const PING_ID1: &[u8] = &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E];

const READ_ID1_ADDR132_LEN4: &[u8] = &[
    0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15,
];

const WRITE_ID1_GOAL512: &[u8] = &[
    0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03, 0x74, 0x00, 0x00, 0x02, 0x00, 0x00, 0xCA, 0x89,
];

const REBOOT_ID1: &[u8] = &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x08, 0x2F, 0x4E];

#[test]
fn crc_matches_known_frames() {
    for frame in [
        PING_ID1,
        READ_ID1_ADDR132_LEN4,
        WRITE_ID1_GOAL512,
        REBOOT_ID1,
    ] {
        let body = &frame[..frame.len() - 2];
        let expected = u16::from_le_bytes([frame[frame.len() - 2], frame[frame.len() - 1]]);
        assert_eq!(
            crc_oneshot(0, body),
            expected,
            "CRC mismatch on frame: {:02X?}",
            frame
        );
    }
}

#[test]
fn crc_accumulate_seed_matches_one_shot() {
    let data: &[u8] = &[
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x10, 0x11, 0x12,
        0x13,
    ];
    let full = crc_oneshot(0, data);
    for split in 0..=data.len() {
        let (a, b) = data.split_at(split);
        let seed = crc_oneshot(0, a);
        assert_eq!(crc_oneshot(seed, b), full, "split at {split}");
    }
}

const BODY_5: &[u8] = &[0x01, 0x00, 0x00, 0x00];
const BODY_6: &[u8] = &[0x02, 0x00, 0x00, 0x00];
const BODY_7: &[u8] = &[0x03, 0x00, 0x00, 0x00];

#[test]
fn write_slot_first_emits_header_then_body() {
    let mut out: HVec<u8, 32> = HVec::new();
    SlotEncoder::<_, Crc>::new(&mut out)
        .first(&slot(5, StatusError::OK, BODY_5), 0x0015)
        .unwrap();
    assert_eq!(
        out.as_slice(),
        &[
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x15, 0x00, 0x55, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00,
        ]
    );
}

#[test]
fn write_slot_middle_emits_body_only() {
    let mut out: HVec<u8, 32> = HVec::new();
    SlotEncoder::<_, Crc>::new(&mut out)
        .middle(&slot(6, StatusError::OK, BODY_6))
        .unwrap();
    assert_eq!(out.as_slice(), &[0x00, 0x06, 0x02, 0x00, 0x00, 0x00]);
}

#[test]
fn write_slot_last_reserves_crc_placeholder() {
    let mut out: HVec<u8, 32> = HVec::new();
    SlotEncoder::<_, Crc>::new(&mut out)
        .last(&slot(7, StatusError::OK, BODY_7), 0xBBAA)
        .unwrap();
    assert_eq!(
        out.as_slice(),
        &[0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0xAA, 0xBB]
    );
}

#[test]
fn write_slot_only_emits_header_body_and_computed_crc() {
    let mut out: HVec<u8, 32> = HVec::new();
    SlotEncoder::<_, Crc>::new(&mut out)
        .only(&slot(5, StatusError::OK, BODY_5), 0x0009)
        .unwrap();
    let header_and_body = [
        0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x55, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00,
    ];
    let crc = crc_oneshot(0, &header_and_body).to_le_bytes();
    let mut expected = [0u8; 16];
    expected[..14].copy_from_slice(&header_and_body);
    expected[14..].copy_from_slice(&crc);
    assert_eq!(out.as_slice(), &expected);
}

#[test]
fn write_slot_three_slave_response_assembles_to_valid_status_frame() {
    let mut out: HVec<u8, 64> = HVec::new();
    {
        let mut w = SlotEncoder::<_, Crc>::new(&mut out);
        w.first(&slot(5, StatusError::OK, BODY_5), 0x0015).unwrap();
        w.middle(&slot(6, StatusError::OK, BODY_6)).unwrap();
        w.last(&slot(7, StatusError::OK, BODY_7), 0xBBAA).unwrap();
    }

    assert_eq!(out.len(), 28);
    assert_eq!(out.as_slice()[26..28], [0xAA, 0xBB]);

    let crc_offset = out.len() - 2;
    let crc = crc_oneshot(0, &out.as_slice()[..crc_offset]);
    let bytes = crc.to_le_bytes();
    out[crc_offset] = bytes[0];
    out[crc_offset + 1] = bytes[1];

    let length_field = u16::from_le_bytes([out[5], out[6]]);
    assert_eq!(length_field as usize, 1 + 3 * 6 + 2);
    assert_eq!(
        crc_oneshot(0, &out.as_slice()[..crc_offset]),
        u16::from_le_bytes([out[crc_offset], out[crc_offset + 1]])
    );
}

#[test]
fn write_slot_overflow_truncates() {
    let mut out: HVec<u8, 8> = HVec::new();
    let err = SlotEncoder::<_, Crc>::new(&mut out)
        .first(&slot(5, StatusError::OK, BODY_5), 0x0015)
        .unwrap_err();
    assert_eq!(err, WriteError::Overflow);
    assert!(out.is_empty());
}

#[test]
fn write_slot_error_emits_caller_supplied_payload_with_error_byte() {
    let zero = [0u8; 4];
    let mut out: HVec<u8, 32> = HVec::new();
    SlotEncoder::<_, Crc>::new(&mut out)
        .middle(&slot(7, StatusError::code(ErrorCode::DataRange), &zero))
        .unwrap();
    assert_eq!(
        out.as_slice(),
        &[
            StatusError::code(ErrorCode::DataRange).as_byte(),
            0x07,
            0,
            0,
            0,
            0,
        ]
    );
}

#[test]
fn parse_ping() {
    let evs = parse_events(PING_ID1);
    match instruction_header(&evs) {
        InstructionHeader::Ping { id } => assert_eq!(id, Id::new(1)),
        other => panic!("not Ping: {other:?}"),
    }
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn parse_read() {
    let evs = parse_events(READ_ID1_ADDR132_LEN4);
    match instruction_header(&evs) {
        InstructionHeader::Read {
            id,
            address,
            length,
        } => {
            assert_eq!(id, Id::new(1));
            assert_eq!(address, 132);
            assert_eq!(length, 4);
        }
        other => panic!("not Read: {other:?}"),
    }
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn parse_write() {
    let evs = parse_events(WRITE_ID1_GOAL512);
    match instruction_header(&evs) {
        InstructionHeader::Write {
            id,
            address,
            length,
        } => {
            assert_eq!(id, Id::new(1));
            assert_eq!(address, 116);
            assert_eq!(length, 4);
        }
        other => panic!("not Write: {other:?}"),
    }
    assert_eq!(
        write_data_unstuffed(WRITE_ID1_GOAL512, &evs),
        vec![0x00, 0x02, 0x00, 0x00]
    );
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn parse_reboot() {
    let evs = parse_events(REBOOT_ID1);
    assert!(matches!(
        instruction_header(&evs),
        InstructionHeader::Reboot { id } if id == Id::new(1)
    ));
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn write_ping_matches_reference() {
    let out = encode(|w| w.ping(Id::new(1)));
    assert_eq!(&out[..], PING_ID1);
}

#[test]
fn write_read_matches_reference() {
    let out = encode(|w| w.read(Id::new(1), 132, 4));
    assert_eq!(&out[..], READ_ID1_ADDR132_LEN4);
}

#[test]
fn write_write_matches_reference() {
    let out = encode(|w| w.write(Id::new(1), 116, &[0x00, 0x02, 0x00, 0x00]));
    assert_eq!(&out[..], WRITE_ID1_GOAL512);
}

#[test]
fn write_reboot_matches_reference() {
    let out = encode(|w| w.reboot(Id::new(1)));
    assert_eq!(&out[..], REBOOT_ID1);
}

#[test]
fn write_status_round_trip() {
    let params = [0x06u8, 0x04, 0x26];
    let mut out: HVec<u8, 64> = HVec::new();
    StatusEncoder::<_, Crc>::new(&mut out)
        .ext(Id::new(1), StatusError::OK, &params)
        .unwrap();

    let evs = parse_events(&out);
    let hdr = status_header(&evs);
    assert_eq!(hdr.id, Id::new(1));
    assert_eq!(hdr.error.as_byte(), 0);
    assert_eq!(status_data_unstuffed(&out, &evs), params);
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn stuffing_round_trip() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xFD, 0x55];
    let out = encode(|w| w.write(Id::new(1), 0x0040, &data));

    let fds = out.iter().filter(|&&b| b == 0xFD).count();
    assert!(fds >= 4, "expected stuffed bytes: {:02X?}", out);

    let evs = parse_events(&out);
    match instruction_header(&evs) {
        InstructionHeader::Write { id, address, .. } => {
            assert_eq!(id, Id::new(1));
            assert_eq!(address, 0x0040);
        }
        other => panic!("not Write: {other:?}"),
    }
    assert_eq!(write_data_unstuffed(&out, &evs), data);
}

#[test]
fn stuffing_at_field_boundary() {
    let data = [0xFDu8, 0x11, 0x22];
    let out = encode(|w| w.write(Id::new(1), 0xFFFF, &data));
    let evs = parse_events(&out);
    match instruction_header(&evs) {
        InstructionHeader::Write { address, .. } => assert_eq!(address, 0xFFFF),
        other => panic!("not Write: {other:?}"),
    }
    assert_eq!(write_data_unstuffed(&out, &evs), data);
}

#[test]
fn unknown_instruction_routes_to_raw() {
    let mut frame: HVec<u8, 32> = HVec::new();
    frame
        .extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x77])
        .unwrap();
    let crc = crc_oneshot(0, &frame);
    frame.extend_from_slice(&crc.to_le_bytes()).unwrap();
    let evs = parse_events(&frame);
    match instruction_header(&evs) {
        InstructionHeader::Raw { id, instr, length } => {
            assert_eq!(id, Id::new(1));
            assert_eq!(instr, 0x77);
            assert_eq!(length, 0);
        }
        other => panic!("expected Raw, got {other:?}"),
    }
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn instruction_byte_constants() {
    assert_eq!(Instruction::Ping.as_u8(), 0x01);
    assert_eq!(Instruction::Status.as_u8(), 0x55);
    assert_eq!(Instruction::SyncWrite.as_u8(), 0x83);
    assert_eq!(Instruction::FastBulkRead.as_u8(), 0x9A);
}

fn fds_in_payload_region(frame: &[u8]) -> usize {
    let payload = &frame[8..frame.len() - 2];
    payload.iter().filter(|&&b| b == 0xFD).count()
}

fn write_with_data(data: &[u8]) -> HVec<u8, 256> {
    encode(|w| w.write(Id::new(1), 0x0010, data))
}

fn assert_write_decodes_to(wire: &[u8], expected_data: &[u8]) {
    let evs = parse_events(wire);
    match instruction_header(&evs) {
        InstructionHeader::Write { address, .. } => assert_eq!(address, 0x0010),
        other => panic!("not Write: {other:?}"),
    }
    assert_eq!(write_data_unstuffed(wire, &evs), expected_data);
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn stuff_no_trigger() {
    let data = [0x01u8, 0x02, 0x03, 0x04];
    let frame = write_with_data(&data);
    assert_eq!(fds_in_payload_region(&frame), 0);
    assert_write_decodes_to(&frame, &data);
}

#[test]
fn stuff_single_trigger_inserts_one_byte() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x00];
    let frame = write_with_data(&data);
    assert_eq!(fds_in_payload_region(&frame), 2);
    assert_write_decodes_to(&frame, &data);
}

#[test]
fn stuff_multiple_triggers() {
    let data = [
        0xFFu8, 0xFF, 0xFD, 0x01, 0x02, 0xFF, 0xFF, 0xFD, 0x03, 0xAA, 0xFF, 0xFF, 0xFD,
    ];
    let frame = write_with_data(&data);
    let logical_fds = data.iter().filter(|&&b| b == 0xFD).count();
    assert_eq!(fds_in_payload_region(&frame), logical_fds + 3);
    assert_write_decodes_to(&frame, &data);
}

#[test]
fn stuff_trigger_at_payload_start() {
    let data = [0xFFu8, 0xFF, 0xFD, 0xAA, 0xBB];
    let frame = write_with_data(&data);
    assert_write_decodes_to(&frame, &data);
}

#[test]
fn stuff_trigger_at_payload_end() {
    let data = [0xAAu8, 0xBB, 0xFF, 0xFF, 0xFD];
    let frame = write_with_data(&data);
    assert_eq!(fds_in_payload_region(&frame), 2);
    assert_write_decodes_to(&frame, &data);
}

#[test]
fn stuff_logical_fd_after_trigger() {
    let data = [0xFFu8, 0xFF, 0xFD, 0xFD, 0x55];
    let frame = write_with_data(&data);
    assert_write_decodes_to(&frame, &data);
}

#[test]
fn stuff_two_adjacent_triggers() {
    let data = [0xFFu8, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0xAA];
    let frame = write_with_data(&data);
    assert_write_decodes_to(&frame, &data);
}

#[test]
fn stuff_trigger_spanning_address_boundary() {
    let data = [0xFDu8, 0x11, 0x22];
    let out = encode(|w| w.write(Id::new(1), 0xFFFF, &data));
    let evs = parse_events(&out);
    match instruction_header(&evs) {
        InstructionHeader::Write { address, .. } => assert_eq!(address, 0xFFFF),
        other => panic!("not Write: {other:?}"),
    }
    assert_eq!(write_data_unstuffed(&out, &evs), data);
}

#[test]
fn stuff_trigger_spanning_address_then_more_in_data() {
    let data = [0xFDu8, 0x42, 0xFF, 0xFF, 0xFD, 0x55, 0x66];
    let out = encode(|w| w.write(Id::new(1), 0xFFFF, &data));
    let evs = parse_events(&out);
    assert!(matches!(
        instruction_header(&evs),
        InstructionHeader::Write { .. }
    ));
    assert_eq!(write_data_unstuffed(&out, &evs), data);
}

#[test]
fn stuff_empty_payload() {
    let out = encode(|w| w.write(Id::new(1), 0x0010, &[]));
    let evs = parse_events(&out);
    assert!(matches!(
        instruction_header(&evs),
        InstructionHeader::Write { length: 0, .. }
    ));
    assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
}

#[test]
fn stuff_status_with_trigger_in_params() {
    let params = [0xFFu8, 0xFF, 0xFD, 0x00, 0x42];
    let mut out: HVec<u8, 64> = HVec::new();
    StatusEncoder::<_, Crc>::new(&mut out)
        .ext(Id::new(1), StatusError::OK, &params)
        .unwrap();
    let evs = parse_events(&out);
    let hdr = status_header(&evs);
    assert_eq!(hdr.error.as_byte(), 0);
    assert_eq!(status_data_unstuffed(&out, &evs), params);
}

#[test]
fn stuff_long_payload_with_many_triggers() {
    let mut data: HVec<u8, 64> = HVec::new();
    for _ in 0..6 {
        data.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x42, 0x55])
            .unwrap();
    }
    let out = encode(|w| w.write(Id::new(1), 0x0010, &data));
    let evs = parse_events(&out);
    assert!(matches!(
        instruction_header(&evs),
        InstructionHeader::Write { .. }
    ));
    assert_eq!(write_data_unstuffed(&out, &evs), data.as_slice());
}
