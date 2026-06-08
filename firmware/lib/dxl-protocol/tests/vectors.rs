//! Reference vector encoding and decoding through the streaming Decoder
//! and the new emitter API.

use dxl_protocol::decoder::{Decoder, ResyncKind, Step};
use dxl_protocol::packet::{self as overlay, ErrorCode, Slot, StatusError};
use dxl_protocol::{
    CrcUmts, InstructionEmitter, PACKET_LEN_GUARD, SlotEmitter, SoftwareCrcUmts, StatusEmitter,
    WriteBuf, WriteError, instruction::Instruction,
};
use heapless::Vec;

type Crc = SoftwareCrcUmts;

fn encode<F>(f: F) -> Vec<u8, 256>
where
    F: FnOnce(&mut InstructionEmitter<'_, Vec<u8, 256>, Crc>) -> Result<(), WriteError>,
{
    let mut out: Vec<u8, 256> = Vec::new();
    {
        let mut w = InstructionEmitter::<_, Crc>::new(&mut out);
        f(&mut w).expect("encode failed");
    }
    out
}

fn feed_full<'a, const M: usize>(dec: &'a mut Decoder<M, Crc>, wire: &[u8]) -> overlay::Packet<'a> {
    let (step, n) = dec.feed(wire);
    assert_eq!(n, wire.len(), "decoder didn't consume the full frame");
    match step {
        Step::Packet(p) => p,
        other => panic!("expected Packet, got {other:?}"),
    }
}

fn slot<'a>(id: u8, error: StatusError, data: &'a [u8]) -> Slot<'a> {
    Slot { id, error, data }
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
            SoftwareCrcUmts::accumulate(0, body),
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
    let full = SoftwareCrcUmts::accumulate(0, data);
    for split in 0..=data.len() {
        let (a, b) = data.split_at(split);
        let seed = SoftwareCrcUmts::accumulate(0, a);
        assert_eq!(
            SoftwareCrcUmts::accumulate(seed, b),
            full,
            "split at {split}"
        );
    }
}

const BODY_5: &[u8] = &[0x01, 0x00, 0x00, 0x00];
const BODY_6: &[u8] = &[0x02, 0x00, 0x00, 0x00];
const BODY_7: &[u8] = &[0x03, 0x00, 0x00, 0x00];

#[test]
fn write_slot_first_emits_header_then_body() {
    let mut out: Vec<u8, 32> = Vec::new();
    SlotEmitter::<_, Crc>::new(&mut out)
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
    let mut out: Vec<u8, 32> = Vec::new();
    SlotEmitter::<_, Crc>::new(&mut out)
        .middle(&slot(6, StatusError::OK, BODY_6))
        .unwrap();
    assert_eq!(out.as_slice(), &[0x00, 0x06, 0x02, 0x00, 0x00, 0x00]);
}

#[test]
fn write_slot_last_reserves_crc_placeholder() {
    let mut out: Vec<u8, 32> = Vec::new();
    SlotEmitter::<_, Crc>::new(&mut out)
        .last(&slot(7, StatusError::OK, BODY_7), 0xBBAA)
        .unwrap();
    assert_eq!(
        out.as_slice(),
        &[0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0xAA, 0xBB]
    );
}

#[test]
fn write_slot_only_emits_header_body_and_computed_crc() {
    let mut out: Vec<u8, 32> = Vec::new();
    SlotEmitter::<_, Crc>::new(&mut out)
        .only(&slot(5, StatusError::OK, BODY_5), 0x0009)
        .unwrap();
    let header_and_body = [
        0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x55, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00,
    ];
    let crc = SoftwareCrcUmts::accumulate(0, &header_and_body).to_le_bytes();
    let mut expected = [0u8; 16];
    expected[..14].copy_from_slice(&header_and_body);
    expected[14..].copy_from_slice(&crc);
    assert_eq!(out.as_slice(), &expected);
}

#[test]
fn write_slot_three_slave_response_assembles_to_valid_status_frame() {
    let mut out: Vec<u8, 64> = Vec::new();
    {
        let mut w = SlotEmitter::<_, Crc>::new(&mut out);
        w.first(&slot(5, StatusError::OK, BODY_5), 0x0015).unwrap();
        w.middle(&slot(6, StatusError::OK, BODY_6)).unwrap();
        w.last(&slot(7, StatusError::OK, BODY_7), 0xBBAA).unwrap();
    }

    assert_eq!(out.len(), 28);
    assert_eq!(out.as_slice()[26..28], [0xAA, 0xBB]);

    let crc_offset = out.len() - 2;
    let crc = SoftwareCrcUmts::accumulate(0, &out.as_slice()[..crc_offset]);
    let bytes = crc.to_le_bytes();
    out[crc_offset] = bytes[0];
    out[crc_offset + 1] = bytes[1];

    let length_field = u16::from_le_bytes([out[5], out[6]]);
    assert_eq!(length_field as usize, 1 + 3 * 6 + 2);
    assert_eq!(
        SoftwareCrcUmts::accumulate(0, &out.as_slice()[..crc_offset]),
        u16::from_le_bytes([out[crc_offset], out[crc_offset + 1]])
    );
}

#[test]
fn write_slot_overflow_truncates() {
    let mut out: Vec<u8, 8> = Vec::new();
    let err = SlotEmitter::<_, Crc>::new(&mut out)
        .first(&slot(5, StatusError::OK, BODY_5), 0x0015)
        .unwrap_err();
    assert_eq!(err, WriteError::Overflow);
    assert!(out.is_empty());
}

#[test]
fn write_slot_error_emits_caller_supplied_payload_with_error_byte() {
    let zero = [0u8; 4];
    let mut out: Vec<u8, 32> = Vec::new();
    SlotEmitter::<_, Crc>::new(&mut out)
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
    let mut dec: Decoder<32, Crc> = Decoder::new();
    match feed_full(&mut dec, PING_ID1) {
        overlay::Packet::Ping(p) => assert_eq!(p.header.id, 1),
        other => panic!("not Ping: {other:?}"),
    }
}

#[test]
fn parse_byte_at_a_time_matches_one_shot() {
    for frame in [
        PING_ID1,
        READ_ID1_ADDR132_LEN4,
        WRITE_ID1_GOAL512,
        REBOOT_ID1,
    ] {
        let mut one_shot_dec: Decoder<64, Crc> = Decoder::new();
        let one_shot = feed_full(&mut one_shot_dec, frame);
        let one_shot_disc = core::mem::discriminant(&one_shot);

        let mut dec: Decoder<64, Crc> = Decoder::new();
        let last = frame.len() - 1;
        for (i, &b) in frame.iter().enumerate().take(last) {
            let (step, n) = dec.feed(&[b]);
            assert_eq!(n, 1);
            assert!(
                matches!(step, Step::NeedMore),
                "frame {frame:02X?} byte {i}: {step:?}"
            );
        }
        let (step, n) = dec.feed(&[frame[last]]);
        assert_eq!(n, 1);
        match step {
            Step::Packet(p) => {
                assert_eq!(
                    core::mem::discriminant(&p),
                    one_shot_disc,
                    "frame {frame:02X?}: byte-stream produced a different variant",
                );
            }
            other => panic!("frame {frame:02X?}: expected Packet at last byte, got {other:?}"),
        }
    }
}

#[test]
fn parse_garbage_then_frame_recovers_frame() {
    let prefix = [0xAAu8, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
    let mut combined: Vec<u8, 32> = Vec::new();
    combined.extend_from_slice(&prefix).unwrap();
    combined.extend_from_slice(PING_ID1).unwrap();

    let mut dec: Decoder<32, Crc> = Decoder::new();
    match feed_full(&mut dec, &combined) {
        overlay::Packet::Ping(p) => assert_eq!(p.header.id, 1),
        other => panic!("expected Ping, got {other:?}"),
    }
}

#[test]
fn parse_read() {
    let mut dec: Decoder<32, Crc> = Decoder::new();
    match feed_full(&mut dec, READ_ID1_ADDR132_LEN4) {
        overlay::Packet::Read(p) => {
            assert_eq!(p.header.id, 1);
            assert_eq!(p.addr.get(), 132);
            assert_eq!(p.length.get(), 4);
        }
        other => panic!("not Read: {other:?}"),
    }
}

#[test]
fn parse_write() {
    let mut dec: Decoder<32, Crc> = Decoder::new();
    match feed_full(&mut dec, WRITE_ID1_GOAL512) {
        overlay::Packet::Write(p) => {
            assert_eq!(p.header.header.id, 1);
            assert_eq!(p.header.addr.get(), 116);
            assert_eq!(p.data, &[0x00, 0x02, 0x00, 0x00]);
        }
        other => panic!("not Write: {other:?}"),
    }
}

#[test]
fn parse_reboot() {
    let mut dec: Decoder<32, Crc> = Decoder::new();
    assert!(matches!(
        feed_full(&mut dec, REBOOT_ID1),
        overlay::Packet::Reboot(p) if p.header.id == 1
    ));
}

#[test]
fn write_ping_matches_reference() {
    let out = encode(|w| w.ping(1));
    assert_eq!(&out[..], PING_ID1);
}

#[test]
fn write_read_matches_reference() {
    let out = encode(|w| w.read(1, 132, 4));
    assert_eq!(&out[..], READ_ID1_ADDR132_LEN4);
}

#[test]
fn write_write_matches_reference() {
    let out = encode(|w| w.write(1, 116, &[0x00, 0x02, 0x00, 0x00]));
    assert_eq!(&out[..], WRITE_ID1_GOAL512);
}

#[test]
fn write_reboot_matches_reference() {
    let out = encode(|w| w.reboot(1));
    assert_eq!(&out[..], REBOOT_ID1);
}

#[test]
fn write_status_round_trip() {
    let params = [0x06u8, 0x04, 0x26];
    let mut out: Vec<u8, 64> = Vec::new();
    StatusEmitter::<_, Crc>::new(&mut out)
        .ext(1, StatusError::OK, &params)
        .unwrap();

    let mut dec: Decoder<64, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Status(p) => {
            assert_eq!(p.header.header.id, 1);
            assert_eq!(p.error().as_byte(), 0);
            assert_eq!(p.params, &params);
        }
        other => panic!("not Status: {other:?}"),
    }
}

#[test]
fn stuffing_round_trip() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xFD, 0x55];
    let out = encode(|w| w.write(1, 0x0040, &data));

    let fds = out.iter().filter(|&&b| b == 0xFD).count();
    assert!(fds >= 4, "expected stuffed bytes: {:02X?}", out);

    let mut dec: Decoder<64, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Write(p) => {
            assert_eq!(p.header.header.id, 1);
            assert_eq!(p.header.addr.get(), 0x0040);
            assert_eq!(p.data, &data);
        }
        other => panic!("not Write: {other:?}"),
    }
}

#[test]
fn stuffing_at_field_boundary() {
    let data = [0xFDu8, 0x11, 0x22];
    let out = encode(|w| w.write(1, 0xFFFF, &data));
    let mut dec: Decoder<64, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Write(p) => {
            assert_eq!(p.header.addr.get(), 0xFFFF);
            assert_eq!(p.data, &data);
        }
        other => panic!("not Write: {other:?}"),
    }
}

#[test]
fn incomplete_yields_needmore() {
    for n in [4usize, 6, 7] {
        let partial = &PING_ID1[..n];
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, consumed) = dec.feed(partial);
        assert_eq!(consumed, n);
        assert!(
            matches!(step, Step::NeedMore),
            "partial len {n}: {step:?}"
        );
    }
}

#[test]
fn bad_crc_resyncs() {
    let mut bad: Vec<u8, 32> = Vec::new();
    bad.extend_from_slice(PING_ID1).unwrap();
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, _) = dec.feed(&bad);
    assert!(matches!(step, Step::Resync(ResyncKind::BadCrc)));
}

#[test]
fn oversized_length_is_rejected() {
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    assert!((PACKET_LEN_GUARD as u32) < 0xFFFF);
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, n) = dec.feed(&bad);
    assert_eq!(n, 7);
    assert!(matches!(step, Step::Resync(ResyncKind::BadLength)));
}

#[test]
fn undersized_length_is_rejected() {
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0x02, 0x00, 0x01];
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, n) = dec.feed(&bad);
    assert_eq!(n, 7);
    assert!(matches!(step, Step::Resync(ResyncKind::BadLength)));
}

#[test]
fn false_header_with_huge_length_does_not_wedge() {
    let mut buf: Vec<u8, 64> = Vec::new();
    buf.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x42, 0xFF, 0xFF, 0x99])
        .unwrap();
    buf.extend_from_slice(PING_ID1).unwrap();

    let mut dec: Decoder<64, Crc> = Decoder::new();
    let (step, n) = dec.feed(&buf);
    assert!(matches!(step, Step::Resync(ResyncKind::BadLength)));
    assert!(n < buf.len());

    let (step2, n2) = dec.feed(&buf[n..]);
    assert!(matches!(step2, Step::Packet(overlay::Packet::Ping(p)) if p.header.id == 1));
    assert!(n2 <= buf.len() - n);
}

#[test]
fn unknown_instruction_routes_to_raw() {
    let mut frame: Vec<u8, 32> = Vec::new();
    frame
        .extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x77])
        .unwrap();
    let crc = SoftwareCrcUmts::accumulate(0, &frame);
    frame.extend_from_slice(&crc.to_le_bytes()).unwrap();
    let mut dec: Decoder<32, Crc> = Decoder::new();
    match feed_full(&mut dec, &frame) {
        overlay::Packet::Raw(r) => {
            assert_eq!(r.header.header.id, 1);
            assert_eq!(r.header.header.instruction.kind(), Instruction::Ext(0x77));
            assert_eq!(r.params, &[] as &[u8]);
        }
        other => panic!("expected Raw, got {other:?}"),
    }
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

fn write_with_data(data: &[u8]) -> Vec<u8, 256> {
    encode(|w| w.write(1, 0x0010, data))
}

fn assert_write_decodes_to(wire: &[u8], expected_data: &[u8]) {
    let mut dec: Decoder<256, Crc> = Decoder::new();
    match feed_full(&mut dec, wire) {
        overlay::Packet::Write(p) => {
            assert_eq!(p.header.addr.get(), 0x0010);
            assert_eq!(p.data, expected_data);
        }
        other => panic!("not Write: {other:?}"),
    }
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
    let out = encode(|w| w.write(1, 0xFFFF, &data));
    let mut dec: Decoder<64, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Write(p) => {
            assert_eq!(p.header.addr.get(), 0xFFFF);
            assert_eq!(p.data, &data);
        }
        other => panic!("not Write: {other:?}"),
    }
}

#[test]
fn stuff_trigger_spanning_address_then_more_in_data() {
    let data = [0xFDu8, 0x42, 0xFF, 0xFF, 0xFD, 0x55, 0x66];
    let out = encode(|w| w.write(1, 0xFFFF, &data));
    let mut dec: Decoder<64, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Write(p) => assert_eq!(p.data, &data),
        other => panic!("not Write: {other:?}"),
    }
}

#[test]
fn stuff_empty_payload() {
    let out = encode(|w| w.write(1, 0x0010, &[]));
    let mut dec: Decoder<32, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Write(p) => assert_eq!(p.data, &[] as &[u8]),
        other => panic!("not Write: {other:?}"),
    }
}

#[test]
fn stuff_status_with_trigger_in_params() {
    let params = [0xFFu8, 0xFF, 0xFD, 0x00, 0x42];
    let mut out: Vec<u8, 64> = Vec::new();
    StatusEmitter::<_, Crc>::new(&mut out)
        .ext(1, StatusError::OK, &params)
        .unwrap();
    let mut dec: Decoder<64, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Status(p) => {
            assert_eq!(p.error().as_byte(), 0);
            assert_eq!(p.params, &params);
        }
        other => panic!("not Status: {other:?}"),
    }
}

#[test]
fn stuff_long_payload_with_many_triggers() {
    let mut data: Vec<u8, 64> = Vec::new();
    for _ in 0..6 {
        data.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x42, 0x55])
            .unwrap();
    }
    let out = encode(|w| w.write(1, 0x0010, &data));
    let mut dec: Decoder<128, Crc> = Decoder::new();
    match feed_full(&mut dec, &out) {
        overlay::Packet::Write(p) => assert_eq!(p.data, &data[..]),
        other => panic!("not Write: {other:?}"),
    }
}
