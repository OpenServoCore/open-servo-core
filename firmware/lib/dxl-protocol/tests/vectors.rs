use dxl_protocol::Instruction;
use dxl_protocol::prelude::*;
use heapless::Vec;

type Wire = Codec<SoftwareCrcUmts>;

const PING_ID1: &[u8] = &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E];

const READ_ID1_ADDR132_LEN4: &[u8] = &[
    0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15,
];

// Write 512 to Goal Position (addr 116, 4 B) on ID 1.
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
fn write_fast_slot_first_emits_header_then_body() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::First {
                packet_length: 0x0015,
            },
            id: 5,
            data: BODY_5,
        },
    )
    .unwrap();
    assert_eq!(
        out.as_slice(),
        &[
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x15, 0x00, 0x55, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00,
        ]
    );
}

#[test]
fn write_fast_slot_middle_emits_body_only() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::Middle,
            id: 6,
            data: BODY_6,
        },
    )
    .unwrap();
    assert_eq!(out.as_slice(), &[0x00, 0x06, 0x02, 0x00, 0x00, 0x00]);
}

#[test]
fn write_fast_slot_last_reserves_crc_placeholder() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::Last,
            id: 7,
            data: BODY_7,
        },
    )
    .unwrap();
    assert_eq!(
        out.as_slice(),
        &[0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0xAA, 0xBB]
    );
}

#[test]
fn write_fast_slot_only_emits_header_body_and_computed_crc() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::Only {
                packet_length: 0x0009,
            },
            id: 5,
            data: BODY_5,
        },
    )
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
fn write_fast_slot_three_slave_chain_assembles_to_valid_status_frame() {
    let mut out: Vec<u8, 64> = Vec::new();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::First {
                packet_length: 0x0015,
            },
            id: 5,
            data: BODY_5,
        },
    )
    .unwrap();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::Middle,
            id: 6,
            data: BODY_6,
        },
    )
    .unwrap();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::Last,
            id: 7,
            data: BODY_7,
        },
    )
    .unwrap();

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
fn write_fast_slot_overflow_truncates() {
    let mut out: Vec<u8, 8> = Vec::new();
    let err = Wire::write_status_reply(
        &mut out,
        &StatusReply::FastSyncRead {
            position: FastPosition::First {
                packet_length: 0x0015,
            },
            id: 5,
            data: BODY_5,
        },
    )
    .unwrap_err();
    assert_eq!(err, WriteError::Overflow);
    assert!(out.is_empty());
}

#[test]
fn write_fast_error_emits_zero_payload_with_error_byte() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write_status_reply(
        &mut out,
        &StatusReply::FastError {
            position: FastPosition::Middle,
            id: 7,
            error: StatusError::DataRange,
            length: 4,
        },
    )
    .unwrap();
    // Middle body shape: error, id, then 4 zero bytes of payload.
    assert_eq!(
        out.as_slice(),
        &[StatusError::DataRange.as_u8(), 0x07, 0, 0, 0, 0]
    );
}

#[test]
fn parse_ping() {
    let (pkt, n) = Wire::parse_one(PING_ID1).unwrap();
    assert_eq!(n, PING_ID1.len());
    match pkt {
        Packet::Ping(p) => assert_eq!(p.id, 1),
        other => panic!("not Ping: {:?}", other),
    }
}

#[test]
fn parse_read() {
    let (pkt, n) = Wire::parse_one(READ_ID1_ADDR132_LEN4).unwrap();
    assert_eq!(n, READ_ID1_ADDR132_LEN4.len());
    match pkt {
        Packet::Read(p) => {
            assert_eq!(p.id, 1);
            assert_eq!(p.address, 132);
            assert_eq!(p.length, 4);
        }
        other => panic!("not Read: {:?}", other),
    }
}

#[test]
fn parse_write() {
    let (pkt, n) = Wire::parse_one(WRITE_ID1_GOAL512).unwrap();
    assert_eq!(n, WRITE_ID1_GOAL512.len());
    match pkt {
        Packet::Write(p) => {
            assert_eq!(p.id, 1);
            assert_eq!(p.address, 116);
            let mut buf = [0u8; 16];
            let n = p.data.copy_to_slice(&mut buf).unwrap();
            assert_eq!(&buf[..n], &[0x00, 0x02, 0x00, 0x00]);
        }
        other => panic!("not Write: {:?}", other),
    }
}

#[test]
fn parse_reboot() {
    let (pkt, _) = Wire::parse_one(REBOOT_ID1).unwrap();
    assert!(matches!(pkt, Packet::Reboot(RebootPacket { id: 1 })));
}

#[test]
fn write_ping_matches_reference() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write(&mut out, &Packet::Ping(PingPacket { id: 1 })).unwrap();
    assert_eq!(&out[..], PING_ID1);
}

#[test]
fn write_read_matches_reference() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Read(ReadPacket {
            id: 1,
            address: 132,
            length: 4,
        }),
    )
    .unwrap();
    assert_eq!(&out[..], READ_ID1_ADDR132_LEN4);
}

#[test]
fn write_write_matches_reference() {
    let data = [0x00u8, 0x02, 0x00, 0x00];
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 116,
            data: Bytes::Raw(&data),
        }),
    )
    .unwrap();
    assert_eq!(&out[..], WRITE_ID1_GOAL512);
}

#[test]
fn write_reboot_matches_reference() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write(&mut out, &Packet::Reboot(RebootPacket { id: 1 })).unwrap();
    assert_eq!(&out[..], REBOOT_ID1);
}

#[cfg(feature = "osc")]
#[test]
fn calibrate_round_trip() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Calibrate(CalibratePacket { id: 1, count: 128 }),
    )
    .unwrap();
    let (pkt, n) = Wire::parse_one(&out).unwrap();
    assert_eq!(n, out.len());
    assert!(matches!(
        pkt,
        Packet::Calibrate(CalibratePacket { id: 1, count: 128 })
    ));
}

#[cfg(feature = "osc")]
#[test]
fn parse_calibrate_broadcast() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Calibrate(CalibratePacket {
            id: BROADCAST_ID,
            count: 64,
        }),
    )
    .unwrap();
    let (pkt, _) = Wire::parse_one(&out).unwrap();
    assert!(matches!(
        pkt,
        Packet::Calibrate(CalibratePacket {
            id: BROADCAST_ID,
            count: 64
        })
    ));
}

#[test]
fn write_status_round_trip() {
    let params = [0x06u8, 0x04, 0x26];
    let mut out: Vec<u8, 64> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Status(StatusPacket {
            id: 1,
            error: 0,
            params: Bytes::Raw(&params),
        }),
    )
    .unwrap();

    let (pkt, n) = Wire::parse_one(&out).unwrap();
    assert_eq!(n, out.len());
    match pkt {
        Packet::Status(p) => {
            assert_eq!(p.id, 1);
            assert_eq!(p.error, 0);
            let mut buf = [0u8; 16];
            let n = p.params.copy_to_slice(&mut buf).unwrap();
            assert_eq!(&buf[..n], &params);
        }
        other => panic!("not Status: {:?}", other),
    }
}

#[test]
fn stuffing_round_trip() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xFD, 0x55];
    let mut out: Vec<u8, 64> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0x0040,
            data: Bytes::Raw(&data),
        }),
    )
    .unwrap();

    let raw_count = out.iter().filter(|&&b| b == 0xFD).count();
    assert!(raw_count >= 4, "expected stuffed bytes: {:02X?}", out);

    let (pkt, n) = Wire::parse_one(&out).unwrap();
    assert_eq!(n, out.len());
    match pkt {
        Packet::Write(p) => {
            assert_eq!(p.id, 1);
            assert_eq!(p.address, 0x0040);
            assert_eq!(p.data.unstuffed_len(), data.len());
            let mut buf = [0u8; 32];
            let n = p.data.copy_to_slice(&mut buf).unwrap();
            assert_eq!(&buf[..n], &data);
        }
        other => panic!("not Write: {:?}", other),
    }
}

#[test]
fn stuffing_at_field_boundary() {
    let data = [0xFDu8, 0x11, 0x22];
    let mut out: Vec<u8, 64> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0xFFFF,
            data: Bytes::Raw(&data),
        }),
    )
    .unwrap();

    let (pkt, _) = Wire::parse_one(&out).unwrap();
    match pkt {
        Packet::Write(p) => {
            assert_eq!(p.address, 0xFFFF);
            let mut buf = [0u8; 16];
            let n = p.data.copy_to_slice(&mut buf).unwrap();
            assert_eq!(&buf[..n], &data);
        }
        _ => panic!("not Write"),
    }
}

#[test]
fn resync_skips_leading_garbage() {
    let mut buf: Vec<u8, 64> = Vec::new();
    buf.extend_from_slice(&[0x00, 0x11, 0x22]).unwrap();
    buf.extend_from_slice(PING_ID1).unwrap();

    match Wire::parse_one(&buf) {
        Err(ParseError::Resync { skip: 3 }) => {}
        other => panic!("expected Resync(3), got {:?}", other),
    }

    let (pkt, n) = Wire::parse_one(&buf[3..]).unwrap();
    assert_eq!(n, PING_ID1.len());
    assert!(matches!(pkt, Packet::Ping(PingPacket { id: 1 })));
}

#[test]
fn incomplete_returns_err() {
    let partial = &PING_ID1[..6];
    assert!(matches!(
        Wire::parse_one(partial),
        Err(ParseError::Incomplete)
    ));
    let partial = &PING_ID1[..7];
    assert!(matches!(
        Wire::parse_one(partial),
        Err(ParseError::Incomplete)
    ));
}

#[test]
fn bad_crc_is_reported() {
    let mut bad: Vec<u8, 32> = Vec::new();
    bad.extend_from_slice(PING_ID1).unwrap();
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    match Wire::parse_one(&bad) {
        Err(ParseError::BadCrc { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadCrc, got {:?}", other),
    }
}

#[test]
fn oversized_length_is_rejected() {
    // length 0xFFFF > MAX_LENGTH.
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    assert!((MAX_LENGTH as u32) < 0xFFFF);
    match Wire::parse_one(&bad) {
        Err(ParseError::BadLength { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadLength, got {:?}", other),
    }
}

#[test]
fn undersized_length_is_rejected() {
    // length 2 < min 3 (instruction + crc).
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0x02, 0x00, 0x01];
    match Wire::parse_one(&bad) {
        Err(ParseError::BadLength { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadLength, got {:?}", other),
    }
}

#[test]
fn false_header_with_huge_length_does_not_wedge() {
    // Phantom header with length=0xFFFF: without cap, parser would wait for ~64 KB.
    let mut buf: Vec<u8, 64> = Vec::new();
    buf.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x42, 0xFF, 0xFF, 0x99])
        .unwrap();
    buf.extend_from_slice(PING_ID1).unwrap();

    let skip = match Wire::parse_one(&buf) {
        Err(ParseError::BadLength { skip }) => skip,
        other => panic!("expected BadLength, got {:?}", other),
    };
    assert_eq!(skip, 4);

    let rest = &buf[skip..];
    let resync = match Wire::parse_one(rest) {
        Err(ParseError::Resync { skip }) => skip,
        other => panic!("expected Resync, got {:?}", other),
    };
    let (pkt, n) = Wire::parse_one(&rest[resync..]).unwrap();
    assert_eq!(n, PING_ID1.len());
    assert!(matches!(pkt, Packet::Ping(PingPacket { id: 1 })));
}

#[test]
fn bad_instruction_is_reported() {
    let mut frame: Vec<u8, 32> = Vec::new();
    frame
        .extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x77])
        .unwrap();
    let crc = SoftwareCrcUmts::accumulate(0, &frame);
    frame.extend_from_slice(&crc.to_le_bytes()).unwrap();
    match Wire::parse_one(&frame) {
        Err(ParseError::BadInstruction { skip }) => assert_eq!(skip, frame.len()),
        other => panic!("expected BadInstruction, got {:?}", other),
    }
}

#[test]
fn instruction_byte_constants() {
    assert_eq!(Instruction::Ping.as_u8(), 0x01);
    assert_eq!(Instruction::Status.as_u8(), 0x55);
    assert_eq!(Instruction::SyncWrite.as_u8(), 0x83);
    assert_eq!(Instruction::FastBulkRead.as_u8(), 0x9A);
}

fn round_trip_data(data: &[u8]) -> heapless::Vec<u8, 256> {
    let mut out: Vec<u8, 256> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(data),
        }),
    )
    .unwrap();
    let (pkt, n) = Wire::parse_one(&out).unwrap();
    assert_eq!(n, out.len());
    match pkt {
        Packet::Write(p) => {
            assert_eq!(p.address, 0x0010);
            let mut buf = [0u8; 256];
            let n = p.data.copy_to_slice(&mut buf).unwrap();
            let mut got: heapless::Vec<u8, 256> = Vec::new();
            got.extend_from_slice(&buf[..n]).unwrap();
            assert_eq!(p.data.unstuffed_len(), data.len());
            assert_eq!(&got[..], data);
            out
        }
        _ => panic!("not Write"),
    }
}

fn fds_in_payload_region(frame: &[u8]) -> usize {
    let payload = &frame[8..frame.len() - 2];
    payload.iter().filter(|&&b| b == 0xFD).count()
}

#[test]
fn stuff_no_trigger() {
    let data = [0x01u8, 0x02, 0x03, 0x04];
    let frame = round_trip_data(&data);
    assert_eq!(fds_in_payload_region(&frame), 0);
}

#[test]
fn stuff_single_trigger_inserts_one_byte() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x00];
    let frame = round_trip_data(&data);
    assert_eq!(fds_in_payload_region(&frame), 2);
}

#[test]
fn stuff_multiple_triggers() {
    let data = [
        0xFFu8, 0xFF, 0xFD, 0x01, 0x02, 0xFF, 0xFF, 0xFD, 0x03, 0xAA, 0xFF, 0xFF, 0xFD,
    ];
    let frame = round_trip_data(&data);
    let logical_fds = data.iter().filter(|&&b| b == 0xFD).count();
    assert_eq!(fds_in_payload_region(&frame), logical_fds + 3);
}

#[test]
fn stuff_trigger_at_payload_start() {
    let data = [0xFFu8, 0xFF, 0xFD, 0xAA, 0xBB];
    let _ = round_trip_data(&data);
}

#[test]
fn stuff_trigger_at_payload_end() {
    let data = [0xAAu8, 0xBB, 0xFF, 0xFF, 0xFD];
    let frame = round_trip_data(&data);
    assert_eq!(fds_in_payload_region(&frame), 2);
}

#[test]
fn stuff_logical_fd_after_trigger() {
    let data = [0xFFu8, 0xFF, 0xFD, 0xFD, 0x55];
    let _ = round_trip_data(&data);
}

#[test]
fn stuff_two_adjacent_triggers() {
    let data = [0xFFu8, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD, 0xAA];
    let _ = round_trip_data(&data);
}

#[test]
fn stuff_trigger_spanning_address_boundary() {
    let mut out: Vec<u8, 64> = Vec::new();
    let data = [0xFDu8, 0x11, 0x22];
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0xFFFF,
            data: Bytes::Raw(&data),
        }),
    )
    .unwrap();
    let (pkt, _) = Wire::parse_one(&out).unwrap();
    let Packet::Write(p) = pkt else {
        panic!("not Write");
    };
    assert_eq!(p.address, 0xFFFF);
    assert_eq!(p.data.unstuffed_len(), data.len());
    let mut buf = [0u8; 8];
    let n = p.data.copy_to_slice(&mut buf).unwrap();
    assert_eq!(&buf[..n], &data);
}

#[test]
fn stuff_trigger_spanning_address_then_more_in_data() {
    let data = [0xFDu8, 0x42, 0xFF, 0xFF, 0xFD, 0x55, 0x66];
    let mut out: Vec<u8, 64> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0xFFFF,
            data: Bytes::Raw(&data),
        }),
    )
    .unwrap();
    let (pkt, _) = Wire::parse_one(&out).unwrap();
    let Packet::Write(p) = pkt else {
        panic!();
    };
    let mut buf = [0u8; 16];
    let n = p.data.copy_to_slice(&mut buf).unwrap();
    assert_eq!(&buf[..n], &data);
}

#[test]
fn stuff_unstuffed_len_matches_iter_count() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x00, 0xFF, 0xFF, 0xFD, 0xFD];
    let mut out: Vec<u8, 64> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&data),
        }),
    )
    .unwrap();
    let (pkt, _) = Wire::parse_one(&out).unwrap();
    let Packet::Write(p) = pkt else {
        panic!();
    };
    assert_eq!(p.data.unstuffed_len(), data.len());
    assert_eq!(p.data.iter().count(), data.len());
}

#[test]
fn stuff_raw_passthrough_does_not_unstuff() {
    let raw = [0xFFu8, 0xFF, 0xFD, 0xFD, 0xAA];
    let bytes = Bytes::Raw(&raw);
    let collected: heapless::Vec<u8, 16> = bytes.iter().collect();
    assert_eq!(&collected[..], &raw);
    assert_eq!(bytes.unstuffed_len(), raw.len());
}

#[test]
fn stuff_empty_payload() {
    let mut out: Vec<u8, 32> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&[]),
        }),
    )
    .unwrap();
    let (pkt, _) = Wire::parse_one(&out).unwrap();
    let Packet::Write(p) = pkt else {
        panic!();
    };
    assert_eq!(p.data.unstuffed_len(), 0);
    assert_eq!(p.data.iter().next(), None);
}

#[test]
fn stuff_status_with_trigger_in_params() {
    let params = [0xFFu8, 0xFF, 0xFD, 0x00, 0x42];
    let mut out: Vec<u8, 64> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Status(StatusPacket {
            id: 1,
            error: 0,
            params: Bytes::Raw(&params),
        }),
    )
    .unwrap();
    let (pkt, _) = Wire::parse_one(&out).unwrap();
    let Packet::Status(p) = pkt else {
        panic!();
    };
    assert_eq!(p.error, 0);
    let mut buf = [0u8; 16];
    let n = p.params.copy_to_slice(&mut buf).unwrap();
    assert_eq!(&buf[..n], &params);
}

#[test]
fn stuff_forwarding_round_trip() {
    let original_data = [0xFFu8, 0xFF, 0xFD, 0x42, 0xAA];
    let mut wire1: Vec<u8, 64> = Vec::new();
    Wire::write(
        &mut wire1,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&original_data),
        }),
    )
    .unwrap();

    let (pkt, _) = Wire::parse_one(&wire1).unwrap();
    let mut wire2: Vec<u8, 64> = Vec::new();
    Wire::write(&mut wire2, &pkt).unwrap();
    assert_eq!(&wire1[..], &wire2[..]);
}

#[test]
fn stuff_long_payload_with_many_triggers() {
    let mut data: heapless::Vec<u8, 64> = Vec::new();
    for _ in 0..6 {
        data.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x42, 0x55])
            .unwrap();
    }
    let mut out: Vec<u8, 128> = Vec::new();
    Wire::write(
        &mut out,
        &Packet::Write(WritePacket {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&data),
        }),
    )
    .unwrap();
    let (pkt, _) = Wire::parse_one(&out).unwrap();
    let Packet::Write(p) = pkt else {
        panic!();
    };
    assert_eq!(p.data.unstuffed_len(), data.len());
    let mut buf = [0u8; 64];
    let n = p.data.copy_to_slice(&mut buf).unwrap();
    assert_eq!(&buf[..n], &data[..]);
}
