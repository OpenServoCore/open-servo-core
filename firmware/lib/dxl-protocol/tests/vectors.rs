use dxl_protocol::{Bytes, Instruction, MAX_LENGTH, Packet, ParseError, crc16, parse_one, write};
use heapless::Vec;

const PING_ID1: &[u8] = &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E];

const READ_ID1_ADDR132_LEN4: &[u8] = &[
    0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15,
];

// Write 512 (0x00000200) to Goal Position (address 116, 4 bytes) on ID 1.
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
            crc16(body),
            expected,
            "CRC mismatch on frame: {:02X?}",
            frame
        );
    }
}

#[test]
fn parse_ping() {
    let (pkt, n) = parse_one(PING_ID1).unwrap();
    assert_eq!(n, PING_ID1.len());
    match pkt {
        Packet::Ping { id } => assert_eq!(id, 1),
        other => panic!("not Ping: {:?}", other),
    }
}

#[test]
fn parse_read() {
    let (pkt, n) = parse_one(READ_ID1_ADDR132_LEN4).unwrap();
    assert_eq!(n, READ_ID1_ADDR132_LEN4.len());
    match pkt {
        Packet::Read {
            id,
            address,
            length,
        } => {
            assert_eq!(id, 1);
            assert_eq!(address, 132);
            assert_eq!(length, 4);
        }
        other => panic!("not Read: {:?}", other),
    }
}

#[test]
fn parse_write() {
    let (pkt, n) = parse_one(WRITE_ID1_GOAL512).unwrap();
    assert_eq!(n, WRITE_ID1_GOAL512.len());
    match pkt {
        Packet::Write { id, address, data } => {
            assert_eq!(id, 1);
            assert_eq!(address, 116);
            let mut buf = [0u8; 16];
            let n = data.copy_into(&mut buf).unwrap();
            assert_eq!(&buf[..n], &[0x00, 0x02, 0x00, 0x00]);
        }
        other => panic!("not Write: {:?}", other),
    }
}

#[test]
fn parse_reboot() {
    let (pkt, _) = parse_one(REBOOT_ID1).unwrap();
    assert!(matches!(pkt, Packet::Reboot { id: 1 }));
}

#[test]
fn write_ping_matches_reference() {
    let mut out: Vec<u8, 32> = Vec::new();
    write(&mut out, &Packet::Ping { id: 1 }).unwrap();
    assert_eq!(&out[..], PING_ID1);
}

#[test]
fn write_read_matches_reference() {
    let mut out: Vec<u8, 32> = Vec::new();
    write(
        &mut out,
        &Packet::Read {
            id: 1,
            address: 132,
            length: 4,
        },
    )
    .unwrap();
    assert_eq!(&out[..], READ_ID1_ADDR132_LEN4);
}

#[test]
fn write_write_matches_reference() {
    let data = [0x00u8, 0x02, 0x00, 0x00];
    let mut out: Vec<u8, 32> = Vec::new();
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 116,
            data: Bytes::Raw(&data),
        },
    )
    .unwrap();
    assert_eq!(&out[..], WRITE_ID1_GOAL512);
}

#[test]
fn write_reboot_matches_reference() {
    let mut out: Vec<u8, 32> = Vec::new();
    write(&mut out, &Packet::Reboot { id: 1 }).unwrap();
    assert_eq!(&out[..], REBOOT_ID1);
}

#[test]
fn write_status_round_trip() {
    let params = [0x06u8, 0x04, 0x26];
    let mut out: Vec<u8, 64> = Vec::new();
    write(
        &mut out,
        &Packet::Status {
            id: 1,
            error: 0,
            params: Bytes::Raw(&params),
        },
    )
    .unwrap();

    let (pkt, n) = parse_one(&out).unwrap();
    assert_eq!(n, out.len());
    match pkt {
        Packet::Status {
            id,
            error,
            params: p,
        } => {
            assert_eq!(id, 1);
            assert_eq!(error, 0);
            let mut buf = [0u8; 16];
            let n = p.copy_into(&mut buf).unwrap();
            assert_eq!(&buf[..n], &params);
        }
        other => panic!("not Status: {:?}", other),
    }
}

#[test]
fn stuffing_round_trip() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xFD, 0x55];
    let mut out: Vec<u8, 64> = Vec::new();
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0x0040,
            data: Bytes::Raw(&data),
        },
    )
    .unwrap();

    let raw_count = out.iter().filter(|&&b| b == 0xFD).count();
    assert!(raw_count >= 4, "expected stuffed bytes: {:02X?}", out);

    let (pkt, n) = parse_one(&out).unwrap();
    assert_eq!(n, out.len());
    match pkt {
        Packet::Write {
            id,
            address,
            data: d,
        } => {
            assert_eq!(id, 1);
            assert_eq!(address, 0x0040);
            assert_eq!(d.unstuffed_len(), data.len());
            let mut buf = [0u8; 32];
            let n = d.copy_into(&mut buf).unwrap();
            assert_eq!(&buf[..n], &data);
        }
        other => panic!("not Write: {:?}", other),
    }
}

#[test]
fn stuffing_at_field_boundary() {
    let data = [0xFDu8, 0x11, 0x22];
    let mut out: Vec<u8, 64> = Vec::new();
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0xFFFF,
            data: Bytes::Raw(&data),
        },
    )
    .unwrap();

    let (pkt, _) = parse_one(&out).unwrap();
    match pkt {
        Packet::Write {
            address, data: d, ..
        } => {
            assert_eq!(address, 0xFFFF);
            let mut buf = [0u8; 16];
            let n = d.copy_into(&mut buf).unwrap();
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

    match parse_one(&buf) {
        Err(ParseError::Resync { skip: 3 }) => {}
        other => panic!("expected Resync(3), got {:?}", other),
    }

    let (pkt, n) = parse_one(&buf[3..]).unwrap();
    assert_eq!(n, PING_ID1.len());
    assert!(matches!(pkt, Packet::Ping { id: 1 }));
}

#[test]
fn incomplete_returns_err() {
    let partial = &PING_ID1[..6];
    assert!(matches!(parse_one(partial), Err(ParseError::Incomplete)));
    let partial = &PING_ID1[..7];
    assert!(matches!(parse_one(partial), Err(ParseError::Incomplete)));
}

#[test]
fn bad_crc_is_reported() {
    let mut bad: Vec<u8, 32> = Vec::new();
    bad.extend_from_slice(PING_ID1).unwrap();
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    match parse_one(&bad) {
        Err(ParseError::BadCrc { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadCrc, got {:?}", other),
    }
}

#[test]
fn oversized_length_is_rejected() {
    // Crafted "header" with length = 0xFFFF, well above MAX_LENGTH.
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    assert!((MAX_LENGTH as u32) < 0xFFFF);
    match parse_one(&bad) {
        Err(ParseError::BadLength { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadLength, got {:?}", other),
    }
}

#[test]
fn undersized_length_is_rejected() {
    // length = 2 is below the 3-byte minimum (instruction + crc).
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0x02, 0x00, 0x01];
    match parse_one(&bad) {
        Err(ParseError::BadLength { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadLength, got {:?}", other),
    }
}

#[test]
fn false_header_with_huge_length_does_not_wedge() {
    // A spurious header with length = 0xFFFF would, before the cap, cause
    // the parser to wait on ~64 KB of phantom data. With the cap it returns
    // BadLength immediately, the caller drops 4 bytes, and the real PING
    // sitting after the noise is recoverable.
    let mut buf: Vec<u8, 64> = Vec::new();
    buf.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x42, 0xFF, 0xFF, 0x99])
        .unwrap();
    buf.extend_from_slice(PING_ID1).unwrap();

    let skip = match parse_one(&buf) {
        Err(ParseError::BadLength { skip }) => skip,
        other => panic!("expected BadLength, got {:?}", other),
    };
    assert_eq!(skip, 4);

    // After dropping the false header, the next call resyncs onto the real
    // PING frame instead of skipping past it.
    let rest = &buf[skip..];
    let resync = match parse_one(rest) {
        Err(ParseError::Resync { skip }) => skip,
        other => panic!("expected Resync, got {:?}", other),
    };
    let (pkt, n) = parse_one(&rest[resync..]).unwrap();
    assert_eq!(n, PING_ID1.len());
    assert!(matches!(pkt, Packet::Ping { id: 1 }));
}

#[test]
fn bad_instruction_is_reported() {
    let mut frame: Vec<u8, 32> = Vec::new();
    frame
        .extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x77])
        .unwrap();
    let crc = crc16(&frame);
    frame.extend_from_slice(&crc.to_le_bytes()).unwrap();
    match parse_one(&frame) {
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
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(data),
        },
    )
    .unwrap();
    let (pkt, n) = parse_one(&out).unwrap();
    assert_eq!(n, out.len());
    match pkt {
        Packet::Write {
            data: d, address, ..
        } => {
            assert_eq!(address, 0x0010);
            let mut buf = [0u8; 256];
            let n = d.copy_into(&mut buf).unwrap();
            let mut got: heapless::Vec<u8, 256> = Vec::new();
            got.extend_from_slice(&buf[..n]).unwrap();
            assert_eq!(d.unstuffed_len(), data.len());
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
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0xFFFF,
            data: Bytes::Raw(&data),
        },
    )
    .unwrap();
    let (pkt, _) = parse_one(&out).unwrap();
    let Packet::Write {
        address, data: d, ..
    } = pkt
    else {
        panic!("not Write");
    };
    assert_eq!(address, 0xFFFF);
    assert_eq!(d.unstuffed_len(), data.len());
    let mut buf = [0u8; 8];
    let n = d.copy_into(&mut buf).unwrap();
    assert_eq!(&buf[..n], &data);
}

#[test]
fn stuff_trigger_spanning_address_then_more_in_data() {
    let data = [0xFDu8, 0x42, 0xFF, 0xFF, 0xFD, 0x55, 0x66];
    let mut out: Vec<u8, 64> = Vec::new();
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0xFFFF,
            data: Bytes::Raw(&data),
        },
    )
    .unwrap();
    let (pkt, _) = parse_one(&out).unwrap();
    let Packet::Write { data: d, .. } = pkt else {
        panic!();
    };
    let mut buf = [0u8; 16];
    let n = d.copy_into(&mut buf).unwrap();
    assert_eq!(&buf[..n], &data);
}

#[test]
fn stuff_unstuffed_len_matches_iter_count() {
    let data = [0xFFu8, 0xFF, 0xFD, 0x00, 0xFF, 0xFF, 0xFD, 0xFD];
    let mut out: Vec<u8, 64> = Vec::new();
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&data),
        },
    )
    .unwrap();
    let (pkt, _) = parse_one(&out).unwrap();
    let Packet::Write { data: d, .. } = pkt else {
        panic!();
    };
    assert_eq!(d.unstuffed_len(), data.len());
    assert_eq!(d.iter().count(), data.len());
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
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&[]),
        },
    )
    .unwrap();
    let (pkt, _) = parse_one(&out).unwrap();
    let Packet::Write { data: d, .. } = pkt else {
        panic!();
    };
    assert_eq!(d.unstuffed_len(), 0);
    assert_eq!(d.iter().next(), None);
}

#[test]
fn stuff_status_with_trigger_in_params() {
    let params = [0xFFu8, 0xFF, 0xFD, 0x00, 0x42];
    let mut out: Vec<u8, 64> = Vec::new();
    write(
        &mut out,
        &Packet::Status {
            id: 1,
            error: 0,
            params: Bytes::Raw(&params),
        },
    )
    .unwrap();
    let (pkt, _) = parse_one(&out).unwrap();
    let Packet::Status {
        error, params: p, ..
    } = pkt
    else {
        panic!();
    };
    assert_eq!(error, 0);
    let mut buf = [0u8; 16];
    let n = p.copy_into(&mut buf).unwrap();
    assert_eq!(&buf[..n], &params);
}

#[test]
fn stuff_forwarding_round_trip() {
    let original_data = [0xFFu8, 0xFF, 0xFD, 0x42, 0xAA];
    let mut wire1: Vec<u8, 64> = Vec::new();
    write(
        &mut wire1,
        &Packet::Write {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&original_data),
        },
    )
    .unwrap();

    let (pkt, _) = parse_one(&wire1).unwrap();
    let mut wire2: Vec<u8, 64> = Vec::new();
    write(&mut wire2, &pkt).unwrap();
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
    write(
        &mut out,
        &Packet::Write {
            id: 1,
            address: 0x0010,
            data: Bytes::Raw(&data),
        },
    )
    .unwrap();
    let (pkt, _) = parse_one(&out).unwrap();
    let Packet::Write { data: d, .. } = pkt else {
        panic!();
    };
    assert_eq!(d.unstuffed_len(), data.len());
    let mut buf = [0u8; 64];
    let n = d.copy_into(&mut buf).unwrap();
    assert_eq!(&buf[..n], &data[..]);
}
