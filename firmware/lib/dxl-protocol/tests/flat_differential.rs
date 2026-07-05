//! Differential: encode with the existing encoders, parse + decode with the
//! new flat codec, assert the decoded structs equal the inputs. The tests
//! read as a spec for the round-trip contract.

use dxl_protocol::frame::{FrameKind, RawFrame, parse};
use dxl_protocol::types::packet::{
    DecodeError, Instruction as Decoded, StatusReply, decode_instruction, decode_status,
};
use dxl_protocol::{
    BulkReadEntry, Bytes, Id, InstructionEncoder, SoftwareCrcUmts as Crc, StatusEncoder,
    StatusError,
};
use heapless::Vec as HVec;

type Buf = HVec<u8, 256>;

fn unstuffed(b: &Bytes) -> HVec<u8, 256> {
    b.iter().collect()
}

/// Encode via `f`, then parse the single frame back out.
fn round_trip<F>(f: F) -> Buf
where
    F: FnOnce(&mut InstructionEncoder<'_, Buf, Crc>),
{
    let mut buf = Buf::new();
    {
        let mut enc = InstructionEncoder::<_, Crc>::new(&mut buf);
        f(&mut enc);
    }
    buf
}

fn decode<'a>(wire: &'a [u8]) -> (RawFrame<'a>, Decoded<'a>) {
    let (frame, n) = parse::<Crc>(wire).expect("parse");
    assert_eq!(n, wire.len(), "parse consumed a short frame");
    let decoded = decode_instruction(&frame).expect("decode_instruction");
    (frame, decoded)
}

#[test]
fn ping_round_trips() {
    let wire = round_trip(|e| e.ping(Id::new(0x11)).unwrap());
    let (_, d) = decode(&wire);
    assert!(matches!(d, Decoded::Ping { id } if id == Id::new(0x11)));
}

#[test]
fn read_round_trips() {
    let wire = round_trip(|e| e.read(Id::new(0x02), 0x0084, 4).unwrap());
    let (_, d) = decode(&wire);
    assert!(matches!(
        d,
        Decoded::Read { id, address: 0x0084, length: 4 } if id == Id::new(0x02)
    ));
}

#[test]
fn write_round_trips_with_data() {
    let data = [0xAA, 0xBB, 0xCC, 0xDD];
    let wire = round_trip(|e| e.write(Id::new(0x03), 0x0074, &data).unwrap());
    let (_, d) = decode(&wire);
    match d {
        Decoded::Write {
            id,
            address,
            data: got,
        } => {
            assert_eq!(id, Id::new(0x03));
            assert_eq!(address, 0x0074);
            assert_eq!(unstuffed(&got).as_slice(), &data);
        }
        other => panic!("expected Write, got {other:?}"),
    }
}

#[test]
fn reg_write_round_trips() {
    let data = [0x10, 0x20];
    let wire = round_trip(|e| e.reg_write(Id::new(0x04), 0x0030, &data).unwrap());
    let (_, d) = decode(&wire);
    match d {
        Decoded::RegWrite {
            id,
            address,
            data: got,
        } => {
            assert_eq!(id, Id::new(0x04));
            assert_eq!(address, 0x0030);
            assert_eq!(unstuffed(&got).as_slice(), &data);
        }
        other => panic!("expected RegWrite, got {other:?}"),
    }
}

#[test]
fn action_and_reboot_round_trip() {
    let wire = round_trip(|e| e.action(Id::new(0x05)).unwrap());
    assert!(matches!(decode(&wire).1, Decoded::Action { id } if id == Id::new(0x05)));

    let wire = round_trip(|e| e.reboot(Id::new(0x06)).unwrap());
    assert!(matches!(decode(&wire).1, Decoded::Reboot { id } if id == Id::new(0x06)));
}

#[test]
fn factory_reset_carries_mode() {
    let wire = round_trip(|e| e.factory_reset(Id::new(0x07), 0xFF).unwrap());
    assert!(matches!(
        decode(&wire).1,
        Decoded::FactoryReset { id, mode: 0xFF } if id == Id::new(0x07)
    ));
}

#[test]
fn clear_and_ctb_carry_body() {
    let body = [0x01, 0x44, 0x58, 0x4C, 0x22];
    let wire = round_trip(|e| e.clear(Id::new(0x08), &body).unwrap());
    match decode(&wire).1 {
        Decoded::Clear { id, body: got } => {
            assert_eq!(id, Id::new(0x08));
            assert_eq!(unstuffed(&got).as_slice(), &body);
        }
        other => panic!("expected Clear, got {other:?}"),
    }

    let ctb = [0xAA, 0xBB];
    let wire = round_trip(|e| e.control_table_backup(Id::new(0x09), &ctb).unwrap());
    match decode(&wire).1 {
        Decoded::ControlTableBackup { id, body: got } => {
            assert_eq!(id, Id::new(0x09));
            assert_eq!(unstuffed(&got).as_slice(), &ctb);
        }
        other => panic!("expected ControlTableBackup, got {other:?}"),
    }
}

#[test]
fn sync_read_carries_ids() {
    let ids = [0x01, 0x02, 0x03];
    let wire = round_trip(|e| e.sync_read(0x0084, 4, &ids).unwrap());
    match decode(&wire).1 {
        Decoded::SyncRead {
            address,
            length,
            ids: got,
        } => {
            assert_eq!(address, 0x0084);
            assert_eq!(length, 4);
            assert_eq!(unstuffed(&got).as_slice(), &ids);
        }
        other => panic!("expected SyncRead, got {other:?}"),
    }
}

#[test]
fn sync_write_carries_body() {
    let body = [0x01, 0xAA, 0xBB, 0x02, 0xCC, 0xDD];
    let wire = round_trip(|e| e.sync_write(0x0080, 2, &body).unwrap());
    match decode(&wire).1 {
        Decoded::SyncWrite {
            address,
            length,
            body: got,
        } => {
            assert_eq!(address, 0x0080);
            assert_eq!(length, 2);
            assert_eq!(unstuffed(&got).as_slice(), &body);
        }
        other => panic!("expected SyncWrite, got {other:?}"),
    }
}

#[test]
fn bulk_read_and_write_carry_body() {
    let entries = [
        BulkReadEntry {
            id: Id::new(1),
            address: 0x0084,
            length: 4,
        },
        BulkReadEntry {
            id: Id::new(2),
            address: 0x0090,
            length: 2,
        },
    ];
    let wire = round_trip(|e| e.bulk_read(&entries).unwrap());
    match decode(&wire).1 {
        Decoded::BulkRead { body } => {
            assert_eq!(
                unstuffed(&body).as_slice(),
                &[0x01, 0x84, 0x00, 0x04, 0x00, 0x02, 0x90, 0x00, 0x02, 0x00]
            );
        }
        other => panic!("expected BulkRead, got {other:?}"),
    }

    let bw = [0x01, 0x84, 0x00, 0x02, 0x00, 0xAA, 0xBB];
    let wire = round_trip(|e| e.bulk_write(&bw).unwrap());
    match decode(&wire).1 {
        Decoded::BulkWrite { body } => assert_eq!(unstuffed(&body).as_slice(), &bw),
        other => panic!("expected BulkWrite, got {other:?}"),
    }
}

#[test]
fn fast_sync_and_bulk_read_round_trip() {
    let ids = [0x01, 0x02];
    let wire = round_trip(|e| e.fast_sync_read(0x0084, 4, &ids).unwrap());
    match decode(&wire).1 {
        Decoded::FastSyncRead {
            address,
            length,
            ids: got,
        } => {
            assert_eq!(address, 0x0084);
            assert_eq!(length, 4);
            assert_eq!(unstuffed(&got).as_slice(), &ids);
        }
        other => panic!("expected FastSyncRead, got {other:?}"),
    }

    let entries = [BulkReadEntry {
        id: Id::new(1),
        address: 0x0084,
        length: 4,
    }];
    let wire = round_trip(|e| e.fast_bulk_read(&entries).unwrap());
    match decode(&wire).1 {
        Decoded::FastBulkRead { body } => {
            assert_eq!(unstuffed(&body).as_slice(), &[0x01, 0x84, 0x00, 0x04, 0x00])
        }
        other => panic!("expected FastBulkRead, got {other:?}"),
    }
}

#[test]
fn ext_instruction_surfaces_raw_params() {
    let params = [0xDE, 0xAD, 0xBE, 0xEF];
    let wire = round_trip(|e| e.ext(Id::new(0x0A), 0xE0, &params).unwrap());
    match decode(&wire).1 {
        Decoded::Ext {
            id,
            instruction,
            params: got,
        } => {
            assert_eq!(id, Id::new(0x0A));
            assert_eq!(instruction, 0xE0);
            assert_eq!(unstuffed(&got).as_slice(), &params);
        }
        other => panic!("expected Ext, got {other:?}"),
    }
}

#[test]
fn write_payload_with_stuffing_trigger_round_trips() {
    // Trigger fully inside data, and a trigger straddling the addr->data
    // boundary (address low byte 0xFF, high 0xFF, first data byte 0xFD).
    for (addr, data) in [
        (
            0x0010u16,
            &[0xFF, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xAA][..],
        ),
        (0xFFFF, &[0xFD, 0x00, 0x01][..]),
    ] {
        let wire = round_trip(|e| e.write(Id::new(1), addr, data).unwrap());
        // The wire must actually contain inserted stuffing bytes.
        assert!(wire.len() > 8 + 2 + 2 + data.len());
        match decode(&wire).1 {
            Decoded::Write {
                address, data: got, ..
            } => {
                assert_eq!(address, addr);
                assert_eq!(unstuffed(&got).as_slice(), data);
            }
            other => panic!("expected Write, got {other:?}"),
        }
    }
}

#[test]
fn status_reply_round_trips() {
    let mut buf = Buf::new();
    let params = [0x10, 0x20, 0xFF, 0xFF, 0xFD, 0x40];
    let err = StatusError::from_byte(0x81);
    StatusEncoder::<_, Crc>::new(&mut buf)
        .read(Id::new(0x0C), err, &params)
        .unwrap();

    let (frame, n) = parse::<Crc>(&buf).expect("parse");
    assert_eq!(n, buf.len());
    assert_eq!(frame.kind, FrameKind::Status);
    let StatusReply {
        id,
        error,
        params: got,
    } = decode_status(&frame).expect("decode_status");
    assert_eq!(id, Id::new(0x0C));
    assert_eq!(error, err);
    assert_eq!(unstuffed(&got).as_slice(), &params);
}

#[test]
fn decode_instruction_rejects_a_status_frame() {
    let mut buf = Buf::new();
    StatusEncoder::<_, Crc>::new(&mut buf)
        .empty(Id::new(0x01), StatusError::OK)
        .unwrap();
    let (frame, _) = parse::<Crc>(&buf).unwrap();
    assert!(matches!(
        decode_instruction(&frame),
        Err(DecodeError::WrongKind)
    ));
}
