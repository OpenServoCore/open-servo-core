//! Byte-identity differential for the fused single-pass emitters in
//! [`dxl_protocol::encode`]: every frame they produce must match the legacy
//! [`InstructionEncoder`] / [`StatusEncoder`] / [`SlotEncoder`] byte-for-byte
//! (the drivers patch CRCs at offsets derived from that layout). The file also
//! round-trips new output back through the flat codec and exercises capacity.

use dxl_protocol::encode::{
    encode_instruction, encode_slot, encode_slot_chunked, encode_status, encode_status_chunked,
};
use dxl_protocol::frame::{FrameKind, parse};
use dxl_protocol::types::packet::{
    ChainStatusBlock, ChainStatusBlocks, Instruction as Decoded, StatusReply, decode_instruction,
    decode_status,
};
use dxl_protocol::{
    BulkReadEntry, Bytes, Chunk, Id, Instruction, InstructionEncoder, Slot, SlotEncoder,
    SlotPosition, SoftwareCrcUmts as Crc, Status, StatusEncoder, StatusError,
};
use heapless::Vec as HVec;

type Buf = HVec<u8, 256>;

fn unstuffed(b: &Bytes) -> HVec<u8, 256> {
    b.iter().collect()
}

// -- instruction differential --------------------------------------------

/// Encode via legacy `f` into a heapless buffer and via `encode_instruction`
/// into an array; assert byte-identical.
fn same_instr<F>(id: Id, instruction: Instruction, params: &[&[u8]], f: F)
where
    F: FnOnce(&mut InstructionEncoder<'_, Buf, Crc>),
{
    let mut legacy = Buf::new();
    {
        let mut enc = InstructionEncoder::<_, Crc>::new(&mut legacy);
        f(&mut enc);
    }
    let mut fresh = [0u8; 256];
    let n = encode_instruction::<Crc>(&mut fresh, id, instruction.as_u8(), params).unwrap();
    assert_eq!(
        &fresh[..n],
        legacy.as_slice(),
        "instruction {instruction:?} mismatch"
    );
}

#[test]
fn instruction_matrix_is_byte_identical() {
    let addr = 0x0084u16.to_le_bytes();
    let len = 4u16.to_le_bytes();
    let data = [0xAA, 0xBB, 0xCC, 0xDD];
    let ids = [0x01, 0x02, 0x03];

    same_instr(Id::new(1), Instruction::Ping, &[], |e| {
        e.ping(Id::new(1)).unwrap()
    });
    same_instr(Id::new(2), Instruction::Read, &[&addr, &len], |e| {
        e.read(Id::new(2), 0x0084, 4).unwrap()
    });
    same_instr(Id::new(3), Instruction::Write, &[&addr, &data], |e| {
        e.write(Id::new(3), 0x0084, &data).unwrap()
    });
    same_instr(Id::new(4), Instruction::RegWrite, &[&addr, &data], |e| {
        e.reg_write(Id::new(4), 0x0084, &data).unwrap()
    });
    same_instr(Id::new(5), Instruction::Action, &[], |e| {
        e.action(Id::new(5)).unwrap()
    });
    same_instr(Id::new(6), Instruction::Reboot, &[], |e| {
        e.reboot(Id::new(6)).unwrap()
    });
    same_instr(Id::new(7), Instruction::FactoryReset, &[&[0x02]], |e| {
        e.factory_reset(Id::new(7), 0x02).unwrap()
    });
    let body = [0x01, 0x44, 0x58, 0x4C, 0x22];
    same_instr(Id::new(8), Instruction::Clear, &[&body], |e| {
        e.clear(Id::new(8), &body).unwrap()
    });
    same_instr(Id::new(9), Instruction::ControlTableBackup, &[&body], |e| {
        e.control_table_backup(Id::new(9), &body).unwrap()
    });
    same_instr(
        Id::BROADCAST,
        Instruction::SyncRead,
        &[&addr, &len, &ids],
        |e| e.sync_read(0x0084, 4, &ids).unwrap(),
    );
    let sw = [0x01, 0xAA, 0xBB, 0x02, 0xCC, 0xDD];
    same_instr(
        Id::BROADCAST,
        Instruction::SyncWrite,
        &[&addr, &len, &sw],
        |e| e.sync_write(0x0084, 4, &sw).unwrap(),
    );
    let bw = [0x01, 0x84, 0x00, 0x02, 0x00, 0xAA, 0xBB];
    same_instr(Id::BROADCAST, Instruction::BulkWrite, &[&bw], |e| {
        e.bulk_write(&bw).unwrap()
    });
    same_instr(
        Id::BROADCAST,
        Instruction::FastSyncRead,
        &[&addr, &len, &ids],
        |e| e.fast_sync_read(0x0084, 4, &ids).unwrap(),
    );
    let params = [0xDE, 0xAD, 0xBE, 0xEF];
    same_instr(Id::new(0x0A), Instruction::Ext(0xE0), &[&params], |e| {
        e.ext(Id::new(0x0A), 0xE0, &params).unwrap()
    });
}

#[test]
fn bulk_read_entries_are_byte_identical() {
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
    // Legacy pushes each entry's 5 bytes through the shared stuffing window;
    // a flattened slice yields the identical wire stream.
    let mut flat: HVec<u8, 32> = HVec::new();
    for e in &entries {
        flat.push(e.id.as_byte()).unwrap();
        flat.extend_from_slice(&e.address.to_le_bytes()).unwrap();
        flat.extend_from_slice(&e.length.to_le_bytes()).unwrap();
    }

    for instr in [Instruction::BulkRead, Instruction::FastBulkRead] {
        let mut legacy = Buf::new();
        {
            let mut enc = InstructionEncoder::<_, Crc>::new(&mut legacy);
            match instr {
                Instruction::BulkRead => enc.bulk_read(&entries).unwrap(),
                _ => enc.fast_bulk_read(&entries).unwrap(),
            }
        }
        let mut fresh = [0u8; 256];
        let n =
            encode_instruction::<Crc>(&mut fresh, Id::BROADCAST, instr.as_u8(), &[&flat]).unwrap();
        assert_eq!(&fresh[..n], legacy.as_slice(), "{instr:?}");
    }
}

#[test]
fn stuffing_triggers_match_legacy() {
    // (a) trigger inside data, (b) straddle addr->data (addr 0xFFFF, data FD..),
    // (c) back-to-back triggers.
    let cases: &[(u16, &[u8])] = &[
        (0x0010, &[0xFF, 0xFF, 0xFD, 0x42, 0xAA]),
        (0xFFFF, &[0xFD, 0x00, 0x01]),
        (0x0010, &[0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD]),
    ];
    for &(addr, data) in cases {
        let mut legacy = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut legacy)
            .write(Id::new(1), addr, data)
            .unwrap();
        let a = addr.to_le_bytes();
        let mut fresh = [0u8; 256];
        let n = encode_instruction::<Crc>(
            &mut fresh,
            Id::new(1),
            Instruction::Write.as_u8(),
            &[&a, data],
        )
        .unwrap();
        assert_eq!(
            &fresh[..n],
            legacy.as_slice(),
            "addr={addr:#06x} data={data:?}"
        );
        // Stuffing must have actually expanded the frame.
        assert!(n > 8 + 2 + data.len() + 2);
    }
}

// -- status differential -------------------------------------------------

fn legacy_status<F>(f: F) -> Buf
where
    F: FnOnce(&mut StatusEncoder<'_, Buf, Crc>),
{
    let mut buf = Buf::new();
    {
        let mut enc = StatusEncoder::<_, Crc>::new(&mut buf);
        f(&mut enc);
    }
    buf
}

#[test]
fn status_frames_are_byte_identical() {
    let id = Id::new(0x07);
    let err = StatusError::from_byte(0x81);

    // Empty.
    let legacy = legacy_status(|e| e.empty(id, err).unwrap());
    let mut fresh = [0u8; 64];
    let n = encode_status::<Crc>(&mut fresh, id, err, &[]).unwrap();
    assert_eq!(&fresh[..n], legacy.as_slice());

    // Ping (model_le + fw).
    let legacy = legacy_status(|e| e.ping(id, StatusError::OK, 0x0203, 0x10).unwrap());
    let payload = [0x03, 0x02, 0x10];
    let n = encode_status::<Crc>(&mut fresh, id, StatusError::OK, &payload).unwrap();
    assert_eq!(&fresh[..n], legacy.as_slice());

    // Read with a stuffing trigger straddling error->payload and inside data.
    let data = [0x10, 0x20, 0xFF, 0xFF, 0xFD, 0x40];
    let legacy = legacy_status(|e| e.read(id, err, &data).unwrap());
    let n = encode_status::<Crc>(&mut fresh, id, err, &data).unwrap();
    assert_eq!(&fresh[..n], legacy.as_slice());
}

#[test]
fn status_chunked_matches_slice_and_legacy() {
    let id = Id::new(0x09);
    let payload = [0x10, 0x00, 0x00, 0x40, 0xFF, 0xFF, 0xFD];

    let legacy = legacy_status(|e| e.read(id, StatusError::OK, &payload).unwrap());
    let mut fresh = [0u8; 64];
    let n = encode_status_chunked::<Crc, _>(
        &mut fresh,
        id,
        StatusError::OK,
        [
            Chunk::Slice(&payload[..1]),
            Chunk::Zero(2),
            Chunk::Slice(&payload[3..]),
        ],
    )
    .unwrap();
    assert_eq!(&fresh[..n], legacy.as_slice());
}

// -- FAST slot differential ----------------------------------------------

#[test]
fn slot_chain_is_byte_identical() {
    for data_len in [1usize, 2, 4, 7] {
        let d0: HVec<u8, 8> = (0..data_len as u8).map(|b| b ^ 0x5A).collect();
        let d1: HVec<u8, 8> = (0..data_len as u8).map(|b| b ^ 0xA5).collect();
        let s0 = Slot {
            id: Id::new(3),
            error: StatusError::OK,
            data: &d0,
        };
        let s1 = Slot {
            id: Id::new(7),
            error: StatusError::from_byte(0x05),
            data: &d1,
        };
        let packet_length = (1 + 2 * (4 + data_len)) as u16;
        let resumed_crc = 0xCA16u16;

        let mut legacy = Buf::new();
        {
            let mut w = SlotEncoder::<_, Crc>::new(&mut legacy);
            w.emit(&s0, SlotPosition::First { packet_length }).unwrap();
            w.emit(&s1, SlotPosition::Successor { crc: resumed_crc })
                .unwrap();
        }

        let mut fresh = [0u8; 256];
        let a = encode_slot::<Crc>(&mut fresh, &s0, SlotPosition::First { packet_length }).unwrap();
        let b = encode_slot::<Crc>(
            &mut fresh[a..],
            &s1,
            SlotPosition::Successor { crc: resumed_crc },
        )
        .unwrap();
        assert_eq!(&fresh[..a + b], legacy.as_slice(), "data_len={data_len}");
    }
}

#[test]
fn slot_chunked_matches_slice_path() {
    let id = Id::new(0x0A);
    let error = StatusError::from_byte(0x07);
    let data = [0x11, 0x00, 0x00, 0x44, 0x55];
    let slot = Slot {
        id,
        error,
        data: &data,
    };
    for position in [
        SlotPosition::First { packet_length: 13 },
        SlotPosition::Successor { crc: 0x1234 },
    ] {
        let mut by_slice = [0u8; 64];
        let a = encode_slot::<Crc>(&mut by_slice, &slot, position).unwrap();

        let mut by_chunks = [0u8; 64];
        let b = encode_slot_chunked::<Crc, _>(
            &mut by_chunks,
            id,
            error,
            position,
            [
                Chunk::Slice(&data[..1]),
                Chunk::Zero(2),
                Chunk::Slice(&data[3..]),
            ],
        )
        .unwrap();
        assert_eq!(&by_slice[..a], &by_chunks[..b], "position={position:?}");
    }
}

// -- round-trip through the flat codec -----------------------------------

#[test]
fn instruction_round_trips() {
    let data = [0xAA, 0xBB, 0xCC, 0xDD];
    let a = 0x0074u16.to_le_bytes();
    let mut buf = [0u8; 64];
    let n = encode_instruction::<Crc>(
        &mut buf,
        Id::new(3),
        Instruction::Write.as_u8(),
        &[&a, &data],
    )
    .unwrap();
    let (frame, consumed) = parse::<Crc>(&buf[..n]).unwrap();
    assert_eq!(consumed, n);
    match decode_instruction(&frame).unwrap() {
        Decoded::Write {
            id,
            address,
            data: got,
        } => {
            assert_eq!(id, Id::new(3));
            assert_eq!(address, 0x0074);
            assert_eq!(unstuffed(&got).as_slice(), &data);
        }
        other => panic!("expected Write, got {other:?}"),
    }
}

#[test]
fn status_round_trips() {
    let id = Id::new(0x0C);
    let err = StatusError::from_byte(0x81);
    let params = [0x10, 0x20, 0xFF, 0xFF, 0xFD, 0x40];
    let mut buf = [0u8; 64];
    let n = encode_status::<Crc>(&mut buf, id, err, &params).unwrap();
    let (frame, _) = parse::<Crc>(&buf[..n]).unwrap();
    assert_eq!(frame.kind, FrameKind::Status);
    let StatusReply {
        id: gid,
        error,
        params: got,
    } = decode_status(&frame).unwrap();
    assert_eq!(gid, id);
    assert_eq!(error, err);
    assert_eq!(unstuffed(&got).as_slice(), &params);
}

#[test]
fn status_enum_payloads_round_trip() {
    // The later wiring chunk lowers `Status` to `(id, error, payload)`; prove
    // the representative payloads round-trip through the new encoder.
    let id = Id::new(0x11);
    let cases: [(Status<'_>, &[u8]); 2] = [
        (
            Status::Empty {
                id,
                error: StatusError::OK,
            },
            &[],
        ),
        (
            Status::Read {
                id,
                error: StatusError::OK,
                data: &[0x10, 0x20, 0x30],
            },
            &[0x10, 0x20, 0x30],
        ),
    ];
    for (status, payload) in cases {
        let (sid, serr) = match status {
            Status::Empty { id, error } => (id, error),
            Status::Read { id, error, .. } => (id, error),
            _ => unreachable!(),
        };
        let mut buf = [0u8; 64];
        let n = encode_status::<Crc>(&mut buf, sid, serr, payload).unwrap();
        let (frame, _) = parse::<Crc>(&buf[..n]).unwrap();
        let reply = decode_status(&frame).unwrap();
        assert_eq!(reply.id, sid);
        assert_eq!(unstuffed(&reply.params).as_slice(), payload);
    }
}

#[test]
fn slot_chain_round_trips_through_chain_walker() {
    let d = [[0xA6u8, 0, 0, 0], [0x1F, 0x08, 0, 0], [0xFF, 0x03, 0, 0]];
    let ids = [3u8, 7, 4];
    let packet_length = (1 + 3 * (4 + 4)) as u16;

    let mut buf = [0u8; 256];
    let mut pos = 0;
    // Slot 0 computes its own cumulative CRC; successors resume off the wire.
    pos += encode_slot::<Crc>(
        &mut buf[pos..],
        &Slot {
            id: Id::new(ids[0]),
            error: StatusError::OK,
            data: &d[0],
        },
        SlotPosition::First { packet_length },
    )
    .unwrap();
    for k in 1..3 {
        let chain_crc = dxl_protocol::crc16_umts_continue(0, &buf[..pos]);
        let chain_crc =
            dxl_protocol::crc16_umts_continue(chain_crc, &[StatusError::OK.as_byte(), ids[k]]);
        let chain_crc = dxl_protocol::crc16_umts_continue(chain_crc, &d[k]);
        pos += encode_slot::<Crc>(
            &mut buf[pos..],
            &Slot {
                id: Id::new(ids[k]),
                error: StatusError::OK,
                data: &d[k],
            },
            SlotPosition::Successor { crc: chain_crc },
        )
        .unwrap();
    }

    let (frame, _) = parse::<Crc>(&buf[..pos]).unwrap();
    assert_eq!(frame.kind, FrameKind::ChainStatus);
    let blocks: HVec<ChainStatusBlock, 4> = ChainStatusBlocks::uniform(&frame, 4)
        .map(|r| r.expect("checkpoint good"))
        .collect();
    assert_eq!(blocks.len(), 3);
    for (blk, (eid, data)) in blocks.iter().zip(ids.iter().zip(d.iter())) {
        assert_eq!(blk.id, Id::new(*eid));
        assert_eq!(blk.data, data);
    }
}

// -- capacity ------------------------------------------------------------

#[test]
fn exact_fit_succeeds_one_short_overflows() {
    let payload = [1u8, 2, 3, 4, 5, 6];
    let mut probe = [0u8; 64];
    let n = encode_status::<Crc>(&mut probe, Id::new(4), StatusError::OK, &payload).unwrap();

    let mut exact = vec![0u8; n];
    assert!(encode_status::<Crc>(&mut exact, Id::new(4), StatusError::OK, &payload).is_ok());

    let mut short = vec![0u8; n - 1];
    assert_eq!(
        encode_status::<Crc>(&mut short, Id::new(4), StatusError::OK, &payload),
        Err(dxl_protocol::WriteError::Overflow)
    );
}

#[test]
fn adversarial_all_trigger_payload_fits_and_matches_legacy() {
    // Maximal stuffing: 30 params of repeating FF FF FD => 10 inserted FDs.
    let mut payload: HVec<u8, 30> = HVec::new();
    for i in 0..30 {
        payload.push([0xFF, 0xFF, 0xFD][i % 3]).unwrap();
    }

    let mut legacy = Buf::new();
    StatusEncoder::<_, Crc>::new(&mut legacy)
        .read(Id::new(2), StatusError::OK, &payload)
        .unwrap();

    let mut fresh = [0u8; 64];
    let n = encode_status::<Crc>(&mut fresh, Id::new(2), StatusError::OK, &payload).unwrap();
    assert_eq!(&fresh[..n], legacy.as_slice());

    // A buffer sized to the emitted worst-case length fits exactly.
    let mut tight = vec![0u8; n];
    assert!(encode_status::<Crc>(&mut tight, Id::new(2), StatusError::OK, &payload).is_ok());
}
