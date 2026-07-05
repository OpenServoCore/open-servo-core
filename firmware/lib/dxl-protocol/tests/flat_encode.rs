//! Round-trip coverage for the fused single-pass emitters in
//! [`dxl_protocol::encode`]: new output feeds back through the flat codec and
//! decodes to the inputs, and capacity checks stay panic-free.

use dxl_protocol::encode::{encode_instruction, encode_slot, encode_status};
use dxl_protocol::frame::{FrameKind, parse};
use dxl_protocol::types::packet::{
    ChainStatusBlock, ChainStatusBlocks, Instruction as Decoded, StatusReply, decode_instruction,
    decode_status,
};
use dxl_protocol::{
    Bytes, Id, Instruction, Slot, SlotPosition, SoftwareCrcUmts as Crc, Status, StatusError,
};
use heapless::Vec as HVec;

fn unstuffed(b: &Bytes) -> HVec<u8, 256> {
    b.iter().collect()
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
