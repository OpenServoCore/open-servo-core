//! FAST Sync/Bulk Read coalesced reply: build an official-format chain with
//! the existing `SlotEncoder`, then walk it with the new codec and verify
//! every cumulative checkpoint. Corruption must surface at the offending
//! block index, not at the end.

use dxl_protocol::SoftwareCrcUmts as Crc;
use dxl_protocol::frame::{FrameKind, parse};
use dxl_protocol::types::packet::{ChainStatusBlocks, ChainStatusError};
use dxl_protocol::{Id, Slot, SlotEncoder, SlotPosition, StatusError, crc16_umts_continue};
use heapless::Vec as HVec;

type Buf = HVec<u8, 256>;

struct Block {
    id: u8,
    error: StatusError,
    data: &'static [u8],
}

/// Build an official-format FAST chain (`err id data crc` per block, crc =
/// cumulative from the sync header). Successor checkpoints are computed off
/// the wire bytes emitted so far.
fn build_chain(blocks: &[Block]) -> Buf {
    let n = blocks.len();
    let data_bytes: usize = blocks.iter().map(|b| b.data.len()).sum();
    // INST(1) + n*(err+id+crc = 4) + Σ data.
    let packet_length = (1 + n * 4 + data_bytes) as u16;

    let mut buf = Buf::new();
    {
        let first = &blocks[0];
        SlotEncoder::<_, Crc>::new(&mut buf)
            .emit(
                &Slot {
                    id: Id::new(first.id),
                    error: first.error,
                    data: first.data,
                },
                SlotPosition::First { packet_length },
            )
            .unwrap();
    }
    for b in &blocks[1..] {
        let mut crc = crc16_umts_continue(0, &buf);
        crc = crc16_umts_continue(crc, &[b.error.as_byte(), b.id]);
        crc = crc16_umts_continue(crc, b.data);
        SlotEncoder::<_, Crc>::new(&mut buf)
            .emit(
                &Slot {
                    id: Id::new(b.id),
                    error: b.error,
                    data: b.data,
                },
                SlotPosition::Successor { crc },
            )
            .unwrap();
    }
    buf
}

const EMANUAL: [Block; 3] = [
    Block {
        id: 3,
        error: StatusError::OK,
        data: &[0xA6, 0x00, 0x00, 0x00],
    },
    Block {
        id: 7,
        error: StatusError::OK,
        data: &[0x1F, 0x08, 0x00, 0x00],
    },
    Block {
        id: 4,
        error: StatusError::OK,
        data: &[0xFF, 0x03, 0x00, 0x00],
    },
];

#[test]
fn emanual_fast_sync_read_vector_has_length_25_and_walks_clean() {
    let wire = build_chain(&EMANUAL);

    // The e-manual example: 3 devices x 4-byte reads -> LEN 25.
    assert_eq!(u16::from_le_bytes([wire[5], wire[6]]), 25);
    assert_eq!(
        &wire[..],
        &[
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, // header, len=25, Status
            0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, // ID3 block
            0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA, // ID7 block
            0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E, // ID4 block
        ]
    );

    let (frame, n) = parse::<Crc>(&wire).unwrap();
    assert_eq!(n, wire.len());
    assert_eq!(frame.kind, FrameKind::ChainStatus);

    let blocks: Vec<_> = ChainStatusBlocks::uniform(&frame, 4)
        .map(|r| r.expect("checkpoint"))
        .collect();
    assert_eq!(blocks.len(), 3);
    assert_eq!(blocks[0].id, Id::new(3));
    assert_eq!(blocks[0].data, &[0xA6, 0x00, 0x00, 0x00]);
    assert_eq!(blocks[1].id, Id::new(7));
    assert_eq!(blocks[1].data, &[0x1F, 0x08, 0x00, 0x00]);
    assert_eq!(blocks[2].id, Id::new(4));
    assert_eq!(blocks[2].data, &[0xFF, 0x03, 0x00, 0x00]);
}

#[test]
fn with_lengths_matches_uniform_and_carries_errors() {
    let blocks = [
        Block {
            id: 1,
            error: StatusError::OK,
            data: &[0xAA, 0xBB],
        },
        Block {
            id: 2,
            error: StatusError::from_byte(0x05),
            data: &[0xCC, 0xDD],
        },
        Block {
            id: 3,
            error: StatusError::OK,
            data: &[0xEE, 0xFF],
        },
    ];
    let wire = build_chain(&blocks);
    let (frame, _) = parse::<Crc>(&wire).unwrap();

    let lengths = [2u16, 2, 2];
    let walked: Vec<_> = ChainStatusBlocks::with_lengths(&frame, &lengths)
        .map(|r| r.expect("checkpoint"))
        .collect();
    assert_eq!(walked.len(), 3);
    assert_eq!(walked[1].id, Id::new(2));
    assert_eq!(walked[1].error, StatusError::from_byte(0x05));
    assert_eq!(walked[1].data, &[0xCC, 0xDD]);
}

#[test]
fn corrupt_mid_chain_checkpoint_fails_at_that_block() {
    let mut wire = build_chain(&EMANUAL);
    // ID7 block (index 1) checkpoint sits at wire[22..24]; flip it.
    wire[22] ^= 0xFF;

    let (frame, _) = parse::<Crc>(&wire).unwrap();
    let results: Vec<_> = ChainStatusBlocks::uniform(&frame, 4).collect();
    // Block 0 is fine; block 1 fuses the walk.
    assert!(results[0].is_ok());
    assert_eq!(results[1], Err(ChainStatusError::Checkpoint { block: 1 }));
    assert_eq!(
        results.len(),
        2,
        "walker must fuse after the bad checkpoint"
    );
}

#[test]
fn corrupt_final_block_checkpoint_fails_at_last_block() {
    let mut wire = build_chain(&EMANUAL);
    // Final block's checkpoint = the frame's trailing CRC pair.
    let last = wire.len() - 1;
    wire[last] ^= 0xFF;

    let (frame, _) = parse::<Crc>(&wire).unwrap();
    let results: Vec<_> = ChainStatusBlocks::uniform(&frame, 4).collect();
    assert!(results[0].is_ok());
    assert!(results[1].is_ok());
    assert_eq!(results[2], Err(ChainStatusError::Checkpoint { block: 2 }));
}

#[test]
fn truncated_chain_reports_truncation() {
    let wire = build_chain(&EMANUAL);
    // Drop the last block's trailing bytes so block 2 can't complete.
    let (frame, _) = parse::<Crc>(&wire).unwrap();
    // Rebuild a RawFrame whose body is short by claiming 3 uniform blocks over
    // a body missing the tail: walk with lengths that overrun the body.
    let lengths = [4u16, 4, 4, 4];
    let results: Vec<_> = ChainStatusBlocks::with_lengths(&frame, &lengths).collect();
    assert_eq!(results.len(), 4);
    assert!(results[0].is_ok() && results[1].is_ok() && results[2].is_ok());
    assert_eq!(results[3], Err(ChainStatusError::Truncated { block: 3 }));
}
