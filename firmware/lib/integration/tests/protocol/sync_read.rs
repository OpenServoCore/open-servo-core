use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::BaudRate;
use osc_core::regions::{CONFIG_REGION_SIZE, config::addr::comms};
use osc_integration::sim::{format_hex, parse_status_stream};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;

#[apply(matrix)]
#[test_log::test]
fn sync_read_replies_in_id_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!("sync_read_replies_in_id_order", format_hex(&rx));

    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(replies.len(), 3);
    for (i, reply) in replies.iter().enumerate() {
        let id = (i as u8) + 1;
        assert_eq!(
            *reply,
            Status::Read {
                id: Id::new(id),
                error: StatusError::OK,
                data: &[id],
            },
        );
    }
}

#[apply(matrix)]
#[test_log::test]
fn sync_read_single_id_replies_once(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[1]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );
}

/// Per `sync_bulk_chain_snoop_contract`: error replies are still wire frames,
/// so slot k+1 still sees its predecessor and the chain stays alive even when
/// every slot errors. Length 0 trips DataRange on every slot.
#[apply(matrix)]
#[test_log::test]
fn sync_read_zero_length_errors_keep_chain_alive(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 0, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    let expected: Vec<Status<'_>> = (1u8..=3)
        .map(|id| Status::Read {
            id: Id::new(id),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        })
        .collect();
    assert_eq!(replies, expected);
}

#[apply(matrix)]
#[test_log::test]
fn sync_read_across_region_boundary_returns_zeros_in_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_sync_read(CONFIG_REGION_END_ADDR - 2, 4, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    let expected: Vec<Status<'_>> = (1u8..=3)
        .map(|id| Status::Read {
            id: Id::new(id),
            error: StatusError::OK,
            data: &[0; 4],
        })
        .collect();
    assert_eq!(replies, expected);
}

#[apply(matrix)]
#[test_log::test]
fn sync_read_all_unknown_ids_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[99]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}
