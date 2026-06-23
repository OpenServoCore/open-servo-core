use crate::support::{Setup, matrix, setup_with};
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::{
        StatusReturnLevel,
        addr::{comms, identity},
    },
};
use osc_core::services::dxl::limits::MAX_CONTROL_RW;
use osc_core::{BaudRate, RegionStorage};
use osc_integration::sim::{DEFAULT_FIRMWARE_VERSION, DeviceId, Sim};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;
const OVER_MAX_CONTROL_RW: u16 = MAX_CONTROL_RW as u16 + 1;

fn servo_id(sim: &Sim, servo: DeviceId) -> u8 {
    sim.servo(servo).shared().table.config.with(|c| c.comms.id)
}

/// One per-servo chunk in a BulkWrite body: `[id, addr_le, len_le, data...]`.
fn chunk(id: u8, addr: u16, data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(5 + data.len());
    out.push(id);
    out.extend_from_slice(&addr.to_le_bytes());
    out.extend_from_slice(&(data.len() as u16).to_le_bytes());
    out.extend_from_slice(data);
    out
}

/// Like [`chunk`], but the body advertises `length` regardless of
/// `data.len()` — for tests where the cap or zero-length check fires
/// before any byte is consumed.
fn chunk_with_len(id: u8, addr: u16, length: u16, data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(5 + data.len());
    out.push(id);
    out.extend_from_slice(&addr.to_le_bytes());
    out.extend_from_slice(&length.to_le_bytes());
    out.extend_from_slice(data);
    out
}

#[apply(matrix)]
#[test_log::test]
fn bulk_write_mutates_all_targets_silently(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    // Heterogeneous (addr, length) per chunk — the bulk-specific shape.
    let mut body = Vec::new();
    body.extend(chunk(1, comms::ID, &[10]));
    body.extend(chunk(2, comms::STATUS_RETURN_LEVEL, &[1]));
    body.extend(chunk(3, comms::RETURN_DELAY_2US, &[50]));

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    assert_eq!(servo_id(&sim, servos[0]), 10, "servo[0] id mutated");
    let s1_srl = sim
        .servo(servos[1])
        .shared()
        .table
        .config
        .with(|c| c.comms.status_return_level);
    assert_eq!(
        s1_srl,
        StatusReturnLevel::Read,
        "servo[1] status_return_level mutated"
    );
    let s2_rdt = sim
        .servo(servos[2])
        .shared()
        .table
        .config
        .with(|c| c.comms.return_delay_2us);
    assert_eq!(s2_rdt, 50, "servo[2] return_delay_2us mutated");
}

#[apply(matrix)]
#[test_log::test]
fn bulk_write_single_target_mutates_only_that_servo(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    // Body has only id=2 entry; servos 1 and 3 see no matched chunk.
    let body = chunk(2, comms::ID, &[20]);

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    assert_eq!(servo_id(&sim, servos[0]), 1, "servo[0] unchanged");
    assert_eq!(servo_id(&sim, servos[1]), 20, "servo[1] mutated");
    assert_eq!(servo_id(&sim, servos[2]), 3, "servo[2] unchanged");
}

/// Per `handle_bulk_write`: `len == 0` rewinds staged and returns. No
/// reply either way. Each chunk in the body advertises length=0 → no
/// servo mutates.
#[apply(matrix)]
#[test_log::test]
fn bulk_write_zero_length_entry_does_not_mutate(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let mut body = Vec::new();
    body.extend(chunk_with_len(1, comms::ID, 0, &[]));
    body.extend(chunk_with_len(2, comms::ID, 0, &[]));
    body.extend(chunk_with_len(3, comms::ID, 0, &[]));

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    for (i, expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), *expected, "servo[{}]", i);
    }
}

/// `length > MAX_CONTROL_RW` trips the dispatcher's cap check before any
/// table write. 0x55 payload keeps the RX edge density high enough for
/// the 1 Mbaud parser to commit the full frame before the cap rejection
/// fires.
#[apply(matrix)]
#[test_log::test]
fn bulk_write_length_over_cap_does_not_mutate(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let payload = vec![0x55u8; OVER_MAX_CONTROL_RW as usize];
    let mut body = Vec::new();
    for id in 1u8..=3 {
        body.extend(chunk_with_len(id, comms::ID, OVER_MAX_CONTROL_RW, &payload));
    }

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    for (i, expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), *expected, "servo[{}]", i);
    }
}

/// `identity::FIRMWARE_VERSION` is RO; `write_bytes` returns Err → the
/// dispatcher's `is_ok()` branch is skipped, no hooks dispatched, no
/// mutation. Silent like every bulk_write path.
#[apply(matrix)]
#[test_log::test]
fn bulk_write_to_ro_field_does_not_mutate(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let mut body = Vec::new();
    for id in 1u8..=3 {
        body.extend(chunk(id, identity::FIRMWARE_VERSION, &[0x42]));
    }

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    for (i, servo) in servos.iter().enumerate() {
        let fw = sim
            .servo(*servo)
            .shared()
            .table
            .config
            .with(|c| c.identity.firmware_version);
        assert_eq!(
            fw, DEFAULT_FIRMWARE_VERSION,
            "servo[{}] firmware_version preserved",
            i,
        );
    }
}

/// `comms::ID` is torque-gated; while `torque_enable=true`, the table's
/// write-lock policy rejects the write → no mutation, no reply.
#[apply(matrix)]
#[test_log::test]
fn bulk_write_under_torque_lock_does_not_mutate(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    for servo in &servos {
        sim.servo(*servo).set_torque_enabled(true);
    }

    let mut body = Vec::new();
    body.extend(chunk(1, comms::ID, &[10]));
    body.extend(chunk(2, comms::ID, &[20]));
    body.extend(chunk(3, comms::ID, &[30]));

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    for (i, &expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), expected, "servo[{}]", i);
    }
}

/// Write straddling the config region end → `write_bytes` returns Err
/// (DataRange) → no mutation. Silent.
#[apply(matrix)]
#[test_log::test]
fn bulk_write_across_region_boundary_does_not_mutate(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let mut body = Vec::new();
    for id in 1u8..=3 {
        body.extend(chunk(
            id,
            CONFIG_REGION_END_ADDR - 2,
            &[0xAA, 0xBB, 0xCC, 0xDD],
        ));
    }

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    for (i, expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), *expected, "servo[{}]", i);
    }
}

/// Body has chunks for ids [1, 99, 3]; servo 99 doesn't exist on the bus.
/// Servos 1 and 3 still process their chunks — proves the parser walks
/// every chunk in body order and a non-existent id mid-body doesn't drop
/// the surrounding ones.
#[apply(matrix)]
#[test_log::test]
fn bulk_write_unknown_id_in_body_skips_that_chunk(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let mut body = Vec::new();
    body.extend(chunk(1, comms::ID, &[10]));
    body.extend(chunk(99, comms::ID, &[99]));
    body.extend(chunk(3, comms::ID, &[30]));

    sim.with_host(host, |h| {
        h.send_bulk_write(&body);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "bulk_write must be silent, got {:?}", rx);

    assert_eq!(servo_id(&sim, servos[0]), 10, "servo[0] mutated");
    assert_eq!(
        servo_id(&sim, servos[1]),
        2,
        "servo[1] unchanged (not in body)",
    );
    assert_eq!(servo_id(&sim, servos[2]), 30, "servo[2] mutated");
}
