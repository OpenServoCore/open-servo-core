use crate::support::{Setup, setup};
use osc_core::RegionStorage;
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::addr::{comms, identity},
};
use osc_core::services::dxl::limits::MAX_CONTROL_RW;
use osc_integration::sim::{DEFAULT_FIRMWARE_VERSION, DeviceId, Host, Servo, Sim, SimTime};

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;
const OVER_MAX_CONTROL_RW: u16 = MAX_CONTROL_RW as u16 + 1;

fn servo_id(sim: &Sim, servo: DeviceId) -> u8 {
    sim.device::<Servo>(servo)
        .shared()
        .table
        .config
        .with(|c| c.comms.id)
}

#[test_log::test]
fn sync_write_mutates_all_targets_silently() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    // body = [id=1, new=10, id=2, new=20, id=3, new=30]; length=1 (1 byte/servo).
    let body = [1u8, 10, 2, 20, 3, 30];

    sim.device_mut::<Host>(host)
        .send_sync_write(comms::ID, 1, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    for (i, &expected) in [10u8, 20, 30].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), expected, "servo[{}]", i);
    }
}

#[test_log::test]
fn sync_write_single_target_mutates_only_that_servo() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    // Body targets only id=2; servos 1 and 3 receive nothing.
    let body = [2u8, 20];

    sim.device_mut::<Host>(host)
        .send_sync_write(comms::ID, 1, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    assert_eq!(servo_id(&sim, servos[0]), 1, "servo[0] unchanged");
    assert_eq!(servo_id(&sim, servos[1]), 20, "servo[1] mutated");
    assert_eq!(servo_id(&sim, servos[2]), 3, "servo[2] unchanged");
}

/// Per `handle_sync_write`: `len == 0` rewinds staged and returns without
/// writing. No reply either way.
#[test_log::test]
fn sync_write_zero_length_does_not_mutate() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    // length=0 → per-servo chunk is just the id with no data bytes.
    let body = [1u8, 2, 3];

    sim.device_mut::<Host>(host)
        .send_sync_write(comms::ID, 0, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    for (i, expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), *expected, "servo[{}]", i);
    }
}

/// `length > MAX_CONTROL_RW` trips the dispatcher's cap check before any
/// table write. 0x55 payload keeps the RX edge density high enough for the
/// 1 Mbaud parser to commit the full frame before the cap rejection fires.
#[test_log::test]
fn sync_write_length_over_cap_does_not_mutate() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    let mut body: Vec<u8> = Vec::new();
    for id in 1u8..=3 {
        body.push(id);
        body.extend(std::iter::repeat_n(0x55u8, OVER_MAX_CONTROL_RW as usize));
    }

    sim.device_mut::<Host>(host)
        .send_sync_write(comms::ID, OVER_MAX_CONTROL_RW, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    for (i, expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), *expected, "servo[{}]", i);
    }
}

/// `identity::FIRMWARE_VERSION` is RO; `write_bytes` returns Err → the
/// dispatcher's `is_ok()` branch is skipped, no hooks dispatched, no
/// mutation. Silent like every sync_write path.
#[test_log::test]
fn sync_write_to_ro_field_does_not_mutate() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    let body = [1u8, 0x42, 2, 0x42, 3, 0x42];

    sim.device_mut::<Host>(host)
        .send_sync_write(identity::FIRMWARE_VERSION, 1, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    for (i, servo) in servos.iter().enumerate() {
        let fw = sim
            .device::<Servo>(*servo)
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
#[test_log::test]
fn sync_write_under_torque_lock_does_not_mutate() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);
    for servo in &servos {
        sim.device::<Servo>(*servo).set_torque_enabled(true);
    }

    let body = [1u8, 10, 2, 20, 3, 30];

    sim.device_mut::<Host>(host)
        .send_sync_write(comms::ID, 1, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    for (i, &expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), expected, "servo[{}]", i);
    }
}

/// Write straddling the config region end → `write_bytes` returns Err
/// (DataRange) → no mutation. Silent.
#[test_log::test]
fn sync_write_across_region_boundary_does_not_mutate() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    // 4-byte body per servo straddling the last byte of CONFIG.
    let mut body: Vec<u8> = Vec::new();
    for id in 1u8..=3 {
        body.push(id);
        body.extend_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
    }

    sim.device_mut::<Host>(host)
        .send_sync_write(CONFIG_REGION_END_ADDR - 2, 4, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    for (i, expected) in [1u8, 2, 3].iter().enumerate() {
        assert_eq!(servo_id(&sim, servos[i]), *expected, "servo[{}]", i);
    }
}

/// Body has chunks for ids [1, 99, 3]; servo 99 doesn't exist on the bus.
/// Servos 1 and 3 still process their chunks — proves the parser walks
/// every chunk in body order and a non-existent id mid-body doesn't drop
/// the surrounding ones.
#[test_log::test]
fn sync_write_unknown_id_in_body_skips_that_chunk() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    let body = [1u8, 10, 99, 99, 3, 30];

    sim.device_mut::<Host>(host)
        .send_sync_write(comms::ID, 1, &body);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "sync_write must be silent, got {:?}", rx);

    assert_eq!(servo_id(&sim, servos[0]), 10, "servo[0] mutated");
    assert_eq!(
        servo_id(&sim, servos[1]),
        2,
        "servo[1] unchanged (not in body)",
    );
    assert_eq!(servo_id(&sim, servos[2]), 30, "servo[2] mutated");
}
