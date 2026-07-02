use bench::{build_action, build_read, build_reg_write, build_write};
use dxl_protocol::types::Id;
use serial_test::serial;

use super::support::{BROADCAST_ID, CONTROL_BASE_ADDR, bus, expect_status_ok, servo_id};

#[test]
#[serial]
fn reg_write_then_action_commits() {
    let mut bus = bus();
    let id = servo_id(&bus);
    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[0]).unwrap()).unwrap();

    expect_status_ok(
        &mut bus,
        &build_reg_write(id, CONTROL_BASE_ADDR, &[1]).unwrap(),
    )
    .expect("reg_write");
    let staged =
        expect_status_ok(&mut bus, &build_read(id, CONTROL_BASE_ADDR, 1).unwrap()).unwrap();
    assert_eq!(staged.data, vec![0u8], "reg_write applied early");

    let _ = bus.xfer(
        &build_action(Id::new(BROADCAST_ID)).unwrap(),
        bench::DEFAULT_IDLE_US,
    );
    let committed =
        expect_status_ok(&mut bus, &build_read(id, CONTROL_BASE_ADDR, 1).unwrap()).unwrap();
    assert_eq!(committed.data, vec![1u8], "action did not commit");

    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[0]).unwrap()).unwrap();
}

#[test]
#[serial]
fn action_with_no_pending_is_noop() {
    let mut bus = bus();
    let id = servo_id(&bus);
    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[0]).unwrap()).unwrap();

    let before =
        expect_status_ok(&mut bus, &build_read(id, CONTROL_BASE_ADDR, 1).unwrap()).unwrap();
    let _ = bus.xfer(
        &build_action(Id::new(BROADCAST_ID)).unwrap(),
        bench::DEFAULT_IDLE_US,
    );
    let after = expect_status_ok(&mut bus, &build_read(id, CONTROL_BASE_ADDR, 1).unwrap()).unwrap();
    assert_eq!(before.data, after.data, "empty Action mutated state");
}
