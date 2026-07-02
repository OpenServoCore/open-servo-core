use bench::{build_read, build_write};
use serial_test::serial;

use super::support::{CONTROL_BASE_ADDR, bus, expect_status_ok, servo_id};

/// Round-trip torque_enable: Write=1 → Read=1 → restore. Every mutating
/// test starts by writing the reset value so an aborted prior test can't
/// leave state stuck; the guard on exit is symmetric belt-and-braces.
#[test]
#[serial]
fn write_torque_enable_round_trip() {
    let mut bus = bus();
    let id = servo_id(&bus);
    // Reset entry state.
    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[0]).unwrap())
        .expect("reset torque_enable");

    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[1]).unwrap())
        .expect("write torque_enable=1");
    let reply = expect_status_ok(&mut bus, &build_read(id, CONTROL_BASE_ADDR, 1).unwrap())
        .expect("read torque_enable");
    assert_eq!(reply.data, vec![1u8], "torque_enable did not persist");

    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[0]).unwrap())
        .expect("restore torque_enable=0");
}
