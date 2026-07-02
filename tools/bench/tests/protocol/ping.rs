use bench::build_ping;
use serial_test::serial;

use super::support::{bus, expect_status_ok, servo_id};

#[test]
#[serial]
fn ping_own_id() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let frame = build_ping(id).expect("encode ping");
    let reply = expect_status_ok(&mut bus, &frame).expect("ping");
    assert_eq!(reply.id, id);
}
