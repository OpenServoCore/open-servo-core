use bench::build_read;
use serial_test::serial;

use super::support::{bus, expect_status_ok, servo_id};

#[test]
#[serial]
fn read_model_number_returns_two_bytes() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let frame = build_read(id, 0, 2).expect("encode read");
    let reply = expect_status_ok(&mut bus, &frame).expect("read");
    assert_eq!(
        reply.data.len(),
        2,
        "expected 2 bytes for model_number, got {}",
        reply.data.len(),
    );
}
