use bench::osc::build_ping;
use serial_test::serial;

use crate::support::bench;

/// PING → status carries model(2) + fw(1), from the responder's id (§5).
#[test]
#[serial]
fn ping_returns_model_and_fw() {
    let mut b = bench();
    let id = b.id();
    let st = b.status_ok(&build_ping(id));
    assert_eq!(st.id, id, "status carries the responder id");
    assert_eq!(st.payload.len(), 3, "ping payload = model(2) + fw(1)");
}
