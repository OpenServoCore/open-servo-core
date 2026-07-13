use bench::osc::{build_ping, build_read};
use osc_core::regions::config::addr::identity::MODEL_NUMBER;
use serial_test::serial;

use crate::support::bench;

/// READ the read-only model_number span and cross-check it against PING, which
/// also reports the model -- the two must agree (no hardcoded model value).
#[test]
#[serial]
fn read_model_number_matches_ping() {
    let mut b = bench();
    let id = b.id();

    let read = b.status_ok(&build_read(id, MODEL_NUMBER, 2)).payload;
    assert_eq!(read.len(), 2, "model_number is 2 bytes");

    let ping = b.status_ok(&build_ping(id)).payload;
    assert_eq!(read, ping[..2], "READ model == PING model");
}
