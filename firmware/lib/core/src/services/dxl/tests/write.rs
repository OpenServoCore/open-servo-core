use super::*;

#[test]
fn write_to_rw_address_succeeds_and_mutates() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    let lc = shared.table.with(|t| t.control.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn write_to_ro_address_replies_access_error() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), 0, &[0xAA, 0xBB]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
    let identity = shared.table.with(|t| t.config.identity);
    assert_eq!(identity.model_number, 0);
}

#[test]
fn write_to_unmapped_address_replies_data_range_error() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), 0xFFFE, &[0x01]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(17), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    let lc = shared.table.with(|t| t.control.lifecycle);
    assert!(!lc.torque_enable);
}

#[test]
fn broadcast_write_applies_but_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(BROADCAST_ID, CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    let lc = shared.table.with(|t| t.control.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn baud_write_stages_via_reply_handle() {
    use crate::regions::config::addr::comms::BAUD_RATE_IDX;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), BAUD_RATE_IDX, &[BaudRate::B1000000 as u8]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.last_baud_staged, Some(BaudRate::B1000000));
}

#[test]
fn id_write_stages_via_reply_handle() {
    use crate::regions::config::addr::comms::ID;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), ID, &[42]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.last_id_staged, Some(42));
}

#[test]
fn rdt_write_stages_via_reply_handle_in_us() {
    use crate::regions::config::addr::comms::RETURN_DELAY_2US;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), RETURN_DELAY_2US, &[100]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.last_rdt_staged, Some(200));
}
