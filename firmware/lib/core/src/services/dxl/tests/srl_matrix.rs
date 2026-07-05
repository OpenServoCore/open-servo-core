use super::*;

#[test]
fn return_level_none_silences_write_ack_but_ping_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));

    let req = encode(|w| w.ping(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 1);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_read_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn return_level_read_silences_write_ack_but_read_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 1);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_write_error_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), 0, &[0xAA, 0xBB]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn return_level_none_silences_unsupported_instruction_error() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.factory_reset(Id::new(0), 0xFF));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn return_level_none_silences_action_ack() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn return_level_none_silences_reboot_ack_but_reboot_still_fires() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn return_level_none_still_replies_to_ping() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
}

#[test]
fn return_level_read_rejects_read_past_map_end_with_data_range() {
    // The flat map is bounded at 1024 bytes: a read whose span runs past the
    // end errors (DataRange) rather than zero-filling. SRL=Read still emits
    // the error Status.
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0xFFFE, 1));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}
