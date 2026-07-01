use super::*;

#[test]
fn reg_write_then_action_commits_to_live_table() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn reg_write_to_ro_address_replies_access_error_immediately() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), 0, &[0xAA, 0xBB]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
}

#[test]
fn reg_write_invalid_value_rejected_at_stage_time() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[2]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn write_preserves_pending_reg_write_chain() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Stage torque_enable=1 via RegWrite; chain pending.
    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    // Direct Write to a different register lands immediately, leaves the
    // RegWrite chain intact.
    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR + 1, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    // Action commits the still-pending RegWrite.
    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn broadcast_reg_write_and_action_silent_but_commits() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(BROADCAST_ID, CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);

    let req = encode(|w| w.action(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn action_with_empty_staging_replies_ok() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
}
