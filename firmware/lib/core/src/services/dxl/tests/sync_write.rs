use super::*;

#[test]
fn sync_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Slots: id=7 (foreign), id=0 (us); both write 0x01 to torque_enable.
    let body = [7, 0x00, 0, 0x01];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn sync_write_to_other_ids_only_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [7, 0x01, 17, 0x01];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));
}

#[test]
fn sync_write_preserves_pending_reg_write_chain() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Stage torque_enable=1 via RegWrite; chain pending.
    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    // SyncWrite to a different register lands immediately, leaves the
    // RegWrite chain intact.
    let body = [0, Mode::PositionPid as u8];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR + 1, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(
        shared.table.with(|t| t.control.lifecycle.mode),
        Mode::PositionPid,
    );
    assert!(!shared.table.with(|t| t.control.lifecycle.torque_enable));

    // Action commits the still-pending RegWrite.
    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(shared.table.with(|t| t.control.lifecycle.torque_enable));
}
