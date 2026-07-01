use super::*;

#[test]
fn bulk_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let addr_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let addr_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let body = [0, addr_lo, addr_hi, 1, 0, 1];
    let req = encode(|w| w.bulk_write(&body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let addr_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let addr_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let body = [17, addr_lo, addr_hi, 1, 0, 1];
    let req = encode(|w| w.bulk_write(&body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_uses_our_tuples_address_not_a_preceding_slots() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Foreign slot (id=7) targets torque_enable; our slot (id=0) targets mode.
    let other_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let other_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let mode_addr = CONTROL_BASE_ADDR + 1;
    let our_lo = (mode_addr & 0xFF) as u8;
    let our_hi = (mode_addr >> 8) as u8;
    let body = [
        7,
        other_lo,
        other_hi,
        1,
        0,
        1, //
        0,
        our_lo,
        our_hi,
        1,
        0,
        Mode::PositionPid as u8,
    ];
    let req = encode(|w| w.bulk_write(&body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
}
