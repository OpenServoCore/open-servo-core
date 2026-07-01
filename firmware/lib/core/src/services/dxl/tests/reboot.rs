use super::*;

#[test]
fn reboot_to_our_id_acks_and_stages_reboot() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_broadcast_stages_reboot_without_ack() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_other_id_silent_and_no_request() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(Id::new(17)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert_eq!(bus.reply.reboot_count, 0);
    assert_eq!(bus.reply.last_reboot_mode, None);
}

#[test]
fn reboot_honors_staged_boot_mode() {
    use crate::regions::control::addr::system::BOOT_MODE;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), BOOT_MODE, &[BootMode::Bootloader as u8]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let req = encode(|w| w.reboot(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::Bootloader));
}
