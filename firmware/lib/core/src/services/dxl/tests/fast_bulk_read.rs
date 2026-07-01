use super::*;

#[test]
fn fast_bulk_read_in_our_slot_emits_slot_reply() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Slot));
}

#[test]
fn fast_bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(5, 0, 2), bre(7, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}
