use super::*;

#[test]
fn fast_sync_read_in_our_slot_emits_slot_reply() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Slot));
}

#[test]
fn fast_sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[5, 7, 9]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_sync_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_sync_read_at_unmapped_address_emits_zero_payload_slot() {
    // Per the memory-shaped read contract the slot carries `length` zero bytes
    // with error=OK, not stale buf contents.
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0xFFFE, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Slot));
    // HEADER(4) + 0xFE + LEN(2) + 0x55 + error + id + 2 data + CRC(2)
    assert_eq!(bus.reply.tx.len(), 14);
    assert_eq!(bus.reply.tx[8], 0);
    assert_eq!(bus.reply.tx[9], 0);
    assert_eq!(&bus.reply.tx[10..12], &[0, 0]);
}
