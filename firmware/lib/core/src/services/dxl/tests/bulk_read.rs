use super::*;

#[test]
fn bulk_read_in_our_slot_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(5, 0, 2), bre(7, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(9, 0xFFFE, 4), bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_past_map_end_replies_data_range() {
    // A coordinated (plain-Status) slot read past the 1024-byte map end
    // answers with a DataRange error Status rather than zero bytes.
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0xFFFE, 1)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}
