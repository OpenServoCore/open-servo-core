use super::*;

#[test]
fn ping_to_our_id_replies() {
    let shared = Shared::new();
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
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Plain));
}

#[test]
fn ping_to_broadcast_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn ping_to_other_id_silent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(17)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}
