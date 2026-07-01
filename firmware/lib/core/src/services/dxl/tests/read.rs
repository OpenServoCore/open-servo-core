use super::*;

#[test]
fn read_model_number_returns_two_bytes() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn read_zero_length_rejects_with_data_range() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0, 0));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}
