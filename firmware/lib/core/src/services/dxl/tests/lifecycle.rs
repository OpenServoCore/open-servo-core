use super::*;

/// Regression: production wakes `Dxl::poll` on edge-ring HT/TC and USART
/// IDLE; a packet whose Header lands on one wake and Crc on the next must
/// still reply. Before `inflight` was lifted onto `Dxl`, the dispatcher
/// re-zeroed it per poll and the Crc-side `commit` saw `inflight = None`,
/// silently dropping the reply.
#[test]
fn instruction_split_across_two_polls_still_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(0)));
    // Split between the instruction byte and the CRC pair so Header lands
    // in poll #1 (creating `inflight`) and Crc lands in poll #2 (which must
    // see that same `inflight` to commit a reply).
    let split = req.len() - 2;

    bus.feed(&req[..split]);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0, "reply emitted before Crc seen");

    bus.feed(&req[split..]);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 1, "split-poll Crc must commit reply");
    let (id, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn unsupported_instruction_replies_instruction_error() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.factory_reset(Id::new(0), 0xFF));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Instruction).as_byte());
}
