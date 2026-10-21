//! The link's instrument kinds (link sec 9): raw wire ops and edge-capture
//! drains, served off the engine's side door. A same-shaped stub without
//! `bench`, so the server carries no cfg.

use crate::engine::shape::InvalidReason;
use crate::engine::{HostBus, SubmitError};
use crate::traits::{EdgeCapture, Providers};

use super::record;
use super::server::RecordSink;

enum Op<'a> {
    Send(&'a [u8]),
    Burst(&'a [u8]),
    Pulse(u16),
    Train {
        gap_us: u16,
        breaks: u8,
        announce: &'a [u8],
    },
    Baud(u32),
    Drain(u8),
    Reset,
}

fn parse(kind: u8, body: &[u8]) -> Option<Op<'_>> {
    Some(match (kind, body) {
        (record::WIRE_SEND, bytes) => Op::Send(bytes),
        (record::WIRE_BURST, stream) => Op::Burst(stream),
        (record::WIRE_PULSE, &[u0, u1]) => Op::Pulse(u16::from_le_bytes([u0, u1])),
        (record::WIRE_TRAIN, &[g0, g1, breaks, ref announce @ ..]) => Op::Train {
            gap_us: u16::from_le_bytes([g0, g1]),
            breaks,
            announce,
        },
        (record::WIRE_BAUD, &[b0, b1, b2, b3]) => Op::Baud(u32::from_le_bytes([b0, b1, b2, b3])),
        (record::EDGE_DRAIN, &[max]) => Op::Drain(max),
        (record::CAPTURE_RESET, []) => Op::Reset,
        _ => return None,
    })
}

/// Serve one instrument request; `true` when it started a wire op, whose
/// DONE the server owes at `WireDone`.
pub(super) fn handle<P: Providers>(
    kind: u8,
    seq: u16,
    body: &[u8],
    bus: &mut HostBus<P>,
    out: &mut [u8],
    sink: &mut impl RecordSink,
) -> bool {
    let Some(op) = parse(kind, body) else {
        sink.record(record::error(out, seq, kind, record::ERR_MALFORMED));
        return false;
    };
    let res = match op {
        Op::Drain(max) => {
            let max = (max as usize).min(record::DRAIN_MAX);
            let mut falls = [0u16; record::DRAIN_MAX];
            let mut rises = [0u16; record::DRAIN_MAX];
            // Rises snapshot FIRST: an edge pair landing between the two
            // ring reads then tears as fall-now/rise-later, and the client
            // defers the surplus falls until their rises drain (the wire
            // idles high, falls lead). Falls-first would tear the other
            // way - a rise shipped a batch ahead of its fall.
            let rn = bus.edges().drain_rises(&mut rises[..max]);
            let fn_ = bus.edges().drain_falls(&mut falls[..max]);
            let overflow = bus.edges().overflow();
            let now = bus.now();
            sink.record(record::done(out, seq, kind, |r| {
                record::write_edges(r, overflow, now, &falls[..fn_], &rises[..rn])
            }));
            return false;
        }
        Op::Reset => {
            bus.edges().reset();
            sink.record(record::done(out, seq, kind, |_| 0));
            return false;
        }
        Op::Send(bytes) => bus.wire_send(bytes),
        Op::Burst(stream) => bus.wire_burst(stream),
        Op::Pulse(us) => bus.wire_pulse_low(us),
        Op::Train {
            gap_us,
            breaks,
            announce,
        } => bus.wire_train(announce, gap_us, breaks),
        Op::Baud(bps) => bus.wire_baud(bps),
    };
    let code = match res {
        Ok(()) => return true,
        Err(SubmitError::Busy) => record::ERR_BUSY,
        Err(SubmitError::Invalid(InvalidReason::TooLong)) => record::ERR_TOO_LONG,
        Err(SubmitError::Invalid(_)) => record::ERR_MALFORMED,
    };
    sink.record(record::error(out, seq, kind, code));
    false
}

#[cfg(test)]
mod tests {
    use std::vec;
    use std::vec::Vec;

    use super::super::record::*;
    use crate::testutil::WireOp;
    use crate::testutil::link::{Resp, ping, req, rig};

    fn error(seq: u16, kind: u8, code: u8) -> Resp {
        Resp::Error { seq, kind, code }
    }

    fn tick_done(seq: u16, kind: u8, tick: u32) -> Resp {
        Resp::Done {
            seq,
            kind,
            result: tick.to_le_bytes().to_vec(),
        }
    }

    #[test]
    fn wire_send_answers_the_release_tick_on_its_seq() {
        let mut r = rig();
        // Raw bytes DE AD: no shape check applies.
        r.send(&req(WIRE_SEND, 0x0BB8, &[0xDE, 0xAD]));
        assert!(r.take().is_empty(), "accepted: no reply until TC");
        assert_eq!(
            r.wire.log(),
            vec![WireOp::Claim, WireOp::Break, WireOp::Send(vec![0xDE, 0xAD])]
        );

        r.clock.advance(55);
        r.bus.on_tx_complete();
        r.pump();
        assert_eq!(r.take(), [tick_done(0x0BB8, WIRE_SEND, 55)]);

        // Retired: the next wire op is accepted.
        r.send(&req(WIRE_PULSE, 2, &[100, 0]));
        assert!(r.take().is_empty(), "no ERROR");
        r.clock.advance(100);
        r.pump();
        assert_eq!(r.take(), [tick_done(2, WIRE_PULSE, 155)]);
    }

    #[test]
    fn wire_ops_and_exchanges_answer_each_other_busy() {
        let mut r = rig();
        r.send(&ping(1, 5));
        r.send(&req(WIRE_SEND, 2, &[0x55]));
        assert_eq!(r.take(), [error(2, WIRE_SEND, ERR_BUSY)]);

        let mut r = rig();
        r.send(&req(WIRE_SEND, 2, &[0x55]));
        r.send(&ping(1, 5));
        r.send(&req(WIRE_BAUD, 3, &[0, 0]));
        assert_eq!(
            r.take(),
            [
                error(1, OSC_EXCHANGE, ERR_BUSY),
                error(3, WIRE_BAUD, ERR_MALFORMED)
            ],
            "an unparsable body answers MALFORMED ahead of BUSY"
        );
    }

    #[test]
    fn edge_capture_answers_while_the_bus_is_busy() {
        let mut r = rig();
        r.send(&ping(1, 5));
        r.send(&req(EDGE_DRAIN, 2, &[64]));
        r.send(&req(CAPTURE_RESET, 3, &[]));
        let got = r.take();
        assert!(
            matches!(
                got[..],
                [
                    Resp::Done {
                        seq: 2,
                        kind: EDGE_DRAIN,
                        ..
                    },
                    Resp::Done {
                        seq: 3,
                        kind: CAPTURE_RESET,
                        ..
                    }
                ]
            ),
            "{got:?}"
        );
    }

    #[test]
    fn malformed_wire_ops_answer_on_their_seq() {
        let cases: [(u8, &[u8]); 9] = [
            // Empty send: a bare break never raises TC.
            (WIRE_SEND, &[]),
            // A burst stream with a truncated frame.
            (WIRE_BURST, &[5, 0xAA]),
            (WIRE_PULSE, &[10]),
            (WIRE_PULSE, &[0, 0]),
            // No announce byte.
            (WIRE_TRAIN, &[0x90, 0x01, 4]),
            (WIRE_TRAIN, &[0x90]),
            (WIRE_BAUD, &[1, 2]),
            (EDGE_DRAIN, &[]),
            (CAPTURE_RESET, &[0]),
        ];
        for (kind, body) in cases {
            let mut r = rig();
            r.send(&req(kind, 9, body));
            assert_eq!(
                r.take(),
                [error(9, kind, ERR_MALFORMED)],
                "kind {kind:#04x} body {body:?}"
            );
            assert!(r.wire.log().is_empty(), "nothing reached the wire");
        }
    }

    fn burst_request(frame_len: usize, frames: usize) -> Vec<u8> {
        let mut body = Vec::new();
        for _ in 0..frames {
            body.push(frame_len as u8);
            body.extend(std::iter::repeat_n(0xAA, frame_len));
        }
        req(WIRE_BURST, 1, &body)
    }

    #[test]
    fn wire_burst_takes_a_full_request_and_no_more() {
        assert_eq!(WIRE_BURST_STREAM_MAX, 507);
        let mut r = rig();
        r.send(&burst_request(168, 3));
        assert!(r.take().is_empty(), "accepted: no reply until TC");
        assert_eq!(r.wire.log()[2], WireOp::Send(vec![0xAA; 168]));

        let mut r = rig();
        r.send(&burst_request(126, 4));
        assert_eq!(r.take(), [error(SEQ_NONE, KIND_NONE, ERR_FRAMING)]);
        assert!(r.wire.log().is_empty(), "nothing reached the wire");
    }

    fn edges(r: &Resp) -> (u8, u32, Vec<u16>, Vec<u16>) {
        let Resp::Done {
            kind: EDGE_DRAIN,
            result: e,
            ..
        } = r
        else {
            panic!("EDGE_DRAIN DONE, got {r:?}");
        };
        let (fall_n, rise_n) = (e[5] as usize, e[6] as usize);
        assert_eq!(e.len(), 7 + 2 * (fall_n + rise_n));
        let t = |i: usize| u16::from_le_bytes([e[7 + 2 * i], e[8 + 2 * i]]);
        (
            e[0],
            u32::from_le_bytes([e[1], e[2], e[3], e[4]]),
            (0..fall_n).map(t).collect(),
            (fall_n..fall_n + rise_n).map(t).collect(),
        )
    }

    #[test]
    fn edge_drain_ships_staged_captures_and_the_drain_anchor() {
        let mut r = rig();
        r.bus.edges().stage(&[100, 300, 65500], &[110, 310]);
        r.clock.advance(0x0001_0032);
        r.send(&req(EDGE_DRAIN, 5, &[64]));
        let got = r.take();
        let (flags, now, falls, rises) = edges(&got[0]);
        assert_eq!(flags, 0, "no overflow");
        assert_eq!(now, 0x0001_0032, "now = the anchor floor");
        assert_eq!(falls, [100, 300, 65500]);
        assert_eq!(rises, [110, 310]);

        // Drained: a second drain answers empty.
        r.send(&req(EDGE_DRAIN, 6, &[64]));
        let (_, _, falls, rises) = edges(&r.take()[0]);
        assert!(falls.is_empty() && rises.is_empty());
    }

    #[test]
    fn edge_drain_clamps_and_reports_sticky_overflow() {
        let mut r = rig();
        let many: Vec<u16> = (0..100).collect();
        r.bus.edges().stage(&many, &[]);
        r.bus.edges().set_overflow();
        r.send(&req(EDGE_DRAIN, 1, &[255]));
        let (flags, _, falls, _) = edges(&r.take()[0]);
        assert_eq!(falls.len(), DRAIN_MAX, "clamped per ring");
        assert_eq!(flags, 1, "overflow flagged");

        // Reset clears captures + the flag.
        r.send(&req(CAPTURE_RESET, 2, &[]));
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 2,
                kind: CAPTURE_RESET,
                result: vec![]
            }]
        );
        assert_eq!(r.bus.edges().resets(), 1);
        r.send(&req(EDGE_DRAIN, 3, &[64]));
        let (flags, _, falls, rises) = edges(&r.take()[0]);
        assert_eq!((flags, falls.len(), rises.len()), (0, 0, 0));
    }

    #[test]
    fn wire_train_dispatches_gap_breaks_and_announce() {
        let mut r = rig();
        // Gap 400 us, 4 breaks, announce FE 05 70.
        r.send(&req(WIRE_TRAIN, 7, &[0x90, 0x01, 4, 0xFE, 0x05, 0x70]));
        assert!(
            r.take().is_empty(),
            "accepted: no reply until the train ends"
        );
        assert_eq!(
            r.wire.log(),
            vec![
                WireOp::Claim,
                WireOp::Break,
                WireOp::Send(vec![0xFE, 0x05, 0x70])
            ]
        );
        r.bus.on_tx_complete();
        for _ in 0..4 {
            r.bus.on_deadline();
        }
        r.pump();
        assert_eq!(r.take(), [tick_done(7, WIRE_TRAIN, 0)]);
    }

    #[test]
    fn wire_baud_dispatches_raw_bps_and_answers_empty() {
        let mut r = rig();
        r.send(&req(WIRE_BAUD, 9, &[0xAF, 0x26, 0x0F, 0x00]));
        r.pump();
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 9,
                kind: WIRE_BAUD,
                result: vec![]
            }]
        );
        assert_eq!(r.baud.applied_raw(), vec![0x000F_26AF]);
    }
}
