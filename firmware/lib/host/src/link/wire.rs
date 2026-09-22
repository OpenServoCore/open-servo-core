//! The link's 0x6x instrument family: raw wire verbs and edge-capture
//! drains, served off the engine's side door. A same-shaped no-op without
//! `bench`, so the server carries no cfg.

use crate::engine::{HostBus, SubmitError};
use crate::traits::{EdgeCapture, Providers};

use super::record;
use super::server::RecordSink;

/// Serve one instrument record; `false` when `rec` is not in the family.
pub(super) fn handle<P: Providers>(
    rec: &[u8],
    bus: &mut HostBus<P>,
    active: &mut Option<u16>,
    out: &mut [u8],
    sink: &mut impl RecordSink,
) -> bool {
    match rec[0] {
        record::REC_WIRE_SEND
        | record::REC_WIRE_BURST
        | record::REC_WIRE_PULSE
        | record::REC_WIRE_TRAIN
        | record::REC_WIRE_BAUD => {
            if rec.len() < 3 {
                sink.record(record::unknown(out, rec[0]));
                return true;
            }
            let seq = u16::from_le_bytes([rec[1], rec[2]]);
            let body = &rec[3..];
            let res = match rec[0] {
                record::REC_WIRE_SEND => bus.wire_send(body),
                record::REC_WIRE_BURST => bus.wire_burst(body),
                record::REC_WIRE_PULSE if body.len() == 2 => {
                    bus.wire_pulse_low(u16::from_le_bytes([body[0], body[1]]))
                }
                record::REC_WIRE_TRAIN if body.len() >= 4 => {
                    bus.wire_train(&body[3..], u16::from_le_bytes([body[0], body[1]]), body[2])
                }
                record::REC_WIRE_BAUD if body.len() == 4 => {
                    bus.wire_baud(u32::from_le_bytes(body.try_into().unwrap()))
                }
                _ => {
                    sink.record(record::rejected(out, seq, record::REASON_MALFORMED));
                    return true;
                }
            };
            match res {
                Ok(()) => *active = Some(seq),
                Err(SubmitError::Busy) => {
                    sink.record(record::rejected(out, seq, record::REASON_BUSY));
                }
                Err(SubmitError::Invalid(_)) => {
                    sink.record(record::rejected(out, seq, record::REASON_MALFORMED));
                }
            }
        }
        record::REC_EDGE_DRAIN if rec.len() >= 2 => {
            let max = (rec[1] as usize).min(record::DRAIN_MAX);
            let mut falls = [0u16; record::DRAIN_MAX];
            let mut rises = [0u16; record::DRAIN_MAX];
            // Rises snapshot FIRST: an edge pair landing between the two
            // ring reads then tears as fall-now/rise-later, and the client
            // defers the surplus falls until their rises drain (the wire
            // idles high, falls lead). Falls-first would tear the other
            // way -- a rise shipped a batch ahead of its fall.
            let rn = bus.edges().drain_rises(&mut rises[..max]);
            let fn_ = bus.edges().drain_falls(&mut falls[..max]);
            let overflow = bus.edges().overflow();
            let now = bus.now();
            sink.record(record::edges(
                out,
                overflow,
                now,
                &falls[..fn_],
                &rises[..rn],
            ));
        }
        record::REC_CAPTURE_RESET => {
            bus.edges().reset();
            sink.record(record::capture_ack(out));
        }
        _ => return false,
    }
    true
}

#[cfg(test)]
mod tests {
    use std::vec;
    use std::vec::Vec;

    use super::super::record::*;
    use crate::testutil::WireOp;
    use crate::testutil::link::{rec, rig, submit_ping};

    #[test]
    fn wire_send_round_trips_on_its_seq() {
        let mut r = rig();
        // seq 0x0BB8, raw bytes DE AD -- no Shape validation applies.
        r.server.on_pipe(
            &rec(&[REC_WIRE_SEND, 0xB8, 0x0B, 0xDE, 0xAD]),
            &mut r.bus,
            &mut r.sink,
        );
        assert!(r.sink.0.is_empty(), "accepted: no reply until TC");
        assert_eq!(
            r.wire.log(),
            vec![WireOp::Claim, WireOp::Break, WireOp::Send(vec![0xDE, 0xAD])]
        );

        r.clock.advance(55);
        r.bus.on_tx_complete();
        r.server.pump(&mut r.bus, &mut r.sink);
        let done = r.sink.0.last().unwrap();
        assert_eq!(done[2], REC_WIRE_DONE);
        assert_eq!(u16::from_le_bytes([done[3], done[4]]), 0x0BB8);
        assert_eq!(u32::from_le_bytes([done[5], done[6], done[7], done[8]]), 55);

        // Retired: the next wire op is accepted.
        r.server.on_pipe(
            &rec(&[REC_WIRE_PULSE, 2, 0, 100, 0]),
            &mut r.bus,
            &mut r.sink,
        );
        assert_eq!(r.sink.0.len(), 1, "no rejection");
    }

    #[test]
    fn wire_ops_and_submits_reject_each_other_busy() {
        let mut r = rig();
        r.server
            .on_pipe(&submit_ping(1, 5), &mut r.bus, &mut r.sink);
        r.server
            .on_pipe(&rec(&[REC_WIRE_SEND, 2, 0, 0x55]), &mut r.bus, &mut r.sink);
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[2], REC_REJECTED);
        assert_eq!(u16::from_le_bytes([rej[3], rej[4]]), 2);
        assert_eq!(rej[5], REASON_BUSY);

        let mut r = rig();
        r.server
            .on_pipe(&rec(&[REC_WIRE_SEND, 2, 0, 0x55]), &mut r.bus, &mut r.sink);
        r.server
            .on_pipe(&submit_ping(1, 5), &mut r.bus, &mut r.sink);
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[2], REC_REJECTED);
        assert_eq!(rej[5], REASON_BUSY);
    }

    #[test]
    fn malformed_wire_ops_reject_on_their_seq() {
        let mut r = rig();
        // Empty send body: a bare break never raises TC.
        r.server
            .on_pipe(&rec(&[REC_WIRE_SEND, 7, 0]), &mut r.bus, &mut r.sink);
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[2], REC_REJECTED);
        assert_eq!(rej[5], REASON_MALFORMED);

        // Burst stream with a truncated frame.
        r.server.on_pipe(
            &rec(&[REC_WIRE_BURST, 8, 0, 5, 0xAA]),
            &mut r.bus,
            &mut r.sink,
        );
        assert_eq!(r.sink.0.last().unwrap()[5], REASON_MALFORMED);

        // Pulse body must be exactly two bytes.
        r.server
            .on_pipe(&rec(&[REC_WIRE_PULSE, 9, 0, 10]), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0.last().unwrap()[5], REASON_MALFORMED);
        assert!(r.wire.log().is_empty(), "nothing reached the wire");
    }

    #[test]
    fn edge_drain_ships_staged_captures_and_the_drain_anchor() {
        let mut r = rig();
        r.bus.edges().stage(&[100, 300, 65500], &[110, 310]);
        r.clock.advance(0x0001_0032);
        r.server
            .on_pipe(&rec(&[REC_EDGE_DRAIN, 64]), &mut r.bus, &mut r.sink);
        let e = r.sink.0.last().unwrap();
        assert_eq!(e[2], REC_EDGES);
        assert_eq!(e[3], 0, "no overflow");
        assert_eq!(
            u32::from_le_bytes([e[4], e[5], e[6], e[7]]),
            0x0001_0032,
            "now = the anchor floor"
        );
        assert_eq!(e[8], 3, "falls");
        assert_eq!(e[9], 2, "rises");
        let t = |i: usize| u16::from_le_bytes([e[10 + 2 * i], e[11 + 2 * i]]);
        assert_eq!([t(0), t(1), t(2)], [100, 300, 65500]);
        assert_eq!([t(3), t(4)], [110, 310]);

        // Drained: a second drain answers empty.
        r.server
            .on_pipe(&rec(&[REC_EDGE_DRAIN, 64]), &mut r.bus, &mut r.sink);
        let e = r.sink.0.last().unwrap();
        assert_eq!((e[8], e[9]), (0, 0));
    }

    #[test]
    fn edge_drain_clamps_and_reports_sticky_overflow() {
        let mut r = rig();
        let many: Vec<u16> = (0..100).collect();
        r.bus.edges().stage(&many, &[]);
        r.bus.edges().set_overflow();
        r.server
            .on_pipe(&rec(&[REC_EDGE_DRAIN, 255]), &mut r.bus, &mut r.sink);
        let e = r.sink.0.last().unwrap();
        assert_eq!(e[8] as usize, DRAIN_MAX, "clamped per ring");
        assert_eq!(e[3], 1, "overflow flagged");

        // Reset clears captures + the flag.
        r.server
            .on_pipe(&rec(&[REC_CAPTURE_RESET]), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0.last().unwrap()[2], REC_CAPTURE_ACK);
        assert_eq!(r.bus.edges().resets(), 1);
        r.server
            .on_pipe(&rec(&[REC_EDGE_DRAIN, 64]), &mut r.bus, &mut r.sink);
        let e = r.sink.0.last().unwrap();
        assert_eq!((e[3], e[8], e[9]), (0, 0, 0));
    }

    #[test]
    fn wire_train_dispatches_gap_breaks_and_announce() {
        let mut r = rig();
        // seq 7, gap 400 us, 4 breaks, announce FE 05 70.
        r.server.on_pipe(
            &rec(&[REC_WIRE_TRAIN, 7, 0, 0x90, 0x01, 4, 0xFE, 0x05, 0x70]),
            &mut r.bus,
            &mut r.sink,
        );
        assert!(
            r.sink.0.is_empty(),
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
        r.server.pump(&mut r.bus, &mut r.sink);
        let done = r.sink.0.last().unwrap();
        assert_eq!(done[2], REC_WIRE_DONE);
        assert_eq!(u16::from_le_bytes([done[3], done[4]]), 7);
        // Short body (no announce byte) is malformed on its seq.
        r.server.on_pipe(
            &rec(&[REC_WIRE_TRAIN, 8, 0, 0x90, 0x01, 4]),
            &mut r.bus,
            &mut r.sink,
        );
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[2], REC_REJECTED);
        assert_eq!(rej[5], REASON_MALFORMED);
    }

    #[test]
    fn wire_baud_dispatches_raw_bps() {
        let mut r = rig();
        r.server.on_pipe(
            &rec(&[REC_WIRE_BAUD, 9, 0, 0xAF, 0x26, 0x0F, 0x00]),
            &mut r.bus,
            &mut r.sink,
        );
        r.server.pump(&mut r.bus, &mut r.sink);
        let done = r.sink.0.last().unwrap();
        assert_eq!(done[2], REC_WIRE_DONE);
        assert_eq!(u16::from_le_bytes([done[3], done[4]]), 9);
        assert_eq!(r.baud.applied_raw(), vec![0x000F_26AF]);
        // Wrong body width rejects.
        r.server
            .on_pipe(&rec(&[REC_WIRE_BAUD, 10, 0, 1, 2]), &mut r.bus, &mut r.sink);
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[2], REC_REJECTED);
        assert_eq!(rej[5], REASON_MALFORMED);
    }
}
