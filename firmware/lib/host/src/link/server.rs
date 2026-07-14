//! The chip-agnostic link server: pipe bytes in, engine commands down,
//! engine events up, records out. Sans-io -- the chip (or a DES harness)
//! feeds `on_pipe` with whatever the carrier delivered and drains records
//! through a [`RecordSink`]; nothing here knows about USB.

use osc_protocol::wire::{BaudRate, Id, Inst};

use crate::engine::{Command, Event, HostBus, SubmitError};
use crate::traits::{Deadline, Providers};

use super::record::{self, SEQ_NONE};

/// Where outbound records go (USB IN endpoint queue, a test log, a DES
/// pipe). Called once per complete record.
pub trait RecordSink {
    fn record(&mut self, record: &[u8]);
}

/// Accumulation bound: comfortably past the largest legal record
/// (EXCHANGE with a 252 B payload = 262 pipe bytes).
const RX_CAP: usize = 512;
/// Outbound scratch: the largest record is a STATUS carrying a full reply
/// payload.
const OUT_CAP: usize = 300;

pub struct LinkServer {
    rx: [u8; RX_CAP],
    rx_len: usize,
    /// The seq owning the engine's outstanding command.
    active: Option<u16>,
    out: [u8; OUT_CAP],
}

impl Default for LinkServer {
    fn default() -> Self {
        Self::new()
    }
}

impl LinkServer {
    pub fn new() -> Self {
        Self {
            rx: [0; RX_CAP],
            rx_len: 0,
            active: None,
            out: [0; OUT_CAP],
        }
    }

    /// Feed bytes as the pipe delivered them (any framing); complete
    /// records drive the engine, replies stream to `sink`.
    pub fn on_pipe<P: Providers>(
        &mut self,
        mut bytes: &[u8],
        bus: &mut HostBus<P>,
        sink: &mut impl RecordSink,
    ) {
        while !bytes.is_empty() {
            let room = RX_CAP - self.rx_len;
            let take = room.min(bytes.len());
            self.rx[self.rx_len..self.rx_len + take].copy_from_slice(&bytes[..take]);
            self.rx_len += take;
            bytes = &bytes[take..];
            self.extract(bus, sink);
            if take == 0 {
                // A full buffer that yields no record is an over-length or
                // desynced prefix; a reliable pipe never produces one, so
                // the client is broken -- drop the buffer and say so.
                self.rx_len = 0;
                sink.record(record::unknown(&mut self.out, 0xFF));
            }
        }
    }

    /// Drain engine events into records. Call freely (idle loop, after ISR
    /// wakes, after `on_pipe`).
    pub fn pump<P: Providers>(&mut self, bus: &mut HostBus<P>, sink: &mut impl RecordSink) {
        loop {
            match bus.poll() {
                Some(Event::Status {
                    slot,
                    id,
                    inst,
                    payload,
                }) => {
                    let seq = self.active.unwrap_or(SEQ_NONE);
                    sink.record(record::status(
                        &mut self.out,
                        seq,
                        slot,
                        id.as_byte(),
                        inst.0,
                        payload.segments(),
                    ));
                }
                Some(Event::Done(t)) => {
                    let seq = self.active.take().unwrap_or(SEQ_NONE);
                    sink.record(record::terminal(&mut self.out, seq, &t));
                }
                None => return,
            }
        }
    }

    fn extract<P: Providers>(&mut self, bus: &mut HostBus<P>, sink: &mut impl RecordSink) {
        loop {
            if self.rx_len < 2 {
                return;
            }
            let len = u16::from_le_bytes([self.rx[0], self.rx[1]]) as usize;
            if len == 0 || len > RX_CAP - 2 {
                // Framing breach on a reliable pipe = client bug; there is
                // no resync point in a length-prefixed stream, so drop.
                self.rx_len = 0;
                sink.record(record::unknown(&mut self.out, 0xFF));
                return;
            }
            if self.rx_len < 2 + len {
                return;
            }
            handle(
                &mut self.active,
                &self.rx[2..2 + len],
                bus,
                &mut self.out,
                sink,
            );
            self.rx.copy_within(2 + len..self.rx_len, 0);
            self.rx_len -= 2 + len;
        }
    }
}

/// One complete record (type + body).
fn handle<P: Providers>(
    active: &mut Option<u16>,
    rec: &[u8],
    bus: &mut HostBus<P>,
    out: &mut [u8],
    sink: &mut impl RecordSink,
) {
    match rec[0] {
        record::REC_HELLO => {
            sink.record(record::info(out, P::Deadline::TICKS_PER_US));
        }
        record::REC_SUBMIT => {
            if rec.len() < 4 {
                sink.record(record::unknown(out, record::REC_SUBMIT));
                return;
            }
            let seq = u16::from_le_bytes([rec[1], rec[2]]);
            let args = &rec[4..];
            let cmd = match rec[3] {
                record::VERB_EXCHANGE if args.len() >= 2 => Command::Exchange {
                    id: Id::new(args[0]),
                    inst: Inst(args[1]),
                    payload: &args[2..],
                },
                record::VERB_RESCUE if args.is_empty() => Command::Rescue,
                record::VERB_HOST_BAUD if args.len() == 1 => match BaudRate::from_idx(args[0]) {
                    Some(rate) => Command::HostBaud(rate),
                    None => {
                        sink.record(record::rejected(out, seq, record::REASON_MALFORMED));
                        return;
                    }
                },
                record::VERB_SET_RESPONSE_DEADLINE if args.len() == 2 => {
                    Command::SetResponseDeadline {
                        us: u16::from_le_bytes([args[0], args[1]]),
                    }
                }
                _ => {
                    sink.record(record::rejected(out, seq, record::REASON_MALFORMED));
                    return;
                }
            };
            match bus.submit(cmd) {
                Ok(()) => *active = Some(seq),
                Err(SubmitError::Busy) => {
                    sink.record(record::rejected(out, seq, record::REASON_BUSY));
                }
                Err(SubmitError::Invalid(r)) => {
                    sink.record(record::rejected(out, seq, record::reason_code(r)));
                }
            }
        }
        rtype => sink.record(record::unknown(out, rtype)),
    }
}

#[cfg(test)]
mod tests {
    use std::vec;
    use std::vec::Vec;

    use osc_protocol::wire::{Opcode, ResultCode};

    use super::super::record::*;
    use super::*;
    use crate::testutil::{
        FakeBaud, FakeDeadline, FakeRing, FakeWire, TestProviders, WireOp, sealed_status,
    };

    #[derive(Default)]
    struct Sink(Vec<Vec<u8>>);

    impl RecordSink for Sink {
        fn record(&mut self, record: &[u8]) {
            self.0.push(record.to_vec());
        }
    }

    struct Rig {
        server: LinkServer,
        bus: HostBus<TestProviders>,
        ring: FakeRing,
        clock: FakeDeadline,
        wire: FakeWire,
        sink: Sink,
    }

    fn rig() -> Rig {
        let ring = FakeRing::new();
        let clock = FakeDeadline::new();
        let wire = FakeWire::default();
        let baud = FakeBaud::default();
        let bus = HostBus::new(
            ring.clone(),
            clock.clone(),
            wire.clone(),
            baud.clone(),
            BaudRate::B1000000,
        );
        Rig {
            server: LinkServer::new(),
            bus,
            ring,
            clock,
            wire,
            sink: Sink::default(),
        }
    }

    /// Length-prefix a type+body into pipe bytes.
    fn rec(body: &[u8]) -> Vec<u8> {
        let mut v = vec![body.len() as u8, (body.len() >> 8) as u8];
        v.extend_from_slice(body);
        v
    }

    fn submit_ping(seq: u16, id: u8) -> Vec<u8> {
        let inst = Inst::instruction(Opcode::Ping, 0);
        rec(&[
            REC_SUBMIT,
            seq as u8,
            (seq >> 8) as u8,
            VERB_EXCHANGE,
            id,
            inst.0,
        ])
    }

    #[test]
    fn hello_answers_info() {
        let mut r = rig();
        let bytes = rec(&[REC_HELLO]);
        r.server.on_pipe(&bytes, &mut r.bus, &mut r.sink);
        assert_eq!(
            r.sink.0,
            vec![vec![6, 0, REC_INFO, LINK_VERSION, 1, 0, 0, 0]]
        );
    }

    #[test]
    fn exchange_round_trip_tags_seq() {
        let mut r = rig();
        r.server
            .on_pipe(&submit_ping(0x1234, 5), &mut r.bus, &mut r.sink);
        assert!(r.sink.0.is_empty(), "no reply yet");
        assert_eq!(r.wire.log()[0], WireOp::Claim, "engine took the wire");

        r.bus.on_tx_complete();
        r.ring.feed(&sealed_status(5, ResultCode::Ok, &[7, 0, 1]));
        r.server.pump(&mut r.bus, &mut r.sink);

        // STATUS then TERMINAL, both carrying seq 0x1234.
        let status = &r.sink.0[0];
        assert_eq!(status[2], REC_STATUS);
        assert_eq!(u16::from_le_bytes([status[3], status[4]]), 0x1234);
        assert_eq!(status[5], 0, "slot");
        assert_eq!(status[6], 5, "responder id");
        assert_eq!(&status[8..11], &[7, 0, 1], "payload");

        let term = &r.sink.0[1];
        assert_eq!(term[2], REC_TERMINAL);
        assert_eq!(u16::from_le_bytes([term[3], term[4]]), 0x1234);
        assert_eq!(term[5], OUTCOME_COMPLETE);
        assert_eq!(term[11], 1, "statuses");

        // The seq is retired: a new submit is accepted.
        r.server
            .on_pipe(&submit_ping(0x1235, 5), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0.len(), 2, "accepted, no rejection");
    }

    #[test]
    fn busy_and_invalid_reject_with_reasons() {
        let mut r = rig();
        r.server
            .on_pipe(&submit_ping(1, 5), &mut r.bus, &mut r.sink);
        r.server
            .on_pipe(&submit_ping(2, 5), &mut r.bus, &mut r.sink);
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[2], REC_REJECTED);
        assert_eq!(u16::from_le_bytes([rej[3], rej[4]]), 2);
        assert_eq!(rej[5], REASON_BUSY);

        let mut r = rig();
        r.server
            .on_pipe(&submit_ping(9, 0), &mut r.bus, &mut r.sink);
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[5], REASON_BAD_ID);
        assert!(r.wire.log().is_empty(), "nothing reached the wire");
    }

    #[test]
    fn records_reassemble_across_pipe_chunks() {
        let mut r = rig();
        let bytes = submit_ping(7, 5);
        let (a, b) = bytes.split_at(3);
        r.server.on_pipe(a, &mut r.bus, &mut r.sink);
        assert!(r.wire.log().is_empty(), "partial record: not submitted");
        r.server.on_pipe(b, &mut r.bus, &mut r.sink);
        assert_eq!(r.wire.log()[0], WireOp::Claim);

        // Two records in one delivery both land.
        let mut r = rig();
        let mut two = rec(&[REC_HELLO]);
        two.extend_from_slice(&rec(&[REC_HELLO]));
        r.server.on_pipe(&two, &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0.len(), 2);
    }

    #[test]
    fn unknown_types_and_malformed_submits_answer() {
        let mut r = rig();
        r.server
            .on_pipe(&rec(&[0x55, 1, 2]), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0[0][2], REC_UNKNOWN);
        assert_eq!(r.sink.0[0][3], 0x55);

        // SUBMIT with an unserved verb: rejected on its seq.
        r.server
            .on_pipe(&rec(&[REC_SUBMIT, 3, 0, 0x7F]), &mut r.bus, &mut r.sink);
        let rej = r.sink.0.last().unwrap();
        assert_eq!(rej[2], REC_REJECTED);
        assert_eq!(rej[5], REASON_MALFORMED);
    }

    #[test]
    fn instant_verbs_terminate_on_their_seq() {
        let mut r = rig();
        let body = [REC_SUBMIT, 0x0A, 0x00, VERB_SET_RESPONSE_DEADLINE, 200, 0];
        r.server.on_pipe(&rec(&body), &mut r.bus, &mut r.sink);
        r.server.pump(&mut r.bus, &mut r.sink);
        let term = r.sink.0.last().unwrap();
        assert_eq!(term[2], REC_TERMINAL);
        assert_eq!(u16::from_le_bytes([term[3], term[4]]), 0x0A);
        assert_eq!(term[5], OUTCOME_COMPLETE);
    }

    #[test]
    fn timeout_terminal_names_the_slot() {
        let mut r = rig();
        r.server
            .on_pipe(&submit_ping(4, 5), &mut r.bus, &mut r.sink);
        r.bus.on_tx_complete();
        r.clock.advance(100_000);
        r.server.pump(&mut r.bus, &mut r.sink);
        let term = r.sink.0.last().unwrap();
        assert_eq!(term[5], OUTCOME_TIMEOUT);
        assert_eq!(term[6], 0, "missing slot");
    }
}
