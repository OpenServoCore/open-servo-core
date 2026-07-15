//! The chip-agnostic link server: pipe bytes in, engine commands down,
//! engine events up, records out. Sans-io -- the chip (or a DES harness)
//! feeds `on_pipe` with whatever the carrier delivered and drains records
//! through a [`RecordSink`]; nothing here knows about USB.

use osc_protocol::wire::{BaudRate, Id, Inst};

use crate::engine::{Command, Event, HostBus, SubmitError};
use crate::traits::{Deadline, EdgeCapture, Providers};

use super::record::{self, DRAIN_MAX, SEQ_NONE};

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

/// An adapter-level action a record requested: nothing the engine serves,
/// everything the chip must. The server queues it (acking on the pipe);
/// the chip drains via [`LinkServer::take_adapter_request`] and acts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdapterRequest {
    /// Flush the pipe, disarm the resident bootloader's arm flag, reset.
    EnterBootloader,
    /// Drive the DUT power rails to this absolute state.
    SetRails { v3v3: bool, v5: bool },
}

/// Adapter-level state the server owns: the queued chip request plus the
/// rails mirror (the server is the rails' only writer after boot, so acks
/// and masked sets compose against it).
struct AdapterState {
    req: Option<AdapterRequest>,
    rails: u8,
}

pub struct LinkServer {
    rx: [u8; RX_CAP],
    rx_len: usize,
    /// The seq owning the engine's outstanding command.
    active: Option<u16>,
    /// The seq owning the outstanding instrument wire op.
    wire_active: Option<u16>,
    adapter: AdapterState,
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
            wire_active: None,
            adapter: AdapterState {
                req: None,
                rails: record::RAILS_BOOT_STATE,
            },
            out: [0; OUT_CAP],
        }
    }

    /// Drain the pending adapter-level request, if any. The chip polls this
    /// after `on_pipe`; the matching ack record is already in the sink.
    pub fn take_adapter_request(&mut self) -> Option<AdapterRequest> {
        self.adapter.req.take()
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
                Some(Event::WireDone { tick }) => {
                    let seq = self.wire_active.take().unwrap_or(SEQ_NONE);
                    sink.record(record::wire_done(&mut self.out, seq, tick));
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
                &mut self.wire_active,
                &mut self.adapter,
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
    wire_active: &mut Option<u16>,
    adapter: &mut AdapterState,
    rec: &[u8],
    bus: &mut HostBus<P>,
    out: &mut [u8],
    sink: &mut impl RecordSink,
) {
    match rec[0] {
        record::REC_HELLO => {
            sink.record(record::info(out, P::Deadline::TICKS_PER_US));
        }
        record::REC_ENTER_BOOTLOADER => {
            adapter.req = Some(AdapterRequest::EnterBootloader);
            sink.record(record::bootloader_ack(out));
        }
        record::REC_SET_RAILS if rec.len() >= 2 => {
            let mask = rec.get(2).copied().unwrap_or(0b11) & 0b11;
            adapter.rails = (adapter.rails & !mask) | (rec[1] & mask);
            if mask != 0 {
                adapter.req = Some(AdapterRequest::SetRails {
                    v3v3: adapter.rails & 1 != 0,
                    v5: adapter.rails & 2 != 0,
                });
            }
            sink.record(record::rails_ack(out, adapter.rails));
        }
        record::REC_WIRE_SEND
        | record::REC_WIRE_BURST
        | record::REC_WIRE_PULSE
        | record::REC_WIRE_TRAIN
        | record::REC_WIRE_BAUD => {
            if rec.len() < 3 {
                sink.record(record::unknown(out, rec[0]));
                return;
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
                    return;
                }
            };
            match res {
                Ok(()) => *wire_active = Some(seq),
                Err(SubmitError::Busy) => {
                    sink.record(record::rejected(out, seq, record::REASON_BUSY));
                }
                Err(SubmitError::Invalid(_)) => {
                    sink.record(record::rejected(out, seq, record::REASON_MALFORMED));
                }
            }
        }
        record::REC_EDGE_DRAIN if rec.len() >= 2 => {
            let max = (rec[1] as usize).min(DRAIN_MAX);
            let mut falls = [0u16; DRAIN_MAX];
            let mut rises = [0u16; DRAIN_MAX];
            let fn_ = bus.edges().drain_falls(&mut falls[..max]);
            let rn = bus.edges().drain_rises(&mut rises[..max]);
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
        FakeBaud, FakeDeadline, FakeEdges, FakeRing, FakeWire, TestProviders, WireOp, sealed_status,
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
        baud: FakeBaud,
        edges: FakeEdges,
        sink: Sink,
    }

    fn rig() -> Rig {
        let ring = FakeRing::new();
        let clock = FakeDeadline::new();
        let wire = FakeWire::default();
        let baud = FakeBaud::default();
        let edges = FakeEdges::default();
        let bus = HostBus::new(
            ring.clone(),
            clock.clone(),
            wire.clone(),
            baud.clone(),
            edges.clone(),
            BaudRate::B1000000,
        );
        Rig {
            server: LinkServer::new(),
            bus,
            ring,
            clock,
            wire,
            baud,
            edges,
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
    fn set_rails_acks_and_queues_absolute_state() {
        let mut r = rig();
        r.server
            .on_pipe(&rec(&[REC_SET_RAILS, 0b01]), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0, vec![vec![2, 0, REC_RAILS_ACK, 0b01]]);
        assert_eq!(
            r.server.take_adapter_request(),
            Some(AdapterRequest::SetRails {
                v3v3: true,
                v5: false
            })
        );

        // Absolute state + single slot: back-to-back sets last-win.
        r.server
            .on_pipe(&rec(&[REC_SET_RAILS, 0b10]), &mut r.bus, &mut r.sink);
        r.server
            .on_pipe(&rec(&[REC_SET_RAILS, 0b11]), &mut r.bus, &mut r.sink);
        assert_eq!(
            r.server.take_adapter_request(),
            Some(AdapterRequest::SetRails {
                v3v3: true,
                v5: true
            })
        );
    }

    #[test]
    fn masked_rails_set_composes_against_the_mirror() {
        let mut r = rig();
        // 5V off, 3V3 untouched: boot mirror is both-on.
        r.server
            .on_pipe(&rec(&[REC_SET_RAILS, 0b00, 0b10]), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0, vec![vec![2, 0, REC_RAILS_ACK, 0b01]]);
        assert_eq!(
            r.server.take_adapter_request(),
            Some(AdapterRequest::SetRails {
                v3v3: true,
                v5: false
            })
        );
        // 3V3 off through the other mask: composed state goes dark.
        r.server
            .on_pipe(&rec(&[REC_SET_RAILS, 0b00, 0b01]), &mut r.bus, &mut r.sink);
        assert_eq!(*r.sink.0.last().unwrap(), vec![2, 0, REC_RAILS_ACK, 0b00]);
    }

    #[test]
    fn zero_mask_reads_the_rails_without_driving_them() {
        let mut r = rig();
        r.server
            .on_pipe(&rec(&[REC_SET_RAILS, 0b11, 0b00]), &mut r.bus, &mut r.sink);
        assert_eq!(
            r.sink.0,
            vec![vec![2, 0, REC_RAILS_ACK, record::RAILS_BOOT_STATE]]
        );
        assert_eq!(r.server.take_adapter_request(), None);
    }

    #[test]
    fn enter_bootloader_acks_and_queues_the_request() {
        let mut r = rig();
        assert_eq!(r.server.take_adapter_request(), None);

        r.server
            .on_pipe(&rec(&[REC_ENTER_BOOTLOADER]), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0, vec![vec![1, 0, REC_BOOTLOADER_ACK]]);
        assert_eq!(
            r.server.take_adapter_request(),
            Some(AdapterRequest::EnterBootloader)
        );
        assert_eq!(r.server.take_adapter_request(), None, "drained");
        assert!(r.wire.log().is_empty(), "nothing reaches the bus");
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
        r.edges.stage(&[100, 300, 65500], &[110, 310]);
        r.clock.advance(0x0001_0032);
        r.server
            .on_pipe(&rec(&[REC_EDGE_DRAIN, 64]), &mut r.bus, &mut r.sink);
        let e = r.sink.0.last().unwrap();
        assert_eq!(e[2], REC_EDGES);
        assert_eq!(e[3], 0, "no overflow");
        assert_eq!(
            u32::from_le_bytes([e[4], e[5], e[6], e[7]]),
            0x0001_0032,
            "now = the unwrap anchor"
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
        r.edges.stage(&many, &[]);
        r.edges.set_overflow();
        r.server
            .on_pipe(&rec(&[REC_EDGE_DRAIN, 255]), &mut r.bus, &mut r.sink);
        let e = r.sink.0.last().unwrap();
        assert_eq!(e[8] as usize, DRAIN_MAX, "clamped per ring");
        assert_eq!(e[3], 1, "overflow flagged");

        // Reset clears captures + the flag.
        r.server
            .on_pipe(&rec(&[REC_CAPTURE_RESET]), &mut r.bus, &mut r.sink);
        assert_eq!(r.sink.0.last().unwrap()[2], REC_CAPTURE_ACK);
        assert_eq!(r.edges.resets(), 1);
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
