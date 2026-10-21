//! The chip-agnostic link server: pipe bytes in, engine commands down,
//! engine events up, records out. Sans-io: the chip (or a DES harness)
//! feeds `on_pipe` with whatever the carrier delivered and drains records
//! through a [`RecordSink`]; nothing here knows about USB.

use osc_protocol::wire::{BaudRate, Id, Inst};

use crate::engine::shape::InvalidReason;
use crate::engine::{Command, Event, HostBus, SubmitError};
use crate::traits::{Deadline, Providers};

use super::record::{self as rec, ExchangeResult, HEADER, Info, KIND_NONE, SEQ_NONE};
use super::wire;

/// Where outbound records go (USB IN endpoint queue, a test log, a DES
/// pipe). Called once per complete record.
pub trait RecordSink {
    fn record(&mut self, record: &[u8]);
}

/// An adapter-level action a request asked for: nothing the engine serves,
/// everything the chip must. The server queues it (answering on the pipe);
/// the chip drains via [`LinkServer::take_adapter_request`] and acts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdapterRequest {
    /// Flush the pipe, disarm the resident bootloader's arm flag, reset.
    EnterBootloader,
    /// Drive the DUT power rails to this absolute state.
    SetRails { v3v3: bool, v5: bool },
}

/// The bus request the engine is working on: its DONE echoes the kind and
/// takes the kind's result layout.
#[derive(Clone, Copy)]
struct Active {
    seq: u16,
    kind: u8,
}

/// Adapter-level state the server owns: the queued chip request plus the
/// rails mirror (the server is the rails' only writer after boot, so masked
/// sets compose against it).
struct AdapterState {
    req: Option<AdapterRequest>,
    rails: u8,
    diag: rec::Diag,
    uptime_ticks: u64,
}

/// Everything but the inbound buffer, so a request borrowed from it can
/// drive the rest.
struct Core {
    features: u16,
    active: Option<Active>,
    adapter: AdapterState,
    out: [u8; rec::MAX_RESPONSE],
}

pub struct LinkServer {
    rx: [u8; rec::MAX_REQUEST],
    rx_len: usize,
    /// A framing breach (link sec 3.3) left the stream without a resync
    /// point: every pipe byte is ignored until the next session.
    breached: bool,
    core: Core,
}

impl LinkServer {
    /// `features`: what the chip has (`FEATURE_RAILS`, `FEATURE_ADAPTER_IAP`).
    /// The server adds `FEATURE_INSTRUMENT` when it serves the instrument
    /// kinds and drops any bit it has no handler for.
    pub fn new(features: u16) -> Self {
        let instrument = if cfg!(feature = "bench") {
            rec::FEATURE_INSTRUMENT
        } else {
            0
        };
        Self {
            rx: [0; rec::MAX_REQUEST],
            rx_len: 0,
            breached: false,
            core: Core {
                features: features & (rec::FEATURE_RAILS | rec::FEATURE_ADAPTER_IAP) | instrument,
                active: None,
                adapter: AdapterState {
                    req: None,
                    rails: rec::RAILS_BOOT_STATE,
                    diag: rec::Diag::default(),
                    uptime_ticks: 0,
                },
                out: [0; rec::MAX_RESPONSE],
            },
        }
    }

    /// Install the chip's boot diagnostics; every INFO carries them.
    pub fn set_diag(&mut self, diag: rec::Diag) {
        self.core.adapter.diag = diag;
    }

    /// The chip's time since boot on the full-width `Deadline` tick count
    /// (the engine's u32 domain wraps in minutes). Refreshed ahead of
    /// `on_pipe`; INFO reports it as `uptime_ms`.
    pub fn set_uptime_ticks(&mut self, ticks: u64) {
        self.core.adapter.uptime_ticks = ticks;
    }

    /// Drain the pending adapter-level request, if any. The chip polls this
    /// after `on_pipe`; the matching response is already in the sink.
    pub fn take_adapter_request(&mut self) -> Option<AdapterRequest> {
        self.core.adapter.req.take()
    }

    /// A new host session opened on the pipe (link sec 2): drop the
    /// half-parsed inbound record and cancel the outstanding op, so the new
    /// session finds the bus free. The carrier drops its own unsent bytes.
    /// Adapter state (rails, diag, uptime) and the engine's fleet-facing
    /// state are the bus's, not the session's, and persist.
    pub fn reset_session<P: Providers>(&mut self, bus: &mut HostBus<P>) {
        self.rx_len = 0;
        self.breached = false;
        self.core.active = None;
        bus.cancel();
    }

    /// Feed bytes as the pipe delivered them (any framing); complete
    /// requests drive the engine, responses stream to `sink`.
    pub fn on_pipe<P: Providers>(
        &mut self,
        mut bytes: &[u8],
        bus: &mut HostBus<P>,
        sink: &mut impl RecordSink,
    ) {
        while !bytes.is_empty() && !self.breached {
            // Never stalls: a full buffer always holds a whole request,
            // `extract` having refused any `len` that would not fit.
            let take = (rec::MAX_REQUEST - self.rx_len).min(bytes.len());
            self.rx[self.rx_len..self.rx_len + take].copy_from_slice(&bytes[..take]);
            self.rx_len += take;
            bytes = &bytes[take..];
            self.extract(bus, sink);
        }
    }

    /// Drain engine events into records. Call freely (idle loop, after ISR
    /// wakes, after `on_pipe`).
    pub fn pump<P: Providers>(&mut self, bus: &mut HostBus<P>, sink: &mut impl RecordSink) {
        let c = &mut self.core;
        loop {
            match bus.poll() {
                Some(Event::Status {
                    slot,
                    id,
                    inst,
                    payload,
                }) => {
                    let seq = c.active.map_or(SEQ_NONE, |a| a.seq);
                    sink.record(rec::osc_frame(
                        &mut c.out,
                        seq,
                        slot,
                        id.as_byte(),
                        inst.0,
                        payload.segments(),
                    ));
                }
                Some(Event::Done(t)) => {
                    let a = c.take_active();
                    let bps = bus.bps();
                    sink.record(rec::done(&mut c.out, a.seq, a.kind, |r| match a.kind {
                        rec::SET_BUS => {
                            r[0] = rec::MODE_OSC;
                            r[1..5].copy_from_slice(&bps.to_le_bytes());
                            5
                        }
                        rec::OSC_DEADLINE => 0,
                        _ => ExchangeResult::of(&t).write(r),
                    }));
                }
                Some(Event::WireDone { tick }) => {
                    let a = c.take_active();
                    sink.record(rec::done(&mut c.out, a.seq, a.kind, |r| {
                        if a.kind == rec::WIRE_BAUD {
                            return 0;
                        }
                        r[..4].copy_from_slice(&tick.to_le_bytes());
                        4
                    }));
                }
                None => return,
            }
        }
    }

    fn extract<P: Providers>(&mut self, bus: &mut HostBus<P>, sink: &mut impl RecordSink) {
        let mut at = 0;
        loop {
            let held = &self.rx[at..self.rx_len];
            if held.len() < 2 {
                break;
            }
            let len = u16::from_le_bytes([held[0], held[1]]) as usize;
            if len < HEADER - 2 || 2 + len > rec::MAX_REQUEST {
                self.rx_len = 0;
                self.breached = true;
                sink.record(rec::error(
                    &mut self.core.out,
                    SEQ_NONE,
                    KIND_NONE,
                    rec::ERR_FRAMING,
                ));
                return;
            }
            if held.len() < 2 + len {
                break;
            }
            let seq = u16::from_le_bytes([held[3], held[4]]);
            self.core
                .request(held[2], seq, &held[HEADER..2 + len], bus, sink);
            at += 2 + len;
        }
        self.rx.copy_within(at..self.rx_len, 0);
        self.rx_len -= at;
    }
}

/// Answered at once whether or not the bus is busy (link sec 3.1); every
/// other kind is a bus request.
fn is_immediate(kind: u8) -> bool {
    matches!(
        kind,
        rec::GET_INFO | rec::RAILS | rec::EDGE_DRAIN | rec::CAPTURE_RESET
    )
}

/// The INFO feature bit a kind needs (0 = none); `None` for an unassigned
/// kind.
fn feature_of(kind: u8) -> Option<u16> {
    Some(match kind {
        rec::GET_INFO | rec::SET_BUS | rec::OSC_EXCHANGE | rec::OSC_RESCUE | rec::OSC_DEADLINE => 0,
        rec::RAILS => rec::FEATURE_RAILS,
        rec::ADAPTER_IAP => rec::FEATURE_ADAPTER_IAP,
        rec::TB_EXCHANGE => rec::FEATURE_TINYBOOT,
        rec::WIRE_SEND..=rec::CAPTURE_RESET => rec::FEATURE_INSTRUMENT,
        _ => return None,
    })
}

pub(super) fn invalid_code(r: InvalidReason) -> u8 {
    match r {
        InvalidReason::BadId => rec::ERR_BAD_ID,
        InvalidReason::BadInst => rec::ERR_BAD_INST,
        InvalidReason::BadPayload => rec::ERR_BAD_PAYLOAD,
        InvalidReason::TooLong => rec::ERR_TOO_LONG,
    }
}

/// A non-instrument request whose body parsed.
enum Req<'a> {
    Info,
    Rails { state: u8, mask: u8 },
    AdapterIap,
    SetBus { bps: u32 },
    Engine(Command<'a>),
}

/// Parse a body exactly; `Err` carries the ERROR code.
fn parse(kind: u8, body: &[u8]) -> Result<Req<'_>, u8> {
    const RAIL_BITS: u8 = 0b11;
    let malformed = Err(rec::ERR_MALFORMED);
    match (kind, body) {
        (rec::GET_INFO, []) => Ok(Req::Info),
        (rec::RAILS, &[state, mask]) if (state | mask) & !RAIL_BITS == 0 => {
            Ok(Req::Rails { state, mask })
        }
        (rec::ADAPTER_IAP, []) => Ok(Req::AdapterIap),
        (rec::SET_BUS, &[mode, b0, b1, b2, b3]) => match mode {
            rec::MODE_OSC => Ok(Req::SetBus {
                bps: u32::from_le_bytes([b0, b1, b2, b3]),
            }),
            rec::MODE_TINYBOOT => Err(rec::ERR_UNSUPPORTED),
            _ => malformed,
        },
        (rec::OSC_EXCHANGE, &[flags, id, inst, w0, w1, w2, w3, ref payload @ ..]) => {
            let window_us = u32::from_le_bytes([w0, w1, w2, w3]);
            let (id, inst) = (Id::new(id), Inst(inst));
            match flags {
                rec::EXCHANGE_STREAM => Ok(Req::Engine(Command::ExchangeStream {
                    id,
                    inst,
                    payload,
                    window_us,
                })),
                0 if window_us == 0 => Ok(Req::Engine(Command::Exchange { id, inst, payload })),
                _ => malformed,
            }
        }
        (rec::OSC_RESCUE, []) => Ok(Req::Engine(Command::Rescue)),
        (rec::OSC_DEADLINE, &[u0, u1]) => Ok(Req::Engine(Command::SetResponseDeadline {
            us: u16::from_le_bytes([u0, u1]),
        })),
        _ => malformed,
    }
}

impl Core {
    fn take_active(&mut self) -> Active {
        self.active.take().unwrap_or(Active {
            seq: SEQ_NONE,
            kind: KIND_NONE,
        })
    }

    fn error(&mut self, seq: u16, kind: u8, code: u8, sink: &mut impl RecordSink) {
        sink.record(rec::error(&mut self.out, seq, kind, code));
    }

    /// One complete request: `body` follows the header.
    fn request<P: Providers>(
        &mut self,
        kind: u8,
        seq: u16,
        body: &[u8],
        bus: &mut HostBus<P>,
        sink: &mut impl RecordSink,
    ) {
        let Some(need) = feature_of(kind) else {
            return self.error(seq, kind, rec::ERR_UNKNOWN_KIND, sink);
        };
        if self.features & need != need {
            return self.error(seq, kind, rec::ERR_UNSUPPORTED, sink);
        }
        if need == rec::FEATURE_INSTRUMENT {
            if wire::handle(kind, seq, body, bus, &mut self.out, sink) {
                self.active = Some(Active { seq, kind });
            }
            return;
        }
        let req = match parse(kind, body) {
            Ok(req) => req,
            Err(code) => return self.error(seq, kind, code, sink),
        };
        if !is_immediate(kind) && bus.busy() {
            return self.error(seq, kind, rec::ERR_BUSY, sink);
        }
        match req {
            Req::Info => {
                let ticks_per_ms = P::Deadline::TICKS_PER_US as u64 * 1_000;
                let info = Info {
                    features: self.features,
                    ticks_per_us: P::Deadline::TICKS_PER_US,
                    max_request: rec::MAX_REQUEST as u16,
                    max_response: rec::MAX_RESPONSE as u16,
                    max_tb_payload: 0,
                    mode: rec::MODE_OSC,
                    bps: bus.bps(),
                    deadline_us: bus.response_deadline_us(),
                    diag: rec::Diag {
                        uptime_ms: (self.adapter.uptime_ticks / ticks_per_ms) as u32,
                        ..self.adapter.diag
                    },
                };
                sink.record(rec::done(&mut self.out, seq, kind, |r| info.write(r)));
            }
            Req::Rails { state, mask } => {
                let a = &mut self.adapter;
                a.rails = (a.rails & !mask) | (state & mask);
                if mask != 0 {
                    a.req = Some(AdapterRequest::SetRails {
                        v3v3: a.rails & 1 != 0,
                        v5: a.rails & 2 != 0,
                    });
                }
                let rails = a.rails;
                sink.record(rec::done(&mut self.out, seq, kind, |r| {
                    r[0] = rails;
                    1
                }));
            }
            Req::AdapterIap => {
                self.adapter.req = Some(AdapterRequest::EnterBootloader);
                sink.record(rec::done(&mut self.out, seq, kind, |_| 0));
            }
            Req::SetBus { bps } => match BaudRate::from_hz(bps) {
                Some(rate) => self.submit(
                    seq,
                    kind,
                    Command::HostBaud { bps: rate.as_hz() },
                    bus,
                    sink,
                ),
                None => self.error(seq, kind, rec::ERR_BAD_RATE, sink),
            },
            Req::Engine(cmd) => self.submit(seq, kind, cmd, bus, sink),
        }
    }

    fn submit<P: Providers>(
        &mut self,
        seq: u16,
        kind: u8,
        cmd: Command<'_>,
        bus: &mut HostBus<P>,
        sink: &mut impl RecordSink,
    ) {
        match bus.submit(cmd) {
            Ok(()) => self.active = Some(Active { seq, kind }),
            Err(SubmitError::Busy) => self.error(seq, kind, rec::ERR_BUSY, sink),
            Err(SubmitError::Invalid(r)) => self.error(seq, kind, invalid_code(r), sink),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::vec;
    use std::vec::Vec;

    use osc_protocol::wire::{Opcode, ResultCode};

    use super::super::record::*;
    use super::*;
    use crate::testutil::link::{Resp, ping, req, rig, stream_arm};
    use crate::testutil::{WireOp, sealed_status};

    /// The adapter's chip features as the rig's server holds them.
    const CHIP: u16 = FEATURE_RAILS | FEATURE_ADAPTER_IAP;

    fn claims(r: &crate::testutil::link::Rig) -> usize {
        r.wire
            .log()
            .iter()
            .filter(|op| **op == WireOp::Claim)
            .count()
    }

    #[test]
    fn get_info_reports_features_limits_and_bus_state() {
        let mut r = rig();
        r.send(&req(GET_INFO, 3, &[]));
        let [
            Resp::Done {
                seq: 3,
                kind,
                result,
            },
        ] = &r.take()[..]
        else {
            panic!("one DONE");
        };
        assert_eq!(*kind, GET_INFO);
        let info = Info::parse(result).expect("INFO result");
        let instrument = if cfg!(feature = "bench") {
            FEATURE_INSTRUMENT
        } else {
            0
        };
        assert_eq!(
            info,
            Info {
                features: CHIP | instrument,
                ticks_per_us: 1,
                max_request: MAX_REQUEST as u16,
                max_response: MAX_RESPONSE as u16,
                max_tb_payload: 0,
                mode: MODE_OSC,
                bps: 1_000_000,
                deadline_us: 1000,
                diag: Diag::default(),
            }
        );
    }

    #[test]
    fn features_the_server_cannot_serve_are_not_reported() {
        let mut r = rig();
        r.server = LinkServer::new(0xFFFF);
        r.send(&req(GET_INFO, 0, &[]));
        let [Resp::Done { result, .. }] = &r.take()[..] else {
            panic!("one DONE");
        };
        let features = Info::parse(result).unwrap().features;
        assert_eq!(
            features & !FEATURE_INSTRUMENT,
            CHIP,
            "no TINYBOOT, no bit 4+"
        );
    }

    #[test]
    fn info_carries_diag_and_live_uptime() {
        let mut r = rig();
        let diag = Diag {
            reset: RESET_IWDG,
            phase: PHASE_PUMP,
            crash_seq: 3,
            mcause: 5,
            mepc: 0x2a1c,
            mtval: 0x2000_4000,
            hse_fail: 1,
            resets: 4,
            uptime_ms: 0,
        };
        r.server.set_diag(diag);
        r.server.set_uptime_ticks(5_000_000_123);
        r.send(&req(GET_INFO, 0, &[]));
        let [Resp::Done { result, .. }] = &r.take()[..] else {
            panic!("one DONE");
        };
        let info = Info::parse(result).unwrap();
        assert_eq!(
            info.diag,
            Diag {
                uptime_ms: 5_000_000,
                ..diag
            },
            "1 tick/us"
        );
    }

    #[test]
    fn info_reports_the_bus_rate_and_deadline_it_inherits() {
        let mut r = rig();
        r.send(&req(SET_BUS, 1, &[MODE_OSC, 0x80, 0x84, 0x1E, 0]));
        r.pump();
        r.send(&req(OSC_DEADLINE, 2, &500u16.to_le_bytes()));
        r.pump();
        r.server.reset_session(&mut r.bus);
        r.take();
        r.send(&req(GET_INFO, 0, &[]));
        let [Resp::Done { result, .. }] = &r.take()[..] else {
            panic!("one DONE");
        };
        let info = Info::parse(result).unwrap();
        assert_eq!((info.bps, info.deadline_us), (2_000_000, 500));
    }

    #[test]
    fn exchange_streams_frames_then_done_on_its_seq() {
        let mut r = rig();
        r.send(&ping(0x1234, 5));
        assert!(r.take().is_empty(), "no reply yet");
        assert_eq!(r.wire.log()[0], WireOp::Claim, "engine took the wire");

        r.bus.on_tx_complete();
        r.ring.frame(&sealed_status(5, ResultCode::Ok, &[7, 0, 1]));
        r.pump();
        let got = r.take();
        let inst = Inst::status(ResultCode::Ok, false).0;
        assert_eq!(
            got[0],
            Resp::Frame {
                seq: 0x1234,
                slot: 0,
                bytes: vec![5, inst, 7, 0, 1]
            }
        );
        let Resp::Done { seq, kind, result } = &got[1] else {
            panic!("DONE");
        };
        assert_eq!((*seq, *kind), (0x1234, OSC_EXCHANGE));
        let x = ExchangeResult::parse(result).expect("exchange result");
        assert_eq!(x.outcome, crate::engine::Outcome::Complete);
        assert_eq!(x.frames, 1);

        // Retired: the next exchange is accepted.
        r.send(&ping(0x1235, 5));
        assert!(r.take().is_empty(), "accepted, no ERROR");
    }

    #[test]
    fn stream_exchange_collects_the_burst_on_its_seq() {
        let mut r = rig();
        r.send(&stream_arm(9, 100_000));
        assert_eq!(r.wire.log()[0], WireOp::Claim, "engine took the wire");

        r.bus.on_tx_complete();
        r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
        r.ring
            .frame(&sealed_status(5, ResultCode::Stream, &[0, 0, 1, 0, 8, 8]));
        r.ring
            .frame(&sealed_status(5, ResultCode::Stream, &[1, 1, 1, 0, 9, 9]));
        r.pump();

        // Ack + two burst frames on seq 9, then the DONE.
        let got = r.take();
        assert_eq!(got.len(), 4);
        for (i, f) in got[..3].iter().enumerate() {
            let Resp::Frame { seq, slot, .. } = f else {
                panic!("FRAME");
            };
            assert_eq!((*seq, *slot as usize), (9, i), "slot = arrival index");
        }
        let Resp::Done { kind, result, .. } = &got[3] else {
            panic!("DONE");
        };
        assert_eq!(*kind, OSC_EXCHANGE);
        assert_eq!(ExchangeResult::parse(result).unwrap().frames, 3);
    }

    #[test]
    fn timeout_names_the_missing_slot() {
        let mut r = rig();
        r.send(&ping(4, 5));
        r.bus.on_tx_complete();
        r.clock.advance(100_000);
        r.pump();
        let [Resp::Done { result, .. }] = &r.take()[..] else {
            panic!("one DONE");
        };
        assert_eq!(
            ExchangeResult::parse(result).unwrap().outcome,
            crate::engine::Outcome::Timeout { slot: 0 }
        );
    }

    #[test]
    fn exchange_body_checks() {
        let inst = Inst::instruction(Opcode::Ping, 0).0;
        let cases: [(&[u8], u8); 5] = [
            // Short of the fixed fields.
            (&[0, 5, inst, 0, 0, 0], ERR_MALFORMED),
            // A window without STREAM.
            (&[0, 5, inst, 1, 0, 0, 0], ERR_MALFORMED),
            // A reserved flag bit.
            (&[0x02, 5, inst, 0, 0, 0, 0], ERR_MALFORMED),
            (&[0, 0, inst, 0, 0, 0, 0], ERR_BAD_ID),
            (
                &[0, 5, Inst::status(ResultCode::Ok, false).0, 0, 0, 0, 0],
                ERR_BAD_INST,
            ),
        ];
        for (body, code) in cases {
            let mut r = rig();
            r.send(&req(OSC_EXCHANGE, 6, body));
            assert_eq!(
                r.take(),
                [Resp::Error {
                    seq: 6,
                    kind: OSC_EXCHANGE,
                    code
                }],
                "{body:?}"
            );
            assert!(r.wire.log().is_empty(), "nothing reached the wire");
        }
    }

    #[test]
    fn exchange_payload_errors_map_from_the_engine() {
        let mut r = rig();
        // A READ needs addr + count.
        let inst = Inst::instruction(Opcode::Read, 0).0;
        r.send(&req(OSC_EXCHANGE, 1, &[0, 5, inst, 0, 0, 0, 0, 1]));
        let mut body = vec![0, 5, Inst::instruction(Opcode::Write, 0).0, 0, 0, 0, 0];
        body.extend_from_slice(&[0; 300]);
        r.send(&req(OSC_EXCHANGE, 2, &body));
        let codes: Vec<u8> = r
            .take()
            .iter()
            .map(|x| match x {
                Resp::Error { code, .. } => *code,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(codes, [ERR_BAD_PAYLOAD, ERR_TOO_LONG]);
    }

    #[test]
    fn bus_requests_answer_busy_while_one_is_outstanding() {
        let mut r = rig();
        r.send(&ping(1, 5));
        r.send(&ping(2, 5));
        r.send(&req(OSC_RESCUE, 3, &[]));
        r.send(&req(OSC_DEADLINE, 4, &[0, 1]));
        r.send(&req(SET_BUS, 5, &[MODE_OSC, 0x20, 0xA1, 0x07, 0]));
        r.send(&req(ADAPTER_IAP, 6, &[]));
        let got = r.take();
        let want: Vec<Resp> = [
            (2, OSC_EXCHANGE),
            (3, OSC_RESCUE),
            (4, OSC_DEADLINE),
            (5, SET_BUS),
            (6, ADAPTER_IAP),
        ]
        .into_iter()
        .map(|(seq, kind)| Resp::Error {
            seq,
            kind,
            code: ERR_BUSY,
        })
        .collect();
        assert_eq!(got, want);
        assert_eq!(r.server.take_adapter_request(), None, "IAP not queued");
    }

    #[test]
    fn immediate_requests_answer_while_the_bus_is_busy() {
        let mut r = rig();
        r.send(&ping(1, 5));
        r.send(&req(GET_INFO, 2, &[]));
        r.send(&req(RAILS, 3, &[0, 0]));
        let got = r.take();
        assert!(
            matches!(
                &got[..],
                [
                    Resp::Done {
                        seq: 2,
                        kind: GET_INFO,
                        ..
                    },
                    Resp::Done {
                        seq: 3,
                        kind: RAILS,
                        ..
                    }
                ]
            ),
            "{got:?}"
        );
        // The exchange still closes on its own seq.
        r.bus.on_tx_complete();
        r.ring.frame(&sealed_status(5, ResultCode::Ok, &[7, 0, 1]));
        r.pump();
        let got = r.take();
        assert!(matches!(
            got.last(),
            Some(Resp::Done {
                seq: 1,
                kind: OSC_EXCHANGE,
                ..
            })
        ));
    }

    #[test]
    fn unknown_and_unsupported_kinds() {
        let mut r = rig();
        r.server = LinkServer::new(0);
        for kind in [0x03, 0x11, 0x23, 0x31, 0x67, 0x7F, 0x80, 0xFF] {
            r.send(&req(kind, 1, &[]));
            assert_eq!(
                r.take(),
                [Resp::Error {
                    seq: 1,
                    kind,
                    code: ERR_UNKNOWN_KIND
                }],
                "kind {kind:#04x}"
            );
        }
        let mut unsupported = vec![RAILS, ADAPTER_IAP, TB_EXCHANGE];
        if !cfg!(feature = "bench") {
            unsupported.extend(WIRE_SEND..=CAPTURE_RESET);
        }
        for kind in unsupported {
            // Precedence: UNSUPPORTED before the body is looked at.
            r.send(&req(kind, 2, &[0xEE; 9]));
            assert_eq!(
                r.take(),
                [Resp::Error {
                    seq: 2,
                    kind,
                    code: ERR_UNSUPPORTED
                }],
                "kind {kind:#04x}"
            );
        }
        assert!(r.wire.log().is_empty());
    }

    #[test]
    fn unknown_kind_wins_over_busy() {
        let mut r = rig();
        r.send(&ping(1, 5));
        r.send(&req(0x40, 2, &[]));
        r.send(&req(OSC_RESCUE, 3, &[1]));
        assert_eq!(
            r.take(),
            [
                Resp::Error {
                    seq: 2,
                    kind: 0x40,
                    code: ERR_UNKNOWN_KIND
                },
                Resp::Error {
                    seq: 3,
                    kind: OSC_RESCUE,
                    code: ERR_MALFORMED
                },
            ],
            "MALFORMED ahead of BUSY"
        );
    }

    #[test]
    fn malformed_bodies() {
        let cases: [(u8, &[u8]); 9] = [
            (GET_INFO, &[0]),
            (RAILS, &[1]),
            (RAILS, &[1, 1, 1]),
            (RAILS, &[0b100, 0b11]),
            (ADAPTER_IAP, &[0]),
            (SET_BUS, &[MODE_OSC, 0x20, 0xA1, 0x07]),
            (SET_BUS, &[2, 0x20, 0xA1, 0x07, 0]),
            (OSC_RESCUE, &[0]),
            (OSC_DEADLINE, &[1, 2, 3]),
        ];
        for (kind, body) in cases {
            let mut r = rig();
            r.send(&req(kind, 7, body));
            assert_eq!(
                r.take(),
                [Resp::Error {
                    seq: 7,
                    kind,
                    code: ERR_MALFORMED
                }],
                "kind {kind:#04x} body {body:?}"
            );
            assert_eq!(r.server.take_adapter_request(), None);
        }
    }

    #[test]
    fn set_bus_moves_the_rate_and_echoes_it() {
        let mut r = rig();
        let mut body = vec![MODE_OSC];
        body.extend_from_slice(&3_000_000u32.to_le_bytes());
        r.send(&req(SET_BUS, 8, &body));
        r.pump();
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 8,
                kind: SET_BUS,
                result: body
            }]
        );
        assert_eq!(r.baud.applied(), vec![BaudRate::B3000000]);
    }

    #[test]
    fn set_bus_refuses_other_rates_and_the_tinyboot_mode() {
        let mut r = rig();
        r.send(&req(SET_BUS, 1, &[MODE_OSC, 0x2F, 0x27, 0x0F, 0]));
        r.send(&req(SET_BUS, 2, &[MODE_TINYBOOT, 0x00, 0xC2, 0x01, 0]));
        assert_eq!(
            r.take(),
            [
                Resp::Error {
                    seq: 1,
                    kind: SET_BUS,
                    code: ERR_BAD_RATE
                },
                Resp::Error {
                    seq: 2,
                    kind: SET_BUS,
                    code: ERR_UNSUPPORTED
                },
            ]
        );
        assert!(r.baud.applied().is_empty());
    }

    #[test]
    fn rescue_and_deadline_close_with_their_results() {
        let mut r = rig();
        r.send(&req(OSC_DEADLINE, 0x0A, &200u16.to_le_bytes()));
        r.pump();
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 0x0A,
                kind: OSC_DEADLINE,
                result: vec![]
            }]
        );
        assert_eq!(r.bus.response_deadline_us(), 200);

        r.send(&req(OSC_RESCUE, 0x0B, &[]));
        assert_eq!(r.wire.log()[..2], [WireOp::Claim, WireOp::HoldLow]);
        r.clock.advance(1_000);
        r.pump();
        let [Resp::Done { seq, kind, result }] = &r.take()[..] else {
            panic!("one DONE");
        };
        assert_eq!((*seq, *kind), (0x0B, OSC_RESCUE));
        let x = ExchangeResult::parse(result).unwrap();
        assert_eq!((x.outcome, x.tick), (crate::engine::Outcome::Sent, 1_000));
        assert_eq!(r.bus.bps(), 500_000, "rescued to 0.5M");
    }

    #[test]
    fn rails_set_and_queue_the_absolute_state() {
        let mut r = rig();
        r.send(&req(RAILS, 1, &[0b01, 0b11]));
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 1,
                kind: RAILS,
                result: vec![0b01]
            }]
        );
        assert_eq!(
            r.server.take_adapter_request(),
            Some(AdapterRequest::SetRails {
                v3v3: true,
                v5: false
            })
        );

        // Absolute state + single slot: back-to-back sets last-win.
        r.send(&req(RAILS, 2, &[0b10, 0b11]));
        r.send(&req(RAILS, 3, &[0b11, 0b11]));
        assert_eq!(
            r.server.take_adapter_request(),
            Some(AdapterRequest::SetRails {
                v3v3: true,
                v5: true
            })
        );
    }

    #[test]
    fn masked_rails_compose_against_the_mirror() {
        let mut r = rig();
        // 5V off, 3V3 untouched: the boot mirror is both on.
        r.send(&req(RAILS, 1, &[0b00, 0b10]));
        r.send(&req(RAILS, 2, &[0b00, 0b01]));
        let states: Vec<Vec<u8>> = r
            .take()
            .into_iter()
            .map(|x| match x {
                Resp::Done { result, .. } => result,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(states, [vec![0b01], vec![0b00]]);
    }

    #[test]
    fn zero_mask_reads_the_rails_without_driving_them() {
        let mut r = rig();
        r.send(&req(RAILS, 1, &[0b11, 0b00]));
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 1,
                kind: RAILS,
                result: vec![RAILS_BOOT_STATE]
            }]
        );
        assert_eq!(r.server.take_adapter_request(), None);
    }

    #[test]
    fn adapter_iap_answers_and_queues_the_request() {
        let mut r = rig();
        r.send(&req(ADAPTER_IAP, 0x55, &[]));
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 0x55,
                kind: ADAPTER_IAP,
                result: vec![]
            }]
        );
        assert_eq!(
            r.server.take_adapter_request(),
            Some(AdapterRequest::EnterBootloader)
        );
        assert_eq!(r.server.take_adapter_request(), None, "drained");
        assert!(r.wire.log().is_empty(), "nothing reaches the bus");
    }

    #[test]
    fn records_reassemble_across_deliveries() {
        let mut r = rig();
        let bytes = ping(7, 5);
        for chunk in bytes.chunks(1) {
            assert_eq!(claims(&r), 0, "partial request: not submitted");
            r.send(chunk);
        }
        assert_eq!(claims(&r), 1);

        // Two requests in one delivery both land.
        let mut r = rig();
        let mut two = req(GET_INFO, 1, &[]);
        two.extend_from_slice(&req(RAILS, 2, &[0, 0]));
        r.send(&two);
        let seqs: Vec<u16> = r
            .take()
            .iter()
            .map(|x| match x {
                Resp::Done { seq, .. } => *seq,
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(seqs, [1, 2]);
    }

    #[test]
    fn a_request_at_the_size_limit_is_accepted() {
        let mut r = rig();
        let body = vec![0; MAX_REQUEST - HEADER];
        r.send(&req(GET_INFO, 1, &body));
        assert_eq!(
            r.take(),
            [Resp::Error {
                seq: 1,
                kind: GET_INFO,
                code: ERR_MALFORMED
            }],
            "parsed as a request, not a breach"
        );
    }

    #[test]
    fn framing_breach_answers_once_and_goes_deaf_until_a_new_session() {
        for len in [0u16, 2, (MAX_REQUEST - 1) as u16] {
            let mut r = rig();
            // A good request ahead of the breach in the same delivery lands.
            let mut bytes = req(GET_INFO, 1, &[]);
            bytes.extend_from_slice(&len.to_le_bytes());
            bytes.extend_from_slice(&ping(2, 5));
            r.send(&bytes);
            let got = r.take();
            assert!(matches!(got[0], Resp::Done { seq: 1, .. }), "{got:?}");
            assert_eq!(
                got[1..],
                [Resp::Error {
                    seq: SEQ_NONE,
                    kind: KIND_NONE,
                    code: ERR_FRAMING
                }],
                "len {len}"
            );
            // Deaf: neither the rest of the delivery nor later ones parse.
            r.send(&ping(3, 5));
            assert!(r.take().is_empty());
            assert!(r.wire.log().is_empty(), "nothing reached the wire");

            r.server.reset_session(&mut r.bus);
            r.send(&req(GET_INFO, 4, &[]));
            assert!(matches!(r.take()[..], [Resp::Done { seq: 4, .. }]));
        }
    }

    #[test]
    fn new_session_drops_the_half_received_request() {
        let mut r = rig();
        let bytes = ping(7, 5);
        r.send(&bytes[..3]);
        r.server.reset_session(&mut r.bus);
        r.send(&req(GET_INFO, 0, &[]));
        assert!(matches!(r.take()[..], [Resp::Done { kind: GET_INFO, .. }]));
        assert!(r.wire.log().is_empty(), "the fragment never submitted");
    }

    #[test]
    fn new_session_cancels_the_exchange_and_frees_the_bus() {
        let mut r = rig();
        r.send(&ping(0x1234, 5));
        r.bus.on_tx_complete();
        r.server.reset_session(&mut r.bus);
        // The old reply lands after the cancel: nobody collects it.
        r.ring.frame(&sealed_status(5, ResultCode::Ok, &[7, 0, 1]));
        r.pump();
        assert!(r.take().is_empty(), "old FRAME + DONE never recorded");

        // The bus is free at once; the new frame waits out the quiet.
        r.send(&ping(0, 5));
        assert!(r.take().is_empty(), "accepted, no ERROR");
        assert_eq!(claims(&r), 1, "paced: no frame yet");
        // Past the old reply's re-armed window.
        r.clock.advance(3_000);
        r.pump();
        assert_eq!(claims(&r), 2, "frame sent after the quiet");
        r.bus.on_tx_complete();
        r.ring.frame(&sealed_status(5, ResultCode::Ok, &[7, 0, 1]));
        r.pump();
        let got = r.take();
        assert!(
            matches!(
                got[..],
                [
                    Resp::Frame { seq: 0, .. },
                    Resp::Done {
                        seq: 0,
                        kind: OSC_EXCHANGE,
                        ..
                    }
                ]
            ),
            "{got:?}"
        );
    }

    #[test]
    fn new_session_cuts_a_frame_still_on_the_wire() {
        let mut r = rig();
        r.send(&ping(1, 5));
        r.server.reset_session(&mut r.bus);
        assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
        // The cut frame's TC lands late and changes nothing.
        r.bus.on_tx_complete();
        r.pump();
        assert!(r.take().is_empty());
        assert!(!r.bus.busy());
    }

    #[test]
    fn new_session_cancels_a_burst_mid_stream() {
        let mut r = rig();
        r.send(&stream_arm(9, 100_000));
        r.bus.on_tx_complete();
        r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
        r.pump();
        assert_eq!(r.take().len(), 1, "the arm's ack reached the old session");

        // Bus reset then SET_CONFIGURATION: two resets, one cancel.
        r.server.reset_session(&mut r.bus);
        r.server.reset_session(&mut r.bus);
        r.ring
            .frame(&sealed_status(5, ResultCode::Stream, &[0, 0, 1, 0, 8, 8]));
        r.pump();
        assert!(r.take().is_empty(), "burst frames never recorded");
        r.send(&ping(0, 5));
        assert!(r.take().is_empty(), "accepted: the bus is free");
    }

    #[test]
    fn adapter_state_survives_a_new_session() {
        let mut r = rig();
        r.send(&req(RAILS, 1, &[0b01, 0b11]));
        r.server.take_adapter_request();
        r.server.reset_session(&mut r.bus);
        r.take();
        r.send(&req(RAILS, 0, &[0, 0]));
        assert_eq!(
            r.take(),
            [Resp::Done {
                seq: 0,
                kind: RAILS,
                result: vec![0b01]
            }]
        );
    }
}
