//! Sans-io session: record framing over the pipe's byte stream, both
//! directions (link sec 3). Encoders reuse `osc_host::link::record`'s
//! vocabulary (the wire truth); this module owns the decode direction the
//! adapter never needs. One bus request is outstanding at a time by link
//! contract; the seq makes any adapter/pipe disagreement loud.

use osc_host::engine::{Command, TbExpect};
use osc_host::link::record as rec;

use crate::error::LinkError;

/// GET_INFO's result (link sec 4.1).
pub use osc_host::link::record::Info as LinkInfo;

/// The bus mode SET_BUS names (link sec 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusMode {
    Osc,
    Tinyboot,
}

impl BusMode {
    pub fn byte(self) -> u8 {
        match self {
            BusMode::Osc => rec::MODE_OSC,
            BusMode::Tinyboot => rec::MODE_TINYBOOT,
        }
    }
}

/// What an encoder sent: the seq its responses carry and the kind its
/// DONE must echo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tag {
    pub seq: u16,
    pub kind: u8,
}

/// One decoded response record (link sec 3.2).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Record {
    Frame { seq: u16, slot: u8, bytes: Vec<u8> },
    Done { seq: u16, kind: u8, result: Vec<u8> },
    Error { seq: u16, kind: u8, code: u8 },
}

pub struct Session {
    rx: Vec<u8>,
    next_seq: u16,
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}

impl Session {
    pub fn new() -> Self {
        Self {
            rx: Vec::new(),
            next_seq: 0,
        }
    }

    /// Append one request record onto `out`, under a fresh seq.
    fn request(&mut self, out: &mut Vec<u8>, kind: u8, body: &[u8]) -> Tag {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        if self.next_seq == rec::SEQ_NONE {
            self.next_seq = 0;
        }
        out.extend_from_slice(&((rec::HEADER - 2 + body.len()) as u16).to_le_bytes());
        out.push(kind);
        out.extend_from_slice(&seq.to_le_bytes());
        out.extend_from_slice(body);
        Tag { seq, kind }
    }

    pub fn encode_get_info(&mut self, out: &mut Vec<u8>) -> Tag {
        self.request(out, rec::GET_INFO, &[])
    }

    pub fn encode_adapter_iap(&mut self, out: &mut Vec<u8>) -> Tag {
        self.request(out, rec::ADAPTER_IAP, &[])
    }

    /// `mask` selects which rails the state byte applies to (bit0 = 3V3,
    /// bit1 = 5V); mask 0 reads the state back without driving anything.
    pub fn encode_rails(&mut self, out: &mut Vec<u8>, state: u8, mask: u8) -> Tag {
        self.request(out, rec::RAILS, &[state, mask])
    }

    pub fn encode_set_bus(&mut self, out: &mut Vec<u8>, mode: BusMode, bps: u32) -> Tag {
        let mut body = vec![mode.byte()];
        body.extend_from_slice(&bps.to_le_bytes());
        self.request(out, rec::SET_BUS, &body)
    }

    /// Encode an engine command as its bus request.
    pub fn encode_submit(&mut self, out: &mut Vec<u8>, cmd: &Command<'_>) -> Tag {
        let exchange = |flags: u8, id: u8, inst: u8, window_us: u32, payload: &[u8]| {
            let mut body = vec![flags, id, inst];
            body.extend_from_slice(&window_us.to_le_bytes());
            body.extend_from_slice(payload);
            body
        };
        match cmd {
            Command::Exchange { id, inst, payload } => {
                let body = exchange(0, id.as_byte(), inst.0, 0, payload);
                self.request(out, rec::OSC_EXCHANGE, &body)
            }
            Command::ExchangeStream {
                id,
                inst,
                payload,
                window_us,
            } => {
                let body = exchange(
                    rec::EXCHANGE_STREAM,
                    id.as_byte(),
                    inst.0,
                    *window_us,
                    payload,
                );
                self.request(out, rec::OSC_EXCHANGE, &body)
            }
            Command::Rescue => self.request(out, rec::OSC_RESCUE, &[]),
            Command::HostBaud { bps } => self.encode_set_bus(out, BusMode::Osc, *bps),
            Command::SetResponseDeadline { us } => {
                self.request(out, rec::OSC_DEADLINE, &us.to_le_bytes())
            }
            Command::TbExchange {
                cmd,
                addr,
                flags,
                data,
                expect,
                window_us,
            } => {
                let expect = match expect {
                    TbExpect::None => rec::EXPECT_NONE,
                    TbExpect::One => rec::EXPECT_ONE,
                    TbExpect::Collect => rec::EXPECT_COLLECT,
                };
                let mut body = vec![expect];
                body.extend_from_slice(&window_us.to_le_bytes());
                body.push(*cmd);
                body.extend_from_slice(&addr.to_le_bytes()[..3]);
                body.push(*flags);
                body.extend_from_slice(data);
                self.request(out, rec::TB_EXCHANGE, &body)
            }
        }
    }

    /// Feed a delivered chunk into the reassembly buffer.
    pub fn on_bytes(&mut self, chunk: &[u8]) {
        self.rx.extend_from_slice(chunk);
    }

    /// Pop the next complete record, if one has fully arrived.
    pub fn next_record(&mut self) -> Result<Option<Record>, LinkError> {
        if self.rx.len() < 2 {
            return Ok(None);
        }
        let len = u16::from_le_bytes([self.rx[0], self.rx[1]]) as usize;
        if len < rec::HEADER - 2 {
            return Err(LinkError::Malformed);
        }
        if self.rx.len() < 2 + len {
            return Ok(None);
        }
        let record = decode(&self.rx[2..2 + len])?;
        self.rx.drain(..2 + len);
        Ok(Some(record))
    }
}

/// The instrument kinds (link sec 9), `bench` only.
#[cfg(feature = "bench")]
impl Session {
    pub fn encode_wire_send(&mut self, out: &mut Vec<u8>, bytes: &[u8]) -> Tag {
        self.request(out, rec::WIRE_SEND, bytes)
    }

    /// Encode a wire burst; `stream` is the `[len(1) bytes]+` frame train.
    pub fn encode_wire_burst(&mut self, out: &mut Vec<u8>, stream: &[u8]) -> Tag {
        self.request(out, rec::WIRE_BURST, stream)
    }

    pub fn encode_wire_pulse(&mut self, out: &mut Vec<u8>, us: u16) -> Tag {
        self.request(out, rec::WIRE_PULSE, &us.to_le_bytes())
    }

    /// Encode a break train: announce frame + `breaks` bare law breaks
    /// `gap_us` apart on the adapter's deadline grid.
    pub fn encode_wire_train(
        &mut self,
        out: &mut Vec<u8>,
        announce: &[u8],
        gap_us: u16,
        breaks: u8,
    ) -> Tag {
        let mut body = gap_us.to_le_bytes().to_vec();
        body.push(breaks);
        body.extend_from_slice(announce);
        self.request(out, rec::WIRE_TRAIN, &body)
    }

    pub fn encode_wire_baud(&mut self, out: &mut Vec<u8>, bps: u32) -> Tag {
        self.request(out, rec::WIRE_BAUD, &bps.to_le_bytes())
    }

    pub fn encode_edge_drain(&mut self, out: &mut Vec<u8>, max: u8) -> Tag {
        self.request(out, rec::EDGE_DRAIN, &[max])
    }

    pub fn encode_capture_reset(&mut self, out: &mut Vec<u8>) -> Tag {
        self.request(out, rec::CAPTURE_RESET, &[])
    }
}

/// `r` is everything after `len`, at least kind and seq.
fn decode(r: &[u8]) -> Result<Record, LinkError> {
    let seq = u16::from_le_bytes([r[1], r[2]]);
    Ok(match (r[0], &r[rec::HEADER - 2..]) {
        (rec::FRAME, [slot, bytes @ ..]) => Record::Frame {
            seq,
            slot: *slot,
            bytes: bytes.to_vec(),
        },
        (rec::DONE, [kind, result @ ..]) => Record::Done {
            seq,
            kind: *kind,
            result: result.to_vec(),
        },
        (rec::ERROR, &[kind, code]) => Record::Error { seq, kind, code },
        _ => return Err(LinkError::Malformed),
    })
}

// Re-exported so callers can name the submit vocabulary without a direct
// osc-host dep.
pub use osc_host::engine::Command as EngineCommand;

#[cfg(test)]
mod tests {
    use osc_host::engine::Outcome;
    use osc_host::link::record::ExchangeResult;
    use osc_protocol::wire::{BaudRate, Id, Inst, Opcode};

    use super::*;

    fn decode_one(bytes: &[u8]) -> Result<Option<Record>, LinkError> {
        let mut s = Session::new();
        s.on_bytes(bytes);
        s.next_record()
    }

    #[test]
    fn responses_decode() {
        let mut buf = [0u8; 64];
        let r = ExchangeResult {
            outcome: Outcome::Timeout { slot: 1 },
            tick: 7,
            frames: 12,
            garble: 188,
            trailing: true,
            laps: 2,
        };
        let done = rec::done(&mut buf, 9, rec::OSC_EXCHANGE, |d| r.write(d)).to_vec();
        let Ok(Some(Record::Done { seq, kind, result })) = decode_one(&done) else {
            panic!("not a DONE");
        };
        assert_eq!((seq, kind), (9, rec::OSC_EXCHANGE));
        assert_eq!(ExchangeResult::parse(&result), Some(r));

        let frame = rec::osc_frame(&mut buf, 3, 2, 5, 0x80, (&[1], &[2])).to_vec();
        assert_eq!(
            decode_one(&frame),
            Ok(Some(Record::Frame {
                seq: 3,
                slot: 2,
                bytes: vec![5, 0x80, 1, 2]
            }))
        );

        let error = rec::error(&mut buf, 4, rec::SET_BUS, rec::ERR_BAD_RATE).to_vec();
        assert_eq!(
            decode_one(&error),
            Ok(Some(Record::Error {
                seq: 4,
                kind: rec::SET_BUS,
                code: rec::ERR_BAD_RATE
            }))
        );
    }

    #[test]
    fn decode_is_strict() {
        for bad in [
            // A request kind.
            &[3, 0, rec::GET_INFO, 0, 0][..],
            // An unassigned response kind.
            &[3, 0, 0x83, 0, 0],
            // DONE without its kind, FRAME without its slot.
            &[3, 0, rec::DONE, 0, 0],
            &[3, 0, rec::FRAME, 0, 0],
            // ERROR short and long.
            &[4, 0, rec::ERROR, 0, 0, 1],
            &[6, 0, rec::ERROR, 0, 0, 1, 2, 3],
            // `len` with no room for kind and seq.
            &[2, 0, rec::DONE, 0],
        ] {
            assert_eq!(decode_one(bad), Err(LinkError::Malformed), "{bad:?}");
        }
        assert_eq!(decode_one(&[9, 0, rec::DONE]), Ok(None), "partial");
    }

    #[test]
    fn encoders_tag_and_frame_each_request() {
        let mut s = Session::new();
        let mut out = Vec::new();
        let a = s.encode_get_info(&mut out);
        let b = s.encode_submit(
            &mut out,
            &Command::HostBaud {
                bps: BaudRate::B2000000.as_hz(),
            },
        );
        let c = s.encode_submit(
            &mut out,
            &Command::Exchange {
                id: Id::new(5),
                inst: Inst::instruction(Opcode::Ping, 0),
                payload: &[],
            },
        );
        assert_eq!(
            [a, b, c],
            [
                Tag {
                    seq: 0,
                    kind: rec::GET_INFO
                },
                Tag {
                    seq: 1,
                    kind: rec::SET_BUS
                },
                Tag {
                    seq: 2,
                    kind: rec::OSC_EXCHANGE
                },
            ]
        );
        let ping = Inst::instruction(Opcode::Ping, 0).0;
        let mut want = vec![3, 0, rec::GET_INFO, 0, 0];
        want.extend_from_slice(&[8, 0, rec::SET_BUS, 1, 0, rec::MODE_OSC, 0x80, 0x84, 0x1E, 0]);
        want.extend_from_slice(&[10, 0, rec::OSC_EXCHANGE, 2, 0, 0, 5, ping, 0, 0, 0, 0]);
        assert_eq!(out, want);
    }

    #[test]
    fn tb_exchange_encodes_the_link_body() {
        let mut s = Session::new();
        let mut out = Vec::new();
        let t = s.encode_submit(
            &mut out,
            &Command::TbExchange {
                cmd: 3,
                addr: 0x0008_0400,
                flags: 0x01,
                data: &[0xDE, 0xAD],
                expect: TbExpect::Collect,
                window_us: 0x0001_0203,
            },
        );
        assert_eq!(t.kind, rec::TB_EXCHANGE);
        assert_eq!(
            out,
            [
                15,
                0,
                rec::TB_EXCHANGE,
                0,
                0,
                rec::EXPECT_COLLECT,
                0x03,
                0x02,
                0x01,
                0x00,
                3,
                0x00,
                0x04,
                0x08,
                0x01,
                0xDE,
                0xAD
            ]
        );
    }

    #[test]
    fn set_bus_encodes_mode_and_bps() {
        let mut s = Session::new();
        let mut out = Vec::new();
        let a = s.encode_set_bus(&mut out, BusMode::Tinyboot, 115_200);
        let b = s.encode_set_bus(&mut out, BusMode::Osc, 3_000_000);
        assert_eq!((a.kind, b.kind), (rec::SET_BUS, rec::SET_BUS));
        assert_eq!(
            out,
            [
                8,
                0,
                rec::SET_BUS,
                0,
                0,
                rec::MODE_TINYBOOT,
                0x00,
                0xC2,
                0x01,
                0x00,
                8,
                0,
                rec::SET_BUS,
                1,
                0,
                rec::MODE_OSC,
                0xC0,
                0xC6,
                0x2D,
                0x00
            ]
        );
    }

    #[test]
    fn seq_skips_seq_none() {
        let mut s = Session::new();
        s.next_seq = rec::SEQ_NONE - 1;
        let mut out = Vec::new();
        assert_eq!(s.encode_get_info(&mut out).seq, rec::SEQ_NONE - 1);
        assert_eq!(s.encode_get_info(&mut out).seq, 0);
    }
}
