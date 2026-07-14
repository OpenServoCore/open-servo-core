//! Sans-io session: record framing over the pipe's byte stream, both
//! directions. Encoders reuse `osc_host::link::record`'s vocabulary (the
//! wire truth); this module owns the decode direction the adapter never
//! needs. Window is 1 by engine contract -- the seq exists to make any
//! adapter/pipe disagreement loud, and lets a future queued adapter widen
//! the window without an API break.

use osc_host::engine::{Command, Outcome};
use osc_host::link::record as rec;
use osc_protocol::wire::Inst;

use crate::error::LinkError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkInfo {
    pub version: u8,
    pub ticks_per_us: u32,
}

/// One decoded adapter->client record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Record {
    Info(LinkInfo),
    Status {
        seq: u16,
        slot: u8,
        id: u8,
        inst: Inst,
        payload: Vec<u8>,
    },
    Terminal {
        seq: u16,
        outcome: Outcome,
        tick: u32,
        statuses: u8,
        garble: u16,
        trailing: bool,
    },
    Rejected {
        seq: u16,
        reason: u8,
    },
    BootloaderAck,
    RailsAck {
        state: u8,
    },
    Unknown {
        rtype: u8,
    },
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

    fn alloc_seq(&mut self) -> u16 {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        if self.next_seq == rec::SEQ_NONE {
            self.next_seq = 0;
        }
        seq
    }

    /// Length-prefix one record body (type + payload) onto `out`.
    fn frame(out: &mut Vec<u8>, body: &[u8]) {
        out.extend_from_slice(&(body.len() as u16).to_le_bytes());
        out.extend_from_slice(body);
    }

    pub fn encode_hello(out: &mut Vec<u8>) {
        Self::frame(out, &[rec::REC_HELLO]);
    }

    pub fn encode_enter_bootloader(out: &mut Vec<u8>) {
        Self::frame(out, &[rec::REC_ENTER_BOOTLOADER]);
    }

    pub fn encode_set_rails(out: &mut Vec<u8>, v3v3: bool, v5: bool) {
        let state = v3v3 as u8 | (v5 as u8) << 1;
        Self::frame(out, &[rec::REC_SET_RAILS, state]);
    }

    /// Encode a SUBMIT; returns the seq that will tag its answer.
    pub fn encode_submit(&mut self, out: &mut Vec<u8>, cmd: &Command<'_>) -> u16 {
        let seq = self.alloc_seq();
        let mut body = vec![rec::REC_SUBMIT, seq as u8, (seq >> 8) as u8];
        match cmd {
            Command::Exchange { id, inst, payload } => {
                body.push(rec::VERB_EXCHANGE);
                body.push(id.as_byte());
                body.push(inst.0);
                body.extend_from_slice(payload);
            }
            Command::Rescue => body.push(rec::VERB_RESCUE),
            Command::HostBaud(rate) => {
                body.push(rec::VERB_HOST_BAUD);
                body.push(rate.as_idx());
            }
            Command::SetResponseDeadline { us } => {
                body.push(rec::VERB_SET_RESPONSE_DEADLINE);
                body.extend_from_slice(&us.to_le_bytes());
            }
        }
        Self::frame(out, &body);
        seq
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
        if len == 0 {
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

fn decode(body: &[u8]) -> Result<Record, LinkError> {
    let field = |range: std::ops::Range<usize>| -> Result<&[u8], LinkError> {
        body.get(range).ok_or(LinkError::Malformed)
    };
    let seq =
        || -> Result<u16, LinkError> { Ok(u16::from_le_bytes(field(1..3)?.try_into().unwrap())) };
    Ok(match body[0] {
        rec::REC_INFO => Record::Info(LinkInfo {
            version: *body.get(1).ok_or(LinkError::Malformed)?,
            ticks_per_us: u32::from_le_bytes(field(2..6)?.try_into().unwrap()),
        }),
        rec::REC_STATUS => Record::Status {
            seq: seq()?,
            slot: *body.get(3).ok_or(LinkError::Malformed)?,
            id: *body.get(4).ok_or(LinkError::Malformed)?,
            inst: Inst(*body.get(5).ok_or(LinkError::Malformed)?),
            payload: body[6..].to_vec(),
        },
        rec::REC_TERMINAL => {
            let outcome = match *body.get(3).ok_or(LinkError::Malformed)? {
                rec::OUTCOME_SENT => Outcome::Sent,
                rec::OUTCOME_COMPLETE => Outcome::Complete,
                rec::OUTCOME_TIMEOUT => Outcome::Timeout {
                    slot: *body.get(4).ok_or(LinkError::Malformed)?,
                },
                _ => return Err(LinkError::Malformed),
            };
            Record::Terminal {
                seq: seq()?,
                outcome,
                tick: u32::from_le_bytes(field(5..9)?.try_into().unwrap()),
                statuses: *body.get(9).ok_or(LinkError::Malformed)?,
                garble: u16::from_le_bytes(field(10..12)?.try_into().unwrap()),
                trailing: *body.get(12).ok_or(LinkError::Malformed)?
                    & rec::FLAG_GARBLE_AFTER_LAST_FRAME
                    != 0,
            }
        }
        rec::REC_REJECTED => Record::Rejected {
            seq: seq()?,
            reason: *body.get(3).ok_or(LinkError::Malformed)?,
        },
        rec::REC_BOOTLOADER_ACK => Record::BootloaderAck,
        rec::REC_RAILS_ACK => Record::RailsAck {
            state: *body.get(1).ok_or(LinkError::Malformed)?,
        },
        rec::REC_UNKNOWN => Record::Unknown {
            rtype: *body.get(1).ok_or(LinkError::Malformed)?,
        },
        _ => return Err(LinkError::Malformed),
    })
}

// Re-exported so callers can name the submit vocabulary without a direct
// osc-host dep.
pub use osc_host::engine::Command as EngineCommand;
