//! The async client: one adapter, one command in flight, typed digests over
//! the raw record stream. Payloads come from `osc_protocol::build`, so a
//! client cannot emit a malformed instruction; the engine re-validates and
//! its rejections surface as [`Error::Link`].

use std::time::Duration;

use futures_lite::future;
use osc_host::engine::{Command, Outcome};
use osc_protocol::build;
use osc_protocol::wire::{BaudRate, Id, Inst, Opcode, ResultCode};

use crate::error::{Error, LinkError, RejectReason};
use crate::pipe::{Pipe, PipeError};
use crate::session::{LinkInfo, Record, Session};

/// One decoded status frame: the responder's verdict plus payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Status {
    /// GREAD list position; 0 for unicast; arrival order under ENUM collect.
    pub slot: u8,
    pub id: u8,
    /// `None` only for a reserved result-code pattern (kept visible rather
    /// than dropped -- a rogue-frame symptom).
    pub result: Option<ResultCode>,
    /// Device-level alarm (protocol sec 5.3 layer 3): the node's fault
    /// register is nonzero. Orthogonal to `result`.
    pub alert: bool,
    pub payload: Vec<u8>,
}

/// The raw digest of one command: streamed statuses plus its terminal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reply {
    pub statuses: Vec<Status>,
    pub outcome: Outcome,
    /// Engine tick at the terminal; [`Client::ticks`] converts.
    pub tick: u32,
    /// Ring bytes that anchored no clean status during the await.
    pub garble: u16,
    /// Garble after the last clean frame (the sec 9.2 trailing-energy bit).
    pub trailing: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ping {
    pub model: u16,
    pub fw: u8,
    pub alert: bool,
}

/// A GREAD chain digest. Chain-level trouble is data, not `Err`: a silent
/// slot truncates `statuses` and names itself, `predecessor-silent` arrives
/// as a per-slot result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chain {
    pub statuses: Vec<Status>,
    /// The slot that produced nothing, when the chain timed out.
    pub timeout_slot: Option<u8>,
}

/// Client-side watchdog on pipe delivery: catches a dead adapter, carries
/// no protocol meaning (the engine owns every protocol window), so it sits
/// orders of magnitude above them.
const DEFAULT_GUARD: Duration = Duration::from_secs(2);

pub struct Client<P: Pipe> {
    pub(crate) pipe: P,
    pub(crate) session: Session,
    pub(crate) anchor: crate::wire::EdgeAnchor,
    info: LinkInfo,
    guard: Duration,
}

impl<P: Pipe> Client<P> {
    /// HELLO -> INFO handshake; the INFO advertises the terminal tick rate.
    pub async fn connect(pipe: P) -> Result<Self, Error> {
        let mut c = Self {
            pipe,
            session: Session::new(),
            anchor: Default::default(),
            info: LinkInfo {
                version: 0,
                ticks_per_us: 1,
            },
            guard: DEFAULT_GUARD,
        };
        let mut out = Vec::new();
        Session::encode_hello(&mut out);
        c.pipe.send(&out).await?;
        match c.next_record().await? {
            Record::Info(info) => {
                c.info = info;
                Ok(c)
            }
            other => Err(desync("INFO", &other)),
        }
    }

    pub fn info(&self) -> LinkInfo {
        self.info
    }

    /// Reach the transport (the fake backend's sim hooks live there).
    pub fn pipe_mut(&mut self) -> &mut P {
        &mut self.pipe
    }

    pub fn set_guard(&mut self, guard: Duration) {
        self.guard = guard;
    }

    /// Convert a terminal tick count (a span, not an instant) to wall time.
    pub fn ticks(&self, span: u32) -> Duration {
        Duration::from_nanos(span as u64 * 1_000 / self.info.ticks_per_us as u64)
    }

    async fn recv_guarded(&mut self) -> Result<Vec<u8>, Error> {
        let recv = async { Some(self.pipe.recv().await) };
        let lapse = async {
            futures_timer::Delay::new(self.guard).await;
            None
        };
        match future::or(recv, lapse).await {
            Some(chunk) => Ok(chunk?),
            None => Err(Error::Pipe(PipeError::Stalled)),
        }
    }

    pub(crate) async fn next_record(&mut self) -> Result<Record, Error> {
        loop {
            if let Some(record) = self.session.next_record()? {
                return Ok(record);
            }
            let chunk = self.recv_guarded().await?;
            self.session.on_bytes(&chunk);
        }
    }

    /// Submit one engine command and collect its full answer.
    pub async fn submit(&mut self, cmd: Command<'_>) -> Result<Reply, Error> {
        let mut out = Vec::new();
        let seq = self.session.encode_submit(&mut out, &cmd);
        self.pipe.send(&out).await?;
        let mut statuses = Vec::new();
        loop {
            match self.next_record().await? {
                Record::Status {
                    seq: s,
                    slot,
                    id,
                    inst,
                    payload,
                } if s == seq => statuses.push(Status {
                    slot,
                    id,
                    result: inst.result(),
                    alert: inst.alert(),
                    payload,
                }),
                Record::Terminal {
                    seq: s,
                    outcome,
                    tick,
                    garble,
                    trailing,
                    ..
                } if s == seq => {
                    return Ok(Reply {
                        statuses,
                        outcome,
                        tick,
                        garble,
                        trailing,
                    });
                }
                Record::Rejected { seq: s, reason } if s == seq => {
                    return Err(Error::Link(LinkError::Rejected(RejectReason::from_byte(
                        reason,
                    ))));
                }
                other => return Err(desync("a record on the active seq", &other)),
            }
        }
    }

    pub async fn exchange(&mut self, id: Id, inst: Inst, payload: &[u8]) -> Result<Reply, Error> {
        self.submit(Command::Exchange { id, inst, payload }).await
    }

    /// Quiet bus time (the sec 8 pacing primitive): wall time on hardware,
    /// sim time on the fake adapter.
    pub async fn pause(&mut self, d: Duration) {
        self.pipe.pause(d).await;
    }

    /// Digest a single-status reply: exactly one OK status or an error.
    fn sole_ok(reply: Reply) -> Result<Status, Error> {
        if let Outcome::Timeout { slot } = reply.outcome {
            return Err(Error::Timeout { slot });
        }
        let [status] = <[Status; 1]>::try_from(reply.statuses)
            .map_err(|got| desync_msg(format!("one status, got {}", got.len())))?;
        match status.result {
            Some(ResultCode::Ok) => Ok(status),
            Some(code) => Err(Error::Servo(code)),
            None => Err(desync_msg("reserved result code".into())),
        }
    }

    pub async fn ping(&mut self, id: Id) -> Result<Ping, Error> {
        let inst = Inst::instruction(Opcode::Ping, 0);
        let status = Self::sole_ok(self.exchange(id, inst, &[]).await?)?;
        if status.payload.len() < 3 {
            return Err(desync_msg("short PING payload".into()));
        }
        Ok(Ping {
            model: u16::from_le_bytes([status.payload[0], status.payload[1]]),
            fw: status.payload[2],
            alert: status.alert,
        })
    }

    pub async fn read(&mut self, id: Id, addr: u16, count: u16) -> Result<Vec<u8>, Error> {
        let mut p = [0u8; 8];
        let n = build::read(&mut p, addr, count).ok_or(Error::Servo(ResultCode::Range))?;
        let inst = Inst::instruction(Opcode::Read, 0);
        let status = Self::sole_ok(self.exchange(id, inst, &p[..n]).await?)?;
        Ok(status.payload)
    }

    /// Single-target PROFILE read (sec 5.2): the payload names a slot the
    /// target pre-configured; the reply streams its gathered spans.
    pub async fn read_profile(&mut self, id: Id, slot: u8) -> Result<Vec<u8>, Error> {
        let mut p = [0u8; 1];
        let n = build::read_profile(&mut p, slot).ok_or(Error::Servo(ResultCode::Range))?;
        let inst = Inst::instruction(Opcode::Read, Inst::FLAG_PROFILE);
        let status = Self::sole_ok(self.exchange(id, inst, &p[..n]).await?)?;
        Ok(status.payload)
    }

    pub async fn write(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), Error> {
        self.write_flags(id, addr, data, 0).await
    }

    /// HOLD-staged write: applied by the next COMMIT.
    pub async fn write_hold(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), Error> {
        self.write_flags(id, addr, data, Inst::FLAG_HOLD).await
    }

    async fn write_flags(
        &mut self,
        id: Id,
        addr: u16,
        data: &[u8],
        flags: u8,
    ) -> Result<(), Error> {
        let mut p = vec![0u8; data.len() + 4];
        let n = build::write(&mut p, addr, data).ok_or(Error::Servo(ResultCode::Limit))?;
        let inst = Inst::instruction(Opcode::Write, flags);
        Self::sole_ok(self.exchange(id, inst, &p[..n]).await?)?;
        Ok(())
    }

    /// Fire-and-forget write: completes at wire-TX end, no status owed.
    pub async fn write_noreply(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), Error> {
        let mut p = vec![0u8; data.len() + 4];
        let n = build::write(&mut p, addr, data).ok_or(Error::Servo(ResultCode::Limit))?;
        let inst = Inst::instruction(Opcode::Write, Inst::FLAG_NOREPLY);
        self.exchange(id, inst, &p[..n]).await?;
        Ok(())
    }

    pub async fn gread(&mut self, ids: &[Id], addr: u16, count: u16) -> Result<Chain, Error> {
        let mut p = vec![0u8; ids.len() + 8];
        let n = build::gread_uniform(&mut p, addr, count, ids)
            .ok_or(Error::Servo(ResultCode::Limit))?;
        let inst = Inst::instruction(Opcode::Gread, 0);
        let reply = self.exchange(Id::BROADCAST, inst, &p[..n]).await?;
        Ok(chain_digest(reply))
    }

    /// Uniform profile-slot GREAD (sec 5.2): each target streams the span
    /// list it pre-configured for `slot`.
    pub async fn gread_profile(&mut self, ids: &[Id], slot: u8) -> Result<Chain, Error> {
        let mut p = vec![0u8; ids.len() + 4];
        let n = build::gread_profile_uniform(&mut p, slot, ids)
            .ok_or(Error::Servo(ResultCode::Limit))?;
        let inst = Inst::instruction(Opcode::Gread, Inst::FLAG_PROFILE);
        let reply = self.exchange(Id::BROADCAST, inst, &p[..n]).await?;
        Ok(chain_digest(reply))
    }

    /// Uniform GWRITE (silent unless a slice is missing -- completes at TX
    /// end). `count` is the per-target data width; every pair must match it.
    pub async fn gwrite(
        &mut self,
        addr: u16,
        count: u8,
        pairs: &[(Id, &[u8])],
    ) -> Result<(), Error> {
        self.gwrite_flags(addr, count, pairs, 0).await
    }

    /// HOLD-staged uniform GWRITE: the fleet applies on the next COMMIT.
    pub async fn gwrite_hold(
        &mut self,
        addr: u16,
        count: u8,
        pairs: &[(Id, &[u8])],
    ) -> Result<(), Error> {
        self.gwrite_flags(addr, count, pairs, Inst::FLAG_HOLD).await
    }

    async fn gwrite_flags(
        &mut self,
        addr: u16,
        count: u8,
        pairs: &[(Id, &[u8])],
        flags: u8,
    ) -> Result<(), Error> {
        let mut p = vec![0u8; 4 + pairs.len() * (1 + count as usize)];
        let mut b = build::GwriteUniform::new(&mut p, addr, count)
            .ok_or(Error::Servo(ResultCode::Limit))?;
        for (id, data) in pairs {
            b.push(*id, data).ok_or(Error::Servo(ResultCode::Limit))?;
        }
        let n = b.finish().ok_or(Error::Servo(ResultCode::Limit))?;
        let inst = Inst::instruction(Opcode::Gwrite, flags);
        self.exchange(Id::BROADCAST, inst, &p[..n]).await?;
        Ok(())
    }

    /// Broadcast COMMIT: applies every held write in the same instant.
    pub async fn commit(&mut self) -> Result<(), Error> {
        let inst = Inst::instruction(Opcode::Commit, 0);
        self.exchange(Id::BROADCAST, inst, &[]).await?;
        Ok(())
    }

    /// Rescue pulse; the host UART lands at 0.5M with the rescued fleet.
    pub async fn rescue(&mut self) -> Result<(), Error> {
        self.submit(Command::Rescue).await?;
        Ok(())
    }

    /// Host-side UART rate only -- servo-first ordering is
    /// [`crate::mgmt::set_baud`]'s job.
    pub async fn host_baud(&mut self, rate: BaudRate) -> Result<(), Error> {
        self.submit(Command::HostBaud(rate)).await?;
        Ok(())
    }

    /// The engine's mirrored copy of the fleet's RESPONSE_DEADLINE register.
    pub async fn set_response_deadline(&mut self, us: u16) -> Result<(), Error> {
        self.submit(Command::SetResponseDeadline { us }).await?;
        Ok(())
    }

    /// Drive both DUT power rails (absolute state); returns the acked
    /// `(3V3, 5V)` state.
    pub async fn set_rails(&mut self, v3v3: bool, v5: bool) -> Result<(bool, bool), Error> {
        self.rails_op(v3v3 as u8 | (v5 as u8) << 1, 0b11).await
    }

    /// Drive the 3V3 rail, 5V untouched.
    pub async fn set_rail_3v3(&mut self, on: bool) -> Result<(bool, bool), Error> {
        self.rails_op(on as u8, 0b01).await
    }

    /// Drive the 5V rail, 3V3 untouched.
    pub async fn set_rail_5v(&mut self, on: bool) -> Result<(bool, bool), Error> {
        self.rails_op((on as u8) << 1, 0b10).await
    }

    /// Read the rails without driving them.
    pub async fn rails(&mut self) -> Result<(bool, bool), Error> {
        self.rails_op(0, 0).await
    }

    async fn rails_op(&mut self, state: u8, mask: u8) -> Result<(bool, bool), Error> {
        let mut out = Vec::new();
        Session::encode_set_rails(&mut out, state, mask);
        self.pipe.send(&out).await?;
        match self.next_record().await? {
            Record::RailsAck { state } => Ok((state & 1 != 0, state & 2 != 0)),
            other => Err(desync("RAILS ack", &other)),
        }
    }

    /// Hand the adapter to its resident bootloader; the ack is the last
    /// record this pipe delivers.
    pub async fn enter_bootloader(&mut self) -> Result<(), Error> {
        let mut out = Vec::new();
        Session::encode_enter_bootloader(&mut out);
        self.pipe.send(&out).await?;
        match self.next_record().await? {
            Record::BootloaderAck => Ok(()),
            other => Err(desync("BOOTLOADER ack", &other)),
        }
    }
}

fn chain_digest(reply: Reply) -> Chain {
    Chain {
        statuses: reply.statuses,
        timeout_slot: match reply.outcome {
            Outcome::Timeout { slot } => Some(slot),
            _ => None,
        },
    }
}

pub(crate) fn desync(expected: &str, got: &Record) -> Error {
    desync_msg(format!("expected {expected}, got {got:?}"))
}

pub(crate) fn desync_msg(msg: String) -> Error {
    Error::Link(LinkError::Desync(msg))
}
