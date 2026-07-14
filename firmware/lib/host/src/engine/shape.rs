//! Exchange validation + await-shape derivation: the host-direction mirror
//! of the servo's decode. Every submitted Exchange runs through [`derive`]
//! before anything touches the wire -- this is where "clients cannot emit a
//! malformed frame" lives. The gate is parse-level validity (exactly what
//! the servo-side parsers accept); semantic verdicts (id ranges, table
//! addressing, `limit`) stay the servo's nack, reachable by construction.

use osc_protocol::FrameBytes;
use osc_protocol::frame::{AssignReq, CalReq, EnumReq, MgmtReq, ProfileReq, ReadReq, WriteReq};
use osc_protocol::group::{
    GreadPerTarget, GreadProfilePerTarget, GreadProfileUniform, GreadUniform, GwritePerTarget,
    GwriteUniform,
};
use osc_protocol::wire::{self, Id, Inst, MgmtOp, Opcode};

/// Why a submit was refused before reaching the wire.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InvalidReason {
    /// Not a wire-valid target (`0x00`, `0xFF`, reserved band).
    BadId,
    /// Status bit set or reserved opcode -- hosts send instructions.
    BadInst,
    /// Payload does not parse for the opcode/flag combination (includes a
    /// unicast CAL: its ack would collide with the train, protocol sec 9.3).
    BadPayload,
    /// Past the sec 5.1 payload ceiling.
    TooLong,
}

/// How many status frames answer the command.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Replies {
    /// None owed: NOREPLY, non-acking broadcast, GWRITE, CAL.
    None,
    /// One status (unicast with reply; broadcast ASSIGN's sole matcher).
    Single,
    /// A GREAD status chain, one per listed slot (protocol sec 6).
    Chain(u8),
    /// Broadcast ENUM: 0..N replies, gathered until the bus goes quiet.
    Collect,
}

/// The engine's plan for one Exchange, derived purely from the instruction.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Shape {
    pub replies: Replies,
    /// Ring footprint of the largest expected reply frame (bytes) -- the
    /// per-slot allowance inside the engine's timeout window.
    pub reply_footprint: u16,
    /// MGMT CAL: `(gap_us, gaps)` break train paced after the announce.
    pub train: Option<(u16, u8)>,
    /// Flash-stall op (SAVE/FACTORY): widen the window (protocol sec 9.4).
    pub slow: bool,
}

/// Largest legal frame footprint -- the allowance when the reply size is
/// unknowable host-side (profile reads, sec 5.2).
const MAX_REPLY: u16 = wire::footprint(u8::MAX) as u16;
/// Empty-payload status footprint (acks, nacks).
const ACK_REPLY: u16 = wire::footprint(3) as u16;
/// PING status: model(2) + fw(1).
const PING_REPLY: u16 = wire::footprint(6) as u16;
/// ENUM status: the full 16-byte UID.
const ENUM_REPLY: u16 = wire::footprint(3 + wire::UID_LEN as u8) as u16;

impl Shape {
    pub fn derive(id: Id, inst: Inst, payload: &[u8]) -> Result<Shape, InvalidReason> {
        if inst.is_status() {
            return Err(InvalidReason::BadInst);
        }
        let Some(op) = inst.opcode() else {
            return Err(InvalidReason::BadInst);
        };
        if !id.is_valid() {
            return Err(InvalidReason::BadId);
        }
        if payload.len() > wire::MAX_PAYLOAD as usize {
            return Err(InvalidReason::TooLong);
        }
        let pay = FrameBytes::from(payload);
        let broadcast = id.is_broadcast();

        let mut shape = match op {
            Opcode::Ping => {
                if !payload.is_empty() {
                    return Err(InvalidReason::BadPayload);
                }
                Shape::plain(broadcast, PING_REPLY)
            }
            Opcode::Read if inst.profile() => {
                ProfileReq::parse(pay).ok_or(InvalidReason::BadPayload)?;
                Shape::plain(broadcast, MAX_REPLY)
            }
            Opcode::Read => {
                let r = ReadReq::parse(pay).ok_or(InvalidReason::BadPayload)?;
                Shape::plain(broadcast, read_reply(r.count))
            }
            Opcode::Write => {
                WriteReq::parse(pay).ok_or(InvalidReason::BadPayload)?;
                Shape::plain(broadcast, ACK_REPLY)
            }
            Opcode::Commit => {
                if !payload.is_empty() {
                    return Err(InvalidReason::BadPayload);
                }
                Shape::plain(broadcast, ACK_REPLY)
            }
            Opcode::Gread => gread(inst, pay)?,
            Opcode::Gwrite => {
                // Parse-gate both forms; replies are never owed (sec 5).
                if inst.per_target() {
                    GwritePerTarget::parse(pay).ok_or(InvalidReason::BadPayload)?;
                } else {
                    GwriteUniform::parse(pay).ok_or(InvalidReason::BadPayload)?;
                }
                Shape::silent()
            }
            Opcode::Mgmt => mgmt(pay, broadcast)?,
        };

        // NOREPLY suppresses the status everywhere the servo honors it --
        // which excludes GREAD slots (a chain slot always replies, sec 6).
        if inst.noreply() && !matches!(shape.replies, Replies::Chain(_)) {
            shape.replies = Replies::None;
        }
        Ok(shape)
    }

    /// Plain-op reply rule (sec 5): unicast answers, broadcast is silent.
    fn plain(broadcast: bool, reply_footprint: u16) -> Shape {
        Shape {
            replies: if broadcast {
                Replies::None
            } else {
                Replies::Single
            },
            reply_footprint,
            train: None,
            slow: false,
        }
    }

    fn silent() -> Shape {
        Shape {
            replies: Replies::None,
            reply_footprint: 0,
            train: None,
            slow: false,
        }
    }
}

/// Reply footprint for a `count`-byte read; an over-ceiling request draws a
/// `limit` nack, so the allowance never exceeds the largest legal frame.
fn read_reply(count: u16) -> u16 {
    wire::footprint(3 + count.min(wire::MAX_PAYLOAD as u16) as u8) as u16
}

fn gread(inst: Inst, pay: FrameBytes<'_>) -> Result<Shape, InvalidReason> {
    let (slots, reply_footprint) = match (inst.profile(), inst.per_target()) {
        (false, false) => {
            let g = GreadUniform::parse(pay).ok_or(InvalidReason::BadPayload)?;
            (g.ids().count(), read_reply(g.count))
        }
        (false, true) => {
            let g = GreadPerTarget::parse(pay).ok_or(InvalidReason::BadPayload)?;
            let widest = g.iter().map(|t| read_reply(t.count)).max().unwrap_or(0);
            (g.iter().count(), widest)
        }
        (true, false) => {
            let g = GreadProfileUniform::parse(pay).ok_or(InvalidReason::BadPayload)?;
            (g.ids().count(), MAX_REPLY)
        }
        (true, true) => {
            let g = GreadProfilePerTarget::parse(pay).ok_or(InvalidReason::BadPayload)?;
            (g.iter().count(), MAX_REPLY)
        }
    };
    Ok(Shape {
        replies: Replies::Chain(slots as u8),
        reply_footprint,
        train: None,
        slow: false,
    })
}

fn mgmt(pay: FrameBytes<'_>, broadcast: bool) -> Result<Shape, InvalidReason> {
    let m = MgmtReq::parse(pay).ok_or(InvalidReason::BadPayload)?;
    Ok(match m.op {
        MgmtOp::Enum => {
            EnumReq::parse(m.args).ok_or(InvalidReason::BadPayload)?;
            Shape {
                replies: if broadcast {
                    Replies::Collect
                } else {
                    Replies::Single
                },
                reply_footprint: ENUM_REPLY,
                train: None,
                slow: false,
            }
        }
        MgmtOp::Assign => {
            AssignReq::parse(m.args).ok_or(InvalidReason::BadPayload)?;
            // The sole matcher acks even under broadcast (sec 9.2).
            Shape {
                replies: Replies::Single,
                reply_footprint: ACK_REPLY,
                train: None,
                slow: false,
            }
        }
        MgmtOp::Cal => {
            let c = CalReq::parse(m.args).ok_or(InvalidReason::BadPayload)?;
            // Broadcast-only (sec 9.3): a unicast CAL's ack would land where
            // the train starts. Refused here, not discovered on the wire.
            if !broadcast {
                return Err(InvalidReason::BadPayload);
            }
            Shape {
                replies: Replies::None,
                reply_footprint: 0,
                train: Some((c.gap_us, c.gaps)),
                slow: false,
            }
        }
        MgmtOp::Save | MgmtOp::Factory => {
            if !m.args.is_empty() {
                return Err(InvalidReason::BadPayload);
            }
            Shape {
                slow: true,
                ..Shape::plain(broadcast, ACK_REPLY)
            }
        }
        // REBOOT args are implementation-defined (sec 9.5): pass through.
        MgmtOp::Reboot => Shape::plain(broadcast, ACK_REPLY),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::build;
    use osc_protocol::wire::MAX_PAYLOAD;

    const UNI: Id = Id(5);
    const BC: Id = Id::BROADCAST;

    fn inst(op: Opcode, flags: u8) -> Inst {
        Inst::instruction(op, flags)
    }

    #[test]
    fn ping_shapes() {
        let s = Shape::derive(UNI, inst(Opcode::Ping, 0), &[]).unwrap();
        assert_eq!(s.replies, Replies::Single);
        assert_eq!(s.reply_footprint, 9);
        let s = Shape::derive(BC, inst(Opcode::Ping, 0), &[]).unwrap();
        assert_eq!(s.replies, Replies::None);
        assert_eq!(
            Shape::derive(UNI, inst(Opcode::Ping, 0), &[1]),
            Err(InvalidReason::BadPayload)
        );
    }

    #[test]
    fn statuses_and_bad_ids_refuse() {
        assert_eq!(
            Shape::derive(UNI, Inst::status(wire::ResultCode::Ok, false), &[]),
            Err(InvalidReason::BadInst)
        );
        assert_eq!(
            Shape::derive(Id(0), inst(Opcode::Ping, 0), &[]),
            Err(InvalidReason::BadId)
        );
        assert_eq!(
            Shape::derive(Id(0xFA), inst(Opcode::Ping, 0), &[]),
            Err(InvalidReason::BadId)
        );
        assert_eq!(
            Shape::derive(UNI, Inst(0x00), &[]),
            Err(InvalidReason::BadInst)
        );
    }

    #[test]
    fn read_reply_footprint_tracks_count() {
        let mut b = [0u8; 8];
        let n = build::read(&mut b, 0x0100, 16).unwrap();
        let s = Shape::derive(UNI, inst(Opcode::Read, 0), &b[..n]).unwrap();
        assert_eq!(s.reply_footprint, wire::footprint(3 + 16) as u16);
        // Over-ceiling read: the allowance caps at the largest legal frame.
        let n = build::read(&mut b, 0x0100, 4096).unwrap();
        let s = Shape::derive(UNI, inst(Opcode::Read, 0), &b[..n]).unwrap();
        assert_eq!(s.reply_footprint, MAX_REPLY);
    }

    #[test]
    fn noreply_silences_plain_but_not_chains() {
        let mut b = [0u8; 16];
        let n = build::write(&mut b, 0x0180, &[1, 2]).unwrap();
        let s = Shape::derive(UNI, inst(Opcode::Write, Inst::FLAG_NOREPLY), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::None);

        let ids = [Id(1), Id(2)];
        let n = build::gread_uniform(&mut b, 0, 4, &ids).unwrap();
        let s = Shape::derive(BC, inst(Opcode::Gread, Inst::FLAG_NOREPLY), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::Chain(2));
    }

    #[test]
    fn gread_chain_counts_slots() {
        let mut b = [0u8; 32];
        let ids = [Id(1), Id(2), Id(9)];
        let n = build::gread_uniform(&mut b, 0x0084, 8, &ids).unwrap();
        let s = Shape::derive(BC, inst(Opcode::Gread, 0), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::Chain(3));
        assert_eq!(s.reply_footprint, wire::footprint(3 + 8) as u16);
    }

    #[test]
    fn gread_per_target_takes_the_widest_slot() {
        let mut b = [0u8; 32];
        let ts = [
            osc_protocol::group::GreadTarget {
                id: Id(1),
                addr: 0,
                count: 2,
            },
            osc_protocol::group::GreadTarget {
                id: Id(2),
                addr: 0,
                count: 40,
            },
        ];
        let n = build::gread_per_target(&mut b, &ts).unwrap();
        let s = Shape::derive(BC, inst(Opcode::Gread, Inst::FLAG_PER_TARGET), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::Chain(2));
        assert_eq!(s.reply_footprint, wire::footprint(3 + 40) as u16);
    }

    #[test]
    fn gwrite_is_silent_and_parse_gated() {
        let mut b = [0u8; 32];
        let mut w = build::GwriteUniform::new(&mut b, 0x74, 4).unwrap();
        w.push(Id(1), &[0, 8, 0, 0]).unwrap();
        let n = w.finish().unwrap();
        let s = Shape::derive(BC, inst(Opcode::Gwrite, Inst::FLAG_HOLD), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::None);
        // Ragged payload refuses.
        assert_eq!(
            Shape::derive(BC, inst(Opcode::Gwrite, 0), &b[..n - 1]),
            Err(InvalidReason::BadPayload)
        );
    }

    #[test]
    fn mgmt_shapes() {
        let mut b = [0u8; 32];

        let prefix = [0u8; wire::UID_LEN];
        let n = build::mgmt_enum(&mut b, 0, &prefix).unwrap();
        let s = Shape::derive(BC, inst(Opcode::Mgmt, 0), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::Collect);
        let s = Shape::derive(UNI, inst(Opcode::Mgmt, 0), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::Single);

        let uid = [3u8; wire::UID_LEN];
        let n = build::mgmt_assign(&mut b, &uid, 7).unwrap();
        let s = Shape::derive(BC, inst(Opcode::Mgmt, 0), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::Single);

        let n = build::mgmt(&mut b, MgmtOp::Save).unwrap();
        let s = Shape::derive(UNI, inst(Opcode::Mgmt, 0), &b[..n]).unwrap();
        assert!(s.slow);
        assert_eq!(s.replies, Replies::Single);
    }

    #[test]
    fn cal_is_broadcast_only_and_carries_the_train() {
        let mut b = [0u8; 8];
        let n = build::mgmt_cal(&mut b, 400, 8).unwrap();
        let s = Shape::derive(BC, inst(Opcode::Mgmt, 0), &b[..n]).unwrap();
        assert_eq!(s.replies, Replies::None);
        assert_eq!(s.train, Some((400, 8)));
        assert_eq!(
            Shape::derive(UNI, inst(Opcode::Mgmt, 0), &b[..n]),
            Err(InvalidReason::BadPayload)
        );
    }

    #[test]
    fn oversized_payload_refuses() {
        let big = [0u8; MAX_PAYLOAD as usize + 1];
        assert_eq!(
            Shape::derive(UNI, inst(Opcode::Write, 0), &big),
            Err(InvalidReason::TooLong)
        );
    }
}
