//! Linearized frame -> decoded request mapping (`docs/osc-native-protocol.md`
//! sec 5, sec 6). Input is a whole, CRC-clean frame (header pre-validated by the
//! framer): `[0x00][ID][LEN][INST][payload][PAD?][crc][crc]`. Output is the
//! chip-agnostic [`Request`] plus its reply contract and chain slot, or a
//! classification the composite acts on without dispatching.
//!
//! Addressing: plain ops address by frame ID (unicast-to-us or broadcast);
//! group ops address by their id-lists regardless of the (broadcast) frame ID.
//! Own instruction frames never echo while we are the responder (F9), so no
//! self-filtering is coded here.

use osc_core::traits::{Request, RequestCtx};
use osc_protocol::FrameBytes;
use osc_protocol::frame::{
    AssignReq, CalReq, EnumReq, Header, MgmtReq, ProfileReq, ReadReq, WriteReq,
};
use osc_protocol::group::{
    GreadPerTarget, GreadProfilePerTarget, GreadProfileUniform, GreadUniform, GwritePerTarget,
    GwriteUniform,
};
use osc_protocol::wire::{Id, Inst, MgmtOp, Opcode};

/// What the frame is, from the composite's point of view.
pub enum Decoded<'a> {
    /// Status frame (INST bit 7): a chain predecessor's reply -- snoop it.
    Status,
    /// Not addressed to us (or a group op that doesn't list us): ignore.
    Skip,
    /// Addressed to us: dispatch this request, reply per `ctx`, chain at `slot`.
    Own(Request<'a>, RequestCtx, u8),
}

pub fn decode(frame: FrameBytes<'_>, id: u8) -> Decoded<'_> {
    let (Some(b0), Some(b1), Some(b2), Some(b3)) = (
        frame.u8_at(0),
        frame.u8_at(1),
        frame.u8_at(2),
        frame.u8_at(3),
    ) else {
        return Decoded::Skip;
    };
    let head = [b0, b1, b2, b3];
    let h = Header::from_bytes(&head);
    if h.inst.is_status() {
        return Decoded::Status;
    }
    let Some(op) = h.inst.opcode() else {
        return Decoded::Skip; // framer already rejects opcode 0
    };
    let p = h.payload_len() as usize;
    let Some(pay) = frame.sub(Header::SIZE, p) else {
        return Decoded::Skip;
    };
    let inst = h.inst;
    let own = Id::new(id);

    match op {
        Opcode::Gread => gread(inst, pay, own),
        Opcode::Gwrite => gwrite(inst, pay, own),
        _ => plain(op, inst, h.id, pay, own),
    }
}

/// PING / READ / WRITE / COMMIT / MGMT: addressed by frame ID, reply at slot 0.
fn plain<'a>(op: Opcode, inst: Inst, frame_id: Id, pay: FrameBytes<'a>, own: Id) -> Decoded<'a> {
    let broadcast = frame_id.is_broadcast();
    if !frame_id.addresses(own) {
        return Decoded::Skip;
    }
    let req = match op {
        Opcode::Ping => Request::Ping,
        // READ + PROFILE names a profile slot instead of addr+count (sec 5.2).
        Opcode::Read if inst.profile() => match ProfileReq::parse(pay) {
            Some(r) => Request::ReadProfile { slot: r.slot },
            None => Request::Unsupported,
        },
        Opcode::Read => match ReadReq::parse(pay) {
            Some(r) => Request::Read {
                addr: r.addr,
                count: r.count,
            },
            None => Request::Unsupported,
        },
        Opcode::Write => match WriteReq::parse(pay) {
            Some(w) => Request::Write {
                addr: w.addr,
                data: w.data,
                hold: inst.hold(),
            },
            None => Request::Unsupported,
        },
        Opcode::Commit => Request::Commit,
        Opcode::Mgmt => match MgmtReq::parse(pay) {
            Some(m) => match m.op {
                MgmtOp::Enum => match EnumReq::parse(m.args) {
                    Some(e) => Request::Enumerate {
                        prefix_len: e.prefix_len,
                        prefix: e.prefix,
                    },
                    None => Request::Unsupported,
                },
                MgmtOp::Assign => match AssignReq::parse(m.args) {
                    Some(a) => Request::Assign {
                        uid: a.uid,
                        new_id: a.new_id,
                    },
                    None => Request::Unsupported,
                },
                // sec 9.3: broadcast-only -- a unicast CAL's ack would put our
                // own break on the wire right where the train starts.
                MgmtOp::Cal => match CalReq::parse(m.args) {
                    Some(c) if broadcast => Request::Calibrate {
                        gap_us: c.gap_us,
                        gaps: c.gaps,
                    },
                    _ => Request::Unsupported,
                },
                _ => Request::Mgmt {
                    op: m.op,
                    args: m.args,
                },
            },
            None => Request::Unsupported,
        },
        // Group opcodes are routed before this call.
        _ => Request::Unsupported,
    };
    // sec 5: a broadcast reply would collide on the shared push-pull wire, so
    // plain ops answer only unicast; NOREPLY suppresses either way. ENUM and
    // ASSIGN are the sec 9.2 carve-out: broadcast-addressed by design, with the
    // dispatcher's UID match electing the sole replier -- a malformed pair
    // decodes Unsupported and stays under broadcast suppression.
    let may_reply = !inst.noreply()
        && (!broadcast || matches!(req, Request::Enumerate { .. } | Request::Assign { .. }));
    Decoded::Own(req, RequestCtx { may_reply }, 0)
}

/// GREAD: our span comes from the id-list; the chain slot is our list position.
/// A GREAD slot always replies (that is the chain, sec 6). A list we can't parse
/// leaves our addressing unknowable -- skip rather than answer blind. With the
/// PROFILE flag the payload names profile slots instead of spans (sec 5.2).
fn gread<'a>(inst: Inst, pay: FrameBytes<'a>, own: Id) -> Decoded<'a> {
    match (inst.profile(), inst.per_target()) {
        (false, true) => match GreadPerTarget::parse(pay).and_then(|g| g.find(own)) {
            Some((slot, t)) => own_read(t.addr, t.count, slot),
            None => Decoded::Skip,
        },
        (false, false) => match GreadUniform::parse(pay) {
            Some(g) => match g.slot_of(own) {
                Some(slot) => own_read(g.addr, g.count, slot),
                None => Decoded::Skip,
            },
            None => Decoded::Skip,
        },
        (true, true) => match GreadProfilePerTarget::parse(pay).and_then(|g| g.find(own)) {
            Some((slot, t)) => own_read_profile(t.slot, slot),
            None => Decoded::Skip,
        },
        (true, false) => match GreadProfileUniform::parse(pay) {
            Some(g) => match g.slot_of(own) {
                Some(slot) => own_read_profile(g.slot, slot),
                None => Decoded::Skip,
            },
            None => Decoded::Skip,
        },
    }
}

fn own_read<'a>(addr: u16, count: u16, slot: u8) -> Decoded<'a> {
    Decoded::Own(
        Request::Read { addr, count },
        RequestCtx { may_reply: true },
        slot,
    )
}

fn own_read_profile<'a>(profile: u8, slot: u8) -> Decoded<'a> {
    Decoded::Own(
        Request::ReadProfile { slot: profile },
        RequestCtx { may_reply: true },
        slot,
    )
}

/// GWRITE: our entry's span + data, applied silently (sec 5: no reply implied).
fn gwrite<'a>(inst: Inst, pay: FrameBytes<'a>, own: Id) -> Decoded<'a> {
    let hold = inst.hold();
    if inst.per_target() {
        match GwritePerTarget::parse(pay).and_then(|g| g.find(own)) {
            Some(t) => own_write(t.addr, t.data, hold),
            None => Decoded::Skip,
        }
    } else {
        match GwriteUniform::parse(pay) {
            Some(g) => match g.find(own) {
                Some(data) => own_write(g.addr, data, hold),
                None => Decoded::Skip,
            },
            None => Decoded::Skip,
        }
    }
}

fn own_write<'a>(addr: u16, data: FrameBytes<'a>, hold: bool) -> Decoded<'a> {
    Decoded::Own(
        Request::Write { addr, data, hold },
        RequestCtx { may_reply: false },
        0,
    )
}
