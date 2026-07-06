//! Linearized frame → decoded request mapping (`docs/osc-native-protocol.md`
//! §5, §6). Input is a whole, CRC-clean frame (header pre-validated by the
//! framer): `[0x00][ID][LEN][INST][payload][PAD?][crc][crc]`. Output is the
//! chip-agnostic [`Request`] plus its reply contract and chain slot, or a
//! classification the composite acts on without dispatching.
//!
//! Addressing: plain ops address by frame ID (unicast-to-us or broadcast);
//! group ops address by their id-lists regardless of the (broadcast) frame ID.
//! Own instruction frames never echo while we are the responder (F9), so no
//! self-filtering is coded here.

use osc_core::traits::{Request, RequestCtx};
use osc_protocol::frame::{Header, MgmtReq, ReadReq, WriteReq};
use osc_protocol::group::{GreadPerTarget, GreadUniform, GwritePerTarget, GwriteUniform};
use osc_protocol::wire::{Id, Inst, Opcode};

/// What the frame is, from the composite's point of view.
pub enum Decoded<'a> {
    /// Status frame (INST bit 7): a chain predecessor's reply — snoop it.
    Status,
    /// Not addressed to us (or a group op that doesn't list us): ignore.
    Skip,
    /// Addressed to us: dispatch this request, reply per `ctx`, chain at `slot`.
    Own(Request<'a>, RequestCtx, u8),
}

pub fn decode(frame: &[u8], id: u8) -> Decoded<'_> {
    let Ok(head) = <&[u8; 4]>::try_from(frame.get(..Header::SIZE).unwrap_or(&[])) else {
        return Decoded::Skip;
    };
    let h = Header::from_bytes(head);
    if h.inst.is_status() {
        return Decoded::Status;
    }
    let Some(op) = h.inst.opcode() else {
        return Decoded::Skip; // framer already rejects opcode 0
    };
    let p = h.payload_len() as usize;
    let Some(pay) = frame.get(Header::SIZE..Header::SIZE + p) else {
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
fn plain<'a>(op: Opcode, inst: Inst, frame_id: Id, pay: &'a [u8], own: Id) -> Decoded<'a> {
    let broadcast = frame_id.is_broadcast();
    if !frame_id.addresses(own) {
        return Decoded::Skip;
    }
    // §5: a broadcast reply would collide on the shared push-pull wire, so
    // plain ops answer only unicast; NOREPLY suppresses either way.
    let may_reply = !inst.noreply() && !broadcast;
    let req = match op {
        Opcode::Ping => Request::Ping,
        // READ + HOLD names a profile slot (§5.2) — deferred, reject cleanly.
        Opcode::Read if inst.hold() => Request::Unsupported,
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
            Some(m) => Request::Mgmt {
                op: m.op,
                args: m.args,
            },
            None => Request::Unsupported,
        },
        // Group opcodes are routed before this call.
        _ => Request::Unsupported,
    };
    Decoded::Own(req, RequestCtx { may_reply }, 0)
}

/// GREAD: our span comes from the id-list; the chain slot is our list position.
/// A GREAD slot always replies (that is the chain, §6). A list we can't parse
/// leaves our addressing unknowable — skip rather than answer blind.
fn gread(inst: Inst, pay: &[u8], own: Id) -> Decoded<'_> {
    if inst.per_target() {
        match GreadPerTarget::parse(pay).and_then(|g| g.find(own)) {
            Some((slot, t)) => own_read(t.addr, t.count, slot),
            None => Decoded::Skip,
        }
    } else {
        match GreadUniform::parse(pay) {
            Some(g) => match g.slot_of(own) {
                Some(slot) => own_read(g.addr, g.count, slot),
                None => Decoded::Skip,
            },
            None => Decoded::Skip,
        }
    }
}

fn own_read<'a>(addr: u16, count: u16, slot: u8) -> Decoded<'a> {
    Decoded::Own(
        Request::Read { addr, count },
        RequestCtx { may_reply: true },
        slot,
    )
}

/// GWRITE: our entry's span + data, applied silently (§5: no reply implied).
fn gwrite(inst: Inst, pay: &[u8], own: Id) -> Decoded<'_> {
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

fn own_write(addr: u16, data: &[u8], hold: bool) -> Decoded<'_> {
    Decoded::Own(
        Request::Write { addr, data, hold },
        RequestCtx { may_reply: false },
        0,
    )
}
