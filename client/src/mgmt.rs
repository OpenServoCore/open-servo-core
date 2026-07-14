//! MGMT choreographies: client policy over engine evidence (the walk and
//! its verdicts live here by phase-2 ruling -- the engine only ships
//! WireEvidence). Everything address-free; the register-touching halves
//! (set_baud, CAL verify, SAVE guard) arrive with the common-block consts.

use osc_protocol::build;
use osc_protocol::wire::{Id, Inst, MgmtOp, Opcode, ResultCode, UID_LEN};

use crate::client::{Client, Reply};
use crate::error::Error;
use crate::pipe::Pipe;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Uid(pub [u8; UID_LEN]);

impl std::fmt::Debug for Uid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for b in self.0.iter().rev() {
            write!(f, "{b:02x}")?;
        }
        Ok(())
    }
}

/// LSB-first bit order, protocol sec 9.2: bit k of the UID stream is
/// `uid[k/8] >> (k%8) & 1` -- the order the UART shifts onto the wire.
fn bit(uid: &[u8; UID_LEN], k: u8) -> bool {
    uid[k as usize / 8] >> (k % 8) & 1 != 0
}

fn matches_prefix(uid: &[u8; UID_LEN], prefix: &[u8; UID_LEN], len: u8) -> bool {
    (0..len).all(|k| bit(uid, k) == bit(prefix, k))
}

/// Post-collision bus recovery (protocol sec 8 host-pacing rule): after a
/// probe drew superimposed replies, every servo's resolver may be parked on
/// the wire-AND junk; the engine paces exactly one starve horizon, which is
/// a boundary race against the servos' own horizon (fleet-measured: probes
/// fired straight through it get framing-dropped). The horizon at the
/// slowest rate is 64 byte-times at 0.5M = 1.28 ms; the extra margin is
/// fleet-measured (2 ms still lost occasional subtrees mid-walk).
const PROBE_SETTLE: std::time::Duration = std::time::Duration::from_millis(5);

async fn settle() {
    futures_timer::Delay::new(PROBE_SETTLE).await;
}

fn with_bit(prefix: &[u8; UID_LEN], k: u8, v: bool) -> [u8; UID_LEN] {
    let mut p = *prefix;
    if v {
        p[k as usize / 8] |= 1 << (k % 8);
    }
    p
}

enum Verdict {
    /// One clean status carrying a full UID; unconfirmed until its children
    /// agree (superimposed twins can read back as one clean frame).
    Candidate(Uid),
    /// Garble, trailing energy, or multiple statuses: descend.
    Collision,
    /// Quiet subtree.
    Empty,
}

async fn probe<P: Pipe>(
    c: &mut Client<P>,
    len: u8,
    prefix: &[u8; UID_LEN],
) -> Result<Verdict, Error> {
    let mut p = [0u8; 2 + UID_LEN];
    let n = build::mgmt_enum(&mut p, len, prefix).expect("prefix_len bounded by the walk");
    let inst = Inst::instruction(Opcode::Mgmt, 0);
    let reply = c.exchange(Id::BROADCAST, inst, &p[..n]).await?;
    let clean = reply.garble == 0 && !reply.trailing;
    let verdict = match (reply.statuses.as_slice(), clean) {
        ([], true) => Verdict::Empty,
        ([s], true) if s.result == Some(ResultCode::Ok) && s.payload.len() == UID_LEN => {
            let uid: [u8; UID_LEN] = s.payload[..].try_into().unwrap();
            // A reply that does not extend the probe's own prefix is
            // superposition garbage wearing a clean CRC, not a matcher.
            if matches_prefix(&uid, prefix, len) {
                Verdict::Candidate(Uid(uid))
            } else {
                Verdict::Collision
            }
        }
        _ => Verdict::Collision,
    };
    if matches!(verdict, Verdict::Collision) {
        settle().await;
    }
    Ok(verdict)
}

/// Empty is the dangerous verdict (a parked fleet reads as silence): a
/// subtree prunes only after repeated settled re-probes stay quiet.
async fn probe_settled<P: Pipe>(
    c: &mut Client<P>,
    len: u8,
    prefix: &[u8; UID_LEN],
) -> Result<Verdict, Error> {
    const EMPTY_RETRIES: usize = 2;
    let mut verdict = probe(c, len, prefix).await?;
    for _ in 0..EMPTY_RETRIES {
        if !matches!(verdict, Verdict::Empty) {
            return Ok(verdict);
        }
        settle().await;
        verdict = probe(c, len, prefix).await?;
    }
    Ok(verdict)
}

/// The protocol sec 9.2 prefix walk: O(bits x N) probes plus two confirm
/// probes per servo. A full-depth collision verdict is retried (slot draws
/// re-roll per probe); a leaf that never resolves is skipped rather than
/// failing the walk.
pub async fn discover<P: Pipe>(c: &mut Client<P>) -> Result<Vec<Uid>, Error> {
    // Roster-stability contract: two consecutive walks must agree. A walk
    // can only UNDERCOUNT (a parked fleet reads as silence; phantoms die at
    // the confirm probes), and the mute occasionally swallows subtrees
    // mid-walk (fleet-measured ~1 walk in 6) -- agreement across walks is
    // what makes the roster trustworthy. The cap bounds a hostile bus; the
    // last walk returns regardless, and a caller who suspects more nodes
    // re-runs.
    const WALKS_CAP: usize = 5;
    let mut prev = walk(c).await?;
    for _ in 1..WALKS_CAP {
        futures_timer::Delay::new(4 * PROBE_SETTLE).await;
        let next = walk(c).await?;
        if next == prev {
            return Ok(next);
        }
        prev = next;
    }
    Ok(prev)
}

async fn walk<P: Pipe>(c: &mut Client<P>) -> Result<Vec<Uid>, Error> {
    const FULL: u8 = (UID_LEN * 8) as u8;
    const FULL_DEPTH_RETRIES: usize = 3;
    let mut found = Vec::new();
    let mut stack = vec![(0u8, [0u8; UID_LEN])];
    while let Some((len, prefix)) = stack.pop() {
        match probe_settled(c, len, &prefix).await? {
            Verdict::Empty => {}
            Verdict::Candidate(uid) if len == FULL => found.push(uid),
            Verdict::Candidate(uid) => {
                // Confirm: the uid's own child must re-elect it and the
                // sibling must be empty; anything else means the clean frame
                // was superimposed twins -- descend normally.
                let b = bit(&uid.0, len);
                let own = probe_settled(c, len + 1, &with_bit(&prefix, len, b)).await?;
                let sib = probe_settled(c, len + 1, &with_bit(&prefix, len, !b)).await?;
                match (own, sib) {
                    (Verdict::Candidate(u2), Verdict::Empty) if u2 == uid => found.push(uid),
                    _ => {
                        stack.push((len + 1, with_bit(&prefix, len, false)));
                        stack.push((len + 1, with_bit(&prefix, len, true)));
                    }
                }
            }
            Verdict::Collision if len == FULL => {
                // Identical UIDs are impossible; this is garble luck. Slot
                // draws re-roll per probe, so retry before giving up.
                let mut resolved = false;
                for _ in 0..FULL_DEPTH_RETRIES {
                    if let Verdict::Candidate(uid) = probe(c, len, &prefix).await? {
                        found.push(uid);
                        resolved = true;
                        break;
                    }
                }
                if !resolved {
                    // Skip the leaf; the caller sees a shorter roster and
                    // re-runs discovery rather than trusting a garbled UID.
                }
            }
            Verdict::Collision => {
                stack.push((len + 1, with_bit(&prefix, len, false)));
                stack.push((len + 1, with_bit(&prefix, len, true)));
            }
        }
    }
    found.sort();
    Ok(found)
}

/// Broadcast ASSIGN: the sole UID matcher takes `new_id` and acks from it.
pub async fn assign<P: Pipe>(c: &mut Client<P>, uid: &Uid, new_id: Id) -> Result<(), Error> {
    let mut p = [0u8; 2 + UID_LEN + 1];
    let n = build::mgmt_assign(&mut p, &uid.0, new_id.as_byte()).expect("assign payload fits");
    let inst = Inst::instruction(Opcode::Mgmt, 0);
    let reply = c.exchange(Id::BROADCAST, inst, &p[..n]).await?;
    expect_ok_from(&reply, new_id)
}

/// Broadcast CAL trains (>= 2 per protocol sec 9.3 boot guidance: the
/// second train finishes weak-step chips). Convergence readback arrives
/// with the common-block consts.
pub async fn cal<P: Pipe>(
    c: &mut Client<P>,
    trains: u8,
    gap_us: u16,
    gaps: u8,
) -> Result<(), Error> {
    let inst = Inst::instruction(Opcode::Mgmt, 0);
    for _ in 0..trains {
        let mut p = [0u8; 8];
        let n = build::mgmt_cal(&mut p, gap_us, gaps).expect("cal payload fits");
        c.exchange(Id::BROADCAST, inst, &p[..n]).await?;
    }
    Ok(())
}

/// MGMT SAVE: the servo gates it itself (torque on answers `access`), so
/// there is no client-side pre-check by design.
pub async fn save<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<(), Error> {
    mgmt_op(c, id, MgmtOp::Save).await
}

pub async fn reboot<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<(), Error> {
    mgmt_op(c, id, MgmtOp::Reboot).await
}

pub async fn factory<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<(), Error> {
    mgmt_op(c, id, MgmtOp::Factory).await
}

async fn mgmt_op<P: Pipe>(c: &mut Client<P>, id: Id, op: MgmtOp) -> Result<(), Error> {
    let mut p = [0u8; 2];
    let n = build::mgmt(&mut p, op).expect("bare mgmt payload fits");
    let inst = Inst::instruction(Opcode::Mgmt, 0);
    let reply = c.exchange(id, inst, &p[..n]).await?;
    expect_ok_from(&reply, id)
}

/// Rescue the bus, then report who answers at the rescue rate.
pub async fn rescue_sweep<P: Pipe>(
    c: &mut Client<P>,
    ids: &[Id],
) -> Result<Vec<(Id, bool)>, Error> {
    c.rescue().await?;
    let mut roster = Vec::with_capacity(ids.len());
    for &id in ids {
        let alive = match c.ping(id).await {
            Ok(_) => true,
            Err(Error::Timeout { .. }) => false,
            Err(e) => return Err(e),
        };
        roster.push((id, alive));
    }
    Ok(roster)
}

fn expect_ok_from(reply: &Reply, id: Id) -> Result<(), Error> {
    use osc_host::engine::Outcome;
    if let Outcome::Timeout { slot } = reply.outcome {
        return Err(Error::Timeout { slot });
    }
    match reply.statuses.as_slice() {
        [s] if s.id == id.as_byte() => match s.result {
            Some(ResultCode::Ok) => Ok(()),
            Some(code) => Err(Error::Servo(code)),
            None => Err(Error::Link(crate::error::LinkError::Desync(
                "reserved result code".into(),
            ))),
        },
        other => Err(Error::Link(crate::error::LinkError::Desync(format!(
            "expected one status from id {}, got {} statuses",
            id.as_byte(),
            other.len()
        )))),
    }
}
