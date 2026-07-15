//! MGMT choreographies: client policy over engine evidence (the walk and
//! its verdicts live here by phase-2 ruling -- the engine only ships
//! WireEvidence). Register-touching halves (set_baud, cal_verify) name only
//! the sec 5.4 common block; SAVE has no client-side guard by design (the
//! servo self-gates and answers `access`).

use osc_protocol::build;
use osc_protocol::table;
use osc_protocol::wire::{BaudRate, Id, Inst, MgmtOp, Opcode, ResultCode, UID_LEN};

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

async fn settle<P: Pipe>(c: &mut Client<P>) {
    c.pause(PROBE_SETTLE).await;
}

fn with_bit(prefix: &[u8; UID_LEN], k: u8, v: bool) -> [u8; UID_LEN] {
    let mut p = *prefix;
    if v {
        p[k as usize / 8] |= 1 << (k % 8);
    }
    p
}

/// One discovered node: the UID that answered and the id it answered from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Found {
    pub uid: Uid,
    pub id: Id,
}

enum Verdict {
    /// One clean status carrying a full UID; unconfirmed until its children
    /// agree (superimposed twins can read back as one clean frame).
    Candidate(Found),
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
                Verdict::Candidate(Found {
                    uid: Uid(uid),
                    id: Id::new(s.id),
                })
            } else {
                Verdict::Collision
            }
        }
        _ => Verdict::Collision,
    };
    if matches!(verdict, Verdict::Collision) {
        settle(c).await;
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
        settle(c).await;
        verdict = probe(c, len, prefix).await?;
    }
    Ok(verdict)
}

/// The protocol sec 9.2 prefix walk: O(bits x N) probes plus two confirm
/// probes per servo. A full-depth collision verdict is retried (slot draws
/// re-roll per probe); a leaf that never resolves is skipped rather than
/// failing the walk.
pub async fn discover<P: Pipe>(c: &mut Client<P>) -> Result<Vec<Found>, Error> {
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
        c.pause(4 * PROBE_SETTLE).await;
        let next = walk(c).await?;
        if next == prev {
            return Ok(next);
        }
        prev = next;
    }
    Ok(prev)
}

async fn walk<P: Pipe>(c: &mut Client<P>) -> Result<Vec<Found>, Error> {
    const FULL: u8 = (UID_LEN * 8) as u8;
    const FULL_DEPTH_RETRIES: usize = 3;
    let mut found: Vec<Found> = Vec::new();
    let mut stack = vec![(0u8, [0u8; UID_LEN])];
    while let Some((len, prefix)) = stack.pop() {
        match probe_settled(c, len, &prefix).await? {
            Verdict::Empty => {}
            Verdict::Candidate(node) if len == FULL => found.push(node),
            Verdict::Candidate(node) => {
                // Confirm: the uid's own child must re-elect it and the
                // sibling must be empty; anything else means the clean frame
                // was superimposed twins -- descend normally.
                let b = bit(&node.uid.0, len);
                let own = probe_settled(c, len + 1, &with_bit(&prefix, len, b)).await?;
                let sib = probe_settled(c, len + 1, &with_bit(&prefix, len, !b)).await?;
                match (own, sib) {
                    (Verdict::Candidate(n2), Verdict::Empty) if n2.uid == node.uid => {
                        found.push(n2)
                    }
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
                    if let Verdict::Candidate(node) = probe(c, len, &prefix).await? {
                        found.push(node);
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
    found.sort_by_key(|f| f.uid);
    Ok(found)
}

/// Whether anything answers a root ENUM probe at the current rate -- a
/// collision counts: garble still means servos are present. Silence is
/// re-probed settled (the walk's hardening) before trusting it.
pub async fn bus_present<P: Pipe>(c: &mut Client<P>) -> Result<bool, Error> {
    Ok(!matches!(
        probe_settled(c, 0, &[0u8; UID_LEN]).await?,
        Verdict::Empty
    ))
}

/// Find the bus rate by ENUM probe: the 1M protocol default first, then the
/// remaining rates ascending. Leaves the host at the found rate (at the last
/// probed rate when nothing answered).
pub async fn find_bus_baud<P: Pipe>(c: &mut Client<P>) -> Result<Option<BaudRate>, Error> {
    const ORDER: [BaudRate; 4] = [
        BaudRate::B1000000,
        BaudRate::B500000,
        BaudRate::B2000000,
        BaudRate::B3000000,
    ];
    for rate in ORDER {
        c.host_baud(rate).await?;
        if bus_present(c).await? {
            return Ok(Some(rate));
        }
    }
    Ok(None)
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
/// second train finishes weak-step chips), blind: no readback. See
/// [`cal_verify`] for the per-servo convergence trace.
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
    ping_sweep(c, ids).await
}

/// Fleet baud migration, servo-first (protocol sec 8): every target acks
/// the register write at the old rate and applies it deferred, then the
/// host follows, then the ping sweep reports the reunion.
pub async fn set_baud<P: Pipe>(
    c: &mut Client<P>,
    ids: &[Id],
    rate: BaudRate,
) -> Result<Vec<(Id, bool)>, Error> {
    for &id in ids {
        match c.write(id, table::BAUD_RATE_IDX, &[rate.as_idx()]).await {
            Ok(()) => {}
            // Indeterminate: the servo may have applied with the ack lost
            // (silicon-observed), and a retry at the old rate can only
            // garble an applied switch -- the reunion sweep is the verdict.
            Err(Error::Timeout { .. }) => {}
            Err(e) => return Err(e),
        }
    }
    c.host_baud(rate).await?;
    // sec 8 pacing: the write burst was cross-rate garble to every servo
    // that already applied -- one settle of quiet lets parked resolvers
    // starve out before the sweep asks for proof of life.
    settle(c).await;
    ping_sweep(c, ids).await
}

/// Per-servo trim trace from a CAL run: `trims[k]` is the applied total
/// after train k+1.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CalTrace {
    pub id: Id,
    pub trims: Vec<i8>,
}

impl CalTrace {
    /// Converged = the last two trains agreed (protocol sec 9.3: a train
    /// that moves nothing has nothing left to move).
    pub fn converged(&self) -> bool {
        matches!(self.trims.as_slice(), [.., a, b] if a == b)
    }
}

/// CAL with convergence readback: one broadcast train at a time, each
/// listed servo's `trim_steps` sampled after every train.
pub async fn cal_verify<P: Pipe>(
    c: &mut Client<P>,
    ids: &[Id],
    trains: u8,
    gap_us: u16,
    gaps: u8,
) -> Result<Vec<CalTrace>, Error> {
    let mut traces: Vec<CalTrace> = ids
        .iter()
        .map(|&id| CalTrace {
            id,
            trims: Vec::with_capacity(trains as usize),
        })
        .collect();
    for _ in 0..trains {
        cal(c, 1, gap_us, gaps).await?;
        for trace in &mut traces {
            let b = c.read(trace.id, table::TRIM_STEPS, 1).await?;
            trace.trims.push(*b.first().unwrap_or(&0) as i8);
        }
    }
    Ok(traces)
}

async fn ping_sweep<P: Pipe>(c: &mut Client<P>, ids: &[Id]) -> Result<Vec<(Id, bool)>, Error> {
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
