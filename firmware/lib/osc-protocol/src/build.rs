//! Instruction payload builders -- the encode mirror of the `frame`/`group`
//! parsers (`docs/osc-native-protocol.md` sec 5, sec 9). Hosts and tests
//! write payloads through these into a caller buffer (typically
//! [`FrameBuf::payload_mut`](crate::reply::FrameBuf::payload_mut)); the
//! returned length feeds `finish`.
//!
//! The contract is parse-level validity, exactly: a built payload always
//! round-trips through its parser, and anything the parser would reject
//! (empty id list, zero CAL fields, ragged geometry, the sec 5.1 ceiling)
//! refuses to build. Semantic validity -- id ranges, table addressing --
//! stays the responder's verdict, reachable on the wire by construction.

use crate::group::{GreadProfileTarget, GreadTarget};
use crate::wire::{self, Id, MgmtOp};

/// Bounds every builder: the destination window is the smaller of the buffer
/// and the sec 5.1 payload ceiling.
#[inline]
fn cap(dst: &[u8]) -> usize {
    dst.len().min(wire::MAX_PAYLOAD as usize)
}

/// READ / GREAD span head: `addr(2) count(2)` little-endian.
#[inline]
pub fn read(dst: &mut [u8], addr: u16, count: u16) -> Option<usize> {
    if cap(dst) < 4 {
        return None;
    }
    dst[..2].copy_from_slice(&addr.to_le_bytes());
    dst[2..4].copy_from_slice(&count.to_le_bytes());
    Some(4)
}

/// READ+PROFILE: `slot(1)` (sec 5.2).
#[inline]
pub fn read_profile(dst: &mut [u8], slot: u8) -> Option<usize> {
    if cap(dst) < 1 {
        return None;
    }
    dst[0] = slot;
    Some(1)
}

/// WRITE: `addr(2)` little-endian, then the data bytes.
#[inline]
pub fn write(dst: &mut [u8], addr: u16, data: &[u8]) -> Option<usize> {
    let total = 2 + data.len();
    if cap(dst) < total {
        return None;
    }
    dst[..2].copy_from_slice(&addr.to_le_bytes());
    dst[2..total].copy_from_slice(data);
    Some(total)
}

/// Uniform GREAD: `addr(2) count(2) id-list(1 each)`.
pub fn gread_uniform(dst: &mut [u8], addr: u16, count: u16, ids: &[Id]) -> Option<usize> {
    let total = 4 + ids.len();
    if ids.is_empty() || cap(dst) < total {
        return None;
    }
    read(dst, addr, count)?;
    for (b, id) in dst[4..total].iter_mut().zip(ids) {
        *b = id.as_byte();
    }
    Some(total)
}

/// PER_TARGET GREAD: `[id(1) addr(2) count(2)]x`.
pub fn gread_per_target(dst: &mut [u8], targets: &[GreadTarget]) -> Option<usize> {
    let total = 5 * targets.len();
    if targets.is_empty() || cap(dst) < total {
        return None;
    }
    for (chunk, t) in dst[..total].as_chunks_mut::<5>().0.iter_mut().zip(targets) {
        chunk[0] = t.id.as_byte();
        chunk[1..3].copy_from_slice(&t.addr.to_le_bytes());
        chunk[3..5].copy_from_slice(&t.count.to_le_bytes());
    }
    Some(total)
}

/// Uniform GREAD+PROFILE: `slot(1) id-list(1 each)` (sec 5.2).
pub fn gread_profile_uniform(dst: &mut [u8], slot: u8, ids: &[Id]) -> Option<usize> {
    let total = 1 + ids.len();
    if ids.is_empty() || cap(dst) < total {
        return None;
    }
    dst[0] = slot;
    for (b, id) in dst[1..total].iter_mut().zip(ids) {
        *b = id.as_byte();
    }
    Some(total)
}

/// PER_TARGET GREAD+PROFILE: `[id(1) slot(1)]x`.
pub fn gread_profile_per_target(dst: &mut [u8], targets: &[GreadProfileTarget]) -> Option<usize> {
    let total = 2 * targets.len();
    if targets.is_empty() || cap(dst) < total {
        return None;
    }
    for (chunk, t) in dst[..total].as_chunks_mut::<2>().0.iter_mut().zip(targets) {
        chunk[0] = t.id.as_byte();
        chunk[1] = t.slot;
    }
    Some(total)
}

/// Uniform GWRITE: `addr(2) count(1)` head, then [`push`](Self::push) per
/// `[id(1) data(count)]` entry. Entry data length is pinned to `count`.
pub struct GwriteUniform<'a> {
    dst: &'a mut [u8],
    len: usize,
    count: u8,
}

impl<'a> GwriteUniform<'a> {
    pub fn new(dst: &'a mut [u8], addr: u16, count: u8) -> Option<Self> {
        if count == 0 || cap(dst) < 3 {
            return None;
        }
        dst[..2].copy_from_slice(&addr.to_le_bytes());
        dst[2] = count;
        Some(Self { dst, len: 3, count })
    }

    pub fn push(&mut self, id: Id, data: &[u8]) -> Option<()> {
        if data.len() != self.count as usize {
            return None;
        }
        let end = self.len + 1 + data.len();
        if cap(self.dst) < end {
            return None;
        }
        self.dst[self.len] = id.as_byte();
        self.dst[self.len + 1..end].copy_from_slice(data);
        self.len = end;
        Some(())
    }

    /// The payload length; `None` when no entry was pushed (the parser
    /// rejects a bare head).
    pub fn finish(self) -> Option<usize> {
        (self.len > 3).then_some(self.len)
    }
}

/// PER_TARGET GWRITE: [`push`](Self::push) per `[id(1) addr(2) count(1)
/// data(count)]` entry, each entry's data sizing itself.
pub struct GwritePerTarget<'a> {
    dst: &'a mut [u8],
    len: usize,
}

impl<'a> GwritePerTarget<'a> {
    pub fn new(dst: &'a mut [u8]) -> Self {
        Self { dst, len: 0 }
    }

    pub fn push(&mut self, id: Id, addr: u16, data: &[u8]) -> Option<()> {
        if data.len() > u8::MAX as usize {
            return None;
        }
        let end = self.len + 4 + data.len();
        if cap(self.dst) < end {
            return None;
        }
        let e = &mut self.dst[self.len..end];
        e[0] = id.as_byte();
        e[1..3].copy_from_slice(&addr.to_le_bytes());
        e[3] = data.len() as u8;
        e[4..].copy_from_slice(data);
        self.len = end;
        Some(())
    }

    /// The payload length; `None` when no entry was pushed.
    pub fn finish(self) -> Option<usize> {
        (self.len > 0).then_some(self.len)
    }
}

/// Bare MGMT payload: `sub-op(1)`, no args (SAVE / REBOOT / FACTORY, sec 9.4,
/// sec 9.5).
#[inline]
pub fn mgmt(dst: &mut [u8], op: MgmtOp) -> Option<usize> {
    if cap(dst) < 1 {
        return None;
    }
    dst[0] = op as u8;
    Some(1)
}

/// MGMT ENUM: `sub-op(1) prefix_len(1)` in bits (0..=128), then the
/// `ceil(prefix_len/8)`-byte LSB-first prefix (sec 9.2).
pub fn mgmt_enum(dst: &mut [u8], prefix_len: u8, prefix: &[u8; wire::UID_LEN]) -> Option<usize> {
    if prefix_len > (wire::UID_LEN * 8) as u8 {
        return None;
    }
    let n = prefix_len.div_ceil(8) as usize;
    let total = 2 + n;
    if cap(dst) < total {
        return None;
    }
    dst[0] = MgmtOp::Enum as u8;
    dst[1] = prefix_len;
    dst[2..total].copy_from_slice(&prefix[..n]);
    Some(total)
}

/// MGMT ASSIGN: `sub-op(1) uid(16) new_id(1)` (sec 9.2). `new_id` range is the
/// matcher's verdict (a nackable `validation`), not a build gate.
pub fn mgmt_assign(dst: &mut [u8], uid: &[u8; wire::UID_LEN], new_id: u8) -> Option<usize> {
    let total = 2 + wire::UID_LEN;
    if cap(dst) < total {
        return None;
    }
    dst[0] = MgmtOp::Assign as u8;
    dst[1..1 + wire::UID_LEN].copy_from_slice(uid);
    dst[1 + wire::UID_LEN] = new_id;
    Some(total)
}

/// MGMT CAL: `sub-op(1) gap_us(2 LE) gaps(1)` (sec 9.3). Zero in either field
/// is no train -- rejected here as the parser rejects it.
pub fn mgmt_cal(dst: &mut [u8], gap_us: u16, gaps: u8) -> Option<usize> {
    if gap_us == 0 || gaps == 0 || cap(dst) < 4 {
        return None;
    }
    dst[0] = MgmtOp::Cal as u8;
    dst[1..3].copy_from_slice(&gap_us.to_le_bytes());
    dst[3] = gaps;
    Some(4)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bytes::FrameBytes;
    use crate::frame::{AssignReq, CalReq, EnumReq, MgmtReq, ProfileReq, ReadReq, WriteReq};
    use crate::group;

    fn fb(b: &[u8]) -> FrameBytes<'_> {
        FrameBytes::from(b)
    }

    #[test]
    fn read_round_trips_and_matches_vector() {
        let mut b = [0u8; 8];
        let n = read(&mut b, 0x0200, 8).unwrap();
        // The sec 3.2 READ test vector's payload bytes.
        assert_eq!(&b[..n], &[0x00, 0x02, 0x08, 0x00]);
        let r = ReadReq::parse(fb(&b[..n])).unwrap();
        assert_eq!((r.addr, r.count), (0x0200, 8));
    }

    #[test]
    fn read_profile_round_trips() {
        let mut b = [0u8; 4];
        let n = read_profile(&mut b, 2).unwrap();
        assert_eq!(ProfileReq::parse(fb(&b[..n])).unwrap().slot, 2);
    }

    #[test]
    fn write_round_trips_and_matches_vector() {
        let mut b = [0u8; 16];
        let n = write(&mut b, 0x0180, &[0x2C, 0x01]).unwrap();
        // The sec 3.2 WRITE test vector's payload bytes.
        assert_eq!(&b[..n], &[0x80, 0x01, 0x2C, 0x01]);
        let w = WriteReq::parse(fb(&b[..n])).unwrap();
        assert_eq!(w.addr, 0x0180);
        assert_eq!(w.data, fb(&[0x2C, 0x01]));
    }

    #[test]
    fn gread_uniform_round_trips() {
        let mut b = [0u8; 16];
        let ids = [Id::new(1), Id::new(2), Id::new(5)];
        let n = gread_uniform(&mut b, 0x0084, 8, &ids).unwrap();
        let g = group::GreadUniform::parse(fb(&b[..n])).unwrap();
        assert_eq!((g.addr, g.count), (0x0084, 8));
        assert!(g.ids().eq(ids));
        assert_eq!(gread_uniform(&mut b, 0x0084, 8, &[]), None);
    }

    #[test]
    fn gread_per_target_round_trips() {
        let mut b = [0u8; 16];
        let ts = [
            GreadTarget {
                id: Id::new(10),
                addr: 0x0200,
                count: 4,
            },
            GreadTarget {
                id: Id::new(20),
                addr: 0x0010,
                count: 2,
            },
        ];
        let n = gread_per_target(&mut b, &ts).unwrap();
        let g = group::GreadPerTarget::parse(fb(&b[..n])).unwrap();
        assert!(g.iter().eq(ts));
        assert_eq!(gread_per_target(&mut b, &[]), None);
    }

    #[test]
    fn gread_profile_forms_round_trip() {
        let mut b = [0u8; 16];
        let ids = [Id::new(1), Id::new(5)];
        let n = gread_profile_uniform(&mut b, 2, &ids).unwrap();
        let g = group::GreadProfileUniform::parse(fb(&b[..n])).unwrap();
        assert_eq!(g.slot, 2);
        assert!(g.ids().eq(ids));

        let ts = [
            GreadProfileTarget {
                id: Id::new(10),
                slot: 0,
            },
            GreadProfileTarget {
                id: Id::new(20),
                slot: 3,
            },
        ];
        let n = gread_profile_per_target(&mut b, &ts).unwrap();
        let g = group::GreadProfilePerTarget::parse(fb(&b[..n])).unwrap();
        assert!(g.iter().eq(ts));
    }

    #[test]
    fn gwrite_uniform_round_trips() {
        let mut b = [0u8; 32];
        let mut w = GwriteUniform::new(&mut b, 0x0180, 4).unwrap();
        w.push(Id::new(1), &[1, 2, 3, 4]).unwrap();
        w.push(Id::new(2), &[5, 6, 7, 8]).unwrap();
        let n = w.finish().unwrap();
        let g = group::GwriteUniform::parse(fb(&b[..n])).unwrap();
        assert_eq!((g.addr, g.count), (0x0180, 4));
        assert_eq!(g.find(Id::new(2)), Some(fb(&[5, 6, 7, 8])));
    }

    #[test]
    fn gwrite_uniform_gates_mirror_the_parser() {
        let mut b = [0u8; 32];
        // count 0 has no stride; wrong-length data breaks it; a bare head
        // would parse as ragged/empty.
        assert!(GwriteUniform::new(&mut b, 0, 0).is_none());
        let mut w = GwriteUniform::new(&mut b, 0, 4).unwrap();
        assert!(w.push(Id::new(1), &[1, 2]).is_none());
        assert!(w.finish().is_none());
    }

    #[test]
    fn gwrite_per_target_round_trips() {
        let mut b = [0u8; 32];
        let mut w = GwritePerTarget::new(&mut b);
        w.push(Id::new(3), 0x0000, &[0xAA, 0xBB]).unwrap();
        w.push(Id::new(4), 0x0100, &[1, 2, 3, 4]).unwrap();
        let n = w.finish().unwrap();
        let g = group::GwritePerTarget::parse(fb(&b[..n])).unwrap();
        assert_eq!(g.find(Id::new(4)).unwrap().data, fb(&[1, 2, 3, 4]));
        assert!(GwritePerTarget::new(&mut b).finish().is_none());
    }

    #[test]
    fn mgmt_forms_round_trip() {
        let mut b = [0u8; 24];

        let n = mgmt(&mut b, MgmtOp::Save).unwrap();
        assert_eq!(MgmtReq::parse(fb(&b[..n])).unwrap().op, MgmtOp::Save);

        let mut prefix = [0u8; wire::UID_LEN];
        prefix[0] = 0b0000_0101;
        let n = mgmt_enum(&mut b, 3, &prefix).unwrap();
        let m = MgmtReq::parse(fb(&b[..n])).unwrap();
        let e = EnumReq::parse(m.args).unwrap();
        assert_eq!(e.prefix_len, 3);
        assert_eq!(e.prefix, prefix);
        assert_eq!(mgmt_enum(&mut b, 129, &prefix), None);

        let uid = [7u8; wire::UID_LEN];
        let n = mgmt_assign(&mut b, &uid, 9).unwrap();
        let a = AssignReq::parse(MgmtReq::parse(fb(&b[..n])).unwrap().args).unwrap();
        assert_eq!((a.uid, a.new_id), (uid, 9));

        let n = mgmt_cal(&mut b, 400, 8).unwrap();
        let c = CalReq::parse(MgmtReq::parse(fb(&b[..n])).unwrap().args).unwrap();
        assert_eq!((c.gap_us, c.gaps), (400, 8));
        assert_eq!(mgmt_cal(&mut b, 0, 8), None);
        assert_eq!(mgmt_cal(&mut b, 400, 0), None);
    }

    #[test]
    fn builders_hold_the_payload_ceiling() {
        // 250 ids would build a 254 B payload -- past the sec 5.1 ceiling even
        // in a roomier buffer.
        let mut b = [0u8; 300];
        let ids = [Id::new(1); 250];
        assert_eq!(gread_uniform(&mut b, 0, 2, &ids), None);
        // One shy of the full unicast space fits: 4 + 248 = 252.
        assert_eq!(gread_uniform(&mut b, 0, 2, &ids[..248]), Some(252));
    }
}
