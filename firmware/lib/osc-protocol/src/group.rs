//! Group instruction payload views (`docs/osc-native-protocol.md` §5, §6).
//!
//! GREAD and GWRITE each come in a uniform and a PER_TARGET form. Frames
//! arrive whole and CRC-verified, so `parse` performs all structural
//! validation up front (payloads cap at 252 B, §5.1, so even the
//! variable-stride walk is trivially cheap); the iterators then run over
//! known-well-formed bytes and stay branch-light. A servo's reply slot is its
//! 0-based position in the listed order (§6) — `slot_of` / `find` report it.
//!
//! Duplicate IDs are legal on the wire; first match wins in `find`/`slot_of`.

use crate::bytes::FrameBytes;
use crate::wire::Id;

/// A resolved PER_TARGET GREAD entry: one span request for one servo.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadTarget {
    pub id: Id,
    pub addr: u16,
    pub count: u16,
}

/// A resolved PER_TARGET GWRITE entry: one servo's own span and data.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GwriteTarget<'a> {
    pub id: Id,
    pub addr: u16,
    pub data: FrameBytes<'a>,
}

/// A resolved PER_TARGET GREAD+PROFILE entry: one profile slot for one servo.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadProfileTarget {
    pub id: Id,
    pub slot: u8,
}

/// Uniform GREAD: `addr(2) count(2) id-list(1 each)` — one span, many servos,
/// replies chain in list order.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadUniform<'a> {
    pub addr: u16,
    pub count: u16,
    ids: FrameBytes<'a>,
}

impl<'a> GreadUniform<'a> {
    pub fn parse(payload: FrameBytes<'a>) -> Option<Self> {
        let addr = payload.u16_le_at(0)?;
        let count = payload.u16_le_at(2)?;
        let ids = payload.sub(4, payload.len().checked_sub(4)?)?;
        if ids.is_empty() {
            return None;
        }
        Some(Self { addr, count, ids })
    }

    pub fn ids(&self) -> impl Iterator<Item = Id> + 'a {
        self.ids.bytes().map(Id::new)
    }

    pub fn slot_of(&self, id: Id) -> Option<u8> {
        self.ids
            .bytes()
            .position(|b| b == id.as_byte())
            .map(|p| p as u8)
    }
}

/// PER_TARGET GREAD: `[id(1) addr(2) count(2)]×` — fixed 5-byte entries.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadPerTarget<'a> {
    entries: FrameBytes<'a>,
}

impl<'a> GreadPerTarget<'a> {
    pub fn parse(payload: FrameBytes<'a>) -> Option<Self> {
        if payload.is_empty() || !payload.len().is_multiple_of(5) {
            return None;
        }
        Some(Self { entries: payload })
    }

    pub fn iter(&self) -> GreadPerTargetIter<'a> {
        GreadPerTargetIter { rest: self.entries }
    }

    pub fn find(&self, id: Id) -> Option<(u8, GreadTarget)> {
        self.iter()
            .enumerate()
            .find(|(_, t)| t.id == id)
            .map(|(i, t)| (i as u8, t))
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GreadPerTargetIter<'a> {
    rest: FrameBytes<'a>,
}

impl Iterator for GreadPerTargetIter<'_> {
    type Item = GreadTarget;

    fn next(&mut self) -> Option<GreadTarget> {
        if self.rest.len() < 5 {
            return None;
        }
        let id = self.rest.u8_at(0)?;
        let addr = self.rest.u16_le_at(1)?;
        let count = self.rest.u16_le_at(3)?;
        self.rest = self.rest.sub(5, self.rest.len() - 5)?;
        Some(GreadTarget {
            id: Id::new(id),
            addr,
            count,
        })
    }
}

/// Uniform GREAD+PROFILE: `slot(1) id-list(1 each)` — one profile slot, many
/// servos, replies chain in list order (§5.2).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadProfileUniform<'a> {
    pub slot: u8,
    ids: FrameBytes<'a>,
}

impl<'a> GreadProfileUniform<'a> {
    pub fn parse(payload: FrameBytes<'a>) -> Option<Self> {
        let slot = payload.u8_at(0)?;
        let ids = payload.sub(1, payload.len().checked_sub(1)?)?;
        if ids.is_empty() {
            return None;
        }
        Some(Self { slot, ids })
    }

    pub fn ids(&self) -> impl Iterator<Item = Id> + 'a {
        self.ids.bytes().map(Id::new)
    }

    pub fn slot_of(&self, id: Id) -> Option<u8> {
        self.ids
            .bytes()
            .position(|b| b == id.as_byte())
            .map(|p| p as u8)
    }
}

/// PER_TARGET GREAD+PROFILE: `[id(1) slot(1)]×` — fixed 2-byte entries.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadProfilePerTarget<'a> {
    entries: FrameBytes<'a>,
}

impl<'a> GreadProfilePerTarget<'a> {
    pub fn parse(payload: FrameBytes<'a>) -> Option<Self> {
        if payload.is_empty() || !payload.len().is_multiple_of(2) {
            return None;
        }
        Some(Self { entries: payload })
    }

    pub fn iter(&self) -> GreadProfilePerTargetIter<'a> {
        GreadProfilePerTargetIter { rest: self.entries }
    }

    pub fn find(&self, id: Id) -> Option<(u8, GreadProfileTarget)> {
        self.iter()
            .enumerate()
            .find(|(_, t)| t.id == id)
            .map(|(i, t)| (i as u8, t))
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GreadProfilePerTargetIter<'a> {
    rest: FrameBytes<'a>,
}

impl Iterator for GreadProfilePerTargetIter<'_> {
    type Item = GreadProfileTarget;

    fn next(&mut self) -> Option<GreadProfileTarget> {
        if self.rest.len() < 2 {
            return None;
        }
        let id = self.rest.u8_at(0)?;
        let slot = self.rest.u8_at(1)?;
        self.rest = self.rest.sub(2, self.rest.len() - 2)?;
        Some(GreadProfileTarget {
            id: Id::new(id),
            slot,
        })
    }
}

/// Uniform GWRITE: `addr(2) count(1) [id(1) data(count)]×` — fixed-stride
/// entries after the 3-byte head.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GwriteUniform<'a> {
    pub addr: u16,
    pub count: u8,
    entries: FrameBytes<'a>,
}

impl<'a> GwriteUniform<'a> {
    pub fn parse(payload: FrameBytes<'a>) -> Option<Self> {
        let addr = payload.u16_le_at(0)?;
        let count = payload.u8_at(2)?;
        if count == 0 {
            return None;
        }
        let entries = payload.sub(3, payload.len().checked_sub(3)?)?;
        let stride = 1 + count as usize;
        // Reject a ragged tail without a divide (rv32ec +zmmul has no hardware
        // remainder): walk in stride steps. Bounded — payloads cap at 252 B.
        let mut rem = entries.len();
        if rem == 0 {
            return None;
        }
        while rem >= stride {
            rem -= stride;
        }
        if rem != 0 {
            return None;
        }
        Some(Self {
            addr,
            count,
            entries,
        })
    }

    pub fn iter(&self) -> GwriteUniformIter<'a> {
        GwriteUniformIter {
            rest: self.entries,
            stride: 1 + self.count as usize,
        }
    }

    pub fn find(&self, id: Id) -> Option<FrameBytes<'a>> {
        self.iter().find(|(i, _)| *i == id).map(|(_, d)| d)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GwriteUniformIter<'a> {
    rest: FrameBytes<'a>,
    stride: usize,
}

impl<'a> Iterator for GwriteUniformIter<'a> {
    type Item = (Id, FrameBytes<'a>);

    fn next(&mut self) -> Option<(Id, FrameBytes<'a>)> {
        if self.rest.len() < self.stride {
            return None;
        }
        let id = self.rest.u8_at(0)?;
        let data = self.rest.sub(1, self.stride - 1)?;
        self.rest = self.rest.sub(self.stride, self.rest.len() - self.stride)?;
        Some((Id::new(id), data))
    }
}

/// PER_TARGET GWRITE: `[id(1) addr(2) count(1) data(count)]×` — variable
/// stride, each entry's `count` sizes its own data.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GwritePerTarget<'a> {
    entries: FrameBytes<'a>,
}

impl<'a> GwritePerTarget<'a> {
    pub fn parse(payload: FrameBytes<'a>) -> Option<Self> {
        if payload.is_empty() {
            return None;
        }
        let mut rest = payload;
        while !rest.is_empty() {
            let count = rest.u8_at(3)? as usize;
            let skip = 4 + count;
            rest = rest.sub(skip, rest.len().checked_sub(skip)?)?;
        }
        Some(Self { entries: payload })
    }

    pub fn iter(&self) -> GwritePerTargetIter<'a> {
        GwritePerTargetIter { rest: self.entries }
    }

    pub fn find(&self, id: Id) -> Option<GwriteTarget<'a>> {
        self.iter().find(|t| t.id == id)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GwritePerTargetIter<'a> {
    rest: FrameBytes<'a>,
}

impl<'a> Iterator for GwritePerTargetIter<'a> {
    type Item = GwriteTarget<'a>;

    fn next(&mut self) -> Option<GwriteTarget<'a>> {
        if self.rest.len() < 4 {
            return None;
        }
        let id = self.rest.u8_at(0)?;
        let addr = self.rest.u16_le_at(1)?;
        let count = self.rest.u8_at(3)? as usize;
        let data = self.rest.sub(4, count)?;
        self.rest = self.rest.sub(4 + count, self.rest.len() - (4 + count))?;
        Some(GwriteTarget {
            id: Id::new(id),
            addr,
            data,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fb(b: &[u8]) -> FrameBytes<'_> {
        FrameBytes::from(b)
    }

    #[test]
    fn gread_uniform_fields_and_slots() {
        // addr=0x0084 count=8, ids [1, 2, 5].
        let p = [0x84, 0x00, 0x08, 0x00, 1, 2, 5];
        let g = GreadUniform::parse(fb(&p)).unwrap();
        assert_eq!(g.addr, 0x0084);
        assert_eq!(g.count, 8);
        let ids: [Id; 3] = [Id::new(1), Id::new(2), Id::new(5)];
        assert!(g.ids().eq(ids));
        assert_eq!(g.slot_of(Id::new(1)), Some(0));
        assert_eq!(g.slot_of(Id::new(2)), Some(1));
        assert_eq!(g.slot_of(Id::new(5)), Some(2));
        assert_eq!(g.slot_of(Id::new(9)), None);
    }

    #[test]
    fn gread_uniform_rejects_empty_and_short() {
        // Head present but no IDs listed.
        assert_eq!(GreadUniform::parse(fb(&[0x84, 0x00, 0x08, 0x00])), None);
        // Fewer than the 4 head bytes.
        assert_eq!(GreadUniform::parse(fb(&[0x84, 0x00, 0x08])), None);
    }

    #[test]
    fn gread_per_target_iteration_and_find() {
        // id 10 addr 0x0200 count 4 · id 20 addr 0x0010 count 2.
        let p = [10, 0x00, 0x02, 0x04, 0x00, 20, 0x10, 0x00, 0x02, 0x00];
        let g = GreadPerTarget::parse(fb(&p)).unwrap();
        let got: [GreadTarget; 2] = [g.iter().next().unwrap(), g.iter().nth(1).unwrap()];
        assert_eq!(
            got,
            [
                GreadTarget {
                    id: Id::new(10),
                    addr: 0x0200,
                    count: 4
                },
                GreadTarget {
                    id: Id::new(20),
                    addr: 0x0010,
                    count: 2
                },
            ]
        );
        let (slot, entry) = g.find(Id::new(20)).unwrap();
        assert_eq!(slot, 1);
        assert_eq!(entry.addr, 0x0010);
        assert_eq!(g.find(Id::new(99)), None);
    }

    #[test]
    fn gread_per_target_rejects_ragged() {
        // 9 bytes: not a whole number of 5-byte entries.
        assert!(GreadPerTarget::parse(fb(&[0; 9])).is_none());
        assert!(GreadPerTarget::parse(fb(&[])).is_none());
    }

    #[test]
    fn gread_profile_uniform_fields_and_slots() {
        // profile slot 2, ids [1, 2, 5].
        let p = [2, 1, 2, 5];
        let g = GreadProfileUniform::parse(fb(&p)).unwrap();
        assert_eq!(g.slot, 2);
        let ids: [Id; 3] = [Id::new(1), Id::new(2), Id::new(5)];
        assert!(g.ids().eq(ids));
        assert_eq!(g.slot_of(Id::new(5)), Some(2));
        assert_eq!(g.slot_of(Id::new(9)), None);
    }

    #[test]
    fn gread_profile_uniform_rejects_empty_and_short() {
        // Slot byte present but no IDs listed.
        assert_eq!(GreadProfileUniform::parse(fb(&[2])), None);
        assert_eq!(GreadProfileUniform::parse(fb(&[])), None);
    }

    #[test]
    fn gread_profile_per_target_iteration_and_find() {
        // id 10 slot 0 · id 20 slot 3.
        let p = [10, 0, 20, 3];
        let g = GreadProfilePerTarget::parse(fb(&p)).unwrap();
        let got: [GreadProfileTarget; 2] = [g.iter().next().unwrap(), g.iter().nth(1).unwrap()];
        assert_eq!(
            got,
            [
                GreadProfileTarget {
                    id: Id::new(10),
                    slot: 0
                },
                GreadProfileTarget {
                    id: Id::new(20),
                    slot: 3
                },
            ]
        );
        let (chain_slot, entry) = g.find(Id::new(20)).unwrap();
        assert_eq!(chain_slot, 1);
        assert_eq!(entry.slot, 3);
        assert_eq!(g.find(Id::new(99)), None);
    }

    #[test]
    fn gread_profile_per_target_rejects_ragged() {
        assert!(GreadProfilePerTarget::parse(fb(&[10, 0, 20])).is_none());
        assert!(GreadProfilePerTarget::parse(fb(&[])).is_none());
    }

    #[test]
    fn gwrite_uniform_slices_and_find() {
        // addr 0x0180 count 4 · id 1 data [1,2,3,4] · id 2 data [5,6,7,8].
        let p = [0x80, 0x01, 4, 1, 1, 2, 3, 4, 2, 5, 6, 7, 8];
        let g = GwriteUniform::parse(fb(&p)).unwrap();
        assert_eq!(g.addr, 0x0180);
        assert_eq!(g.count, 4);
        let entries: [(Id, FrameBytes); 2] = [
            (Id::new(1), fb(&[1, 2, 3, 4])),
            (Id::new(2), fb(&[5, 6, 7, 8])),
        ];
        assert!(g.iter().eq(entries));
        assert_eq!(g.find(Id::new(2)), Some(fb(&[5, 6, 7, 8])));
        assert_eq!(g.find(Id::new(3)), None);
    }

    #[test]
    fn gwrite_uniform_rejects_ragged_and_zero_count() {
        // Trailing byte past the last whole entry.
        let ragged = [0x80, 0x01, 4, 1, 1, 2, 3, 4, 2];
        assert!(GwriteUniform::parse(fb(&ragged)).is_none());
        // count 0 has no valid stride.
        assert!(GwriteUniform::parse(fb(&[0x80, 0x01, 0])).is_none());
    }

    #[test]
    fn gwrite_per_target_mixed_counts() {
        // id 3 addr 0x0000 count 2 [AA BB] · id 4 addr 0x0100 count 4 [1 2 3 4].
        let p = [3, 0x00, 0x00, 2, 0xAA, 0xBB, 4, 0x00, 0x01, 4, 1, 2, 3, 4];
        let g = GwritePerTarget::parse(fb(&p)).unwrap();
        let targets: [GwriteTarget; 2] = [g.iter().next().unwrap(), g.iter().nth(1).unwrap()];
        assert_eq!(
            targets,
            [
                GwriteTarget {
                    id: Id::new(3),
                    addr: 0x0000,
                    data: fb(&[0xAA, 0xBB])
                },
                GwriteTarget {
                    id: Id::new(4),
                    addr: 0x0100,
                    data: fb(&[1, 2, 3, 4])
                },
            ]
        );
        assert_eq!(g.find(Id::new(4)).unwrap().data, fb(&[1, 2, 3, 4]));
        assert_eq!(g.find(Id::new(9)), None);
    }

    #[test]
    fn gwrite_per_target_rejects_truncated() {
        // Final entry claims count 4 but only 3 data bytes follow.
        let p = [3, 0x00, 0x00, 2, 0xAA, 0xBB, 4, 0x00, 0x01, 4, 1, 2, 3];
        assert!(GwritePerTarget::parse(fb(&p)).is_none());
        assert!(GwritePerTarget::parse(fb(&[])).is_none());
    }

    #[test]
    fn gwrite_uniform_goal_position_hot_path() {
        // SyncWrite-style: goal position (addr 0x0074), 4 B each, 3 servos.
        let p = [
            0x74, 0x00, 4, // addr, count
            1, 0x00, 0x08, 0x00, 0x00, // id 1
            2, 0x00, 0x0C, 0x00, 0x00, // id 2 (middle)
            3, 0x00, 0x10, 0x00, 0x00, // id 3
        ];
        let g = GwriteUniform::parse(fb(&p)).unwrap();
        assert_eq!(g.find(Id::new(2)), Some(fb(&[0x00, 0x0C, 0x00, 0x00])));
    }

    /// Every group payload parses identically no matter where the ring seam
    /// splits it: the contiguous parse equals the split parse at every point.
    #[test]
    fn group_parse_is_seam_split_invariant() {
        let gread_uniform = [0x84, 0x00, 0x08, 0x00, 1, 2, 5];
        let gread_per_target = [10, 0x00, 0x02, 0x04, 0x00, 20, 0x10, 0x00, 0x02, 0x00];
        let gwrite_uniform = [0x80, 0x01, 4, 1, 1, 2, 3, 4, 2, 5, 6, 7, 8];
        let gwrite_per_target = [3, 0x00, 0x00, 2, 0xAA, 0xBB, 4, 0x00, 0x01, 4, 1, 2, 3, 4];
        let gread_profile_uniform = [2, 1, 2, 5];
        let gread_profile_per_target = [10, 0, 20, 3];

        for k in 0..=gread_uniform.len() {
            let s = FrameBytes::new(&gread_uniform[..k], &gread_uniform[k..]);
            assert_eq!(
                GreadUniform::parse(s),
                GreadUniform::parse(fb(&gread_uniform))
            );
            let g = GreadUniform::parse(s).unwrap();
            assert_eq!(g.slot_of(Id::new(5)), Some(2));
        }
        for k in 0..=gread_per_target.len() {
            let s = FrameBytes::new(&gread_per_target[..k], &gread_per_target[k..]);
            assert_eq!(
                GreadPerTarget::parse(s),
                GreadPerTarget::parse(fb(&gread_per_target))
            );
            let g = GreadPerTarget::parse(s).unwrap();
            assert_eq!(g.find(Id::new(20)).unwrap().1.addr, 0x0010);
        }
        for k in 0..=gwrite_uniform.len() {
            let s = FrameBytes::new(&gwrite_uniform[..k], &gwrite_uniform[k..]);
            assert_eq!(
                GwriteUniform::parse(s),
                GwriteUniform::parse(fb(&gwrite_uniform))
            );
            let g = GwriteUniform::parse(s).unwrap();
            assert_eq!(g.find(Id::new(2)), Some(fb(&[5, 6, 7, 8])));
        }
        for k in 0..=gwrite_per_target.len() {
            let s = FrameBytes::new(&gwrite_per_target[..k], &gwrite_per_target[k..]);
            assert_eq!(
                GwritePerTarget::parse(s),
                GwritePerTarget::parse(fb(&gwrite_per_target))
            );
            let g = GwritePerTarget::parse(s).unwrap();
            assert_eq!(g.find(Id::new(4)).unwrap().data, fb(&[1, 2, 3, 4]));
        }
        for k in 0..=gread_profile_uniform.len() {
            let s = FrameBytes::new(&gread_profile_uniform[..k], &gread_profile_uniform[k..]);
            assert_eq!(
                GreadProfileUniform::parse(s),
                GreadProfileUniform::parse(fb(&gread_profile_uniform))
            );
            let g = GreadProfileUniform::parse(s).unwrap();
            assert_eq!(g.slot_of(Id::new(5)), Some(2));
        }
        for k in 0..=gread_profile_per_target.len() {
            let s = FrameBytes::new(
                &gread_profile_per_target[..k],
                &gread_profile_per_target[k..],
            );
            assert_eq!(
                GreadProfilePerTarget::parse(s),
                GreadProfilePerTarget::parse(fb(&gread_profile_per_target))
            );
            let g = GreadProfilePerTarget::parse(s).unwrap();
            assert_eq!(g.find(Id::new(20)).unwrap().1.slot, 3);
        }
    }
}
