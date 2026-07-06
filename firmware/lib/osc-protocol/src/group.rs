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

use crate::wire::Id;

#[inline]
fn le_u16(b: &[u8], i: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*b.get(i)?, *b.get(i + 1)?]))
}

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
    pub data: &'a [u8],
}

/// Uniform GREAD: `addr(2) count(2) id-list(1 each)` — one span, many servos,
/// replies chain in list order.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadUniform<'a> {
    pub addr: u16,
    pub count: u16,
    ids: &'a [u8],
}

impl<'a> GreadUniform<'a> {
    pub fn parse(payload: &'a [u8]) -> Option<Self> {
        let addr = le_u16(payload, 0)?;
        let count = le_u16(payload, 2)?;
        let ids = payload.get(4..)?;
        if ids.is_empty() {
            return None;
        }
        Some(Self { addr, count, ids })
    }

    pub fn ids(&self) -> impl Iterator<Item = Id> + '_ {
        self.ids.iter().map(|&b| Id::new(b))
    }

    pub fn slot_of(&self, id: Id) -> Option<u8> {
        self.ids
            .iter()
            .position(|&b| b == id.as_byte())
            .map(|p| p as u8)
    }
}

/// PER_TARGET GREAD: `[id(1) addr(2) count(2)]×` — fixed 5-byte entries.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GreadPerTarget<'a> {
    entries: &'a [u8],
}

impl<'a> GreadPerTarget<'a> {
    pub fn parse(payload: &'a [u8]) -> Option<Self> {
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
    rest: &'a [u8],
}

impl Iterator for GreadPerTargetIter<'_> {
    type Item = GreadTarget;

    fn next(&mut self) -> Option<GreadTarget> {
        let (chunk, rest) = self.rest.split_at_checked(5)?;
        self.rest = rest;
        let &[id, a0, a1, c0, c1] = chunk else {
            return None;
        };
        Some(GreadTarget {
            id: Id::new(id),
            addr: u16::from_le_bytes([a0, a1]),
            count: u16::from_le_bytes([c0, c1]),
        })
    }
}

/// Uniform GWRITE: `addr(2) count(1) [id(1) data(count)]×` — fixed-stride
/// entries after the 3-byte head.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GwriteUniform<'a> {
    pub addr: u16,
    pub count: u8,
    entries: &'a [u8],
}

impl<'a> GwriteUniform<'a> {
    pub fn parse(payload: &'a [u8]) -> Option<Self> {
        let addr = le_u16(payload, 0)?;
        let count = *payload.get(2)?;
        if count == 0 {
            return None;
        }
        let entries = payload.get(3..)?;
        let stride = 1 + count as usize;
        if entries.is_empty() || !entries.len().is_multiple_of(stride) {
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

    pub fn find(&self, id: Id) -> Option<&'a [u8]> {
        self.iter().find(|(i, _)| *i == id).map(|(_, d)| d)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct GwriteUniformIter<'a> {
    rest: &'a [u8],
    stride: usize,
}

impl<'a> Iterator for GwriteUniformIter<'a> {
    type Item = (Id, &'a [u8]);

    fn next(&mut self) -> Option<(Id, &'a [u8])> {
        let (chunk, rest) = self.rest.split_at_checked(self.stride)?;
        self.rest = rest;
        let (id, data) = chunk.split_first()?;
        Some((Id::new(*id), data))
    }
}

/// PER_TARGET GWRITE: `[id(1) addr(2) count(1) data(count)]×` — variable
/// stride, each entry's `count` sizes its own data.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GwritePerTarget<'a> {
    entries: &'a [u8],
}

impl<'a> GwritePerTarget<'a> {
    pub fn parse(payload: &'a [u8]) -> Option<Self> {
        if payload.is_empty() {
            return None;
        }
        let mut rest = payload;
        while !rest.is_empty() {
            let count = *rest.get(3)? as usize;
            rest = rest.get(4 + count..)?;
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
    rest: &'a [u8],
}

impl<'a> Iterator for GwritePerTargetIter<'a> {
    type Item = GwriteTarget<'a>;

    fn next(&mut self) -> Option<GwriteTarget<'a>> {
        let &[id, a0, a1, count] = self.rest.get(..4)? else {
            return None;
        };
        let (data, rest) = self.rest.get(4..)?.split_at_checked(count as usize)?;
        self.rest = rest;
        Some(GwriteTarget {
            id: Id::new(id),
            addr: u16::from_le_bytes([a0, a1]),
            data,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gread_uniform_fields_and_slots() {
        // addr=0x0084 count=8, ids [1, 2, 5].
        let p = [0x84, 0x00, 0x08, 0x00, 1, 2, 5];
        let g = GreadUniform::parse(&p).unwrap();
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
        assert_eq!(GreadUniform::parse(&[0x84, 0x00, 0x08, 0x00]), None);
        // Fewer than the 4 head bytes.
        assert_eq!(GreadUniform::parse(&[0x84, 0x00, 0x08]), None);
    }

    #[test]
    fn gread_per_target_iteration_and_find() {
        // id 10 addr 0x0200 count 4 · id 20 addr 0x0010 count 2.
        let p = [10, 0x00, 0x02, 0x04, 0x00, 20, 0x10, 0x00, 0x02, 0x00];
        let g = GreadPerTarget::parse(&p).unwrap();
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
        assert!(GreadPerTarget::parse(&[0; 9]).is_none());
        assert!(GreadPerTarget::parse(&[]).is_none());
    }

    #[test]
    fn gwrite_uniform_slices_and_find() {
        // addr 0x0180 count 4 · id 1 data [1,2,3,4] · id 2 data [5,6,7,8].
        let p = [0x80, 0x01, 4, 1, 1, 2, 3, 4, 2, 5, 6, 7, 8];
        let g = GwriteUniform::parse(&p).unwrap();
        assert_eq!(g.addr, 0x0180);
        assert_eq!(g.count, 4);
        let entries: [(Id, &[u8]); 2] = [(Id::new(1), &[1, 2, 3, 4]), (Id::new(2), &[5, 6, 7, 8])];
        assert!(g.iter().eq(entries));
        assert_eq!(g.find(Id::new(2)), Some(&[5, 6, 7, 8][..]));
        assert_eq!(g.find(Id::new(3)), None);
    }

    #[test]
    fn gwrite_uniform_rejects_ragged_and_zero_count() {
        // Trailing byte past the last whole entry.
        let ragged = [0x80, 0x01, 4, 1, 1, 2, 3, 4, 2];
        assert!(GwriteUniform::parse(&ragged).is_none());
        // count 0 has no valid stride.
        assert!(GwriteUniform::parse(&[0x80, 0x01, 0]).is_none());
    }

    #[test]
    fn gwrite_per_target_mixed_counts() {
        // id 3 addr 0x0000 count 2 [AA BB] · id 4 addr 0x0100 count 4 [1 2 3 4].
        let p = [3, 0x00, 0x00, 2, 0xAA, 0xBB, 4, 0x00, 0x01, 4, 1, 2, 3, 4];
        let g = GwritePerTarget::parse(&p).unwrap();
        let targets: [GwriteTarget; 2] = [g.iter().next().unwrap(), g.iter().nth(1).unwrap()];
        assert_eq!(
            targets,
            [
                GwriteTarget {
                    id: Id::new(3),
                    addr: 0x0000,
                    data: &[0xAA, 0xBB]
                },
                GwriteTarget {
                    id: Id::new(4),
                    addr: 0x0100,
                    data: &[1, 2, 3, 4]
                },
            ]
        );
        assert_eq!(g.find(Id::new(4)).unwrap().data, &[1, 2, 3, 4]);
        assert_eq!(g.find(Id::new(9)), None);
    }

    #[test]
    fn gwrite_per_target_rejects_truncated() {
        // Final entry claims count 4 but only 3 data bytes follow.
        let p = [3, 0x00, 0x00, 2, 0xAA, 0xBB, 4, 0x00, 0x01, 4, 1, 2, 3];
        assert!(GwritePerTarget::parse(&p).is_none());
        assert!(GwritePerTarget::parse(&[]).is_none());
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
        let g = GwriteUniform::parse(&p).unwrap();
        assert_eq!(g.find(Id::new(2)), Some(&[0x00, 0x0C, 0x00, 0x00][..]));
    }
}
