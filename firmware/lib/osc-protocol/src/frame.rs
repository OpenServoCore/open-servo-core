//! osc-native RX views: a `#[repr(C)]` header over ring bytes plus zero-copy
//! payload parsers (`docs/osc-native-protocol.md` §3.1, §5). Layout only — the
//! chip owns the ring and its cursor; these borrow into it.

use crate::bytes::FrameBytes;
use crate::wire::{self, Id, Inst, MgmtOp};

/// Why a header cannot be dispatched. Frame-level rejects (§5.3 layer 1): the
/// frame is dropped, no reply.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FrameError {
    ShortLen,
    BadId,
    BadOpcode,
}

/// The four fixed bytes at a frame anchor: the break's `0x00` ring byte, then
/// `ID`, `LEN`, `INST` (§3.1). All fields are `u8`-transparent, so the struct
/// is four bytes at alignment 1 and any bit pattern is a valid `Header` — that
/// is what makes the pointer casts below sound.
#[repr(C)]
pub struct Header {
    pub brk: u8,
    pub id: Id,
    pub len: u8,
    pub inst: Inst,
}

impl Header {
    pub const SIZE: usize = 4;

    #[inline]
    pub fn from_bytes(b: &[u8; 4]) -> &Header {
        // SAFETY: `Header` is four repr(transparent) u8 newtypes (size 4,
        // align 1); every 4-byte pattern is a valid `Header`, and the borrow's
        // lifetime is tied to `b`.
        unsafe { &*(b.as_ptr() as *const Header) }
    }

    /// # Safety
    /// `p` must point to 4 readable bytes. The caller picks the lifetime `'a`
    /// and must keep the ring memory valid for it.
    #[inline]
    pub unsafe fn from_anchor<'a>(p: *const u8) -> &'a Header {
        // SAFETY: caller guarantees 4 readable bytes at `p`; `Header` is 4
        // u8-transparent fields (align 1), valid for any bit pattern.
        unsafe { &*(p as *const Header) }
    }

    /// `LEN` must cover INST + CRC (≥ 3, §3.1), `ID` addressable, and — for
    /// instruction frames — the opcode nonzero (§5).
    pub fn validate(&self) -> Result<(), FrameError> {
        if self.len < 3 {
            return Err(FrameError::ShortLen);
        }
        if !self.id.is_valid() {
            return Err(FrameError::BadId);
        }
        if !self.inst.is_status() && self.inst.opcode().is_none() {
            return Err(FrameError::BadOpcode);
        }
        Ok(())
    }

    /// Ring bytes from the anchor to the frame's end, exclusive (§3.1).
    #[inline]
    pub fn frame_end(&self) -> usize {
        wire::footprint(self.len)
    }

    #[inline]
    pub fn covered_len(&self) -> usize {
        wire::covered_len(self.len)
    }

    #[inline]
    pub fn payload_len(&self) -> u8 {
        wire::payload_len(self.len)
    }
}

/// READ / GREAD span request: `addr(2), count(2)` little-endian (§5).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ReadReq {
    pub addr: u16,
    pub count: u16,
}

impl ReadReq {
    #[inline]
    pub fn parse(payload: FrameBytes) -> Option<ReadReq> {
        if payload.len() != 4 {
            return None;
        }
        Some(ReadReq {
            addr: payload.u16_le_at(0)?,
            count: payload.u16_le_at(2)?,
        })
    }
}

/// READ+PROFILE slot request: `slot(1)` names a profile slot instead of
/// addr+count (§5.2).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ProfileReq {
    pub slot: u8,
}

impl ProfileReq {
    #[inline]
    pub fn parse(payload: FrameBytes) -> Option<ProfileReq> {
        if payload.len() != 1 {
            return None;
        }
        Some(ProfileReq {
            slot: payload.u8_at(0)?,
        })
    }
}

/// WRITE request: `addr(2)` little-endian, then the data bytes (§5).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WriteReq<'a> {
    pub addr: u16,
    pub data: FrameBytes<'a>,
}

impl<'a> WriteReq<'a> {
    #[inline]
    pub fn parse(payload: FrameBytes<'a>) -> Option<WriteReq<'a>> {
        let addr = payload.u16_le_at(0)?;
        let data = payload.sub(2, payload.len().checked_sub(2)?)?;
        Some(WriteReq { addr, data })
    }
}

/// MGMT request: sub-op byte, then its args (§9).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MgmtReq<'a> {
    pub op: MgmtOp,
    pub args: FrameBytes<'a>,
}

impl<'a> MgmtReq<'a> {
    #[inline]
    pub fn parse(payload: FrameBytes<'a>) -> Option<MgmtReq<'a>> {
        let op = MgmtOp::from_byte(payload.u8_at(0)?)?;
        let args = payload.sub(1, payload.len().checked_sub(1)?)?;
        Some(MgmtReq { op, args })
    }
}

/// MGMT ENUM args: `prefix_len(1)` in bits (0..=128), then the
/// `ceil(prefix_len/8)`-byte prefix (§9.2). Unused prefix bytes are zero.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EnumReq {
    pub prefix_len: u8,
    pub prefix: [u8; wire::UID_LEN],
}

impl EnumReq {
    pub const MAX_PREFIX_BITS: u8 = (wire::UID_LEN * 8) as u8;

    #[inline]
    pub fn parse(args: FrameBytes) -> Option<EnumReq> {
        let prefix_len = args.u8_at(0)?;
        if prefix_len > Self::MAX_PREFIX_BITS {
            return None;
        }
        let n = prefix_len.div_ceil(8) as usize;
        if args.len() != 1 + n {
            return None;
        }
        let mut prefix = [0u8; wire::UID_LEN];
        args.sub(1, n)?.copy_into(&mut prefix[..n])?;
        Some(EnumReq { prefix_len, prefix })
    }
}

/// §9.2: does `uid` begin with the `prefix_len`-bit prefix? The prefix is an
/// LSB-first bit stream — bit `k` is `uid[k/8] >> (k%8) & 1`, the order the
/// UART itself shifts bits onto the wire.
pub fn uid_prefix_matches(
    uid: &[u8; wire::UID_LEN],
    prefix_len: u8,
    prefix: &[u8; wire::UID_LEN],
) -> bool {
    let full = (prefix_len / 8) as usize;
    let rem = prefix_len % 8;
    if uid[..full] != prefix[..full] {
        return false;
    }
    rem == 0 || (uid[full] ^ prefix[full]) & ((1 << rem) - 1) == 0
}

/// MGMT ASSIGN args: `uid(16)`, `new_id(1)` (§9.2).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct AssignReq {
    pub uid: [u8; wire::UID_LEN],
    pub new_id: u8,
}

impl AssignReq {
    #[inline]
    pub fn parse(args: FrameBytes) -> Option<AssignReq> {
        if args.len() != wire::UID_LEN + 1 {
            return None;
        }
        let mut uid = [0u8; wire::UID_LEN];
        args.sub(0, wire::UID_LEN)?.copy_into(&mut uid)?;
        Some(AssignReq {
            uid,
            new_id: args.u8_at(wire::UID_LEN)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{Opcode, ResultCode};

    #[test]
    fn header_over_ping_vector() {
        let h = Header::from_bytes(&[0x00, 0x01, 0x03, 0x10]);
        assert_eq!(h.brk, 0x00);
        assert_eq!(h.id, Id::new(1));
        assert_eq!(h.len, 3);
        assert_eq!(h.inst.opcode(), Some(Opcode::Ping));
        assert_eq!(h.payload_len(), 0);
        assert_eq!(h.covered_len(), 4);
        assert_eq!(h.frame_end(), 6);
        assert_eq!(h.validate(), Ok(()));
    }

    #[test]
    fn header_payload_len() {
        // Odd-payload WRITE vector: LEN 6 → p = 3 (no pad, §3.1).
        let h = Header::from_bytes(&[0x00, 0x02, 0x06, 0x30]);
        assert_eq!(h.payload_len(), 3);
        assert_eq!(h.frame_end(), 9);
    }

    #[test]
    fn validate_rejects() {
        assert_eq!(
            Header::from_bytes(&[0x00, 0x01, 0x02, 0x10]).validate(),
            Err(FrameError::ShortLen)
        );
        // Even LEN is legal (§3.1: any LEN >= 3).
        assert_eq!(
            Header::from_bytes(&[0x00, 0x01, 0x04, 0x10]).validate(),
            Ok(())
        );
        assert_eq!(
            Header::from_bytes(&[0x00, 0x00, 0x03, 0x10]).validate(),
            Err(FrameError::BadId)
        );
        assert_eq!(
            Header::from_bytes(&[0x00, 0x01, 0x03, 0x00]).validate(),
            Err(FrameError::BadOpcode)
        );
    }

    fn fb(b: &[u8]) -> FrameBytes<'_> {
        FrameBytes::from(b)
    }

    #[test]
    fn read_req_parse() {
        assert_eq!(
            ReadReq::parse(fb(&[0x00, 0x02, 0x08, 0x00])),
            Some(ReadReq {
                addr: 0x0200,
                count: 8
            })
        );
        assert_eq!(ReadReq::parse(fb(&[0x00, 0x02, 0x08])), None);
        assert_eq!(ReadReq::parse(fb(&[0, 0, 0, 0, 0])), None);
    }

    #[test]
    fn profile_req_parse() {
        assert_eq!(ProfileReq::parse(fb(&[2])), Some(ProfileReq { slot: 2 }));
        assert_eq!(ProfileReq::parse(fb(&[])), None);
        assert_eq!(ProfileReq::parse(fb(&[2, 0])), None);
    }

    #[test]
    fn write_req_parse() {
        assert_eq!(
            WriteReq::parse(fb(&[0x80, 0x01, 0x2C, 0x01])),
            Some(WriteReq {
                addr: 0x0180,
                data: fb(&[0x2C, 0x01])
            })
        );
        // Empty data is legal (>= 2 bytes = addr only).
        assert_eq!(
            WriteReq::parse(fb(&[0x00, 0x01])),
            Some(WriteReq {
                addr: 0x0100,
                data: fb(&[])
            })
        );
        assert_eq!(WriteReq::parse(fb(&[0x80])), None);
    }

    #[test]
    fn mgmt_req_parse() {
        assert_eq!(
            MgmtReq::parse(fb(&[0x02, 0xAA])),
            Some(MgmtReq {
                op: MgmtOp::Assign,
                args: fb(&[0xAA])
            })
        );
        assert_eq!(MgmtReq::parse(fb(&[])), None);
        assert_eq!(MgmtReq::parse(fb(&[0x00])), None);
    }

    #[test]
    fn enum_req_parse() {
        // Empty prefix: the tree root, matches every servo.
        assert_eq!(
            EnumReq::parse(fb(&[0])),
            Some(EnumReq {
                prefix_len: 0,
                prefix: [0; 16]
            })
        );
        // 10 bits → 2 prefix bytes, upper bytes zero-filled.
        let e = EnumReq::parse(fb(&[10, 0xAB, 0x03])).unwrap();
        assert_eq!(e.prefix_len, 10);
        assert_eq!(e.prefix[..2], [0xAB, 0x03]);
        assert_eq!(e.prefix[2..], [0; 14]);
        // Full 128-bit prefix.
        let mut full = [0u8; 17];
        full[0] = 128;
        for (i, b) in full[1..].iter_mut().enumerate() {
            *b = i as u8 + 1;
        }
        let e = EnumReq::parse(fb(&full)).unwrap();
        assert_eq!(e.prefix, full[1..]);
        // Rejects: over-long prefix_len, byte count not matching prefix_len.
        assert_eq!(EnumReq::parse(fb(&[129])), None);
        assert_eq!(EnumReq::parse(fb(&[10, 0xAB])), None);
        assert_eq!(EnumReq::parse(fb(&[8, 0xAB, 0xCD])), None);
        assert_eq!(EnumReq::parse(fb(&[])), None);
    }

    #[test]
    fn uid_prefix_match_boundaries() {
        let mut uid = [0u8; 16];
        uid[0] = 0b1010_0101;
        uid[1] = 0xFF;
        uid[15] = 0x80;
        let m = |len: u8, prefix: &[u8]| {
            let mut p = [0u8; 16];
            p[..prefix.len()].copy_from_slice(prefix);
            uid_prefix_matches(&uid, len, &p)
        };
        // Length 0 matches everything.
        assert!(m(0, &[]));
        // Bit 0 is the LSB of byte 0 (LSB-first stream).
        assert!(m(1, &[0b1]));
        assert!(!m(1, &[0b0]));
        // Partial byte: only the low `rem` bits compare.
        assert!(m(3, &[0b101]));
        assert!(!m(3, &[0b111])); // bit 1 differs
        assert!(m(3, &[0b1111_1101])); // high bits past the prefix are masked off
        // Byte boundary: 8 bits = whole first byte.
        assert!(m(8, &[0b1010_0101]));
        assert!(!m(8, &[0b0010_0101]));
        // 127 bits: everything but the UID's top bit.
        let mut p = uid;
        p[15] = 0x00; // differs only in bit 127
        assert!(uid_prefix_matches(&uid, 127, &p));
        assert!(!uid_prefix_matches(&uid, 128, &p));
        // 128 bits: exact match.
        assert!(uid_prefix_matches(&uid, 128, &uid));
    }

    #[test]
    fn assign_req_parse() {
        let mut args = [0u8; 17];
        for (i, b) in args[..16].iter_mut().enumerate() {
            *b = i as u8 + 1;
        }
        args[16] = 7;
        let want: [u8; 16] = args[..16].try_into().unwrap();
        assert_eq!(
            AssignReq::parse(fb(&args)),
            Some(AssignReq {
                uid: want,
                new_id: 7
            })
        );
        assert_eq!(AssignReq::parse(fb(&args[..16])), None);
        assert_eq!(AssignReq::parse(fb(&[])), None);
    }

    #[test]
    fn status_frame_validates() {
        let bytes = [0x00, 0x07, 0x03, Inst::status(ResultCode::Ok, false).0];
        let h = Header::from_bytes(&bytes);
        assert!(h.inst.is_status());
        assert_eq!(h.inst.opcode(), None);
        assert_eq!(h.validate(), Ok(()));
    }
}
