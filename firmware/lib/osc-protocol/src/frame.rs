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
    fn status_frame_validates() {
        let bytes = [0x00, 0x07, 0x03, Inst::status(ResultCode::Ok, false).0];
        let h = Header::from_bytes(&bytes);
        assert!(h.inst.is_status());
        assert_eq!(h.inst.opcode(), None);
        assert_eq!(h.validate(), Ok(()));
    }
}
