//! osc-native RX views: a `#[repr(C)]` header over ring bytes plus zero-copy
//! payload parsers (`docs/osc-native-protocol.md` §3.1, §5). Layout only — the
//! chip owns the ring and its cursor; these borrow into it.

use crate::wire::{self, Id, Inst, MgmtOp};

/// Why a header cannot be dispatched. Frame-level rejects (§5.3 layer 1): the
/// frame is dropped, no reply.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FrameError {
    EvenLen,
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

    /// `LEN` must be odd (PAD invariant, §3.1), `ID` addressable, and — for
    /// instruction frames — the opcode nonzero (§5).
    pub fn validate(&self) -> Result<(), FrameError> {
        if self.len & 1 == 0 {
            return Err(FrameError::EvenLen);
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
        wire::payload_len(self.len, self.inst.pad())
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
    pub fn parse(payload: &[u8]) -> Option<ReadReq> {
        let b: &[u8; 4] = payload.try_into().ok()?;
        Some(ReadReq {
            addr: u16::from_le_bytes([b[0], b[1]]),
            count: u16::from_le_bytes([b[2], b[3]]),
        })
    }
}

/// WRITE request: `addr(2)` little-endian, then the data bytes (§5).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WriteReq<'a> {
    pub addr: u16,
    pub data: &'a [u8],
}

impl<'a> WriteReq<'a> {
    #[inline]
    pub fn parse(payload: &'a [u8]) -> Option<WriteReq<'a>> {
        let (addr, data) = payload.split_first_chunk::<2>()?;
        Some(WriteReq {
            addr: u16::from_le_bytes(*addr),
            data,
        })
    }
}

/// MGMT request: sub-op byte, then its args (§9).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MgmtReq<'a> {
    pub op: MgmtOp,
    pub args: &'a [u8],
}

impl<'a> MgmtReq<'a> {
    #[inline]
    pub fn parse(payload: &'a [u8]) -> Option<MgmtReq<'a>> {
        let (&op, args) = payload.split_first()?;
        Some(MgmtReq {
            op: MgmtOp::from_byte(op)?,
            args,
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
    fn header_payload_len_drops_pad() {
        // Padded WRITE vector: LEN 7, PAD set → p = 3.
        let h = Header::from_bytes(&[0x00, 0x02, 0x07, 0x32]);
        assert!(h.inst.pad());
        assert_eq!(h.payload_len(), 3);
        assert_eq!(h.frame_end(), 10);
    }

    #[test]
    fn validate_rejects() {
        assert_eq!(
            Header::from_bytes(&[0x00, 0x01, 0x04, 0x10]).validate(),
            Err(FrameError::EvenLen)
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

    #[test]
    fn read_req_parse() {
        assert_eq!(
            ReadReq::parse(&[0x00, 0x02, 0x08, 0x00]),
            Some(ReadReq {
                addr: 0x0200,
                count: 8
            })
        );
        assert_eq!(ReadReq::parse(&[0x00, 0x02, 0x08]), None);
        assert_eq!(ReadReq::parse(&[0, 0, 0, 0, 0]), None);
    }

    #[test]
    fn write_req_parse() {
        assert_eq!(
            WriteReq::parse(&[0x80, 0x01, 0x2C, 0x01]),
            Some(WriteReq {
                addr: 0x0180,
                data: &[0x2C, 0x01]
            })
        );
        // Empty data is legal (>= 2 bytes = addr only).
        assert_eq!(
            WriteReq::parse(&[0x00, 0x01]),
            Some(WriteReq {
                addr: 0x0100,
                data: &[]
            })
        );
        assert_eq!(WriteReq::parse(&[0x80]), None);
    }

    #[test]
    fn mgmt_req_parse() {
        assert_eq!(
            MgmtReq::parse(&[0x02, 0xAA]),
            Some(MgmtReq {
                op: MgmtOp::Assign,
                args: &[0xAA]
            })
        );
        assert_eq!(MgmtReq::parse(&[]), None);
        assert_eq!(MgmtReq::parse(&[0x00]), None);
    }

    #[test]
    fn status_frame_validates() {
        let bytes = [
            0x00,
            0x07,
            0x03,
            Inst::status(ResultCode::Ok, false, false).0,
        ];
        let h = Header::from_bytes(&bytes);
        assert!(h.inst.is_status());
        assert_eq!(h.inst.opcode(), None);
        assert_eq!(h.validate(), Ok(()));
    }
}
