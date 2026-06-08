//! Frame header overlay — present at offset 0 of every DXL 2.0 packet.

use super::instruction::InstructionByte;

/// Byte-aligned little-endian u16. Two `u8` fields keep alignment at 1 so
/// every overlay struct that contains one stays alignment-1 and casts
/// soundly from any byte offset.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct U16Le {
    pub lo: u8,
    pub hi: u8,
}

impl U16Le {
    #[inline]
    pub const fn from_u16(v: u16) -> Self {
        let [lo, hi] = v.to_le_bytes();
        Self { lo, hi }
    }

    #[inline]
    pub const fn get(&self) -> u16 {
        u16::from_le_bytes([self.lo, self.hi])
    }

    #[inline]
    pub fn set(&mut self, v: u16) {
        let [lo, hi] = v.to_le_bytes();
        self.lo = lo;
        self.hi = hi;
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Header {
    pub sync: [u8; 4],
    pub id: u8,
    pub len: U16Le,
    pub instruction: InstructionByte,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    #[test]
    fn u16le_alignment_and_size() {
        assert_eq!(align_of::<U16Le>(), 1);
        assert_eq!(size_of::<U16Le>(), 2);
    }

    #[test]
    fn u16le_round_trip() {
        let v = U16Le::from_u16(0x1234);
        assert_eq!(v.lo, 0x34);
        assert_eq!(v.hi, 0x12);
        assert_eq!(v.get(), 0x1234);
    }

    #[test]
    fn header_alignment_and_size() {
        assert_eq!(align_of::<Header>(), 1);
        assert_eq!(size_of::<Header>(), 8);
    }

    #[test]
    fn header_offsets() {
        assert_eq!(offset_of!(Header, sync), 0);
        assert_eq!(offset_of!(Header, id), 4);
        assert_eq!(offset_of!(Header, len), 5);
        assert_eq!(offset_of!(Header, instruction), 7);
    }
}
