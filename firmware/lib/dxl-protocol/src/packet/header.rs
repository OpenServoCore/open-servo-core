//! Frame header overlay -- at offset 0 of every DXL 2.0 packet.

use super::instruction::InstructionByte;

/// Little-endian u16 as two `u8` fields -- alignment 1, so containing
/// overlay structs cast soundly from any byte offset.
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

/// DXL 2.0 packet ID byte. Transparent over `u8` so overlay layout matches
/// the wire. The decoder casts wire bytes into `Id` without validation;
/// `is_broadcast` / `is_reserved` are informational. `try_servo` is the
/// validating constructor for caller-supplied input.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Id(pub u8);

impl Id {
    pub const BROADCAST: Self = Self(0xFE);

    #[inline]
    pub const fn new(b: u8) -> Self {
        Self(b)
    }

    /// Validated servo-id constructor. Rejects `0xFD` (stuffing byte),
    /// `0xFE` (broadcast), and `0xFF` (reserved).
    #[inline]
    pub const fn try_servo(b: u8) -> Option<Self> {
        match b {
            0xFD..=0xFF => None,
            _ => Some(Self(b)),
        }
    }

    #[inline]
    pub const fn as_byte(self) -> u8 {
        self.0
    }

    #[inline]
    pub const fn is_broadcast(self) -> bool {
        self.0 == 0xFE
    }

    /// True for `0xFD` and `0xFF`. Broadcast (`0xFE`) is a real addressing
    /// value, not reserved.
    #[inline]
    pub const fn is_reserved(self) -> bool {
        matches!(self.0, 0xFD | 0xFF)
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct Header {
    pub sync: [u8; 4],
    pub id: Id,
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

    #[test]
    fn id_layout() {
        assert_eq!(align_of::<Id>(), 1);
        assert_eq!(size_of::<Id>(), 1);
    }

    #[test]
    fn id_classifications() {
        assert!(Id::BROADCAST.is_broadcast());
        assert!(!Id::BROADCAST.is_reserved());
        assert!(Id::new(0xFD).is_reserved());
        assert!(Id::new(0xFF).is_reserved());
        assert!(!Id::new(0x01).is_reserved());
        assert!(!Id::new(0x01).is_broadcast());
    }

    #[test]
    fn try_servo_rejects_reserved_and_broadcast() {
        assert_eq!(Id::try_servo(0x00), Some(Id::new(0x00)));
        assert_eq!(Id::try_servo(0xFC), Some(Id::new(0xFC)));
        assert_eq!(Id::try_servo(0xFD), None);
        assert_eq!(Id::try_servo(0xFE), None);
        assert_eq!(Id::try_servo(0xFF), None);
    }
}
