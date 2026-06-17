//! DXL 2.0 packet ID byte.

/// Packet ID byte. The parser surfaces every observed value via [`new`]; use
/// [`try_servo`] for caller-supplied input that must address a real slave.
///
/// [`new`]: Self::new
/// [`try_servo`]: Self::try_servo
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifications() {
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
