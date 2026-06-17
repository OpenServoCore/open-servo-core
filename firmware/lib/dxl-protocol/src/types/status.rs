//! Status error byte (bits 0..=6 = code, bit 7 = sticky hardware alert).

/// DXL 2.0 Status error byte. Bit 7 = sticky hardware Alert; bits 0..=6 =
/// error code (only `0..=7` spec-defined).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusError(pub u8);

impl StatusError {
    pub const OK: Self = Self(0);
    pub const ALERT: u8 = 0x80;
    pub const CODE_MASK: u8 = 0x7F;

    #[inline]
    pub const fn from_byte(b: u8) -> Self {
        Self(b)
    }

    #[inline]
    pub const fn as_byte(self) -> u8 {
        self.0
    }

    #[inline]
    pub const fn new(alert: bool, code: ErrorCode) -> Self {
        let alert_bit = if alert { Self::ALERT } else { 0 };
        Self(alert_bit | code as u8)
    }

    #[inline]
    pub const fn code(code: ErrorCode) -> Self {
        Self(code as u8)
    }

    #[inline]
    pub const fn alert(self) -> bool {
        self.0 & Self::ALERT != 0
    }

    #[inline]
    pub const fn raw_code(self) -> u8 {
        self.0 & Self::CODE_MASK
    }

    /// Typed view of bits 0..=6; `None` for unspec'd values.
    #[inline]
    pub const fn kind(self) -> Option<ErrorCode> {
        ErrorCode::from_u8(self.raw_code())
    }

    #[inline]
    pub const fn is_ok(self) -> bool {
        self.0 == 0
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorCode {
    None = 0x00,
    Result = 0x01,
    Instruction = 0x02,
    Crc = 0x03,
    DataRange = 0x04,
    DataLength = 0x05,
    DataLimit = 0x06,
    Access = 0x07,
}

impl ErrorCode {
    #[inline]
    pub const fn from_u8(b: u8) -> Option<Self> {
        Some(match b {
            0x00 => Self::None,
            0x01 => Self::Result,
            0x02 => Self::Instruction,
            0x03 => Self::Crc,
            0x04 => Self::DataRange,
            0x05 => Self::DataLength,
            0x06 => Self::DataLimit,
            0x07 => Self::Access,
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ok_round_trips() {
        assert_eq!(StatusError::OK.as_byte(), 0);
        assert!(StatusError::OK.is_ok());
        assert!(!StatusError::OK.alert());
        assert_eq!(StatusError::OK.kind(), Some(ErrorCode::None));
    }

    #[test]
    fn new_packs_alert_bit_and_code() {
        let e = StatusError::new(true, ErrorCode::Access);
        assert_eq!(e.as_byte(), 0x80 | 0x07);
        assert!(e.alert());
        assert_eq!(e.raw_code(), 0x07);
        assert_eq!(e.kind(), Some(ErrorCode::Access));
    }

    #[test]
    fn code_alone_omits_alert() {
        let e = StatusError::code(ErrorCode::DataRange);
        assert_eq!(e.as_byte(), 0x04);
        assert!(!e.alert());
        assert!(!e.is_ok());
    }

    #[test]
    fn unspec_code_returns_none_kind() {
        assert_eq!(StatusError::from_byte(0x09).kind(), None);
    }

    #[test]
    fn error_code_from_u8_covers_spec_range() {
        for b in 0..=7u8 {
            assert!(ErrorCode::from_u8(b).is_some());
        }
        assert!(ErrorCode::from_u8(0x08).is_none());
    }
}
