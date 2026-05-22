#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum StatusError {
    None = 0x00,
    Result = 0x01,
    Instruction = 0x02,
    Crc = 0x03,
    DataRange = 0x04,
    DataLength = 0x05,
    DataLimit = 0x06,
    Access = 0x07,
}

impl StatusError {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

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
