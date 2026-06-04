#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Instruction {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    RegWrite = 0x04,
    Action = 0x05,
    FactoryReset = 0x06,
    Reboot = 0x08,
    Clear = 0x10,
    ControlTableBackup = 0x20,
    Status = 0x55,
    SyncRead = 0x82,
    SyncWrite = 0x83,
    FastSyncRead = 0x8A,
    BulkRead = 0x92,
    BulkWrite = 0x93,
    FastBulkRead = 0x9A,
}

impl Instruction {
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    pub const fn from_u8(b: u8) -> Option<Self> {
        Some(match b {
            0x01 => Self::Ping,
            0x02 => Self::Read,
            0x03 => Self::Write,
            0x04 => Self::RegWrite,
            0x05 => Self::Action,
            0x06 => Self::FactoryReset,
            0x08 => Self::Reboot,
            0x10 => Self::Clear,
            0x20 => Self::ControlTableBackup,
            0x55 => Self::Status,
            0x82 => Self::SyncRead,
            0x83 => Self::SyncWrite,
            0x8A => Self::FastSyncRead,
            0x92 => Self::BulkRead,
            0x93 => Self::BulkWrite,
            0x9A => Self::FastBulkRead,
            _ => return None,
        })
    }
}
