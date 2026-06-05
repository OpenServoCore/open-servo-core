/// DXL 2.0 instruction byte. `Ext(b)` carries any byte outside the standard
/// set — construct via [`Self::from_u8`], which routes unknown bytes there;
/// pattern-match `Ext(b)` to dispatch to an [`InstructionExt`](super::InstructionExt)
/// or [`StatusExt`](super::StatusExt) extension.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Instruction {
    Ping,
    Read,
    Write,
    RegWrite,
    Action,
    FactoryReset,
    Reboot,
    Clear,
    ControlTableBackup,
    Status,
    SyncRead,
    SyncWrite,
    FastSyncRead,
    BulkRead,
    BulkWrite,
    FastBulkRead,
    Ext(u8),
}

impl Instruction {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Ping => 0x01,
            Self::Read => 0x02,
            Self::Write => 0x03,
            Self::RegWrite => 0x04,
            Self::Action => 0x05,
            Self::FactoryReset => 0x06,
            Self::Reboot => 0x08,
            Self::Clear => 0x10,
            Self::ControlTableBackup => 0x20,
            Self::Status => 0x55,
            Self::SyncRead => 0x82,
            Self::SyncWrite => 0x83,
            Self::FastSyncRead => 0x8A,
            Self::BulkRead => 0x92,
            Self::BulkWrite => 0x93,
            Self::FastBulkRead => 0x9A,
            Self::Ext(b) => b,
        }
    }

    /// Total: standard bytes map to their named variant; everything else
    /// becomes [`Self::Ext`] for extension dispatch.
    pub const fn from_u8(b: u8) -> Self {
        match b {
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
            _ => Self::Ext(b),
        }
    }
}
