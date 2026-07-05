//! DXL 2.0 instruction byte and shared request-body entry shapes.

use super::id::Id;

/// DXL 2.0 instruction byte. `Ext(b)` carries any non-standard byte --
/// pattern-match on it to dispatch chip-specific extensions.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    /// Param bytes after the instruction byte. `Status` handles its error
    /// byte separately and is not counted here.
    pub const fn fixed_param_bytes(self) -> usize {
        use Instruction::*;
        match self {
            Read | SyncRead | SyncWrite | FastSyncRead => 4,
            Write | RegWrite => 2,
            FactoryReset => 1,
            _ => 0,
        }
    }

    /// Fixed bytes between the instruction byte and the body: params, plus
    /// `Status`'s error byte.
    pub const fn header_extra_bytes(self) -> usize {
        match self {
            Self::Status => 1,
            _ => self.fixed_param_bytes(),
        }
    }
}

/// Per-slot entry describing one servo's `(id, address, length)` in a Bulk
/// Read / Fast Bulk Read request body.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BulkReadEntry {
    pub id: Id,
    pub address: u16,
    pub length: u16,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_u8_round_trips_every_standard_byte() {
        let standard = [
            Instruction::Ping,
            Instruction::Read,
            Instruction::Write,
            Instruction::RegWrite,
            Instruction::Action,
            Instruction::FactoryReset,
            Instruction::Reboot,
            Instruction::Clear,
            Instruction::ControlTableBackup,
            Instruction::Status,
            Instruction::SyncRead,
            Instruction::SyncWrite,
            Instruction::FastSyncRead,
            Instruction::BulkRead,
            Instruction::BulkWrite,
            Instruction::FastBulkRead,
        ];
        for i in standard {
            assert_eq!(Instruction::from_u8(i.as_u8()), i);
        }
    }

    #[test]
    fn unknown_byte_surfaces_as_ext() {
        assert_eq!(Instruction::from_u8(0xC3), Instruction::Ext(0xC3));
        assert_eq!(Instruction::Ext(0xC3).as_u8(), 0xC3);
    }

    #[test]
    fn header_extra_bytes_status_includes_error_byte() {
        assert_eq!(Instruction::Status.header_extra_bytes(), 1);
        assert_eq!(Instruction::Read.header_extra_bytes(), 4);
        assert_eq!(Instruction::Ping.header_extra_bytes(), 0);
    }

    #[test]
    fn as_u8_matches_spec_constants() {
        assert_eq!(Instruction::Ping.as_u8(), 0x01);
        assert_eq!(Instruction::Status.as_u8(), 0x55);
        assert_eq!(Instruction::SyncWrite.as_u8(), 0x83);
        assert_eq!(Instruction::FastBulkRead.as_u8(), 0x9A);
    }
}
