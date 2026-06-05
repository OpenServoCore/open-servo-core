use crate::wire::{Bytes, Overflow};

use super::extension::{Extension, NoExt};

#[derive(Copy, Clone, Debug)]
pub struct PingPacket {
    pub id: u8,
}

impl PingPacket {
    pub const fn new(id: u8) -> Self {
        Self { id }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ReadPacket {
    pub id: u8,
    pub address: u16,
    pub length: u16,
}

impl ReadPacket {
    pub const fn new(id: u8, address: u16, length: u16) -> Self {
        Self {
            id,
            address,
            length,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct WritePacket<'a> {
    pub id: u8,
    pub address: u16,
    pub data: Bytes<'a>,
}

impl<'a> WritePacket<'a> {
    pub const fn new(id: u8, address: u16, data: &'a [u8]) -> Self {
        Self {
            id,
            address,
            data: Bytes::unstuffed(data),
        }
    }

    pub fn copy_data_to_slice(&self, dst: &mut [u8]) -> Result<usize, Overflow> {
        self.data.copy_to_slice(dst)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RegWritePacket<'a> {
    pub id: u8,
    pub address: u16,
    pub data: Bytes<'a>,
}

impl<'a> RegWritePacket<'a> {
    pub const fn new(id: u8, address: u16, data: &'a [u8]) -> Self {
        Self {
            id,
            address,
            data: Bytes::unstuffed(data),
        }
    }

    pub fn copy_data_to_slice(&self, dst: &mut [u8]) -> Result<usize, Overflow> {
        self.data.copy_to_slice(dst)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ActionPacket {
    pub id: u8,
}

impl ActionPacket {
    pub const fn new(id: u8) -> Self {
        Self { id }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FactoryResetPacket {
    pub id: u8,
    pub mode: u8,
}

impl FactoryResetPacket {
    pub const fn new(id: u8, mode: u8) -> Self {
        Self { id, mode }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RebootPacket {
    pub id: u8,
}

impl RebootPacket {
    pub const fn new(id: u8) -> Self {
        Self { id }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ClearPacket<'a> {
    pub id: u8,
    pub body: Bytes<'a>,
}

impl<'a> ClearPacket<'a> {
    pub const fn new(id: u8, body: &'a [u8]) -> Self {
        Self {
            id,
            body: Bytes::unstuffed(body),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ControlTableBackupPacket<'a> {
    pub id: u8,
    pub body: Bytes<'a>,
}

impl<'a> ControlTableBackupPacket<'a> {
    pub const fn new(id: u8, body: &'a [u8]) -> Self {
        Self {
            id,
            body: Bytes::unstuffed(body),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RawStatus<'a> {
    pub id: u8,
    pub error: u8,
    pub params: Bytes<'a>,
}

impl<'a> RawStatus<'a> {
    pub const fn new(id: u8, error: u8, params: &'a [u8]) -> Self {
        Self {
            id,
            error,
            params: Bytes::unstuffed(params),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SyncReadPacket<'a> {
    pub address: u16,
    pub length: u16,
    pub ids: Bytes<'a>,
}

impl<'a> SyncReadPacket<'a> {
    pub const fn new(address: u16, length: u16, ids: &'a [u8]) -> Self {
        Self {
            address,
            length,
            ids: Bytes::unstuffed(ids),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWritePacket<'a> {
    pub address: u16,
    pub length: u16,
    pub body: Bytes<'a>,
}

impl<'a> SyncWritePacket<'a> {
    pub const fn new(address: u16, length: u16, body: &'a [u8]) -> Self {
        Self {
            address,
            length,
            body: Bytes::unstuffed(body),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkReadPacket<'a> {
    pub body: Bytes<'a>,
}

impl<'a> BulkReadPacket<'a> {
    pub const fn new(body: &'a [u8]) -> Self {
        Self {
            body: Bytes::unstuffed(body),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWritePacket<'a> {
    pub body: Bytes<'a>,
}

impl<'a> BulkWritePacket<'a> {
    pub const fn new(body: &'a [u8]) -> Self {
        Self {
            body: Bytes::unstuffed(body),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadPacket<'a> {
    pub address: u16,
    pub length: u16,
    pub ids: Bytes<'a>,
}

impl<'a> FastSyncReadPacket<'a> {
    pub const fn new(address: u16, length: u16, ids: &'a [u8]) -> Self {
        Self {
            address,
            length,
            ids: Bytes::unstuffed(ids),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadPacket<'a> {
    pub body: Bytes<'a>,
}

impl<'a> FastBulkReadPacket<'a> {
    pub const fn new(body: &'a [u8]) -> Self {
        Self {
            body: Bytes::unstuffed(body),
        }
    }
}

/// Decoded DXL frame. The optional `X` parameter plugs in a vendor extension
/// (see [`Extension`]); pure-DXL callers leave it at the [`NoExt`] default,
/// which makes [`Packet::Ext`] statically uninhabited (`Infallible`).
#[derive(Copy, Clone, Debug)]
pub enum Packet<'a, X: Extension = NoExt> {
    Ping(PingPacket),
    Read(ReadPacket),
    Write(WritePacket<'a>),
    RegWrite(RegWritePacket<'a>),
    Action(ActionPacket),
    FactoryReset(FactoryResetPacket),
    Reboot(RebootPacket),
    Clear(ClearPacket<'a>),
    ControlTableBackup(ControlTableBackupPacket<'a>),
    Status(RawStatus<'a>),
    SyncRead(SyncReadPacket<'a>),
    SyncWrite(SyncWritePacket<'a>),
    BulkRead(BulkReadPacket<'a>),
    BulkWrite(BulkWritePacket<'a>),
    FastSyncRead(FastSyncReadPacket<'a>),
    FastBulkRead(FastBulkReadPacket<'a>),
    Ext(X::Variant<'a>),
}
