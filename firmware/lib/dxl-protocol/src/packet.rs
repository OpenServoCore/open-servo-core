use crate::bytes::{Bytes, Overflow};

pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
pub const BROADCAST_ID: u8 = 0xFE;

/// Cap on the wire `Length` field. Bounds how long the parser waits on a
/// phantom header in random traffic; real frames stay well under this.
pub const MAX_LENGTH: usize = 1024;

/// Bytes before the parameter region of any request packet:
/// `HEADER(4) + ID(1) + LENGTH(2) + INSTRUCTION(1)`.
pub const REQUEST_HEADER_BYTES: usize = 8;

/// Bytes before the parameter region of any Status response packet:
/// `HEADER(4) + ID(1) + LENGTH(2) + INSTRUCTION(1) + ERROR(1)`.
pub const RESPONSE_HEADER_BYTES: usize = 9;

/// Trailing CRC bytes on every framed packet.
pub const CRC_BYTES: usize = 2;

/// Per-slave entry width in SyncRead / FastSyncRead request bodies: just `id`.
pub const SYNC_REQUEST_SLOT_BYTES: usize = 1;

/// Per-slave entry width in BulkRead / FastBulkRead request bodies:
/// `id(1) + addr_le16(2) + len_le16(2)`.
pub const BULK_REQUEST_SLOT_BYTES: usize = 5;

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
            data: Bytes::raw(data),
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
            data: Bytes::raw(data),
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

/// OpenServoCore extension — see [`crate::Instruction::Calibrate`].
#[cfg(feature = "osc")]
#[derive(Copy, Clone, Debug)]
pub struct CalibratePacket {
    pub id: u8,
    /// Requested zero-byte payload length in the slave's Status reply.
    /// Bounded `1..=128` by the dispatcher.
    pub count: u16,
}

#[cfg(feature = "osc")]
impl CalibratePacket {
    pub const fn new(id: u8, count: u16) -> Self {
        Self { id, count }
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
            body: Bytes::raw(body),
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
            body: Bytes::raw(body),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct StatusPacket<'a> {
    pub id: u8,
    pub error: u8,
    pub params: Bytes<'a>,
}

impl<'a> StatusPacket<'a> {
    pub const fn new(id: u8, error: u8, params: &'a [u8]) -> Self {
        Self {
            id,
            error,
            params: Bytes::raw(params),
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
            ids: Bytes::raw(ids),
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
            body: Bytes::raw(body),
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
            body: Bytes::raw(body),
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
            body: Bytes::raw(body),
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
            ids: Bytes::raw(ids),
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
            body: Bytes::raw(body),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Packet<'a> {
    Ping(PingPacket),
    Read(ReadPacket),
    Write(WritePacket<'a>),
    RegWrite(RegWritePacket<'a>),
    Action(ActionPacket),
    FactoryReset(FactoryResetPacket),
    Reboot(RebootPacket),
    Clear(ClearPacket<'a>),
    ControlTableBackup(ControlTableBackupPacket<'a>),
    Status(StatusPacket<'a>),
    SyncRead(SyncReadPacket<'a>),
    SyncWrite(SyncWritePacket<'a>),
    BulkRead(BulkReadPacket<'a>),
    BulkWrite(BulkWritePacket<'a>),
    FastSyncRead(FastSyncReadPacket<'a>),
    FastBulkRead(FastBulkReadPacket<'a>),
    #[cfg(feature = "osc")]
    Calibrate(CalibratePacket),
}
