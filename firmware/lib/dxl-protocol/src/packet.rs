//! Streaming-parser overlay types. Each `#[repr(C)]` struct mirrors the
//! on-the-wire (unstuffed) byte layout exactly, with alignment 1 — so the
//! decoder can cast a `&[u8]` accumulator into any of these structs at any
//! offset.

#![allow(dead_code)]

use crate::instruction::InstructionByte;

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

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PingPacket {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ReadPacket {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct WriteHeader {
    pub header: Header,
    pub addr: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ActionPacket {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct RebootPacket {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FactoryResetPacket {
    pub header: Header,
    pub mode: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct StatusHeader {
    pub header: Header,
    pub error: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct SyncReadHeader {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct SyncWriteHeader {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkReadHeader {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkWriteHeader {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadHeader {
    pub header: Header,
    pub addr: U16Le,
    pub length: U16Le,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadHeader {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct RawHeader {
    pub header: Header,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkReadEntry {
    pub id: u8,
    pub addr: U16Le,
    pub length: U16Le,
}

// ───── variable-length wrappers ─────

#[derive(Copy, Clone, Debug)]
pub struct WritePacket<'a> {
    pub header: &'a WriteHeader,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct StatusPacket<'a> {
    pub header: &'a StatusHeader,
    pub params: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncReadPacket<'a> {
    pub header: &'a SyncReadHeader,
    pub ids: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadPacket<'a> {
    pub header: &'a FastSyncReadHeader,
    pub ids: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct BulkReadPacket<'a> {
    pub header: &'a BulkReadHeader,
    pub entries: &'a [BulkReadEntry],
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadPacket<'a> {
    pub header: &'a FastBulkReadHeader,
    pub entries: &'a [BulkReadEntry],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWritePacket<'a> {
    pub header: &'a SyncWriteHeader,
    pub body: &'a [u8],
}

impl<'a> SyncWritePacket<'a> {
    pub fn entries(&self) -> SyncWriteEntries<'a> {
        SyncWriteEntries {
            cursor: self.body,
            data_len: self.header.length.get() as usize,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWriteEntry<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncWriteEntries<'a> {
    cursor: &'a [u8],
    data_len: usize,
}

impl<'a> Iterator for SyncWriteEntries<'a> {
    type Item = SyncWriteEntry<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        let stride = 1 + self.data_len;
        if self.cursor.len() < stride {
            return None;
        }
        let (head, rest) = self.cursor.split_at(stride);
        self.cursor = rest;
        Some(SyncWriteEntry {
            id: head[0],
            data: &head[1..],
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWritePacket<'a> {
    pub header: &'a BulkWriteHeader,
    pub body: &'a [u8],
}

impl<'a> BulkWritePacket<'a> {
    pub fn entries(&self) -> BulkWriteEntries<'a> {
        BulkWriteEntries { cursor: self.body }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWriteEntry<'a> {
    pub id: u8,
    pub addr: u16,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct BulkWriteEntries<'a> {
    cursor: &'a [u8],
}

impl<'a> Iterator for BulkWriteEntries<'a> {
    type Item = BulkWriteEntry<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.cursor.len() < 5 {
            return None;
        }
        let id = self.cursor[0];
        let addr = u16::from_le_bytes([self.cursor[1], self.cursor[2]]);
        let length = u16::from_le_bytes([self.cursor[3], self.cursor[4]]) as usize;
        let total = 5 + length;
        if self.cursor.len() < total {
            return None;
        }
        let (head, rest) = self.cursor.split_at(total);
        self.cursor = rest;
        Some(BulkWriteEntry {
            id,
            addr,
            data: &head[5..],
        })
    }
}

#[derive(Copy, Clone, Debug)]
pub struct RawPacket<'a> {
    pub header: &'a RawHeader,
    pub params: &'a [u8],
}

// ───── status error byte ─────

/// DXL 2.0 Status error byte. Bit 7 = sticky hardware Alert; bits 0..=6 =
/// error code (only `0..=7` are spec-defined). `#[repr(transparent)]` over
/// `u8` keeps wire layout identical and lets it sit inside `#[repr(C)]`
/// overlay structs.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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

    /// Shorthand for `new(false, code)`.
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

    /// Typed view of bits 0..=6; `None` for reserved/unknown values.
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

// ───── status payload overlays ─────

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct PingStatus {
    pub model: U16Le,
    pub fw_version: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadStatus<'a> {
    pub error: StatusError,
    pub payload: &'a [u8],
}

impl<'a> FastSyncReadStatus<'a> {
    pub fn slots(&self, slot_length: u16) -> FastSyncSlotIter<'a> {
        FastSyncSlotIter {
            cursor: self.payload,
            slot_length: slot_length as usize,
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadStatus<'a> {
    pub error: StatusError,
    pub payload: &'a [u8],
}

impl<'a> FastBulkReadStatus<'a> {
    pub fn slots<L: IntoIterator<Item = u16>>(
        &self,
        lengths: L,
    ) -> FastBulkSlotIter<'a, L::IntoIter> {
        FastBulkSlotIter {
            cursor: self.payload,
            lengths: lengths.into_iter(),
            slot0_error: self.error,
            slot_index: 0,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Slot<'a> {
    pub id: u8,
    pub error: StatusError,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncSlotIter<'a> {
    cursor: &'a [u8],
    slot_length: usize,
    slot0_error: StatusError,
    slot_index: usize,
}

impl<'a> Iterator for FastSyncSlotIter<'a> {
    type Item = Slot<'a>;
    fn next(&mut self) -> Option<Slot<'a>> {
        next_slot(
            &mut self.cursor,
            self.slot_length,
            self.slot0_error,
            &mut self.slot_index,
        )
    }
}

pub struct FastBulkSlotIter<'a, L: Iterator<Item = u16>> {
    cursor: &'a [u8],
    lengths: L,
    slot0_error: StatusError,
    slot_index: usize,
}

impl<'a, L: Iterator<Item = u16>> Iterator for FastBulkSlotIter<'a, L> {
    type Item = Slot<'a>;
    fn next(&mut self) -> Option<Slot<'a>> {
        let len = self.lengths.next()? as usize;
        next_slot(
            &mut self.cursor,
            len,
            self.slot0_error,
            &mut self.slot_index,
        )
    }
}

fn next_slot<'a>(
    cursor: &mut &'a [u8],
    slot_length: usize,
    slot0_error: StatusError,
    slot_index: &mut usize,
) -> Option<Slot<'a>> {
    let (id, error) = if *slot_index == 0 {
        if cursor.is_empty() {
            return None;
        }
        let (id, rest) = cursor.split_first()?;
        *cursor = rest;
        (*id, slot0_error)
    } else {
        if cursor.len() < 2 {
            return None;
        }
        let error = StatusError::from_byte(cursor[0]);
        let id = cursor[1];
        *cursor = &cursor[2..];
        (id, error)
    };
    if cursor.len() < slot_length {
        return None;
    }
    let (data, rest) = cursor.split_at(slot_length);
    *cursor = rest;
    *slot_index += 1;
    Some(Slot { id, error, data })
}

// ───── status-context dispatch ─────

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RequestKind {
    Ping,
    Read,
    Write,
    RegWrite,
    Action,
    Reboot,
    FactoryReset,
    SyncRead,
    SyncWrite,
    BulkRead,
    BulkWrite,
    FastSyncRead,
    FastBulkRead,
}

/// Unified Status — same shape for parser output (`StatusPacket::interpret`)
/// and writer input (encode side). `id` and `error` are baked into each
/// variant so the encoder can construct a Status without a parent
/// `StatusHeader` to borrow from.
#[derive(Copy, Clone, Debug)]
pub enum Status<'a> {
    /// Write-style ack — no payload. Used for Write/RegWrite/Action/
    /// Reboot/FactoryReset/SyncWrite/BulkWrite replies; also the
    /// short-payload-Ping fallback. `error == StatusError::OK` on success.
    Empty { id: u8, error: StatusError },

    /// Ping reply — 3-byte fixed-shape payload. Owned (not borrowed) so
    /// the same variant covers encode and decode.
    Ping {
        id: u8,
        error: StatusError,
        status: PingStatus,
    },

    /// Read/SyncRead/BulkRead reply — opaque register bytes.
    Read {
        id: u8,
        error: StatusError,
        data: &'a [u8],
    },

    /// Fast Sync Read coalesced reply (master-side decode). `status` holds
    /// slot 0's error byte and the multi-slot payload; iterate via
    /// `status.slots(slot_length)`. Writer rejects this variant — fast
    /// replies are emitted one slot at a time via `Writer::write_slot`.
    FastSyncRead {
        id: u8,
        status: FastSyncReadStatus<'a>,
    },

    /// Fast Bulk Read counterpart.
    FastBulkRead {
        id: u8,
        status: FastBulkReadStatus<'a>,
    },

    /// Encode-only escape hatch — arbitrary Status frame with chip-defined
    /// payload bytes (e.g. OSC `Calibrate` reply). `interpret()` never
    /// produces this variant.
    Raw {
        id: u8,
        error: StatusError,
        payload: &'a [u8],
    },
}

impl<'a> StatusPacket<'a> {
    #[inline]
    pub fn error(&self) -> StatusError {
        StatusError::from_byte(self.header.error)
    }

    pub fn interpret(self, req: RequestKind) -> Status<'a> {
        let id = self.header.header.id;
        let error = self.error();
        match req {
            RequestKind::Write
            | RequestKind::RegWrite
            | RequestKind::Action
            | RequestKind::Reboot
            | RequestKind::FactoryReset
            | RequestKind::SyncWrite
            | RequestKind::BulkWrite => Status::Empty { id, error },

            RequestKind::Ping => {
                if self.params.len() < core::mem::size_of::<PingStatus>() {
                    Status::Empty { id, error }
                } else {
                    // SAFETY: PingStatus is repr(C), align 1, all u8/U16Le
                    // fields. Length checked above; `self.params` is
                    // initialized for its full length.
                    let p = unsafe { *(self.params.as_ptr() as *const PingStatus) };
                    Status::Ping { id, error, status: p }
                }
            }

            RequestKind::Read | RequestKind::SyncRead | RequestKind::BulkRead => {
                Status::Read { id, error, data: self.params }
            }

            RequestKind::FastSyncRead => Status::FastSyncRead {
                id,
                status: FastSyncReadStatus { error, payload: self.params },
            },

            RequestKind::FastBulkRead => Status::FastBulkRead {
                id,
                status: FastBulkReadStatus { error, payload: self.params },
            },
        }
    }
}

// ───── wire-shape slot accessors ─────
//
// Wire-position math for multi-slot Sync/Bulk Read requests: given an `id`,
// find which slot in the wire layout it occupies and how many response bytes
// precede it in the coalesced reply. The math is identical whether the
// caller is the addressed slave (composing its own slot) or a master/sniffer
// (decoding a captured response) — these are protocol-shape conveniences,
// not chip policy.

use crate::{CRC_BYTES, FAST_RESPONSE_SLOT_BYTES, FAST_RESPONSE_SLOT0_BYTES, RESPONSE_HEADER_BYTES};

/// Position of our slot in the coalesced Fast Status response. Variants that
/// emit the response header carry the DXL `Length` field for the whole
/// multi-slot frame; `Last` carries the chain CRC.
///
/// Chain producers (slaves emitting `Last` in a multi-slave coalesced reply)
/// don't know the running CRC at frame-build time — it depends on the wire
/// bytes of every prior slave. They emit a placeholder value here and let
/// the chip's fire-time ISR overwrite the trailing two bytes with the real
/// CRC. Callers that *do* know the CRC at build time (single-slot tests,
/// replay tools, sniffers) pass the real value.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SlotPosition {
    /// Single-slot response — emits full header and locally-computed CRC.
    Only { packet_length: u16 },
    /// First of N — emits header + body, no CRC (successors continue).
    First { packet_length: u16 },
    /// Body only.
    Middle,
    /// Body + caller-supplied CRC bytes.
    Last { crc: u16 },
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SyncSlotInfo {
    pub index: usize,
    pub bytes_before: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct BulkSlotInfo {
    pub index: usize,
    pub address: u16,
    pub length: u16,
    pub bytes_before: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FastSlotInfo {
    pub our_slot: usize,
    pub n_slots: usize,
    pub address: u16,
    pub length: u16,
    pub packet_length: u16,
    pub bytes_before: u32,
}

impl FastSlotInfo {
    pub const fn position(&self) -> SlotPosition {
        match (self.our_slot, self.n_slots) {
            (0, 1) => SlotPosition::Only {
                packet_length: self.packet_length,
            },
            (0, _) => SlotPosition::First {
                packet_length: self.packet_length,
            },
            (k, n) if k + 1 == n => SlotPosition::Last { crc: 0 },
            _ => SlotPosition::Middle,
        }
    }
}

impl<'a> SyncReadPacket<'a> {
    pub fn find_slot(&self, id: u8) -> Option<SyncSlotInfo> {
        let index = self.ids.iter().position(|b| *b == id)?;
        let length = self.header.length.get();
        let per_slot = (RESPONSE_HEADER_BYTES as u32)
            .saturating_add(length as u32)
            .saturating_add(CRC_BYTES as u32);
        Some(SyncSlotInfo {
            index,
            bytes_before: per_slot.saturating_mul(index as u32),
        })
    }
}

impl<'a> BulkReadPacket<'a> {
    pub fn find_slot(&self, id: u8) -> Option<BulkSlotInfo> {
        let mut bytes_before = 0u32;
        for (index, entry) in self.entries.iter().enumerate() {
            let length = entry.length.get();
            if entry.id == id {
                return Some(BulkSlotInfo {
                    index,
                    address: entry.addr.get(),
                    length,
                    bytes_before,
                });
            }
            bytes_before = bytes_before
                .saturating_add(RESPONSE_HEADER_BYTES as u32)
                .saturating_add(length as u32)
                .saturating_add(CRC_BYTES as u32);
        }
        None
    }
}

impl<'a> FastSyncReadPacket<'a> {
    /// `max_slots` bounds how many ids the walk inspects; callers with a
    /// fixed slot budget pass it in so a malformed request can't unbalance
    /// downstream loops. Returns `None` if our id isn't in the list or the
    /// list exceeds `max_slots`.
    pub fn find_slot(&self, id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let length = self.header.length.get();
        let mut our_slot = None;
        let mut n_slots = 0usize;
        for (i, &slot_id) in self.ids.iter().enumerate() {
            if i >= max_slots {
                return None;
            }
            if slot_id == id && our_slot.is_none() {
                our_slot = Some(i);
            }
            n_slots = i + 1;
        }
        let our_slot = our_slot?;
        let payload = length as u32;
        let bytes_before = if our_slot == 0 {
            0
        } else {
            FAST_RESPONSE_SLOT0_BYTES as u32
                + payload
                + (our_slot as u32 - 1) * (FAST_RESPONSE_SLOT_BYTES as u32 + payload)
        };
        let packet_length = 3u16 + (n_slots as u16) * (2 + length);
        Some(FastSlotInfo {
            our_slot,
            n_slots,
            address: self.header.addr.get(),
            length,
            packet_length,
            bytes_before,
        })
    }
}

impl<'a> FastBulkReadPacket<'a> {
    /// `max_slots` bounds how many entries the walk inspects.
    pub fn find_slot(&self, id: u8, max_slots: usize) -> Option<FastSlotInfo> {
        let mut found: Option<(usize, u16, u16, u32)> = None;
        let mut n_slots = 0usize;
        let mut total_payload = 0u32;
        let mut bytes_before = 0u32;
        for (i, entry) in self.entries.iter().enumerate() {
            if i >= max_slots {
                return None;
            }
            let entry_len = entry.length.get();
            if entry.id == id && found.is_none() {
                found = Some((i, entry.addr.get(), entry_len, bytes_before));
            }
            let prefix = if i == 0 {
                FAST_RESPONSE_SLOT0_BYTES as u32
            } else {
                FAST_RESPONSE_SLOT_BYTES as u32
            };
            bytes_before = bytes_before
                .saturating_add(prefix)
                .saturating_add(entry_len as u32);
            total_payload = total_payload.saturating_add(entry_len as u32);
            n_slots = i + 1;
        }
        let (our_slot, address, length, bytes_before) = found?;
        let packet_length = (3u32 + (n_slots as u32) * 2 + total_payload) as u16;
        Some(FastSlotInfo {
            our_slot,
            n_slots,
            address,
            length,
            packet_length,
            bytes_before,
        })
    }
}

impl<'a> SyncWritePacket<'a> {
    /// Returns the typed entry for `id`, or `None` if not present.
    pub fn find_entry(&self, id: u8) -> Option<SyncWriteEntry<'a>> {
        self.entries().find(|e| e.id == id)
    }
}

impl<'a> BulkWritePacket<'a> {
    /// Returns the typed entry for `id`, or `None` if not present.
    pub fn find_entry(&self, id: u8) -> Option<BulkWriteEntry<'a>> {
        self.entries().find(|e| e.id == id)
    }
}

// ───── top-level decode enum ─────

#[derive(Copy, Clone, Debug)]
pub enum Packet<'a> {
    Ping(&'a PingPacket),
    Read(&'a ReadPacket),
    Write(WritePacket<'a>),
    RegWrite(WritePacket<'a>),
    Action(&'a ActionPacket),
    Reboot(&'a RebootPacket),
    FactoryReset(&'a FactoryResetPacket),
    Status(StatusPacket<'a>),
    SyncRead(SyncReadPacket<'a>),
    SyncWrite(SyncWritePacket<'a>),
    BulkRead(BulkReadPacket<'a>),
    BulkWrite(BulkWritePacket<'a>),
    FastSyncRead(FastSyncReadPacket<'a>),
    FastBulkRead(FastBulkReadPacket<'a>),
    Raw(RawPacket<'a>),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instruction::Instruction;
    use core::mem::{align_of, offset_of, size_of};

    // ─── alignment: every overlay struct must be align 1 so the decoder
    //     can cast its `[MaybeUninit<u8>; M]` accumulator at any offset. ───

    #[test]
    fn alignment_is_one_everywhere() {
        assert_eq!(align_of::<U16Le>(), 1);
        assert_eq!(align_of::<Header>(), 1);
        assert_eq!(align_of::<PingPacket>(), 1);
        assert_eq!(align_of::<ReadPacket>(), 1);
        assert_eq!(align_of::<WriteHeader>(), 1);
        assert_eq!(align_of::<ActionPacket>(), 1);
        assert_eq!(align_of::<RebootPacket>(), 1);
        assert_eq!(align_of::<FactoryResetPacket>(), 1);
        assert_eq!(align_of::<StatusHeader>(), 1);
        assert_eq!(align_of::<SyncReadHeader>(), 1);
        assert_eq!(align_of::<SyncWriteHeader>(), 1);
        assert_eq!(align_of::<BulkReadHeader>(), 1);
        assert_eq!(align_of::<BulkWriteHeader>(), 1);
        assert_eq!(align_of::<FastSyncReadHeader>(), 1);
        assert_eq!(align_of::<FastBulkReadHeader>(), 1);
        assert_eq!(align_of::<RawHeader>(), 1);
        assert_eq!(align_of::<BulkReadEntry>(), 1);
        assert_eq!(align_of::<PingStatus>(), 1);
    }

    // ─── sizes match the on-the-wire (unstuffed) byte layout. ───

    #[test]
    fn sizes_match_wire() {
        assert_eq!(size_of::<U16Le>(), 2);
        assert_eq!(size_of::<Header>(), 8);
        assert_eq!(size_of::<PingPacket>(), 8);
        assert_eq!(size_of::<ReadPacket>(), 12);
        assert_eq!(size_of::<WriteHeader>(), 10);
        assert_eq!(size_of::<ActionPacket>(), 8);
        assert_eq!(size_of::<RebootPacket>(), 8);
        assert_eq!(size_of::<FactoryResetPacket>(), 9);
        assert_eq!(size_of::<StatusHeader>(), 9);
        assert_eq!(size_of::<SyncReadHeader>(), 12);
        assert_eq!(size_of::<SyncWriteHeader>(), 12);
        assert_eq!(size_of::<BulkReadHeader>(), 8);
        assert_eq!(size_of::<BulkWriteHeader>(), 8);
        assert_eq!(size_of::<FastSyncReadHeader>(), 12);
        assert_eq!(size_of::<FastBulkReadHeader>(), 8);
        assert_eq!(size_of::<RawHeader>(), 8);
        assert_eq!(size_of::<BulkReadEntry>(), 5);
        assert_eq!(size_of::<PingStatus>(), 3);
    }

    // ─── field offsets match the on-the-wire byte positions. ───

    #[test]
    fn header_offsets() {
        assert_eq!(offset_of!(Header, sync), 0);
        assert_eq!(offset_of!(Header, id), 4);
        assert_eq!(offset_of!(Header, len), 5);
        assert_eq!(offset_of!(Header, instruction), 7);
    }

    #[test]
    fn read_packet_offsets() {
        assert_eq!(offset_of!(ReadPacket, header), 0);
        assert_eq!(offset_of!(ReadPacket, addr), 8);
        assert_eq!(offset_of!(ReadPacket, length), 10);
    }

    #[test]
    fn write_header_offsets() {
        assert_eq!(offset_of!(WriteHeader, header), 0);
        assert_eq!(offset_of!(WriteHeader, addr), 8);
    }

    #[test]
    fn factory_reset_offsets() {
        assert_eq!(offset_of!(FactoryResetPacket, header), 0);
        assert_eq!(offset_of!(FactoryResetPacket, mode), 8);
    }

    #[test]
    fn status_header_offsets() {
        assert_eq!(offset_of!(StatusHeader, header), 0);
        assert_eq!(offset_of!(StatusHeader, error), 8);
    }

    #[test]
    fn sync_read_header_offsets() {
        assert_eq!(offset_of!(SyncReadHeader, header), 0);
        assert_eq!(offset_of!(SyncReadHeader, addr), 8);
        assert_eq!(offset_of!(SyncReadHeader, length), 10);
    }

    #[test]
    fn sync_write_header_offsets() {
        assert_eq!(offset_of!(SyncWriteHeader, header), 0);
        assert_eq!(offset_of!(SyncWriteHeader, addr), 8);
        assert_eq!(offset_of!(SyncWriteHeader, length), 10);
    }

    #[test]
    fn fast_sync_read_header_offsets() {
        assert_eq!(offset_of!(FastSyncReadHeader, header), 0);
        assert_eq!(offset_of!(FastSyncReadHeader, addr), 8);
        assert_eq!(offset_of!(FastSyncReadHeader, length), 10);
    }

    #[test]
    fn bulk_read_entry_offsets() {
        assert_eq!(offset_of!(BulkReadEntry, id), 0);
        assert_eq!(offset_of!(BulkReadEntry, addr), 1);
        assert_eq!(offset_of!(BulkReadEntry, length), 3);
    }

    #[test]
    fn ping_status_offsets() {
        assert_eq!(offset_of!(PingStatus, model), 0);
        assert_eq!(offset_of!(PingStatus, fw_version), 2);
    }

    // ─── U16Le round-trip. ───

    #[test]
    fn u16le_round_trip() {
        let v = U16Le::from_u16(0x1234);
        assert_eq!(v.lo, 0x34);
        assert_eq!(v.hi, 0x12);
        assert_eq!(v.get(), 0x1234);
    }

    // ─── overlay-onto-bytes round-trip: cast a hand-built byte array into
    //     each overlay struct and confirm the fields match. ───

    #[test]
    fn read_packet_overlay() {
        let bytes = [
            0xFF, 0xFF, 0xFD, 0x00, // sync
            0x07, // id
            0x07, 0x00,       // len = 7
            Instruction::Read.as_u8(), // instruction
            0x84, 0x00, // addr = 0x0084
            0x04, 0x00, // length = 4
        ];
        let p: &ReadPacket = unsafe { &*(bytes.as_ptr() as *const ReadPacket) };
        assert_eq!(p.header.sync, [0xFF, 0xFF, 0xFD, 0x00]);
        assert_eq!(p.header.id, 0x07);
        assert_eq!(p.header.len.get(), 7);
        assert_eq!(p.header.instruction.kind(), Instruction::Read);
        assert_eq!(p.addr.get(), 0x0084);
        assert_eq!(p.length.get(), 4);
    }

    #[test]
    fn bulk_read_entries_overlay() {
        let body: [u8; 10] = [
            0x01, 0x10, 0x00, 0x04, 0x00, // id=1 addr=0x0010 len=4
            0x02, 0x20, 0x00, 0x08, 0x00, // id=2 addr=0x0020 len=8
        ];
        let entries: &[BulkReadEntry] =
            unsafe { core::slice::from_raw_parts(body.as_ptr() as *const BulkReadEntry, 2) };
        assert_eq!(entries[0].id, 1);
        assert_eq!(entries[0].addr.get(), 0x0010);
        assert_eq!(entries[0].length.get(), 4);
        assert_eq!(entries[1].id, 2);
        assert_eq!(entries[1].addr.get(), 0x0020);
        assert_eq!(entries[1].length.get(), 8);
    }

    // ─── variable-stride iterators. ───

    fn make_sync_write(buf: &[u8]) -> SyncWritePacket<'_> {
        let header: &SyncWriteHeader = unsafe { &*(buf.as_ptr() as *const SyncWriteHeader) };
        SyncWritePacket {
            header,
            body: &buf[size_of::<SyncWriteHeader>()..],
        }
    }

    #[test]
    fn sync_write_entries_walks_full_entries() {
        // Header: sync + id + len(=12) + instr + addr(=0x50) + length(=3).
        // Body: two stride-4 entries [id, d0, d1, d2].
        let buf: [u8; 12 + 8] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x0C,
            0x00,
            Instruction::SyncWrite.as_u8(),
            0x50,
            0x00,
            0x03,
            0x00,
            1,
            10,
            11,
            12, //
            2,
            20,
            21,
            22,
        ];
        let p = make_sync_write(&buf);
        let mut it = p.entries();
        let e0 = it.next().unwrap();
        assert_eq!(e0.id, 1);
        assert_eq!(e0.data, &[10, 11, 12]);
        let e1 = it.next().unwrap();
        assert_eq!(e1.id, 2);
        assert_eq!(e1.data, &[20, 21, 22]);
        assert!(it.next().is_none());
    }

    #[test]
    fn sync_write_entries_drops_trailing_partial() {
        let buf: [u8; 12 + 6] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x0C,
            0x00,
            Instruction::SyncWrite.as_u8(),
            0x50,
            0x00,
            0x03,
            0x00,
            1,
            10,
            11,
            12, //
            2,
            20, // truncated
        ];
        let p = make_sync_write(&buf);
        assert_eq!(p.entries().count(), 1);
    }

    fn make_bulk_write<'a>(buf: &'a [u8]) -> BulkWritePacket<'a> {
        let header: &BulkWriteHeader = unsafe { &*(buf.as_ptr() as *const BulkWriteHeader) };
        BulkWritePacket {
            header,
            body: &buf[size_of::<BulkWriteHeader>()..],
        }
    }

    #[test]
    fn bulk_write_entries_walks_varying_lengths() {
        let buf: [u8; 8 + 17] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x11,
            0x00,
            Instruction::BulkWrite.as_u8(), //
            1,
            0x50,
            0x00,
            0x03,
            0x00,
            0xAA,
            0xBB,
            0xCC, //
            2,
            0x54,
            0x02,
            0x04,
            0x00,
            0x10,
            0x20,
            0x30,
            0x40,
        ];
        let p = make_bulk_write(&buf);
        let mut it = p.entries();
        let e0 = it.next().unwrap();
        assert_eq!(e0.id, 1);
        assert_eq!(e0.addr, 0x0050);
        assert_eq!(e0.data, &[0xAA, 0xBB, 0xCC]);
        let e1 = it.next().unwrap();
        assert_eq!(e1.id, 2);
        assert_eq!(e1.addr, 0x0254);
        assert_eq!(e1.data, &[0x10, 0x20, 0x30, 0x40]);
        assert!(it.next().is_none());
    }

    #[test]
    fn bulk_write_entries_drops_truncated_data() {
        let buf: [u8; 8 + 7] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0xFE,
            0x07,
            0x00,
            Instruction::BulkWrite.as_u8(), //
            3,
            0x00,
            0x01,
            0x04,
            0x00,
            0xAA,
            0xBB, // claims 4, has 2
        ];
        let p = make_bulk_write(&buf);
        assert!(p.entries().next().is_none());
    }

    // ─── Fast Sync/Bulk Read status slot iteration. ───

    #[test]
    fn fast_sync_status_slots_walk_uniform_length() {
        // 3 slots, length=4. payload = id0 + data0 + err1 + id1 + data1 + err2 + id2 + data2
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, //
            0x02, 20, 0xB0, 0xB1, 0xB2, 0xB3, //
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3,
        ];
        let s = FastSyncReadStatus {
            error: StatusError::from_byte(0x01),
            payload: &payload,
        };
        let collected: heapless::Vec<(u8, StatusError, &[u8]), 4> =
            s.slots(4).map(|sl| (sl.id, sl.error, sl.data)).collect();
        assert_eq!(collected.len(), 3);
        assert_eq!(
            collected[0],
            (10, StatusError::from_byte(0x01), &[0xA0, 0xA1, 0xA2, 0xA3][..])
        );
        assert_eq!(
            collected[1],
            (20, StatusError::from_byte(0x02), &[0xB0, 0xB1, 0xB2, 0xB3][..])
        );
        assert_eq!(
            collected[2],
            (30, StatusError::OK, &[0xC0, 0xC1, 0xC2, 0xC3][..])
        );
    }

    #[test]
    fn fast_sync_status_slots_stop_on_truncation() {
        let payload = [10, 0xA0, 0xA1, 0xA2, 0xA3, 0x00, 20, 0xB0]; // slot 1 missing 3 bytes
        let s = FastSyncReadStatus {
            error: StatusError::OK,
            payload: &payload,
        };
        assert_eq!(s.slots(4).count(), 1);
    }

    #[test]
    fn fast_bulk_status_slots_varying_lengths() {
        let payload = [
            10, 0xA0, 0xA1, 0xA2, 0xA3, //
            0x02, 20, 0xB0, 0xB1, //
            0x00, 30, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,
        ];
        let s = FastBulkReadStatus {
            error: StatusError::from_byte(0x05),
            payload: &payload,
        };
        let collected: heapless::Vec<(u8, StatusError, usize), 4> = s
            .slots([4u16, 2, 6].iter().copied())
            .map(|sl| (sl.id, sl.error, sl.data.len()))
            .collect();
        assert_eq!(collected.len(), 3);
        assert_eq!(collected[0], (10, StatusError::from_byte(0x05), 4));
        assert_eq!(collected[1], (20, StatusError::from_byte(0x02), 2));
        assert_eq!(collected[2], (30, StatusError::OK, 6));
    }

    // ─── interpret() dispatches every RequestKind to the right Status variant. ───

    fn make_status<'a>(buf: &'a [u8]) -> StatusPacket<'a> {
        let header: &StatusHeader = unsafe { &*(buf.as_ptr() as *const StatusHeader) };
        StatusPacket {
            header,
            params: &buf[size_of::<StatusHeader>()..],
        }
    }

    #[test]
    fn interpret_ping_decodes_overlay() {
        let buf: [u8; 9 + 3] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x06,
            0x00,
            Instruction::Status.as_u8(),
            0x00, //
            0xFC,
            0x03,
            0x2A, // model=1020, fw=0x2A
        ];
        let s = make_status(&buf);
        assert_eq!(s.error(), StatusError::OK);
        match s.interpret(RequestKind::Ping) {
            Status::Ping { id, error, status } => {
                assert_eq!(id, 0x01);
                assert_eq!(error, StatusError::OK);
                assert_eq!(status.model.get(), 1020);
                assert_eq!(status.fw_version, 0x2A);
            }
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn interpret_ping_short_payload_falls_back_to_empty() {
        let buf: [u8; 9 + 1] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x04,
            0x00,
            Instruction::Status.as_u8(),
            0x00,
            0xFC,
        ];
        let s = make_status(&buf);
        assert!(matches!(s.interpret(RequestKind::Ping), Status::Empty { .. }));
    }

    #[test]
    fn interpret_read_wraps_payload_slice() {
        let buf: [u8; 9 + 4] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x07,
            0x00,
            Instruction::Status.as_u8(),
            0x00, //
            0x01,
            0x02,
            0x03,
            0x04,
        ];
        let s = make_status(&buf);
        for req in [
            RequestKind::Read,
            RequestKind::SyncRead,
            RequestKind::BulkRead,
        ] {
            match s.interpret(req) {
                Status::Read { id, error, data } => {
                    assert_eq!(id, 0x01);
                    assert_eq!(error, StatusError::OK);
                    assert_eq!(data, &[0x01, 0x02, 0x03, 0x04]);
                }
                other => panic!("{req:?} expected Read, got {other:?}"),
            }
        }
    }

    #[test]
    fn interpret_write_family_is_empty() {
        let buf: [u8; 9] = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, Instruction::Status.as_u8(), 0x00];
        let s = make_status(&buf);
        for req in [
            RequestKind::Write,
            RequestKind::RegWrite,
            RequestKind::Action,
            RequestKind::Reboot,
            RequestKind::FactoryReset,
            RequestKind::SyncWrite,
            RequestKind::BulkWrite,
        ] {
            assert!(
                matches!(s.interpret(req), Status::Empty { .. }),
                "{req:?} should produce Status::Empty"
            );
        }
    }

    #[test]
    fn interpret_fast_sync_carries_error_and_payload() {
        let buf: [u8; 9 + 5] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x08,
            0x00,
            Instruction::Status.as_u8(),
            0x07, //
            10,
            0xAA,
            0xBB,
            0xCC,
            0xDD,
        ];
        let s = make_status(&buf);
        match s.interpret(RequestKind::FastSyncRead) {
            Status::FastSyncRead { id, status } => {
                assert_eq!(id, 0x01);
                assert_eq!(status.error, StatusError::code(ErrorCode::Access));
                assert_eq!(status.payload, &[10, 0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected FastSyncRead, got {other:?}"),
        }
    }

    // ─── wire-shape slot accessors ───

    fn make_sync_read(buf: &[u8]) -> SyncReadPacket<'_> {
        let header: &SyncReadHeader = unsafe { &*(buf.as_ptr() as *const SyncReadHeader) };
        SyncReadPacket {
            header,
            ids: &buf[size_of::<SyncReadHeader>()..],
        }
    }

    fn make_bulk_read(buf: &[u8], n_entries: usize) -> BulkReadPacket<'_> {
        let header: &BulkReadHeader = unsafe { &*(buf.as_ptr() as *const BulkReadHeader) };
        let entries: &[BulkReadEntry] = unsafe {
            core::slice::from_raw_parts(
                buf.as_ptr().add(size_of::<BulkReadHeader>()) as *const BulkReadEntry,
                n_entries,
            )
        };
        BulkReadPacket { header, entries }
    }

    fn make_fast_sync_read(buf: &[u8]) -> FastSyncReadPacket<'_> {
        let header: &FastSyncReadHeader =
            unsafe { &*(buf.as_ptr() as *const FastSyncReadHeader) };
        FastSyncReadPacket {
            header,
            ids: &buf[size_of::<FastSyncReadHeader>()..],
        }
    }

    fn make_fast_bulk_read(buf: &[u8], n_entries: usize) -> FastBulkReadPacket<'_> {
        let header: &FastBulkReadHeader =
            unsafe { &*(buf.as_ptr() as *const FastBulkReadHeader) };
        let entries: &[BulkReadEntry] = unsafe {
            core::slice::from_raw_parts(
                buf.as_ptr().add(size_of::<FastBulkReadHeader>()) as *const BulkReadEntry,
                n_entries,
            )
        };
        FastBulkReadPacket { header, entries }
    }

    #[test]
    fn sync_read_find_slot_locates_id_and_offset() {
        // length=4, ids=[1,2,3]
        let buf: [u8; 12 + 3] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, Instruction::SyncRead.as_u8(), //
            0x50, 0x00, 0x04, 0x00, //
            1, 2, 3,
        ];
        let p = make_sync_read(&buf);
        assert_eq!(p.find_slot(1).unwrap(), SyncSlotInfo { index: 0, bytes_before: 0 });
        // per-slot = header(9) + length(4) + crc(2) = 15
        assert_eq!(p.find_slot(2).unwrap(), SyncSlotInfo { index: 1, bytes_before: 15 });
        assert_eq!(p.find_slot(3).unwrap(), SyncSlotInfo { index: 2, bytes_before: 30 });
        assert!(p.find_slot(9).is_none());
    }

    #[test]
    fn bulk_read_find_slot_carries_per_entry_addr_and_length() {
        let buf: [u8; 8 + 10] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0D, 0x00, Instruction::BulkRead.as_u8(), //
            1, 0x10, 0x00, 0x04, 0x00, // id=1 addr=0x0010 len=4
            2, 0x20, 0x00, 0x08, 0x00, // id=2 addr=0x0020 len=8
        ];
        let p = make_bulk_read(&buf, 2);
        let i0 = p.find_slot(1).unwrap();
        assert_eq!(i0.index, 0);
        assert_eq!(i0.address, 0x0010);
        assert_eq!(i0.length, 4);
        assert_eq!(i0.bytes_before, 0);
        let i1 = p.find_slot(2).unwrap();
        assert_eq!(i1.index, 1);
        assert_eq!(i1.address, 0x0020);
        assert_eq!(i1.length, 8);
        // header(9) + 4 + crc(2) = 15
        assert_eq!(i1.bytes_before, 15);
        assert!(p.find_slot(9).is_none());
    }

    #[test]
    fn fast_sync_find_slot_populates_packet_length_and_offset() {
        // length=2, ids=[9,7,0]
        let buf: [u8; 12 + 3] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, Instruction::FastSyncRead.as_u8(), //
            0x10, 0x00, 0x02, 0x00, //
            9, 7, 0,
        ];
        let p = make_fast_sync_read(&buf);
        let info = p.find_slot(0, 32).unwrap();
        assert_eq!(info.our_slot, 2);
        assert_eq!(info.n_slots, 3);
        assert_eq!(info.address, 0x10);
        assert_eq!(info.length, 2);
        // LEN = 3 + 3*(2+2) = 15
        assert_eq!(info.packet_length, 15);
        assert!(make_fast_sync_read(&buf).find_slot(0, 2).is_none());
    }

    #[test]
    fn fast_bulk_find_slot_handles_varying_lengths() {
        // entries: (1,0,4), (2,0,8), (3,0,2)
        let buf: [u8; 8 + 15] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x12, 0x00, Instruction::FastBulkRead.as_u8(), //
            1, 0, 0, 4, 0, //
            2, 0, 0, 8, 0, //
            3, 0, 0, 2, 0,
        ];
        let p = make_fast_bulk_read(&buf, 3);
        assert_eq!(p.find_slot(1, 32).unwrap().bytes_before, 0);
        // FAST_RESPONSE_SLOT0_BYTES(10) + 4 = 14
        assert_eq!(p.find_slot(2, 32).unwrap().bytes_before, 14);
        // 14 + FAST_RESPONSE_SLOT_BYTES(2) + 8 = 24
        assert_eq!(p.find_slot(3, 32).unwrap().bytes_before, 24);
        // LEN = 3 + 3*2 + (4+8+2) = 23
        assert_eq!(p.find_slot(1, 32).unwrap().packet_length, 23);
    }

    #[test]
    fn fast_slot_position_classifies_correctly() {
        let mk = |our_slot, n_slots| FastSlotInfo {
            our_slot,
            n_slots,
            address: 0,
            length: 0,
            packet_length: 7,
            bytes_before: 0,
        };
        assert_eq!(mk(0, 1).position(), SlotPosition::Only { packet_length: 7 });
        assert_eq!(mk(0, 3).position(), SlotPosition::First { packet_length: 7 });
        assert_eq!(mk(1, 3).position(), SlotPosition::Middle);
        assert_eq!(mk(2, 3).position(), SlotPosition::Last { crc: 0 });
    }

    #[test]
    fn sync_write_find_entry_returns_target() {
        let buf: [u8; 12 + 8] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0C, 0x00, Instruction::SyncWrite.as_u8(), //
            0x50, 0x00, 0x03, 0x00, //
            1, 10, 11, 12, //
            2, 20, 21, 22,
        ];
        let p = make_sync_write(&buf);
        let e = p.find_entry(2).unwrap();
        assert_eq!(e.id, 2);
        assert_eq!(e.data, &[20, 21, 22]);
        assert!(p.find_entry(9).is_none());
    }

    #[test]
    fn bulk_write_find_entry_returns_target_with_addr() {
        let buf: [u8; 8 + 17] = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, Instruction::BulkWrite.as_u8(), //
            1, 0x50, 0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC, //
            2, 0x54, 0x02, 0x04, 0x00, 0x10, 0x20, 0x30, 0x40,
        ];
        let p = make_bulk_write(&buf);
        let e = p.find_entry(2).unwrap();
        assert_eq!(e.addr, 0x0254);
        assert_eq!(e.data, &[0x10, 0x20, 0x30, 0x40]);
        assert!(p.find_entry(9).is_none());
    }

    #[test]
    fn interpret_fast_bulk_carries_error_and_payload() {
        let buf: [u8; 9 + 5] = [
            0xFF,
            0xFF,
            0xFD,
            0x00,
            0x01,
            0x08,
            0x00,
            Instruction::Status.as_u8(),
            0x07, //
            10,
            0xAA,
            0xBB,
            0xCC,
            0xDD,
        ];
        let s = make_status(&buf);
        match s.interpret(RequestKind::FastBulkRead) {
            Status::FastBulkRead { id, status } => {
                assert_eq!(id, 0x01);
                assert_eq!(status.error, StatusError::code(ErrorCode::Access));
                assert_eq!(status.payload, &[10, 0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected FastBulkRead, got {other:?}"),
        }
    }
}
