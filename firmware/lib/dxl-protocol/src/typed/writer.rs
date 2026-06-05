use crate::wire::{BROADCAST_ID, CrcUmts, RawFrame, WriteBuf, WriteError, write_raw};

use super::instruction_ext::InstructionExt;
use super::instruction::Instruction;
use super::packet::Packet;

pub(crate) fn write<W: WriteBuf, CRC: CrcUmts, X: InstructionExt>(
    out: &mut W,
    packet: &Packet<'_, X>,
) -> Result<(), WriteError> {
    match packet {
        Packet::Ping(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::Ping.as_u8(),
                params: core::iter::empty::<u8>(),
            },
        ),
        Packet::Read(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::Read.as_u8(),
                params: U16Pair::new(p.address, p.length),
            },
        ),
        Packet::Write(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::Write.as_u8(),
                params: U16One::new(p.address).into_iter().chain(p.data),
            },
        ),
        Packet::RegWrite(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::RegWrite.as_u8(),
                params: U16One::new(p.address).into_iter().chain(p.data),
            },
        ),
        Packet::Action(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::Action.as_u8(),
                params: core::iter::empty::<u8>(),
            },
        ),
        Packet::FactoryReset(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::FactoryReset.as_u8(),
                params: core::iter::once(p.mode),
            },
        ),
        Packet::Reboot(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::Reboot.as_u8(),
                params: core::iter::empty::<u8>(),
            },
        ),
        Packet::Clear(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::Clear.as_u8(),
                params: p.body,
            },
        ),
        Packet::ControlTableBackup(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::ControlTableBackup.as_u8(),
                params: p.body,
            },
        ),
        Packet::Status(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: p.id,
                instruction: Instruction::Status.as_u8(),
                params: core::iter::once(p.error).chain(p.params),
            },
        ),
        Packet::SyncRead(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: BROADCAST_ID,
                instruction: Instruction::SyncRead.as_u8(),
                params: U16Pair::new(p.address, p.length).into_iter().chain(p.ids),
            },
        ),
        Packet::SyncWrite(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: BROADCAST_ID,
                instruction: Instruction::SyncWrite.as_u8(),
                params: U16Pair::new(p.address, p.length).into_iter().chain(p.body),
            },
        ),
        Packet::FastSyncRead(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: BROADCAST_ID,
                instruction: Instruction::FastSyncRead.as_u8(),
                params: U16Pair::new(p.address, p.length).into_iter().chain(p.ids),
            },
        ),
        Packet::BulkRead(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: BROADCAST_ID,
                instruction: Instruction::BulkRead.as_u8(),
                params: p.body,
            },
        ),
        Packet::BulkWrite(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: BROADCAST_ID,
                instruction: Instruction::BulkWrite.as_u8(),
                params: p.body,
            },
        ),
        Packet::FastBulkRead(p) => write_raw::<W, _, CRC>(
            out,
            RawFrame {
                id: BROADCAST_ID,
                instruction: Instruction::FastBulkRead.as_u8(),
                params: p.body,
            },
        ),
        Packet::Ext(v) => X::write::<W, CRC>(v, out),
    }
}

#[derive(Copy, Clone)]
struct U16One {
    bytes: [u8; 2],
    i: u8,
}

impl U16One {
    fn new(v: u16) -> Self {
        Self {
            bytes: v.to_le_bytes(),
            i: 0,
        }
    }
}

impl IntoIterator for U16One {
    type Item = u8;
    type IntoIter = U16OneIter;
    fn into_iter(self) -> U16OneIter {
        U16OneIter(self)
    }
}

struct U16OneIter(U16One);

impl Iterator for U16OneIter {
    type Item = u8;
    fn next(&mut self) -> Option<u8> {
        if self.0.i < 2 {
            let b = self.0.bytes[self.0.i as usize];
            self.0.i += 1;
            Some(b)
        } else {
            None
        }
    }
}

#[derive(Copy, Clone)]
struct U16Pair {
    bytes: [u8; 4],
    i: u8,
}

impl U16Pair {
    fn new(a: u16, b: u16) -> Self {
        let aa = a.to_le_bytes();
        let bb = b.to_le_bytes();
        Self {
            bytes: [aa[0], aa[1], bb[0], bb[1]],
            i: 0,
        }
    }
}

impl IntoIterator for U16Pair {
    type Item = u8;
    type IntoIter = U16PairIter;
    fn into_iter(self) -> U16PairIter {
        U16PairIter(self)
    }
}

struct U16PairIter(U16Pair);

impl Iterator for U16PairIter {
    type Item = u8;
    fn next(&mut self) -> Option<u8> {
        if self.0.i < 4 {
            let b = self.0.bytes[self.0.i as usize];
            self.0.i += 1;
            Some(b)
        } else {
            None
        }
    }
}
