use crate::wire::{BROADCAST_ID, CrcUmts, WriteBuf, WriteError, write_raw};

use super::instruction::Instruction;
use super::packet::Packet;

pub(crate) fn write<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    packet: &Packet<'_>,
) -> Result<(), WriteError> {
    match packet {
        Packet::Ping(p) => write_raw::<W, _, CRC>(
            out,
            p.id,
            Instruction::Ping.as_u8(),
            &mut core::iter::empty(),
        ),
        Packet::Read(p) => {
            let mut params = U16Pair::new(p.address, p.length).into_iter();
            write_raw::<W, _, CRC>(out, p.id, Instruction::Read.as_u8(), &mut params)
        }
        Packet::Write(p) => {
            let mut params = U16One::new(p.address).into_iter().chain(p.data.iter());
            write_raw::<W, _, CRC>(out, p.id, Instruction::Write.as_u8(), &mut params)
        }
        Packet::RegWrite(p) => {
            let mut params = U16One::new(p.address).into_iter().chain(p.data.iter());
            write_raw::<W, _, CRC>(out, p.id, Instruction::RegWrite.as_u8(), &mut params)
        }
        Packet::Action(p) => write_raw::<W, _, CRC>(
            out,
            p.id,
            Instruction::Action.as_u8(),
            &mut core::iter::empty(),
        ),
        Packet::FactoryReset(p) => write_raw::<W, _, CRC>(
            out,
            p.id,
            Instruction::FactoryReset.as_u8(),
            &mut core::iter::once(p.mode),
        ),
        Packet::Reboot(p) => write_raw::<W, _, CRC>(
            out,
            p.id,
            Instruction::Reboot.as_u8(),
            &mut core::iter::empty(),
        ),
        #[cfg(feature = "osc")]
        Packet::Calibrate(p) => {
            let mut params = U16One::new(p.count).into_iter();
            write_raw::<W, _, CRC>(out, p.id, Instruction::Calibrate.as_u8(), &mut params)
        }
        Packet::Clear(p) => {
            write_raw::<W, _, CRC>(out, p.id, Instruction::Clear.as_u8(), &mut p.body.iter())
        }
        Packet::ControlTableBackup(p) => write_raw::<W, _, CRC>(
            out,
            p.id,
            Instruction::ControlTableBackup.as_u8(),
            &mut p.body.iter(),
        ),
        Packet::Status(p) => {
            let mut params = core::iter::once(p.error).chain(p.params.iter());
            write_raw::<W, _, CRC>(out, p.id, Instruction::Status.as_u8(), &mut params)
        }
        Packet::SyncRead(p) => {
            let mut params = U16Pair::new(p.address, p.length)
                .into_iter()
                .chain(p.ids.iter());
            write_raw::<W, _, CRC>(
                out,
                BROADCAST_ID,
                Instruction::SyncRead.as_u8(),
                &mut params,
            )
        }
        Packet::SyncWrite(p) => {
            let mut params = U16Pair::new(p.address, p.length)
                .into_iter()
                .chain(p.body.iter());
            write_raw::<W, _, CRC>(
                out,
                BROADCAST_ID,
                Instruction::SyncWrite.as_u8(),
                &mut params,
            )
        }
        Packet::FastSyncRead(p) => {
            let mut params = U16Pair::new(p.address, p.length)
                .into_iter()
                .chain(p.ids.iter());
            write_raw::<W, _, CRC>(
                out,
                BROADCAST_ID,
                Instruction::FastSyncRead.as_u8(),
                &mut params,
            )
        }
        Packet::BulkRead(p) => write_raw::<W, _, CRC>(
            out,
            BROADCAST_ID,
            Instruction::BulkRead.as_u8(),
            &mut p.body.iter(),
        ),
        Packet::BulkWrite(p) => write_raw::<W, _, CRC>(
            out,
            BROADCAST_ID,
            Instruction::BulkWrite.as_u8(),
            &mut p.body.iter(),
        ),
        Packet::FastBulkRead(p) => write_raw::<W, _, CRC>(
            out,
            BROADCAST_ID,
            Instruction::FastBulkRead.as_u8(),
            &mut p.body.iter(),
        ),
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
