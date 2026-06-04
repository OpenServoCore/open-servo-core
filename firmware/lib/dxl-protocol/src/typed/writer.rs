use crate::wire::{CrcUmts, HEADER, WriteBuf, WriteError};

use super::instruction::Instruction;
use super::packet::Packet;

pub(crate) fn write<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    packet: &Packet<'_>,
) -> Result<(), WriteError> {
    match packet {
        Packet::Ping(p) => {
            write_packet::<W, _, CRC>(out, p.id, Instruction::Ping, &mut core::iter::empty())
        }
        Packet::Read(p) => {
            let mut params = U16Pair::new(p.address, p.length).into_iter();
            write_packet::<W, _, CRC>(out, p.id, Instruction::Read, &mut params)
        }
        Packet::Write(p) => {
            let mut params = U16One::new(p.address).into_iter().chain(p.data.iter());
            write_packet::<W, _, CRC>(out, p.id, Instruction::Write, &mut params)
        }
        Packet::RegWrite(p) => {
            let mut params = U16One::new(p.address).into_iter().chain(p.data.iter());
            write_packet::<W, _, CRC>(out, p.id, Instruction::RegWrite, &mut params)
        }
        Packet::Action(p) => {
            write_packet::<W, _, CRC>(out, p.id, Instruction::Action, &mut core::iter::empty())
        }
        Packet::FactoryReset(p) => write_packet::<W, _, CRC>(
            out,
            p.id,
            Instruction::FactoryReset,
            &mut core::iter::once(p.mode),
        ),
        Packet::Reboot(p) => {
            write_packet::<W, _, CRC>(out, p.id, Instruction::Reboot, &mut core::iter::empty())
        }
        #[cfg(feature = "osc")]
        Packet::Calibrate(p) => {
            let mut params = U16One::new(p.count).into_iter();
            write_packet::<W, _, CRC>(out, p.id, Instruction::Calibrate, &mut params)
        }
        Packet::Clear(p) => {
            write_packet::<W, _, CRC>(out, p.id, Instruction::Clear, &mut p.body.iter())
        }
        Packet::ControlTableBackup(p) => write_packet::<W, _, CRC>(
            out,
            p.id,
            Instruction::ControlTableBackup,
            &mut p.body.iter(),
        ),
        Packet::Status(p) => {
            let mut params = core::iter::once(p.error).chain(p.params.iter());
            write_packet::<W, _, CRC>(out, p.id, Instruction::Status, &mut params)
        }
        Packet::SyncRead(p) => {
            let mut params = U16Pair::new(p.address, p.length)
                .into_iter()
                .chain(p.ids.iter());
            write_packet::<W, _, CRC>(out, BROADCAST, Instruction::SyncRead, &mut params)
        }
        Packet::SyncWrite(p) => {
            let mut params = U16Pair::new(p.address, p.length)
                .into_iter()
                .chain(p.body.iter());
            write_packet::<W, _, CRC>(out, BROADCAST, Instruction::SyncWrite, &mut params)
        }
        Packet::FastSyncRead(p) => {
            let mut params = U16Pair::new(p.address, p.length)
                .into_iter()
                .chain(p.ids.iter());
            write_packet::<W, _, CRC>(out, BROADCAST, Instruction::FastSyncRead, &mut params)
        }
        Packet::BulkRead(p) => {
            write_packet::<W, _, CRC>(out, BROADCAST, Instruction::BulkRead, &mut p.body.iter())
        }
        Packet::BulkWrite(p) => {
            write_packet::<W, _, CRC>(out, BROADCAST, Instruction::BulkWrite, &mut p.body.iter())
        }
        Packet::FastBulkRead(p) => write_packet::<W, _, CRC>(
            out,
            BROADCAST,
            Instruction::FastBulkRead,
            &mut p.body.iter(),
        ),
    }
}

const BROADCAST: u8 = crate::wire::BROADCAST_ID;

/// id != 0xFF, instruction != 0xFD required so the unstuffed header..instruction
/// prefix can't itself complete a stuffing trigger.
///
/// On failure (including partial Overflow), `out` is truncated back to entry length
/// — callers using a DMA TX buffer can rely on prior frames staying intact.
pub(crate) fn write_packet<W: WriteBuf, I: Iterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    id: u8,
    instruction: Instruction,
    params: &mut I,
) -> Result<(), WriteError> {
    if id == 0xFF {
        return Err(WriteError::Invalid);
    }
    let start = out.len();
    match write_packet_body::<W, _, CRC>(out, start, id, instruction, params) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn write_packet_body<W: WriteBuf, I: Iterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    start: usize,
    id: u8,
    instruction: Instruction,
    params: &mut I,
) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(id)?;
    let len_pos = out.len();
    out.push(0)?;
    out.push(0)?;
    out.push(instruction.as_u8())?;

    let mut last2: [u8; 2] = [0, instruction.as_u8()];
    for b in params.by_ref() {
        out.push(b)?;
        if last2[0] == 0xFF && last2[1] == 0xFF && b == 0xFD {
            out.push(0xFD)?;
            last2 = [0xFD, 0xFD];
        } else {
            last2 = [last2[1], b];
        }
    }

    let stuffed_params_len = out.len() - (len_pos + 2 + 1);
    let length_value = (1 + stuffed_params_len + 2) as u16;
    let len_bytes = length_value.to_le_bytes();
    out.set(len_pos, len_bytes[0]);
    out.set(len_pos + 1, len_bytes[1]);

    let crc = CRC::accumulate(0, &out.as_slice()[start..]);
    let crc_bytes = crc.to_le_bytes();
    out.push(crc_bytes[0])?;
    out.push(crc_bytes[1])?;

    Ok(())
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
