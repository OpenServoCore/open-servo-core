use crate::Instruction;
use crate::buf::WriteBuf;
use crate::crc::crc16;
use crate::parser::{HEADER, Packet};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WriteError {
    Overflow,
    Invalid,
}

pub fn write<W: WriteBuf>(out: &mut W, packet: &Packet<'_>) -> Result<(), WriteError> {
    match packet {
        Packet::Ping { id } => write_packet(out, *id, Instruction::Ping, &mut core::iter::empty()),
        Packet::Read {
            id,
            address,
            length,
        } => {
            let mut params = U16Pair::new(*address, *length).into_iter();
            write_packet(out, *id, Instruction::Read, &mut params)
        }
        Packet::Write { id, address, data } => {
            let mut params = U16One::new(*address).into_iter().chain(data.iter());
            write_packet(out, *id, Instruction::Write, &mut params)
        }
        Packet::RegWrite { id, address, data } => {
            let mut params = U16One::new(*address).into_iter().chain(data.iter());
            write_packet(out, *id, Instruction::RegWrite, &mut params)
        }
        Packet::Action { id } => {
            write_packet(out, *id, Instruction::Action, &mut core::iter::empty())
        }
        Packet::FactoryReset { id, mode } => write_packet(
            out,
            *id,
            Instruction::FactoryReset,
            &mut core::iter::once(*mode),
        ),
        Packet::Reboot { id } => {
            write_packet(out, *id, Instruction::Reboot, &mut core::iter::empty())
        }
        Packet::Clear { id, body } => write_packet(out, *id, Instruction::Clear, &mut body.iter()),
        Packet::ControlTableBackup { id, body } => {
            write_packet(out, *id, Instruction::ControlTableBackup, &mut body.iter())
        }
        Packet::Status { id, error, params } => {
            let mut p = core::iter::once(*error).chain(params.iter());
            write_packet(out, *id, Instruction::Status, &mut p)
        }
        Packet::SyncRead {
            address,
            length,
            ids,
        } => {
            let mut p = U16Pair::new(*address, *length)
                .into_iter()
                .chain(ids.iter());
            write_packet(out, BROADCAST, Instruction::SyncRead, &mut p)
        }
        Packet::SyncWrite {
            address,
            length,
            body,
        } => {
            let mut p = U16Pair::new(*address, *length)
                .into_iter()
                .chain(body.iter());
            write_packet(out, BROADCAST, Instruction::SyncWrite, &mut p)
        }
        Packet::FastSyncRead {
            address,
            length,
            ids,
        } => {
            let mut p = U16Pair::new(*address, *length)
                .into_iter()
                .chain(ids.iter());
            write_packet(out, BROADCAST, Instruction::FastSyncRead, &mut p)
        }
        Packet::BulkRead { body } => {
            write_packet(out, BROADCAST, Instruction::BulkRead, &mut body.iter())
        }
        Packet::BulkWrite { body } => {
            write_packet(out, BROADCAST, Instruction::BulkWrite, &mut body.iter())
        }
        Packet::FastBulkRead { body } => {
            write_packet(out, BROADCAST, Instruction::FastBulkRead, &mut body.iter())
        }
    }
}

const BROADCAST: u8 = crate::parser::BROADCAST_ID;

/// id != 0xFF and instruction byte != 0xFD are required so the unstuffed
/// prefix (header..instruction) cannot itself complete a stuffing trigger.
///
/// On any failure (including `Overflow` partway through), `out` is truncated
/// back to its length on entry — callers using `out` as a DMA TX buffer can
/// assume that a failed write leaves prior frames untouched.
fn write_packet<W: WriteBuf, I: Iterator<Item = u8>>(
    out: &mut W,
    id: u8,
    instruction: Instruction,
    params: &mut I,
) -> Result<(), WriteError> {
    if id == 0xFF {
        return Err(WriteError::Invalid);
    }
    let start = out.len();
    match write_packet_body(out, start, id, instruction, params) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn write_packet_body<W: WriteBuf, I: Iterator<Item = u8>>(
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

    let crc = crc16(&out.as_slice()[start..]);
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
