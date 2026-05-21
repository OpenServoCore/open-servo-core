use core::sync::atomic::Ordering;

use dxl_protocol::{BROADCAST_ID, Bytes, Packet, ParseError, parse_one, write};
use osc_core::{RegmapError, Shared};

use crate::board::Ch32Board;
use crate::statics::{DXL_RX_BUF, DXL_RX_READER, DXL_RX_WRITE_POS};

const ERR_INSTRUCTION: u8 = 0x02;
const ERR_DATA_RANGE: u8 = 0x04;
const ERR_ACCESS: u8 = 0x07;

pub fn poll(shared: &Shared, board: &mut Ch32Board) {
    let write_pos = DXL_RX_WRITE_POS.load(Ordering::Acquire);
    // SAFETY: `DXL_RX_READER` is exclusively owned by this function.
    let reader = unsafe { &mut *DXL_RX_READER.get() };
    // SAFETY: DMA writes DXL_RX_BUF circularly; we only read indices below
    // the IDLE-published write_pos.
    let ring = unsafe { &*DXL_RX_BUF.get() };
    reader.ingest(ring, write_pos);

    loop {
        match parse_one(reader.peek()) {
            Ok((packet, used)) => {
                dispatch(shared, board, &packet);
                reader.consume(used);
            }
            Err(ParseError::Incomplete) => break,
            Err(ParseError::Resync { skip })
            | Err(ParseError::BadCrc { skip })
            | Err(ParseError::BadInstruction { skip })
            | Err(ParseError::BadLength { skip }) => {
                reader.consume(skip);
            }
        }
    }
}

fn dispatch(shared: &Shared, board: &mut Ch32Board, packet: &Packet<'_>) {
    let our_id = our_id(shared);
    let target_id = packet_id(packet);

    let respond = match (packet, target_id) {
        (Packet::Ping { .. }, Some(id)) => id == our_id || id == BROADCAST_ID,
        (_, Some(id)) => id == our_id,
        (_, None) => false,
    };
    if !respond {
        return;
    }

    match packet {
        Packet::Ping { .. } => send_ping_status(shared, board, our_id),
        Packet::Read {
            address, length, ..
        } => send_read_status(shared, board, our_id, *address, *length),
        _ => send_status(board, our_id, ERR_INSTRUCTION, &[]),
    }
}

fn our_id(shared: &Shared) -> u8 {
    // SAFETY: comms.id is a single byte; ISR contexts read but don't mutate.
    unsafe { (*shared.table.config.get()).comms.id }
}

fn packet_id(packet: &Packet<'_>) -> Option<u8> {
    match *packet {
        Packet::Ping { id }
        | Packet::Read { id, .. }
        | Packet::Write { id, .. }
        | Packet::RegWrite { id, .. }
        | Packet::Action { id }
        | Packet::FactoryReset { id, .. }
        | Packet::Reboot { id }
        | Packet::Clear { id, .. }
        | Packet::ControlTableBackup { id, .. }
        | Packet::Status { id, .. } => Some(id),
        Packet::SyncRead { .. }
        | Packet::SyncWrite { .. }
        | Packet::BulkRead { .. }
        | Packet::BulkWrite { .. }
        | Packet::FastSyncRead { .. }
        | Packet::FastBulkRead { .. } => None,
    }
}

fn send_ping_status(shared: &Shared, board: &mut Ch32Board, id: u8) {
    // SAFETY: identity is read-only after seed_config_defaults; no concurrent writers.
    let identity = unsafe { (*shared.table.config.get()).identity };
    let model = identity.model_number.to_le_bytes();
    let fw = identity.firmware_version as u8;
    let params = [model[0], model[1], fw];
    send_status(board, id, 0, &params);
}

fn send_read_status(shared: &Shared, board: &mut Ch32Board, id: u8, address: u16, length: u16) {
    // Cap at largest payload that fits a Status reply in DXL_TX_BUF:
    // HEADER(4) + id(1) + len(2) + instr(1) + err(1) + params + crc(2) = 11 + N.
    // DXL_TX_BUF_LEN = 256, so N <= 240; we cap at 128 for now (single block).
    const MAX_READ: usize = 128;

    let len = length as usize;
    if len == 0 || len > MAX_READ {
        send_status(board, id, ERR_DATA_RANGE, &[]);
        return;
    }

    let mut buf = [0u8; MAX_READ];
    match shared.table.read_bytes(address, &mut buf[..len]) {
        Ok(()) => send_status(board, id, 0, &buf[..len]),
        Err(RegmapError::OutOfRange) => send_status(board, id, ERR_DATA_RANGE, &[]),
        Err(RegmapError::AccessError) => send_status(board, id, ERR_ACCESS, &[]),
    }
}

fn send_status(board: &mut Ch32Board, id: u8, error: u8, params: &[u8]) {
    let buf = board.dxl_tx_buf();
    buf.clear();
    let packet = Packet::Status {
        id,
        error,
        params: Bytes::Raw(params),
    };
    if write(buf, &packet).is_err() {
        buf.clear();
        return;
    }
    board.start_dxl_tx();
}
