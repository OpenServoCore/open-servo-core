use super::bytes::Bytes;
use super::crc::CrcUmts;
use super::frame::{BROADCAST_ID, HEADER, PACKET_LEN_GUARD, PACKET_LEN_MIN, RawFrame};
use super::rx_view::RxView;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    Incomplete,
    Resync { skip: usize },
    BadCrc { skip: usize },
    BadInstruction { skip: usize },
    BadLength { skip: usize },
}

/// Status instruction byte. Hardcoded here so the wire parser can detect
/// Fast chain headers (broadcast + Status with multi-slot length) without
/// depending on the typed `Instruction` enum.
const STATUS_INSTRUCTION_BYTE: u8 = 0x55;

/// Wire-layer parse: locate the frame, validate length and CRC, and return a
/// `RawFrame` with the params slice + raw instruction byte. Does NOT resolve
/// the instruction byte to the `Instruction` enum or decode the params.
///
/// On error variants other than `Incomplete`, `skip` is the number of virtual
/// bytes the caller should drop before retrying.
pub fn parse_raw<'a, CRC: CrcUmts>(
    rx: RxView<'a>,
) -> Result<(RawFrame<Bytes<'a>>, usize), ParseError> {
    let header_off = match rx.find_header() {
        Ok(0) => 0,
        Ok(n) => return Err(ParseError::Resync { skip: n }),
        Err(keep) => {
            let skip = rx.len() - keep;
            if skip == 0 {
                return Err(ParseError::Incomplete);
            }
            return Err(ParseError::Resync { skip });
        }
    };

    let total = rx.len();
    if total - header_off < 7 {
        return Err(ParseError::Incomplete);
    }

    let id = rx.get(header_off + 4).unwrap();
    let length = u16::from_le_bytes([
        rx.get(header_off + 5).unwrap(),
        rx.get(header_off + 6).unwrap(),
    ]) as usize;

    // Bad length: don't trust this header. Step past 4 bytes rather than
    // honoring its claimed size, so a phantom can't mask a real frame after.
    if !(PACKET_LEN_MIN..=PACKET_LEN_GUARD).contains(&length) {
        return Err(ParseError::BadLength { skip: HEADER.len() });
    }

    let frame_len = 7 + length;
    if total - header_off < frame_len {
        // Fast First/Only chain headers use BROADCAST_ID + Status with a
        // length covering the WHOLE multi-slot reply. When such a header
        // shows up incomplete, the missing bytes never land here — they're
        // on the wire during another node's TX. Resync past this phantom.
        if total - header_off >= 8
            && id == BROADCAST_ID
            && rx.get(header_off + 7) == Some(STATUS_INSTRUCTION_BYTE)
        {
            return Err(ParseError::BadInstruction { skip: HEADER.len() });
        }
        return Err(ParseError::Incomplete);
    }

    let frame_start = header_off;
    let crc_pos = frame_start + frame_len - 2;
    let computed = rx.crc::<CRC>(frame_start, crc_pos);
    let received = u16::from_le_bytes([rx.get(crc_pos).unwrap(), rx.get(crc_pos + 1).unwrap()]);
    if computed != received {
        // Could be a corrupted real frame *or* a phantom header — drop past
        // the header and let resync find the next one.
        return Err(ParseError::BadCrc { skip: HEADER.len() });
    }

    let instruction = rx.get(frame_start + 7).unwrap();
    let params_start = frame_start + 8;
    let params_end = crc_pos;
    let params = rx.slice_stuffed(params_start, params_end);

    Ok((
        RawFrame {
            id,
            instruction,
            params,
        },
        frame_start + frame_len,
    ))
}
