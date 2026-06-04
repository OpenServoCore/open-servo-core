use super::buf::{WriteBuf, WriteError};
use super::crc::CrcUmts;
use super::frame::{HEADER, RawFrame};

/// `frame.id != 0xFF` and `frame.instruction != 0xFD` required so the
/// unstuffed header..instruction prefix can't itself complete a stuffing
/// trigger.
///
/// On failure (including partial Overflow), `out` is truncated back to entry
/// length — callers using a DMA TX buffer can rely on prior frames staying
/// intact.
pub fn write_raw<W: WriteBuf, I: IntoIterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    frame: RawFrame<I>,
) -> Result<(), WriteError> {
    if frame.id == 0xFF {
        return Err(WriteError::Invalid);
    }
    let start = out.len();
    match write_raw_body::<W, _, CRC>(out, start, frame) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn write_raw_body<W: WriteBuf, I: IntoIterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    start: usize,
    frame: RawFrame<I>,
) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(frame.id)?;
    let len_pos = out.len();
    out.push(0)?;
    out.push(0)?;
    out.push(frame.instruction)?;

    let mut last2: [u8; 2] = [0, frame.instruction];
    for b in frame.params {
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
