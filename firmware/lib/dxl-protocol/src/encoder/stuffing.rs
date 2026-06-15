//! DXL 2.0 byte-stuffing window. Sliding 2-byte trailing context plus the
//! byte just pushed; when the triple matches `FF FF FD`, an inline `FD`
//! follows so the embedded sequence cannot synthesize a fake header.

use crate::buf::{WriteBuf, WriteError};
use crate::wire::HEADER;

const STUFFING_TRIGGER: [u8; 3] = [HEADER[0], HEADER[1], HEADER[2]];
const STUFFING_BYTE: u8 = 0xFD;

/// Sliding-window state for the byte-stuffing encoder: after each pushed
/// param byte, a `0xFF 0xFF 0xFD` window triggers an inline `0xFD`.
pub(in crate::encoder) struct Stuffer([u8; 2]);

impl Stuffer {
    /// Seed the window with the instruction byte so a trigger straddling the
    /// instr->params boundary still emits the stuffing FD.
    pub(in crate::encoder) fn new(instruction: u8) -> Self {
        Self([0, instruction])
    }

    pub(in crate::encoder) fn push<W: WriteBuf>(
        &mut self,
        out: &mut W,
        b: u8,
    ) -> Result<(), WriteError> {
        out.push(b)?;
        if [self.0[0], self.0[1], b] == STUFFING_TRIGGER {
            out.push(STUFFING_BYTE)?;
            self.0 = [self.0[1], STUFFING_BYTE];
        } else {
            self.0 = [self.0[1], b];
        }
        Ok(())
    }
}
