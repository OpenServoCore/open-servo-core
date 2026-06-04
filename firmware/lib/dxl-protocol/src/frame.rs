use crate::bytes::Bytes;

/// Wire-layer view of a single validated DXL frame: header, ID, length, and
/// CRC are accounted for, but the instruction byte has not been resolved to
/// the `Instruction` enum and `params` has not been interpreted. Use this as
/// the boundary between wire parsing and typed decode.
#[derive(Copy, Clone, Debug)]
pub struct RawFrame<'a> {
    pub id: u8,
    pub instruction: u8,
    pub params: Bytes<'a>,
}
