//! Value types shared by the frame emitters: the write-failure code and the
//! chunk-source run yielded by a control-table read on its way to the wire.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WriteError {
    Overflow,
    Invalid,
}

/// One run yielded by a chunk-iterator on the way to the wire: either a
/// borrowed slice the encoder writes verbatim, or a phantom span the
/// encoder materializes as `n` zero bytes. Used by the streamed encoder
/// paths so the dispatcher can hand a control-table read iterator (or any
/// other source) to the FAST slot / chunked Status emitters without a
/// scratch buffer in between.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Chunk<'a> {
    Slice(&'a [u8]),
    Zero(u16),
}
