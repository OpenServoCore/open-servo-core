//! sec 5.6 TEL rows (osc-servo): a row is the bytes of up to two chunks of
//! the control table at their table layout, copied whole each control
//! tick, and a burst streams at one rate only.

use crate::wire::{self, BaudRate};

/// The one rate a burst streams at: a frame of 16 rows of
/// [`ROW_BYTES_MAX`] clears the wire inside its batch's ticks.
pub const BAUD: BaudRate = BaudRate::B3000000;

/// The longest row: the chunks' total, bytes.
pub const ROW_BYTES_MAX: usize = 12;

/// Chunks a row copies.
pub const CHUNKS_MAX: usize = 2;

/// Table bytes a chunk may cover: `[0, TABLE_BYTES)`.
pub const TABLE_BYTES: u16 = 1024;

/// One run of table bytes a row carries: `len` bytes from `addr`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Chunk {
    pub addr: u16,
    pub len: u16,
}

impl Chunk {
    pub const fn new(addr: u16, len: u16) -> Self {
        Self { addr, len }
    }

    /// Even ends, inside the table, at most a row.
    const fn fits(&self) -> bool {
        (self.addr | self.len) & 1 == 0
            && self.len as usize <= ROW_BYTES_MAX
            && self.addr as u32 + self.len as u32 <= TABLE_BYTES as u32
    }
}

/// A row's chunks in CONTROL order (`chunk0_addr`, `chunk0_len`,
/// `chunk1_addr`, `chunk1_len`); a second chunk of length 0 is none.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Chunks(pub [Chunk; CHUNKS_MAX]);

impl Chunks {
    pub const NONE: Self = Self([Chunk::new(0, 0); CHUNKS_MAX]);

    pub const fn one(addr: u16, len: u16) -> Self {
        Self([Chunk::new(addr, len), Chunk::new(0, 0)])
    }

    pub const fn two(a: Chunk, b: Chunk) -> Self {
        Self([a, b])
    }

    pub const fn row_bytes(&self) -> usize {
        self.0[0].len as usize + self.0[1].len as usize
    }

    /// A row a burst arms with: each chunk even and inside the table, the
    /// first one nonempty, the row at most [`ROW_BYTES_MAX`].
    pub const fn valid(&self) -> bool {
        let [a, b] = self.0;
        a.len != 0 && a.fits() && b.fits() && self.row_bytes() <= ROW_BYTES_MAX
    }

    /// Where table bytes `[addr, addr + width)` ride in a row: their offset,
    /// or None when no one chunk holds them all.
    pub const fn offset(&self, addr: u16, width: u16) -> Option<usize> {
        let mut at = 0;
        let mut i = 0;
        while i < CHUNKS_MAX {
            let c = self.0[i];
            if addr >= c.addr && addr as u32 + width as u32 <= c.addr as u32 + c.len as u32 {
                return Some(at + (addr - c.addr) as usize);
            }
            at += c.len as usize;
            i += 1;
        }
        None
    }

    /// The four CONTROL registers, LE: one write before `tel_count`.
    pub const fn to_le_bytes(&self) -> [u8; 4 * CHUNKS_MAX] {
        let [a, b] = self.0;
        let [a0, a1] = a.addr.to_le_bytes();
        let [l0, l1] = a.len.to_le_bytes();
        let [b0, b1] = b.addr.to_le_bytes();
        let [m0, m1] = b.len.to_le_bytes();
        [a0, a1, l0, l1, b0, b1, m0, m1]
    }

    pub const fn from_le_bytes(b: [u8; 4 * CHUNKS_MAX]) -> Self {
        let [a0, a1, l0, l1, b0, b1, m0, m1] = b;
        let w = u16::from_le_bytes;
        Self([
            Chunk::new(w([a0, a1]), w([l0, l1])),
            Chunk::new(w([b0, b1]), w([m0, m1])),
        ])
    }
}

/// One TEL frame payload (sec 5.6), split: the head and the rows.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StreamFrame<'a> {
    pub seq: u8,
    pub last: bool,
    /// The first row's `sample_tick`, low 16 bits.
    pub tick: u16,
    rows: &'a [u8],
    row_bytes: usize,
}

impl<'a> StreamFrame<'a> {
    /// None when `payload` cannot be a frame of `row_bytes` rows: shorter
    /// than the head, no row, or a ragged row remainder.
    pub fn parse(row_bytes: usize, payload: &'a [u8]) -> Option<Self> {
        let (head, rows) = payload.split_first_chunk::<{ wire::STREAM_HDR }>()?;
        let n = rows.len().checked_div(row_bytes)?;
        if n == 0 || n * row_bytes != rows.len() {
            return None;
        }
        let [seq, flags, t0, t1] = *head;
        Some(Self {
            seq,
            last: flags & wire::STREAM_FLAG_LAST != 0,
            tick: u16::from_le_bytes([t0, t1]),
            rows,
            row_bytes,
        })
    }

    pub fn len(&self) -> usize {
        self.rows.len() / self.row_bytes
    }

    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    /// The rows in order, each the chunks' bytes back to back.
    pub fn rows(&self) -> core::slice::ChunksExact<'a, u8> {
        self.rows.chunks_exact(self.row_bytes)
    }
}

/// Serialize the stream head at the front of `buf`: everything row 0
/// knows. Order-free vs the rows: it never touches bytes past
/// [`wire::STREAM_HDR`]; a `buf` shorter than that stays untouched.
pub fn encode_stream_head(stream_seq: u8, last: bool, tick: u16, buf: &mut [u8]) {
    if let Some(h) = buf.first_chunk_mut::<{ wire::STREAM_HDR }>() {
        let [t0, t1] = tick.to_le_bytes();
        *h = [
            stream_seq,
            if last { wire::STREAM_FLAG_LAST } else { 0 },
            t0,
            t1,
        ];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chunks_admit_even_rows_inside_the_table() {
        assert!(Chunks::one(0x228, 12).valid());
        assert!(Chunks::two(Chunk::new(0x224, 6), Chunk::new(0x23E, 2)).valid());
        assert!(Chunks::one(TABLE_BYTES - 2, 2).valid());
        assert!(!Chunks::NONE.valid(), "no row");
        assert!(!Chunks::two(Chunk::new(0, 0), Chunk::new(0x224, 2)).valid());
        assert!(!Chunks::one(0x225, 2).valid(), "odd addr");
        assert!(!Chunks::one(0x224, 3).valid(), "odd len");
        assert!(!Chunks::one(0x224, 14).valid(), "over the row");
        assert!(!Chunks::two(Chunk::new(0x224, 8), Chunk::new(0x244, 6)).valid());
        assert!(!Chunks::one(TABLE_BYTES - 2, 4).valid(), "past the table");
        assert!(!Chunks::one(u16::MAX - 1, 2).valid());
    }

    #[test]
    fn offsets_find_a_field_inside_one_chunk() {
        let c = Chunks::two(Chunk::new(0x224, 4), Chunk::new(0x244, 8));
        assert_eq!(c.row_bytes(), 12);
        assert_eq!(c.offset(0x224, 2), Some(0));
        assert_eq!(c.offset(0x226, 2), Some(2));
        assert_eq!(c.offset(0x244, 4), Some(4));
        assert_eq!(c.offset(0x248, 4), Some(8));
        assert_eq!(c.offset(0x228, 2), None);
        assert_eq!(c.offset(0x226, 4), None, "straddles a chunk end");
        assert_eq!(Chunks::NONE.offset(0, 2), None);
    }

    #[test]
    fn chunks_round_trip_the_control_registers() {
        let c = Chunks::two(Chunk::new(0x224, 6), Chunk::new(0x23E, 2));
        let b = c.to_le_bytes();
        assert_eq!(b, [0x24, 0x02, 6, 0, 0x3E, 0x02, 2, 0]);
        assert_eq!(Chunks::from_le_bytes(b), c);
    }

    /// Head [seq][flags][tick u16], then the rows: the count comes from the
    /// length alone.
    #[test]
    fn stream_frame_takes_its_row_count_from_the_length() {
        let p = [0x42, wire::STREAM_FLAG_LAST, 0x34, 0x12, 1, 0, 2, 0, 3, 0];
        let f = StreamFrame::parse(2, &p).expect("a frame");
        assert_eq!((f.seq, f.last, f.tick, f.len()), (0x42, true, 0x1234, 3));
        let mut rows = f.rows();
        assert_eq!(rows.next(), Some(&[1, 0][..]));
        assert_eq!(rows.nth(1), Some(&[3, 0][..]));
        assert_eq!(rows.next(), None);
        assert!(StreamFrame::parse(4, &p).is_none(), "a ragged remainder");
        assert!(StreamFrame::parse(2, &p[..4]).is_none(), "no row");
        assert!(StreamFrame::parse(2, &p[..3]).is_none(), "no head");
        assert!(StreamFrame::parse(0, &p).is_none());
    }

    #[test]
    fn head_carries_seq_flags_and_the_stamp() {
        let mut buf = [0xEE; wire::STREAM_HDR + 1];
        encode_stream_head(0x42, true, 0x1234, &mut buf);
        assert_eq!(buf, [0x42, wire::STREAM_FLAG_LAST, 0x34, 0x12, 0xEE]);
        let mut short = [0xEE; wire::STREAM_HDR - 1];
        encode_stream_head(1, false, 2, &mut short);
        assert_eq!(short, [0xEE; wire::STREAM_HDR - 1]);
    }

    /// The host detects the burst end from the head the servo encodes.
    #[test]
    fn stream_last_reads_the_encoded_last_flag() {
        use crate::FrameBytes;
        use crate::wire::{Inst, ResultCode, stream_last};
        let inst = Inst::status(ResultCode::Stream, false);
        let mut buf = [0u8; wire::STREAM_HDR + 2];
        encode_stream_head(0, true, 0, &mut buf);
        assert!(stream_last(inst, FrameBytes::from(&buf[..])));
        encode_stream_head(0, false, 0, &mut buf);
        assert!(!stream_last(inst, FrameBytes::from(&buf[..])));
    }
}
