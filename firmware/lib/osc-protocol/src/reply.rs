//! osc-native TX frame buffer: one halfword-aligned scratch that both servo
//! statuses and host instructions encode into (`docs/osc-native-protocol.md`
//! §3.1, §3.2, §4.2). The literal `0x00` at offset 0 is the CRC prefix — the
//! CRC-engine DMA reads from offset 0, the UART TX DMA from offset 1.

use crate::wire::{self, Id, Inst};

/// A frame under construction: `[0x00][ID][LEN][INST][payload][PAD?][crc][crc]`.
///
/// Alignment matters twice over — the hardware CRC DMA reads halfwords from
/// offset 0 (§3.2, needs 2), and payload copies land at offset 4, so word
/// alignment lets `copy_from_slice` word-copy from word-aligned sources
/// instead of byte-looping. `N` bounds the largest frame this buffer can
/// hold; the finished `LEN` is read back from `bytes[2]`, so the struct stays
/// just the array.
#[repr(C, align(4))]
pub struct FrameBuf<const N: usize> {
    bytes: [u8; N],
}

impl<const N: usize> FrameBuf<N> {
    /// Smallest layout is the empty-payload frame: prefix + header + CRC.
    const MIN: usize = 7;

    #[inline]
    pub const fn new() -> Self {
        const { assert!(N >= Self::MIN) }
        Self { bytes: [0; N] }
    }

    /// Lay down the fixed prefix; `LEN` (byte 2) stays a placeholder until
    /// [`finish`](Self::finish). `inst` is written as-is — the PAD bit is set
    /// by `finish` from the payload length.
    #[inline]
    pub fn start(&mut self, id: Id, inst: Inst) {
        self.bytes[0] = wire::CRC_PREFIX;
        self.bytes[1] = id.as_byte();
        self.bytes[2] = 0;
        self.bytes[3] = inst.0;
    }

    /// The payload region, leaving room for the pad byte and CRC.
    #[inline]
    pub fn payload_mut(&mut self) -> &mut [u8] {
        &mut self.bytes[4..N - 3]
    }

    /// Seal the layout for a `p`-byte payload: apply the pad invariant (pad iff
    /// `p` odd, §3.1) to both the trailing byte and the INST PAD flag, then
    /// write `LEN`. CRC bytes are filled by [`set_crc`](Self::set_crc).
    #[inline]
    pub fn finish(&mut self, p: u8) {
        debug_assert!(p <= wire::MAX_PAYLOAD);
        debug_assert!((p as usize) <= N - 7);
        if wire::needs_pad(p) {
            self.bytes[4 + p as usize] = 0;
            self.bytes[3] |= Inst::FLAG_PAD;
        } else {
            self.bytes[3] &= !Inst::FLAG_PAD;
        }
        self.bytes[2] = wire::len_for(p);
    }

    /// CRC-covered span `[0, covered_len)` — valid after `finish`.
    #[inline]
    pub fn covered(&self) -> &[u8] {
        &self.bytes[..wire::covered_len(self.bytes[2])]
    }

    /// Write the little-endian CRC into the two bytes after the covered span.
    #[inline]
    pub fn set_crc(&mut self, crc: u16) {
        let end = wire::covered_len(self.bytes[2]);
        self.bytes[end..end + 2].copy_from_slice(&crc.to_le_bytes());
    }

    /// The whole frame `[0, footprint)`. The chip's UART DMA sends `frame()[1..]`
    /// and the CRC DMA reads `frame()` from offset 0.
    #[inline]
    pub fn frame(&self) -> &[u8] {
        &self.bytes[..wire::footprint(self.bytes[2])]
    }

    /// Raw storage — escape hatch for the chip driver's zero-copy tail layout
    /// (§4.2), where the buffer holds only header and CRC tail around an
    /// externally streamed payload. Layout invariants become the caller's.
    #[inline]
    pub fn bytes(&self) -> &[u8; N] {
        &self.bytes
    }

    /// Mutable counterpart of [`bytes`](Self::bytes); same contract.
    #[inline]
    pub fn bytes_mut(&mut self) -> &mut [u8; N] {
        &mut self.bytes
    }

    /// Compute and store the CRC over the covered span, returning the frame.
    #[cfg(feature = "software-crc")]
    #[inline]
    pub fn seal(&mut self) -> &[u8] {
        let crc = crate::crc::osc_crc(self.covered());
        self.set_crc(crc);
        self.frame()
    }
}

impl<const N: usize> Default for FrameBuf<N> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, feature = "software-crc"))]
mod tests {
    use super::*;
    use crate::frame::Header;
    use crate::wire::{Opcode, ResultCode};

    #[test]
    fn ping_vector() {
        let mut b = FrameBuf::<16>::new();
        b.start(Id::new(1), Inst::instruction(Opcode::Ping, 0));
        b.finish(0);
        assert_eq!(b.seal(), &[0x00, 0x01, 0x03, 0x10, 0x0A, 0x74]);
    }

    #[test]
    fn write_vector() {
        let mut b = FrameBuf::<16>::new();
        b.start(Id::new(5), Inst::instruction(Opcode::Write, 0));
        let pl = b.payload_mut();
        pl[..2].copy_from_slice(&0x0180u16.to_le_bytes());
        pl[2] = 0x2C;
        pl[3] = 0x01;
        b.finish(4);
        assert_eq!(
            b.seal(),
            &[0x00, 0x05, 0x07, 0x30, 0x80, 0x01, 0x2C, 0x01, 0xAE, 0xC9]
        );
    }

    #[test]
    fn read_vector() {
        let mut b = FrameBuf::<16>::new();
        b.start(Id::new(3), Inst::instruction(Opcode::Read, 0));
        let pl = b.payload_mut();
        pl[..2].copy_from_slice(&0x0200u16.to_le_bytes());
        pl[2..4].copy_from_slice(&8u16.to_le_bytes());
        b.finish(4);
        assert_eq!(
            b.seal(),
            &[0x00, 0x03, 0x07, 0x20, 0x00, 0x02, 0x08, 0x00, 0x70, 0x19]
        );
    }

    #[test]
    fn write_padded_vector_sets_pad_bit() {
        let mut b = FrameBuf::<16>::new();
        b.start(Id::new(2), Inst::instruction(Opcode::Write, 0));
        let pl = b.payload_mut();
        pl[..2].copy_from_slice(&0x0100u16.to_le_bytes());
        pl[2] = 0xAA;
        b.finish(3);
        let frame = b.seal();
        assert_eq!(frame[3], 0x32);
        assert_eq!(
            frame,
            &[0x00, 0x02, 0x07, 0x32, 0x00, 0x01, 0xAA, 0x00, 0xA8, 0x46]
        );
    }

    #[test]
    fn status_frame_round_trips() {
        let mut b = FrameBuf::<16>::new();
        b.start(Id::new(7), Inst::status(ResultCode::Ok, false, false));
        b.finish(0);
        let frame = b.seal();

        let h = Header::from_bytes(frame[..4].try_into().unwrap());
        assert_eq!(h.id, Id::new(7));
        assert!(h.inst.is_status());
        assert_eq!(h.inst.result(), Some(ResultCode::Ok));
        assert_eq!(h.validate(), Ok(()));
    }
}
