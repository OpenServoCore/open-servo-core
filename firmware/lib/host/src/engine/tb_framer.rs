//! Host RX framer for tinyboot frames: a ring walker anchored by the sync
//! word (link sec 7.1).
//!
//! tinyboot frames carry no break, so a candidate starts at any `AA 55` at
//! or past the anchor; the header makes its end computable, `max_data`
//! bounds a false claim, and the CRC verdict is the only accept authority.
//! A failed claim gives up one byte only: a real frame may start inside it.

use osc_protocol::FrameBytes;

use super::framer::payload_view;

pub const SYNC: [u8; 2] = [0xAA, 0x55];
/// `AA 55 cmd status addr(3) flags len(2)`.
pub const HEADER: usize = 10;
/// Header plus the trailing `crc(2)`.
pub const OVERHEAD: usize = HEADER + 2;
pub const STATUS_REQUEST: u8 = 0;

/// CRC-16/CCITT: poly 0x1021, init 0xFFFF, non-reflected, no xorout.
pub fn tb_crc(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// One CRC-clean, non-request frame view over the ring. `data_pos` names
/// the data's ring index so a caller can re-materialize the view after the
/// borrow ends, as [`Frame::payload_pos`](super::framer::Frame::payload_pos).
#[derive(Debug)]
pub struct TbFrame<'a> {
    pub cmd: u8,
    pub status: u8,
    pub addr: u32,
    pub flags: u8,
    pub data: FrameBytes<'a>,
    pub data_pos: usize,
}

/// Outcome of one [`TbFramer::step`]; callers loop until `Idle` or
/// `Partial`.
#[derive(Debug)]
pub enum TbStep<'a> {
    /// No unconsumed bytes.
    Idle,
    /// Unconsumed bytes wait for a frame's remainder or a sync's second
    /// byte. Whether either ever lands is the engine's deadline.
    Partial,
    /// One frame consumed.
    Frame(TbFrame<'a>),
    /// `n` ring bytes consumed that held no frame.
    Garble(u16),
    /// `n` whole rings landed unread: the walk resumes at the same ring
    /// index, over newer bytes.
    Lapped(u16),
}

/// The walker: a cursor (`anchor`) on the
/// [`RxRing::written`](crate::traits::RxRing::written) count.
#[derive(Default)]
pub struct TbFramer {
    anchor: u32,
}

impl TbFramer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Jump to `written`, discarding anything unconsumed. Called only when
    /// the bus is provably quiet (before a TX window opens).
    pub fn resync(&mut self, written: u32) {
        self.anchor = written;
    }

    /// Ring content no verdict has consumed.
    pub fn unresolved(&self, written: u32) -> u16 {
        u16::try_from(written.wrapping_sub(self.anchor)).unwrap_or(u16::MAX)
    }

    /// Walk one step. The ring length must be a power of two. `max_data`:
    /// frames whose len exceeds it are false syncs (link sec 7.1).
    pub fn step<'a>(&mut self, ring: &'a [u8], written: u32, max_data: u16) -> TbStep<'a> {
        debug_assert!(ring.len().is_power_of_two());
        let unread = written.wrapping_sub(self.anchor);
        let laps = unread / ring.len() as u32;
        if laps > 0 {
            self.anchor = self.anchor.wrapping_add(laps * ring.len() as u32);
            return TbStep::Lapped(u16::try_from(laps).unwrap_or(u16::MAX));
        }
        if unread == 0 {
            return TbStep::Idle;
        }
        let mask = ring.len() - 1;
        let at = self.anchor as usize & mask;
        let unread = unread as usize;
        let byte = |i: usize| ring[(at + i) & mask];

        let skip = (0..unread)
            .find(|&i| byte(i) == SYNC[0] && (i + 1 == unread || byte(i + 1) == SYNC[1]))
            .unwrap_or(unread);
        if skip > 0 {
            return self.consume(skip);
        }
        // A lone `AA` last: half a sync, perhaps.
        if unread < HEADER {
            return TbStep::Partial;
        }

        let len = u16::from_le_bytes([byte(8), byte(9)]) as usize;
        let footprint = OVERHEAD + len;
        if len > max_data as usize || footprint > ring.len() {
            return self.consume(1);
        }
        if unread < footprint {
            return TbStep::Partial;
        }
        let crc = ring_crc(ring, at, HEADER + len);
        if crc != u16::from_le_bytes([byte(HEADER + len), byte(HEADER + len + 1)]) {
            return self.consume(1);
        }
        let status = byte(3);
        if status == STATUS_REQUEST {
            return self.consume(footprint);
        }
        let data_pos = (at + HEADER) & mask;
        self.anchor = self.anchor.wrapping_add(footprint as u32);
        TbStep::Frame(TbFrame {
            cmd: byte(2),
            status,
            addr: u32::from_le_bytes([byte(4), byte(5), byte(6), 0]),
            flags: byte(7),
            data: payload_view(ring, data_pos, len),
            data_pos,
        })
    }

    fn consume(&mut self, n: usize) -> TbStep<'static> {
        self.anchor = self.anchor.wrapping_add(n as u32);
        TbStep::Garble(u16::try_from(n).unwrap_or(u16::MAX))
    }
}

/// tinyboot CRC over a possibly-wrapped ring span.
fn ring_crc(ring: &[u8], start: usize, n: usize) -> u16 {
    let head = n.min(ring.len() - start);
    let crc = tb_crc(0xFFFF, &ring[start..start + head]);
    tb_crc(crc, &ring[..n - head])
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX: u16 = 32;

    /// A 64-byte test ring with a running write count.
    struct Ring {
        buf: [u8; 64],
        written: u32,
    }

    impl Ring {
        fn new() -> Self {
            Ring {
                buf: [0xFF; 64],
                written: 0,
            }
        }

        fn feed(&mut self, bytes: &[u8]) {
            for &b in bytes {
                self.buf[self.written as usize & 63] = b;
                self.written += 1;
            }
        }

        fn step(&self, f: &mut TbFramer) -> TbStep<'_> {
            f.step(&self.buf, self.written, MAX)
        }
    }

    /// A sealed tinyboot frame in a fixed buffer.
    struct Wire {
        buf: [u8; 64],
        len: usize,
    }

    impl Wire {
        fn as_slice(&self) -> &[u8] {
            &self.buf[..self.len]
        }
    }

    fn tb_frame(cmd: u8, status: u8, addr: u32, flags: u8, data: &[u8]) -> Wire {
        let mut buf = [0u8; 64];
        let a = addr.to_le_bytes();
        let n = (data.len() as u16).to_le_bytes();
        buf[..HEADER]
            .copy_from_slice(&[0xAA, 0x55, cmd, status, a[0], a[1], a[2], flags, n[0], n[1]]);
        buf[HEADER..HEADER + data.len()].copy_from_slice(data);
        let crc = tb_crc(0xFFFF, &buf[..HEADER + data.len()]).to_le_bytes();
        buf[HEADER + data.len()..OVERHEAD + data.len()].copy_from_slice(&crc);
        Wire {
            buf,
            len: OVERHEAD + data.len(),
        }
    }

    fn reply(cmd: u8, data: &[u8]) -> Wire {
        tb_frame(cmd, 1, 0x0012_3456, 0x80, data)
    }

    fn expect_frame<'a>(step: TbStep<'a>) -> TbFrame<'a> {
        match step {
            TbStep::Frame(f) => f,
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    /// Walk to `Idle` or `Partial`; the cmds of the frames found and the
    /// garble total.
    fn walk(f: &mut TbFramer, ring: &Ring) -> (std::vec::Vec<u8>, u32) {
        let mut cmds = std::vec::Vec::new();
        let mut garble = 0;
        loop {
            match ring.step(f) {
                TbStep::Frame(fr) => cmds.push(fr.cmd),
                TbStep::Garble(n) => garble += n as u32,
                TbStep::Lapped(_) => {}
                TbStep::Partial | TbStep::Idle => return (cmds, garble),
            }
        }
    }

    #[test]
    fn crc_check_value() {
        assert_eq!(tb_crc(0xFFFF, b"123456789"), 0x29B1);
    }

    #[test]
    fn clean_frame_parses() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        ring.feed(tb_frame(3, 2, 0x0008_0400, 0x01, &[0xDE, 0xAD]).as_slice());

        let got = expect_frame(ring.step(&mut f));
        assert_eq!(got.cmd, 3);
        assert_eq!(got.status, 2);
        assert_eq!(got.addr, 0x0008_0400);
        assert_eq!(got.flags, 0x01);
        assert!(got.data.bytes().eq([0xDE, 0xAD]));
        assert_eq!(got.data_pos, HEADER);
        assert!(matches!(ring.step(&mut f), TbStep::Idle));
    }

    #[test]
    fn frame_split_across_the_wrap_parses() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        ring.feed(&[0xEE; 58]);
        f.resync(58);
        ring.feed(reply(5, &[0x11, 0x22, 0x33, 0x44]).as_slice());

        let got = expect_frame(ring.step(&mut f));
        assert_eq!(got.cmd, 5);
        assert_eq!(got.addr, 0x0012_3456);
        assert_eq!(got.data_pos, 4);
        assert!(got.data.bytes().eq([0x11, 0x22, 0x33, 0x44]));
        assert!(matches!(ring.step(&mut f), TbStep::Idle));
    }

    #[test]
    fn crc_failure_then_a_clean_frame_right_behind() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        let mut bad = reply(1, &[7, 7, 7]);
        bad.buf[HEADER + 1] ^= 0xFF;
        ring.feed(bad.as_slice());
        ring.feed(reply(2, &[9]).as_slice());

        assert!(matches!(ring.step(&mut f), TbStep::Garble(1)));
        assert_eq!(walk(&mut f, &ring), (std::vec![2], bad.len as u32 - 1));
    }

    #[test]
    fn false_sync_over_max_data_then_a_clean_frame() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        // A sync whose len claims 0x0100 bytes, well over MAX.
        let garble = [0xAA, 0x55, 0, 1, 0, 0, 0, 0, 0x00, 0x01];
        ring.feed(&garble);
        ring.feed(reply(4, &[1, 2]).as_slice());

        assert!(matches!(ring.step(&mut f), TbStep::Garble(1)));
        assert_eq!(walk(&mut f, &ring), (std::vec![4], garble.len() as u32 - 1));
    }

    #[test]
    fn request_status_frame_is_garble() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        let req = tb_frame(1, STATUS_REQUEST, 0, 0, &[5, 6]);
        ring.feed(req.as_slice());
        ring.feed(reply(1, &[]).as_slice());

        assert!(matches!(ring.step(&mut f), TbStep::Garble(n) if n as usize == req.len));
        assert_eq!(expect_frame(ring.step(&mut f)).cmd, 1);
        assert!(matches!(ring.step(&mut f), TbStep::Idle));
    }

    #[test]
    fn back_to_back_frames_walk_in_order() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        ring.feed(reply(1, &[1]).as_slice());
        ring.feed(reply(2, &[2, 2]).as_slice());

        assert_eq!(expect_frame(ring.step(&mut f)).cmd, 1);
        assert_eq!(expect_frame(ring.step(&mut f)).cmd, 2);
        assert!(matches!(ring.step(&mut f), TbStep::Idle));
    }

    #[test]
    fn garble_before_a_frame_is_skipped_to_the_sync() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        ring.feed(&[0x00, 0xAA, 0x13, 0x55]);
        ring.feed(reply(6, &[3]).as_slice());

        assert!(matches!(ring.step(&mut f), TbStep::Garble(4)));
        assert_eq!(expect_frame(ring.step(&mut f)).cmd, 6);
    }

    #[test]
    fn a_lone_trailing_sync_byte_is_partial() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        ring.feed(&[0xAA]);
        assert!(matches!(ring.step(&mut f), TbStep::Partial));
        assert_eq!(f.unresolved(ring.written), 1);

        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        ring.feed(&[0x01, 0x02, 0xAA]);
        assert!(matches!(ring.step(&mut f), TbStep::Garble(2)));
        assert!(matches!(ring.step(&mut f), TbStep::Partial));
        let frame = reply(8, &[4]);
        ring.feed(&frame.as_slice()[1..]);
        assert_eq!(expect_frame(ring.step(&mut f)).cmd, 8);
    }

    #[test]
    fn half_received_frame_is_partial_and_unresolved() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        let frame = reply(9, &[1, 2, 3, 4, 5]);
        let bytes = frame.as_slice();
        ring.feed(&bytes[..6]);
        assert!(matches!(ring.step(&mut f), TbStep::Partial));
        assert_eq!(f.unresolved(ring.written), 6);
        ring.feed(&bytes[6..12]);
        assert!(matches!(ring.step(&mut f), TbStep::Partial));
        assert_eq!(f.unresolved(ring.written), 12);
        ring.feed(&bytes[12..]);
        assert_eq!(expect_frame(ring.step(&mut f)).cmd, 9);
        assert_eq!(f.unresolved(ring.written), 0);
    }

    #[test]
    fn a_ring_landed_unread_is_a_lap() {
        let mut ring = Ring::new();
        let mut f = TbFramer::new();
        ring.feed(&[0xEE; 64]);
        assert!(matches!(ring.step(&mut f), TbStep::Lapped(1)));
        assert!(matches!(ring.step(&mut f), TbStep::Idle));

        ring.feed(&[0xEE; 3 * 64 + 10]);
        assert!(matches!(ring.step(&mut f), TbStep::Lapped(3)));
        ring.feed(reply(2, &[2]).as_slice());
        assert!(matches!(ring.step(&mut f), TbStep::Garble(10)));
        assert_eq!(expect_frame(ring.step(&mut f)).cmd, 2);
    }
}
