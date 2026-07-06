//! Test-support builders and checkers over the wire images the sim records
//! (`docs/osc-native-protocol.md` §3). Tests read these to assert on decoded
//! shape, not raw bytes.

use osc_protocol::crc::osc_crc;
use osc_protocol::frame::Header;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Id, Inst, Opcode};

use super::WireFrame;

/// Build and seal a host instruction frame: `FrameBuf::seal` output, leading
/// 0x00 CRC prefix included (the wire replays it as the break byte).
pub fn instruction(id: u8, op: Opcode, flags: u8, payload: &[u8]) -> Vec<u8> {
    let mut b = FrameBuf::<264>::new();
    b.start(Id::new(id), Inst::instruction(op, flags));
    b.payload_mut()[..payload.len()].copy_from_slice(payload);
    b.finish(payload.len() as u8);
    b.seal().to_vec()
}

/// Assert a recorded frame is well-formed: LEN odd (PAD invariant, §3.1) and
/// the trailing CRC matches osc-CRC over the covered span (§3.2).
pub fn assert_valid(frame: &WireFrame) {
    let b = &frame.bytes;
    assert!(b.len() >= 6, "frame too short: {b:02X?}");
    let len = b[2];
    assert_eq!(len & 1, 1, "LEN must be odd (PAD invariant): {b:02X?}");
    let covered = wire::covered_len(len);
    assert!(b.len() >= covered + 2, "frame truncated: {b:02X?}");
    let want = osc_crc(&b[..covered]);
    let got = u16::from_le_bytes([b[covered], b[covered + 1]]);
    assert_eq!(want, got, "CRC mismatch on {b:02X?}");
}

/// Decode a status frame into its `INST` byte and payload slice (pad dropped).
pub fn status(frame: &WireFrame) -> (Inst, &[u8]) {
    let b = &frame.bytes;
    let head: &[u8; 4] = b[..4].try_into().expect("frame has a 4-byte header");
    let h = Header::from_bytes(head);
    let p = h.payload_len() as usize;
    (h.inst, &b[Header::SIZE..Header::SIZE + p])
}
