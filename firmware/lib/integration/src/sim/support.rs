//! Test-support builders and checkers over the wire images the sim records
//! (`docs/osc-native-protocol.md` sec 3). Tests read these to assert on decoded
//! shape, not raw bytes.

use osc_protocol::crc::osc_crc;
use osc_protocol::frame::Header;
use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{self, Id, Inst, Opcode};
use osc_servo_core::tel::{STREAM_PAYLOAD_MAX, STREAM_SAMPLES_MAX, TelSample, encode_stream};

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

/// Assert a recorded frame is well-formed: LEN covers INST + CRC (sec 3.1) and
/// the trailing CRC matches osc-CRC over the covered span (sec 3.2 -- the leading
/// break byte is a no-op).
pub fn assert_valid(frame: &WireFrame) {
    let b = &frame.bytes;
    assert!(b.len() >= 6, "frame too short: {b:02X?}");
    let len = b[2];
    assert!(len >= 3, "LEN must cover INST + CRC: {b:02X?}");
    let covered = wire::covered_len(len);
    assert!(b.len() >= covered + 2, "frame truncated: {b:02X?}");
    let want = osc_crc(&b[..covered]);
    let got = u16::from_le_bytes([b[covered], b[covered + 1]]);
    assert_eq!(want, got, "CRC mismatch on {b:02X?}");
}

/// [`assert_valid`] as a predicate -- the host-side keep/drop gate for
/// unsolicited TEL burst frames (a corrupt frame drops; the stream seq
/// numbering exposes the gap).
pub fn frame_crc_ok(frame: &WireFrame) -> bool {
    let b = &frame.bytes;
    if b.len() < 6 || b[2] < 3 {
        return false;
    }
    let covered = wire::covered_len(b[2]);
    if b.len() < covered + 2 {
        return false;
    }
    osc_crc(&b[..covered]) == u16::from_le_bytes([b[covered], b[covered + 1]])
}

/// Deterministic fast-tick sample for per-burst tick `i` -- the pump feeds
/// these, so tests compute expected payload bytes from the same function.
pub fn tel_sample(i: u32) -> TelSample {
    TelSample {
        pos: 0x4000u16.wrapping_add(i as u16),
        current: -(i as i16) - 1,
        current_trough: 0xB000u16.wrapping_add(i as u16),
        duty_q15: 0x2000i16.wrapping_add(i as i16),
        vdiff: (-300i16).wrapping_sub(i as i16),
        vbus: 1800u16.wrapping_add(i as u16),
        current_raw: 0x0100u16.wrapping_add(i as u16),
        vmotor_a: 0x0A00u16.wrapping_add(i as u16),
        vmotor_b: 0x0B00u16.wrapping_add(i as u16),
        vbus_raw: 0x0C00u16.wrapping_add(i as u16),
        ntc_raw: 0x0D00u16.wrapping_add(i as u16),
        window_valid: i.is_multiple_of(2),
        fault: false,
    }
}

/// Expected payload of burst frame `seq` for a `count`-sample burst whose
/// ticks ran uninterrupted from 0 -- the pump's synthesis run through the
/// same encoder, so tests pin payload bytes end to end.
pub fn expect_tel_payload(mask: u16, count: u32, seq: usize) -> Vec<u8> {
    let samples: Vec<TelSample> = (0..count).map(tel_sample).collect();
    let a = seq * STREAM_SAMPLES_MAX;
    let b = (a + STREAM_SAMPLES_MAX).min(count as usize);
    let mut buf = [0u8; STREAM_PAYLOAD_MAX];
    let n = encode_stream(
        mask,
        seq as u8,
        b == count as usize,
        &samples[a..b],
        &mut buf,
    );
    buf[..n].to_vec()
}

/// Decode a status frame into its `INST` byte and payload slice.
pub fn status(frame: &WireFrame) -> (Inst, &[u8]) {
    let b = &frame.bytes;
    let head: &[u8; 4] = b[..4].try_into().expect("frame has a 4-byte header");
    let h = Header::from_bytes(head);
    let p = h.payload_len() as usize;
    (h.inst, &b[Header::SIZE..Header::SIZE + p])
}
