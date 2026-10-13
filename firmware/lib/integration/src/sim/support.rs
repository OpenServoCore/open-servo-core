//! Test-support builders and checkers over the wire images the sim records
//! (`docs/osc-native-protocol.md` sec 3). Tests read these to assert on decoded
//! shape, not raw bytes.

use osc_protocol::crc::osc_crc;
use osc_protocol::frame::Header;
use osc_protocol::reply::FrameBuf;
use osc_protocol::tel::encode_stream_head;
use osc_protocol::wire::{self, Id, Inst, Opcode, ResultCode};
use osc_servo_core::regions::telemetry::tick_flag;
use osc_servo_core::tel::{Chunks, STREAM_HDR, STREAM_SAMPLES_MAX};
use osc_servo_core::{ControlTableCell, RegionStorage, RegionStorageRaw, TelemetryTick};

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

/// Build and seal a status frame as a servo would put it on the wire - for a
/// host that forges one (a status-shaped flood, a garbled predecessor).
pub fn status_frame(id: u8, result: ResultCode, data: &[u8]) -> Vec<u8> {
    let mut b = FrameBuf::<264>::new();
    b.start(Id::new(id), Inst::status(result, false));
    b.payload_mut()[..data.len()].copy_from_slice(data);
    b.finish(data.len() as u8);
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

/// One fast tick as the sim plays it into a servo's table: the tick block
/// the ADC DMA, the ISR and the kernel write (`TelemetryTick`; the sim
/// stamps `sample_tick` itself) and the rail estimate the row sees, as
/// the medium step last published it.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelSample {
    pub tick: TelemetryTick,
    pub vbus_counts: u16,
}

/// Deterministic fast-tick sample for per-burst tick `i`, every field
/// moving every tick: the pump feeds these, so tests compute expected
/// payload bytes from the same function. `sample_tick` is `i`: the pump's
/// stand-in for the ISR's count.
pub fn tel_sample(i: u32) -> TelSample {
    let raw = |base: u16| base | (i as u16 & 0xFF);
    let valid = if i.is_multiple_of(2) {
        tick_flag::I_VALID | tick_flag::V_VALID
    } else {
        0
    };
    TelSample {
        tick: TelemetryTick {
            i_tick_counts: -(i as i16) - 1,
            vdiff_counts: (-300i16).wrapping_sub(i as i16),
            duty_ticks: (i % 2401) as i16 - 1200,
            pos: raw(0x400),
            current_trough: raw(0xB00),
            current_raw: raw(0x100),
            vmotor_a: raw(0xA00),
            vmotor_b: raw(0x600),
            vbus_raw: raw(0xC00),
            ntc_raw: raw(0xD00),
            vcal: raw(0x500),
            vmotor_a_trough: raw(0x700),
            vmotor_b_trough: raw(0x800),
            tick_flags: valid,
            sample_tick: i,
            _scan_trough: [raw(0x900); 7],
            _rsvd_align: 0,
        },
        vbus_counts: 1800u16.wrapping_add(i as u16),
    }
}

/// The row `chunks` copy from a table holding `s` at tick `tick`: the
/// table's bytes at the chunks' addresses, back to back.
pub fn tel_row(chunks: Chunks, s: &TelSample, tick: u32) -> Vec<u8> {
    let table = ControlTableCell::new();
    table.with_mut(|t| {
        t.telemetry.tick = TelemetryTick {
            sample_tick: tick,
            ..s.tick
        };
        t.telemetry.estimates.vbus_counts = s.vbus_counts;
    });
    let base = table.region_ptr().cast::<u8>();
    let mut row = Vec::new();
    for c in chunks.0 {
        for a in c.addr..c.addr + c.len {
            // SAFETY: test-only read of the local table; the chunks a test
            // arms lie inside it (`Chunks::valid`).
            row.push(unsafe { base.add(a as usize).read() });
        }
    }
    row
}

/// Expected payload of burst frame `seq` for a `count`-sample burst of
/// `chunks` whose ticks ran uninterrupted from 0: the pump's synthesis
/// copied the way the servo copies it, so tests pin payload bytes end to
/// end.
pub fn expect_tel_payload(chunks: Chunks, count: u32, seq: usize) -> Vec<u8> {
    let samples: Vec<TelSample> = (0..count).map(tel_sample).collect();
    expect_tel_payload_rows(chunks, &samples, seq)
}

/// Expected payload of burst frame `seq` when the burst's ticks served
/// exactly `rows`, in order, from tick 0 (a played-back track, sliced at
/// its cursor).
pub fn expect_tel_payload_rows(chunks: Chunks, rows: &[TelSample], seq: usize) -> Vec<u8> {
    let a = seq * STREAM_SAMPLES_MAX;
    let b = (a + STREAM_SAMPLES_MAX).min(rows.len());
    let mut buf = vec![0u8; STREAM_HDR];
    encode_stream_head(seq as u8, b == rows.len(), a as u16, &mut buf);
    for (k, s) in rows[a..b].iter().enumerate() {
        buf.extend(tel_row(chunks, s, (a + k) as u32));
    }
    buf
}

/// Decode a status frame into its `INST` byte and payload slice.
pub fn status(frame: &WireFrame) -> (Inst, &[u8]) {
    let b = &frame.bytes;
    let head: &[u8; 4] = b[..4].try_into().expect("frame has a 4-byte header");
    let h = Header::from_bytes(head);
    let p = h.payload_len() as usize;
    (h.inst, &b[Header::SIZE..Header::SIZE + p])
}
