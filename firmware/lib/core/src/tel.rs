//! TEL stream payload layout: the bounded burst of osc-native status
//! frames (result = `Stream`, protocol sec 5.6) a servo emits on the main
//! bus after a committed nonzero `tel_count` write, at [`BAUD`] only. Each
//! frame is break-delimited and osc-CRC-16 protected, so a corrupt frame
//! is detectable; this module owns only the payload bytes between INST and
//! CRC. Layout, all LE:
//!
//!   [0]    stream_seq  u8, increments per frame, wraps
//!   [1]    flags       bit 0 = LAST frame of the burst; rest reserved 0
//!   [2..4] tick        u16, row 0's `sample_tick`, low half
//!   [4..]  rows        per row the bytes of the CONTROL chunks at their
//!                      table layout; the count is in LEN

pub use osc_protocol::tel::{BAUD, CHUNKS_MAX, Chunk, Chunks, ROW_BYTES_MAX};
use osc_protocol::wire;

use crate::budget::LAW_BREAK_BIT_TIMES;

/// The TEL `duty` a logical Q15 duty drives at `pwm_arr`: the kernel's
/// `window::drive_ticks` width, negative in reverse. Hosts quantize a goal
/// with it to compare against the rows.
pub fn duty_counts(duty_q15: i16, pwm_arr: u16) -> i16 {
    let ticks = crate::estimator::window::drive_ticks(duty_q15, pwm_arr) as i16;
    if duty_q15 < 0 { -ticks } else { ticks }
}

/// Payload flags bit 0: last frame of the burst; the line frees after it.
pub const FLAG_LAST: u8 = 1 << 0;

/// `TelemetryTick::tick_flags` bits, written every tick.
pub mod tick_flag {
    /// The shunt read the window `duty_ticks` drove: `i_tick_counts` is
    /// fresh, not held.
    pub const I_VALID: u16 = 1 << 0;
    /// A fault is latched.
    pub const FAULT: u16 = 1 << 1;
    /// The terminal taps read the window: `vdiff_counts` is fresh.
    pub const V_VALID: u16 = 1 << 2;
}

/// Fixed batch size: one frame carries up to 16 fast-tick rows (the
/// burst's last frame may carry fewer).
pub const STREAM_SAMPLES_MAX: usize = wire::STREAM_SAMPLES_MAX;

/// The longest burst a `tel_count` write arms: one second of control
/// ticks. The host is silent for the whole burst (protocol sec 5.6).
pub const BURST_ROWS_MAX: u16 = crate::budget::TICK_HZ as u16;

const _: () = assert!(crate::budget::TICK_HZ <= u16::MAX as u32);

pub const STREAM_HDR: usize = wire::STREAM_HDR;

/// Ticks of a batch a frame leaves its TX lane besides its wire time: the
/// lane breaks on one tick, arms on the next, and sees the frame end on a
/// tick (`osc_servo_drivers::tel`).
pub const LANE_TICKS: usize = 2;

/// A full frame of `row`-byte rows, break to CRC, clears the [`BAUD`] wire
/// inside its batch's ticks less [`LANE_TICKS`].
pub const fn frame_fits(row: usize) -> bool {
    let tick_bits = BAUD.as_hz() / crate::budget::TICK_HZ;
    let bits = (STREAM_SAMPLES_MAX - LANE_TICKS) as u32 * tick_bits;
    let chars = (bits.saturating_sub(LAW_BREAK_BIT_TIMES) / wire::CHAR_BIT_TIMES) as usize;
    // ID, LEN, INST and CRC around the payload (sec 3.1).
    let framing = wire::footprint(wire::len_for(0)) - 1;
    framing + STREAM_HDR + STREAM_SAMPLES_MAX * row <= chars
}

const _: () = assert!(frame_fits(ROW_BYTES_MAX));

pub const STREAM_PAYLOAD_MAX: usize = STREAM_HDR + STREAM_SAMPLES_MAX * ROW_BYTES_MAX;

/// The kernel's per-fast-tick stream hook; core never names the transport.
pub trait TelStream {
    /// The tick block is published: the row's first chunk may be taken.
    fn on_tick(&mut self);
    /// The tick runs a SLOW or thermometer stage: the TX lane takes no step
    /// on it. Any time before [`Self::on_tick_end`].
    fn hold(&mut self) {}
    /// The tick's last work, after the drive is written: the row's second
    /// chunk, then the TX lane.
    fn on_tick_end(&mut self) {}
}

/// Stream absent (boards without a burst producer, tests).
impl TelStream for () {
    fn on_tick(&mut self) {}
}

/// Serialize the stream head at the front of `buf`: everything row 0
/// knows. Order-free vs the rows: it never touches bytes past
/// [`STREAM_HDR`]; a `buf` shorter than that stays untouched.
pub fn encode_stream_head(stream_seq: u8, last: bool, tick: u16, buf: &mut [u8]) {
    if let Some(h) = buf.first_chunk_mut::<STREAM_HDR>() {
        let [t0, t1] = tick.to_le_bytes();
        *h = [stream_seq, if last { FLAG_LAST } else { 0 }, t0, t1];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame of 16 12 B rows at 3 M takes 694 us of the batch's 700 us
    /// less the lane's two ticks; 13 B rows would take 747. The buffers
    /// hold the widest row.
    #[test]
    fn twelve_byte_rows_fit_the_three_megabaud_batch() {
        assert!(frame_fits(ROW_BYTES_MAX));
        assert!(!frame_fits(ROW_BYTES_MAX + 1));
        assert_eq!(STREAM_PAYLOAD_MAX, STREAM_HDR + 16 * 12);
        assert!(STREAM_PAYLOAD_MAX <= wire::MAX_PAYLOAD as usize);
    }

    /// The host detects the burst end through osc-protocol's mirror of the
    /// LAST bit; the two consts must never drift.
    #[test]
    fn flag_last_matches_the_protocol_mirror() {
        use osc_protocol::FrameBytes;
        use osc_protocol::wire::{Inst, ResultCode, STREAM_FLAG_LAST, stream_last};
        assert_eq!(FLAG_LAST, STREAM_FLAG_LAST);
        let inst = Inst::status(ResultCode::Stream, false);
        let mut buf = [0u8; STREAM_HDR + 2];
        encode_stream_head(0, true, 0, &mut buf);
        assert!(stream_last(inst, FrameBytes::from(&buf[..])));
        encode_stream_head(0, false, 0, &mut buf);
        assert!(!stream_last(inst, FrameBytes::from(&buf[..])));
    }

    #[test]
    fn head_carries_seq_flags_and_the_stamp() {
        let mut buf = [0xEE; STREAM_HDR + 1];
        encode_stream_head(0x42, true, 0x1234, &mut buf);
        assert_eq!(buf, [0x42, FLAG_LAST, 0x34, 0x12, 0xEE]);
        let mut short = [0xEE; STREAM_HDR - 1];
        encode_stream_head(1, false, 2, &mut short);
        assert_eq!(short, [0xEE; STREAM_HDR - 1]);
    }
}
