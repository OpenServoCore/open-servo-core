//! TEL stream fit: the bounded burst of osc-native status frames (result =
//! `Stream`, protocol sec 5.6) a servo emits on the main bus after a
//! committed nonzero `tel_count` write, at [`BAUD`] only, sized to the
//! kernel's tick. The payload layout is `osc_protocol::tel`'s.

pub use osc_protocol::tel::{BAUD, CHUNKS_MAX, Chunk, Chunks, ROW_BYTES_MAX};
use osc_protocol::wire::{self, LAW_BREAK_BIT_TIMES};

/// Fixed batch size: one frame carries up to 16 fast-tick rows (the
/// burst's last frame may carry fewer).
pub const STREAM_SAMPLES_MAX: usize = wire::STREAM_SAMPLES_MAX;

pub const STREAM_HDR: usize = wire::STREAM_HDR;

/// Ticks of a batch a frame leaves its TX lane besides its wire time: the
/// lane breaks on one tick, arms on the next, and sees the frame end on a
/// tick (`osc_servo_drivers::tel`).
pub const LANE_TICKS: usize = 2;

/// A full frame of `row`-byte rows, break to CRC, clears the [`BAUD`] wire
/// inside its batch's ticks less [`LANE_TICKS`].
pub const fn frame_fits(row: usize) -> bool {
    let tick_bits = BAUD.as_hz() / crate::kernel::TICK_HZ;
    let bits = (STREAM_SAMPLES_MAX - LANE_TICKS) as u32 * tick_bits;
    let chars = (bits.saturating_sub(LAW_BREAK_BIT_TIMES) / wire::CHAR_BIT_TIMES) as usize;
    // ID, LEN, INST and CRC around the payload (sec 3.1).
    let framing = wire::footprint(wire::len_for(0)) - 1;
    framing + STREAM_HDR + STREAM_SAMPLES_MAX * row <= chars
}

const _: () = assert!(frame_fits(ROW_BYTES_MAX));

pub const STREAM_PAYLOAD_MAX: usize = STREAM_HDR + STREAM_SAMPLES_MAX * ROW_BYTES_MAX;

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
}
