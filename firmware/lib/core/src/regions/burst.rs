//! BURST region: readback surface for the high-rate shunt capture. The buffer
//! is `BURST_LEN` raw codes, frame by frame: the shunt, then each extra
//! channel `control.burst.chans` selected. That is far past the `MAX_PAYLOAD`
//! reply ceiling, so it is exposed as a paged span instead of a field: the
//! host selects `control.burst.page` and one READ from the section base
//! returns that page plus the per-page header. `chans_echo` / `frame_len` sit
//! past that READ and hold for the whole capture. All-RO; the chip is the
//! sole writer.
//!
//! The page handshake carries no lock. The copier writes `page_echo =
//! PAGE_MID_COPY`, then the samples, then `page_echo = page`; the reply
//! snapshot reads offset 0 first, so an observed `page_echo == page` proves
//! the samples behind it belong to that page. Any interleave reads back
//! `PAGE_MID_COPY` or the previous page and the host retries.

use control_table::{Block, Section};

/// Samples per capture: a whole number of pages, and at the 1.083 us
/// conversion period a ~1.04 ms window. Divisible by every `frame_len`, and so
/// is its half, the step sample.
pub const BURST_LEN: usize = 960;
/// Samples per readback page: 240 B, which with the header fits one 252 B reply.
pub const PAGE_SAMPLES: usize = 120;
/// Readback pages covering the buffer.
pub const PAGES: usize = BURST_LEN / PAGE_SAMPLES;

/// `page_echo` while the copier is mid-page: matches no valid page.
pub const PAGE_MID_COPY: u8 = 0xFF;

/// `state` values. Plain consts, not an `Enum` derive: the field is RO, so no
/// discriminant validation ever runs on it and a u8 cannot be made unsound.
pub mod state {
    pub const IDLE: u8 = 0;
    pub const ARMED: u8 = 1;
    pub const CAPTURING: u8 = 2;
    pub const DONE: u8 = 3;
    pub const REJECTED: u8 = 4;
}

/// `control.burst.chans` bits. The shunt is always slot 0 of a frame; the
/// selected extras follow in bit order.
pub mod chans {
    pub const VMOTOR_A: u8 = 1 << 0;
    pub const VMOTOR_B: u8 = 1 << 1;
    pub const VBUS: u8 = 1 << 2;
    pub const ALL: u8 = VMOTOR_A | VMOTOR_B | VBUS;
}

/// Conversions per frame for a `chans` mask.
#[inline]
pub const fn frame_len(mask: u8) -> u8 {
    1 + (mask & chans::ALL).count_ones() as u8
}

/// The widest frame: the shunt plus every extra.
pub const FRAME_MAX: usize = frame_len(chans::ALL) as usize;

/// `start_dir` / `restore_dir` encoding: the TIM1 CTLR1.DIR bit verbatim, so
/// the published byte reads as the silicon does. Under center-aligned PWM DOWN
/// is the crest-to-trough half, which is where a peak scan's TC lands -- so
/// `restore_dir == DOWN` is the witness that the crest scan landed second and
/// the trough slots are back at offset 0.
pub mod dir {
    pub const UP: u8 = 0;
    pub const DOWN: u8 = 1;
}

/// Sample index range `[start, end)` of readback `page`; `None` past the last
/// page (sec 5.3: `range`).
#[inline]
pub const fn page_span(page: u8) -> Option<(usize, usize)> {
    if page as usize >= PAGES {
        return None;
    }
    let at = page as usize * PAGE_SAMPLES;
    Some((at, at + PAGE_SAMPLES))
}

/// One READ from the section base returns `samples` for the selected page and
/// every per-page header word, so a host never has to correlate two replies.
/// `step_index` is measured from the DMA counter at the step, not assumed;
/// `start_cnt` / `start_dir` / `pwm_arr` cross-check the trace's own PWM
/// edges, which are the ruler. `restore_dir` is the scan-geometry witness:
/// DOWN means the crest scan landed second, as it does at bringup.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct BurstWindow {
    #[ct_field(access = ro)]
    pub page_echo: u8,
    #[ct_field(access = ro)]
    pub state: u8,
    #[ct_field(access = ro)]
    pub samples: [u16; PAGE_SAMPLES],
    #[ct_field(access = ro)]
    pub samples_len: u16,
    #[ct_field(access = ro)]
    pub step_index: u16,
    #[ct_field(access = ro)]
    pub start_cnt: u16,
    #[ct_field(access = ro)]
    pub pwm_arr: u16,
    #[ct_field(access = ro)]
    pub start_dir: u8,
    #[ct_field(access = ro)]
    pub restore_dir: u8,
    #[ct_field(access = ro)]
    pub chans_echo: u8,
    #[ct_field(access = ro)]
    pub frame_len: u8,
    #[ct_field(skip)]
    pub _rsvd: [u8; 2],
}

#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::BURST_BASE_ADDR,
    size = crate::regions::BURST_REGION_SIZE,
)]
pub struct BurstRegs {
    pub window: BurstWindow,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn region_fits_declared_size() {
        assert_eq!(
            size_of::<BurstRegs>(),
            crate::regions::BURST_REGION_SIZE as usize
        );
    }

    /// The header must sit behind the samples so one 252 B READ from the
    /// section base carries the page AND every word describing it.
    #[test]
    fn one_read_covers_the_page_and_the_header() {
        use addr::window;
        assert_eq!(window::PAGE_ECHO, crate::regions::BURST_BASE_ADDR);
        assert_eq!(window::SAMPLES, window::PAGE_ECHO + 2);
        assert_eq!(
            window::SAMPLES_LEN,
            window::SAMPLES + 2 * PAGE_SAMPLES as u16
        );
        assert_eq!(window::RESTORE_DIR, window::SAMPLES_LEN + 9);
        assert_eq!(
            window::RESTORE_DIR + 1 - window::PAGE_ECHO,
            osc_protocol::wire::MAX_PAYLOAD as u16
        );
    }

    /// The published dir byte is the TIM1 CTLR1.DIR bit, not a re-encoding:
    /// a host comparing against the reference manual must be right.
    #[test]
    fn dir_encoding_is_the_timer_bit() {
        assert_eq!((dir::UP, dir::DOWN), (0, 1));
    }

    /// Slot 0 is the shunt, then one slot per selected extra; every frame
    /// length tiles the capture and lands the step half on a frame boundary.
    #[test]
    fn frame_len_counts_the_shunt_and_every_extra() {
        let expect = [1, 2, 2, 3, 2, 3, 3, 4];
        for (mask, &len) in expect.iter().enumerate() {
            let n = frame_len(mask as u8);
            assert_eq!(n, len, "chans {mask:#05b}");
            assert_eq!(BURST_LEN % n as usize, 0, "chans {mask:#05b}");
            assert_eq!((BURST_LEN / 2) % n as usize, 0, "chans {mask:#05b}");
        }
        assert_eq!(chans::ALL, 7);
        assert_eq!(FRAME_MAX, 4);
    }

    #[test]
    fn pages_tile_the_buffer_exactly() {
        assert_eq!(PAGES * PAGE_SAMPLES, BURST_LEN);
        assert_eq!(page_span(0), Some((0, PAGE_SAMPLES)));
        assert_eq!(
            page_span(PAGES as u8 - 1),
            Some((BURST_LEN - PAGE_SAMPLES, BURST_LEN))
        );
        assert_eq!(page_span(PAGES as u8), None);
        assert_eq!(page_span(PAGE_MID_COPY), None);
    }
}
