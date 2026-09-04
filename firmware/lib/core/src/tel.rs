//! TEL streaming frame layout: the per-fast-tick binary side-channel
//! (USART2/TEL pad) the host identification fitter consumes. Transport-
//! independent and mirrored verbatim by the host parser, so every layout
//! fact lives here. Frame = 3 B header (sync, seq, flags) + the fields
//! selected by `tel_mask` in bit order, little-endian; length is a pure
//! function of the mask. No CRC: the link is a point-to-point wire and
//! sync + seq continuity is the integrity mechanism - a corrupt byte
//! surfaces as a realign plus seq gap at the host.

/// Not 0x00/0xFF (dominant idle/saturated data bytes); false locks resolve
/// via seq continuity, so low collision odds suffice.
pub const SYNC: u8 = 0xA7;

pub const HDR_LEN: usize = 3;

/// Header flags bit 0: this tick's drive window met the sampling floors
/// (current/vdiff carry a fresh measurement, not last-valid hold).
pub const FLAG_WINDOW_VALID: u8 = 1 << 0;

/// `tel_mask` bits, in canonical frame order. All v1 fields are 2 bytes.
pub const BIT_POS: u16 = 1 << 0;
pub const BIT_CURRENT: u16 = 1 << 1;
pub const BIT_CURRENT_TROUGH: u16 = 1 << 2;
pub const BIT_DUTY: u16 = 1 << 3;
pub const BIT_VDIFF: u16 = 1 << 4;
pub const BIT_VBUS: u16 = 1 << 5;
pub const MASK_ALL: u16 = 0x3F;

/// One fast tick on the wire: 3 Mbaud / 10 bits per byte / 20 kHz = 15 B.
/// A longer frame cannot sustain one-per-tick; masks over budget are
/// rejected. The all-bits v1 frame lands exactly here.
pub const FRAME_BUDGET: usize = 15;

pub const fn frame_len(mask: u16) -> usize {
    HDR_LEN + 2 * (mask & MASK_ALL).count_ones() as usize
}

/// Reserved bits and over-budget frames are both invalid; mask 0 is valid
/// (stream off). Budget is structural headroom - every v1 mask fits.
pub const fn mask_valid(mask: u16) -> bool {
    mask & !MASK_ALL == 0 && frame_len(mask) <= FRAME_BUDGET
}

/// One fast tick's streamable primitives, in device counts. `current` is
/// the signed bias-subtracted window sample (the fitter's domain, matching
/// `i_hat_counts`); `current_trough` and `vbus` stay raw unsigned like
/// their telemetry counterparts.
#[derive(Copy, Clone, Debug, Default)]
pub struct TelSample {
    pub pos: u16,
    pub current: i16,
    pub current_trough: u16,
    pub duty_q15: i16,
    pub vdiff: i16,
    pub vbus: u16,
    pub window_valid: bool,
}

/// The kernel's per-fast-tick stream hook; core never names the transport.
pub trait TelStream {
    fn on_tick(&mut self, sample: &TelSample);
}

/// Stream absent (boards without TEL wiring, tests).
impl TelStream for () {
    fn on_tick(&mut self, _sample: &TelSample) {}
}

/// Serialize one frame into `buf`, returning its length. `mask` reserved
/// bits are ignored (callers gate on `mask_valid`); the fixed-size buffer
/// makes the writes structurally in-bounds.
pub fn encode(mask: u16, seq: u8, s: &TelSample, buf: &mut [u8; FRAME_BUDGET]) -> usize {
    buf[0] = SYNC;
    buf[1] = seq;
    buf[2] = if s.window_valid { FLAG_WINDOW_VALID } else { 0 };
    let mut n = HDR_LEN;
    let m = mask & MASK_ALL;
    let mut put = |bit: u16, le: [u8; 2]| {
        if m & bit != 0 {
            buf[n] = le[0];
            buf[n + 1] = le[1];
            n += 2;
        }
    };
    put(BIT_POS, s.pos.to_le_bytes());
    put(BIT_CURRENT, s.current.to_le_bytes());
    put(BIT_CURRENT_TROUGH, s.current_trough.to_le_bytes());
    put(BIT_DUTY, s.duty_q15.to_le_bytes());
    put(BIT_VDIFF, s.vdiff.to_le_bytes());
    put(BIT_VBUS, s.vbus.to_le_bytes());
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_len_counts_selected_fields() {
        assert_eq!(frame_len(0), HDR_LEN);
        assert_eq!(frame_len(BIT_POS), HDR_LEN + 2);
        assert_eq!(frame_len(BIT_POS | BIT_DUTY | BIT_VDIFF), HDR_LEN + 6);
        assert_eq!(frame_len(MASK_ALL), FRAME_BUDGET);
    }

    #[test]
    fn all_bits_is_exactly_budget_and_valid() {
        assert_eq!(frame_len(MASK_ALL), FRAME_BUDGET);
        assert!(mask_valid(MASK_ALL));
        assert!(mask_valid(0));
    }

    #[test]
    fn reserved_bits_are_invalid() {
        assert!(!mask_valid(1 << 6));
        assert!(!mask_valid(1 << 15));
        assert!(!mask_valid(MASK_ALL | 1 << 6));
    }

    #[test]
    fn encode_golden_all_fields() {
        let s = TelSample {
            pos: 0x1234,
            current: -2,
            current_trough: 0xBEEF,
            duty_q15: 0x7FFF,
            vdiff: -300,
            vbus: 1822,
            window_valid: true,
        };
        let mut buf = [0u8; FRAME_BUDGET];
        let n = encode(MASK_ALL, 0x42, &s, &mut buf);
        assert_eq!(n, FRAME_BUDGET);
        let mut want = [0u8; FRAME_BUDGET];
        want[..3].copy_from_slice(&[SYNC, 0x42, FLAG_WINDOW_VALID]);
        want[3..5].copy_from_slice(&0x1234u16.to_le_bytes());
        want[5..7].copy_from_slice(&(-2i16).to_le_bytes());
        want[7..9].copy_from_slice(&0xBEEFu16.to_le_bytes());
        want[9..11].copy_from_slice(&0x7FFFi16.to_le_bytes());
        want[11..13].copy_from_slice(&(-300i16).to_le_bytes());
        want[13..15].copy_from_slice(&1822u16.to_le_bytes());
        assert_eq!(buf, want);
    }

    #[test]
    fn encode_subset_packs_in_bit_order() {
        let s = TelSample {
            pos: 100,
            current: -7,
            duty_q15: 5,
            vdiff: 9,
            ..Default::default()
        };
        let mut buf = [0u8; FRAME_BUDGET];
        // ladder mask: pos + current + duty + vdiff = 11 B
        let mask = BIT_POS | BIT_CURRENT | BIT_DUTY | BIT_VDIFF;
        let n = encode(mask, 7, &s, &mut buf);
        assert_eq!(n, 11);
        assert_eq!(buf[2], 0, "window_valid false");
        assert_eq!(buf[3..5], 100u16.to_le_bytes());
        assert_eq!(buf[5..7], (-7i16).to_le_bytes());
        assert_eq!(buf[7..9], 5i16.to_le_bytes());
        assert_eq!(buf[9..11], 9i16.to_le_bytes());
    }

    #[test]
    fn encode_tail_stays_untouched() {
        let mut buf = [0xEEu8; FRAME_BUDGET];
        let n = encode(BIT_VBUS, 1, &TelSample::default(), &mut buf);
        assert_eq!(n, 5);
        assert!(buf[5..].iter().all(|&b| b == 0xEE));
    }
}
