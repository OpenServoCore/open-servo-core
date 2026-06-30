//! Per-baud TIM2 IC1F input-capture filter. fDTS pinned at HCLK = 144 MHz
//! (CKD=DIV_1 set in `tick::init`). Walker subtracts the active delay
//! from each IC stamp so the stored tick reads as wire-edge time, not
//! filter-output time.

use ch32_metapac::TIM2;
use ch32_metapac::timer::vals::FilterValue;
use portable_atomic::{AtomicU32, Ordering};

/// Per-baud CC filter LUT. Each entry is `(ICxF bits, filter delay in
/// tick32 ticks)` for every legal `ICxF` value, sorted by delay.
/// `filter_for_brr` picks the **largest delay ≤ brr/3 (≈ 0.333·bit)** at
/// the configured baud — tighter than production's natural pick because
/// the bench wiring (PB10 AF-OD + PB11 ~30 kΩ internal pull-up, no
/// transceiver) has τ ≈ 500–900 ns line rise. A 1-bit-time HIGH gap
/// between two consecutive falls (every byte whose predecessor has
/// `b7=0`) only leaves ~400 ns of stable HIGH for the filter to
/// recognize before the next fall, so the delay must fit inside that.
///
/// Each entry's delay = N / fSAMPLING, scaled to 144 MHz ticks. RM
/// `CH32FV2x_V3xRM` §14.4.7 ICxF table.
const ICF_LUT: &[(u8, u32)] = &[
    (0b0001, 2),   // fCK_INT,    N=2
    (0b0010, 4),   // fCK_INT,    N=4
    (0b0011, 8),   // fCK_INT,    N=8
    (0b0100, 12),  // fDTS/2,     N=6
    (0b0101, 16),  // fDTS/2,     N=8
    (0b0110, 24),  // fDTS/4,     N=6
    (0b0111, 32),  // fDTS/4,     N=8
    (0b1000, 48),  // fDTS/8,     N=6
    (0b1001, 64),  // fDTS/8,     N=8
    (0b1010, 80),  // fDTS/16,    N=5
    (0b1011, 96),  // fDTS/16,    N=6
    (0b1100, 128), // fDTS/16,    N=8
    (0b1101, 160), // fDTS/32,    N=5
    (0b1110, 192), // fDTS/32,    N=6
    (0b1111, 256), // fDTS/32,    N=8
];

/// Largest LUT entry whose delay is ≤ `brr/3` (≈ 0.333·bit). The earlier
/// "strictly less than `brr`" and "≤ 2·brr/3" rules ate edges
/// wire-empirically: at 1 Mbaud (brr=144) with the weak internal pull-up
/// on PB11, the line takes ~400–600 ns to cross the Schmitt threshold
/// after OD release, leaving only ~400 ns of stable HIGH inside a 1-bit
/// inter-byte gap. The filter needs `delay` ns of stable HIGH to confirm
/// state before it can detect the next fall, so `delay > 400 ns` loses
/// anchors on every byte whose predecessor has b7=0. `brr/3` puts the
/// delay safely under that window at every baud and still rejects
/// glitches narrower than ~1/3·bit, which is well below any real wire
/// noise.
const fn filter_for_brr(brr: u32) -> (u8, u32) {
    let mut best = ICF_LUT[0];
    let mut i = 0;
    while i < ICF_LUT.len() {
        let (bits, delay) = ICF_LUT[i];
        if delay * 3 <= brr && delay >= best.1 {
            best = (bits, delay);
        }
        i += 1;
    }
    best
}

static CC_FILTER_DELAY_TICKS: AtomicU32 = AtomicU32::new(0);

#[inline]
pub(super) fn delay_ticks() -> u32 {
    CC_FILTER_DELAY_TICKS.load(Ordering::Relaxed)
}

/// Reconfigure TIM2 IC1F for the given USART BRR and update the matching
/// delay register. Live write per V20x RM §14.4.7 — IC1F has no "channel
/// must be disabled" restriction (unlike CC1S).
pub(super) fn apply_for_brr(brr: u32) {
    let (icf_bits, delay_ticks) = filter_for_brr(brr);
    TIM2.chctlr_input(0).modify(|w| {
        w.set_icf(0, FilterValue::from_bits(icf_bits));
    });
    CC_FILTER_DELAY_TICKS.store(delay_ticks, Ordering::Release);
}
