//! Hardware-atomic 32-bit wire tick. TIM2 (low 16) + TIM3 (high 16) at
//! HCLK = 144 MHz; `read_tick32` returns a coherent (lo, hi) snapshot.
//! Both TX (`tx::scheduler` deadline math) and RX (boundary stamps)
//! consume `tick32`, so the clock pair lives at the top level rather
//! than under either side.

use ch32_metapac::{RCC, TIM2, TIM3};

pub const WIRE_HZ: u32 = 144_000_000;

pub const fn wire_ticks_per_us() -> u32 {
    WIRE_HZ / 1_000_000
}

/// Coherent (TIM2, TIM3) snapshot. Three loads in the common case, six
/// if TIM2 wraps between the first read and the TIM3 read.
#[inline]
pub fn read_tick32() -> u32 {
    loop {
        let lo_1 = TIM2.cnt().read();
        let hi = TIM3.cnt().read();
        let lo_2 = TIM2.cnt().read();
        // `lo_2 >= lo_1` rejects a TIM2 wrap between the loads -- but not
        // TIM3's startup phase lead: TIM3.CNT increments a few HCLK
        // cycles BEFORE TIM2 wraps (CEN order in `init`), so a read
        // inside that window pairs hi = N+1 with lo ~ 0xFFFx, a tick
        // ~65536 in the future. Reject the tail outright (bench
        // signature: elapsed-since-t0 jumps 455 us, tripping bounded
        // polls like feed_bytes' TXE wait -> spurious BRKSEND "ERR busy"
        // truncating the frame mid-send).
        if lo_2 >= lo_1 && lo_2 < 0xFFC0 {
            return ((hi as u32) << 16) | (lo_2 as u32);
        }
    }
}

/// TIM2 free-runs at HCLK as the low half; TIM3 with PSC=0xFFFF counts
/// once per TIM2 wrap as the high half. No capture channels -- the IC
/// stamping machinery is gone; the pair exists only for `read_tick32`.
///
/// Phase-lock startup: enable TIM3.CEN before TIM2.CEN so TIM3's
/// prescaler leads TIM2's counter by the AHB write gap. TIM3.CNT then
/// increments a few cycles before TIM2 wraps, which `read_tick32`
/// rejects unambiguously (see above).
pub fn init() {
    RCC.apb1pcenr().modify(|w| {
        w.set_tim2en(true);
        w.set_tim3en(true);
    });

    TIM2.psc().write_value(0);
    TIM2.atrlr().write_value(0xFFFF);
    TIM2.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_arpe(false);
    });
    TIM2.swevgr().write(|w| w.set_ug(true)); // load PSC/ARR
    TIM2.intfr().write(|w| w.set_uif(false));

    TIM3.psc().write_value(0xFFFF);
    TIM3.atrlr().write_value(0xFFFF);
    TIM3.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_arpe(false);
    });
    TIM3.swevgr().write(|w| w.set_ug(true));
    TIM3.intfr().write(|w| w.set_uif(false));

    critical_section::with(|_| {
        TIM3.ctlr1().modify(|w| w.set_cen(true));
        TIM2.ctlr1().modify(|w| w.set_cen(true));
    });
}
