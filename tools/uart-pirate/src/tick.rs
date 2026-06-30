//! Hardware-atomic 32-bit wire tick. TIM2 (low 16) + TIM3 (high 16) at
//! HCLK = 144 MHz, with TIM3 latched by TIM2's TRGO via TI1S XOR + TRC
//! pulse. `read_tick32` returns a coherent (lo, hi) snapshot; falling-edge
//! IC captures on PB10 land as paired (TIM2.CCR1, TIM3.CCR1) values that
//! the walker lifts in `rx::rings::falling_at`.
//!
//! Both TX (`tx::scheduler` deadline math) and RX (`rx::walker`
//! ceiling) consume `tick32`, so the clock pair lives at the top level
//! rather than under either side.

use ch32_metapac::timer::vals::{CcmrInputCcs, Ckd, Mms};
use ch32_metapac::{AFIO, GPIOA, RCC, TIM2, TIM3};

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
        if lo_2 >= lo_1 {
            return ((hi as u32) << 16) | (lo_2 as u32);
        }
    }
}

/// Enable RCC for TIM2/TIM3/AFIO/IOPA, pull down PA0/PA1 so the TI1S XOR
/// collapses to PB10 alone, apply TIM2 partial remap #2 (CH3→PB10), then
/// configure TIM2 + TIM3 for hardware-atomic IC capture and start both
/// running.
///
/// TIM2 master + TIM3 high-half. TIM2 PSC=0, ARR=0xFFFF, CKD=DIV_1 pins
/// fDTS at HCLK = 144 MHz; the IC1F filter is set per-baud by
/// `rx::filter::apply_for_brr` (largest delay ≤ brr/3). No CC walker
/// cadence: the event-driven walker runs off USART3 IDLE + DMA1_CH6/CH3
/// HT/TC, not TIM2 IRQs.
///
/// Hardware atomic 32-bit IC capture:
///
/// - CTLR2.TI1S=1 routes XOR(CH1_pin, CH2_pin, CH3_pin) → TI1 internal.
///   PA0/PA1 (CH1/CH2 under TIM2_RM=0b10) are pulled down below, so TI1
///   mirrors PB10's falling edges directly.
/// - CCMR1.CC1S=TI4 (= "normal" CC1 ← TI1). CCER CC1P=1 falling,
///   CC1E=1. DMAINTENR CC1DE=1 kicks DMA1_CH5 (TIM2.CCR1 → low half of
///   every IC entry).
/// - CTLR2.MMS=COMPARE_PULSE: TRGO emits a one-cycle pulse on every
///   CC1IF set, driving TIM3 TRC → TIM3.CCR1 latches the matching high
///   half. The pair forms an atomic 32-bit tick.
///
/// The 1-cycle TRC-sync race window at every TIM2 wrap leaves bad pairs
/// off-by-+65536 (combined > ceiling), which the walker detects and
/// corrects by subtracting one wrap.
///
/// Phase-lock startup: enable TIM3.CEN before TIM2.CEN so TIM3's
/// prescaler leads TIM2's counter by the AHB write gap. TIM3.CNT
/// increments a few cycles before TIM2 wraps, biasing any wrap-race IC
/// pair to off-by-+65536 which the walker detects unambiguously.
///
/// SAFETY: PA0/PA1 must be physically unrouted on the board. A future
/// variant that wires either pin to external signal will inject XOR
/// noise and break IC capture; re-evaluate this block before that
/// point.
pub fn init() {
    RCC.apb2pcenr().modify(|w| {
        w.set_iopaen(true);
        w.set_afioen(true);
    });
    RCC.apb1pcenr().modify(|w| {
        w.set_tim2en(true);
        w.set_tim3en(true);
    });
    AFIO.pcfr1().modify(|w| {
        w.set_tim2_rm(0b10); // 10 = CH3/CH4 → PB10/PB11
    });

    GPIOA.outdr().modify(|w| {
        w.set_odr(0, false);
        w.set_odr(1, false);
    });
    // MODE=00 (input), CNF=10 (input with pull-up/down) → 0b1000.
    // ODR(0)=0 / ODR(1)=0 above selects pull-down.
    GPIOA.cfglr().modify(|w| {
        let mut v = w.0;
        v &= !0xFF;
        v |= 0b1000u32; // PA0
        v |= 0b1000u32 << 4; // PA1
        w.0 = v;
    });

    TIM2.psc().write_value(0);
    TIM2.atrlr().write_value(0xFFFF);
    TIM2.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_arpe(false);
        w.set_ckd(Ckd::DIV_1);
    });
    TIM2.ctlr2().modify(|w| {
        w.set_mms(Mms::COMPARE_PULSE);
        w.set_ti1s(true);
    });
    TIM2.chctlr_input(0).modify(|w| {
        // CCMR1. CC1S in bits [1:0]. IC1F (bits [7:4]) is written
        // separately by `rx::filter::apply_for_brr` from the per-baud LUT.
        w.set_ccs(0, CcmrInputCcs::TI4); // normal mapping → IC1 ← TI1
    });
    TIM2.ccer().modify(|w| {
        w.set_ccp(0, true); // CC1P=1 falling edge
        w.set_cce(0, true); // CC1E=1
    });
    TIM2.dmaintenr().write(|w| {
        w.set_ccde(0, true); // CC1DE (IC capture → DMA1_CH5)
    });
    TIM2.swevgr().write(|w| w.set_ug(true)); // load PSC/ARR
    TIM2.intfr().write(|w| {
        w.set_uif(false);
        w.set_ccif(0, false);
        w.set_ccif(1, false);
        w.set_ccif(2, false);
        w.set_ccif(3, false);
    });

    // TIM3: PSC=0xFFFF → CK_CNT = HCLK/65536 = TIM2 wrap rate. No slave
    // mode (SMS=0); TS=1 keeps TRC routed to ITR1 = TIM2 TRGO.
    // CCMR1.CC1S=TRC latches TIM3.CCR1 on every TRGO pulse; CCER CC1P=0
    // picks the rising edge of the one-cycle pulse. DMAINTENR CC1DE=1
    // kicks DMA1_CH6 (TIM3.CCR1 → high half of every IC entry).
    TIM3.psc().write_value(0xFFFF);
    TIM3.atrlr().write_value(0xFFFF);
    TIM3.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_arpe(false);
    });
    TIM3.smcfgr().modify(|w| {
        w.set_sms(0); // disable slave clock — TIM3 free-runs on CK_INT
        w.set_ts(1); // ITR1 = TIM2 TRGO → TRC
    });
    TIM3.chctlr_input(0).modify(|w| {
        w.set_ccs(0, CcmrInputCcs::TRC); // CC1 from TRC
    });
    TIM3.ccer().modify(|w| {
        w.set_ccp(0, false); // CC1P=0 rising edge of TRGO pulse
        w.set_cce(0, true);
    });
    TIM3.dmaintenr().write(|w| {
        w.set_ccde(0, true); // CC1DE → DMA1_CH6
    });
    TIM3.swevgr().write(|w| w.set_ug(true));
    TIM3.intfr().write(|w| {
        w.set_uif(false);
        w.set_ccif(0, false);
    });

    critical_section::with(|_| {
        TIM3.ctlr1().modify(|w| w.set_cen(true));
        TIM2.ctlr1().modify(|w| w.set_cen(true));
    });
}
